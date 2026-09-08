#!/usr/bin/env python3
"""Check the contents and native startup path of portable release archives.

This is deliberately separate from ``check_release_assets.py``.  The latter
answers whether a zip exists; this checker answers whether it contains the
current Web application *and* the independent MCP/Agent entry point.  On the
native target it also extracts a fresh archive and starts the embedded server,
so the release path does not rely on a developer's virtual environment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from pathlib import Path, PurePosixPath


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TARGETS = ("macos-arm64", "windows-x64")


@dataclass(frozen=True)
class Check:
    target: str
    name: str
    ok: bool
    detail: str


def read_version() -> str:
    init_py = ROOT / "backend" / "src" / "sim4wis" / "__init__.py"
    for line in init_py.read_text(encoding="utf-8").splitlines():
        if line.startswith("__version__"):
            return line.split("=", 1)[1].strip().strip("\"'")
    raise RuntimeError(f"could not find __version__ in {init_py}")


def archive_path(version: str, target: str) -> Path:
    return ROOT / "dist_portable" / f"4WIS_Simulator_v{version}_{target}.zip"


def package_dir(target: str) -> str:
    return f"4WIS_Simulator_{target}"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _entry(prefix: str, rel: str) -> str:
    return f"{prefix}/{rel}"


def _static_checks(target: str, version: str) -> list[Check]:
    path = archive_path(version, target)
    checks: list[Check] = []
    if not path.is_file():
        return [Check(target, "archive", False, f"missing {path.relative_to(ROOT)}")]

    prefix = package_dir(target)
    try:
        with zipfile.ZipFile(path) as archive:
            names = set(archive.namelist())
            bad_name = next(
                (
                    name
                    for name in names
                    if PurePosixPath(name).is_absolute() or ".." in PurePosixPath(name).parts
                ),
                None,
            )
            checks.append(
                Check(target, "safe archive paths", bad_name is None,
                      "no absolute/traversal entries" if bad_name is None else f"unsafe entry: {bad_name}")
            )
            broken = archive.testzip()
            checks.append(
                Check(target, "zip CRC", broken is None,
                      "all members readable" if broken is None else f"CRC failure: {broken}")
            )

            required = (
                "app/dist/index.html",
                "app/src/sim4wis/__init__.py",
                "app/src/sim4wis/main.py",
                "app/src/sim4wis_mcp/__init__.py",
                "app/src/sim4wis_mcp/server.py",
                "vendor/mcp/__init__.py",
                "scripts/check_golden_experiments.py",
                "scripts/study_single_wheel_failure.py",
                "scripts/reporting.py",
                "docs/golden_experiments.json",
                "docs/user_manual.html",
                "docs/interaction_guide.md",
                "docs/agent_interface_design.md",
                f"docs/v{version}_changelog.md",
                f"docs/reports/evo-coder/2026-09-09-release-v{version}.md",
                "projects/default.yaml",
                "scripts_lib/slalom.yaml",
                "procedures/step_steer_golden.yaml",
            )
            for rel in required:
                present = _entry(prefix, rel) in names
                checks.append(Check(target, rel, present, "present" if present else "missing"))

            runtime_rel, ui_launcher, mcp_launcher = (
                ("runtime/bin/python3", "start.command", "agent_mcp.command")
                if target == "macos-arm64"
                else ("runtime/python.exe", "start.bat", "agent_mcp.bat")
            )
            for rel in (runtime_rel, ui_launcher, mcp_launcher):
                present = _entry(prefix, rel) in names
                checks.append(Check(target, rel, present, "present" if present else "missing"))

            for rel in ("app/src/sim4wis/__init__.py", "app/src/sim4wis/main.py",
                        "app/src/sim4wis_mcp/server.py"):
                source = ROOT / "backend" / "src" / rel.removeprefix("app/src/")
                packaged = _entry(prefix, rel)
                same = packaged in names and _sha256(archive.read(packaged)) == _sha256(source.read_bytes())
                checks.append(Check(target, f"current source {rel}", same,
                                    "SHA-256 matches source" if same else "missing or stale source"))

            version_entry = _entry(prefix, "app/src/sim4wis/__init__.py")
            has_version = version_entry in names and f'__version__ = "{version}"'.encode() in archive.read(version_entry)
            checks.append(Check(target, "package version", has_version,
                                f"v{version}" if has_version else "backend version marker missing"))

            launcher = _entry(prefix, mcp_launcher)
            if launcher in names:
                raw = archive.read(launcher)
                expected = b"sim4wis_mcp.server" in raw and b"--print-config" not in raw
                # The launcher must forward arguments for --print-config; it
                # need not bake that flag into its normal stdio invocation.
                forwards = b'"$@"' in raw if target == "macos-arm64" else b"%*" in raw
                checks.append(Check(target, "MCP launcher", expected and forwards,
                                    "starts facade and forwards arguments" if expected and forwards
                                    else "missing facade command or argument forwarding"))
                if target == "windows-x64":
                    checks.append(Check(target, "Windows launcher line endings", b"\r\n" in raw,
                                        "CRLF" if b"\r\n" in raw else "expected CRLF"))
                else:
                    mode = (archive.getinfo(launcher).external_attr >> 16) & 0o777
                    checks.append(Check(target, "macOS MCP launcher executable", bool(mode & 0o111),
                                        oct(mode) if mode else "not executable"))
    except (OSError, zipfile.BadZipFile) as exc:
        checks.append(Check(target, "archive readable", False, f"{type(exc).__name__}: {exc}"))
    return checks


def _native_target() -> str | None:
    system = platform.system().lower()
    machine = platform.machine().lower()
    if system == "darwin" and machine in {"arm64", "aarch64"}:
        return "macos-arm64"
    if system == "windows" and machine in {"amd64", "x86_64"}:
        return "windows-x64"
    return None


def _free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _http_json(url: str, timeout: float = 1.0) -> tuple[int, bytes]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return int(response.status), response.read()


def _extract_with_archive_modes(archive: zipfile.ZipFile, dest: Path) -> None:
    """Extract after static path validation, retaining Unix execute bits.

    ``zipfile.extractall`` intentionally does not apply external attributes.
    That is safe as a library default, but it makes a fresh macOS portable
    archive look non-runnable even though its ZIP entries carry the correct
    executable mode.  This checker has already rejected traversal names and
    CRC failures before calling this helper.
    """
    archive.extractall(dest)
    for info in archive.infolist():
        if info.is_dir():
            continue
        mode = (info.external_attr >> 16) & 0o777
        if mode:
            (dest / info.filename).chmod(mode)


def _native_smoke(target: str, version: str) -> list[Check]:
    native = _native_target()
    if native != target:
        return [Check(target, "native smoke", True,
                      f"not run on {platform.system()} {platform.machine()}; static archive checks completed")]

    path = archive_path(version, target)
    prefix = package_dir(target)
    checks: list[Check] = []
    with tempfile.TemporaryDirectory(prefix="4wis-portable-") as temp:
        temp_root = Path(temp)
        with zipfile.ZipFile(path) as archive:
            _extract_with_archive_modes(archive, temp_root)
        pkg = temp_root / prefix
        python = pkg / ("runtime/bin/python3" if target == "macos-arm64" else "runtime/python.exe")
        env = os.environ.copy()
        env["SIM4WIS_DATA_DIR"] = str(pkg)
        env["PYTHONPATH"] = os.pathsep.join((str(pkg / "app" / "src"), str(pkg / "vendor")))

        imports = subprocess.run(
            [str(python), "-c", "import sim4wis, sim4wis_mcp; from sim4wis.main import create_app; assert create_app(); print(sim4wis.__version__)"],
            cwd=pkg,
            env=env,
            text=True,
            capture_output=True,
            timeout=45,
        )
        checks.append(Check(target, "embedded imports", imports.returncode == 0,
                            imports.stdout.strip() if imports.returncode == 0
                            else (imports.stdout + imports.stderr)[-1000:]))

        config = subprocess.run(
            [str(python), "-m", "sim4wis_mcp.server", "--print-config"],
            cwd=pkg,
            env=env,
            text=True,
            capture_output=True,
            timeout=45,
        )
        try:
            printed = json.loads(config.stdout)
            command = printed["mcpServers"]["sim4wis"]["command"]
            config_ok = config.returncode == 0 and bool(command)
        except (KeyError, TypeError, json.JSONDecodeError):
            config_ok = False
        checks.append(Check(target, "MCP --print-config", config_ok,
                            "valid MCP host JSON" if config_ok else (config.stdout + config.stderr)[-1000:]))

        if imports.returncode != 0:
            return checks

        port = _free_port()
        server = subprocess.Popen(
            [str(python), "-m", "uvicorn", "sim4wis.main:app", "--host", "127.0.0.1", "--port", str(port)],
            cwd=pkg,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            deadline = time.monotonic() + 30.0
            health_error = "server did not become healthy"
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    health_error = f"server exited with {server.returncode}"
                    break
                try:
                    status, body = _http_json(f"http://127.0.0.1:{port}/health")
                    if status == 200 and json.loads(body).get("status") == "ok":
                        health_error = ""
                        break
                except (OSError, ValueError, urllib.error.URLError) as exc:
                    health_error = str(exc)
                time.sleep(0.2)
            checks.append(Check(target, "embedded backend health", not health_error,
                                "GET /health = ok" if not health_error else health_error))
            if health_error:
                return checks

            status, body = _http_json(f"http://127.0.0.1:{port}/api/version")
            version_ok = status == 200 and json.loads(body).get("version") == version
            checks.append(Check(target, "embedded backend version", version_ok,
                                f"v{version}" if version_ok else body.decode("utf-8", "replace")[:500]))
            status, body = _http_json(f"http://127.0.0.1:{port}/api/agent/capabilities")
            agent_ok = status == 200 and json.loads(body).get("contract") == "4wis.agent.v1"
            checks.append(Check(target, "embedded Agent API", agent_ok,
                                "4wis.agent.v1" if agent_ok else body.decode("utf-8", "replace")[:500]))
            status, body = _http_json(f"http://127.0.0.1:{port}/")
            ui_ok = status == 200 and b"<div" in body.lower()
            checks.append(Check(target, "embedded Web UI", ui_ok,
                                "served from app/dist" if ui_ok else body.decode("utf-8", "replace")[:500]))
        finally:
            if server.poll() is None:
                server.terminate()
                try:
                    server.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait(timeout=10)
    return checks


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", nargs="+", choices=DEFAULT_TARGETS, default=list(DEFAULT_TARGETS))
    parser.add_argument("--native-smoke", action="store_true",
                        help="extract and start the archive when this host matches the target")
    parser.add_argument("--json", action="store_true", help="also print a machine-readable result array")
    args = parser.parse_args()

    version = read_version()
    checks: list[Check] = []
    for target in args.targets:
        target_checks = _static_checks(target, version)
        checks.extend(target_checks)
        # Do not extract an archive until its names and CRC have passed the
        # static safety checks above.
        if args.native_smoke and all(check.ok for check in target_checks):
            checks.extend(_native_smoke(target, version))

    for check in checks:
        state = "ok" if check.ok else "failed"
        print(f"{check.target}: {check.name}: {state} ({check.detail})")
    if args.json:
        print(json.dumps([asdict(check) for check in checks], ensure_ascii=False, indent=2))
    return 0 if all(check.ok for check in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
