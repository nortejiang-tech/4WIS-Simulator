#!/usr/bin/env python3
"""Assemble portable, one-click 4WIS Simulator packages.

A portable package bundles an embedded Python (python-build-standalone), the
backend source, the built frontend, vendored dependencies, and the editable
data folders (projects / scripts_lib / plugins), plus a double-click launcher.
No Python/Node needs to be installed on the target machine.

Cross-assembly: this runs on macOS but can assemble Windows packages too, by
downloading the matching standalone Python and pip-downloading the target's
wheels (`pip install --target --platform`). It does NOT compile anything.

Usage:
    python scripts/build_portable.py --targets macos-arm64 windows-x64
    python scripts/build_portable.py --targets macos-arm64 --update   # fast re-sync

Output: dist_portable/4WIS_Simulator_<target>/  + a .zip next to it.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import shutil
import subprocess
import sys
import tarfile
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
FRONTEND_DIST = REPO / "frontend" / "dist"
BACKEND_SRC = REPO / "backend" / "src"
DATA_FOLDERS = ["projects", "scripts_lib", "plugins", "targets"]
OUT_ROOT = REPO / "dist_portable"
CACHE = OUT_ROOT / ".cache"

PY_VER = "3.12"
PY_ABI = "cp312"
DEPS = ["fastapi", "uvicorn", "websockets", "pydantic", "numpy", "pyyaml"]


def _version() -> str:
    txt = (BACKEND_SRC / "sim4wis" / "__init__.py").read_text()
    for line in txt.splitlines():
        if line.startswith("__version__"):
            return line.split("=")[1].strip().strip('"').strip("'")
    return "0.0.0"


VERSION = _version()

PBS_RELEASES = "https://api.github.com/repos/astral-sh/python-build-standalone/releases/latest"

TARGETS = {
    "macos-arm64": {
        "pbs_glob": "cpython-3.12.*-aarch64-apple-darwin-install_only.tar.gz",
        "pip_platforms": [
            "macosx_11_0_arm64", "macosx_12_0_arm64",
            "macosx_13_0_arm64", "macosx_14_0_arm64",
        ],
        "python_rel": "bin/python3",
        "launcher": "command",
    },
    "windows-x64": {
        "pbs_glob": "cpython-3.12.*-x86_64-pc-windows-msvc-install_only.tar.gz",
        "pip_platforms": ["win_amd64"],
        "python_rel": "python.exe",
        "launcher": "bat",
    },
}


def log(msg: str) -> None:
    print(f"\033[36m[build]\033[0m {msg}", flush=True)


def resolve_pbs_asset(glob: str) -> tuple[str, str]:
    """Return (asset_name, download_url) for the standalone Python matching glob."""
    req = urllib.request.Request(PBS_RELEASES, headers={"Accept": "application/vnd.github+json"})
    with urllib.request.urlopen(req, timeout=30) as r:
        rel = json.load(r)
    for asset in rel.get("assets", []):
        if fnmatch.fnmatch(asset["name"], glob):
            return asset["name"], asset["browser_download_url"]
    raise RuntimeError(f"no python-build-standalone asset matches {glob!r} in {rel.get('tag_name')}")


def download(url: str, dest: Path) -> Path:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0:
        log(f"cached {dest.name}")
        return dest
    log(f"downloading {dest.name} …")
    # curl handles the GitHub→CDN redirect + retries more reliably than urllib here.
    subprocess.run(
        ["curl", "-fL", "--retry", "3", "--retry-delay", "2", "-m", "600",
         "-o", str(dest), url],
        check=True,
    )
    return dest


def extract_runtime(tarball: Path, dest: Path) -> None:
    """Extract a PBS install_only tarball (top-level `python/`) into dest/runtime."""
    runtime = dest / "runtime"
    if runtime.exists():
        shutil.rmtree(runtime)
    with tarfile.open(tarball) as t:
        t.extractall(dest, filter="data")
    # PBS install_only extracts to `<dest>/python/...`
    (dest / "python").rename(runtime)


def vendor_deps(vendor: Path, pip_platforms: list[str]) -> None:
    if vendor.exists():
        shutil.rmtree(vendor)
    vendor.mkdir(parents=True)
    cmd = [
        sys.executable, "-m", "pip", "install",
        "--target", str(vendor),
        "--only-binary=:all:",
        "--python-version", PY_VER,
        "--implementation", "cp",
        "--abi", PY_ABI,
    ]
    for plat in pip_platforms:
        cmd += ["--platform", plat]
    cmd += DEPS
    log("vendoring deps: " + " ".join(DEPS))
    subprocess.run(cmd, check=True)


def copy_app_and_data(pkg: Path) -> None:
    # backend source
    app_src = pkg / "app" / "src"
    if app_src.exists():
        shutil.rmtree(app_src)
    shutil.copytree(BACKEND_SRC / "sim4wis", app_src / "sim4wis",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.egg-info"))
    # frontend dist
    app_dist = pkg / "app" / "dist"
    if app_dist.exists():
        shutil.rmtree(app_dist)
    shutil.copytree(FRONTEND_DIST, app_dist)
    # editable data folders
    for d in DATA_FOLDERS:
        src = REPO / d
        dst = pkg / d
        if dst.exists():
            shutil.rmtree(dst)
        if src.exists():
            shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        else:
            dst.mkdir(parents=True, exist_ok=True)


LAUNCHER_COMMAND = """#!/bin/sh
# 4WIS Simulator — double-click to start. Opens in your default browser.
DIR="$(cd "$(dirname "$0")" && pwd)"
export SIM4WIS_DATA_DIR="$DIR"
export PYTHONPATH="$DIR/app/src:$DIR/vendor"
PORT=8010
PY="$DIR/runtime/bin/python3"
( sleep 2; open "http://127.0.0.1:$PORT/" ) &
echo "4WIS Simulator → http://127.0.0.1:$PORT/  (close this window to stop)"
exec "$PY" -m uvicorn sim4wis.main:app --host 127.0.0.1 --port $PORT
"""

LAUNCHER_BAT = """@echo off
REM 4WIS Simulator - double-click to start. Opens in your default browser.
set "DIR=%~dp0"
set "SIM4WIS_DATA_DIR=%DIR%"
set "PYTHONPATH=%DIR%app\\src;%DIR%vendor"
set PORT=8010
start "" "http://127.0.0.1:%PORT%/"
echo 4WIS Simulator -^> http://127.0.0.1:%PORT%/  (close this window to stop)
"%DIR%runtime\\python.exe" -m uvicorn sim4wis.main:app --host 127.0.0.1 --port %PORT%
"""

README = """4WIS Simulator — 便携版使用说明
==================================

【启动】
  - macOS:  双击 start.command
            (首次会被 Gatekeeper 拦截 → 右键点 start.command → 打开 → 再点"打开"。
             或在终端执行: xattr -dr com.apple.quarantine "这个文件夹路径")
  - Windows: 双击 start.bat
  启动后会自动打开浏览器到 http://127.0.0.1:8010/
  关闭那个黑色命令行窗口即可停止仿真。

【改参数 — 不用重新打包】
  - projects/*.yaml      整车/悬架/场景/控制器参数。改完在界面"项目"里点"加载"。
  - scripts_lib/*.yaml   动作脚本工况。
  - plugins/strategies/  放 FMU/策略插件。
  改完重启(或在界面里重新加载)即可生效。

【改后端逻辑 — 改源码重启即可】
  app/src/sim4wis/*.py 是明文源码,直接编辑后重启 start 脚本生效,无需重新构建。
  (仅当新增第三方 Python 库时才需要往 vendor/ 补对应平台的 wheel。)

【改界面】 需要在开发机上改 React 源码并 npm run build,再替换 app/dist。

目录结构:
  start.command / start.bat   启动器
  runtime/                     内嵌 Python
  vendor/                      Python 依赖
  app/src, app/dist            后端源码 + 前端
  projects/ scripts_lib/ plugins/   可编辑配置
"""


def write_launcher_and_readme(pkg: Path, kind: str) -> None:
    if kind == "command":
        p = pkg / "start.command"
        p.write_text(LAUNCHER_COMMAND)
        p.chmod(0o755)
    else:
        # Path.write_text doesn't accept newline= until 3.10; force CRLF
        # explicitly using write_bytes so the .bat works on Windows.
        crlf = LAUNCHER_BAT.replace("\r\n", "\n").replace("\n", "\r\n")
        (pkg / "start.bat").write_bytes(crlf.encode("utf-8"))
    (pkg / "使用说明.txt").write_text(README, encoding="utf-8")


def build(target: str, update: bool) -> Path:
    spec = TARGETS[target]
    pkg = OUT_ROOT / f"4WIS_Simulator_{target}"
    pkg.mkdir(parents=True, exist_ok=True)

    if not update:
        name, url = resolve_pbs_asset(spec["pbs_glob"])
        tarball = download(url, CACHE / name)
        log(f"extracting runtime for {target}")
        extract_runtime(tarball, pkg)
        vendor_deps(pkg / "vendor", spec["pip_platforms"])
    else:
        log(f"[update] keeping runtime + vendor for {target}")

    log(f"copying app + data for {target}")
    copy_app_and_data(pkg)
    write_launcher_and_readme(pkg, spec["launcher"])

    # Keep the on-disk dir name stable (so --update reuses runtime/vendor),
    # but stamp the version into the zip filename.
    log(f"zipping {pkg.name} (v{VERSION})")
    zip_base = OUT_ROOT / f"4WIS_Simulator_v{VERSION}_{target}"
    zip_path = shutil.make_archive(str(zip_base), "zip", root_dir=OUT_ROOT, base_dir=pkg.name)
    size_mb = Path(zip_path).stat().st_size / 1e6
    log(f"done {target}: {zip_path}  ({size_mb:.0f} MB)")
    return Path(zip_path)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--targets", nargs="+", default=["macos-arm64", "windows-x64"],
                    choices=list(TARGETS))
    ap.add_argument("--update", action="store_true",
                    help="only re-sync app + data folders (skip runtime/vendor download)")
    args = ap.parse_args()

    if not (FRONTEND_DIST / "index.html").is_file():
        sys.exit("frontend not built — run `cd frontend && npm run build` first")

    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    zips = [build(t, args.update) for t in args.targets]
    print("\n".join(str(z) for z in zips))


if __name__ == "__main__":
    main()
