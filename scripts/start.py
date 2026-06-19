#!/usr/bin/env python3
"""4WIS Simulator — cross-platform launcher.

Usage:
    python scripts/start.py            # dev mode (backend + vite dev server)
    python scripts/start.py --build    # build frontend, then serve via FastAPI

Behaviour:
    1. (first run) ensures backend Python deps + frontend node deps are installed
    2. spawns the backend (FastAPI on :8010)
    3. either spawns Vite dev server (:5173) or serves the built bundle
    4. opens default browser on the right URL
    5. on Ctrl+C, terminates all children cleanly
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BACKEND = ROOT / "backend"
FRONTEND = ROOT / "frontend"


def _info(msg: str) -> None:
    print(f"[start] {msg}")


def _run(cmd: list[str], cwd: Path | None = None) -> None:
    _info(f"$ {' '.join(cmd)}  (cwd={cwd or ROOT})")
    subprocess.check_call(cmd, cwd=cwd)


def ensure_backend_deps() -> None:
    """Install backend in editable mode if sim4wis is not importable."""
    try:
        import sim4wis  # noqa: F401
        return
    except ImportError:
        pass
    _info("Installing backend dependencies (pip install -e backend)…")
    _run([sys.executable, "-m", "pip", "install", "-e", str(BACKEND)])


def ensure_frontend_deps() -> None:
    if (FRONTEND / "node_modules").exists():
        return
    npm = shutil.which("npm")
    if not npm:
        sys.exit("npm not found — please install Node.js 18+ first.")
    _info("Installing frontend dependencies (npm install)…")
    _run([npm, "install"], cwd=FRONTEND)


def spawn_backend() -> subprocess.Popen:
    _info("Starting backend on http://127.0.0.1:8010 …")
    return subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "sim4wis.main:app", "--host", "127.0.0.1", "--port", "8010"],
        cwd=BACKEND,
    )


def spawn_vite_dev() -> subprocess.Popen:
    npm = shutil.which("npm")
    assert npm
    _info("Starting Vite dev server on http://127.0.0.1:5173 …")
    return subprocess.Popen([npm, "run", "dev"], cwd=FRONTEND)


def build_frontend() -> None:
    npm = shutil.which("npm")
    assert npm
    _info("Building frontend (npm run build)…")
    _run([npm, "run", "build"], cwd=FRONTEND)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true", help="Build frontend bundle instead of dev server")
    parser.add_argument("--no-browser", action="store_true", help="Skip auto-opening the browser")
    args = parser.parse_args()

    ensure_backend_deps()
    ensure_frontend_deps()
    if args.build:
        build_frontend()

    procs: list[subprocess.Popen] = []
    procs.append(spawn_backend())
    if not args.build:
        procs.append(spawn_vite_dev())

    # Give the servers a moment to bind their ports before launching the browser.
    time.sleep(2.0)
    url = "http://127.0.0.1:5173" if not args.build else "http://127.0.0.1:8010"
    if not args.no_browser:
        _info(f"Opening browser → {url}")
        webbrowser.open(url)

    _info("Ctrl+C to stop all services.")
    try:
        while True:
            for p in procs:
                if p.poll() is not None:
                    _info(f"child PID {p.pid} exited with code {p.returncode}")
                    raise SystemExit(p.returncode or 1)
            time.sleep(0.5)
    except KeyboardInterrupt:
        _info("Shutting down…")
    finally:
        for p in procs:
            if p.poll() is None:
                try:
                    if os.name == "nt":
                        p.send_signal(signal.CTRL_BREAK_EVENT)
                    else:
                        p.terminate()
                except Exception:
                    pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
    return 0


if __name__ == "__main__":
    sys.exit(main())
