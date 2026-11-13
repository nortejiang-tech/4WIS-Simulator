"""Repackage a previously validated native artifact without rebuilding its core."""
import shutil
import zipfile
from pathlib import Path, PurePosixPath

import build_portable

root = Path(__file__).resolve().parents[1]
artifact = root / "verified-artifact"
archives = list(artifact.rglob(f"4WIS_Simulator_v{build_portable.VERSION}_windows-x64.zip"))
wheels = list(artifact.rglob("sim4wis-*-cp312-cp312-win_amd64.whl"))
if len(archives) != 1 or len(wheels) != 1:
    raise RuntimeError("expected one verified package and one matching Windows wheel")
with zipfile.ZipFile(archives[0]) as archive:
    if any(PurePosixPath(n).is_absolute() or ".." in PurePosixPath(n).parts for n in archive.namelist()):
        raise ValueError("unsafe archive member")
    if archive.testzip() is not None:
        raise ValueError("corrupt archive")
    archive.extractall(build_portable.OUT_ROOT)
pkg = build_portable.OUT_ROOT / "4WIS_Simulator_windows-x64"
shutil.copytree(pkg / "app/dist", build_portable.FRONTEND_DIST, dirs_exist_ok=True)
build_portable.build("windows-x64", update=True, native_wheel=wheels[0])
