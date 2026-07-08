from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "check_release_assets.py"


def load_checker():
    spec = importlib.util.spec_from_file_location("check_release_assets", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def write_minimal_release_tree(root: Path, checker, version: str = "9.8.7") -> None:
    (root / "backend" / "src" / "sim4wis").mkdir(parents=True)
    (root / "backend" / "src" / "sim4wis" / "__init__.py").write_text(
        f'__version__ = "{version}"\n',
        encoding="utf-8",
    )
    (root / "README.md").write_text(
        f"# Test 4WIS\n\nCurrent version `v{version}`.\n" + ("x" * 1200),
        encoding="utf-8",
    )

    manual = (
        f"<title>4WIS Simulator v{version} 使用说明书（图文版）</title>\n"
        f"<h1>4WIS Simulator 使用说明书 v{version}</h1>\n"
        "本说明书全部截图由 <code>python scripts/build_manual.py</code> 从当前版本实跑生成。\n"
    )
    (root / "docs").mkdir()
    (root / "docs" / "user_manual.html").write_text(manual + ("m" * 100_000), encoding="utf-8")

    fig_root = root / "docs" / "manual_figs"
    fig_root.mkdir()
    for name in checker.REQUIRED_MANUAL_FIGS:
        (fig_root / name).write_bytes(b"f" * 2048)

    for rel in checker.REQUIRED_REPORTS:
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"r" * 2048)

    for rel in checker.REQUIRED_SCRIPTS:
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"s" * 2048)


def test_release_asset_checker_accepts_current_delivery_materials(tmp_path: Path) -> None:
    checker = load_checker()
    write_minimal_release_tree(tmp_path, checker)

    code, checks = checker.check_release_assets(tmp_path)

    assert code == 0
    assert checks
    assert all(check.ok for check in checks)


def test_release_asset_checker_fails_missing_manual_figure(tmp_path: Path) -> None:
    checker = load_checker()
    write_minimal_release_tree(tmp_path, checker)
    (tmp_path / "docs" / "manual_figs" / "21_cmdk.png").unlink()

    code, checks = checker.check_release_assets(tmp_path)

    assert code == 1
    assert any(check.path.name == "21_cmdk.png" and not check.ok for check in checks)


def test_release_asset_checker_requires_portable_zips_only_when_requested(tmp_path: Path) -> None:
    checker = load_checker()
    write_minimal_release_tree(tmp_path, checker, version="1.2.3")

    code, _checks = checker.check_release_assets(tmp_path)
    assert code == 0

    code, checks = checker.check_release_assets(tmp_path, require_portable_zips=True)
    assert code == 1
    assert any("4WIS_Simulator_v1.2.3_macos-arm64.zip" in str(check.path) for check in checks if not check.ok)

    dist = tmp_path / "dist_portable"
    dist.mkdir()
    for target in checker.DEFAULT_TARGETS:
        (dist / f"4WIS_Simulator_v1.2.3_{target}.zip").write_bytes(b"z" * 1_000_000)
    code, checks = checker.check_release_assets(tmp_path, require_portable_zips=True)
    assert code == 0
    assert all(check.ok for check in checks)
