"""Vehicle profile YAML I/O.

Profiles are chassis/tyre/steering-geometry snapshots only. They deliberately
exclude scene, controller, path, disturbance, and recording settings so one
vehicle can be reused across many projects.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Any

import yaml

from sim4wis.core.state import VehicleParams
from sim4wis.paths import vehicle_profiles_dir
from sim4wis.project.params_codec import params_from_dict, params_to_dict

BUILTIN_LS9_NAME = "LS9"
BUILTIN_LS9_ALIASES = {"im_ls9"}


def profiles_root() -> Path:
    return vehicle_profiles_dir()


def ensure_root() -> Path:
    root = profiles_root()
    root.mkdir(parents=True, exist_ok=True)
    return root


def _builtin_ls9() -> dict[str, Any]:
    return {
        "profile": {
            "name": BUILTIN_LS9_NAME,
            "label": "LS9",
            "description": (
                "智己 LS9 公开规格 + 用户提供的转向几何表。"
                "主销、接地印迹等未公开项目仍为工程估算，可在负载特性页继续标定。"
            ),
            "builtin": True,
            "sources": [
                "https://www.immotors.com/website/vehicle_config/ls9",
                "https://www.saicmotor.com/chinese/xwzx/mtbd/2025/63146.shtml",
                "用户提供 LS9 转向几何截图：b=1565.172mm, l=3160mm, a=840mm, "
                "r=146.451mm, c=352.052mm, h=176.324mm, θ=85.361°",
            ],
        },
        "vehicle": params_to_dict(VehicleParams()),
    }


def list_profiles() -> list[dict[str, Any]]:
    items = [{
        "name": BUILTIN_LS9_NAME,
        "label": "LS9",
        "builtin": True,
    }]
    root = ensure_root()
    for path in sorted(root.glob("*.yaml")):
        raw = _read_file(path)
        meta = raw.get("profile", {}) if isinstance(raw, dict) else {}
        name = str(meta.get("name") or path.stem)
        if name == BUILTIN_LS9_NAME:
            items[0] = {
                "name": name,
                "label": str(meta.get("label") or "LS9"),
                "builtin": bool(meta.get("builtin", False)),
            }
            continue
        items.append({
            "name": name,
            "label": str(meta.get("label") or name),
            "builtin": False,
        })
    return items


def _read_file(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        return {}
    return raw


def load_profile(name: str) -> dict[str, Any]:
    path = ensure_root() / f"{name}.yaml"
    if path.exists():
        raw = _read_file(path)
    elif name == BUILTIN_LS9_NAME or name in BUILTIN_LS9_ALIASES:
        raw = _builtin_ls9()
    else:
        raise FileNotFoundError(name)
    vehicle = params_to_dict(params_from_dict(raw.get("vehicle"), VehicleParams()))
    profile = dict(raw.get("profile") or {})
    profile.setdefault("name", name)
    profile.setdefault("label", name)
    return {"profile": profile, "vehicle": vehicle}


def load_profile_params(name: str) -> VehicleParams:
    raw = load_profile(name)
    return params_from_dict(raw.get("vehicle"), VehicleParams())


def save_profile(
    name: str,
    params: VehicleParams,
    *,
    label: str | None = None,
    description: str = "",
) -> Path:
    root = ensure_root()
    path = root / f"{name}.yaml"
    data = {
        "profile": {
            "name": name,
            "label": label or name,
            "description": description,
            "builtin": False,
        },
        "vehicle": params_to_dict(params),
    }
    fd, tmp = tempfile.mkstemp(prefix=f".{name}.", suffix=".yaml.tmp", dir=str(root))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def delete_profile(name: str) -> bool:
    path = ensure_root() / f"{name}.yaml"
    if not path.exists():
        return False
    path.unlink()
    return True
