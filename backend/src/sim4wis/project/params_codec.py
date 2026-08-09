"""VehicleParams dict conversion helpers shared by projects/profiles/API."""

from __future__ import annotations

import dataclasses
from typing import Any

from sim4wis.core.state import SteeringGeometryParams, SuspensionParams, VehicleParams


def params_to_dict(params: VehicleParams) -> dict[str, Any]:
    return dataclasses.asdict(params)


def params_from_dict(data: dict[str, Any] | None, base: VehicleParams | None = None) -> VehicleParams:
    """Build VehicleParams from a partial/full dict while ignoring unknown keys."""
    if data is None:
        return base or VehicleParams()
    base = base or VehicleParams()
    raw = dict(data)
    # B2 backward compat: legacy `parking_scrub_coeff` populates both new
    # split fields if the new ones weren't supplied.
    if "parking_scrub_coeff" in raw:
        legacy = raw["parking_scrub_coeff"]
        raw.setdefault("parking_lateral_coeff", legacy)
        raw.setdefault("parking_torque_coeff", legacy)
    susp_raw = raw.pop("suspension", None)
    geom_raw = raw.pop("steering_geometry", None)
    # K&C is a measured characteristic, not a scalar: it arrives either inline
    # (a `kc:` section) or by naming a file under kc_profiles/. Either way it is
    # parsed into the typed VehicleKC before it reaches VehicleParams, so the
    # models never see raw dicts or engineering units.
    kc_raw = raw.pop("kc", None)
    kc_profile = raw.pop("kc_profile", None)

    vehicle_fields = {f.name for f in dataclasses.fields(VehicleParams)}
    vehicle_fields.discard("suspension")
    vehicle_fields.discard("steering_geometry")
    vehicle_fields.discard("kc")
    vehicle_updates = {k: v for k, v in raw.items() if k in vehicle_fields}

    susp = base.suspension
    if isinstance(susp_raw, dict):
        susp_fields = {f.name for f in dataclasses.fields(SuspensionParams)}
        susp_updates = {k: v for k, v in susp_raw.items() if k in susp_fields}
        susp = dataclasses.replace(susp, **susp_updates)

    geom = base.steering_geometry
    if isinstance(geom_raw, dict):
        geom_fields = {f.name for f in dataclasses.fields(SteeringGeometryParams)}
        geom_updates = {k: v for k, v in geom_raw.items() if k in geom_fields}
        geom = dataclasses.replace(geom, **geom_updates)

    kc = base.kc
    if kc_profile:
        from sim4wis.vehicle.kc import load_kc_profile
        from sim4wis.paths import kc_profiles_dir
        kc = load_kc_profile(kc_profiles_dir() / f"{kc_profile}.yaml")
    elif isinstance(kc_raw, dict):
        from sim4wis.vehicle.kc import kc_from_dict
        kc = kc_from_dict(kc_raw)

    return dataclasses.replace(
        base,
        suspension=susp,
        steering_geometry=geom,
        kc=kc,
        **vehicle_updates,
    )
