"""Vehicle model registry.

The product-facing simulation line is intentionally limited to two fidelity
levels: ``kinematic`` and ``simplified_dynamic``. ``multibody`` remains
available for research/regression projects, but it should not drive the shape
of new UI/API contracts by default.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from sim4wis.core.state import VehicleParams
from sim4wis.vehicle.base import VehicleModel
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.kinematic import KinematicModel
from sim4wis.vehicle.multibody import MultiBodyModel

ModelLayer = Literal["primary", "research"]


@dataclass(frozen=True)
class VehicleModelInfo:
    id: str
    label: str
    layer: ModelLayer
    description: str


MODEL_REGISTRY: tuple[VehicleModelInfo, ...] = (
    VehicleModelInfo(
        id="kinematic",
        label="运动学",
        layer="primary",
        description="纯滚动几何模型，适合策略和 ICR 验证。",
    ),
    VehicleModelInfo(
        id="simplified_dynamic",
        label="动力学",
        layer="primary",
        description="3-DOF 整车 + 轮胎力 + 载荷转移，是工程分析主线模型。",
    ),
    VehicleModelInfo(
        id="multibody",
        label="多体 14DOF",
        layer="research",
        description="研究/回归用高阶模型，保留但不作为新功能默认适配目标。",
    ),
)


def model_infos(*, include_research: bool = True) -> list[VehicleModelInfo]:
    if include_research:
        return list(MODEL_REGISTRY)
    return [m for m in MODEL_REGISTRY if m.layer == "primary"]


def model_ids(*, include_research: bool = True) -> list[str]:
    return [m.id for m in model_infos(include_research=include_research)]


def is_known_model(model_type: str) -> bool:
    return model_type in set(model_ids())


def model_layer(model_type: str) -> ModelLayer:
    for info in MODEL_REGISTRY:
        if info.id == model_type:
            return info.layer
    raise ValueError(f"unknown model type: {model_type!r}")


def make_vehicle_model(params: VehicleParams, model_type: str) -> VehicleModel:
    if model_type == "kinematic":
        return KinematicModel(params)
    if model_type == "simplified_dynamic":
        return SimplifiedDynamicModel(params)
    if model_type == "multibody":
        return MultiBodyModel(params)
    raise ValueError(f"unknown model type: {model_type!r}")


def serialize_model_info(info: VehicleModelInfo) -> dict[str, str]:
    return {
        "id": info.id,
        "label": info.label,
        "layer": info.layer,
        "description": info.description,
    }
