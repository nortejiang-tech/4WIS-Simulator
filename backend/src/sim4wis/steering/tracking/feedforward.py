"""Feedforward blocks — the 积木 (lego blocks) of the tracking layer.

Three blocks are required by the frozen spec (docs/steering_control_layer_
requirements.md §4.8), each an independent, optional, configurable component
that stacks onto any feedback controller:

    rack_force   动力学/齿条力前馈 — compensate the known load torque,
                 optionally through a low-pass (a noisy rack-force estimate
                 must not become a noisy command)
    velocity     请求速度前馈 — τ = b·ω_ref + J·α_ref; the classic servo
                 velocity/acceleration feedforward that lets the feedback
                 loop spend its authority on the error, not the motion
    friction     系统级摩擦补偿前馈 — F_c·tanh(ω_ref/ε): smooth, saturating,
                 no chatter at zero crossing (a sign() would inject a step
                 the rate loop then has to undo)

A block is pure computation from the *request* and the *measured load* — it
has no state except what its own filtering needs. The stack is the sum; the
spec switches each block on and configures it independently:

    feedforward:
      blocks:
        - {type: rack_force, gain: 1.0, lowpass_hz: 15}
        - {type: velocity, damping: 4.0, inertia: 0.6}
        - {type: friction, friction_nm: 0.5, eps_rad_s: 0.01}
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar


@dataclass
class FeedforwardContext:
    """The request side of one control step, in the road-wheel domain."""

    target_rate: float = 0.0      # requested wheel rate [rad/s]
    target_accel: float = 0.0     # requested wheel acceleration [rad/s²]
    load_torque: float = 0.0      # known load [N·m] (rack force referred)
    speed_ms: float = 0.0         # vehicle speed, for scheduling
    dt: float = 5.0e-4            # this step's control period [s]


class FeedforwardBlock(ABC):
    """One additive torque term. Enabled flag is part of the block, so a
    spec can carry a block configured but switched off."""

    name: ClassVar[str] = "abstract"
    enabled: bool = True

    @abstractmethod
    def compute(self, ctx: FeedforwardContext) -> float:
        """Torque contribution [N·m]."""


class RackForceFeedforward(FeedforwardBlock):
    """动力学/齿条力前馈 — compensate the known load."""

    name: ClassVar[str] = "rack_force"

    def __init__(self, gain: float = 1.0, lowpass_hz: float = 0.0) -> None:
        self.gain = float(gain)
        self.lowpass_hz = float(lowpass_hz)
        self._filt = 0.0

    def compute(self, ctx: FeedforwardContext) -> float:
        if not self.enabled:
            return 0.0
        v = float(ctx.load_torque)
        if self.lowpass_hz > 0.0:
            # Filter state advances per call — the caller must call exactly
            # once per control step, which the coupling guarantees.
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.lowpass_hz * max(ctx.dt, 1e-9))
            self._filt += (v - self._filt) * alpha
            v = self._filt
        return self.gain * v


class VelocityFeedforward(FeedforwardBlock):
    """请求速度前馈 — τ = b·ω_ref + J·α_ref."""

    name: ClassVar[str] = "velocity"

    def __init__(self, damping: float = 4.0, inertia: float = 0.6,
                 accel_term: bool = True) -> None:
        self.damping = float(damping)
        self.inertia = float(inertia)
        self.accel_term = bool(accel_term)

    def compute(self, ctx: FeedforwardContext) -> float:
        if not self.enabled:
            return 0.0
        out = self.damping * float(ctx.target_rate)
        if self.accel_term:
            out += self.inertia * float(ctx.target_accel)
        return out


class FrictionFeedforward(FeedforwardBlock):
    """系统级摩擦补偿前馈 — F_c·tanh(ω_ref/ε), smooth through zero."""

    name: ClassVar[str] = "friction"

    def __init__(self, friction_nm: float = 0.5, eps_rad_s: float = 0.01) -> None:
        self.friction_nm = float(friction_nm)
        self.eps = max(float(eps_rad_s), 1e-9)

    def compute(self, ctx: FeedforwardContext) -> float:
        if not self.enabled:
            return 0.0
        return self.friction_nm * math.tanh(float(ctx.target_rate) / self.eps)


BLOCKS: dict[str, type[FeedforwardBlock]] = {
    cls.name: cls
    for cls in (RackForceFeedforward, VelocityFeedforward, FrictionFeedforward)
}


@dataclass
class FeedforwardStack:
    """An ordered, per-block-switchable sum of feedforward torques."""

    blocks: list[FeedforwardBlock] = field(default_factory=list)

    def compute(self, ctx: FeedforwardContext) -> float:
        return sum(b.compute(ctx) for b in self.blocks)

    def describe(self) -> list[str]:
        return [f"{b.name}{'' if b.enabled else ' (off)'}" for b in self.blocks]


def make_feedforward(spec: dict[str, Any] | None) -> FeedforwardStack:
    """Build a stack from a spec dict: {blocks: [{type, ...}]} or None."""
    if not spec:
        return FeedforwardStack()
    if not isinstance(spec, dict):
        raise ValueError(f"feedforward spec must be a mapping, got {type(spec)}")
    blocks: list[FeedforwardBlock] = []
    for entry in spec.get("blocks", []):
        if not isinstance(entry, dict) or "type" not in entry:
            raise ValueError(f"feedforward block needs a type: {entry!r}")
        cls = BLOCKS.get(entry["type"])
        if cls is None:
            raise ValueError(
                f"unknown feedforward block {entry['type']!r}; "
                f"known: {sorted(BLOCKS)}")
        enabled = bool(entry.pop("enabled", True))
        block = cls(**{k: v for k, v in entry.items() if k != "type"})
        block.enabled = enabled
        blocks.append(block)
    return FeedforwardStack(blocks)
