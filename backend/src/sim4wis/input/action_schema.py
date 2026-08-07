"""YAML / dict schema for action-sequence driving scripts.

Each action is identified by its `action` field. Times `t` are absolute
seconds from script start, sorted ascending. Some actions (ramps, brake)
specify a `duration` over which they apply; the runner integrates them
internally.

Available actions (Phase 2a v1):
    drive          set throttle + steering instantly
    set_strategy   switch active strategy
    set_mode_params set strategy-specific mode_params dict
    throttle_ramp  linearly ramp throttle from `from` → `to` over `duration`
    steer_ramp     same for steering
    brake          full friction brake (brake=1) for `duration` (or until v=0)
    wait_until     blocking wait — until {t, distance, speed_below}
    reset          reset vehicle pose & trajectory
    stop           terminate script

The schema uses dict-with-`action`-tag discriminator. Validation is done via
pydantic, but if pydantic is missing we fall back to manual validation (so
the script schema is importable even in slim envs like the dev sandbox).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal


@dataclass
class Action:
    t: float
    action: str
    # Per-action kwargs live here; the runner reads them by name.
    args: dict[str, Any] = field(default_factory=dict)


def validate_actions(raw_actions: list[dict[str, Any]]) -> list[Action]:
    """Lightweight validation + normalisation, independent of pydantic."""
    actions: list[Action] = []
    for i, a in enumerate(raw_actions):
        if not isinstance(a, dict):
            raise ValueError(f"action {i} is not a dict")
        if "action" not in a:
            raise ValueError(f"action {i} missing 'action' field")
        if "t" not in a:
            raise ValueError(f"action {i} missing 't' field")
        name = str(a["action"])
        t = float(a["t"])
        args = {k: v for k, v in a.items() if k not in ("action", "t")}
        _check_action_args(name, args, i)
        actions.append(Action(t=t, action=name, args=args))
    actions.sort(key=lambda x: x.t)
    return actions


def _check_action_args(name: str, args: dict[str, Any], idx: int) -> None:
    """Per-action argument sanity check. Raises on mismatch."""
    expected = _EXPECTED_ARGS.get(name)
    if expected is None:
        raise ValueError(f"action {idx}: unknown action '{name}'")
    missing = [k for k in expected if k not in args]
    if missing:
        raise ValueError(f"action {idx} ({name}): missing args {missing}")


_EXPECTED_ARGS: dict[str, tuple[str, ...]] = {
    "drive":           ("throttle", "steering"),
    "set_strategy":    ("name",),
    "set_mode_params": ("params",),
    "throttle_ramp":   ("from_", "to", "duration"),
    "steer_ramp":      ("from_", "to", "duration"),
    "brake":           ("duration",),
    "wait_until":      (),  # one of t/distance/speed_below
    "reset":           (),
    "stop":            (),
}


@dataclass
class Script:
    name: str = "untitled"
    description: str = ""
    loop: bool = False
    actions: list[Action] = field(default_factory=list)
    # Optional reference path/cone layout to lay down when this script loads,
    # so a standard maneuver (slalom / DLC / parking) shows its ground markers
    # for manual or visual reference. Shape: {"name": str, "params": {...}}.
    path_template: dict[str, Any] | None = None

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Script:
        s = d.get("script", d)  # accept either top-level or wrapped under "script"
        pt = s.get("path_template")
        return cls(
            name=str(s.get("name", "untitled")),
            description=str(s.get("description", "")),
            loop=bool(s.get("loop", False)),
            actions=validate_actions(list(s.get("actions") or [])),
            path_template=pt if isinstance(pt, dict) else None,
        )

    def to_dict(self) -> dict[str, Any]:
        script: dict[str, Any] = {
            "name": self.name,
            "description": self.description,
            "loop": self.loop,
            "actions": [
                {"t": a.t, "action": a.action, **a.args}
                for a in self.actions
            ],
        }
        if self.path_template:
            script["path_template"] = self.path_template
        return {"script": script}


# Re-export for tests / external use
__all__ = ["Action", "Script", "validate_actions"]
