"""Assist map — the object an EPS calibration engineer actually edits.

A boost curve is a table, not a formula, because that is how it is calibrated:
breakpoints in torsion-bar torque, a family of curves over speed, and a
calibrator moving individual points until the car feels right. Fitting it to a
smooth function would be tidier and would misrepresent the workflow.

Convention: assist is expressed as **torque at the pinion [N·m]**, positive in
the direction of the driver's input. Referring it to the pinion rather than to
the motor shaft keeps the calibration independent of the gear ratio and of the
architecture — the same curve describes a column-assist and a rack-assist car
that deliver the same help, which is exactly what a system engineer comparing
architectures wants.

Shape of a production curve, and why:

    deadband     below ~0.3 N·m nothing happens, or the car would wander;
                 this is also where on-centre feel is won or lost
    rising       progressive, so effort builds with cornering demand
    saturation   assist stops growing near the mechanical limit
    speed        heavy assist at parking, little at motorway speed —
                 the whole point of a speed-scheduled map
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

#: Torsion-bar torque breakpoints [N·m]. Dense at the bottom because that is
#: where on-centre behaviour lives and where a calibrator spends the time.
DEFAULT_TAU_BP: tuple[float, ...] = (0.0, 0.3, 0.8, 1.5, 2.5, 3.5, 5.0, 8.0)

#: Speed breakpoints [km/h].
DEFAULT_SPEED_BP: tuple[float, ...] = (0.0, 20.0, 60.0, 120.0)

#: Assist torque at the pinion [N·m], table[speed][torque].
#:
#: Calibrated so the LS9-class default (2.9 t, 20 mm pinion) needs ~3.5 N·m at
#: the hand wheel for full-lock parking on high grip: 3.5 + 205 = 208.5 N·m at
#: the pinion, i.e. ~10.4 kN of rack force. That is a plausible heavy-SUV
#: parking effort, and it is an *estimate* — no bench data backs it.
DEFAULT_TABLE: tuple[tuple[float, ...], ...] = (
    (0.0, 0.0, 25.0, 75.0, 150.0, 205.0, 235.0, 250.0),   #   0 km/h
    (0.0, 0.0, 20.0, 62.0, 125.0, 172.0, 200.0, 215.0),   #  20 km/h
    (0.0, 0.0, 10.0, 32.0,  68.0,  96.0, 115.0, 128.0),   #  60 km/h
    (0.0, 0.0,  5.0, 17.0,  36.0,  52.0,  64.0,  72.0),   # 120 km/h
)


class AssistMapError(ValueError):
    """A boost curve that is malformed or would misbehave as a control input."""


@dataclass
class AssistMap:
    """Speed-scheduled boost curve, bilinearly interpolated."""

    name: str = "default"
    tau_bp: tuple[float, ...] = DEFAULT_TAU_BP
    speed_bp: tuple[float, ...] = DEFAULT_SPEED_BP
    table: tuple[tuple[float, ...], ...] = DEFAULT_TABLE
    description: str = "LS9-class heavy SUV, estimated"
    _grid: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.validate()
        self._grid = np.asarray(self.table, dtype=np.float64)

    # ---- validation --------------------------------------------------------

    def validate(self) -> None:
        tau = np.asarray(self.tau_bp, dtype=np.float64)
        spd = np.asarray(self.speed_bp, dtype=np.float64)
        grid = np.asarray(self.table, dtype=np.float64)

        if grid.shape != (spd.size, tau.size):
            raise AssistMapError(
                f"table is {grid.shape}, expected ({spd.size}, {tau.size}) "
                "= (speed breakpoints, torque breakpoints)"
            )
        if tau.size < 2 or spd.size < 2:
            raise AssistMapError("need at least two breakpoints on each axis")
        if np.any(np.diff(tau) <= 0) or np.any(np.diff(spd) <= 0):
            raise AssistMapError("breakpoints must be strictly increasing")
        if tau[0] != 0.0:
            raise AssistMapError("the torque axis must start at 0")
        if np.any(grid < 0):
            raise AssistMapError("negative assist would fight the driver")
        # Non-monotone assist means more driver effort buys less help — the car
        # gets an effort reversal the driver feels as a catch. Real calibrations
        # are monotone in torque; treat a violation as a defect, not a taste.
        if np.any(np.diff(grid, axis=1) < 0):
            bad = int(np.argmax(np.any(np.diff(grid, axis=1) < 0, axis=1)))
            raise AssistMapError(
                f"assist must not decrease with torsion-bar torque "
                f"(speed row {self.speed_bp[bad]} km/h) — the driver would feel a catch"
            )

    # ---- evaluation --------------------------------------------------------

    def assist_torque(self, tau_sensor: float, speed_ms: float) -> float:
        """Pinion assist torque [N·m] for a sensor reading and a road speed.

        Sign follows the driver's input; the table holds magnitudes only.
        Off the ends of either axis the edge value is held, which is what a
        production ECU does with a lookup table.
        """
        mag = abs(float(tau_sensor))
        v_kmh = abs(float(speed_ms)) * 3.6

        tau = np.asarray(self.tau_bp, dtype=np.float64)
        spd = np.asarray(self.speed_bp, dtype=np.float64)

        # Interpolate along torque within each bracketing speed row, then
        # between the two rows.
        j = int(np.clip(np.searchsorted(spd, v_kmh) - 1, 0, spd.size - 2))
        w = 0.0 if spd[j + 1] == spd[j] else (v_kmh - spd[j]) / (spd[j + 1] - spd[j])
        w = float(np.clip(w, 0.0, 1.0))

        lo = float(np.interp(mag, tau, self._grid[j]))
        hi = float(np.interp(mag, tau, self._grid[j + 1]))
        out = lo + (hi - lo) * w
        return out if tau_sensor >= 0.0 else -out

    def boost_ratio(self, tau_sensor: float, speed_ms: float) -> float:
        """Assist per unit driver torque — the number a calibration review quotes."""
        mag = abs(float(tau_sensor))
        if mag < 1e-9:
            return 0.0
        return abs(self.assist_torque(tau_sensor, speed_ms)) / mag

    # ---- serialisation -----------------------------------------------------

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "description": self.description,
            "tau_bp": list(self.tau_bp),
            "speed_bp": list(self.speed_bp),
            "table": [list(row) for row in self.table],
        }

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> AssistMap:
        return cls(
            name=str(data.get("name", "custom")),
            description=str(data.get("description", "")),
            tau_bp=tuple(float(x) for x in data["tau_bp"]),          # type: ignore[arg-type]
            speed_bp=tuple(float(x) for x in data["speed_bp"]),      # type: ignore[arg-type]
            table=tuple(tuple(float(x) for x in row) for row in data["table"]),  # type: ignore[arg-type]
        )

    def scaled(self, factor: float, name: str | None = None) -> AssistMap:
        """A uniformly scaled copy — the simplest thing a sweep varies.

        `assist_scale` as a study axis answers "how much assist does this car
        actually need", which is a sizing question, without anyone having to
        hand-author a second table.
        """
        if factor < 0:
            raise AssistMapError("assist scale must not be negative")
        return AssistMap(
            name=name or f"{self.name}x{factor:g}",
            description=f"{self.description} (scaled {factor:g})",
            tau_bp=self.tau_bp,
            speed_bp=self.speed_bp,
            table=tuple(tuple(v * factor for v in row) for row in self.table),
        )


#: Named calibrations. `none` is the unassisted baseline — manual steering,
#: and the reference every assist comparison is made against.
LIBRARY: dict[str, AssistMap] = {
    "default": AssistMap(),
    "none": AssistMap(
        name="none",
        description="无助力（机械转向基准）",
        table=tuple(tuple(0.0 for _ in DEFAULT_TAU_BP) for _ in DEFAULT_SPEED_BP),
    ),
}


def get(name: str) -> AssistMap:
    if name not in LIBRARY:
        raise AssistMapError(
            f"unknown assist map {name!r}; available: {', '.join(sorted(LIBRARY))}"
        )
    return LIBRARY[name]
