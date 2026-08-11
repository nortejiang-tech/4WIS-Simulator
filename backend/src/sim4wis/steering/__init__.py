"""steering — the steering system as a plant.

The vehicle side of this simulator is mature; the steering system was only ever
a diagnostic output. This package supplies what sits between the driver's hands
and the pinion — torsion bar, torque sensor, assist map, motor, friction — so
that an assist curve can be developed, a motor can be sized, and hand-wheel
torque exists as a signal at all.

Off by default (`SteeringSystemParams.enabled = False`), in which case nothing
here runs and the vehicle behaves exactly as it did before.

See docs/v2_steering_platform_plan.md.
"""

from sim4wis.steering import architecture
from sim4wis.steering.architecture import Architecture, ArchitectureError
from sim4wis.steering.assist import AssistMap, AssistMapError
from sim4wis.steering.params import (
    ColumnParams,
    MotorParams,
    RackParams,
    SteeringSystemParams,
)

__all__ = [
    "Architecture",
    "ArchitectureError",
    "AssistMap",
    "architecture",
    "AssistMapError",
    "ColumnParams",
    "MotorParams",
    "RackParams",
    "SteeringSystemParams",
]
