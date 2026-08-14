"""Per-corner angle-tracking control layer (docs/steering_control_layer_requirements.md).

The strategy still commands wheel angles; this layer decides how the corner
actuators get there. It sits between ``delta_cmd`` and the tyre for every
corner an architecture gives an actuator to — SBW front corners, RWS rear
corners, all four on 4WIS — and it is the home of the control-strategy work:
PID variants, LQR, MPC, SMC, ADRC, H∞, plus the pluggable feedforward blocks
(load/rack-force, velocity, friction compensation).

Submodules:

    controller   the AngleTrackingController protocol, the open_loop reference,
                 and the registry every strategy registers with
    plant        the torque-mode corner actuator (2nd order + Coulomb friction)
    feedback     the angle sensor model (quantisation, delay, optional noise)
    coupling     CornerTracker — one controller+plant+sensor per corner, and
                 the multi-rate step the vehicle models call

Two contracts this package lives by:

1. **The disabled layer is bit-identical.** ``angle_control.enabled`` defaults
   to False and every path this package adds is bypassed then — the legacy
   actuator models run exactly as before (the repo's ``k = 0`` convention).
2. **open_loop reproduces the legacy by-wire update bit-exactly.** The legacy
   ``ByWirePlant`` angle channel is a first-order bandwidth limit plus a rate
   limit, run once per 5 ms outer frame. ``OpenLoopController`` implements
   exactly that update, so enabling the layer with the default controller
   changes nothing measurable — pinned by test.
"""

from sim4wis.steering.tracking.controller import (
    AngleTrackingController,
    OpenLoopController,
    TrackingOutput,
    make_controller,
)
from sim4wis.steering.tracking.coupling import (
    CornerTracker,
    corner_tracking_step,
    make_corner_trackers,
)
from sim4wis.steering.tracking.feedback import AngleSensor
from sim4wis.steering.tracking.plant import CornerActuatorPlant

__all__ = [
    "AngleTrackingController",
    "AngleSensor",
    "CornerActuatorPlant",
    "CornerTracker",
    "OpenLoopController",
    "TrackingOutput",
    "corner_tracking_step",
    "make_controller",
    "make_corner_trackers",
]
