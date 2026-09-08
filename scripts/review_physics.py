#!/usr/bin/env python3
"""Reproducible ASTRA review: independent oracles and step-size sensitivity.

No external data or fitted targets. --source-root permits read-only comparison
with a git-archive snapshot of the original backend. Results are verification,
not validation against an actual vehicle.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1] / 'backend/src')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
sys.path.insert(0, str(args.source_root.resolve()))
import numpy as np  # noqa: E402
from sim4wis.core.state import (  # noqa: E402
    ControlCommand,
    EnvironmentState,
    VehicleParams,
)
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel  # noqa: E402
from sim4wis.vehicle.multibody import MultiBodyModel  # noqa: E402
from sim4wis.vehicle.tire import LinearTireModel  # noqa: E402


def clean(**kw):
    return replace(VehicleParams(), drag_coeff_cd=0., rolling_resistance_coeff=0.,
                   aero_lift_coeff_front=0., aero_lift_coeff_rear=0.,
                   static_toe_front=0., static_toe_rear=0., camber_thrust_coeff=0.,
                   longitudinal_mode='torque', **kw)


class KnownForce(LinearTireModel):
    def forces(self, alpha, kappa, fz, mu):
        return 0., .1 * fz, 0.


def run_turn(cls, dt, sign=1., tire_kind="linear"):
    p = clean(tire_model=tire_kind)
    model = cls(p)
    model.state.vx = 10.
    model.state.wheel_omega[:] = 10. / p.tire_radius
    cmd = ControlCommand.zero()
    for i in range(round(3. / dt)):
        cmd.delta_cmd[:2] = sign * .06 if i * dt >= .2 else 0.
        state = model.step(dt, cmd, EnvironmentState(mu=.9))
    return [float(x) for x in (state.vx, state.vy, state.yaw_rate,
                               state.x, state.y, state.psi)]


t0 = time.monotonic()
out = {'source_root': str(args.source_root.resolve()), 'scope': 'internal verification; no external vehicle validation', 'models': {}}
for cls, tire_kind in ((c, t) for c in (SimplifiedDynamicModel, MultiBodyModel) for t in ("linear", "pacejka")):
    key = cls.__name__ + ":" + tire_kind
    model = cls(clean(cg_to_front=1.), KnownForce())
    state = model.step(1e-5, ControlCommand.zero(), EnvironmentState())
    known = {'yaw_acceleration_rad_s2': state.yaw_rate / 1e-5,
             'expected_yaw_acceleration': 0., 'ay_m_s2': state.ay, 'expected_ay': .981}
    model = cls(clean(cg_to_front=1., tire_model=tire_kind))
    model.state.vx = 10.
    model.state.wheel_omega[:] = 10. / model.params.tire_radius
    for _ in range(200):
        state = model.step(.005, ControlCommand.zero(), EnvironmentState(mu=.9))
    coast = {'speed_error_m_s': state.vx - 10., 'position_error_m': state.x - 10.,
             'yaw_rad': state.psi, 'heave_m': state.z, 'pitch_rad': state.pitch}
    steps = [.005, .0025, .00125, .000625]
    values = [run_turn(cls, dt, tire_kind=tire_kind) for dt in steps]
    reference = np.asarray(values[-1][:3])
    errors = [float(np.linalg.norm((np.asarray(v[:3]) - reference) / [10., 1., .1])) for v in values]
    right = run_turn(cls, .005, -1., tire_kind=tire_kind)
    mirror = np.asarray(values[0]) - np.asarray(right) * [1., -1., -1., 1., -1., -1.]
    out['models'][key] = {'force_through_cg': known, 'unforced_coast_1s': coast,
        'step_size_s': steps, 'final_vx_vy_r_x_y_psi': values, 'scaled_endpoint_error': errors,
        'left_right_mirror_max_abs_error': float(max(abs(mirror))),
        'fine_step_yaw_relative_difference_pct': abs(values[0][2] / values[-1][2] - 1.) * 100.}
    print(key, json.dumps(out['models'][key], ensure_ascii=False), flush=True)
out['elapsed_s'] = time.monotonic() - t0
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
