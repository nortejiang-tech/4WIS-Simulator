# Strategy plugins

Drop FMU files exported from Simulink (or other FMI 2.0 compliant tools) here,
along with a sidecar YAML describing input/output mapping.

## Layout

```
plugins/strategies/
├── my_mpc.fmu              ← FMU exported from Simulink
└── my_mpc.fmu.yaml         ← sidecar describing variable mapping
```

## Sidecar format

```yaml
name: my_mpc                 # name shown in the strategy selector
description: "MPC + ideal-Ackermann feedforward"
inputs:                      # FMU input variable name → sim4wis state path
  vx:        state.vx
  yaw_rate:  state.yaw_rate
  throttle:  driver.throttle
  steering:  driver.steering
  fz_fl:     state.fz[0]     # any indexed array attribute works
outputs:                     # ControlCommand field ← FMU output variable
  delta_cmd[0]:       delta_fl
  delta_cmd[1]:       delta_fr
  delta_cmd[2]:       delta_rl
  delta_cmd[3]:       delta_rr
  wheel_speed_cmd[0]: omega_fl
  wheel_speed_cmd[1]: omega_fr
  wheel_speed_cmd[2]: omega_rl
  wheel_speed_cmd[3]: omega_rr
step_size_ms: 10
```

## Exporting from Simulink

1. Build your controller in a Simulink model (.slx) with explicit Inport/Outport blocks
2. Tools → Build Model → Configure Embedded Coder for FMU
   (requires *Simulink Coder* + *FMI Kit for Simulink*)
3. Build → produces `<model>.fmu`
4. Copy the `.fmu` here, write the sidecar, restart the simulator
   (or `POST /api/plugins/reload`)

## Dependencies

The runtime needs the `fmpy` Python package:

```bash
pip install fmpy
```

If `fmpy` is not installed, the simulator still starts — your strategy just
isn't available until the dependency is installed.
