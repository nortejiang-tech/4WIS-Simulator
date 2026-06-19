"""Controller strategies.

Built-in (Phase 1):
    - ackermann          Front-axle Ackermann (legacy reference)
    - ideal_ackermann    4WIS ideal Ackermann — all four wheels share one ICR
    - rear_steer         Rear wheels in-phase / counter-phase with front
    - crab               All four wheels parallel — pure lateral translation
    - zero_radius        Spin in place around vehicle center

Plug-in mechanisms (Phase 2+): Simulink FMU, MATLAB Engine, external socket.

Filled in Step 2.
"""
