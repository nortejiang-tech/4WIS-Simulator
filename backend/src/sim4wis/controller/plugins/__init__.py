"""Plugin loaders for external control strategies.

Phase 2 plugin types:
    - FMUControllerStrategy        Simulink-exported FMU + YAML sidecar
    - MatlabEngineControllerStrategy (Step 14)
    - HTTPControllerStrategy       (future — ROS / external ECU bridge)

Plugins are discovered at startup by `loader.discover_and_register()` which
scans `<repo>/plugins/strategies/` for `*.fmu` files; each FMU is expected
to have a sibling `<name>.fmu.yaml` describing its input/output mapping.
"""
