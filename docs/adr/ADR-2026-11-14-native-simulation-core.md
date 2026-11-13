# ADR — C++ numerical core behind the existing application

Date: 2026-11-14. Baseline: v0.105.0 / a1a4d40c91ca21f5a8f77fcd95ff55770f1f6784.
Branch: `codex/native-simulation-core`. Status: implementation accepted by user.

The user chose a native simulation core, retaining the UI, Python services and
MCP. The branch must remain independent of main and the existing release.

## Evidence and decision

A 3 s, 600 outer-step, off-grid staggered-fault experiment took 0.623 s
(planar) and 0.730 s (multibody) on the development Mac before changes. These
are initial single-run observations, not a benchmark claim. cProfile attributed
33% / 44% to body derivative evaluations, 20% / 18% to linkage geometry, and
8% / 7% to wheel-spin updates. Profile overhead is excluded from wall times.

Use C++17 for these numerical blocks, with a small CPython extension and
contiguous float64 buffers. No additional numerical engine or solver dependency.
Setuptools builds the extension with the host compiler. Disable fast-math and
floating-point contraction. The Python reference remains callable explicitly.

Port both body RK4 integrations, built-in linear/Pacejka tyre forces, the
capacity-bracketed nonlinear backward-Euler wheel solver, and planar linkage
geometry. Preserve suspension/K&C formulas, off-centre CG transformations,
previous-step compliance loads, tyre relaxation, solver tolerances, brake
zero-crossing treatment, integration order and fault-boundary splitting.
Steering plants/controllers, fault scheduling, recordings and transports remain
in Python. A custom Python tyre or overridden model hook must keep its Python
behavior; never silently replace an extension with a built-in force law.

Engine selection is explicit (`SIM4WIS_ENGINE=auto|python|native`), and metadata
reports availability and selection. `auto` can run the reference when no binary
is installed. `native` must report a missing binary as an error. Numerical
errors from a loaded native core propagate; they never trigger a silent retry.

## Acceptance and boundaries

Differential tests compare all recorded channels and exact event times across
both engines, both body models and built-in tyres, with simultaneous/staggered
faults, reverse, braking, load sensitivity, K&C and detailed steering. Independent
force/moment and angular-impulse assertions remain. Require full existing backend
regression, actual MCP stdio tests, and native builds/tests on Mac and Windows.
Report median end-to-end timings over repeated, alternating runs, with warmup,
hardware/compiler details and raw samples. No speed claim from microbenchmarks
alone. A performance acceptance target is median speedup >= 1.5 on representative
default-model fault workloads; correctness is a separate prerequisite.

Reference parity establishes preservation of v0.105.0, not vehicle correlation.
The first-order partitioned scheme and existing physical limitations remain.
Rollback is `SIM4WIS_ENGINE=python` or checkout of v0.105.0; no migration of user
projects or destructive data operation is required.

## Build references

- [CPython buffer protocol](https://docs.python.org/3/c-api/buffer.html)
- [Setuptools extension modules](https://setuptools.pypa.io/en/latest/userguide/ext_modules.html)
