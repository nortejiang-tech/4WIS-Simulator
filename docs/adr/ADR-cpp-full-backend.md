# ADR — independent full C++ service route

Status: Accepted as the canonical product and release route by
[ADR-2026-11-14-cpp-canonical-release.md](ADR-2026-11-14-cpp-canonical-release.md).
Date: 2026-11-14. Branch: `codex/cpp-full-backend`. Implementation complete;
platform release evidence is recorded separately in
`docs/reports/cpp_v0.107.0/acceptance.md`. Base: e609c4c, the already committed
native simulation route. The native route remains only as a maintenance porting
bridge; the full C++ route is the product release line.

## Decision and scope

Move built-in simulation state, steering/controller execution, fault scheduling,
recording, path planning, metrics, load/sizing/target analysis, batch/study jobs,
file storage, HTTP/WebSocket and the 18-tool MCP interface into one C++17 binary.
Reuse the exact `backend/native/core.hpp` numerical equations from the native
route; do not trade integration accuracy or event timing for speed. C++17 also
avoids maintaining a second Rust translation of the same verified equations.
The existing React browser presentation remains, served by the native process.
This is a full backend migration, not a native-widget GUI rewrite.

User-authored Python/FMU/MATLAB code runs only when selected, in an explicit
compatibility worker. Built-in execution does not invoke Python; package tests
run the six frozen goldens with a deliberately nonexistent plugin interpreter.
The worker has response deadlines and separate stdout/diagnostic streams, but
is not a security sandbox. Real external FMU/MATLAB models and their licensed
runtimes remain outside the available acceptance evidence.

## Contracts, deterministic inputs and operations

Versioned OpenAPI, MCP schemas, defaults and catalogs are generated at development
time, then read directly by C++. The Python service is the independent reference
and rollback path. NumPy's pinned PCG64/normal generation is reproduced for
seeded sensor noise. Unknown physics overrides and incompatible schedules are
rejected rather than silently ignored. Background jobs retain the original
50-entry completed-history boundary and release completed thread resources.
Windows uses wide native paths for argv, environment settings and file I/O.
HTTP responses explicitly advertise the server's connection-close behavior.

Dependencies are pinned in `cpp_backend/third_party/manifest.json`; source and
licenses are bundled, hash-verified and built offline. MSVC uses the static CRT
and strict floating point; Apple Clang disables fast-math and contraction.
No silent fallback to the Python service exists in the built-in C++ route.

## Evidence and limits

Acceptance uses all recorded channels, exact fault boundaries, independent
force/impulse checks inherited from the shared core, built-in controller and
steering traces, analysis/study/target results, actual MCP lifecycle and tool
calls, existing browser workflows, and extracted Mac/Windows packages.
Repeated end-to-end benchmarks include full C++ process startup, KPI calculation
and JSON serialization/parsing. Raw samples and build identities are retained.

Software equivalence does not establish vehicle correlation. Reduced planar
physics, research multibody behavior, operator-selected parameters, observation-
only sensor faults, and symmetric duplicated single-wheel sizing loads remain
explicit limits. Main tracks this selected full C++ route; v0.105.0 remains the
previous stable rollback release. There is no user-data schema migration.
