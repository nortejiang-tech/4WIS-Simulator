# C++ backend — v0.107.0

This branch keeps the existing React browser presentation and runs simulation,
controllers, faults, experiments, studies, metrics, target checks, storage,
HTTP/WebSocket and MCP in a standalone C++17 process. The Python service in
`backend/` is the differential test oracle and optional user-code adapter; it
is not launched for built-in execution. This is the canonical product backend
on `main`. The native-kernel route remains available as a maintenance porting
bridge and does not publish separate product releases.

Build offline with `cmake -S cpp_backend -B cpp_backend/build -DCMAKE_BUILD_TYPE=Release`
and `cmake --build cpp_backend/build --config Release --parallel 4`.
Mac release builds add `-DCMAKE_OSX_DEPLOYMENT_TARGET=13.3` to the configure command.
The floating-point `std::to_chars` API requires macOS 13.3 or newer. The
deployment target is not a claim of runtime testing on every older macOS.
Run `sim4wis-cpp --root REPOSITORY --serve --port 8020`; build the browser assets
with `npm ci && npm run build` under `frontend` first. The kernel-only route
normally uses 8010, so both can run side by side with separate data folders.
`--self-test` runs six frozen golden cases without Python. `--print-config`
emits a complete MCP configuration; `--mcp` attaches to the matching C++
service or starts one locally. STDIO contains only JSON-RPC; diagnostics go to
stderr. MCP uses the advertised wheel indexes 0=FL, 1=FR, 2=RL, 3=RR; the HTTP
fault API additionally accepts legacy wheel names and fault aliases.

All dependencies are pinned sources bundled in `third_party/sources.tar.gz`.
CMake verifies `sources.sha256` and extracts into the build directory without
network access. Upstream identities, archive checksums and licenses are
retained beside the bundle. Strict floating point is enabled; integration
steps, fault event splitting and force budgets are not enlarged to gain speed.

Python strategies and the existing FMU/MATLAB strategy adapters are optional
user-code plugins run in a separate compatibility process. Portable archives
include a Python runtime and basic dependencies. `SIM4WIS_PLUGIN_PYTHON` can
select another interpreter. FMU/MATLAB engines, model files, licenses and
additional packages remain external prerequisites, exactly as on the original
route. This process is crash/timeout containment, not an untrusted-code sandbox.
The file-gain angle-controller and inverse solver/envelope features retain the
original framework/refusal boundaries; they do not silently simulate missing
adapters. Real FMU/MATLAB models and vehicle/bench correlation are not accepted
by software parity tests.

Validation: `python -m pytest cpp_backend/tests -n 4` compares numerical traces,
all built-in steering architectures and controllers, random streams, paths,
load analysis, metrics, study/target results, API sessions and official MCP.
The existing 58 browser workflows run with `PW_CPP_EXECUTABLE` pointing at the
binary. `tools/package.py` creates archives; `tools/check_package.py` extracts
them to a path containing Chinese characters and spaces and tests the shipped binary, production
frontend, multifault cases, exact artifacts, MCP and plugin runtime.

Final platform evidence and measured performance are in
`docs/reports/cpp_v0.107.0/acceptance.md` and `comparison.html`. Software
consistency is distinct from physical validation. The multibody model remains
research-grade, the planar model is reduced order, sensor faults affect only
observations in this version, and actuator sizing uses a symmetric duplicated
single-wheel load approximation. See the shared sizing correction report for
the original mixed-wheel interpolation defect fixed on both candidate routes.

Rollback is stopping this executable and starting the previous stable v0.105.0
package. No production database migration is performed.
