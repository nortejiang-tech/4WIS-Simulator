# Independent full C++ service route

Status: implementation in progress; acceptance pending. Base: e609c4c, native simulation route. Branch: codex/cpp-full-backend. The native route remains independently releasable and main is untouched.

The full route reuses the exact verified `backend/native/core.hpp` equations. State ownership, controllers, faults, experiments, storage, API/WebSocket and MCP are being moved into a standalone C++ executable. The React presentation remains the existing browser interface; C++ serves its production assets. User-authored Python strategies require a separately identified compatibility worker, if used, and must never silently become the execution engine for built-in simulations.

No performance claim or full-function acceptance is made before the route inventory, differential numerical traces, existing browser workflows, MCP lifecycle, and Mac/Windows extracted packages are checked. Missing capability must produce a visible error rather than a placeholder result. The original Python service remains the test oracle and rollback route, not a hidden C++ service dependency.

Dependencies are pinned upstream sources in `cpp_backend/third_party/manifest.json`, with licenses retained. Normal release builds use these vendored files without fetching floating revisions. Build with strict floating point; do not enlarge simulation steps or relax the existing physical tests to obtain speed.
