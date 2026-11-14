# STAGE — native simulation core, software delivery complete

Baseline and decision: [ADR](adr/ADR-2026-11-14-native-simulation-core.md).
The branch `codex/native-simulation-core` is retained as an intermediate
maintenance port into the selected full C++ backend. Product main/release is
v0.107.0; A is not an independent release line.

1. DONE: inspected baseline, toolchains and profile; created independent branch for user-selected C++ kernel scope.
2. DONE: checked C++ bindings, explicit engine selection/fallback, model and tire hot paths, package integration.
3. DONE: independent force/impulse/virtual-work checks, differential and complete regressions, MCP, five-repeat whole-experiment benchmarks. Shared sizing-source and cross-version step-budget defects repaired in both routes.
4. DONE: Mac and native Windows tests, extracted packages, package-local Windows C++ DLL proof, branch publication preparation and final evidence. See [acceptance](reports/native_v0.106.0/acceptance.md) for exact commits, artifacts and test scope.

Selected route and maintenance handoff: [HANDOFF](HANDOFF_cpp_routes_2026-11-14.md).
User target-machine GUI soak, physical inputs, bench and vehicle correlation remain PENDING.
The pre-existing untracked draft directory is preserved and excluded. Main is
advanced through B by fast-forward; no user-data migration/rollback is performed.

Planner owned this high-risk numerical migration, ABI, precision, platform packaging,
independent oracles and acceptance. EVO-Coder calls: 0; accounting is in the release
Markdown/JSON reports. The final release notes and authoritative GitHub read-back
establish publication status; this stage document does not substitute for them.
