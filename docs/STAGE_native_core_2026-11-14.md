# STAGE — native simulation core

Baseline and decision: [ADR](adr/ADR-2026-11-14-native-simulation-core.md).
Preserve the pre-existing untracked `draft_03c081a7_folder/`. No merge/tag/release
of main is part of this independent-branch task.

1. DONE: baseline inspection, toolchains, initial profile, independent branch,
   user-selected core-only scope.
2. IN PROGRESS: C++ numerical blocks, checked Python bindings, explicit engine
   selection and fallback, package build integration.
3. PENDING: differential and independent physics verification, full backend and
   MCP regression, repeated whole-experiment benchmark.
4. PENDING: Mac/Windows native CI, branch publication, evidence and HANDOFF.

Planner owns this high-risk numerical migration, ABI safety, precision flags,
test oracles and final acceptance. No local Coder is assigned physics formulas
or acceptance commands. Record any narrow local Coder use separately; zero use
also requires a Markdown/JSON report.
