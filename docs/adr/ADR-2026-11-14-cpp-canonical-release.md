# ADR — v0.107.0 full C++ backend becomes the sole product release line

## Status

Accepted, 2026-11-14. `codex/cpp-full-backend` is selected. Main is advanced by
fast-forward from the verified v0.105.0 base. A C++ numerical-core route remains
as an internal migration bridge and no longer has independent product releases.

## Context

- The B branch is a descendant of main's published v0.105.0 commit; the proposed
  adoption requires no merge conflict or history rewrite.
- Both routes use byte-identical `backend/native/core.hpp` and retain the same
  integration cadence. Differential tests found no extra physical fidelity in
  either route.
- On one Apple M5 Pro / 64 GiB host, eight repeated 14 s cases measured B at
  40.82–50.75× the Python reference and 14.90–20.69× A. B's timer additionally
  includes process startup, KPI calculation, and JSON write/read; Windows B
  performance was not measured.
- A and B passed their recorded Mac/Windows software, browser, API/MCP, and
  package gates. Vehicle/bench correlation and target-machine long-run checks
  remain pending. Exact evidence is in `docs/reports/cpp_v0.107.0/acceptance.md`.

## Decision

1. Make B v0.107.0 the sole current product release and advance `main` to the
   selected full C++ backend commit by fast-forward.
2. Keep `codex/native-simulation-core` for maintenance porting and comparison.
   It may be used as the interim Python-hosted/C++-kernel state while a change
   is validated, then the change must be ported and parity-checked in B before
   any product release. Do not publish future A-only releases.
3. Preserve the existing A and B preview releases as historical evidence, label
   them superseded, and direct users to stable v0.107.0. Do not delete tags or
   assets.
4. Use the existing tested Mac/Windows B package bytes for v0.107.0. Their
   source is the tested application commit `e79ca99`; the selected main/release
   commit adds the accepted release decision and docs without changing binaries.
5. Keep v0.105.0 available as the prior stable rollback point. If rollback is
   needed after main advances, move forward with a reviewed revert or hotfix;
   do not force-reset the public branch.

## Rationale

B provides the substantially higher measured batch throughput and removes the
Python service from built-in execution. A still has value as a smaller-scope
porting checkpoint, but maintaining it as a separately downloadable product
would create two release lines without a measured runtime benefit. Keeping it as
a bridge preserves the safer incremental implementation path while ensuring
users receive one tested architecture.

## Consequences

- `main`, `v0.107.0`, and the official Mac/Windows downloads identify B.
- Maintenance that is staged through A is incomplete until B parity and release
  acceptance pass; A artifacts are not product releases.
- Existing preview artifacts remain accessible but are explicitly historical.
- The C++ desktop interface is still the React browser UI, not native widgets.
  macOS 13.3 deployment is a compile target; the actual older-OS, target-device,
  physical bench, and vehicle checks remain pending.

