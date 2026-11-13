# Windows second verification attempt

Run: https://github.com/nortejiang-tech/4WIS-Simulator/actions/runs/43469624771
Source: 3173a8e585e9cd5e3aba8e07d7441773fc110245

- Backend: 1,119 passed, including all native and physics tests.
- Frontend unit tests: 54 passed.
- Browser: 56 passed, 2 failed. This run is **FAIL**, not a release acceptance.
- Failure 1: Windows screenshot baselines did not exist. All three actual images were manually inspected alongside the retained macOS baselines. The chart, roof view and geometry are consistent; font rasterization differs by platform. Reviewed Windows baselines are now retained.
- Failure 2: the shared software renderer produced only 113 frame intervals in the 4.5 s roof-view measurement at the default 1280×720 surface. This does not establish target-machine GPU performance. The cadence test now declares the same fixed 960×540 surface on every OS, keeps every numerical frame/median/autocorrelation threshold unchanged, and attaches measured diagnostics. This change narrows the test to cadence at a reproducible raster workload; it must not be reported as a 1280×720 Windows performance pass.
- Benchmark and packaging are independent of the browser cadence check and now retain their evidence even when a browser check fails. A failed job still remains failed; no test result is suppressed.
- All production code and numerical source fingerprints are unchanged by this follow-up.
