# Windows initial verification: FAIL

Run 43465284538, commit 42c1e18, Windows Python 3.12.10: 1112 passed and 7 failed in 1155.90 s.

- Three test inputs relied on the host locale when reading UTF-8 YAML. Their readers now explicitly use UTF-8.
- One hashing test wrote its fixture through newline translation but expected the hash of LF bytes. The fixture now writes those exact bytes; production hashing remains byte-for-byte.
- A release-gate unit test mocked command() but accidentally executed command_capture(), requiring an uninstalled frontend. Both execution boundaries are now mocked.
- Two readiness tests found absent ignored manual source figures and platform-specific regenerated reference metrics. The original figures embedded in the current manual are now tracked; CI regenerates the derived analytic review through its existing gate before the readiness tests. Existing analytic tolerances are unchanged.

All native physics parity tests passed in this first attempt. No failing assertion was skipped or weakened. The rerun must pass the complete suite, frontend and package checks before a Windows preview is published.
