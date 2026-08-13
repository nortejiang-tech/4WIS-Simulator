"""Calibration / correlation workbench (C3).

The path from "engineering estimates" to "calibrated, with residuals" (D9).
Stage 3.1a — the data pipeline and residual panel — lives in
:mod:`sim4wis.calibration.residual`: bench CSV + same-condition procedure ->
per-channel residual table and overlay report. Stages 3.1b (parameter fitting)
and 3.1c (parameter-space margins in targets) build on top of it, and the
guardrail from the plan holds: the first version outputs residuals only and
never rewrites the parameter library.
"""
