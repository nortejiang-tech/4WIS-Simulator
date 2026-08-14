"""Calibration / correlation workbench (C3).

The path from "engineering estimates" to "calibrated, with residuals" (D9).
Stage 3.1a — the data pipeline and residual panel — lives in
:mod:`sim4wis.calibration.residual`: bench CSV + same-condition procedure ->
per-channel residual table and overlay report. Stage 3.1b — parameter
identification — lives in :mod:`sim4wis.calibration.identify`: a
deterministic least-squares fit of the on-centre steering scalars against the
same comparison. Stage 3.1c (parameter-space margins in targets) builds on
top of both, and the guardrail from the plan holds throughout: fitting
reports, it never rewrites the parameter library.
"""
