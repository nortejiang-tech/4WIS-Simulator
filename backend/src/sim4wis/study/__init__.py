"""study — the research layer.

One level above `experiment`. Where an `Experiment` is "run this manoeuvre and
give me the channels", a `StudySpec` is "here is a question, here is the grid I
want it answered over, here are the metrics that answer it, and here is the
criterion that decides". The layer expands the grid onto the existing batch
runner, computes metrics over the resulting runs, aggregates across them, and
renders a verdict.

It exists because both research studies in this repository had to bypass the
experiment engine to get three things it does not provide — custom metrics,
cross-run aggregation, and in-loop injection — and rebuilt a bespoke harness
each time. See docs/agent_interface_design.md.

Three faces sit on top of it (GUI, CLI, MCP server) and none of them hold
logic; everything here is reachable over `/api/study/*`.
"""

from sim4wis.study.spec import (
    CompareSpec,
    Criterion,
    ExprMetric,
    ReportSpec,
    SolveFor,
    StudySpec,
    SweepAxis,
)

__all__ = [
    "Criterion",
    "CompareSpec",
    "ExprMetric",
    "ReportSpec",
    "SolveFor",
    "StudySpec",
    "SweepAxis",
]
