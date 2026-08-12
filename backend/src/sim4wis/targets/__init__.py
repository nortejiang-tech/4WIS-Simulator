"""Target sets and compliance — requirements as something the tool can check.

    spec        the schema: bands, entries, versioned sets
    library     shipped sets, plus whatever the user drops in targets_dir()
    measure     adapters turning study results and sizing runs into measurements
    compliance  the evaluator, and the table it produces
    report      HTML rendering of that table

See docs/targets_guide.md for how to write one.
"""

from sim4wis.targets.compliance import ComplianceReport, ComplianceRow, evaluate
from sim4wis.targets.library import BUILTIN, catalogue, get, load_all
from sim4wis.targets.measure import Measurement, from_sizing, from_study
from sim4wis.targets.spec import Band, TargetEntry, TargetError, TargetSet

__all__ = [
    "BUILTIN",
    "Band",
    "ComplianceReport",
    "ComplianceRow",
    "Measurement",
    "TargetEntry",
    "TargetError",
    "TargetSet",
    "catalogue",
    "evaluate",
    "from_sizing",
    "from_study",
    "get",
    "load_all",
]
