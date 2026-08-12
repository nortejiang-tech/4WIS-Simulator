"""Target sets — requirements, as a document the tool can check.

What this is *not*: a second copy of `study.criteria`. The two look alike and
answer different questions, and conflating them is how a tool ends up with
thresholds nobody can trace.

    study criteria      belong to a **question**. Written per study, thrown
                        away with it, chosen to make one claim falsifiable.
                        "does raising rear authority past 6 deg cost overshoot?"

    targets             belong to a **product**. Named, versioned, owned,
                        reused across every study and every configuration,
                        and sourced. "this programme requires parking effort
                        at or below 5.0 N.m."

The second is what a system engineer actually works against, and the compliance
table it produces answers the question this tool exists to answer for them:
**does this steering configuration meet its requirements?** — not "here are
some curves".

Three decisions are worth stating because they are what make it a requirements
document rather than a pile of numbers.

**Every requirement is a band.** One-sided limits are the half-open case, not a
separate kind. It sounds like a small unification and it is not: an on-centre
torque gradient that is too *low* is as wrong as one that is too high, a
centring torque that is too strong is as wrong as one too weak, and a schema
that can only say "less than" quietly pushes every such requirement into being
written as its one-sided half. So `Band` is the primitive and everything
reduces to it.

**Target and limit are different things.** Requirement practice distinguishes
the value you are trying to hit from the value you must not cross; collapsing
them into one number loses the information that decides whether a result is
"fine" or "we got away with it". A configuration inside the limit but outside
the target is MARGINAL, and a review reads that differently from MET.

**`source` is mandatory.** The first question in any requirements review is who
says so. A number with no provenance is a preference, and it will be argued
away by whoever wants it argued away. The schema refuses to store one.
"""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

#: Same comparison grammar as `study.spec` — deliberately one language in the
#: product rather than two. Reproduced rather than imported because a target
#: rejects `!=` (see `Band.parse`), and a schema that accepts a clause it
#: cannot represent is worse than a little duplication.
_CLAUSE_RE = re.compile(
    r"^\s*(abs\s+)?(<=|>=|<|>|==|!=)\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$"
)


class TargetError(ValueError):
    """A malformed target set, or one that cannot be represented."""


@dataclass(frozen=True)
class Band:
    """An acceptable interval, possibly half-open.

    `lo`/`hi` of None mean unbounded on that side. `use_abs` compares the
    magnitude, which is how a symmetric requirement ("deviation within 1 deg
    either way") is normally written.
    """

    lo: float | None = None
    hi: float | None = None
    lo_strict: bool = False          # True → lo < v rather than lo <= v
    hi_strict: bool = False
    use_abs: bool = False

    # ---- construction -------------------------------------------------------

    @staticmethod
    def parse(value: Any) -> Band:
        """From a clause string (`"<= 5.0"`) or an inclusive pair (`[0.4, 1.2]`)."""
        if isinstance(value, Band):
            return value
        if isinstance(value, (list, tuple)):
            if len(value) != 2:
                raise TargetError(
                    f"a band pair needs exactly [lo, hi]; got {list(value)!r}"
                )
            lo, hi = float(value[0]), float(value[1])
            if lo > hi:
                raise TargetError(f"band [{lo}, {hi}] is inverted")
            return Band(lo=lo, hi=hi)
        if not isinstance(value, str):
            raise TargetError(f"cannot read {value!r} as a band")

        m = _CLAUSE_RE.match(value)
        if m is None:
            raise TargetError(
                f"unparseable clause {value!r}. Expected e.g. '<= 5.0', '>= 0.4', "
                "'abs <= 1.0', or a pair [0.4, 1.2]"
            )
        use_abs, op, num = bool(m.group(1)), m.group(2), float(m.group(3))
        if op == "!=":
            raise TargetError(
                "'!=' is not a requirement: it excludes a single value of measure "
                "zero and nothing can be designed to it. Write the band you mean."
            )
        if op in ("<", "<="):
            return Band(hi=num, hi_strict=(op == "<"), use_abs=use_abs)
        if op in (">", ">="):
            return Band(lo=num, lo_strict=(op == ">"), use_abs=use_abs)
        return Band(lo=num, hi=num, use_abs=use_abs)      # '=='

    # ---- evaluation ---------------------------------------------------------

    def _v(self, value: float) -> float:
        return abs(value) if self.use_abs else value

    def contains(self, value: float) -> bool:
        v = self._v(value)
        if self.lo is not None:
            if (v <= self.lo) if self.lo_strict else (v < self.lo):
                return False
        if self.hi is not None:
            if (v >= self.hi) if self.hi_strict else (v > self.hi):
                return False
        return True

    def margin(self, value: float) -> float | None:
        """Signed headroom as a fraction, or None where it is undefined.

        Positive means inside with room, zero means exactly on a bound,
        negative means outside. The normalisation differs by shape because the
        engineering meaning does:

        * half-open — fraction of the bound still unused: ``(hi - v) / |hi|``
        * closed band — fraction of the half-width still unused, so the centre
          of the band is 1.0 and either edge is 0.0

        None where there is nothing to normalise against: an unbounded band, a
        bound of exactly zero, or an equality requirement. Reporting `inf` or
        a percentage of zero there would be a made-up number, and the report
        already leaves undefined ratios blank (`compare` does the same for a
        zero reference).
        """
        v = self._v(value)
        if self.lo is not None and self.hi is not None:
            half = (self.hi - self.lo) / 2.0
            if half <= 0.0:
                return None                     # equality: no width to speak of
            centre = (self.hi + self.lo) / 2.0
            return (half - abs(v - centre)) / half
        if self.hi is not None:
            return None if self.hi == 0.0 else (self.hi - v) / abs(self.hi)
        if self.lo is not None:
            return None if self.lo == 0.0 else (v - self.lo) / abs(self.lo)
        return None                             # unbounded

    def encloses(self, other: Band) -> bool:
        """True when `other` lies entirely inside this band."""
        if self.use_abs != other.use_abs:
            return False
        if self.lo is not None and (other.lo is None or other.lo < self.lo):
            return False
        if self.hi is not None and (other.hi is None or other.hi > self.hi):
            return False
        return True

    # ---- presentation -------------------------------------------------------

    def describe(self) -> str:
        def n(x: float) -> str:
            return f"{x:g}"

        pre = "|·| " if self.use_abs else ""
        if self.lo is not None and self.hi is not None:
            if self.lo == self.hi:
                return f"{pre}= {n(self.lo)}"
            return f"{pre}∈ [{n(self.lo)}, {n(self.hi)}]"
        if self.hi is not None:
            return f"{pre}{'<' if self.hi_strict else '≤'} {n(self.hi)}"
        if self.lo is not None:
            return f"{pre}{'>' if self.lo_strict else '≥'} {n(self.lo)}"
        return "—"

    def to_dict(self) -> dict[str, Any]:
        return {
            "lo": self.lo, "hi": self.hi, "use_abs": self.use_abs,
            "text": self.describe(),
        }


class TargetEntry(BaseModel):
    """One requirement: a metric, an operating point, and what it must do."""

    model_config = ConfigDict(protected_namespaces=())

    id: str = Field(..., min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_\-]+$")
    metric: str = Field(..., min_length=1)
    #: Operating point, in the same selector language study criteria use:
    #: "all", or equality filters joined by "and" — "scenario == parking_full_lock".
    at: str = "all"
    #: The bound that must not be crossed. Required: an entry with only a
    #: desired value judges nothing.
    limit: Any
    #: The value being aimed for. Optional, and must lie inside `limit`.
    target: Any = None
    severity: Literal["must", "should"] = "must"
    unit: str = ""
    #: Where the number came from. Mandatory on purpose — see the module note.
    source: str = Field(..., min_length=1)
    rationale: str = ""

    @field_validator("limit", "target")
    @classmethod
    def _parse_band(cls, v: Any) -> Any:
        if v is None:
            return None
        return Band.parse(v)

    @model_validator(mode="after")
    def _check(self) -> TargetEntry:
        limit: Band = self.limit
        if limit.lo is None and limit.hi is None:
            raise TargetError(f"target {self.id!r}: an unbounded limit requires nothing")
        target: Band | None = self.target
        if target is not None and not limit.encloses(target):
            raise TargetError(
                f"target {self.id!r}: the desired band {target.describe()} is not "
                f"inside the limit {limit.describe()} — a goal looser than the "
                "requirement is a mistake, not a relaxation"
            )
        return self

    @property
    def limit_band(self) -> Band:
        return self.limit

    @property
    def target_band(self) -> Band | None:
        return self.target

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "metric": self.metric,
            "at": self.at,
            "limit": self.limit_band.to_dict(),
            "target": self.target_band.to_dict() if self.target_band else None,
            "severity": self.severity,
            "unit": self.unit,
            "source": self.source,
            "rationale": self.rationale,
        }


class TargetSet(BaseModel):
    """A named, versioned set of requirements for one product configuration."""

    model_config = ConfigDict(protected_namespaces=())

    name: str = Field(..., min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_\-]+$")
    version: int = Field(1, ge=1)
    title: str = ""
    #: What configuration these apply to. A target set that does not say what
    #: it is for will be applied to something it was never written for.
    applies_to: str = Field(..., min_length=1)
    owner: str = ""
    notes: str = ""
    entries: list[TargetEntry] = Field(..., min_length=1)

    @model_validator(mode="after")
    def _unique_ids(self) -> TargetSet:
        ids = [e.id for e in self.entries]
        dupes = sorted({i for i in ids if ids.count(i) > 1})
        if dupes:
            raise TargetError(f"duplicate target id(s): {', '.join(dupes)}")
        return self

    @property
    def ref(self) -> str:
        """`name@version` — how a study pins the requirements it was judged on."""
        return f"{self.name}@{self.version}"

    def digest(self) -> str:
        payload = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]

    def metrics(self) -> list[str]:
        seen: list[str] = []
        for e in self.entries:
            if e.metric not in seen:
                seen.append(e.metric)
        return seen

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "version": self.version,
            "title": self.title,
            "applies_to": self.applies_to,
            "owner": self.owner,
            "notes": self.notes,
            "entries": [e.to_dict() for e in self.entries],
        }
