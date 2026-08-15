"""sim4wis command line — the second face on the study layer.

`sim4wis` with no arguments still starts the server, because that is what it
has always done and the portable launchers, which call uvicorn directly, are
not the only thing that might invoke it.

Everything else goes through `/api/study/*` — the same contract the MCP server
will speak — so a capability can never exist on the command line and be
missing from the agent interface. When no backend is reachable the commands
fall back to running the study layer in-process and say so; refusing to work
without a server would be friction with nothing behind it, since it is the
same code either way.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

DEFAULT_BACKEND = os.environ.get("SIM4WIS_BACKEND_HTTP", "http://127.0.0.1:8010")
_TIMEOUT = 10.0


# ---------------------------------------------------------------------------
# Transport
# ---------------------------------------------------------------------------


class BackendUnavailableError(RuntimeError):
    pass


def _get(base: str, path: str, params: dict[str, Any] | None = None) -> Any:
    url = f"{base.rstrip('/')}{path}"
    if params:
        url += "?" + urllib.parse.urlencode(params)
    try:
        with urllib.request.urlopen(url, timeout=_TIMEOUT) as r:   # noqa: S310
            return json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"{e.code} {e.reason}: {e.read().decode('utf-8', 'replace')[:400]}") from e
    except (urllib.error.URLError, OSError) as e:
        raise BackendUnavailableError(str(e)) from e


def _post(base: str, path: str, body: Any, timeout: float = _TIMEOUT) -> Any:
    req = urllib.request.Request(
        f"{base.rstrip('/')}{path}",
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:    # noqa: S310
            return json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"{e.code} {e.reason}: {e.read().decode('utf-8', 'replace')[:400]}") from e
    except (urllib.error.URLError, OSError) as e:
        raise BackendUnavailableError(str(e)) from e


def _backend_up(base: str) -> bool:
    try:
        _get(base, "/health")
        return True
    except Exception:                                # noqa: BLE001
        return False


# ---------------------------------------------------------------------------
# Spec loading + printing
# ---------------------------------------------------------------------------


def _load_spec(path: str) -> dict[str, Any]:
    p = Path(path)
    if not p.is_file():
        raise SystemExit(f"spec not found: {path}")
    text = p.read_text(encoding="utf-8")
    if p.suffix.lower() in (".yaml", ".yml"):
        import yaml

        return yaml.safe_load(text)
    return json.loads(text)


def _fmt(v: Any, digits: int = 5) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{digits}g}"
    return str(v)


def _table(headers: list[str], rows: list[list[Any]]) -> str:
    cells = [[_fmt(c) for c in row] for row in rows]
    widths = [len(h) for h in headers]
    for row in cells:
        for i, c in enumerate(row):
            widths[i] = max(widths[i], len(c))
    line = "  ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    sep = "  ".join("-" * w for w in widths)
    body = "\n".join("  ".join(c.ljust(widths[i]) for i, c in enumerate(row)) for row in cells)
    return f"{line}\n{sep}\n{body}"


def _print_summary(s: dict[str, Any]) -> None:
    print(f"\n{s['study']} — {s['question']}")
    prov = s.get("provenance") or {}
    print(f"  model={s['model']}  digest={s['spec_digest']}  git={prov.get('git_sha')}  "
          f"params={prov.get('params_hash')}  {s.get('elapsed_s')}s")

    axes = sorted({k for r in s["rows"] for k in r["coords"]})
    metrics = s["metrics"]
    headers = [*axes, *metrics]
    rows = [[*(r["coords"].get(a) for a in axes),
             *(r["metrics"].get(m) for m in metrics)] for r in s["rows"]]
    print("\n" + _table(headers, rows))

    verdicts = s.get("verdicts") or []
    if verdicts:
        print()
        for v in verdicts:
            mark = "PASS" if v["passed"] else "FAIL"
            extra = ""
            if not v["passed"] and v.get("worst_label"):
                extra = f"  worst: {v['worst_label']} = {_fmt(v['worst_value'])}"
            elif v.get("note"):
                extra = f"  ({v['note']})"
            print(f"  [{mark}] {v['metric']} {v['must']} @ {v['at']}"
                  f"  {v['failed']}/{v['checked']} failed{extra}")

    comp = s.get("compliance")
    if comp:
        _print_compliance(comp)

    for w in s.get("warnings") or []:
        print(f"  ! {w}")
    errs = [(r["label"], m, e) for r in s["rows"] for m, e in (r.get("errors") or {}).items()]
    for label, m, e in errs[:10]:
        print(f"  ! {label}: {m}: {e}")
    if s.get("report_path"):
        print(f"\n  report: {s['report_path']}")


_MARK = {"met": "MET ", "marginal": "MARG", "violated": "FAIL", "not_evaluated": "----"}


def _print_compliance(c: dict[str, Any]) -> None:
    """The compliance table, worst first."""
    counts = c["counts"]
    print(f"\n目标符合性 {c['ref']} — {c['verdict_label'].upper()} "
          f"({c['verdict']})  覆盖 {c['coverage']:.0%}"
          f"  超标 {counts['violated']} / 边际 {counts['marginal']} / "
          f"未评估 {counts['not_evaluated']}")
    order = {"violated": 0, "not_evaluated": 1, "marginal": 2, "met": 3}
    rows = []
    for r in sorted(c["rows"], key=lambda r: (order[r["status"]], r["id"])):
        margin = "—" if r["margin_pct"] is None else f"{r['margin_pct']:+.1f}%"
        detail = r["note"] or (f"worst: {r['worst_label']}"
                               if r["status"] in ("violated", "marginal")
                               and r["worst_label"] else "")
        rows.append([_MARK[r["status"]], "must" if r["severity"] == "must" else "-",
                     r["id"], r["at"], r["target"] or "—", r["limit"],
                     _fmt(r["worst_value"]), margin, detail])
    print(_table(["", "级别", "需求", "工况", "目标", "限值", "实测", "余量", "说明"], rows))
    if any(r["severity"] != "must" for r in c["rows"]):
        print("  「级别」为空的行是参考项：会记录、不参与集合判定，也不计入覆盖率。")
    if c["verdict"] == "incomplete":
        print("  「未评估」不是通过：这些要求本次没有测到，或测量值不可信。")


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------


def _local_run(payload: dict[str, Any], dry: bool) -> dict[str, Any]:
    from sim4wis.study.runner import dry_run, run_sync
    from sim4wis.study.spec import StudySpec

    spec = StudySpec.model_validate(payload)
    if dry:
        return dry_run(spec)
    _, result = run_sync(spec)
    return result.to_summary()


def cmd_study_run(args: argparse.Namespace) -> int:
    payload = _load_spec(args.spec)
    use_http = not args.local and _backend_up(args.backend)

    if args.dry_run:
        out = (_post(args.backend, "/api/study/dry-run", payload) if use_http
               else _local_run(payload, dry=True))
        if args.json:
            print(json.dumps(out, ensure_ascii=False, indent=1))
            return 0 if out.get("ok") else 1
        print(f"spec: {'OK' if out['ok'] else 'INVALID'}   digest={out.get('spec_digest')}")
        print(f"grid: {out['grid']} cell(s), {out['sim_seconds']}s simulated, "
              f"~{out['est_wall_seconds']}s wall (rough)")
        if out.get("cells"):
            print("cells: " + ", ".join(out["cells"][:12])
                  + (" …" if len(out["cells"]) > 12 else ""))
        for w in out.get("warnings", []):
            print(f"  ! {w}")
        for p in out.get("problems", []):
            print(f"  ✗ {p}")
        return 0 if out["ok"] else 1

    if not use_http:
        print("(no backend reachable — running in-process)", file=sys.stderr)
        summary = _local_run(payload, dry=False)
    else:
        started = _post(args.backend, "/api/study/run", payload)
        job_id = started["job_id"]
        est = started.get("estimate", {})
        print(f"study job {job_id}: {est.get('runs')} run(s), "
              f"~{est.get('est_wall_seconds')}s", file=sys.stderr)
        import time

        while True:
            job = _get(args.backend, f"/api/study/jobs/{job_id}")
            if job["status"] != "running":
                break
            time.sleep(0.5)
        if job["status"] == "error":
            print(f"study failed: {job['error']}", file=sys.stderr)
            return 1
        summary = job["result"]

    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=1))
    else:
        _print_summary(summary)
    passed = summary.get("all_passed")
    return 0 if passed is not False else 2


def cmd_study_list(args: argparse.Namespace) -> int:
    if _backend_up(args.backend) and not args.local:
        data = _get(args.backend, "/api/study")["studies"]
    else:
        from sim4wis.study import store

        data = store.list_studies()
    if args.json:
        print(json.dumps(data, ensure_ascii=False, indent=1))
        return 0
    if not data:
        print("no studies yet")
        return 0
    print(_table(
        ["study_id", "study", "model", "rows", "passed", "created"],
        [[d["study_id"], d["study"], d["model"], d["rows"], d.get("all_passed"),
          d.get("created_at", "")] for d in data],
    ))
    return 0


def cmd_study_show(args: argparse.Namespace) -> int:
    if _backend_up(args.backend) and not args.local:
        s = _get(args.backend, f"/api/study/{args.study_id}")
    else:
        from sim4wis.study import store

        s = store.load(args.study_id)
    if args.json:
        print(json.dumps(s, ensure_ascii=False, indent=1))
    else:
        _print_summary(s)
    return 0


def cmd_study_trace(args: argparse.Namespace) -> int:
    params = {"channels": args.channels, "max_points": args.max_points}
    if _backend_up(args.backend) and not args.local:
        data = _get(args.backend, f"/api/study/trace/{args.run_id}", params)
    else:
        from sim4wis.experiment import store as run_store

        meta = run_store.load_run_meta(args.run_id)
        n = int(meta.get("n_samples") or 0)
        dec = max(1, -(-n // args.max_points)) if n else 1
        names = [c.strip() for c in args.channels.split(",") if c.strip()]
        data = {
            "run_id": args.run_id, "decimate": dec, "of_total": n,
            "channels": run_store.load_run_channels(args.run_id, names, decimate=dec),
        }
    print(json.dumps(data, ensure_ascii=False))
    return 0


def cmd_targets_list(args: argparse.Namespace) -> int:
    from sim4wis.targets.library import catalogue

    items = catalogue()
    if args.json:
        print(json.dumps(items, ensure_ascii=False, indent=2))
        return 0
    print(_table(
        ["ref", "标题", "适用", "条数", "强制", "内置"],
        [[c["ref"], c["title"], c["applies_to"], c["entries"], c["must"], c["builtin"]]
         for c in items],
    ))
    return 0


def cmd_targets_show(args: argparse.Namespace) -> int:
    from sim4wis.targets.library import get

    ts = get(args.ref)
    if args.json:
        print(json.dumps(ts.to_dict(), ensure_ascii=False, indent=2))
        return 0
    print(f"{ts.ref} — {ts.title}\n  适用：{ts.applies_to}\n  归口：{ts.owner or '—'}"
          f"\n  digest：{ts.digest()}")
    if ts.notes:
        print(f"  说明：{ts.notes}")
    print()
    print(_table(
        ["需求", "指标", "工况", "目标", "限值", "级别", "依据"],
        [[e.id, e.metric, e.at,
          e.target_band.describe() if e.target_band else "—",
          e.limit_band.describe(), e.severity, e.source] for e in ts.entries],
    ))
    return 0


def cmd_targets_check(args: argparse.Namespace) -> int:
    """Check a target set against a fresh actuator-sizing pass."""
    from pathlib import Path

    from sim4wis.core.state import VehicleParams
    from sim4wis.project.vehicle_profiles import load_profile_params
    from sim4wis.steering.sizing import size_actuator
    from sim4wis.targets import compliance, library, measure
    from sim4wis.targets import report as target_report

    if args.vehicle:
        params = load_profile_params(args.vehicle)
    else:
        params = VehicleParams()
    if not params.steering_system.enabled:
        import dataclasses

        params = dataclasses.replace(
            params,
            steering_system=dataclasses.replace(params.steering_system, enabled=True),
        )
    req = size_actuator(params)
    rep = compliance.evaluate(library.get(args.ref), measure.from_sizing(req))
    margins_dict = None
    if getattr(args, "margins", False):
        from sim4wis.targets import robustness

        axes = robustness.DEFAULT_AXES
        if getattr(args, "axes", None):
            wanted = {a.strip() for a in args.axes.split(",") if a.strip()}
            axes = tuple(a for a in robustness.DEFAULT_AXES if a.name in wanted)
            unknown = wanted - {a.name for a in axes}
            if unknown:
                print(f"unknown axis(es) {sorted(unknown)}; "
                      f"known: {[a.name for a in robustness.DEFAULT_AXES]}")
                return 2
        print(f"parameter-space margins: {len(axes)} axes × both directions, "
              "bisecting a sizing pass each step …")
        margins = robustness.axis_margins(
            params, library.get(args.ref), axes,
            bisect_steps=args.bisect_steps,
            progress=lambda name, n: print(f"  [{n:3d}] {name} done", flush=True),
        )
        if getattr(args, "fit_record", None):
            n = robustness.annotate_with_fit(margins, args.fit_record)
            print(f"annotated {n} axis(es) from {args.fit_record}")
        margins_dict = margins.to_dict()
        _print_margins(margins_dict)
    if args.json:
        d = rep.to_dict()
        if margins_dict is not None:
            d["margins"] = margins_dict
        print(json.dumps(d, ensure_ascii=False, indent=2))
    else:
        _print_compliance(rep.to_dict())
    if args.out:
        path = target_report.render(
            rep, Path(args.out),
            notes=f"作动器选型走查 · 架构 {params.steering_system.architecture}",
            margins=margins_dict,
        )
        print(f"\n  report: {path}")
    return 0 if rep.verdict == "compliant" else 2


def _print_margins(margins: dict) -> None:
    print("\n参数空间余量（判定翻转点，单轴扰动）：")
    print(f"  {'axis':<42}{'nominal':>10}{'flip↓':>12}{'flip↑':>12}"
          f"{'fitted':>10}{'inside':>8}  翻转条目")
    for a in margins["axes"]:
        def fmt(v):
            return "—" if v is None else f"{v:.4g}"
        inside = "—" if a["fitted_inside"] is None else ("是" if a["fitted_inside"] else "否")
        entries = "; ".join(
            (a["flip_low_entries"] or []) + (a["flip_high_entries"] or []))
        print(f"  {a['axis']:<42}{fmt(a['nominal']):>10}"
              f"{fmt(a['flip_low']):>12}{fmt(a['flip_high']):>12}"
              f"{fmt(a['fitted']):>10}{inside:>8}  {entries or '—'}")
    print(f"  evals {margins['n_evals']} · {margins['wall_s']:.0f} s · "
          f"nominal violated: {', '.join(margins['nominal_violated']) or '无'}")


def cmd_calibrate_residual(args: argparse.Namespace) -> int:
    """C3 · 3.1a: bench CSV + same-condition procedure -> residual report."""
    from sim4wis.calibration.residual import residual_panel

    channels = (
        [c.strip() for c in args.channels.split(",") if c.strip()]
        if args.channels else None
    )
    out = residual_panel(args.reference, args.procedure,
                         out_html=args.out, channels=channels)
    print(f"align: {out['shift_s']:+.3f} s on {out['align_channel']}")
    for c, r in sorted(out["residuals"].items()):
        print(f"  {c:<24} rms {r['rms']:.4g}  peak {r['peak']:.4g}"
              f"  rms/σ {r['rms_over_std']:.3f}  corr {r['corr']:.4f}")
    print(f"\nreport: {out['report']}")
    return 0


def cmd_calibrate_fit(args: argparse.Namespace) -> int:
    """C3 · 3.1b: fit steering parameters to the bench CSV (report only)."""
    import json as _json
    from pathlib import Path as _Path

    from sim4wis.calibration.identify import DEFAULT_MAX_EVALS, fit_reference

    params = (
        [p.strip() for p in args.params.split(",") if p.strip()]
        if args.params else None
    )
    channels = (
        [c.strip() for c in args.channels.split(",") if c.strip()]
        if args.channels else None
    )

    def progress(n: int, loss: float, x) -> None:
        print(f"  [{n:4d}] loss {loss:.4f}  " +
              "  ".join(f"{v:.4g}" for v in x), flush=True)

    out = fit_reference(
        args.reference, args.procedure,
        params=params, channels=channels,
        max_evals=args.max_evals or DEFAULT_MAX_EVALS,
        out_html=args.out,
        progress=progress if args.verbose else None,
    )
    print("parameters (default -> fitted):")
    for name, p in out["params"].items():
        rel = (p["fitted"] - p["default"]) / max(abs(p["default"]), 1e-12)
        print(f"  {name:<40} {p['default']:10.4g} -> {p['fitted']:<10.4g}"
              f" ({rel:+.1%}, bounds {p['bounds'][0]:g}–{p['bounds'][1]:g})")
    print(f"\nloss {out['loss']['before']:.4f} -> {out['loss']['after']:.4f}"
          f"  ({out['loss']['n_evals']} evals, {out['loss']['wall_s']:.1f} s)")
    print("residuals (rms/σ, default -> fitted):")
    for c in sorted(out["residuals_after"]):
        b = out["residuals_before"].get(c, {})
        a = out["residuals_after"][c]
        print(f"  {c:<24} {b.get('rms_over_std', float('nan')):.3f}"
              f" -> {a['rms_over_std']:.3f}   corr {a['corr']:.4f}")
    if args.json:
        record = dict(out)
        record.pop("report", None)
        _Path(args.json).write_text(
            _json.dumps(record, ensure_ascii=False, indent=1, default=str),
            encoding="utf-8")
        print(f"\nrecord: {args.json}")
    if "report" in out:
        print(f"report: {out['report']}")
    print("\nidentification only — the parameter library is not rewritten")
    return 0


def cmd_tune(args: argparse.Namespace) -> int:
    """The tuning workbench on the command line (direction 4)."""
    from sim4wis.steering.tracking import tuning

    plant = json.loads(args.plant_json) if args.plant_json else None
    conditions = json.loads(args.conditions_json) if args.conditions_json else None
    if args.grid:
        result = tuning.grid_search(args.controller, points=args.grid,
                                    plant_kwargs=plant, conditions=conditions)
    else:
        result = tuning.tune(args.controller, max_evals=args.evals,
                             plant_kwargs=plant, conditions=conditions)
    record = result.to_record()
    margins = None
    if args.margins:
        margins = [m.to_dict() for m in tuning.tune_margins(
            args.controller, result.tuned_params, plant_kwargs=plant,
            conditions=conditions, ratio=args.margins_ratio)]
        record["margins"] = margins
    if args.out:
        Path(args.out).write_text(json.dumps(record, ensure_ascii=False, indent=1),
                                  encoding="utf-8")
        print(f"record: {args.out}")
    if args.json:
        print(json.dumps(record, ensure_ascii=False, indent=1))
        return 0
    print(f"{args.controller}: cost {record['cost_analytic']:.4f} "
          f"-> {record['cost_tuned']:.4f} ({record['n_evals']} evals, "
          f"accepted={'yes' if record['accepted'] else 'no'})")
    print("  " + ", ".join(f"{k}={v:.4g}" for k, v in record["tuned_params"].items()))
    if margins:
        print("margins (cost bar = "
              f"{args.margins_ratio:.1f}x nominal):")
        for m in margins:
            lo = "—" if m["flip_low"] is None else f"{m['flip_low']:.4g}"
            hi = "—" if m["flip_high"] is None else f"{m['flip_high']:.4g}"
            print(f"  {m['name']:<16} nominal {m['nominal']:.4g}  "
                  f"flips low {lo} / high {hi}")
    return 0


def cmd_capabilities(args: argparse.Namespace) -> int:
    if _backend_up(args.backend) and not args.local:
        caps = _get(args.backend, "/api/study/capabilities")
    else:
        from sim4wis.controller.registry import available_strategies
        from sim4wis.steering import architecture as steering_arch
        from sim4wis.study.metrics import describe_metrics
        from sim4wis.study.runner import expected_channels
        from sim4wis.vehicle.model_registry import model_infos

        caps = {
            "steering_architectures": steering_arch.describe_all(),
            "models": [{"id": m.id, "label": m.label, "layer": m.layer,
                        "description": m.description} for m in model_infos()],
            "metrics": describe_metrics(),
            "strategies": available_strategies(),
            "channels": expected_channels(),
        }
    if args.json:
        print(json.dumps(caps, ensure_ascii=False, indent=1))
        return 0
    print("models:")
    print(_table(["id", "layer", "label", "description"],
                 [[m["id"], m["layer"], m["label"], m["description"]] for m in caps["models"]]))
    print("\nmetrics:")
    print(_table(["name", "unit", "requires", "solvable", "description"],
                 [[m["name"], m["unit"], m["requires"], m["solvable"], m["description"]]
                  for m in caps["metrics"]]))
    if caps.get("steering_architectures"):
        print("\nsteering architectures:")
        print(_table(["id", "front", "assist at", "rear", "ratio", "label"],
                     [[a["id"], a["front_path"], a["assist_at"], a["rear_axle"],
                       a["motor_gear_ratio"], a["label"]]
                      for a in caps["steering_architectures"]]))
    print("\nstrategies: " + ", ".join(caps["strategies"]))
    return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="sim4wis",
        description="4WIS Simulator — server and research CLI. "
                    "With no arguments, starts the backend.",
    )
    p.add_argument("--backend", default=DEFAULT_BACKEND,
                   help=f"backend base URL (default {DEFAULT_BACKEND})")
    p.add_argument("--local", action="store_true",
                   help="run in-process instead of calling a backend")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    sub = p.add_subparsers(dest="command")

    study = sub.add_parser("study", help="run and inspect studies")
    ssub = study.add_subparsers(dest="subcommand", required=True)

    run = ssub.add_parser("run", help="run a study spec (YAML or JSON)")
    run.add_argument("spec")
    run.add_argument("--dry-run", action="store_true",
                     help="validate and estimate cost without running anything")
    run.set_defaults(func=cmd_study_run)

    lst = ssub.add_parser("list", help="list stored studies")
    lst.set_defaults(func=cmd_study_list)

    show = ssub.add_parser("show", help="show one study's result")
    show.add_argument("study_id")
    show.set_defaults(func=cmd_study_show)

    tr = ssub.add_parser("trace", help="downsampled channels for one run")
    tr.add_argument("run_id")
    tr.add_argument("--channels", required=True, help="comma-separated channel names")
    tr.add_argument("--max-points", type=int, default=200)
    tr.set_defaults(func=cmd_study_trace)

    tg = sub.add_parser("targets", help="requirement sets and compliance")
    tsub = tg.add_subparsers(dest="subcommand", required=True)

    tl = tsub.add_parser("list", help="list available target sets")
    tl.set_defaults(func=cmd_targets_list)

    ts = tsub.add_parser("show", help="show one target set")
    ts.add_argument("ref", help="name or name@version")
    ts.set_defaults(func=cmd_targets_show)

    tc = tsub.add_parser("check", help="check a target set against an actuator sizing pass")
    tc.add_argument("ref", help="name or name@version")
    tc.add_argument("--vehicle", help="vehicle profile name (default: built-in LS9)")
    tc.add_argument("--out", help="write an HTML compliance report here")
    tc.add_argument("--margins", action="store_true",
                    help="also bisect parameter-space margins: how far each steering "
                         "scalar can move before the violated-must set changes (C5/C3c). "
                         "Costs one sizing pass per bisection step — minutes, not seconds")
    tc.add_argument("--axes", default=None,
                    help="comma-separated axes to stress (default: all; see --margins)")
    tc.add_argument("--bisect-steps", type=int, default=6,
                    help="bisection depth per direction (default: 6)")
    tc.add_argument("--fit-record", default=None,
                    help="a calibrate-fit JSON record: annotates each axis with its "
                         "identified value and whether the verdict survives it (C3c)")
    tc.set_defaults(func=cmd_targets_check)

    caps = sub.add_parser("capabilities", help="models, metrics, strategies")
    caps.set_defaults(func=cmd_capabilities)

    tun = sub.add_parser("tune", help="tuning workbench: corner-plant controller tuning")
    tun.add_argument("controller", help="pid_single / pid_cascade / lqr")
    tun.add_argument("--plant-json", default=None,
                     help='plant kwargs as JSON, e.g. '
                          '{"transmission_stiffness_nms_per_rad": 1200.0}')
    tun.add_argument("--conditions-json", default=None,
                     help="cost conditions as a JSON list of "
                          "{target, load_torque, t_end, weight}")
    tun.add_argument("--evals", type=int, default=120,
                     help="Nelder-Mead evaluation budget (default 120)")
    tun.add_argument("--grid", type=int, default=0,
                     help="grid-search mode with N points per axis (0 = Nelder-Mead)")
    tun.add_argument("--margins", action="store_true",
                     help="bisect parameter-space margins on the tuned gains")
    tun.add_argument("--margins-ratio", type=float, default=1.5,
                     help="cost bar for the margins bisection (default 1.5x)")
    tun.add_argument("--out", default=None,
                     help="write the tuning record (with margins) as JSON")
    tun.set_defaults(func=cmd_tune)

    cal = sub.add_parser("calibrate", help="calibration workbench (C3)")
    csub = cal.add_subparsers(dest="subcommand", required=True)

    cres = csub.add_parser(
        "residual",
        help="residual panel: bench CSV vs the same-condition sim (3.1a)",
    )
    cres.add_argument("--reference", required=True,
                      help="bench CSV with a t column plus standard channel columns")
    cres.add_argument("--procedure", required=True,
                      help="study-spec file pinning the same condition")
    cres.add_argument("--out", default=None, help="HTML report path (default: next to the CSV)")
    cres.add_argument("--channels", default=None,
                      help="comma-separated channel subset (default: all common)")
    cres.set_defaults(func=cmd_calibrate_residual)

    cfit = csub.add_parser(
        "fit",
        help="parameter identification: least-squares fit of steering scalars (3.1b)",
    )
    cfit.add_argument("--reference", required=True,
                      help="bench CSV with a t column plus standard channel columns")
    cfit.add_argument("--procedure", required=True,
                      help="study-spec file pinning the same condition")
    cfit.add_argument("--params", default=None,
                      help="comma-separated parameters to fit (default: all fittable; "
                           "see calibrate fit --help listing)")
    cfit.add_argument("--max-evals", type=int, default=None,
                      help="optimiser evaluation budget (default: 300)")
    cfit.add_argument("--out", default=None, help="HTML report path")
    cfit.add_argument("--json", default=None,
                      help="write the fit record (params, residuals, provenance) as JSON")
    cfit.add_argument("--channels", default=None,
                      help="comma-separated loss-channel subset (default: all common "
                           "except steer_hand_angle)")
    cfit.add_argument("-v", "--verbose", action="store_true",
                      help="print the optimiser trajectory")
    cfit.set_defaults(func=cmd_calibrate_fit)

    return p


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        from sim4wis.main import run

        run()
        return 0

    args = build_parser().parse_args(argv)
    if not getattr(args, "func", None):
        build_parser().print_help()
        return 1
    try:
        return int(args.func(args))
    except BackendUnavailableError as e:
        print(f"backend unreachable at {args.backend}: {e}", file=sys.stderr)
        return 3
    except (RuntimeError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":                           # pragma: no cover
    raise SystemExit(main())
