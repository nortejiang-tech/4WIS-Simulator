"""Discoverable full-data and graphical outputs shared by all run sources.

Full exports stream the persisted rows without decimation. SVG previews use a
bounded subset and say so explicitly; neither is a new simulation or data fit.
"""
from __future__ import annotations

import csv
import hashlib
import html
import json
import math
from pathlib import Path
from typing import Any

from sim4wis.experiment import store


def run_file(run_id: str, name: str) -> Path:
    store.load_run_meta(run_id)  # validates the id and existence
    return store.ensure_runs_root() / store._safe_name(run_id) / name


def channel_unit(name: str) -> str:
    if name == "t": return "s"
    if name in {"pose_x", "pose_y", "pose_z"} or name.startswith(("icr_", "susp_defl_")): return "m"
    if name in {"vx", "vy"}: return "m/s"
    if name in {"ax", "ay"}: return "m/s^2"
    if name in {"yaw_rate", "steer_motor_speed"} or name.startswith(("omega_", "wheel_speed_cmd_")): return "rad/s"
    if name in {"pose_psi", "roll", "pitch", "steer_hand_angle", "steer_angle_deviation"} or name.startswith(("delta_", "deltacmd_", "slip_alpha_", "grip_alpha_peak_", "steer_corner_deviation_", "linkage_arm_tie_angle_", "linkage_tie_rack_angle_")): return "rad"
    if name.startswith(("fz_", "rack_force_", "tire_fx_", "tire_fy_", "grip_capacity_", "grip_margin_")): return "N"
    if name in {"steer_hand_torque", "steer_torque_sensor", "steer_assist_torque", "steer_motor_torque"} or name.startswith(("torque_steer_", "motor_torque_", "drive_torque_cmd_")): return "N*m"
    if name in {"steer_plant_active", "mu_avg", "strategy_index"} or name.startswith(("driver_", "slip_kappa_", "grip_", "wheel_locked_", "brake_cmd_", "linkage_efficiency_")): return "1"
    return "unspecified"  # do not guess units for plugin/legacy channels


def manifest(run_id: str) -> dict[str, Any]:
    meta = store.load_run_meta(run_id)
    path = run_file(run_id, "data.csv")
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""): digest.update(chunk)
    with path.open(newline="") as f: names = next(csv.reader(f))
    base = f"/api/runs/{run_id}"
    return {"contract": "4wis.artifacts.v1", "run_id": run_id,
            "source": meta.get("source", "experiment"), "samples": meta.get("n_samples"),
            "channels": [{"name": n, "unit": channel_unit(n)} for n in names],
            "sampling": meta.get("sampling", f"recorded at requested {meta.get('record_hz', 'unknown')} Hz"),
            "full_data_decimated": False, "nonfinite_encoding": "empty CSV field / JSON null",
            "complete": meta.get("complete", meta.get("execution_status") != "failed"),
            "execution_status": meta.get("execution_status", "recorded"),
            "dropped_samples": (meta.get("recording") or {}).get("dropped_samples", 0),
            "csv_sha256": digest.hexdigest(), "csv_bytes": path.stat().st_size,
            "validity": "Internal verification; no independent vehicle correlation. Model validity remains in metadata.",
            "outputs": {"csv": f"{base}/export.csv", "json": f"{base}/export.json",
                        "bundle": f"{base}/bundle.zip", "metadata": base,
                        "trajectory": f"{base}/plot.svg?kind=trajectory",
                        "chart": f"{base}/plot.svg?kind=chart&channel=yaw_rate",
                        "analysis": f"/?run={run_id}"}}


def json_rows(run_id: str):
    yield '{"contract":"4wis.data.v1","manifest":'
    yield json.dumps(manifest(run_id), ensure_ascii=False, allow_nan=False)
    yield ',"rows":['
    with run_file(run_id, "data.csv").open(newline="") as f:
        first = True
        for row in csv.DictReader(f):
            out = {}
            for key, value in row.items():
                try:
                    number = float(value)
                    out[key] = number if math.isfinite(number) else None
                except (TypeError, ValueError): out[key] = None
            if not first: yield ","
            yield json.dumps(out, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            first = False
    yield "]}"


def plot_svg(run_id: str, kind: str = "chart", channel: str = "yaw_rate") -> str:
    meta = store.load_run_meta(run_id)
    stride = max(1, math.ceil(meta.get("n_samples", 0) / 2000))
    xname, yname = ("pose_x", "pose_y") if kind == "trajectory" else ("t", channel)
    data = store.load_run_channels(run_id, list({xname, yname} - {"t"}), stride)
    if yname not in data or xname not in data:
        raise ValueError(f"unknown or unavailable plot channel: {yname}")
    pairs = [(float(x), float(y)) for x, y in zip(data[xname], data[yname])
             if x is not None and y is not None and math.isfinite(x) and math.isfinite(y)]
    if not pairs: raise ValueError("no finite samples to plot")
    xs, ys = zip(*pairs)
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    dx, dy = max(xmax - xmin, 1e-6), max(ymax - ymin, 1e-6)
    if kind == "trajectory":
        # Preserve metric aspect ratio; never stretch a circle into an ellipse.
        scale = min(670 / dx, 270 / dy)
        dx, dy = 670 / scale, 270 / scale
        xmin, ymin = (xmin + xmax - dx) / 2, (ymin + ymax - dy) / 2
    else:
        pad = max(dy * .06, .01)
        ymin -= pad; dy += 2 * pad
    points = " ".join(f"{70 + (x-xmin)/dx*670:.3f},{320-(y-ymin)/dy*270:.3f}" for x,y in pairs)
    title = "Trajectory (equal scale)" if kind == "trajectory" else f"{yname} [{channel_unit(yname)}]"
    labels = "".join(f'<text x="62" y="{320-i*54}" text-anchor="end">{ymin+dy*i/5:.4g}</text>'
                     f'<line x1="70" x2="740" y1="{320-i*54}" y2="{320-i*54}" stroke="#e2e8f0"/>' for i in range(6))
    xlabels = "".join(f'<text x="{70+i*134}" y="340" text-anchor="middle">{xmin+dx*i/5:.4g}</text>' for i in range(6))
    return f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 400" role="img" aria-label="{html.escape(title)}">
<rect width="800" height="400" fill="#ffffff"/><g font-family="sans-serif" font-size="12" fill="#334155">
<text x="70" y="28" font-size="18">{html.escape(title)}</text>{labels}{xlabels}
<polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="2"/>
<text x="400" y="363" text-anchor="middle">{html.escape(xname)} [{channel_unit(xname)}]</text>
<text x="70" y="385">Preview: every {stride} recorded sample(s); full data available separately. {html.escape(run_id)}</text>
</g></svg>'''
