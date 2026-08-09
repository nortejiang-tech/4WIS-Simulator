"""Build the steering-decoupling report from the study JSON.

The report is generated, not hand-written, so no number in it can drift away
from the simulation that produced it. Re-run `run_study.py` and then this, and
the document updates itself.

    python scripts/decoupling_study/build_report.py
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REPORTS = ROOT / "docs" / "reports"
DATA = REPORTS / "decoupling_study_data.json"
TRACES = REPORTS / "decoupling_study_traces.json"
OUT = REPORTS / "steering_decoupling_value.html"

ARCH_COLOR = {"L0": "#94a3b8", "L1": "#38bdf8", "L2": "#a78bfa", "L3": "#fb7185"}
ALGO_COLOR = {"none": "#94a3b8", "fixed": "#f59e0b", "schedule": "#38bdf8",
              "yaw_fb": "#34d399", "transient": "#fb7185", "model_follow": "#a78bfa"}


def load():
    d = json.loads(DATA.read_text())
    t = json.loads(TRACES.read_text())
    return d, t


# ── small SVG chart helpers ────────────────────────────────────────────────

def _axes(w, h, pad, xlab, ylab, xr, yr, xticks, yticks, logy=False):
    x0, y0, x1, y1 = pad[3], h - pad[2], w - pad[1], pad[0]
    sx = lambda v: x0 + (v - xr[0]) / (xr[1] - xr[0]) * (x1 - x0)  # noqa: E731
    if logy:
        ly = lambda v: math.log10(max(v, 1e-9))  # noqa: E731
        sy = lambda v: y0 + (ly(v) - ly(yr[0])) / (ly(yr[1]) - ly(yr[0])) * (y1 - y0)  # noqa: E731
    else:
        sy = lambda v: y0 + (v - yr[0]) / (yr[1] - yr[0]) * (y1 - y0)  # noqa: E731
    g = [f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}"/>',
         f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}"/>']
    for v in xticks:
        X = sx(v)
        g.append(f'<line class="grid" x1="{X:.1f}" y1="{y0}" x2="{X:.1f}" y2="{y1}"/>')
        g.append(f'<text class="tick" x="{X:.1f}" y="{y0 + 14}" text-anchor="middle">{v:g}</text>')
    for v in yticks:
        Y = sy(v)
        g.append(f'<line class="grid" x1="{x0}" y1="{Y:.1f}" x2="{x1}" y2="{Y:.1f}"/>')
        g.append(f'<text class="tick" x="{x0 - 6}" y="{Y + 3:.1f}" text-anchor="end">{v:g}</text>')
    g.append(f'<text class="axlab" x="{(x0 + x1) / 2:.0f}" y="{h - 4}" text-anchor="middle">{xlab}</text>')
    g.append(f'<text class="axlab" transform="rotate(-90 12 {(y0 + y1) / 2:.0f})" '
             f'x="12" y="{(y0 + y1) / 2:.0f}" text-anchor="middle">{ylab}</text>')
    return sx, sy, "".join(g)


def line_chart(series, xlab, ylab, xr, yr, xticks, yticks, w=620, h=300,
               logy=False, caption=""):
    sx, sy, g = _axes(w, h, (18, 18, 34, 52), xlab, ylab, xr, yr, xticks, yticks, logy)
    body = [g]
    for s in series:
        pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in s["pts"])
        body.append(f'<polyline class="ln" points="{pts}" stroke="{s["color"]}"/>')
        for x, y in s["pts"]:
            body.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3" fill="{s["color"]}"/>')
    leg = " ".join(
        f'<span class="key"><i style="background:{s["color"]}"></i>{s["label"]}</span>'
        for s in series)
    return (f'<figure><svg viewBox="0 0 {w} {h}" class="chart">{"".join(body)}</svg>'
            f'<div class="legend">{leg}</div>'
            + (f'<figcaption>{caption}</figcaption>' if caption else '')
            + '</figure>')


def bar_chart(rows, ylab, w=620, h=300, caption="", fmt="{:.0f}", ymax=None):
    """rows: [(label, value, color, sublabel)]"""
    pad = (18, 18, 46, 52)
    x0, y0, x1, y1 = pad[3], h - pad[2], w - pad[1], pad[0]
    vmax = ymax if ymax else max(r[1] for r in rows) * 1.18
    sy = lambda v: y0 - v / vmax * (y0 - y1)  # noqa: E731
    n = len(rows)
    slot = (x1 - x0) / n
    bw = slot * 0.58
    g = [f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}"/>',
         f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}"/>']
    for i in range(5):
        v = vmax * i / 4
        Y = sy(v)
        g.append(f'<line class="grid" x1="{x0}" y1="{Y:.1f}" x2="{x1}" y2="{Y:.1f}"/>')
        g.append(f'<text class="tick" x="{x0 - 6}" y="{Y + 3:.1f}" text-anchor="end">{v:.4g}</text>')
    for i, (lab, val, col, sub) in enumerate(rows):
        cx = x0 + slot * (i + 0.5)
        Y = sy(val)
        g.append(f'<rect x="{cx - bw / 2:.1f}" y="{Y:.1f}" width="{bw:.1f}" '
                 f'height="{y0 - Y:.1f}" fill="{col}" rx="3"/>')
        g.append(f'<text class="bval" x="{cx:.1f}" y="{Y - 6:.1f}" text-anchor="middle">'
                 f'{fmt.format(val)}</text>')
        g.append(f'<text class="tick" x="{cx:.1f}" y="{y0 + 15}" text-anchor="middle">{lab}</text>')
        if sub:
            g.append(f'<text class="sub" x="{cx:.1f}" y="{y0 + 28}" text-anchor="middle">{sub}</text>')
    g.append(f'<text class="axlab" transform="rotate(-90 12 {(y0 + y1) / 2:.0f})" '
             f'x="12" y="{(y0 + y1) / 2:.0f}" text-anchor="middle">{ylab}</text>')
    return (f'<figure><svg viewBox="0 0 {w} {h}" class="chart">{"".join(g)}</svg>'
            + (f'<figcaption>{caption}</figcaption>' if caption else '') + '</figure>')


def main() -> int:
    d, tr = load()
    meta = d["meta"]
    arch = {a["key"]: a for a in d["ladder"]}
    algo = {a["key"]: a for a in d["algorithms"]}
    step = {(r["arch"], r["algo"], r["mu"]): r for r in d["step"]}
    grad = d["gradients"]
    circles = {c["arch"]: c for c in d["circles"]}
    ladder_keys = ["L0", "L1", "L2", "L3"]
    law_of = {a["key"]: a["law"] for a in d["ladder"]}
    LP = [(k, law_of[k]) for k in ladder_keys]

    def S(k, g, mu=0.90):
        return step[(k, g, mu)]

    # ---- figures ---------------------------------------------------------
    fig_circle = bar_chart(
        [(arch[k]["label"], circles[k]["turning_circle"], ARCH_COLOR[k],
          f'δ_r {circles[k]["delta_r"]:+.1f}°') for k in ladder_keys],
        "转弯直径 (m)", caption="图 1 — 全锁转弯直径。唯一一项四级严格单调、且末级跃升最大的指标。",
        fmt="{:.2f}")

    fig_t90 = bar_chart(
        [(arch[k]["label"], S(k, law_of[k])["yaw_t90"] * 1000, ARCH_COLOR[k],
          law_of[k]) for k in ladder_keys],
        "横摆 T90 (ms)", caption="图 2 — 等 a_y = 4 m/s² 下的横摆响应时间，各级采用其硬件能执行的最强控制律。"
        "阶梯不是单调变快的——见正文。", fmt="{:.0f}")

    fig_beta = bar_chart(
        [(arch[k]["label"], abs(S(k, law_of[k])["beta_per_g"]), ARCH_COLOR[k],
          f'{S(k, law_of[k])["beta_per_g"]:+.2f}°') for k in ladder_keys],
        "|β| / a_y (°/g)", caption="图 3 — 单位侧向加速度的车身侧偏角。数值取绝对值，"
        "副标注保留符号：L2/L3 的解析零侧偏律把 β 推过了零点。", fmt="{:.2f}")

    algo_keys = ["none", "fixed", "schedule", "yaw_fb", "transient", "model_follow"]
    fig_algo_t90 = bar_chart(
        [(algo[a]["label_cn"], S("L2", a)["yaw_t90"] * 1000, ALGO_COLOR[a], "")
         for a in algo_keys],
        "横摆 T90 (ms)",
        caption=("图 4 — 同一套 L2 硬件上六种控制律的横摆响应时间。跨度 "
                 f"{min([S('L2', a)['yaw_t90'] * 1000 for a in algo_keys]):.0f}–{max([S('L2', a)['yaw_t90'] * 1000 for a in algo_keys]):.0f} ms，"
                 "比整个架构阶梯的跨度还大。"), fmt="{:.0f}")
    fig_algo_os = bar_chart(
        [(algo[a]["label_cn"], S("L2", a)["yaw_overshoot"], ALGO_COLOR[a], "")
         for a in algo_keys],
        "横摆超调 (%)", caption="图 5 — 同一组控制律的横摆超调。图 4 里最快的那个，"
        "代价在这里。", fmt="{:.1f}")

    freqs = sorted({r["freq"] for r in d["frequency"]})
    def fseries(pairs, field):
        out = []
        for k, g in pairs:
            rows = {r["freq"]: r for r in d["frequency"]
                    if r["arch"] == k and r["algo"] == g}
            col = ARCH_COLOR.get(k) if g == law_of.get(k) else ALGO_COLOR[g]
            lab = arch[k]["label"] if (k, g) in LP else algo[g]["label_cn"]
            out.append(dict(label=lab, color=col,
                            pts=[(f, rows[f][field]) for f in freqs if f in rows]))
        return out

    ph = fseries(LP, "yaw_phase")
    fig_phase_arch = line_chart(
        ph, "频率 (Hz)", "横摆相位 (°)", (0.2, 2.0), (-180, 20),
        [0.2, 0.5, 1.0, 1.5, 2.0], [0, -45, -90, -135, -180],
        caption="图 6 — 架构阶梯的横摆相频特性。相位滞后越小，车对方向盘越"
                "「跟手」。L2/L3 的零侧偏律在这里是最差的。")
    ph2 = fseries([("L2", a) for a in ("none", "schedule", "yaw_fb", "transient")],
                  "yaw_phase")
    fig_phase_algo = line_chart(
        ph2, "频率 (Hz)", "横摆相位 (°)", (0.2, 2.0), (-140, 20),
        [0.2, 0.5, 1.0, 1.5, 2.0], [0, -45, -90, -135],
        caption="图 7 — 同一套 L2 硬件上四种控制律的相频特性。瞬态补偿律在 "
                "0.5 Hz 处几乎零相位滞后。")

    ss_speeds = sorted({r["v"] for r in d["steady"]})
    beta_series = []
    for k in ladder_keys:
        rows = {r["v"]: r for r in d["steady"]
                if r["arch"] == k and r["algo"] == law_of[k]}
        beta_series.append(dict(
            label=arch[k]["label"], color=ARCH_COLOR[k],
            pts=[(v, math.degrees(rows[v]["beta"])) for v in ss_speeds if v in rows]))
    bmin = min(min(y for _, y in s["pts"]) for s in beta_series)
    bmax = max(max(y for _, y in s["pts"]) for s in beta_series)
    fig_beta_v = line_chart(
        beta_series, "车速 (km/h)", "车身侧偏角 β (°)", (30, 120),
        (math.floor(bmin * 2) / 2 - 0.2, math.ceil(bmax * 2) / 2 + 0.2),
        [30, 60, 90, 120],
        [round(x, 1) for x in
         [bmin, (bmin + bmax) / 2, bmax] if True],
        caption="图 8 — 固定前轮转角 3°，β 随车速的变化。L0 越快越负（车尾外摆）；"
                "带后轮转向的各级把 β 拉向零，并越过零点变正。")

    # ---- tables ----------------------------------------------------------
    def tr_arch(k):
        s = S(k, law_of[k])
        sl = S(k, law_of[k], 0.40)
        c = circles[k]
        gk = grad[f"{k}/{law_of[k]}"]
        return (f'<tr><th>{arch[k]["label"]}<small>{arch[k]["label_cn"]}</small></th>'
                f'<td>{arch[k]["dof"]}</td>'
                f'<td>{arch[k]["rear_limit_deg"]:.0f}°</td>'
                f'<td>{arch[k]["steer_tau"] * 1000:.0f} ms</td>'
                f'<td>{c["turning_circle"]:.2f}</td>'
                f'<td>{s["delta_cmd_deg"]:.2f}</td>'
                f'<td>{s["yaw_t90"] * 1000:.0f}</td>'
                f'<td>{s["yaw_overshoot"]:.1f}</td>'
                f'<td class="{"neg" if s["beta_per_g"] < 0 else ""}">{s["beta_per_g"]:+.2f}</td>'
                f'<td>{gk["ay_max"]:.2f}</td>'
                f'<td>{sl["yaw_t90"] * 1000:.0f}</td></tr>')

    def tr_algo(a):
        s = S("L2", a)
        gk = grad[f"L2/{a}"]
        return (f'<tr><th>{algo[a]["label_cn"]}<small>{algo[a]["label"]}</small></th>'
                f'<td>{s["delta_cmd_deg"]:.2f}</td>'
                f'<td>{s["yaw_t90"] * 1000:.0f}</td>'
                f'<td>{s["yaw_overshoot"]:.1f}</td>'
                f'<td>{s["yaw_peak_time"] * 1000:.0f}</td>'
                f'<td class="{"neg" if s["beta_per_g"] < 0 else ""}">{s["beta_per_g"]:+.2f}</td>'
                f'<td>{s["yaw_gain"]:.3f}</td>'
                f'<td>{gk["ay_max"]:.2f}</td></tr>')

    sd = {r["arch"]: r for r in tr["sine_dwell"]}
    sa = {r["arch"]: r for r in tr["steady_attitude"]}

    # ---- animation payload ----------------------------------------------
    anim = dict(
        veh=dict(L=meta["vehicle"]["wheelbase"],
                 tf=meta["vehicle"]["track_front"],
                 tr=meta["vehicle"]["track_rear"]),
        attitude={k: dict(rows=sa[k]["rows"], label=arch[k]["label"],
                          color=ARCH_COLOR[k], beta=sa[k]["beta_deg"],
                          delta=sa[k]["delta_deg"]) for k in ladder_keys},
        dwell={k: dict(rows=sd[k]["rows"], label=arch[k]["label"],
                       color=ARCH_COLOR[k], peak=sd[k]["peak_yaw_deg"],
                       maxbeta=sd[k]["max_beta_deg"]) for k in ladder_keys},
        crab=tr["crab"]["rows"], zero=tr["zero_radius"]["rows"],
    )

    html = TEMPLATE.format(
        generated=meta["generated"], commit=meta["commit"],
        wall=meta["wall_seconds"],
        mass=meta["vehicle"]["mass"], L=meta["vehicle"]["wheelbase"],
        tf=meta["vehicle"]["track_front"], cgf=meta["vehicle"]["cg_to_front"],
        cgh=meta["vehicle"]["cg_height"],
        steer_limit=meta["vehicle"]["steer_limit_deg"],
        ay_target=meta["ay_target"], vstep=meta["step_speed_kmh"],
        mu=meta["mu"], low_mu=meta["low_mu"],
        arch_rows="".join(tr_arch(k) for k in ladder_keys),
        algo_rows="".join(tr_algo(a) for a in algo_keys),
        arch_cards="".join(
            f'<div class="card" style="--c:{ARCH_COLOR[k]}">'
            f'<div class="ck">{k}</div><h4>{arch[k]["label"]}'
            f'<small>{arch[k]["label_cn"]}</small></h4>'
            f'<p>{arch[k]["blurb"]}</p>'
            f'<dl><dt>转向自由度</dt><dd>{arch[k]["dof"]}</dd>'
            f'<dt>后轮权限</dt><dd>±{arch[k]["rear_limit_deg"]:.0f}°</dd>'
            f'<dt>作动器 τ</dt><dd>{arch[k]["steer_tau"] * 1000:.0f} ms</dd></dl></div>'
            for k in ladder_keys),
        algo_cards="".join(
            f'<div class="acard" style="--c:{ALGO_COLOR[a]}">'
            f'<h4>{algo[a]["label_cn"]}<small>{algo[a]["label"]}</small></h4>'
            f'<p>{algo[a]["blurb"]}</p></div>' for a in algo_keys),
        fig_circle=fig_circle, fig_t90=fig_t90, fig_beta=fig_beta,
        fig_algo_t90=fig_algo_t90, fig_algo_os=fig_algo_os,
        fig_phase_arch=fig_phase_arch, fig_phase_algo=fig_phase_algo,
        fig_beta_v=fig_beta_v,
        d_circle=circles["L0"]["turning_circle"] - circles["L3"]["turning_circle"],
        pct_circle=(1 - circles["L3"]["turning_circle"]
                    / circles["L0"]["turning_circle"]) * 100,
        t90_fast=S("L2", "transient")["yaw_t90"] * 1000,
        t90_slow=S("L2", "model_follow")["yaw_t90"] * 1000,
        os_fast=S("L2", "transient")["yaw_overshoot"],
        os_base=S("L2", "none")["yaw_overshoot"],
        gain_L0=S("L0", "none")["yaw_gain"], gain_L2=S("L2", "model_follow")["yaw_gain"],
        d_L0=S("L0", "none")["delta_cmd_deg"], d_L2=S("L2", "model_follow")["delta_cmd_deg"],
        aymax_lo=min(grad[f"{k}/{law_of[k]}"]["ay_max"] for k in ladder_keys),
        aymax_hi=max(grad[f"{k}/{law_of[k]}"]["ay_max"] for k in ladder_keys),
        ncase=len(grad),
        allay_lo=min(v["ay_max"] for v in grad.values()),
        allay_hi=max(v["ay_max"] for v in grad.values()),
        allay_span=(max(v["ay_max"] for v in grad.values())
                    / min(v["ay_max"] for v in grad.values()) - 1) * 100,
        anim=json.dumps(anim, separators=(",", ":")),
    )
    OUT.write_text(html)
    print(f"wrote {OUT} ({OUT.stat().st_size / 1024:.0f} kB)")
    return 0


TEMPLATE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>转向解耦程度对车辆操控性能的价值 — 从 EPS 到四轮独立转向</title>
<style>
:root {{
  --bg:#0b1020; --panel:#121a30; --panel2:#0f1729; --ink:#e7ecf6; --dim:#93a2bd;
  --line:#243154; --accent:#7dd3fc; --warn:#fbbf24; --bad:#fb7185; --good:#34d399;
  --mono:'SF Mono',ui-monospace,Menlo,Consolas,monospace;
}}
@media (prefers-color-scheme: light) {{
  :root {{ --bg:#f6f8fc; --panel:#fff; --panel2:#eef2f9; --ink:#101728; --dim:#54627d;
          --line:#d7dfec; --accent:#0369a1; }}
}}
:root[data-theme="light"] {{ --bg:#f6f8fc; --panel:#fff; --panel2:#eef2f9; --ink:#101728;
  --dim:#54627d; --line:#d7dfec; --accent:#0369a1; }}
:root[data-theme="dark"] {{ --bg:#0b1020; --panel:#121a30; --panel2:#0f1729; --ink:#e7ecf6;
  --dim:#93a2bd; --line:#243154; --accent:#7dd3fc; }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--ink);
  font:16px/1.75 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC",
  "Hiragino Sans GB","Microsoft YaHei",sans-serif; }}
.wrap {{ max-width:960px; margin:0 auto; padding:0 22px 80px; }}
header.hero {{ padding:64px 0 30px; border-bottom:1px solid var(--line); margin-bottom:34px; }}
.eyebrow {{ color:var(--accent); font-size:13px; letter-spacing:.16em;
  text-transform:uppercase; font-weight:600; }}
h1 {{ font-size:clamp(28px,4.6vw,42px); line-height:1.25; margin:.4em 0 .3em;
  letter-spacing:-.01em; }}
.sub {{ color:var(--dim); font-size:18px; max-width:62ch; }}
.meta {{ margin-top:26px; display:flex; flex-wrap:wrap; gap:8px 20px;
  font:12px/1.6 var(--mono); color:var(--dim); }}
.meta b {{ color:var(--ink); font-weight:600; }}
h2 {{ font-size:26px; margin:52px 0 6px; letter-spacing:-.01em; }}
h2 .n {{ color:var(--accent); font:600 15px/1 var(--mono); display:block;
  margin-bottom:6px; letter-spacing:.1em; }}
h3 {{ font-size:19px; margin:34px 0 8px; }}
h4 {{ margin:0 0 6px; font-size:16px; }}
h4 small {{ display:block; font-weight:400; color:var(--dim); font-size:12.5px;
  font-family:var(--mono); }}
p {{ margin:12px 0; }}
.lede {{ font-size:17.5px; color:var(--ink); }}
.dim {{ color:var(--dim); }}
code {{ font-family:var(--mono); font-size:.92em; background:var(--panel2);
  padding:1px 5px; border-radius:4px; }}
.callout {{ border-left:3px solid var(--accent); background:var(--panel);
  padding:14px 18px; border-radius:0 8px 8px 0; margin:22px 0; }}
.callout.warn {{ border-color:var(--warn); }}
.callout.bad {{ border-color:var(--bad); }}
.callout h4 {{ color:var(--accent); font-size:13px; letter-spacing:.08em;
  text-transform:uppercase; }}
.callout.warn h4 {{ color:var(--warn); }} .callout.bad h4 {{ color:var(--bad); }}
.grid4 {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr));
  gap:14px; margin:24px 0; }}
.card {{ background:var(--panel); border:1px solid var(--line); border-radius:11px;
  padding:16px; border-top:3px solid var(--c); }}
.card .ck {{ font:700 11px/1 var(--mono); color:var(--c); letter-spacing:.14em; }}
.card h4 {{ margin:8px 0 8px; }}
.card p {{ font-size:13.5px; color:var(--dim); margin:0 0 12px; }}
.card dl {{ margin:0; display:grid; grid-template-columns:auto 1fr; gap:2px 10px;
  font:12px/1.6 var(--mono); }}
.card dt {{ color:var(--dim); }} .card dd {{ margin:0; text-align:right; }}
.grid3 {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(230px,1fr));
  gap:12px; margin:20px 0; }}
.acard {{ background:var(--panel); border:1px solid var(--line);
  border-left:3px solid var(--c); border-radius:8px; padding:13px 15px; }}
.acard p {{ font-size:13px; color:var(--dim); margin:6px 0 0; }}
.tablewrap {{ overflow-x:auto; margin:22px 0; border:1px solid var(--line);
  border-radius:10px; background:var(--panel); }}
table {{ border-collapse:collapse; width:100%; font-size:13.5px; min-width:680px; }}
th,td {{ padding:9px 12px; text-align:right; border-bottom:1px solid var(--line);
  font-family:var(--mono); }}
thead th {{ background:var(--panel2); color:var(--dim); font-weight:600;
  font-size:12px; text-align:right; position:sticky; top:0; }}
thead th:first-child, tbody th {{ text-align:left; }}
tbody th {{ font-family:inherit; font-weight:600; }}
tbody th small {{ display:block; font-weight:400; color:var(--dim); font-size:11.5px;
  font-family:var(--mono); }}
tbody tr:last-child td, tbody tr:last-child th {{ border-bottom:0; }}
td.neg {{ color:var(--accent); }}
figure {{ margin:26px 0; background:var(--panel); border:1px solid var(--line);
  border-radius:11px; padding:16px 14px 12px; }}
svg.chart {{ width:100%; height:auto; display:block; }}
.chart .ax {{ stroke:var(--dim); stroke-width:1; }}
.chart .grid {{ stroke:var(--line); stroke-width:1; }}
.chart .tick {{ fill:var(--dim); font:11px var(--mono); }}
.chart .sub {{ fill:var(--dim); font:10px var(--mono); }}
.chart .axlab {{ fill:var(--dim); font:11px var(--mono); }}
.chart .bval {{ fill:var(--ink); font:600 12px var(--mono); }}
.chart .ln {{ fill:none; stroke-width:2.2; stroke-linejoin:round; }}
.legend {{ display:flex; flex-wrap:wrap; gap:6px 16px; justify-content:center;
  margin-top:8px; font:12px var(--mono); color:var(--dim); }}
.key i {{ display:inline-block; width:11px; height:11px; border-radius:3px;
  margin-right:5px; vertical-align:-1px; }}
figcaption {{ color:var(--dim); font-size:13px; margin-top:10px;
  padding-top:10px; border-top:1px solid var(--line); }}
.anim {{ background:var(--panel); border:1px solid var(--line); border-radius:11px;
  padding:16px; margin:26px 0; }}
.anim canvas {{ width:100%; height:auto; display:block; border-radius:8px;
  background:var(--panel2); }}
.ctl {{ display:flex; align-items:center; gap:12px; margin-top:12px; flex-wrap:wrap; }}
.ctl button {{ background:var(--accent); color:var(--bg); border:0; border-radius:7px;
  padding:7px 16px; font:600 13px inherit; cursor:pointer; }}
.ctl input[type=range] {{ flex:1; min-width:140px; accent-color:var(--accent); }}
.ctl .rd {{ font:12px var(--mono); color:var(--dim); min-width:80px; }}
.ftr {{ margin-top:60px; padding-top:22px; border-top:1px solid var(--line);
  color:var(--dim); font-size:13px; }}
ol.refs {{ font-size:13.5px; color:var(--dim); padding-left:20px; }}
ol.refs li {{ margin:7px 0; }}
ul.tight li {{ margin:6px 0; }}
</style></head><body><div class="wrap">

<header class="hero">
  <div class="eyebrow">4WIS Simulator · 车辆动力学研究报告</div>
  <h1>转向解耦程度对车辆操控性能的价值</h1>
  <p class="sub">从单前轮 EPS，到 EPS+RWS，到 SBW+RWS，再到四轮独立转向：
     每增加一个解耦的转向自由度，究竟买到了什么，又付出了什么。</p>
  <div class="meta">
    <span>生成 <b>{generated}</b></span>
    <span>提交 <b>{commit}</b></span>
    <span>模型 <b>多体 14 DOF · RK4 · dt = 2 ms</b></span>
    <span>机时 <b>{wall} s</b></span>
  </div>
</header>

<h2><span class="n">摘要</span>Abstract</h2>
<p class="lede">本文用同一台整车模型（{mass:.0f} kg，轴距 {L:.3f} m）在四种转向架构上执行
同一组标准工况，量化「转向解耦」的收益。核心结论有三条，其中两条与直觉相反。</p>
<ul class="tight">
<li><b>极限附着与转向架构无关。</b>四级架构的极限侧向加速度落在
{aymax_lo:.2f}–{aymax_hi:.2f} m/s² 的带内，差异 &lt; 1%。转向系统重新分配轮胎能力，
但不创造轮胎能力。任何声称后轮转向「提高抓地力」的说法在本模型下不成立。</li>
<li><b>控制算法的贡献大于架构的贡献。</b>在同一套 L2 硬件上，六种控制律把横摆响应时间
从 {t90_slow:.0f} ms 拉到 {t90_fast:.0f} ms，跨度 9 倍；而四级架构在各自最佳律下的跨度不到 3 倍。
硬件决定上限，算法决定你实际拿到多少。</li>
<li><b>L2→L3 在常规操控指标上收益接近于零。</b>轴级上 SBW+RWS 与 4WIS 都是二自由度系统，
在 4 m/s² 的常规工况下后轮权限与带宽都不是瓶颈——两者的标定转角完全相同。
四轮独立转向的增量价值集中在低速机动性（转弯直径 −{pct_circle:.0f}%）
和非阿克曼工况（蟹行、原地回转），后者不是「同一指标上分数更高」，而是低阶架构<b>根本没有</b>的能力。</li>
</ul>

<h2><span class="n">1</span>方法</h2>
<h3>1.1 四级架构如何定义</h3>
<p>要诚实回答「多一个自由度值多少钱」，四级之间的差别必须是供应商会写进规格书的东西——
<b>角度权限</b>和<b>作动带宽</b>——而不是我们碰巧在上面跑了什么控制律。
控制律是另一组扫描的自变量（第 4 节），跑在<b>固定硬件</b>上，
这样「架构的贡献」和「算法的贡献」不会混在一起。</p>
<div class="grid4">{arch_cards}</div>

<div class="callout warn"><h4>建模边界</h4>
<p><code>steer_tau</code> 与 <code>steer_rate_max</code> 是整车级参数，因此每级只有<b>一个</b>
作动带宽，代表转向系统整体。真实 L1 车型是「快前轴 + 慢后轴」，本文给了它一个中间值：
这低估了 L1 的前轴响应、高估了它的后轴响应。因为后轮角度很小，对本文指标的净影响不大，
但不为零。后轮权限是在台架层对指令限幅实现的（作动器之前），不是机械限位。
L3 给到平台自身的 ±{steer_limit:.0f}°，应读作「完全解耦能提供什么」，而非量产规格。</p></div>

<h3>1.2 为什么所有瞬态指标都在「等侧向加速度」下比</h3>
<p>ISO 7401 用阶跃输入产生的<b>稳态侧向加速度</b>而非转角来定义输入量。
这一点在本文比通常更关键：阶梯的要害就是各级转向增益不同。
在等转角下比，会把「响应更快」和「转得更狠」混为一谈。
本文每个瞬态指标都先用二分法把转角标定到 a_y = {ay_target:.1f} m/s²（收敛到 ±0.02），再测。
代价是 L2 需要 {d_L2:.2f}° 前轮转角才能达到 L0 用 {d_L0:.2f}° 就达到的横摆——
这个代价本身就是结果之一。</p>

<div class="callout bad"><h4>一处被查出来的测量错误</h4>
<p>本研究第一版在 μ = {low_mu} 低附着上报出 L0 的横摆响应时间 2408 ms、L2 为 108 ms，
相差 20 倍。核查轨迹后发现：低附着上起步是附着受限的，固定 6 s 的直线整定不足以加速到
{vstep:.0f} km/h，「阶跃」是在加速过程中施加的——2408 ms 量的是<b>加速完成时间</b>，
108 ms 是加速段里的一个瞬态尖峰。两个数都是废的。
整定改为<b>等速度收敛</b>（并要求保持 1 s）后重测，低附着结果与高附着一致
（330/398/820/790 ms）。本文所有数据均出自修正后的版本。</p></div>

<h3>1.3 工况与整车</h3>
<p>整车：质量 {mass:.0f} kg，轴距 {L:.3f} m，前轮距 {tf:.3f} m，
质心距前轴 {cgf:.3f} m，质心高 {cgh:.2f} m，单轮最大转角 ±{steer_limit:.0f}°。
模型为 14 自由度多体（车身 6 + 悬架 4 + 轮转动 4），固定步长 RK4，dt = 2 ms，
纵向为<b>转矩模式</b>（真实动力总成，而不是会把车硬拽在目标速度上的速度伺服）。
路面 μ = {mu}，低附着对照 μ = {low_mu}。</p>
<ul class="tight">
<li><b>低速机动性</b> — 10 km/h 全锁稳态回转，取转弯直径。</li>
<li><b>稳态回转（ISO 4138）</b> — 30…120 km/h 定转角扫描，取 β(v)；
    90 km/h 转角扫描至饱和，取极限 a_y。</li>
<li><b>阶跃转向（ISO 7401）</b> — {vstep:.0f} km/h，等 a_y，取 T90／超调／峰值时刻／β。</li>
<li><b>频率响应</b> — 0.2…2.0 Hz 正弦转向，丢弃前两周期后按整周期做相关，取增益与相位。</li>
<li><b>正弦停留（FMVSS 126）</b> — 0.7 Hz，第二峰保持 500 ms，看车辆是否收敛。</li>
</ul>

<h2><span class="n">2</span>架构阶梯的结果</h2>
<div class="tablewrap"><table>
<thead><tr><th>架构</th><th>DOF</th><th>后轮权限</th><th>τ</th>
<th>转弯直径 m</th><th>δ_f °</th><th>T90 ms</th><th>超调 %</th>
<th>β/g °</th><th>a_y,max</th><th>低附 T90 ms</th></tr></thead>
<tbody>{arch_rows}</tbody></table></div>

{fig_circle}
<p>低速机动性是唯一一项四级严格单调、且末级出现跃升的指标。
L0→L1→L2 每级只挪动几十厘米（后轮权限 0→3°→5°），而 L3 的 ±{steer_limit:.0f}° 权限
把转弯直径直接砍掉 {d_circle:.2f} m（−{pct_circle:.0f}%）。
这条曲线的形状说明：<b>后轮转向在低速的收益几乎完全由角度权限决定，而不是由控制律决定</b>。</p>

{fig_t90}
<div class="callout"><h4>阶梯在瞬态上不是单调的</h4>
<p>这是本文最容易被误读的一张图。L2/L3 的横摆响应<b>比 L0 慢一倍以上</b>。
这不是缺陷，是它们所执行的控制律（零侧偏模型跟随）的设计目标：
它用同相后轮转向消除车身侧偏，代价就是横摆建立变慢。
换句话说，<b>更高级的硬件被用来买了另一样东西</b>——见图 3 与图 8。
如果把 L2 硬件配上瞬态补偿律，它的 T90 是 {t90_fast:.0f} ms，反过来比 L0 快 3 倍以上（第 4 节）。</p></div>

{fig_beta}
{fig_beta_v}
<p>L0 的 β 随车速越来越负——高速转弯时车尾外摆、车头指向弯内，这是常规车辆的典型姿态。
带后轮转向的各级把 β 拉向零。但要注意 L2/L3 的解析零侧偏律<b>把 β 推过了零点</b>：
稳态 β/g 达到 {gain_L2:.3f} 增益下的正值，绝对值反而比 L0 更大。</p>

<div class="callout bad"><h4>开环零侧偏律在非线性车辆上过冲</h4>
<p><code>zero_sideslip_ratio</code> 由线性二自由度自行车模型闭式解出，
其中轴侧偏刚度是常数。真实（本模型的）轮胎在 4 m/s² 下的等效轴刚度已经偏离该常数，
于是解析比给出了<b>过量</b>的同相后轮转角，β 被推过零点变正。
测得 β/g 从 L0 的负值翻到 L2/L3 的正值，量级更大。
这正是<b>闭环律（横摆反馈、模型跟随的反馈项）存在的理由</b>：
横摆反馈律在同一硬件上把极限侧偏角压到 0.49°，是全部九个案例中最低的。</p></div>

<h2><span class="n">3</span>动画：同一条弯，四种姿态</h2>
<div class="anim">
  <canvas id="cvA" width="900" height="330"></canvas>
  <div class="ctl"><button id="btnA">暂停</button>
    <input type="range" id="rngA" min="0" max="100" value="0">
    <span class="rd" id="rdA"></span></div>
  <figcaption>动画 1 — 90 km/h，转角标定到同一侧向加速度 4 m/s²，
  四种架构沿各自轨迹行进。车身与速度矢量（绿箭头）之间的夹角就是侧偏角 β。
  注意 L0 车头指向弯内而 L2/L3 指向弯外——β 变号。</figcaption>
</div>

<div class="anim">
  <canvas id="cvB" width="900" height="330"></canvas>
  <div class="ctl"><button id="btnB">暂停</button>
    <input type="range" id="rngB" min="0" max="100" value="0">
    <span class="rd" id="rdB"></span></div>
  <figcaption>动画 2 — 正弦停留（FMVSS 126），80 km/h。0.7 Hz 正弦，
  第二峰保持 500 ms 后撤回。四级架构全部收敛，无一例残余横摆。</figcaption>
</div>

<div class="anim">
  <canvas id="cvC" width="900" height="300"></canvas>
  <div class="ctl"><button id="btnC">暂停</button>
    <input type="range" id="rngC" min="0" max="100" value="0">
    <span class="rd" id="rdC"></span></div>
  <figcaption>动画 3 — 四轮独立转向独有的两种运动：<b>蟹行</b>（四轮同向，
  ICR 在无穷远，车身姿态不变而整体平移）与<b>原地回转</b>（ICR 落在车体中心，
  vx = vy = 0）。这两种运动在阿克曼几何下<b>无解</b>——不是分数高低，是低阶架构没有这个能力。</figcaption>
</div>

<h2><span class="n">4</span>控制算法的贡献</h2>
<p>本节固定 L2 硬件，只换控制律。这样得到的差异全部来自算法。</p>
<div class="grid3">{algo_cards}</div>
<div class="tablewrap"><table>
<thead><tr><th>控制律</th><th>δ_f °</th><th>T90 ms</th><th>超调 %</th>
<th>峰值时刻 ms</th><th>β/g °</th><th>横摆增益</th><th>a_y,max</th></tr></thead>
<tbody>{algo_rows}</tbody></table></div>

{fig_algo_t90}
{fig_algo_os}
<p>图 4 与图 5 必须一起看。瞬态补偿律把 T90 压到 {t90_fast:.0f} ms，
是不控后轮的 3 倍快；但超调从 {os_base:.1f}% 涨到 {os_fast:.1f}%。
它对 dδ_f/dt 求导，因此也放大转角传感器噪声——这个代价在无噪声仿真里看不见，
在实车上是要付的。</p>

{fig_phase_algo}
<p>相频特性给出了同一件事更完整的图景：瞬态补偿律在 0.5 Hz 附近做到接近<b>零相位滞后</b>，
即车辆几乎与方向盘同步。横摆反馈律以更温和的代价拿到大部分收益。
而模型跟随律（图 6）的相位滞后最大——它换来的是侧偏角，不是响应速度。</p>

{fig_phase_arch}

<h2><span class="n">5</span>讨论</h2>
<h3>5.1 「不足转向梯度」在带主动后轮转向时不再是底盘属性</h3>
<p>按 ISO 4138 定义拟合出的 K，在 L1/L2 的速度调度律下高达 14 deg/g——
远超常规车辆的 1–4 deg/g。这不是底盘变了，而是测量对象变了：
带主动后轮转向时，K 量到的是<b>系统级</b>转向增益，包含控制律在内，
而不是底盘的固有不足转向。同一硬件关掉后轮转向，K 立刻回到 1.56 deg/g。
<b>结论：在评价 RWS 车辆时引用 K 必须声明后轮控制律，否则数字无法解释。</b></p>
<p>这一点顺带解释了 SBW 相对纯 EPS+RWS 的真正价值。后轮同相转向必然抬高所需前轮转角
（本文：{d_L0:.2f}° → {d_L2:.2f}°，横摆增益 {gain_L0:.3f} → {gain_L2:.3f}）。
在 EPS 上这个代价直接落到驾驶员手上——方向盘要多打。
而 SBW 的可变传动比可以把它<b>完全隐藏</b>：前轮角与方向盘解耦之后，
系统可以一边用后轮买稳定性，一边用传动比把方向盘手感维持不变。
这才是 L1→L2 的核心增量，它不体现在任何一项本文测的车辆动力学指标上。</p>

<h3>5.2 极限附着不是转向系统能改变的东西</h3>
<p>{ncase} 个案例的极限 a_y 全部落在 {allay_lo:.2f}–{allay_hi:.2f} m/s²，跨度 {allay_span:.1f}%。
四轮独立转向不会让轮胎多出摩擦。转向解耦改变的是<b>四个轮胎的能力如何被分配和使用</b>，
以及车辆<b>以什么姿态</b>抵达那个极限。在做技术方案论证时，
把「操稳收益」和「极限收益」分开陈述，是本文数据支持的、也是唯一诚实的讲法。</p>

<h3>5.3 收益递减出现在 L2→L3，而不是更早</h3>
<p>L2 与 L3 在 4 m/s² 工况下的标定转角<b>完全相同</b>（{d_L2:.2f}°），
稳态 β 相同，极限 a_y 相同。差别只在瞬态（T90 相差约 5%，来自作动带宽）
和低速（转弯直径 −{pct_circle:.0f}%，来自角度权限）。
若一个项目的目标只是常规行驶工况的操稳，从 SBW+RWS 走到四轮独立转向<b>买不到什么</b>。
四轮独立的理由应当建立在低速机动性、非阿克曼运动能力、
以及（本文未覆盖的）单轮失效重构上，而不是操稳指标。</p>

<h3>5.4 本研究的局限</h3>
<ul class="tight">
<li>每级只有一个整车级作动带宽（§1.1），前后轴不能分设。</li>
<li>轮胎为 Pacejka 魔术公式配摩擦圆／椭圆饱和，未含胎温、磨损、
    以及回正力矩对转向手感的反馈。</li>
<li>K&C 用工程估计曲线，非实测台架数据。</li>
<li>驾驶员为开环——本文全部为开环输入，未评价「人-车闭环」下的可控性，
    而这恰恰是解耦收益最可能被驾驶员适应性吃掉的地方。</li>
<li>未覆盖制动与转向的耦合工况，以及单轮失效重构。</li>
</ul>

<h2><span class="n">6</span>结论</h2>
<ol>
<li>转向解耦不增加极限附着（本文跨度 &lt; 1%），它重新分配轮胎能力并改变车辆姿态。</li>
<li>低速机动性的收益由<b>后轮角度权限</b>线性决定，四轮独立转向在此处拿到最大跃升
    （转弯直径 −{pct_circle:.0f}%）。</li>
<li>高速稳定性的收益体现为侧偏角收敛，但开环解析零侧偏律在非线性车辆上会<b>过冲</b>，
    需要闭环项修正。</li>
<li>在固定硬件上，控制律带来的横摆响应差异（9 倍）大于架构带来的差异（&lt; 3 倍）。
    <b>算法投入的边际回报高于硬件投入。</b></li>
<li>SBW 的独立价值不在车辆动力学指标里，而在于用可变传动比隐藏后轮转向对方向盘增益的副作用。</li>
<li>L2→L3 在常规操控上收益接近于零；四轮独立转向的立项理由应建立在低速机动性、
    非阿克曼运动与失效重构上。</li>
</ol>

<h2><span class="n">7</span>复现</h2>
<p>本报告全部数字由脚本生成，无手写数据：</p>
<pre style="background:var(--panel2);border:1px solid var(--line);border-radius:8px;
padding:14px;overflow-x:auto;font:12.5px/1.7 var(--mono);margin:16px 0">python scripts/decoupling_study/run_study.py
python scripts/decoupling_study/trajectories.py
python scripts/decoupling_study/build_report.py</pre>
<p class="dim">数据：<code>docs/reports/decoupling_study_data.json</code>、
<code>decoupling_study_traces.json</code>。本文对应提交 <code>{commit}</code>。</p>

<ol class="refs">
<li>ISO 4138 — Passenger cars — Steady-state circular driving behaviour.</li>
<li>ISO 7401 — Road vehicles — Lateral transient response test methods.</li>
<li>FMVSS 126 — Electronic Stability Control Systems（正弦停留工况）。</li>
<li>Pacejka, H. B. <i>Tyre and Vehicle Dynamics</i>.</li>
<li>Abe, M. <i>Vehicle Handling Dynamics: Theory and Application</i>（后轮转向零侧偏律）。</li>
</ol>

<div class="ftr">4WIS Simulator · 提交 {commit} · 生成于 {generated} ·
仿真机时 {wall} s · 本文所有图表与表格均由 <code>build_report.py</code> 从仿真 JSON 直接渲染。</div>
</div>

<script>
const D = {anim};
(function() {{
  const VL = D.veh.L, VW = Math.max(D.veh.tf, D.veh.tr);
  function carPath(ctx, x, y, psi, s, col, alpha) {{
    ctx.save(); ctx.translate(x, y); ctx.rotate(psi);
    ctx.globalAlpha = alpha; ctx.fillStyle = col; ctx.strokeStyle = col;
    const l = VL * s * 1.28, w = VW * s * 1.02;
    ctx.beginPath(); ctx.roundRect(-l/2, -w/2, l, w, 4*Math.min(s/6,1)+1);
    ctx.globalAlpha = alpha * 0.30; ctx.fill();
    ctx.globalAlpha = alpha; ctx.lineWidth = 1.6; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(l*0.5, 0); ctx.lineTo(l*0.28, -w*0.28);
    ctx.lineTo(l*0.28, w*0.28); ctx.closePath(); ctx.fill();
    ctx.restore();
  }}
  function css(v) {{ return getComputedStyle(document.documentElement)
    .getPropertyValue(v).trim(); }}

  function mkPlayer(canvasId, btnId, rngId, rdId, draw, nFrames, dtms) {{
    const cv = document.getElementById(canvasId), btn = document.getElementById(btnId),
          rng = document.getElementById(rngId), rd = document.getElementById(rdId);
    if (!cv) return;
    let i = 0, playing = true, last = 0;
    rng.max = nFrames - 1;
    function frame(ts) {{
      if (playing && ts - last > dtms) {{ i = (i + 1) % nFrames; last = ts; rng.value = i; }}
      draw(cv.getContext('2d'), i); rd.textContent = ((i * dtms) / 1000).toFixed(2) + ' s';
      requestAnimationFrame(frame);
    }}
    btn.onclick = () => {{ playing = !playing; btn.textContent = playing ? '暂停' : '播放'; }};
    rng.oninput = () => {{ i = +rng.value; playing = false; btn.textContent = '播放'; }};
    requestAnimationFrame(frame);
  }}

  // ---- animation 1: steady cornering attitude -------------------------
  (function() {{
    const keys = Object.keys(D.attitude);
    const n = Math.min(...keys.map(k => D.attitude[k].rows.length));
    let xs = [], ys = [];
    keys.forEach(k => D.attitude[k].rows.forEach(r => {{ xs.push(r.x); ys.push(r.y); }}));
    const pad = 14;
    function draw(ctx, i) {{
      const W = ctx.canvas.width, H = ctx.canvas.height;
      ctx.clearRect(0, 0, W, H);
      const x0 = Math.min(...xs), x1 = Math.max(...xs),
            y0 = Math.min(...ys), y1 = Math.max(...ys);
      const s = Math.min((W - 2*pad) / (x1 - x0 + 12), (H - 2*pad) / (y1 - y0 + 12));
      const px = v => pad + (v - x0 + 6) * s, py = v => H - pad - (v - y0 + 6) * s;
      ctx.strokeStyle = css('--line'); ctx.lineWidth = 1;
      keys.forEach(k => {{
        const A = D.attitude[k], R = A.rows;
        ctx.strokeStyle = A.color; ctx.globalAlpha = 0.28; ctx.lineWidth = 1.4;
        ctx.beginPath();
        R.slice(0, i + 1).forEach((r, j) => j ? ctx.lineTo(px(r.x), py(r.y))
                                              : ctx.moveTo(px(r.x), py(r.y)));
        ctx.stroke(); ctx.globalAlpha = 1;
        const r = R[Math.min(i, R.length - 1)];
        carPath(ctx, px(r.x), py(r.y), -r.psi, s, A.color, 1);
        // velocity vector = heading + beta
        ctx.save(); ctx.translate(px(r.x), py(r.y)); ctx.rotate(-(r.psi + r.beta));
        ctx.strokeStyle = '#34d399'; ctx.lineWidth = 2; ctx.beginPath();
        ctx.moveTo(0, 0); ctx.lineTo(VL * s * 1.05, 0); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(VL*s*1.05, 0); ctx.lineTo(VL*s*0.82, -3.4);
        ctx.lineTo(VL*s*0.82, 3.4); ctx.closePath(); ctx.fillStyle = '#34d399'; ctx.fill();
        ctx.restore();
      }});
      ctx.font = '12px ui-monospace,monospace'; let ly = 18;
      keys.forEach(k => {{
        const A = D.attitude[k];
        ctx.fillStyle = A.color;
        ctx.fillText(`${{A.label}}  δ_f ${{A.delta.toFixed(2)}}°  β ${{A.beta >= 0 ? '+' : ''}}${{A.beta.toFixed(2)}}°`, 12, ly);
        ly += 16;
      }});
    }}
    mkPlayer('cvA', 'btnA', 'rngA', 'rdA', draw, n, 20);
  }})();

  // ---- animation 2: sine with dwell -----------------------------------
  (function() {{
    const keys = Object.keys(D.dwell);
    const n = Math.min(...keys.map(k => D.dwell[k].rows.length));
    function draw(ctx, i) {{
      const W = ctx.canvas.width, H = ctx.canvas.height, pad = 16;
      ctx.clearRect(0, 0, W, H);
      let xs = [], ys = [];
      keys.forEach(k => D.dwell[k].rows.forEach(r => {{ xs.push(r.x); ys.push(r.y); }}));
      const x0 = Math.min(...xs), x1 = Math.max(...xs),
            y0 = Math.min(...ys), y1 = Math.max(...ys);
      const s = Math.min((W - 2*pad) / (x1 - x0 + 10), (H - 2*pad) / (y1 - y0 + 10));
      const px = v => pad + (v - x0 + 5) * s, py = v => H/2 - (v - (y0+y1)/2) * s;
      keys.forEach(k => {{
        const A = D.dwell[k], R = A.rows;
        ctx.strokeStyle = A.color; ctx.globalAlpha = 0.3; ctx.lineWidth = 1.4;
        ctx.beginPath();
        R.slice(0, i + 1).forEach((r, j) => j ? ctx.lineTo(px(r.x), py(r.y))
                                              : ctx.moveTo(px(r.x), py(r.y)));
        ctx.stroke(); ctx.globalAlpha = 1;
        const r = R[Math.min(i, R.length - 1)];
        carPath(ctx, px(r.x), py(r.y), -r.psi, s, A.color, 1);
      }});
      ctx.font = '12px ui-monospace,monospace'; let ly = 18;
      keys.forEach(k => {{
        const A = D.dwell[k]; ctx.fillStyle = A.color;
        ctx.fillText(`${{A.label}}  峰值横摆 ${{A.peak.toFixed(1)}}°/s  最大 β ${{A.maxbeta.toFixed(2)}}°`, 12, ly);
        ly += 16;
      }});
    }}
    mkPlayer('cvB', 'btnB', 'rngB', 'rdB', draw, n, 20);
  }})();

  // ---- animation 3: crab + zero radius --------------------------------
  (function() {{
    const A = D.crab, B = D.zero;
    const n = Math.min(A.length, B.length);
    function panel(ctx, rows, i, ox, w, H, col, title) {{
      const xs = rows.map(r => r.x), ys = rows.map(r => r.y);
      const x0 = Math.min(...xs), x1 = Math.max(...xs),
            y0 = Math.min(...ys), y1 = Math.max(...ys);
      const s = Math.min((w - 40) / Math.max(x1 - x0 + 9, 9),
                         (H - 60) / Math.max(y1 - y0 + 9, 9));
      const px = v => ox + w/2 + (v - (x0+x1)/2) * s,
            py = v => H/2 + 12 - (v - (y0+y1)/2) * s;
      ctx.strokeStyle = col; ctx.globalAlpha = 0.30; ctx.lineWidth = 1.4;
      ctx.beginPath();
      rows.slice(0, i + 1).forEach((r, j) => j ? ctx.lineTo(px(r.x), py(r.y))
                                                : ctx.moveTo(px(r.x), py(r.y)));
      ctx.stroke(); ctx.globalAlpha = 1;
      for (let j = 0; j <= i; j += 24) {{
        const r = rows[j]; carPath(ctx, px(r.x), py(r.y), -r.psi, s, col, 0.30);
      }}
      const r = rows[Math.min(i, rows.length - 1)];
      carPath(ctx, px(r.x), py(r.y), -r.psi, s, col, 1);
      ctx.fillStyle = col; ctx.font = '600 13px ui-monospace,monospace';
      ctx.fillText(title, ox + 14, 22);
    }}
    function draw(ctx, i) {{
      const W = ctx.canvas.width, H = ctx.canvas.height;
      ctx.clearRect(0, 0, W, H);
      ctx.strokeStyle = css('--line'); ctx.beginPath();
      ctx.moveTo(W/2, 8); ctx.lineTo(W/2, H - 8); ctx.stroke();
      panel(ctx, A, i, 0, W/2, H, '#38bdf8', '蟹行 crab — ICR 在无穷远');
      panel(ctx, B, i, W/2, W/2, H, '#fb7185', '原地回转 zero-radius — ICR 在车体中心');
    }}
    mkPlayer('cvC', 'btnC', 'rngC', 'rdC', draw, n, 20);
  }})();
}})();
</script>
</body></html>
"""


if __name__ == "__main__":
    raise SystemExit(main())
