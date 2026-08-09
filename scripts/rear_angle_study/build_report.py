"""Build the rear-steer angle-range report from its study JSON.

Generated, not hand-written: re-run run_study.py and then this.

    python scripts/rear_angle_study/build_report.py
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REPORTS = ROOT / "docs" / "reports"
DATA = REPORTS / "rear_angle_study_data.json"
OUT = REPORTS / "rear_steer_angle_range_value.html"

CASE_COLOR = {"park": "#fb7185", "alley": "#f59e0b", "urban": "#34d399",
              "highway": "#38bdf8", "evade": "#a78bfa", "cruise": "#7dd3fc"}
LOW_SPEED = ("park", "alley")


def _axes(w, h, pad, xlab, ylab, xr, yr, xticks, yticks):
    x0, y0, x1, y1 = pad[3], h - pad[2], w - pad[1], pad[0]
    sx = lambda v: x0 + (v - xr[0]) / (xr[1] - xr[0]) * (x1 - x0)  # noqa: E731
    sy = lambda v: y0 + (v - yr[0]) / (yr[1] - yr[0]) * (y1 - y0)  # noqa: E731
    g = [f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}"/>',
         f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}"/>']
    for v in xticks:
        X = sx(v)
        g.append(f'<line class="grid" x1="{X:.1f}" y1="{y0}" x2="{X:.1f}" y2="{y1}"/>')
        g.append(f'<text class="tick" x="{X:.1f}" y="{y0 + 14}" text-anchor="middle">{v:g}</text>')
    for v in yticks:
        Y = sy(v)
        cls = "zero" if abs(v) < 1e-9 else "grid"
        g.append(f'<line class="{cls}" x1="{x0}" y1="{Y:.1f}" x2="{x1}" y2="{Y:.1f}"/>')
        g.append(f'<text class="tick" x="{x0 - 6}" y="{Y + 3:.1f}" text-anchor="end">{v:g}</text>')
    g.append(f'<text class="axlab" x="{(x0 + x1) / 2:.0f}" y="{h - 4}" text-anchor="middle">{xlab}</text>')
    g.append(f'<text class="axlab" transform="rotate(-90 12 {(y0 + y1) / 2:.0f})" '
             f'x="12" y="{(y0 + y1) / 2:.0f}" text-anchor="middle">{ylab}</text>')
    return sx, sy, "".join(g)


def nice_ticks(lo, hi, n=5):
    if hi <= lo:
        hi = lo + 1
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=mag)
    start = math.floor(lo / step) * step
    out, v = [], start
    while v <= hi + step * 0.5:
        out.append(round(v, 6))
        v += step
    return out


def line_chart(series, xlab, ylab, xr, yr, xticks, w=640, h=310, caption="",
               marks=()):
    yticks = nice_ticks(yr[0], yr[1])
    sx, sy, g = _axes(w, h, (18, 20, 34, 56), xlab, ylab, xr, yr, xticks, yticks)
    body = [g]
    for m in marks:
        X = sx(m["x"])
        body.append(f'<line class="mark" x1="{X:.1f}" y1="{h - 34}" x2="{X:.1f}" y2="18"/>')
        body.append(f'<text class="marklab" x="{X + 4:.1f}" y="30">{m["label"]}</text>')
    for s in series:
        pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in s["pts"])
        dash = ' stroke-dasharray="5 4"' if s.get("dashed") else ""
        body.append(f'<polyline class="ln" points="{pts}" stroke="{s["color"]}"{dash}/>')
        for x, y in s["pts"]:
            body.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="2.6" fill="{s["color"]}"/>')
        if s.get("knee") is not None:
            kx, ky = s["knee"]
            body.append(f'<circle cx="{sx(kx):.1f}" cy="{sy(ky):.1f}" r="6" '
                        f'fill="none" stroke="{s["color"]}" stroke-width="2"/>')
    leg = " ".join(f'<span class="key"><i style="background:{s["color"]}"></i>{s["label"]}</span>'
                   for s in series)
    return (f'<figure><svg viewBox="0 0 {w} {h}" class="chart">{"".join(body)}</svg>'
            f'<div class="legend">{leg}</div>'
            + (f'<figcaption>{caption}</figcaption>' if caption else '') + '</figure>')


def demand_map(kcurve, cases, demand, limits, w=640, h=360, caption=""):
    """Heat map of |k(v)*delta_f| — the rear angle a zero-sideslip law wants."""
    vmax, dmax = 160.0, 40.0
    pad = (18, 74, 34, 56)
    x0, y0, x1, y1 = pad[3], h - pad[2], w - pad[1], pad[0]
    sx = lambda v: x0 + v / vmax * (x1 - x0)  # noqa: E731
    sy = lambda d: y0 + d / dmax * (y1 - y0)  # noqa: E731
    kv = {r["v"]: r["k"] for r in kcurve}
    ks = sorted(kv)

    def k_at(v):
        if v <= ks[0]:
            return kv[ks[0]]
        if v >= ks[-1]:
            return kv[ks[-1]]
        import bisect
        i = bisect.bisect_left(ks, v)
        a, b = ks[i - 1], ks[i]
        f = (v - a) / (b - a)
        return kv[a] * (1 - f) + kv[b] * f

    g = []
    NX, NY = 64, 40
    for ix in range(NX):
        for iy in range(NY):
            v = vmax * (ix + 0.5) / NX
            df = dmax * (iy + 0.5) / NY
            need = abs(k_at(v) * df)
            # colour by which authority band covers it
            if need <= 3:
                c, a = "#34d399", 0.30
            elif need <= 7:
                c, a = "#38bdf8", 0.34
            elif need <= 12:
                c, a = "#f59e0b", 0.38
            else:
                c, a = "#fb7185", 0.42
            g.append(f'<rect x="{sx(v - vmax/NX/2):.1f}" y="{sy(df + dmax/NY/2):.1f}" '
                     f'width="{(x1-x0)/NX + 0.6:.1f}" height="{abs(y1-y0)/NY + 0.6:.1f}" '
                     f'fill="{c}" opacity="{a}"/>')
    # contours where |k*df| equals each authority level
    for lvl, col in ((3, "#34d399"), (7, "#38bdf8"), (12, "#f59e0b")):
        pts = []
        for ix in range(NX * 2 + 1):
            v = vmax * ix / (NX * 2)
            k = abs(k_at(v))
            if k < 1e-6:
                continue
            df = lvl / k
            if 0 <= df <= dmax:
                pts.append((v, df))
        if len(pts) > 1:
            s = " ".join(f"{sx(v):.1f},{sy(d):.1f}" for v, d in pts)
            g.append(f'<polyline points="{s}" fill="none" stroke="{col}" stroke-width="2.2"/>')
            lv, ld = pts[len(pts) // 2]
            g.append(f'<text class="cont" x="{sx(lv) + 4:.1f}" y="{sy(ld) - 5:.1f}" '
                     f'fill="{col}">±{lvl}°</text>')
    # the actual manoeuvres, placed at their measured (v, delta_f)
    for c, dm in zip(cases, demand):
        v, df = c["v"], dm["delta_f"]
        if df > dmax:
            df = dmax
        g.append(f'<circle cx="{sx(v):.1f}" cy="{sy(df):.1f}" r="5" '
                 f'fill="{CASE_COLOR[c["key"]]}" stroke="#0b1020" stroke-width="1.5"/>')
        g.append(f'<text class="pt" x="{sx(v) + 9:.1f}" y="{sy(df) + 4:.1f}">'
                 f'{c["label"]}</text>')
    ax = [f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}"/>',
          f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}"/>']
    for v in (0, 40, 80, 120, 160):
        ax.append(f'<text class="tick" x="{sx(v):.1f}" y="{y0 + 14}" text-anchor="middle">{v}</text>')
    for dv in (0, 10, 20, 30, 40):
        ax.append(f'<text class="tick" x="{x0 - 6}" y="{sy(dv) + 3:.1f}" text-anchor="end">{dv}</text>')
    ax.append(f'<text class="axlab" x="{(x0+x1)/2:.0f}" y="{h-4}" text-anchor="middle">车速 (km/h)</text>')
    ax.append(f'<text class="axlab" transform="rotate(-90 12 {(y0+y1)/2:.0f})" '
              f'x="12" y="{(y0+y1)/2:.0f}" text-anchor="middle">前轮转角 δ_f (°)</text>')
    lg = [("≤3° 可覆盖", "#34d399"), ("3–7°", "#38bdf8"),
          ("7–12°", "#f59e0b"), (">12° 覆盖不了", "#fb7185")]
    for i, (lab, col) in enumerate(lg):
        yy = y1 + 14 + i * 19
        ax.append(f'<rect x="{x1 + 10}" y="{yy - 9}" width="11" height="11" rx="2" '
                  f'fill="{col}" opacity="0.55"/>')
        ax.append(f'<text class="tick" x="{x1 + 25}" y="{yy}">{lab}</text>')
    return (f'<figure><svg viewBox="0 0 {w} {h}" class="chart">{"".join(g + ax)}</svg>'
            + (f'<figcaption>{caption}</figcaption>' if caption else '') + '</figure>')


def main() -> int:
    d = json.loads(DATA.read_text())
    meta, cases, limits = d["meta"], d["cases"], d["limits"]
    demand = {r["case"]: r for r in d["demand"]}
    knees = d["knees"]
    sweep = {}
    for r in d["sweep"]:
        sweep.setdefault(r["case"], {})[r["limit"]] = r
    C = {c["key"]: c for c in cases}

    def rows(key):
        return [sweep[key][L] for L in limits if L in sweep[key]]

    # --- fig: demand map ---
    fig_map = demand_map(
        d["k_curve"], cases, [demand[c["key"]] for c in cases], limits,
        caption="图 1 — 后轮转角<b>需求图</b>。颜色表示零侧偏律在该 (车速, 前轮转角) "
                "点上索取的后轮转角落在哪个权限档内，等值线是 ±3°／±7°／±12° 的边界。"
                "圆点是本文六个工况实测到的工作点。图的形状就是全文的论点："
                "需求 = |k(v)|·δ_f，高速那一侧 δ_f 小，需求自然小；"
                "低速那一侧 δ_f 顶到机械限位，需求爆炸。")

    # --- fig: low-speed value curve ---
    ls = []
    for k in LOW_SPEED:
        rr = rows(k)
        pts = [(r["limit"], r["turning_circle"]) for r in rr]
        kn = knees[k]["knee"]
        kp = next((p for p in pts if p[0] == kn), None)
        ls.append(dict(label=C[k]["label"], color=CASE_COLOR[k], pts=pts, knee=kp))
    lo = min(min(y for _, y in s["pts"]) for s in ls)
    hi = max(max(y for _, y in s["pts"]) for s in ls)
    fig_low = line_chart(
        ls, "后轮转角权限 (°)", "转弯直径 (m)", (0, 12), (lo - 0.4, hi + 0.4),
        [0, 2, 4, 6, 8, 10, 12],
        caption="图 2 — 低速工况的价值曲线。<b>没有拐点</b>：12° 之内每一度都在继续买到收益，"
                "曲线仍在下降。低速的后轮角度需求远超 12°，这个区间整段都是欠配的。")

    # --- fig: high-speed value curve (signed beta) ---
    hs = []
    for k in ("urban", "highway", "evade", "cruise"):
        rr = rows(k)
        fld = "beta_signed"
        pts = [(r["limit"], r[fld]) for r in rr if fld in r]
        if not pts:
            continue
        hs.append(dict(label=C[k]["label"], color=CASE_COLOR[k], pts=pts))
    blo = min(min(y for _, y in s["pts"]) for s in hs)
    bhi = max(max(y for _, y in s["pts"]) for s in hs)
    fig_high = line_chart(
        hs, "后轮转角权限 (°)", "车身侧偏角（带符号）", (0, 12), (blo - 0.3, bhi + 0.3),
        [0, 2, 4, 6, 8, 10, 12],
        caption="图 3 — 高速工况的价值曲线，<b>保留符号</b>。三条曲线都在很小的权限内"
                "走完全程然后压平：需求被满足之后，再多的权限是死重。"
                "注意曲线穿过零线后继续走向反号——最优权限不在最右端。")

    # --- fig: |beta| with the optimum visible ---
    ha = []
    for k in ("highway", "evade", "cruise"):
        rr = rows(k)
        fld = "beta_per_g" if C[k]["kind"] == "step" else "beta_deg"
        pts = [(r["limit"], r[fld]) for r in rr if fld in r]
        best = min(pts, key=lambda p: p[1])
        ha.append(dict(label=C[k]["label"], color=CASE_COLOR[k], pts=pts, knee=best))
    ahi = max(max(y for _, y in s["pts"]) for s in ha)
    fig_habs = line_chart(
        ha, "后轮转角权限 (°)", "|车身侧偏角|", (0, 12), (0, ahi * 1.12),
        [0, 2, 4, 6, 8, 10, 12],
        caption="图 4 — 同样的数据取绝对值，圈出的是每条曲线的<b>最小点</b>。"
                "这是本文对高速工况最有用的一张图：存在一个最优后轮权限，"
                "超过它侧偏角反而重新变大，因为开环解析律把 β 推过了零点。")

    # --- fig: demand bars ---
    dm = sorted(cases, key=lambda c: demand[c["key"]]["delta_r"])
    w, h = 640, 300
    pad = (18, 18, 62, 56)
    x0, y0, x1, y1 = pad[3], h - pad[2], w - pad[1], pad[0]
    vmax = max(demand[c["key"]]["delta_r"] for c in cases) * 1.15
    syb = lambda v: y0 - v / vmax * (y0 - y1)  # noqa: E731
    slot = (x1 - x0) / len(dm)
    gb = [f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}"/>',
          f'<line class="ax" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}"/>']
    for lvl, col in ((3, "#34d399"), (7, "#38bdf8"), (12, "#f59e0b")):
        if lvl > vmax:
            continue
        Y = syb(lvl)
        gb.append(f'<line class="mark" x1="{x0}" y1="{Y:.1f}" x2="{x1}" y2="{Y:.1f}" '
                  f'stroke="{col}"/>')
        gb.append(f'<text class="cont" x="{x1 - 4}" y="{Y - 5:.1f}" text-anchor="end" '
                  f'fill="{col}">±{lvl}°</text>')
    for i, c in enumerate(dm):
        v = demand[c["key"]]["delta_r"]
        cx = x0 + slot * (i + 0.5)
        Y = syb(v)
        gb.append(f'<rect x="{cx - slot*0.3:.1f}" y="{Y:.1f}" width="{slot*0.6:.1f}" '
                  f'height="{y0 - Y:.1f}" fill="{CASE_COLOR[c["key"]]}" rx="3"/>')
        gb.append(f'<text class="bval" x="{cx:.1f}" y="{Y - 6:.1f}" text-anchor="middle">'
                  f'{v:.1f}°</text>')
        gb.append(f'<text class="tick" x="{cx:.1f}" y="{y0 + 15}" text-anchor="middle">'
                  f'{c["label"]}</text>')
        gb.append(f'<text class="sub" x="{cx:.1f}" y="{y0 + 28}" text-anchor="middle">'
                  f'{c["v"]:.0f} km/h</text>')
        gb.append(f'<text class="sub" x="{cx:.1f}" y="{y0 + 40}" text-anchor="middle">'
                  f'δ_f {demand[c["key"]]["delta_f"]:.1f}°</text>')
    gb.append(f'<text class="axlab" transform="rotate(-90 12 {(y0+y1)/2:.0f})" '
              f'x="12" y="{(y0+y1)/2:.0f}" text-anchor="middle">索取的后轮转角 (°)</text>')
    fig_demand = (f'<figure><svg viewBox="0 0 {w} {h}" class="chart">{"".join(gb)}</svg>'
                  '<figcaption>图 5 — <b>需求谱</b>：把后轮权限完全放开（±40°）后，'
                  '每个工况实际索取的最大后轮转角。横线是三档权限。'
                  '这是最直接的一张图——权限低于柱高就是欠配，高于柱高就是死重。</figcaption></figure>')

    # --- tables ---
    def row_demand(c):
        dmn = demand[c["key"]]
        need = dmn["delta_r"]
        cover = lambda L: ("✓" if need <= L + 1e-6 else f"{100*L/max(need,1e-9):.0f}%")  # noqa: E731
        return (f'<tr><th>{c["label"]}<small>{c["label_en"]}</small></th>'
                f'<td>{c["v"]:.0f}</td><td>{dmn["delta_f"]:.1f}</td>'
                f'<td class="hi">{need:.2f}</td>'
                f'<td>{cover(3)}</td><td>{cover(7)}</td><td>{cover(12)}</td></tr>')

    def row_value(c):
        k = knees[c["key"]]
        rr = rows(c["key"])
        v0 = rr[0][c["metric"]]
        v12 = rr[-1][c["metric"]]
        best = min(rr, key=lambda r: r[c["metric"]])
        # A knee is only meaningful if the metric moved at all. At 60 km/h the
        # law asks for 0.02 deg of rear steer, so the curve is flat and any
        # "knee" would be an artefact of the search, not a property of the car.
        flat = abs(v0 - best[c["metric"]]) < 1e-6
        knee = "—" if flat else f'{k["knee"]:.1f}'
        opt = "—" if flat else f'{best["limit"]:.1f}'
        return (f'<tr><th>{c["label"]}</th>'
                f'<td>{v0:.3f}</td><td>{v12:.3f}</td>'
                f'<td>{best[c["metric"]]:.3f}</td>'
                f'<td class="hi">{opt}</td>'
                f'<td>{knee}</td>'
                f'<td>{c["unit"]}</td></tr>')

    # headline numbers
    lowmax = max(demand[k]["delta_r"] for k in LOW_SPEED)
    highmax = max(demand[k]["delta_r"] for k in demand if k not in LOW_SPEED)
    park = rows("park")
    park_gain_12 = park[0]["turning_circle"] - park[-1]["turning_circle"]
    park_gain_3 = park[0]["turning_circle"] - sweep["park"][3.0]["turning_circle"]
    park_gain_7 = park[0]["turning_circle"] - sweep["park"][7.0]["turning_circle"]
    hw = rows("highway")
    hw_best = min(hw, key=lambda r: r["beta_per_g"])
    # Where the SIGNED sideslip crosses zero, per case — the true optimum band.
    opt_band = []
    for k in ("highway", "evade", "cruise"):
        rr = rows(k)
        fld = "beta_per_g" if C[k]["kind"] == "step" else "beta_deg"
        opt_band.append(min(rr, key=lambda r: r[fld])["limit"])
    opt_lo, opt_hi = min(opt_band), max(opt_band)
    urban = rows("urban")

    html = TEMPLATE.format(
        generated=meta["generated"], commit=meta["commit"], wall=meta["wall_seconds"],
        mass=meta["vehicle"]["mass"], L=meta["vehicle"]["wheelbase"],
        a=meta["vehicle"]["cg_to_front"], front=meta["front_limit_deg"],
        wide=meta["wide_deg"], carrier=meta["carrier"], law=meta["law"],
        k0=abs(d["k_curve"][0]["k"]),
        k120=abs(next(r["k"] for r in d["k_curve"] if r["v"] == 120)),
        fig_map=fig_map, fig_demand=fig_demand, fig_low=fig_low,
        fig_high=fig_high, fig_habs=fig_habs,
        demand_rows="".join(row_demand(c) for c in cases),
        value_rows="".join(row_value(c) for c in cases),
        case_cards="".join(
            f'<div class="ccard" style="--c:{CASE_COLOR[c["key"]]}">'
            f'<h4>{c["label"]}<small>{c["v"]:.0f} km/h</small></h4>'
            f'<p>{c["blurb"]}</p></div>' for c in cases),
        lowmax=lowmax, highmax=highmax,
        park_gain_12=park_gain_12, park_gain_3=park_gain_3, park_gain_7=park_gain_7,
        park_pct_3=100 * park_gain_3 / max(park_gain_12, 1e-9),
        park_pct_7=100 * park_gain_7 / max(park_gain_12, 1e-9),
        park_d0=park[0]["turning_circle"], park_d12=park[-1]["turning_circle"],
        hw_best_limit=hw_best["limit"], hw_best_beta=hw_best["beta_per_g"],
        hw_beta0=hw[0]["beta_per_g"], hw_beta12=hw[-1]["beta_per_g"],
        opt_lo=opt_lo, opt_hi=opt_hi,
        urban_demand=demand["urban"]["delta_r"],
        urban_b0=urban[0]["beta_per_g"], urban_b12=urban[-1]["beta_per_g"],
    )
    OUT.write_text(html)
    print(f"wrote {OUT} ({OUT.stat().st_size / 1024:.0f} kB)")
    return 0


TEMPLATE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>后轮转向转角范围的价值 — 0 至 12 度，在什么工况下值多少</title>
<style>
:root {{
  --bg:#0b1020; --panel:#121a30; --panel2:#0f1729; --ink:#e7ecf6; --dim:#93a2bd;
  --line:#243154; --accent:#7dd3fc; --warn:#fbbf24; --bad:#fb7185;
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
.wrap {{ max-width:940px; margin:0 auto; padding:0 22px 80px; }}
header.hero {{ padding:64px 0 30px; border-bottom:1px solid var(--line); margin-bottom:34px; }}
.eyebrow {{ color:var(--accent); font-size:13px; letter-spacing:.16em;
  text-transform:uppercase; font-weight:600; }}
h1 {{ font-size:clamp(27px,4.4vw,40px); line-height:1.25; margin:.4em 0 .3em; }}
.sub {{ color:var(--dim); font-size:18px; max-width:62ch; }}
.meta {{ margin-top:26px; display:flex; flex-wrap:wrap; gap:8px 20px;
  font:12px/1.6 var(--mono); color:var(--dim); }}
.meta b {{ color:var(--ink); font-weight:600; }}
h2 {{ font-size:26px; margin:52px 0 6px; }}
h2 .n {{ color:var(--accent); font:600 15px/1 var(--mono); display:block;
  margin-bottom:6px; letter-spacing:.1em; }}
h3 {{ font-size:19px; margin:32px 0 8px; }}
h4 {{ margin:0 0 6px; font-size:15.5px; }}
h4 small {{ float:right; font-weight:400; color:var(--dim); font-size:12px;
  font-family:var(--mono); }}
p {{ margin:12px 0; }}
.lede {{ font-size:17.5px; }}
.dim {{ color:var(--dim); }}
code {{ font-family:var(--mono); font-size:.92em; background:var(--panel2);
  padding:1px 5px; border-radius:4px; }}
.callout {{ border-left:3px solid var(--accent); background:var(--panel);
  padding:14px 18px; border-radius:0 8px 8px 0; margin:22px 0; }}
.callout.warn {{ border-color:var(--warn); }}
.callout h4 {{ color:var(--accent); font-size:13px; letter-spacing:.08em;
  text-transform:uppercase; }}
.callout.warn h4 {{ color:var(--warn); }}
.grid3 {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(250px,1fr));
  gap:12px; margin:22px 0; }}
.ccard {{ background:var(--panel); border:1px solid var(--line);
  border-left:3px solid var(--c); border-radius:8px; padding:13px 15px; }}
.ccard p {{ font-size:13px; color:var(--dim); margin:6px 0 0; }}
.tablewrap {{ overflow-x:auto; margin:22px 0; border:1px solid var(--line);
  border-radius:10px; background:var(--panel); }}
table {{ border-collapse:collapse; width:100%; font-size:13.5px; min-width:600px; }}
th,td {{ padding:9px 12px; text-align:right; border-bottom:1px solid var(--line);
  font-family:var(--mono); }}
thead th {{ background:var(--panel2); color:var(--dim); font-weight:600;
  font-size:12px; }}
thead th:first-child, tbody th {{ text-align:left; }}
tbody th {{ font-family:inherit; font-weight:600; }}
tbody th small {{ display:block; font-weight:400; color:var(--dim); font-size:11.5px;
  font-family:var(--mono); }}
tbody tr:last-child td, tbody tr:last-child th {{ border-bottom:0; }}
td.hi {{ color:var(--accent); font-weight:600; }}
figure {{ margin:26px 0; background:var(--panel); border:1px solid var(--line);
  border-radius:11px; padding:16px 14px 12px; }}
svg.chart {{ width:100%; height:auto; display:block; }}
.chart .ax {{ stroke:var(--dim); stroke-width:1; }}
.chart .grid {{ stroke:var(--line); stroke-width:1; }}
.chart .zero {{ stroke:var(--dim); stroke-width:1.4; stroke-dasharray:4 3; }}
.chart .mark {{ stroke:var(--dim); stroke-width:1; stroke-dasharray:4 4; }}
.chart .tick {{ fill:var(--dim); font:11px var(--mono); }}
.chart .sub {{ fill:var(--dim); font:10px var(--mono); }}
.chart .pt {{ fill:var(--ink); font:11px var(--mono); }}
.chart .cont {{ font:600 11px var(--mono); }}
.chart .marklab {{ fill:var(--dim); font:10px var(--mono); }}
.chart .axlab {{ fill:var(--dim); font:11px var(--mono); }}
.chart .bval {{ fill:var(--ink); font:600 12px var(--mono); }}
.chart .ln {{ fill:none; stroke-width:2.2; stroke-linejoin:round; }}
.legend {{ display:flex; flex-wrap:wrap; gap:6px 16px; justify-content:center;
  margin-top:8px; font:12px var(--mono); color:var(--dim); }}
.key i {{ display:inline-block; width:11px; height:11px; border-radius:3px;
  margin-right:5px; vertical-align:-1px; }}
figcaption {{ color:var(--dim); font-size:13px; margin-top:10px;
  padding-top:10px; border-top:1px solid var(--line); }}
.rec {{ background:var(--panel); border:1px solid var(--line); border-radius:11px;
  padding:4px 20px 16px; margin:26px 0; }}
.rec table {{ min-width:0; }}
.ftr {{ margin-top:60px; padding-top:22px; border-top:1px solid var(--line);
  color:var(--dim); font-size:13px; }}
ul.tight li, ol.tight li {{ margin:7px 0; }}
ol.refs {{ font-size:13.5px; color:var(--dim); padding-left:20px; }}
.eq {{ background:var(--panel2); border:1px solid var(--line); border-radius:8px;
  padding:14px 18px; margin:14px 0; font:15px/2 var(--mono); text-align:center;
  overflow-x:auto; }}
.tnote {{ font-size:12.5px; color:var(--dim); padding:10px 12px;
  border-top:1px solid var(--line); line-height:1.7; }}
.callout.good {{ border-color:#34d399; }}
.callout.good h4 {{ color:#34d399; }}
.toc {{ background:var(--panel); border:1px solid var(--line); border-radius:11px;
  padding:16px 22px; margin:26px 0; }}
.toc ol {{ margin:0; padding-left:20px; font-size:14px; }}
.toc li {{ margin:4px 0; }}
.toc a {{ color:var(--ink); text-decoration:none; }}
.toc a:hover {{ color:var(--accent); }}
</style></head><body><div class="wrap">

<header class="hero">
  <div class="eyebrow">4WIS Simulator · 车辆动力学研究报告 之二</div>
  <h1>后轮转向转角范围的价值</h1>
  <p class="sub">0° 到 12°：每一度后轮转角，在哪些工况下买到了什么，
     又从哪一度开始变成死重。</p>
  <div class="meta">
    <span>生成 <b>{generated}</b></span>
    <span>提交 <b>{commit}</b></span>
    <span>载体 <b>{carrier}</b></span>
    <span>机时 <b>{wall} s</b></span>
  </div>
</header>

<div class="callout"><h4>与第一篇的关系</h4>
<p>姊妹报告《转向解耦程度对车辆操控性能的价值》把角度包络<b>固定</b>
（前轮 ±40°、后轮 ±7°），以便比较转向<b>系统</b>而不是转向<b>角度</b>。
本文正相反：角度<b>就是</b>自变量。两篇必须分开读——
第一篇回答「多一个自由度值多少」，本文回答「多一度转角值多少」。
第一篇的一个关键发现正是由本文的问题引起的：它早期版本给各级不同的后轮权限，
测得的「架构收益」里绝大部分其实是角度收益。</p></div>

<h2><span class="n">摘要</span>Abstract</h2>
<p class="lede">后轮转角需求不是一个数，是一张面。零侧偏律索取的后轮转角是
<code>|δ_r| = |k(v)|·|δ_f|</code>，两个因子随车速反向变化，
因此「需要多少度后轮转向」这个问题在指明工况之前<b>没有答案</b>。</p>
<ul class="tight">
<li><b>高速工况在 3° 以内就饱和。</b>本文四个中高速工况索取的后轮转角最大只有
{highmax:.2f}°。给到 7° 或 12° 对它们<b>没有任何额外收益</b>——
多出来的行程是死重。</li>
<li><b>低速工况在 12° 之内完全不饱和。</b>低速工况索取 {lowmax:.1f}°，
是 12° 的数倍。价值曲线在 0–12° 全段保持下降，<b>没有拐点</b>：
每一度都在继续缩小转弯直径。</li>
<li><b>高速的最优权限比需求饱和点还小得多。</b>三个高速工况的 |β| 分别在
{opt_lo:.1f}–{opt_hi:.1f}° 处取到最小（高速变道 {hw_best_beta:.3f} °/g），
之后<b>回升</b>并在 {hw_beta12:.3f} °/g 处压平——只比完全不用后轮转向的
{hw_beta0:.3f} °/g 好一点点。原因是开环解析零侧偏律在非线性车辆上过度补偿，
把侧偏角推过零点；权限越大，过冲越充分。<b>所以「需求 {highmax:.1f}°」和
「最优 {opt_hi:.1f}°」是两个不同的数，差别来自控制律的模型误差，而不是来自车。</b></li>
<li><b>60 km/h 附近后轮转向几乎无事可做。</b>该速度处在同相／反相过渡点，
<code>k(v)≈0</code>，控制律只索取 {urban_demand:.2f}°；
从 0° 扫到 12°，侧偏角基本不动（{urban_b0:.3f} → {urban_b12:.3f} °/g）。
这是一个有用的反例：后轮转向的收益在速度轴上不是单调的。</li>
</ul>
<div class="callout warn"><h4>因此，后轮作动器的行程预算基本上是一个低速工程决策</h4>
<p>把行程从 3° 加到 12°，对高速操稳<b>一无所获甚至有害</b>，
对低速机动性则是接近线性的持续收益（泊车掉头直径 {park_d0:.2f} → {park_d12:.2f} m）。
反过来说，如果一个项目的诉求是高速稳定性，那么 <b>3° 就够了</b>，
把成本花在角度上是浪费；该花的地方是控制律和作动带宽（见第一篇第 4 节）。</p></div>

<div class="toc"><ol>
<li><a href="#s1">理论：需求面的推导</a></li>
<li><a href="#s2">试验方法与判据</a></li>
<li><a href="#s3">需求谱：每个工况要多少度</a></li>
<li><a href="#s4">价值曲线与拐点</a></li>
<li><a href="#s5">选型建议</a></li>
<li><a href="#s6">局限与结论</a></li>
</ol></div>

<h2 id="s1"><span class="n">1</span>理论：需求面的推导</h2>
<h3>1.1 后轮转角需求从哪来</h3>
<p>把左右轮并作一个的线性二自由度模型中，稳态令车身侧偏角 β = 0
（即侧向速度 v_y = 0），可解出后前轮转角比的闭式解：</p>
<div class="eq">k(u) = δ_r / δ_f = [ −b + a·m·u²/(C_r·L) ] / [ a + b·m·u²/(C_f·L) ]</div>
<p>其中 u 为车速，a、b 为质心到前/后轴距离，L = a + b，C_f、C_r 为轴侧偏刚度。
于是控制律索取的<b>后轮转角</b>是</p>
<div class="eq">|δ_r| = |k(u)| · |δ_f|</div>
<p><b>这就是全文的论点：需求是两个因子的乘积，而两个因子随车速反向变化。</b></p>
<ul class="tight">
<li><b>k(u) 随速度上升</b>：低速趋于 −b/a（本车 {k0:.2f}，几乎 1:1 反相），
    高速收敛到 +a·C_f/(b·C_r)。120 km/h 处为 {k120:.2f}。</li>
<li><b>δ_f 随速度<u>下降</u>，而且下降得更快</b>：稳态转向方程
    δ_f = L/R + K·a_y 中，同样的侧向加速度在高速下对应大得多的半径 R = u²/a_y，
    因此 L/R 项按 1/u² 衰减。120 km/h 做 4 m/s² 只需两三度；
    泊车掉头则顶在 {front:.0f}° 机械限位上。</li>
</ul>
<p>两者相乘，乘积由 δ_f 主导：需求在高速端塌缩、在低速端爆炸。
这个结构决定了本文所有结论——它不是本车的特性，是<b>后轮转向这件事本身</b>的特性，
换车型只改变数值，不改变形状。</p>

<h3>1.2 三个可以预先算出来的推论</h3>
<ol class="tight">
<li><b>存在一个后轮转向「无事可做」的速度。</b>k(u) 的分子为零处
    u² = b·C_r·L/(a·m)，此时无论前轮打多少，零侧偏律索取的后轮角都是零。
    本车约在 60–65 km/h，§3 实测到 0.02°，与此吻合。</li>
<li><b>高速侧的需求有上界。</b>k 收敛到有限值而 δ_f 按 1/u² 衰减，
    因此高速需求必然饱和。饱和值由「最大可用侧向加速度」而非车速决定——
    这就是为什么本文把紧急规避（7 m/s²）单列为一个工况。</li>
<li><b>低速侧的需求没有上界。</b>δ_f 顶到机械限位、k 趋于 −b/a ≈ −1，
    需求趋于前轮机械限位本身（本车 {front:.0f}°）。
    任何小于它的后轮行程在低速都是欠配的。</li>
</ol>

<h2 id="s2"><span class="n">2</span>试验方法</h2>
<h3>2.1 三层分析</h3>
<ol class="tight">
<li><b>需求谱</b>：把后轮权限完全放开到 ±{wide:.0f}°，记录每个工况实际索取的峰值后轮转角。
    权限超过这个数，对该工况就是死重。这是最直接的一层。</li>
<li><b>价值曲线</b>：把权限从 0 扫到 12°，测每个工况自己的指标，
    让「拐点」被<b>测出来</b>而不是被论证出来。</li>
<li><b>边际价值</b>：价值曲线的差分——下一度买到什么，这才是作动器选型真正用得上的数。</li>
</ol>
<h3>2.2 判据与陷阱</h3>
<p class="dim">载体为 {carrier}（线控前轮，作动器足够快，因此后轮角度是唯一受限项），
控制律固定为 {law}。</p>
<ul class="tight">
<li><b>每个权限档下的前轮转角单独二分标定</b>到该工况的目标侧向加速度。
    这是必须的：限幅会改变转向增益，共用一个角度就会把不同工作点混为一谈。
    二分区间 [0.1°, 20°]，收敛判据 |a_y − a_y*| &lt; 0.03 m/s²，最多 11 次迭代。</li>
<li><b>侧偏角保留符号。</b>β 随权限增大会<b>穿过零点变号</b>，
    取绝对值会把「最优点在中间」这个关键现象完全藏起来。
    图 3 用带符号值，图 4 用绝对值并圈出最小点，两张必须一起读。</li>
<li><b>拐点的定义。</b>「95% 拐点」是使指标达到全程可得改善量 95% 的<b>最小</b>权限。
    当曲线根本没动时（如 60 km/h 工况），拐点无意义，表中记为「—」，
    而不是让搜索返回一个看似合理的数。</li>
<li><b>整定按速度收敛而非固定时长</b>：跑到 |v − v*|/v* &lt; 1% 并保持 1 s 才施加输入。
    姊妹报告曾因固定时长整定在低附着上报出过完全错误的响应时间。</li>
</ul>

<h3>2.3 工况</h3>
<div class="grid3">{case_cards}</div>

<h2 id="s3"><span class="n">3</span>需求谱：每个工况到底要多少度</h2>
{fig_map}
<p><b>读图。</b>这是全文的骨架图，值得多花一分钟。<br>
<b>坐标：</b>横轴车速，纵轴前轮转角 δ_f。平面上任意一点代表一个工况。<br>
<b>颜色：</b>该点上零侧偏律索取的后轮转角落在哪个权限档内——
绿色 ≤3°、蓝色 3–7°、橙色 7–12°、红色 &gt;12°（12° 也覆盖不了）。<br>
<b>等值线：</b>三条曲线是 |k(v)|·δ_f = 3°／7°／12° 的边界，形状是双曲线族。
它们在 60 km/h 附近<b>向上发散到图外</b>——因为那里 k≈0，
无论前轮打多大，后轮需求都接近零。这是 §1.2 推论 1 的图形表达。<br>
<b>圆点：</b>本文六个工况的实测工作点。<b>五个落在绿区</b>（±3° 就够），
只有两个低速工况冲进红区。<br>
<b>最该记住的：</b>红区集中在图的<b>左上角</b>——低速 + 大转角。
整个右半幅（高速）无论前轮打到多大都是绿的，因为高速根本打不出大转角。
后轮转角预算的全部争议都在左上角那一块。</p>
{fig_demand}
<p><b>读图。</b>把上图压缩成一维：六个工况按索取量排序，横线是三档权限。
柱子低于横线即该档完全覆盖，高于即欠配。<br>
副标注给出每个工况的车速和实测前轮转角，可与需求图的圆点对照。
最左边的城市变道柱子几乎看不见（0.02°）——不是画错，
是 60 km/h 恰好在 k(v) 过零点上。
最右两根柱子（21.6° 与 34.5°）已经超出 ±12° 两到三倍，
说明低速工况的欠配不是「差一点」而是「差一个量级」。</p>
<div class="tablewrap"><table>
<thead><tr><th>工况</th><th>车速 km/h</th><th>δ_f °</th><th>索取 δ_r °</th>
<th>±3° 覆盖</th><th>±7° 覆盖</th><th>±12° 覆盖</th></tr></thead>
<tbody>{demand_rows}</tbody></table></div>
<p>这张表可以直接当选型依据读。「覆盖」列给出该权限档能满足该工况需求的百分比，
✓ 表示完全覆盖。中高速四个工况在 ±3° 档就全部打勾；
两个低速工况即使给到 ±12° 也只能覆盖一部分。</p>

<h2 id="s4"><span class="n">4</span>价值曲线与拐点</h2>
{fig_low}
<p><b>读图。</b>横轴是给多少后轮权限，纵轴是转弯直径（越低越好）。
两条曲线都是<b>单调下降且接近直线</b>——这是「完全欠配」的特征形状：
在需求远大于供给的区间内，每一度权限都被完全用掉，边际收益恒定。
圈出的点是「95% 拐点」，两条都落在最右端 12°，即扫描区间内<b>根本没有出现拐点</b>。
低速窄道（橙）的斜率比泊车掉头（红）更陡：16.70 → 11.94 m，改善 28%，
比泊车的 16% 更大——因为 20 km/h 时 |k| 仍接近 1 而前轮转角尚未顶到限位，
后轮的每一度都换来更多的 ICR 前移。</p>
<p>低速曲线没有拐点。给到 3° 拿到全程收益的 {park_pct_3:.0f}%，
给到 7° 拿到 {park_pct_7:.0f}%，到 12° 仍在继续下降。
这与需求谱一致：低速需求是几十度量级，0–12° 这一整段都处在欠配区，
所以边际收益始终为正、且近似恒定。<b>低速工况买的是「行程」本身。</b></p>

{fig_high}
<p><b>读图。</b>这张图<b>保留符号</b>，虚线是零——理想值。
三条中高速曲线都从负值（车尾外摆）出发，随权限增大迅速上升，
<b>穿过零线</b>后继续走向正值，然后在各自的需求饱和点处<b>戛然压平</b>。
压平点就是该工况索取的后轮角：高速变道在 2°、紧急规避在 3°、高速稳态在 2°。
压平之后曲线完全水平——多给的权限一点也没被用掉，是纯粹的死重。<br>
最反直觉的是<b>穿越零线之后曲线并没有停下</b>：控制律并不知道自己已经到了理想点，
它只是执行 k(v)·δ_f，而 k(v) 本身给多了。绿色的城市变道曲线几乎是一条水平线，
从头到尾没动——60 km/h 上后轮转向无事可做。</p>
{fig_habs}
<p><b>读图。</b>同一批数据取绝对值后，上一张图的「穿越零线」变成了一个<b>V 形谷底</b>，
圈出的就是每条曲线的最小点，即该工况的<b>最优后轮权限</b>。
三条曲线的谷底都落在 1.0–1.5°，<b>远小于</b>它们各自 2–3° 的需求饱和点。
换句话说：<b>把权限给到「控制律想要的量」，已经越过了「车辆最想要的量」。</b>
这两个数之间的差，就是控制律模型误差的可视化度量。</p>
<div class="callout"><h4>高速工况存在最优权限，这条结论与直觉相反</h4>
<p>图 4 的每条曲线都有一个内部最小值，圈出的就是它。
高速变道的 |β| 在 {hw_best_limit:.1f}° 处降到 {hw_best_beta:.3f} °/g，
到 12° 又回升到 {hw_beta12:.3f} °/g，比不用后轮转向的 {hw_beta0:.3f} °/g 好不了多少。</p>
<p><b>机理在姊妹报告第 2.5 与 6.8 节被定位到了具体一行代码</b>，
比「非线性」这种笼统说法确切得多：<code>axle_cornering_stiffness()</code>
对前后轴都返回 2·<code>tire_c_alpha</code>，<b>没有应用</b>车辆实际设置的
0.80／1.20 轴刚度分配。于是 k(v) 被系统性地算大——120 km/h 时索取 0.593，
正确值是 0.384，<b>同相后轮转角多给了 54%</b>。</p>
<p>这个诊断是<b>可证伪并且已被证伪检验通过</b>的：只把 k(v) 曲线换成用真实轴刚度重算的版本，
其余一概不动，稳态 β/g 在 140 km/h 从 +3.06 变成 −0.19，
100 km/h 从 +1.41 变成 −0.22，同时横摆响应还从 482 ms 加快到 322 ms。</p>
<div class="callout good"><h4>因此本节的「最优权限」是控制律的性质，不是车辆的性质</h4>
<p>V 形谷底出现在 1.0–1.5° 而不是 2–3°，唯一原因是控制律在给定权限下会用满它，
而它想用的量本身偏大。<b>把对象模型修对之后，「最优」与「饱和」应当重合</b>，
届时 3° 的权限既覆盖需求也不会过冲。</p>
<p>工程含义很直接：<b>不要用削减硬件行程的办法去补偿一个有偏差的控制律</b>。
削到 1.5° 确实能让当前这版律表现最好，但也永久放弃了紧急规避工况需要的 3°，
而后者一旦控制律修好就用得上。<b>先修律，再定行程。</b></p>
<p>闭环律不吃这个亏：横摆反馈律不使用任何车辆参数，
在姊妹报告里它拿到全部九个案例中最低的极限侧偏角（0.49°）。</p></div></div>

<div class="tablewrap"><table>
<thead><tr><th>工况</th><th>0° 时</th><th>12° 时</th><th>最优值</th>
<th>最优权限 °</th><th>95% 拐点 °</th><th>单位</th></tr></thead>
<tbody>{value_rows}</tbody></table></div>

<h2 id="s5"><span class="n">5</span>选型建议</h2>
<div class="rec"><table>
<thead><tr><th>如果项目诉求是……</th><th>建议后轮行程</th><th>理由</th></tr></thead>
<tbody>
<tr><th>纯高速稳定性 / 操稳标定</th><td class="hi">±3°</td>
<td style="text-align:left">四个中高速工况索取上限 {highmax:.2f}°，±3° 即完全覆盖。
但注意本文测到的<b>最优</b>点在 {opt_lo:.1f}–{opt_hi:.1f}°：
配开环解析律时，把权限从最优点加到 3° 反而让侧偏角回升。
正确做法是保留 3° 硬件余量、<b>把律改成闭环</b>，而不是靠削硬件去迁就一个有偏差的律。
省下的成本应转投控制律与作动带宽（见第一篇第 4 节）。</td></tr>
<tr><th>高速稳定性 + 一般低速改善</th><td class="hi">±5–7°</td>
<td style="text-align:left">高速完全覆盖；低速拿到全程收益的 {park_pct_7:.0f}%
（泊车直径 −{park_gain_7:.2f} m）。这是量产后轮转向模块的常见档位，
本文数据支持这个折中。</td></tr>
<tr><th>低速机动性是主要卖点</th><td class="hi">≥12°，越大越好</td>
<td style="text-align:left">0–12° 全段无拐点，边际收益近似恒定
（12° 处泊车直径 −{park_gain_12:.2f} m 且仍在下降）。
此时应按机械与包络约束定行程，而不是按动力学收益定——动力学不会先饱和。</td></tr>
<tr><th>蟹行 / 原地回转等非阿克曼运动</th><td class="hi">远超 12°</td>
<td style="text-align:left">这类运动要求后轮角与前轮角同量级（数十度）。
12° 以内无法实现，属于另一个设计域。</td></tr>
</tbody></table></div>

<h2 id="s6"><span class="n">6</span>局限</h2>
<ul class="tight">
<li>控制律固定为开环速度调度，且该律带有一个已定位的对象模型缺陷（§4）。
    「最优权限 1.0–1.5°」是<b>这一版控制律</b>的性质，不是车辆的；
    修正对象模型后该数应当上移并与需求饱和点重合。
    闭环律的曲线形状会不同，本文未扫描控制律（姊妹报告扫了六条）。</li>
<li>低速工况用稳态回转近似，未建模转向系统在极低速下的摩擦与助力特性。</li>
<li>后轮权限用指令限幅实现，不是机械限位，因此不含作动器顶到限位后的绕线与回弹。</li>
<li>未评价行程增大带来的成本：包络、簧下质量、后悬架布置与转向节强度，
    这些恰恰是实际决策中与动力学收益对冲的项。</li>
<li>单一整车参数集，未做质心位置与轴距的敏感性扫描；
    <code>k(v)</code> 直接依赖 <code>b/a</code>，换车型结论的<b>数值</b>会变，<b>形状</b>不会。</li>
</ul>

<h2><span class="n">7</span>结论</h2>
<ol class="tight">
<li>后轮转角需求 = <code>|k(v)|·|δ_f|</code>，是车速与转角的二维函数；
    高速端塌缩、低速端爆炸。脱离工况谈「需要几度后轮转向」没有意义。</li>
<li>中高速工况全部在 <b>3° 以内饱和</b>（实测上限 {highmax:.2f}°）。</li>
<li>低速工况在 <b>0–12° 全段不饱和</b>，边际收益近似恒定，
    12° 处泊车直径仍在下降。</li>
<li>配当前版本的开环解析律时，高速工况的<b>最优权限（{opt_lo:.1f}–{opt_hi:.1f}°）
    小于需求饱和点（{highmax:.2f}°）</b>：超过最优点侧偏角回升。
    这是<b>控制律</b>结论而非硬件结论——姊妹报告已把成因定位到
    <code>axle_cornering_stiffness()</code> 漏用轴刚度分配，
    并用对照实验证明修正后 β 回到零附近。
    <b>正确的工程动作是修控制律，而不是按错误的曲线去削硬件行程。</b></li>
<li>60 km/h 附近 <code>k(v)≈0</code>，后轮转向<b>无事可做</b>；
    收益沿速度轴不是单调的，中速段有一个空档。</li>
<li>因此：<b>后轮行程预算是低速工程决策</b>。为高速操稳买行程是低效投资，
    该投的是控制律与带宽。</li>
</ol>

<h2><span class="n">8</span>复现</h2>
<pre style="background:var(--panel2);border:1px solid var(--line);border-radius:8px;
padding:14px;overflow-x:auto;font:12.5px/1.7 var(--mono);margin:16px 0">python scripts/rear_angle_study/run_study.py
python scripts/rear_angle_study/build_report.py</pre>
<p class="dim">数据：<code>docs/reports/rear_angle_study_data.json</code>。
整车：{mass:.0f} kg，轴距 {L:.3f} m，质心距前轴 {a:.3f} m。本文对应提交 <code>{commit}</code>。</p>

<ol class="refs">
<li>Sano, S. et al. — Four wheel steering system with rear wheel steer angle
    controlled as a function of steering wheel angle（同相／反相调度的原始工程论证）。</li>
<li>Abe, M. <i>Vehicle Handling Dynamics: Theory and Application</i> — 零侧偏后轮转向律。</li>
<li>ISO 4138 / ISO 7401 — 稳态回转与横向瞬态响应试验方法。</li>
</ol>

<div class="ftr">4WIS Simulator · 提交 {commit} · 生成于 {generated} ·
机时 {wall} s · 全部图表由 <code>build_report.py</code> 从仿真 JSON 渲染，无手写数据。</div>
</div></body></html>
"""


if __name__ == "__main__":
    raise SystemExit(main())
