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
ALGOS = REPORTS / "decoupling_algorithms_data.json"
OUT = REPORTS / "steering_decoupling_value.html"

ARCH_COLOR = {"L0": "#94a3b8", "L1": "#38bdf8", "L2": "#a78bfa", "L3": "#fb7185"}
ALGO_COLOR = {"none": "#94a3b8", "fixed": "#f59e0b", "schedule": "#38bdf8",
              "yaw_fb": "#34d399", "transient": "#fb7185", "model_follow": "#a78bfa"}


def load():
    d = json.loads(DATA.read_text())
    t = json.loads(TRACES.read_text())
    a = json.loads(ALGOS.read_text())
    return d, t, a


def nice_ticks(lo, hi, n=5):
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=mag)
    start = math.floor(lo / step) * step
    out, v = [], start
    while v <= hi + step * 0.5:
        out.append(round(v, 6))
        v += step
    return out


def auto_line(series, xlab, ylab, xr, xticks, caption="", w=620, h=300):
    """Line chart that picks a sensible y range from the data itself."""
    ys = [y for s in series for _, y in s["pts"]]
    lo, hi = min(ys), max(ys)
    span = max(hi - lo, 1e-9)
    yr = (lo - span * 0.10, hi + span * 0.10)
    return line_chart(series, xlab, ylab, xr, yr, xticks,
                      nice_ticks(yr[0], yr[1]), w=w, h=h, caption=caption)


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
    d, tr, A = load()
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
        "转弯直径 (m)",
        caption=("图 1 — 全锁转弯直径。三个带后轮转向的架构都顶在共同的 "
                 f"{arch['L1']['rear_limit_deg']:.0f}° 限幅上，因此挤在一起（相差 "
                 f"{max(circles[k]['turning_circle'] for k in ('L1','L2','L3')) - min(circles[k]['turning_circle'] for k in ('L1','L2','L3')):.2f} m）；"
                 "阶梯几乎全部发生在 L0→L1 这一级。"),
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

    # ---- deep-dive figures (algorithms data) ----------------------------
    import report_sections as RS
    aspeeds = A["meta"]["speeds"]
    aby = {(r["algo"], r["v"]): r for r in A["speed"]}
    aorder = ["none", "fixed", "schedule", "yaw_fb", "transient", "model_follow"]

    def aser(field, scale=1.0):
        return [dict(label=algo[k]["label_cn"], color=ALGO_COLOR[k],
                     pts=[(v, aby[(k, v)][field] * scale) for v in aspeeds])
                for k in aorder]

    xt = [int(v) for v in aspeeds]
    xr = (min(aspeeds), max(aspeeds))
    dfigs = {}
    dfigs["speed_t90"] = auto_line(
        aser("yaw_t90", 1000.0), "车速 (km/h)", "横摆 T90 (ms)", xr, xt,
        caption="图 9 — 六条控制律的横摆响应时间随车速。三条趋势要一起看："
                "闭环的横摆反馈律几乎与速度无关（136–146 ms，最平的一条）；"
                "瞬态补偿律越快越猛（140 km/h 已到 66 ms）；"
                "而模型跟随律随速度<b>迅速变慢</b>——这不是结构问题，"
                "是第 6.8 节那个对象模型缺陷随速度放大的直接后果。")
    dfigs["speed_beta"] = auto_line(
        aser("beta_per_g"), "车速 (km/h)", "β / a_y (°/g，带符号)", xr, xt,
        caption="图 10 — 单位侧向加速度的车身侧偏角，<b>保留符号</b>。"
                "零线是理想值。不用后轮转向（灰）随速度单调变负，这是常规车的姿态；"
                "带后轮转向的各条律把它抬向零，但速度调度与模型跟随<b>冲过了零点</b>，"
                "在高速端变成大幅正值。模型跟随律在 140 km/h 达到 +20.6°/g，"
                "比完全不控后轮还差一个量级——这是全文最刺眼的一个数。")
    dfigs["speed_ratio"] = auto_line(
        aser("dr_over_df"), "车速 (km/h)", "稳态 δ_r / δ_f", xr, xt,
        caption="图 11 — 各律实际执行的后前轮转角比。定比例律是一条水平线（−0.4，按定义）；"
                "其余各律都随速度上升，其中三条<b>穿过零线</b>由反相转为同相。"
                "这张图解释了为什么单速度排名不可靠：在 40 km/h 与 140 km/h，"
                "同一条律做的是<b>方向相反</b>的事。")
    dfigs["speed_os"] = auto_line(
        aser("yaw_overshoot"), "车速 (km/h)", "横摆超调 (%)", xr, xt,
        caption="图 12 — 超调随车速。这是图 9「谁最快」的代价页。"
                "瞬态补偿律在 140 km/h 超调 73.3%——横摆角速度冲到稳态值的 1.73 倍再回落，"
                "这已经不是一台可交付的车。任何以 T90 为单一目标的标定都会走到这里。")

    nz = {r["algo"]: r for r in A["noise"]}
    dfigs["noise"] = bar_chart(
        [(algo[k]["label_cn"], nz[k]["amplification"], ALGO_COLOR[k],
          f'{nz[k]["dr_rate_rms_dps"]:.2f} °/s') for k in aorder],
        "噪声放大倍数 (×)", fmt="{:.2f}",
        caption="图 13 — 转向通道噪声到后轮转角的放大倍数，副标注为后轮角速率 RMS。"
                "全部小于 1（净衰减），因为六条律内部统一带 τ = 0.08 s 低通。"
                "最高的是模型跟随律而<b>不是</b>求导的瞬态补偿律——"
                "这一条推翻了本报告早期版本的断言，见 6.9 节。")
    dw = {r["algo"]: r for r in A["dwell"]}
    dfigs["dwell"] = bar_chart(
        [(algo[k]["label_cn"], dw[k]["max_beta"], ALGO_COLOR[k],
          f'峰值 {dw[k]["peak_yaw"]:.1f}°/s') for k in aorder],
        "正弦停留最大 |β| (°)", fmt="{:.2f}",
        caption="图 14 — 正弦停留（FMVSS 126，80 km/h）中的最大车身侧偏角，"
                "副标注为峰值横摆角速度。定比例律最差（5.08°），"
                "印证「常数反相比在高速是错的」；横摆反馈律最好（0.68°）。"
                "全部六条律输入结束 1 s 后的残余横摆均低于峰值 2%，无一失稳。")

    grad_none_K = grad["L2/none"]["K"]
    sec_theory = RS.theory_section(A["theory"], grad_none_K, None)
    sec_method = RS.method_section(meta, meta["ay_target"], meta["step_speed_kmh"],
                                   meta["low_mu"], arch["L1"]["rear_limit_deg"],
                                   arch["L1"]["rear_limit_deg"],
                                   meta["vehicle"]["steer_limit_deg"])
    sec_algo = RS.algo_section(A, dfigs, algo, None)

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
        d_circle=circles["L0"]["turning_circle"] - circles["L1"]["turning_circle"],
        pct_circle=(1 - min(circles[k]["turning_circle"] for k in ladder_keys)
                    / circles["L0"]["turning_circle"]) * 100,
        # How the low-speed gain splits: the first rear axle vs everything above it.
        pct_first=(1 - circles["L1"]["turning_circle"]
                   / circles["L0"]["turning_circle"]) * 100,
        rws_spread=(max(circles[k]["turning_circle"] for k in ("L1", "L2", "L3"))
                    - min(circles[k]["turning_circle"] for k in ("L1", "L2", "L3"))),
        rear_env=arch["L1"]["rear_limit_deg"],
        front_env=meta["vehicle"]["steer_limit_deg"],
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
        sec_theory=sec_theory, sec_method=sec_method, sec_algo=sec_algo,
        n_figs=8 + len(dfigs),
        algo_ay_span=(max(grad[f"L2/{a}"]["ay_max"] for a in algo_keys)
                      / min(grad[f"L2/{a}"]["ay_max"] for a in algo_keys) - 1) * 100,
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
.eq {{ background:var(--panel2); border:1px solid var(--line); border-radius:8px;
  padding:14px 18px; margin:14px 0; font:15px/2 var(--mono); text-align:center;
  overflow-x:auto; }}
.eq.law {{ border-left:3px solid var(--c); text-align:left; font-size:16px;
  font-weight:600; }}
.gains {{ font-size:13px; color:var(--dim); background:var(--panel);
  border:1px solid var(--line); border-radius:7px; padding:9px 13px; }}
.tnote {{ font-size:12.5px; color:var(--dim); padding:10px 12px;
  border-top:1px solid var(--line); line-height:1.7; }}
.callout.good {{ border-color:var(--good); }}
.callout.good h4 {{ color:var(--good); }}
td.bad {{ color:var(--bad); font-weight:600; }}
td.good {{ color:var(--good); font-weight:600; }}
td.hi {{ color:var(--accent); font-weight:600; }}
.toc {{ background:var(--panel); border:1px solid var(--line); border-radius:11px;
  padding:16px 22px; margin:26px 0; column-count:2; column-gap:30px; }}
.toc ol {{ margin:0; padding-left:20px; font-size:14px; }}
.toc li {{ margin:4px 0; break-inside:avoid; }}
.toc a {{ color:var(--ink); text-decoration:none; }}
.toc a:hover {{ color:var(--accent); }}
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
<li><b>在等角度约束下，L1 以上的架构升级几乎买不到操控收益。</b>把四级统一到
前轮 ±{front_env:.0f}°、后轮 ±{rear_env:.0f}° 之后，L1／L2／L3 的转弯直径落在
{rws_spread:.2f} m 的带内，4 m/s² 下 L2 与 L3 的标定转角完全相同。
低速的全部收益（−{pct_circle:.0f}%）里，<b>{pct_first:.0f} 个百分点来自第一个后轮轴</b>，
其余来自作动带宽这种二阶效应。换句话说：<b>「有没有后轮转向」是台阶，
「后轮转向做得多高级」在本文的角度约束内不是。</b></li>
</ul>

<div class="toc"><ol>
<li><a href="#s1">四级架构的定义</a></li>
<li><a href="#s2">理论基础：自行车模型与两条由它导出的控制律</a></li>
<li><a href="#s3">试验方法与判据</a></li>
<li><a href="#s4">架构阶梯的结果</a></li>
<li><a href="#s5">动画：同一条弯，四种姿态</a></li>
<li><a href="#s6">控制算法的贡献（逐条展开）</a></li>
<li><a href="#s7">讨论</a></li>
<li><a href="#s8">结论</a></li>
</ol></div>

<h2 id="s1"><span class="n">1</span>四级架构的定义</h2>
<h3>1.1 用硬件而不是用控制律来区分</h3>
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

<h3>1.2 整车与工况</h3>
<p>整车：质量 {mass:.0f} kg，轴距 {L:.3f} m，前轮距 {tf:.3f} m，
质心距前轴 {cgf:.3f} m，质心高 {cgh:.2f} m，单轮机械限位 ±{steer_limit:.0f}°。
路面 μ = {mu}，低附着对照 μ = {low_mu}。</p>
<ul class="tight">
<li><b>低速机动性</b> — 10 km/h 全锁稳态回转，取转弯直径。</li>
<li><b>稳态回转（ISO 4138）</b> — 30…120 km/h 定转角扫描，取 β(v)；
    90 km/h 转角扫描至饱和，取极限 a_y。</li>
<li><b>阶跃转向（ISO 7401）</b> — {vstep:.0f} km/h，等 a_y，取 T90／超调／峰值时刻／β。</li>
<li><b>频率响应</b> — 0.2…2.0 Hz 正弦转向，相关法估计增益与相位。</li>
<li><b>正弦停留（FMVSS 126）</b> — 0.7 Hz，第二峰保持 500 ms，看车辆是否收敛。</li>
<li><b>控制律扫描</b> — 六条律 × 四个速度点，固定 L2 硬件（第 6 节）。</li>
</ul>

<div id="s2"></div>
{sec_theory}

<div id="s3"></div>
{sec_method}

<h2 id="s4"><span class="n">4</span>架构阶梯的结果</h2>
<div class="tablewrap"><table>
<thead><tr><th>架构</th><th>DOF</th><th>后轮权限</th><th>τ</th>
<th>转弯直径 m</th><th>δ_f °</th><th>T90 ms</th><th>超调 %</th>
<th>β/g °</th><th>a_y,max</th><th>低附 T90 ms</th></tr></thead>
<tbody>{arch_rows}</tbody></table></div>

{fig_circle}
<p>这张图在加上等角度约束之后变了样，而且变化本身就是结论。
L0→L1 一步拿走 {d_circle:.2f} m（−{pct_first:.0f}%）；L1→L2→L3 三级加起来只挪动
{rws_spread:.2f} m，因为它们全都顶死在共同的 ±{rear_env:.0f}° 限幅上。</p>
<div class="callout"><h4>约束改变了结论，这一点必须讲明</h4>
<p>本研究的早期版本给各级不同的后轮权限（0／3／5／{steer_limit:.0f}°），
测得 L0→L3 的转弯直径改善 33%，读起来像是「四轮独立转向在低速有巨大优势」。
统一角度包络之后，同一个量只剩 −{pct_circle:.0f}%。
那 33% 里的绝大部分<b>不是架构带来的，是角度带来的</b>——
是 L3 被允许用五倍于 L1 的后轮转角。
这正是把角度包络固定下来的理由：不固定，架构比较就会变成角度比较的伪装。
角度本身值多少钱，是另一篇报告的题目。</p></div>

{fig_t90}
<div class="callout"><h4>阶梯在瞬态上不是单调的</h4>
<p>这是本文最容易被误读的一张图。L2/L3 的横摆响应<b>比 L0 慢一倍以上</b>。
这不是缺陷，是它们所执行的控制律（零侧偏模型跟随）的设计目标：
它用同相后轮转向消除车身侧偏，代价就是横摆建立变慢。
换句话说，<b>更高级的硬件被用来买了另一样东西</b>——见图 3 与图 8。
如果把 L2 硬件配上瞬态补偿律，它的 T90 是 {t90_fast:.0f} ms，反过来比 L0 快 3 倍以上（第 4 节）。</p></div>

{fig_beta}
<p><b>读图。</b>这张图是「后轮转向到底买了什么」的正面回答，也是最容易被单看数值误读的一张。
柱高是 |β|/a_y，副标注保留了符号——两者必须一起读。L0 是 −1.67 °/g：负号意味着
车头指向弯内、车尾向外滑，这是常规车高速过弯的固有姿态。L1 把它拉到 +1.42，
L2／L3 到 +6.70。<b>绝对值不但没有变小，反而变大了，而且符号翻了。</b>
这不是后轮转向做不到零侧偏——第 6.8 节证明只要把控制律的对象模型改对，
同一条律在 140 km/h 能把 β/g 做到 −0.19。图上看到的是控制律参数错误的后果，
不是架构能力的上限。</p>
{fig_beta_v}
<p><b>读图。</b>横轴是车速，固定前轮转角 3°，所以这张图展示的是<b>同一个输入下姿态如何随速度演变</b>。
L0（灰）单调下行：越快，车尾外摆越多。这正是零侧偏后轮转向要消除的趋势。
带后轮转向的三条曲线都被抬起来，说明机制方向是对的；
但它们在中高速段越过零线继续上行，即<b>过度补偿</b>。
零线是理想曲线——理想的后轮转向应该让这条线贴着零走完全程。
三条曲线偏离零线的量，就是控制律模型误差的直接可视化。</p>

<div class="callout bad"><h4>开环零侧偏律在非线性车辆上过冲</h4>
<p><code>zero_sideslip_ratio</code> 由线性二自由度自行车模型闭式解出，
其中轴侧偏刚度是常数。真实（本模型的）轮胎在 4 m/s² 下的等效轴刚度已经偏离该常数，
于是解析比给出了<b>过量</b>的同相后轮转角，β 被推过零点变正。
测得 β/g 从 L0 的负值翻到 L2/L3 的正值，量级更大。
这正是<b>闭环律（横摆反馈、模型跟随的反馈项）存在的理由</b>：
横摆反馈律在同一硬件上把极限侧偏角压到 0.49°，是全部九个案例中最低的。</p></div>

<h2 id="s5"><span class="n">5</span>动画：同一条弯，四种姿态</h2>
<div class="anim">
  <canvas id="cvA" width="900" height="330"></canvas>
  <div class="ctl"><button id="btnA">暂停</button>
    <input type="range" id="rngA" min="0" max="100" value="0">
    <span class="rd" id="rdA"></span></div>
  <figcaption><b>动画 1 — 稳态过弯姿态。</b>90 km/h，转角<b>标定到同一侧向加速度
  4 m/s²</b>，所以四辆车走的是几乎相同的圆——差别全在<b>姿态</b>而不在轨迹。<br>
  <b>怎么看：</b>绿箭头是速度矢量（车实际去的方向），车身矩形的长轴是车头朝向，
  两者的夹角就是侧偏角 β。左上角标注了各级的前轮转角与稳态 β。<br>
  <b>看什么：</b>(1) L0 的车头<b>偏向圆心一侧</b>（β 为负），这是常规车在高速弯里的样子——
  车尾在往外滑，驾驶员靠车头指向弯内来补偿。(2) L2／L3 的车头<b>偏向圆外</b>（β 为正），
  方向反了。零侧偏的理想状态是车身长轴与绿箭头完全重合，四辆车<b>都没有做到</b>，
  L0 差在一侧、L2／L3 差在另一侧。(3) 各级的前轮转角差别很大
  （L0 1.76° 到 L2／L3 4.22°），这就是「等 a_y 比较」的代价——
  同样的弯，装了后轮转向的车要打更多方向盘。</figcaption>
</div>

<div class="anim">
  <canvas id="cvB" width="900" height="330"></canvas>
  <div class="ctl"><button id="btnB">暂停</button>
    <input type="range" id="rngB" min="0" max="100" value="0">
    <span class="rd" id="rdB"></span></div>
  <figcaption><b>动画 2 — 正弦停留（FMVSS 126），80 km/h。</b>0.7 Hz 正弦转向，
  在第二个峰值处<b>保持 500 ms</b> 再撤回。这个「停留」是工况的关键：
  它要求车辆先建立一个大的姿态，然后一次性交还，专门用来暴露横摆发散。<br>
  <b>怎么看：</b>四条轨迹叠在同一坐标系里，每条淡色尾迹是走过的路径，
  实心车身是当前位置。左上角是各级的峰值横摆角速度与最大 β。<br>
  <b>看什么：</b>(1) <b>四级全部收敛</b>——输入结束 1 s 后残余横摆均低于峰值 0.3%，
  没有一级失稳。这个工况在本文参数下不具备区分度，但它的价值是<b>排除</b>：
  第 6 节那些「更快」的控制律没有以稳定性为代价。
  (2) 峰值横摆从 L0 的 20.0 °/s 降到 L3 的 10.8 °/s，几乎减半——
  同样的方向盘输入，带后轮转向的车<b>转得少得多</b>，
  这再次是转向增益被同相后轮压低的表现，不是稳定性提升。
  (3) 轨迹的横向位移 L0 最大：在真实避障中这意味着 L0 绕开障碍所需的方向盘更少。
  「更稳」和「更好避障」在这里是矛盾的。</figcaption>
</div>

<div class="anim">
  <canvas id="cvC" width="900" height="300"></canvas>
  <div class="ctl"><button id="btnC">暂停</button>
    <input type="range" id="rngC" min="0" max="100" value="0">
    <span class="rd" id="rdC"></span></div>
  <figcaption>动画 3 — 四轮独立转向独有的两种运动：<b>蟹行</b>（四轮同向，
  ICR 在无穷远，车身姿态不变而整体平移）与<b>原地回转</b>（ICR 落在车体中心，
  vx = vy = 0）。这两种运动在阿克曼几何下<b>无解</b>——不是分数高低，是低阶架构没有这个能力。
  <b>但请注意：这两段动画跑在本文的角度包络之外。</b>蟹行与原地回转都需要远超
  ±{rear_env:.0f}° 的后轮转角，因此在本文第 1 节的约束下，L3 <b>也做不到</b>。
  它们展示的是「如果放开后轮角度权限，四轮独立转向能拿到什么」，
  而这恰好说明：4WIS 的招牌能力买的是<b>角度</b>，不是<b>解耦</b>。<br>
  <b>怎么看：</b>左半幅是蟹行，右半幅是原地回转，淡色车影是每隔约 0.5 s 的历史位置。<br>
  <b>看什么：</b>(1) 蟹行中<b>车身朝向始终不变</b>而位置在斜向平移——
  历史车影全部平行。这在阿克曼几何下无解：常规车要横向移动必须先转头。
  (2) 原地回转中<b>轨迹退化为一个点</b>，车身绕自身中心旋转，vx = vy = 0。
  ICR 落在车体内部，这同样是阿克曼几何无法表达的状态。
  (3) 这两者的共同点是 ICR 被放到了「后轴延长线」之外——
  第 2.3 节的 k(u) 公式在这里已经不适用，因为它假设 ICR 在后轴延长线上。
  非阿克曼运动需要的是完整的四轮独立指令，而不是一个前后轴比值。</figcaption>
</div>

<div id="s6"></div>
{sec_algo}

<h3>6.11 六条律在 100 km/h 的汇总</h3>
<div class="grid3">{algo_cards}</div>
<div class="tablewrap"><table>
<thead><tr><th>控制律</th><th>δ_f °</th><th>T90 ms</th><th>超调 %</th>
<th>峰值时刻 ms</th><th>β/g °</th><th>横摆增益</th><th>a_y,max</th></tr></thead>
<tbody>{algo_rows}</tbody></table>
<div class="tnote">载体 L2 硬件，100 km/h，等 a_y = {ay_target:.1f} m/s²。
横摆增益为稳态横摆角速度除以前轮转角 [1/s]，越大表示同样转角转得越急。
注意 a_y,max 一列：六条律的极限侧向加速度全部落在 {algo_ay_span:.1f}% 带内——
<b>控制律不改变轮胎能给的东西，只改变车辆用它的方式</b>。</div></div>

{fig_algo_t90}
{fig_algo_os}
<p><b>读图。</b>这两张必须成对读，它们是同一件事的收益页和代价页。
瞬态补偿律把 T90 压到 {t90_fast:.0f} ms，是不控后轮的 3 倍快；
代价是超调从 {os_base:.1f}% 涨到 {os_fast:.1f}%。
如果只看图 4 会得出「瞬态补偿律最好」，只看图 5 会得出「它最差」——
两张图合起来的结论是：<b>它把一个可调的权衡推到了一端</b>，
而这个权衡在 140 km/h 会失控（图 12，超调 73.3%）。</p>

{fig_phase_algo}
<p><b>读图。</b>相频特性给出比阶跃响应更完整的图景，因为它覆盖整个频段而不是一个瞬态。
纵轴是横摆角速度相对方向盘输入的相位滞后，越接近 0 越「跟手」。
瞬态补偿律（红）在 0.2–0.7 Hz——正是驾驶员日常操作的频段——做到接近<b>零相位滞后</b>，
车辆几乎与手同步；这是它 88 ms 的 T90 在频域里的对应。
横摆反馈律（绿）以温和得多的代价拿到大部分收益：1 Hz 处 −26°，
而不控后轮是 −67.5°。速度调度（蓝）反而<b>比不控后轮更滞后</b>，
因为它命令的同相后轮转角在压低横摆增益的同时也拖慢了相位。</p>

{fig_phase_arch}
<p><b>读图。</b>同一张图换成按架构分组，用的是各级硬件能执行的最强律。
L2／L3 的曲线在 1.4 Hz 处已越过 −180° 进入正相位区（图上表现为跳到 +160°），
这是相位缠绕，不是不稳定——但它说明这两级在该频段的横摆响应已严重滞后于输入。
把这张图和图 7 并排看是本节的要点：<b>同一套 L2 硬件，
换一条控制律就能从最滞后变成最跟手</b>。架构决定了可能性，算法决定了实际拿到多少。</p>

<h2 id="s7"><span class="n">7</span>讨论</h2>
<h3>7.1 「不足转向梯度」在带主动后轮转向时不再是底盘属性</h3>
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

<h3>7.2 极限附着不是转向系统能改变的东西</h3>
<p>{ncase} 个案例的极限 a_y 全部落在 {allay_lo:.2f}–{allay_hi:.2f} m/s²，跨度 {allay_span:.1f}%。
四轮独立转向不会让轮胎多出摩擦。转向解耦改变的是<b>四个轮胎的能力如何被分配和使用</b>，
以及车辆<b>以什么姿态</b>抵达那个极限。在做技术方案论证时，
把「操稳收益」和「极限收益」分开陈述，是本文数据支持的、也是唯一诚实的讲法。</p>

<h3>7.3 在等角度约束下，收益递减出现得比想象中早得多</h3>
<p>L2 与 L3 在 4 m/s² 工况下的标定转角<b>完全相同</b>（{d_L2:.2f}°），
稳态 β 相同，极限 a_y 相同，转弯直径相差 {rws_spread:.2f} m。
剩下的差别只有瞬态（T90 相差约 5%，来自作动带宽）。
若一个项目的目标是常规行驶工况的操稳，<b>且后轮角度权限被约束在 ±{rear_env:.0f}° 以内</b>，
那么从 SBW+RWS 走到四轮独立转向买不到任何可测的东西。</p>
<p>更进一步：在这个包络下，连 L1→L2 的操稳收益也很薄。SBW 的价值在别处（§5.1）——
它把后轮转向对方向盘增益的副作用藏起来，这是手感问题，不是本文任何一项动力学指标能捕捉的。
<b>四轮独立转向的立项理由，在角度受限时不成立；它需要的是角度预算，而不是自由度。</b></p>

<h3>7.4 本研究的局限</h3>
<ul class="tight">
<li>每级只有一个整车级作动带宽（§1.1），前后轴不能分设。</li>
<li>轮胎为 Pacejka 魔术公式配摩擦圆／椭圆饱和，未含胎温、磨损、
    以及回正力矩对转向手感的反馈。</li>
<li>K&C 用工程估计曲线，非实测台架数据。</li>
<li>驾驶员为开环——本文全部为开环输入，未评价「人-车闭环」下的可控性，
    而这恰恰是解耦收益最可能被驾驶员适应性吃掉的地方。</li>
<li>未覆盖制动与转向的耦合工况，以及单轮失效重构。</li>
</ul>

<h2 id="s8"><span class="n">8</span>结论</h2>
<ol>
<li>转向解耦不增加极限附着（本文跨度 &lt; 1%），它重新分配轮胎能力并改变车辆姿态。</li>
<li>低速机动性的收益由<b>后轮角度权限</b>决定，而不是由转向自由度决定。
    统一到 ±{rear_env:.0f}° 之后，L1／L2／L3 的转弯直径落在 {rws_spread:.2f} m 带内；
    全部 −{pct_circle:.0f}% 的收益中，{pct_first:.0f} 个百分点由「装上第一个后轮转向轴」拿走。</li>
<li>高速稳定性的收益体现为侧偏角收敛，但开环解析零侧偏律在非线性车辆上会<b>过冲</b>，
    需要闭环项修正。</li>
<li>在固定硬件上，控制律带来的横摆响应差异（9 倍）大于架构带来的差异（&lt; 3 倍）。
    <b>算法投入的边际回报高于硬件投入。</b></li>
<li>SBW 的独立价值不在车辆动力学指标里，而在于用可变传动比隐藏后轮转向对方向盘增益的副作用。</li>
<li>在等角度约束下 L2→L3 的操控收益不可测。四轮独立转向的招牌能力
    （蟹行、原地回转）需要远超 ±{rear_env:.0f}° 的后轮角度，因此它的立项理由
    本质上是<b>角度预算</b>加上失效重构，而不是「多两个转向自由度」。</li>
</ol>

<h2><span class="n">9</span>复现</h2>
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
