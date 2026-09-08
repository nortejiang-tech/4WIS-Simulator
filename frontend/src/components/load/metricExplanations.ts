// Per-metric principle explanations for the real-time wheel panel and the
// KPI strip. Same content schema as chartExplanations so MetricInfo can
// render them with KaTeX out of the box.

import type { ExplanationContent } from "./ChartBox";
import { KINGPIN_FORMULA, KINGPIN_DERIVATION, RACK_FORMULA, LINKAGE_INDICATOR } from "../model/physicsExplanations";

// ───────── 实时四轮负载（per-wheel live data from WebSocket） ─────────

export const LIVE_DELTA: ExplanationContent = {
  title: "δ —— 实际车轮转角",
  sections: [
    {
      heading: "含义",
      body: `<p>当前这只轮的<strong>实际</strong>主销转角（已考虑作动器一阶滞后 $\\tau_{steer}$ 和速率限幅 $\\dot\\delta_{max}$）。
驾驶员/策略输出的 $\\delta_{cmd}$ 经作动器后才到这里。</p>`,
    },
    {
      heading: "怎么算",
      body: `<p>每个仿真步内：</p>
$$\\dot\\delta = \\text{clip}\\left(\\frac{\\delta_{cmd} - \\delta_{actual}}{\\tau_{steer}},\\ \\pm\\dot\\delta_{max}\\right)$$
$$\\delta_{actual}(t+dt) = \\delta_{actual}(t) + \\dot\\delta \\cdot dt$$
<p>所以你看到的 δ 永远<strong>滞后</strong>于 $\\delta_{cmd}$ 一点点，量级 $\\sim \\tau_{steer}$（默认 60ms）。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>对比 $\\delta$ 与 $\\delta_{cmd}$ 能看到作动器跟随误差——是 4WIS 控制器设计的关键观测量。
急转弯时大的跟随误差意味着作动器带宽不够；要么换电机，要么把策略 $\\delta_{cmd}$ 加滤波。</p>`,
    },
  ],
};

export const LIVE_TAU: ExplanationContent = {
  title: "τ —— 主销阻力矩",
  sections: [
    { heading: "力矩与符号", body: KINGPIN_FORMULA },
    { heading: "来源与适用范围", body: KINGPIN_DERIVATION },
  ],
};

export const LIVE_RACK: ExplanationContent = {
  title: "Rack —— 齿条力",
  sections: [
    { heading: "虚功与传动比", body: RACK_FORMULA },
    { heading: "几何边界", body: LINKAGE_INDICATOR },
  ],
};

export const LIVE_FY_BODY: ExplanationContent = {
  title: "Fy_body —— 单轮对车身横向的推力",
  sections: [
    {
      heading: "含义",
      body: `<p>该轮的轮胎力投影到<strong>车身坐标系 Y 方向</strong>的分量。正号 = 推车身往 +Y（左）；负号 = 推往右。</p>`,
    },
    {
      heading: "怎么算",
      body: `<p>轮胎力 $(F_x, F_y)$ 在<strong>轮坐标系</strong>下（沿轮滚动方向 / 垂直滚动方向）。
车身坐标系下的 Y 分量：</p>
$$F_{y,body} = \\sin(\\delta)\\cdot F_x + \\cos(\\delta)\\cdot F_y$$
<p>当 $\\delta = 0$（车轮直行）时 $F_{y,body} = F_y$；当 $\\delta = 90°$（轮垂直车身）时 $F_{y,body} = F_x$。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>各模式下四轮对车身的<strong>横向贡献分布</strong>可视化。
各轮力的符号取决于实际滑移、toe/camber 和瞬态状态，不能仅由策略名称判断。平稳左转时，前后轮的侧向合力通常均朝左。</p>`,
    },
  ],
};

export const LIVE_ETA: ExplanationContent = {
  title: "η —— 机构几何指标",
  sections: [{ heading: "含义与边界", body: LINKAGE_INDICATOR }],
};

export const LIVE_SIDE_LEFT: ExplanationContent = {
  title: "左侧 —— 左两轮 Fy_body 合力",
  sections: [
    {
      heading: "含义与算法",
      body: `<p>$\\Sigma F_{y,left} = F_{y,body,FL} + F_{y,body,RL}$</p>
<p>把左侧两轮（FL + RL）对车身 Y 方向的推力相加。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>结合右侧合力看车身整体的<strong>横向拉力对称性</strong>。
直行时对称 toe/camber 可产生左右反向力而合力为零；不应仅以左右单轮力不同判定故障。结合实际滑移、载荷与路面 μ 检查。</p>`,
    },
  ],
};

export const LIVE_SIDE_RIGHT: ExplanationContent = {
  title: "右侧 —— 右两轮 Fy_body 合力",
  sections: [
    {
      heading: "含义与算法",
      body: `<p>$\\Sigma F_{y,right} = F_{y,body,FR} + F_{y,body,RR}$</p>
<p>把右侧两轮（FR + RR）对车身 Y 方向的推力相加。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>同左侧。两者之差可能来自轮胎、载荷、定位角和路面差异；两者之和就是整车 $\\Sigma F_y$。</p>`,
    },
  ],
};

export const LIVE_SIDE_TOTAL: ExplanationContent = {
  title: "合力 —— 整车横向 ΣFy",
  sections: [
    {
      heading: "含义与算法",
      body: `<p>$\\Sigma F_y = F_{y,left} + F_{y,right}$</p>
<p>整车在车身 Y 方向<strong>横向加速度的来源</strong>（除去重力和气动横向力）。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>$\\Sigma F_y > 0$ → 车被推向左（+Y）；$< 0$ → 推向右。
直行时应当 ≈ 0；转弯时应当与曲率方向一致（向心力来源）。
非 0 的直行 $\\Sigma F_y$ 表示有横向不平衡，要么调 toe/camber，要么补偿策略。</p>`,
    },
  ],
};

export const LIVE_SOURCE: ExplanationContent = {
  title: "侧向力来源标签",
  sections: [
    {
      heading: "含义",
      body: `<p>面板右下角小字（<code>kinematic_estimate</code> / <code>tire_model</code> / <code>multibody</code>）告诉你<strong>这些 $F_y$ 数字是从哪里来的</strong>。</p>`,
    },
    {
      heading: "三种来源",
      body: `<ul>
<li><code>kinematic_estimate</code>：你选了<strong>运动学模型</strong>。运动学不算真实轮胎力，所以这里的 $F_y$ 是
<strong>从假设的车身侧向加速度反推</strong>的估算值（用整车质量 × 估算的 $a_y$ 再按 Fz 比例分配）。
仅供"知道大致量级"参考，<strong>不要</strong>当真做选型。</li>
<li><code>tire_model</code>：<strong>简化动力学模型</strong>。用 Pacejka/linear 轮胎模型算的真实 $F_y$。</li>
<li><code>multibody</code>：<strong>多体 14DOF 模型</strong>。带垂向悬架自由度的 $F_y$；是否更接近实车仍取决于标定与验证。</li>
</ul>
<p>右上角的"动力学模型"切换可改这个。</p>`,
    },
  ],
};

// ───────── KPI 6 项（从 sweep summary 取） ─────────

export const KPI_PEAK_RACK: ExplanationContent = {
  title: "峰值齿条力",
  sections: [
    {
      heading: "含义",
      body: `<p>整个 sweep 范围内（速度 × 转角 × 四轮）<strong>齿条轴向力绝对值的最大值</strong>。</p>`,
    },
    {
      heading: "怎么算",
      body: `$$F_{rack,peak} = \\max_{v,\\delta,wheel} |F_{rack}(v,\\delta,wheel)|$$
<p>来自 <code>summary.peak_abs_rack_force</code>。同时返回 <code>_at</code> 字段告诉你在哪个 $(v,\\delta,wheel)$ 组合下取到。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>这是<strong>齿条 / 滚珠丝杠 / 轴承的最大受力上限</strong>，直接对应零件选型。
该峰值只覆盖当前扫描范围；实际选型须另行验证载荷包络、疲劳、温升和设计裕量。</p>`,
    },
  ],
};

export const KPI_MIN_EFF: ExplanationContent = {
  title: "最低机构几何指标",
  sections: [{ heading: "含义与边界", body: LINKAGE_INDICATOR }],
};

export const KPI_MAX_UTIL: ExplanationContent = {
  title: "最大附着利用率",
  sections: [
    {
      heading: "含义",
      body: `<p>整个 sweep 范围内某轮<strong>合力幅值占 friction circle</strong> $\\mu F_z$ <strong>的最大百分比</strong>。
100% = 该轮已经把所有摩擦"用光了"，再加任何力都会滑。</p>`,
    },
    {
      heading: "怎么算",
      body: `$$u(v,\\delta) = \\frac{\\sqrt{F_x^2 + F_y^2}}{\\mu F_z}, \\quad u_{max} = \\max u$$
<p>来自 <code>summary.max_friction_utilization</code>。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>判断本次 sweep<strong>是否进入饱和区</strong>：</p>
<ul>
<li>$u_{max} < 0.8$ → 整段线性区，τ/F_rack 数据完全可信用作机构 sizing；</li>
<li>$0.8 \\le u_{max} < 0.95$ → 已经在饱和肩部，曲线变软但还可用；</li>
<li>$u_{max} > 0.95$ → 红色警告，曲线峰值由 $\\mu F_z$ 限制，要降转角/降 μ/降车速再扫一次。</li>
</ul>`,
    },
  ],
};

export const KPI_DELTA_EQ: ExplanationContent = {
  title: "零输出自然转角 δ_eq @ 剖面车速",
  sections: [
    {
      heading: "含义",
      body: `<p>在<strong>当前剖面车速</strong>下，让该轮齿条力穿过 0 的 $\\delta_{cmd}$ 值。
等价说法：电机不输出扭矩就能"挂住"的车轮命令角。</p>`,
    },
    {
      heading: "⚠ 这不是整车自由稳态",
      body: `<p>用的是<strong>单轮台架口径</strong>（车身锁直行 + 其余三轮锁 0），所以只反映:</p>
<ul>
<li>EPS/作动器要持续克服多少静态力矩；</li>
<li>底盘对齐参数（toe/camber/scrub）的偏置量级。</li>
</ul>
<p>真整车四轮联立自由稳态需要解 3-DOF 耦合方程，是 v0.8 计划项。详见 slot 4 的 $\\delta_{eq}$ 图的「原理」按钮。</p>`,
    },
    {
      heading: "怎么算",
      body: `<p>对选中车轮在该车速下的齿条力曲线 $F_{rack}(\\delta_{cmd})$ 做线性插值求零交点，取绝对值最小的那个。
后端 <code>per_speed_equilibrium[speed=v_profile].delta_eq_deg</code>。</p>`,
    },
  ],
};

export const KPI_RACK_AT_ZERO: ExplanationContent = {
  title: "0° 保持齿条力",
  sections: [
    {
      heading: "含义",
      body: `<p>在<strong>当前剖面车速</strong>下，当 $\\delta_{cmd} = 0$（命令角为直行位）时该轮齿条上的<strong>残余力</strong>。</p>`,
    },
    {
      heading: "怎么算",
      body: `<p>对选中车轮在该车速下的齿条力曲线在 $\\delta_{cmd} = 0$ 处插值：</p>
$$F_{rack,0}(v) := F_{rack}(\\delta_{cmd}=0,\\ v)$$
<p>后端 <code>per_speed_equilibrium[speed=v_profile].rack_at_zero</code>。</p>`,
    },
    {
      heading: "工程价值",
      body: `<p>这是<strong>"直行位电机要持续输出多少力"</strong>的直接读数。物理来源是 toe + camber thrust + driving force × scrub：</p>
$$F_{rack,0} \\propto \\frac{C_\\gamma\\gamma F_z(s+t_m) - c_\\alpha\\cdot\\text{toe}\\cdot(s+t_m) + F_{x,drive}\\cdot s}{L_{arm}\\cdot \\eta}$$
<p>非 0 的 $F_{rack,0}$ 直接转化为电机的静态发热电流，影响热设计。LS9 默认下约 -100 ~ -140 N，约对应 0.2 N·m 持续电机扭矩，可接受。</p>`,
    },
  ],
};
