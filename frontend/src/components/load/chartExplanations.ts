// 教科书风格的原理讲解：每张图都按「现象 → 直觉解释 → 物理推导 → 公式 → 边界」结构。
// 公式用 LaTeX，包在 $...$ / $$...$$ 里由 KaTeX 在弹窗里渲染。
// SVG 图示用纯文本嵌入。

import type { ExplanationContent } from "./ChartBox";

const svgKingpinDiagram = `
<svg viewBox="0 0 360 220" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <defs>
    <marker id="kpArrFy" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#34d399"/></marker>
    <marker id="kpArrFx" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#f59e0b"/></marker>
  </defs>
  <rect x="0" y="0" width="360" height="220" fill="#0f172a" rx="6"/>
  <line x1="20" y1="170" x2="340" y2="170" stroke="#334155" stroke-dasharray="4 4"/>
  <text x="22" y="185" fill="#64748b" font-size="10">地面</text>
  <line x1="200" y1="40" x2="170" y2="170" stroke="#60a5fa" stroke-width="2" stroke-dasharray="6 3"/>
  <text x="206" y="48" fill="#60a5fa" font-size="11">主销 KP 轴</text>
  <text x="170" y="36" fill="#60a5fa" font-size="10">ε caster</text>
  <ellipse cx="190" cy="170" rx="22" ry="5" fill="#94a3b8" opacity=".7"/>
  <text x="220" y="173" fill="#94a3b8" font-size="10">接地印迹</text>
  <line x1="190" y1="170" x2="190" y2="120" stroke="#34d399" stroke-width="2" marker-end="url(#kpArrFy)"/>
  <text x="194" y="118" fill="#34d399" font-size="12">F_y 侧向力</text>
  <line x1="190" y1="170" x2="260" y2="170" stroke="#f59e0b" stroke-width="2" marker-end="url(#kpArrFx)"/>
  <text x="250" y="163" fill="#f59e0b" font-size="12">F_x 纵向力</text>
  <line x1="170" y1="170" x2="190" y2="170" stroke="#a78bfa" stroke-width="3"/>
  <text x="160" y="200" fill="#a78bfa" font-size="11">机械拖距 t = r·tan(ε) + scrub</text>
</svg>`;

const svgTwoCurves = `
<svg viewBox="0 0 380 240" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <rect x="0" y="0" width="380" height="240" fill="#0f172a" rx="6"/>
  <line x1="40" y1="200" x2="360" y2="200" stroke="#475569" stroke-width="1.5"/>
  <line x1="40" y1="200" x2="40"  y2="20"  stroke="#475569" stroke-width="1.5"/>
  <text x="345" y="218" fill="#94a3b8" font-size="11">δ_cmd</text>
  <text x="6"   y="20"  fill="#94a3b8" font-size="11">|τ|</text>
  <line x1="40" y1="60"  x2="360" y2="60" stroke="#f87171" stroke-width="1" stroke-dasharray="4 3"/>
  <text x="270" y="55" fill="#f87171" font-size="11">μ·F_z (饱和极限)</text>
  <!-- Ideal (red dashed): linear extrapolation past saturation -->
  <line x1="40" y1="200" x2="360" y2="20" stroke="#ef4444" stroke-width="2" stroke-dasharray="6 4"/>
  <text x="220" y="100" fill="#ef4444" font-size="11">悬架原生 τ_ideal（无饱和）</text>
  <!-- Actual (blue): saturates at μ·Fz -->
  <path d="M 40 200 Q 90 100 130 65 Q 200 55 360 75" stroke="#60a5fa" stroke-width="2.5" fill="none"/>
  <text x="200" y="148" fill="#60a5fa" font-size="11">轮胎裁后实际 τ（蓝实线）</text>
  <text x="55" y="222" fill="#fbbf24" font-size="10">两曲线分叉点 ≈ α_peak</text>
</svg>`;

const svgTireSaturation = `
<svg viewBox="0 0 380 240" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <rect x="0" y="0" width="380" height="240" fill="#0f172a" rx="6"/>
  <line x1="40" y1="200" x2="360" y2="200" stroke="#475569" stroke-width="1.5"/>
  <line x1="40" y1="200" x2="40"  y2="20"  stroke="#475569" stroke-width="1.5"/>
  <text x="345" y="218" fill="#94a3b8" font-size="11">α 侧偏角</text>
  <text x="6"   y="20"  fill="#94a3b8" font-size="11">|F_y|</text>
  <line x1="40" y1="60"  x2="360" y2="60" stroke="#f87171" stroke-width="1" stroke-dasharray="4 3"/>
  <text x="282" y="55" fill="#f87171" font-size="11">μ·F_z (饱和极限)</text>
  <path d="M 40 200 L 100 60 L 360 60" stroke="#94a3b8" stroke-width="1.5" fill="none" opacity=".5"/>
  <text x="146" y="100" fill="#94a3b8" font-size="11" opacity=".6">Linear 模型（硬剪）</text>
  <path d="M 40 200 Q 90 100 130 65 Q 200 55 360 75" stroke="#34d399" stroke-width="2.5" fill="none"/>
  <text x="200" y="148" fill="#34d399" font-size="11">Pacejka（柔和饱和 + 后峰下降）</text>
  <circle cx="130" cy="65" r="3.5" fill="#fbbf24"/>
  <text x="100" y="55" fill="#fbbf24" font-size="10">α_peak ≈ 3°</text>
</svg>`;

const svgLinkage = `
<svg viewBox="0 0 400 220" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <defs>
    <marker id="lnkArrFx" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#fbbf24"/></marker>
  </defs>
  <rect x="0" y="0" width="400" height="220" fill="#0f172a" rx="6"/>
  <line x1="40" y1="170" x2="360" y2="170" stroke="#94a3b8" stroke-width="3"/>
  <text x="6" y="174" fill="#94a3b8" font-size="11">齿条</text>
  <circle cx="120" cy="60" r="6" fill="#60a5fa"/>
  <text x="80"  y="55" fill="#60a5fa" font-size="11">主销 K</text>
  <line x1="120" y1="60" x2="180" y2="120" stroke="#22d3ee" stroke-width="3"/>
  <text x="124" y="100" fill="#22d3ee" font-size="11">梯形臂 r</text>
  <line x1="180" y1="120" x2="260" y2="170" stroke="#fbbf24" stroke-width="3"/>
  <text x="216" y="148" fill="#fbbf24" font-size="11">横拉杆 c</text>
  <circle cx="260" cy="170" r="5" fill="#fbbf24"/>
  <circle cx="180" cy="120" r="5" fill="#22d3ee"/>
  <path d="M 120 78 A 18 18 0 0 1 138 70" stroke="#a78bfa" fill="none" stroke-width="1.5"/>
  <text x="138" y="82" fill="#a78bfa" font-size="10">arm-tie</text>
  <path d="M 240 170 A 14 14 0 0 0 252 158" stroke="#a78bfa" fill="none" stroke-width="1.5"/>
  <text x="226" y="158" fill="#a78bfa" font-size="10">tie-rack</text>
  <line x1="285" y1="170" x2="310" y2="170" stroke="#fbbf24" marker-end="url(#lnkArrFx)"/>
  <text x="270" y="190" fill="#fbbf24" font-size="11">齿条移动</text>
</svg>`;

const svgEquilibrium = `
<svg viewBox="0 0 380 240" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <rect x="0" y="0" width="380" height="240" fill="#0f172a" rx="6"/>
  <line x1="40" y1="120" x2="360" y2="120" stroke="#475569" stroke-width="1.5"/>
  <line x1="190" y1="20" x2="190" y2="220" stroke="#475569" stroke-width="1.5"/>
  <text x="345" y="138" fill="#94a3b8" font-size="11">δ_cmd</text>
  <text x="200" y="30" fill="#94a3b8" font-size="11">F_rack</text>
  <path d="M 40 60 Q 190 130 360 180" stroke="#34d399" stroke-width="2.5" fill="none"/>
  <circle cx="218" cy="120" r="5" fill="#fbbf24"/>
  <line x1="218" y1="120" x2="218" y2="170" stroke="#fbbf24" stroke-dasharray="3 3"/>
  <text x="222" y="180" fill="#fbbf24" font-size="11">δ_eq（零点）</text>
  <line x1="190" y1="120" x2="190" y2="98" stroke="#f87171" stroke-width="2"/>
  <text x="100" y="92" fill="#f87171" font-size="11">δ=0 时残余齿条力</text>
</svg>`;

const svgPower4WIS = `
<svg viewBox="0 0 380 220" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <defs>
    <marker id="wis4ArrLeft" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#34d399"/></marker>
    <marker id="wis4ArrRight" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="6" markerHeight="6" orient="auto">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#f59e0b"/></marker>
  </defs>
  <rect x="0" y="0" width="380" height="220" fill="#0f172a" rx="6"/>
  <rect x="120" y="50" width="140" height="110" fill="#1e293b" stroke="#475569" stroke-width="1.5" rx="8"/>
  <text x="178" y="110" fill="#64748b" font-size="11">车身</text>
  <rect x="100" y="40" width="20" height="30" fill="#60a5fa"/>
  <text x="80"  y="36" fill="#60a5fa" font-size="11">FL</text>
  <rect x="260" y="40" width="20" height="30" fill="#60a5fa"/>
  <text x="282" y="36" fill="#60a5fa" font-size="11">FR</text>
  <rect x="100" y="140" width="20" height="30" fill="#a78bfa"/>
  <text x="80"  y="190" fill="#a78bfa" font-size="11">RL</text>
  <rect x="260" y="140" width="20" height="30" fill="#a78bfa"/>
  <text x="282" y="190" fill="#a78bfa" font-size="11">RR</text>
  <line x1="110" y1="55" x2="80"  y2="55" stroke="#34d399" stroke-width="2" marker-end="url(#wis4ArrLeft)"/>
  <line x1="110" y1="155" x2="80" y2="155" stroke="#34d399" stroke-width="2" marker-end="url(#wis4ArrLeft)"/>
  <line x1="270" y1="55" x2="310" y2="55" stroke="#f59e0b" stroke-width="2" marker-end="url(#wis4ArrRight)"/>
  <line x1="270" y1="155" x2="310" y2="155" stroke="#f59e0b" stroke-width="2" marker-end="url(#wis4ArrRight)"/>
  <text x="22" y="105" fill="#34d399" font-size="11">左侧 ΣFy_左</text>
  <text x="312" y="105" fill="#f59e0b" font-size="11">ΣFy_右</text>
</svg>`;

const svgSensitivity = `
<svg viewBox="0 0 380 220" xmlns="http://www.w3.org/2000/svg" class="explain-svg">
  <rect x="0" y="0" width="380" height="220" fill="#0f172a" rx="6"/>
  <line x1="40" y1="180" x2="360" y2="180" stroke="#475569" stroke-width="1.5"/>
  <line x1="40" y1="180" x2="40"  y2="20"  stroke="#475569" stroke-width="1.5"/>
  <text x="320" y="200" fill="#94a3b8" font-size="11">参数</text>
  <text x="6"   y="20"  fill="#94a3b8" font-size="11">δ_eq</text>
  <line x1="40" y1="100" x2="360" y2="100" stroke="#475569" stroke-dasharray="3 3"/>
  <text x="45"  y="96" fill="#64748b" font-size="10">0°</text>
  <line x1="40" y1="150" x2="360" y2="40" stroke="#34d399" stroke-width="2.5"/>
  <text x="200" y="80" fill="#34d399" font-size="11">敏感度 = 斜率</text>
  <circle cx="100" cy="130" r="4" fill="#fbbf24"/>
  <circle cx="300" cy="60"  r="4" fill="#fbbf24"/>
  <text x="80" y="148" fill="#fbbf24" font-size="11">min</text>
  <text x="280" y="50" fill="#fbbf24" font-size="11">max</text>
</svg>`;

// 公用「红线 vs 蓝线」段落，主图（τ/F_rack/Fy）都共享
const dualCurveSection = {
  heading: "★ 两条曲线的含义",
  body: `<p>每张主图上有<strong>两条曲线</strong>：</p>
<ul>
<li><span style="color:#60a5fa">■</span> <strong>蓝色实线（actual）</strong>：经 Pacejka 平滑饱和的实际值，受 $|F| \\le \\mu F_z$ 限制，进入饱和区后会平台化；</li>
<li><span style="color:#ef4444">■</span> <strong>红色虚线（ideal）</strong>：把"轮胎能拿到任意大小的力"代入悬架几何后的"<strong>本征</strong>"线性外推响应，<strong>不裁 friction ellipse</strong>，把悬架/主销/拉杆几何固有的趋势暴露出来。</li>
</ul>
<p>两条曲线在<strong>线性区重合</strong>（小 $\\alpha$ 时 Pacejka 与线性相等），在<strong>饱和区分叉</strong>。
读图时：</p>
<ul>
<li>看蓝线的<strong>峰值/平台</strong> → 知道电机/齿条/接地极限受力是多少；</li>
<li>看红线的<strong>斜率</strong> → 知道悬架几何对小转角时的回正灵敏度（这<strong>不是</strong>定值，随车速 $\\propto v^2$ 增长）；</li>
<li>看两线<strong>分叉幅度</strong> → 知道轮胎"压制"了多少潜在响应。</li>
</ul>`,
  figure: svgTwoCurves,
};

// W1 单独一节：解释 bicycle coupling 把速度依赖引入 α 的物理来源
const bicycleSection = {
  heading: "2. 为什么车速会这样改变曲线形状 —— bicycle coupling",
  body: `<p>这是 v0.7.5 加入的核心物理升级。原本的台架口径 $\\alpha_i = -\\delta_i$（速度被约掉）<strong>不符合实车</strong>：
当 FL 转 $\\delta$ 而其他三轮锁 0 时，车身一定会发展出<strong>侧偏 $\\beta$</strong> 和<strong>横摆角速度 $r$</strong> 才能达到稳态平衡。</p>
<p>解 2×2 稳态 bicycle 方程：</p>
$$\\begin{cases} \\sum_i F_{y,i} = m V r \\\\ \\sum_i x_i F_{y,i} = 0 \\end{cases}$$
<p>其中 $F_{y,i} = -c_\\alpha(F_{z,i}) \\cdot \\alpha_i$ 且 $\\alpha_i = \\beta + r\\,x_i/V - \\delta_i$。
对 LS9 对称底盘 + 单 FL 转 $\\delta$ 的情形解析解为：</p>
$$r = \\frac{\\delta\\,V}{2L}, \\quad \\beta = \\frac{\\delta}{4}\\left(1 - \\frac{mV^2}{2 c_\\alpha L}\\right)$$
$$\\boxed{\\;\\alpha_{FL} = -\\delta \\cdot \\left(\\frac{1}{2} + \\frac{mV^2}{8 c_\\alpha L}\\right)\\;}$$
<p>所以斜率增益是 $(1/2 + mV^2/(8c_\\alpha L))$：</p>
<table style="font-family:ui-monospace,monospace;font-size:13px;border-collapse:collapse;margin:8px 0">
<tr><th style="padding:4px 12px;border-bottom:1px solid #475569;text-align:right">V (km/h)</th><th style="padding:4px 12px;border-bottom:1px solid #475569;text-align:right">增益</th><th style="padding:4px 12px;border-bottom:1px solid #475569;text-align:right">饱和 δ</th></tr>
<tr><td style="text-align:right;padding:2px 12px">0</td><td style="text-align:right;padding:2px 12px">0.50</td><td style="text-align:right;padding:2px 12px">~5.7°</td></tr>
<tr><td style="text-align:right;padding:2px 12px">36</td><td style="text-align:right;padding:2px 12px">0.60</td><td style="text-align:right;padding:2px 12px">~4.75°</td></tr>
<tr><td style="text-align:right;padding:2px 12px">108</td><td style="text-align:right;padding:2px 12px">1.36</td><td style="text-align:right;padding:2px 12px">~2.1°</td></tr>
<tr><td style="text-align:right;padding:2px 12px">180</td><td style="text-align:right;padding:2px 12px">2.89</td><td style="text-align:right;padding:2px 12px">~1.0°</td></tr>
</table>
<p>所以高速下：<strong>线性区收缩</strong>（饱和早）、<strong>线性斜率变大</strong>（同 $\\delta$ 拿到更多 α）。
这两个现象都是 4WIS 工程师熟悉的常识，台架口径模型只是把它们藏起来了。</p>`,
};

export const EXPLANATIONS: Record<string, ExplanationContent> = {
  torque: {
    title: "转向阻力矩 τ vs δ —— 电机要克服的总力矩",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴是命令车轮转角 $\\delta_{cmd}$（度），纵轴是该轮主销总阻力矩 $\\tau$（N·m）。曲线大致呈"S 形"：
中央近线性，两侧逐渐进入<strong>饱和平台</strong>。</p>
<p><strong>不同剖面车速差异巨大</strong>：</p>
<ul>
<li><strong>线性区斜率</strong>随车速 $\\propto v^2$ 增长（v=10→200 km/h 增长约 5.8 倍）；</li>
<li><strong>饱和发生点</strong>随车速向中央<strong>收缩</strong>（高速下 $\\delta = 1°$ 就快到饱和）；</li>
<li><strong>饱和平台高度</strong>随车速略下降（气动升力降 $F_z$）。</li>
</ul>
<p>这<strong>不是</strong>简单的"$\\mu F_z$ 平台变低"那么简单——根本原因是 v0.7.5+ 加入了 <strong>bicycle coupling</strong> 让单轮 sweep 不再假设车身锁直行（详见下方第 2 节）。</p>`,
      },
      dualCurveSection,
      bicycleSection,
      {
        heading: "2. 直觉：为什么蓝线会有「平台」",
        body: `<p>把车轮想成在地上滑的一块橡皮：稍微歪一点，地面对它的回正力随歪角线性增大；
歪到一定角度，橡皮已经在打滑，<strong>地面顶它的力到了 $\\mu F_z$ 这个天花板就不再涨了</strong>。
继续歪只是让它滑得更厉害，并不会得到更大的力。这就是蓝线平台的物理本质 —— 轮胎进入饱和。</p>
<p>红线没有这个限制，所以一直按线性外推 —— 它对应的是"假设轮胎能拿出任意大的 $F_y$，几何会把它转换成多大的主销力矩"。</p>`,
        figure: svgTireSaturation,
      },
      {
        heading: "3. 物理推导",
        body: `<p>主销力矩 $\\tau$ 来自 4 项 (Reimpell §3.10)：</p>
$$\\tau = \\underbrace{F_y \\cdot (s + t_m + t_p)}_{\\text{侧向力 × 等效拖距}}
     + \\underbrace{F_x \\cdot s}_{\\text{纵向力 × scrub}}
     + \\underbrace{M_z}_{\\text{自回正}}
     + \\underbrace{F_z \\sin(KPI) \\cdot s \\sin\\delta}_{\\text{KPI 抬升}}$$
<p>其中 $t_m = r\\tan\\varepsilon$ 为机械拖距，$s$ 为主销偏置（scrub_radius），$t_p$ 为气胎拖距。</p>
<p>注：$F_y \\cdot (s + t_m + t_p)$ 是 Reimpell 的<strong>工程等效力臂</strong>写法，把 $F_y$ 通过倾斜的主销轴产生的一阶 3D 耦合折算成一个有效杠杆，严格 3D 推导会得到稍小一点的 $F_y \\cdot t_m$ 但要补 $F_z$ 的二阶项。EPS 选型行业用前者作标准。</p>
<p><strong>蓝线（actual）</strong>用 Pacejka 平滑饱和后的 $F_y, F_x$ —— 摩擦椭圆一次成型：</p>
$$\\left(\\frac{F_x}{\\mu F_z}\\right)^2 + \\left(\\frac{F_y}{\\mu F_z}\\right)^2 \\le 1$$
<p>其中 $F_y$ 用<strong>等效滑移角</strong>把 camber 折进去：$\\alpha_{eq} = \\alpha - \\dfrac{C_\\gamma \\gamma F_z}{c_\\alpha(F_z)}$；
$F_x$ 用<strong>等效滑移率</strong>把驱动力折进去：$\\kappa_{eq} = F_{x,drive}/c_\\kappa$。
这样 camber thrust 和 driving force 都通过同一个 Pacejka 共享 friction ellipse，避免之前"二次裁剪"造成的硬拐角。</p>
<p><strong>红线（ideal）</strong>用线性 demand（不裁）：</p>
$$F_{y,ideal} = -c_\\alpha(F_z) \\cdot \\alpha + C_\\gamma \\gamma F_z + F_{y,parking}$$
<p>注意 $c_\\alpha(F_z) = c_{\\alpha,0}(F_z/F_{z,nom})^{0.8}$ —— 随<strong>动态 $F_z$</strong>软化，所以高速气动升力降 $F_z$ 时<strong>红线斜率也跟着软化</strong>，不是定值。</p>`,
        figure: svgKingpinDiagram,
      },
      {
        heading: "4. 工程价值",
        body: `<p>$|\\tau|$ 的<strong>峰值</strong>决定 → 电机最大输出扭矩需求（看蓝线，因为电机只需要克服实际力）；
<strong>$\\delta=0$ 处的 $\\tau$</strong> 决定 → 直行位电机持续保持力矩；
<strong>过零点斜率</strong>反映 → 转向回正灵敏度（看红线更能反映悬架本征行为）。</p>`,
      },
    ],
  },

  rackForce: {
    title: "齿条力 F_rack vs δ —— 把主销力矩翻译成齿条受力",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴齿条轴向力 $F_{rack}$（N）。形状和 $\\tau$ 高度相似，
但是 $|F_{rack}|$ 比 $|\\tau|$ 数值高很多（典型 60-200×），因为梯形臂 $r \\approx 0.15$ m 是力臂的倒数关系。
车速越高，线性区斜率越陡、饱和发生得越早——同 $\\tau$ 一样来自 bicycle coupling。</p>`,
      },
      dualCurveSection,
      bicycleSection,
      {
        heading: "2. 物理推导",
        body: `<p>梯形机构本质是一个 4 连杆：主销转动 $\\delta$ → 梯形臂转动 → 拉动横拉杆 → 推齿条。
力的关系：</p>
$$F_{rack} = \\frac{\\tau}{r \\cdot \\eta_{linkage}}$$
$$\\eta_{linkage} = |\\sin(\\theta_{arm{\\text -}tie})| \\cdot \\cos(\\theta_{tie{\\text -}rack}) \\cdot \\eta_{rack}$$
<p>当 $\\delta = 0$ 时 arm-tie 接近 $90°$（高效），$\\eta$ 最大；当 $\\delta$ 偏离零位很多时 arm-tie 偏离 $90°$（拉杆和梯形臂越接近共线），<strong>$\\eta$ 下降很快</strong>，相同的 $\\tau$ 需要更大的 $F_{rack}$。这就是为什么 $|F_{rack}|$ 曲线两端比 $|\\tau|$ 曲线两端"翘"得更厉害。</p>`,
        figure: svgLinkage,
      },
      {
        heading: "3. 工程价值",
        body: `<p>$|F_{rack}|$ 的<strong>峰值</strong>决定 → 齿条受力上限，对滚珠丝杠/齿轮选型至关重要；
$\\delta=0$ 处的 $F_{rack}$ 决定 → 直行位齿条恒载（即"电机零输出"的关键）；
<strong>两侧的非对称性</strong>反映 → 梯形机构本身的几何不对称（一般来说应该对称）。</p>
<p>注意：电机扭矩需求 $\\tau_{motor} = F_{rack} \\cdot r_p / (i \\cdot \\eta)$，其中 $r_p$ 是齿轮节圆半径，$i$ 是减速比。</p>`,
      },
    ],
  },

  sideForce: {
    title: "单轮侧向力 Fy_body vs δ —— 这一只轮子对车身横向的推力",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴 $F_{y,body}$（N）。这是该轮在<strong>车身坐标系 Y 方向</strong>的力分量。
轮胎力 $F_y$ 是在<strong>轮坐标系</strong>下，所以要做一次旋转：</p>
$$F_{y,body} = \\sin(\\delta) \\cdot F_x + \\cos(\\delta) \\cdot F_y$$
<p>同样有 bicycle coupling 的速度依赖：高车速下小 $\\delta$ 就能拿到大 $F_y$。</p>`,
      },
      dualCurveSection,
      bicycleSection,
      {
        heading: "2. 物理含义",
        body: `<p>在低速（停车）区域，主导力是<strong>接地斑扭转产生的静态横向力</strong>：</p>
$$F_{y,parking} = k_{lat} \\cdot \\mu F_z \\cdot \\tanh\\left(\\frac{\\delta}{\\delta_{sat}}\\right) \\cdot \\text{blend}_{lowspeed}(v)$$
<p>在中高速区域，主导力是<strong>轮胎侧偏力</strong>：$F_y \\propto -c_\\alpha \\alpha$，
受 friction circle 限制 $|F_y| \\le \\mu F_z$。同时 <strong>camber thrust</strong> 也贡献 $C_\\gamma \\gamma F_z$。</p>`,
      },
      {
        heading: "3. 工程价值",
        body: `<p>用来核对各驾驶模式的<strong>合力方向</strong>是否合理。比如：</p>
<ul>
<li>蟹行：四轮 $\\delta$ 相同 → $F_{y,body}$ 同号且数值相当 ✓</li>
<li>原地零半径：左右对称镜像，左轮 $F_{y,body}$ 与右轮 $F_{y,body}$ 异号 ✓</li>
<li>单轮强制：只有该轮非零，其它三轮 ≈ 0 ✓</li>
</ul>`,
      },
    ],
  },

  equilibrium: {
    title: "零输出自然转角 δ_eq —— 单轮台架口径下的齿条零点",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴是车速 $v$（km/h），纵轴是该轮的 $\\delta_{eq}$（°）。这个量回答的问题是：
<strong>"假定车身严格直行，单个轮子绕主销自由时，齿条力等于 0 的角度在哪里？"</strong></p>`,
      },
      {
        heading: "2. 物理口径（v0.7.5 升级后）",
        body: `<p>从 v0.7.5 开始，单轮 sweep 已经包含了<strong>稳态 bicycle coupling</strong>：</p>
<ul>
<li>选中的那一只轮按命令角 $\\delta$ 转动，其它三轮锁 0；</li>
<li>车身按 2×2 稳态 bicycle 方程 $\\sum F_y = m V r,\\ \\sum x_i F_{y,i} = 0$ 解出 $(\\beta, r)$；</li>
<li>每个轮的实际 $\\alpha_i = \\beta + r \\cdot x_i/V - \\delta_i$ 反应到 Pacejka 拿到真实力；</li>
<li>$F_{rack}$ 由真实 $\\tau$ 经梯形机构得到，零交点就是 $\\delta_{eq}$。</li>
</ul>
<p>所以 $\\delta_{eq}(v)$ 这条曲线<strong>现在反映的是稳态弯道平衡下作动器需要的命令偏置</strong>，比 v0.7.4 的"车身锁直行"口径物理意义更强。</p>
<p>⚠ <strong>但仍然不是真"驾驶员放手"自由稳态</strong>。真自由态意味着所有四个作动器都不出力，
这是 over-determined 系统（4 个力矩平衡方程 + 整车 3-DOF），需要 v0.8 完整求解器。本图<strong>只算了选中那只轮</strong>的零作动条件。</p>`,
        figure: svgEquilibrium,
      },
      {
        heading: "3. 数学定义",
        body: `<p>$$\\delta_{eq}(v) := \\arg\\min_\\delta |F_{rack}(\\delta, v)|_{\\text{single-wheel}}$$
其中 $F_{rack}(\\delta, v)|_{\\text{single-wheel}}$ 是上面说的台架口径下的齿条力曲线。
"$\\arg\\min$"取距 0 最近的零交点；如果 $F_{rack}$ 不变号（罕见，重度饱和时），回退到 $\\tau_{steer}$ 的零点。</p>`,
      },
      {
        heading: "4. 为什么 δ_eq 还会随车速漂",
        body: `<p>在单轮台架口径下，让 $\\delta_{eq}$ 不为 0 的偏置源有：</p>
<ul>
<li><strong>静态 toe</strong>：出厂前轮就指向内侧约 0.1°；这是<strong>速度无关</strong>偏置；</li>
<li><strong>外倾 camber</strong>：产生 $F_y = C_\\gamma \\gamma F_z$（α=0 也有）；随 $F_z$ 速度依赖；</li>
<li><strong>驱动力 $F_x$</strong>：维持车速需要 $F_x = C_{rr} m g + \\frac{1}{2}\\rho C_d A v^2$，经主销 scrub 产生力矩；<strong>随 $v^2$ 增长</strong>；</li>
<li><strong>气动升力</strong>：$F_{lift} = \\frac{1}{2}\\rho C_l A v^2$ 改变 $F_z$，<strong>μ·Fz 上限和 $c_\\alpha(F_z)$ 都随车速变</strong>。</li>
</ul>
<p>所以 $\\delta_{eq}(v)$ 这条曲线告诉你：作动器在不同车速下需要持续克服的静态保持力，**不是定值**。这就是这个图的真正工程价值。</p>`,
      },
      {
        heading: "5. 工程价值",
        body: `<p><strong>电机选型</strong>：在 $\\delta_{cmd}=0$ 上维持直行，电机需要持续输出 $F_{rack}(0) \\cdot r_p / (i \\eta)$ 的扭矩；不同车速下这个量不同，<strong>最大值</strong>决定电机静态扭矩需求。</p>
<p><strong>底盘标定</strong>：通过敏感度面板可以反推应该调哪些参数（toe / camber / scrub）让 $\\delta_{eq}$ 落在期望位置。</p>
<p><strong>不要</strong>用这个图反推"驾驶员实际方向盘感受到哪里"——那是整车闭环问题，需要 v0.8 整车耦合求解器。</p>`,
      },
    ],
  },

  efficiency: {
    title: "几何效率 η_linkage vs δ —— 梯形机构传力的「利用率」",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴是<strong>主销梯形 + 齿条传动</strong>的瞬时机械效率（0–1 之间）。
通常在 $\\delta=0$ 附近最高（≈ 0.8–0.9），在 $\\delta$ 接近最大转角时陡降到 0.5 以下。</p>
<p>注：这张图<strong>仅看蓝线</strong>，因为效率是纯几何量，与轮胎是否饱和无关。</p>`,
      },
      {
        heading: "2. 物理含义",
        body: `<p>$$\\eta_{linkage} = |\\sin(\\theta_{arm{\\text -}tie})| \\cdot \\cos(\\theta_{tie{\\text -}rack}) \\cdot \\eta_{rack}$$
<p>当 $\\theta_{arm{\\text -}tie} = 90°$ 时 $\\sin = 1$（机构臂力最大）；
当 $\\theta_{tie{\\text -}rack} = 0°$ 时 $\\cos = 1$（拉杆方向与齿条同向，分量损失最小）。</p>`,
        figure: svgLinkage,
      },
      {
        heading: "3. 为什么 δ 大时 η 会降",
        body: `<p>这是<strong>四连杆机构在极限位置的奇异性</strong>。
车轮转到极限角度时，梯形臂和拉杆逐渐共线，arm-tie 偏离 90°，杠杆比恶化。
所以同样的主销力矩 $\\tau$，需要更大的齿条力 $F_{rack}$，相当于"效率下降"。</p>
<p>底盘设计上希望 $\\eta \\ge 0.6$ 在工作转角范围内。如果某段 $\\eta < 0.3$，说明梯形几何选得不好（很可能 c/r 比例失调）。</p>`,
      },
    ],
  },

  armTie: {
    title: "梯形臂-拉杆夹角 arm-tie vs δ",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴 arm-tie 夹角（°）。理想梯形机构下，arm-tie ≈ 90° 在零位附近，转到极限位会下降。</p>
<p>这是纯几何曲线，与轮胎模型无关。</p>`,
      },
      {
        heading: "2. 物理意义",
        body: `<p>arm-tie 决定 $\\tau$ 经梯形臂传递到拉杆时的<strong>有效力臂</strong>。
$\\sin(\\theta_{arm{\\text -}tie})$ 越接近 1，力臂越接近于梯形臂全长 $r$；越接近 0 越接近共线，力臂趋零。</p>
<p>它和 $\\eta_{linkage}$ 是相关量（$\\eta \\propto |\\sin(\\theta_{arm{\\text -}tie})|$），但单独看夹角能直观判断机构是否接近奇异。</p>`,
        figure: svgLinkage,
      },
    ],
  },

  tieRack: {
    title: "拉杆-齿条夹角 tie-rack vs δ",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴 tie-rack 夹角（°）。这是横拉杆方向偏离齿条轴线的角度。纯几何，与轮胎模型无关。</p>`,
      },
      {
        heading: "2. 物理意义",
        body: `<p>拉杆要把推/拉力传给齿条，<strong>只有沿齿条轴线的分量有用</strong>，垂直分量被齿条轴承吃掉。
$\\cos(\\theta_{tie{\\text -}rack})$ 就是这个有效系数。$\\theta_{tie{\\text -}rack} = 0°$ 时分量全部有效；越大，"白干活"的部分越多。</p>
<p>典型设计在 5°–15° 之间。如果在零位就 $> 20°$，说明硬点几何选得不合理（h 偏置过大）。</p>`,
        figure: svgLinkage,
      },
    ],
  },

  wheelSideForce: {
    title: "四轮侧向力 @最高车速 —— 各轮对车身横推的贡献",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴是<strong>选定车速下</strong>四个轮各自的 $F_{y,body}$（N）。
四条曲线可以一眼看出"哪个轮是主推手 / 哪个轮在拖后腿"。</p>`,
      },
      {
        heading: "2. 几种典型 pattern",
        body: `<p><strong>单轮模式</strong>：只有选中的轮在动，其它三轮的 $F_{y,body}$ 是因为 camber thrust + 静态接地斑产生的小量。</p>
<p><strong>阿克曼模式</strong>：前两轮转向、后两轮直行。前轮 $F_{y,body}$ 异号（左轮正、右轮负，因为它们各自向圆心方向偏）。</p>
<p><strong>蟹行模式</strong>：四轮同向偏转，理论上 $F_{y,body}$ 同号，并且大致相等。</p>
<p><strong>零半径</strong>：左右各两轮镜像偏转，左侧两轮和右侧两轮 $F_{y,body}$ 异号。</p>`,
        figure: svgPower4WIS,
      },
    ],
  },

  sideForceSum: {
    title: "左右轮侧向合力 @最高车速 —— 左右失衡看这里",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴 $\\delta_{cmd}$，纵轴是车身坐标 Y 方向的<strong>左侧两轮合力 / 右侧两轮合力 / 总合力</strong>。</p>`,
      },
      {
        heading: "2. 物理含义",
        body: `<p>总合力 $\\Sigma F_y = F_{y,L} + F_{y,R}$ 决定车<strong>整车的横向加速度方向</strong>。
$\\Sigma F_y > 0$ → 车体被推向 +Y（左），$\\Sigma F_y < 0$ → 推向 -Y（右）。</p>
<p>左右分别看可以发现<strong>不对称</strong>：
比如蟹行模式下应该左 = 右，如果左右差太多说明左右轮 $\\delta$ 不同步或左右悬架/轮胎参数不对称。</p>`,
        figure: svgPower4WIS,
      },
      {
        heading: "3. 工程价值",
        body: `<p>检查策略的<strong>对称性</strong>。在直行（$\\delta=0$）时，对称模型下左右合力应都 ≈ 0；
如果有非零残量，说明 toe / camber / scrub 引入了左右偏置。
这也对应到 $\\delta_{eq}$ 图：非零 $\\delta_{eq}$ 的物理本质就是"左右轮合力在 $\\delta=0$ 处不平衡"。</p>`,
      },
    ],
  },

  sensitivity: {
    title: "δ_eq 敏感度 —— 哪个旋钮把零输出位置调得最快",
    sections: [
      {
        heading: "1. 看到什么",
        body: `<p>横轴是你选中的某一个底盘参数的取值，纵轴是该参数取该值时的 $\\delta_{eq}$（°）。<strong>斜率就是敏感度</strong>。</p>`,
        figure: svgSensitivity,
      },
      {
        heading: "2. 物理：每个参数的影响路径",
        body: `<ul>
<li><strong>toe</strong>：直接改变 $\\delta_{actual}$ → 改变 $\\alpha$ → 几乎线性影响 $\\delta_{eq}$。最强杠杆。</li>
<li><strong>$C_\\gamma$ / camber</strong>：camber thrust $C_\\gamma \\gamma F_z$ → 主销力矩 → 通过齿条几何转化成 $\\delta_{eq}$ 偏置。</li>
<li><strong>$C_{rr}$ / $C_d$</strong>：影响 $F_x = C_{rr} m g + \\frac{1}{2}\\rho C_d A v^2$，再经 scrub → 主销力矩。仅高速显著。</li>
<li><strong>$C_l$（升力）</strong>：改变 $F_z$ → 改变 $\\mu F_z$ → 改变 saturation 平台 → 通过 friction-circle 间接影响。仅高速显著。</li>
<li><strong>scrub_radius</strong>：直接放大 $F_x$ 的主销力矩贡献。</li>
</ul>`,
      },
      {
        heading: "3. 工程价值",
        body: `<p><strong>底盘标定</strong>：实测 $\\delta_{eq}$ 偏离设计 0.05°，看这张图可以反推 toe 要调多少。</p>
<p><strong>制造容差预算</strong>：toe 装配公差 ±0.1°，对应到 $\\delta_{eq}$ 的方差是多少？看曲线斜率乘以容差。</p>
<p><strong>方案对比</strong>：两个 $C_d$ 方案（0.30 vs 0.32）造成 $\\delta_{eq}$ 在 120 km/h 的差异是多少？把车速选到 120 km/h，扫 $C_d$ 范围即可。</p>`,
      },
    ],
  },
};
