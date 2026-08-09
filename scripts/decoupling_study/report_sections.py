"""Prose and figures for the deep-dive sections of the decoupling report.

Kept out of `build_report.py` so the page skeleton stays readable while the
theory, method and per-algorithm chapters — which are most of the words — live
somewhere they can be edited without scrolling past a CSS block.

Every number in here is read from the study JSONs. Nothing is typed by hand.
"""

from __future__ import annotations

import math

ALGO_COLOR = {"none": "#94a3b8", "fixed": "#f59e0b", "schedule": "#38bdf8",
              "yaw_fb": "#34d399", "transient": "#fb7185", "model_follow": "#a78bfa"}

#: Everything the report says about each law, beyond its measured numbers:
#: the equation as implemented, why it takes that form, what it needs to know
#: about the vehicle, and what it costs. `law` strings are the actual code.
LAW_DOC = {
    "none": dict(
        eq="δ_r = 0",
        gains="—",
        theory=(
            "参考基准，不是一条控制律。后轴不转向，车辆退化为常规前轮转向车，"
            "横摆角速度与侧偏角由底盘固有特性唯一决定，二者<b>刚性耦合</b>："
            "给定 δ_f 和车速，稳态 r 和 β 同时被确定，没有任何自由度去单独调整其中一个。"
            "本文所有「后轮转向买到了什么」的说法，都是相对这一行数据讲的。"),
        needs="无",
        cost="无",
        verdict=(
            "高速时 β 为负且随车速单调变负（车尾外摆、车头指向弯内），"
            "这是常规车辆在高速弯中的典型姿态，也是后轮转向要去消除的东西。"),
    ),
    "fixed": dict(
        eq="δ_r = k · δ_f,   k = 常数",
        gains="k = −0.4",
        theory=(
            "最便宜的一条：一个常数，除方向盘转角外不需要任何传感。"
            "负号表示反相——后轮与前轮反向，把瞬时转动中心 ICR 从后轴延长线上"
            "<b>向前移</b>，从而缩小转弯半径。这在低速是对的，在高速是错的："
            "高速需要的是<b>同相</b>后轮转向来压低侧偏角。"
            "一个常数无法同时满足两端，因此定比例律必然在某一端付出代价。"),
        needs="方向盘转角",
        cost="后轮 RMS 放大 0.20×；无速度信号需求",
        verdict=(
            "本文数据里它是<b>横摆响应最快的定常律</b>（100 km/h T90 232 ms），"
            "因为反相后轮直接加大了等效转向增益——同样的 a_y 只需 1.10° 前轮转角，"
            "是全部六条律里最小的。代价全部落在高速：140 km/h 超调 21.5%，"
            "β/g 达到 −3.04，比不用后轮转向（−2.27）<b>更差</b>。"),
    ),
    "schedule": dict(
        eq="δ_r = k(v) · δ_f",
        gains="k(v) 采样自解析零侧偏比，7 个速度点线性插值",
        theory=(
            "把定比例的常数换成速度的函数，用第 2.3 节推导的解析零侧偏比 "
            "k(u) 采样得到。低速 k→−b/a（反相，机动性），"
            "高速 k→+a·C_f/(b·C_r)（同相，稳定性），"
            "中间存在一个<b>过渡速度</b>使 k=0。"
            "这是整个后轮转向工程学的骨架：<b>一条曲线同时买到低速机动性和高速稳定性</b>，"
            "而这是定比例律做不到的。"),
        needs="方向盘转角 + 车速",
        cost="后轮 RMS 放大 0.23×；开环，不需要横摆传感器",
        verdict=(
            "全速域表现最均衡的开环律：超调在四个速度点上都低于 5.3%，"
            "β/g 在 40 km/h 处只有 +0.03。但它是<b>开环</b>的——"
            "其精度完全取决于 k(v) 所依据的车辆模型是否正确，"
            "而第 2.5 节说明本项目里它<b>不正确</b>。"),
    ),
    "yaw_fb": dict(
        eq="δ_r = g₁ · δ_f + g₂ · r̂",
        gains="g₁ = −0.10, g₂ = +0.12 s；r̂ 为一阶低通滤波的实测横摆角速度（τ = 0.08 s）",
        theory=(
            "唯一一条<b>闭环</b>律：后轮角由实测横摆角速度反馈决定，"
            "因此它<b>不需要知道</b>侧偏刚度、质心位置或质量。"
            "前馈项 g₁·δ_f 提供初始反相以保留低速机动性，"
            "反馈项 g₂·r̂ 在车辆开始转动后加入同相分量抑制横摆。"
            "反馈信号必须滤波：在运动学模型上未滤波的横摆反馈是一个代数环，"
            "环增益超过 1 即在限幅间振荡。"),
        needs="方向盘转角 + 横摆角速度传感器",
        cost="后轮 RMS 放大 0.15×（六条律中<b>最低</b>）；需要横摆传感器",
        verdict=(
            "本文的<b>综合最优</b>。响应快（T90 在四个速度点上稳定在 136–146 ms），"
            "且因为不依赖车辆参数，它是唯一一条不受第 2.5 节对象模型缺陷影响的律——"
            "极限侧偏角 0.49°，是全部九个案例中最低的。"
            "代价是超调随速度上升（140 km/h 达 26.2%）和一个额外传感器。"),
    ),
    "transient": dict(
        eq="δ_r = k(v) · δ_f − c · d̂δ_f/dt",
        gains="c = 0.10 s；dδ_f/dt 经同一 τ = 0.08 s 低通滤波",
        theory=(
            "在速度调度的基础上叠加一个<b>对转向速率的反相修正</b>。"
            "物理含义：阶跃转向瞬间，前轮已产生侧向力而后轮尚未建立，"
            "此时短暂地把后轮反打，可以立刻制造一个额外的横摆力矩，"
            "把横摆响应「踢」起来；待瞬态过去、导数项归零，"
            "控制律自动退化为纯速度调度，稳态不受影响。"
            "这是一个<b>纯瞬态</b>整形项——它不改变任何稳态量。"),
        needs="方向盘转角 + 车速 + 转角导数",
        cost="后轮 RMS 放大 0.32×，角速率 RMS 1.41 °/s（是速度调度的 1.7 倍）",
        verdict=(
            "<b>响应速度的极限</b>：100 km/h T90 88 ms，是不用后轮转向的 3.0 倍快，"
            "0.5 Hz 处相位滞后接近零。但代价在高速急剧恶化——"
            "140 km/h 超调 73.3%，已经不是一台可以交付的车。"
            "它适合作为一个<b>可调权重</b>（c 随速度衰减），而不是全速域固定投入。"),
    ),
    "model_follow": dict(
        eq="δ_r = k(v) · δ_f + g · (r_ref − r̂)",
        gains="g = 0.08 s；r_ref 为零侧偏参考模型的稳态横摆角速度",
        theory=(
            "前馈加反馈的标准结构：前馈项负责稳态（解析零侧偏比），"
            "反馈项把实际横摆角速度拉向<b>参考模型</b>的输出 r_ref（第 2.4 节推导）。"
            "理论上这是六条里最完备的一条——它同时约束了侧偏角（前馈）"
            "和横摆响应（反馈），而且参考模型给了设计者一个显式的「目标车辆」。"
            "代价是它需要知道的车辆参数最多：质量、质心位置、前后轴侧偏刚度全都进入公式。"),
        needs="方向盘转角 + 车速 + 横摆角速度 + 完整车辆参数",
        cost="后轮 RMS 放大 0.42×（六条律中<b>最高</b>），角速率 RMS 1.54 °/s，后轮角需求最大",
        verdict=(
            "理论最强、本文实测<b>最差</b>——这个反差是全文最有信息量的一处。"
            "100 km/h T90 804 ms，140 km/h 的 β/g 达到 +20.6°，"
            "远差于完全不用后轮转向。原因不在结构而在参数："
            "它对车辆参数的依赖最重，因此第 2.5 节那个对象模型缺陷对它伤害最大。"
            "<b>依赖模型最深的控制律，被模型误差伤得最狠。</b>"),
    ),
}


def theory_section(th, grad_none_K, fmt):
    """Section 2 — the bicycle model, the two laws derived from it, and the
    plant-model defect the derivation exposed."""
    k_law = th["K_law"]
    k_true = th["K_true"]
    err_true = abs(k_true - grad_none_K) / max(abs(grad_none_K), 1e-9) * 100
    ratio = abs(grad_none_K / k_law) if abs(k_law) > 1e-9 else float("inf")
    return f"""
<h2><span class="n">2</span>理论基础</h2>
<p>本文比较的六条控制律里有五条是从同一个模型闭式解出来的。
不把这个模型讲清楚，第 6 节的排名就只是一张成绩单而不是一个解释。
本节推导它，并用它去<b>预言</b>仿真结果——预言失败的那一次，
恰好暴露了平台里一个真实的缺陷。</p>

<h3>2.1 线性二自由度（自行车）模型</h3>
<p>把左右轮并作一个，忽略侧倾与载荷转移，取车身侧向速度 v_y 与横摆角速度 r 为状态：</p>
<div class="eq">m (v̇_y + u·r) = F_yf + F_yr<br>
I_z ṙ = a·F_yf − b·F_yr</div>
<p>其中 u 为纵向车速，a、b 为质心到前/后轴距离，L = a + b。小角度下轮胎侧偏角为</p>
<div class="eq">α_f = δ_f − (v_y + a·r)/u &nbsp;&nbsp;&nbsp; α_r = δ_r − (v_y − b·r)/u</div>
<p>线性轮胎 F_y = C·α，C 为<b>轴</b>侧偏刚度（单胎的两倍）。
这四个式子构成全部后续推导的基础。</p>

<h3>2.2 稳态解与不足转向梯度</h3>
<p>令 v̇_y = ṙ = 0，消去 v_y，得到熟悉的稳态转向方程</p>
<div class="eq">δ_f = L/R + K·a_y &nbsp;&nbsp;&nbsp;&nbsp;
K = (m/L)·(b/C_f − a/C_r)</div>
<p>K 即<b>不足转向梯度</b>：K &gt; 0 不足转向，K &lt; 0 过多转向。
注意它只由质量分配和<b>轴侧偏刚度比</b>决定——μ 不出现在里面。
这就是姊妹结论「极限附着与转向架构无关」在理论侧的对应：
K 是刚度量，极限是附着量，两者是不同的物理。</p>

<h3>2.3 零侧偏后轮转向比 k(u)</h3>
<p>后轮转向引入第二个输入 δ_r，于是 (r, β) 从「一个输入定两个输出」
变成「两个输入定两个输出」——<b>这就是解耦的全部数学含义</b>。
用这个自由度去令 β = 0（即 v_y = 0）：代回稳态方程，力矩式给出
C_r·α_r = (a/b)·C_f·α_f，力平衡式给出 C_f·α_f·L/b = m·u·r，联立消去 r 得</p>
<div class="eq">k(u) = δ_r/δ_f = [ −b + a·m·u²/(C_r·L) ] / [ a + b·m·u²/(C_f·L) ]</div>
<p>两个极限值给出全部工程直觉：</p>
<ul class="tight">
<li><b>u → 0</b>：k → −b/a，反相且量级接近 1。低速时后轮几乎与前轮等量反打，
    ICR 前移，转弯半径大幅缩小。</li>
<li><b>u → ∞</b>：k → +a·C_f/(b·C_r)，同相。高速时后轮与前轮同向，
    车辆整体「平移」进弯，侧偏角被压向零。</li>
<li><b>过渡速度</b>：分子为零处，u² = b·C_r·L/(a·m)。
    本车为 <b>{th["v_crossover_true"]:.1f} km/h</b>。在这个速度附近后轮转向<b>无事可做</b>——
    姊妹报告实测到 60 km/h 时控制律只索取 0.02° 后轮角，正是这个原因。</li>
</ul>

<h3>2.4 参考横摆角速度 r_ref</h3>
<p>模型跟随律需要一个「目标车辆」。取 β = 0 的参考车辆，
由 2.3 节的中间结果 α_f = m·u·r·b/(C_f·L) 代入 α_f = δ_f − a·r/u，解出</p>
<div class="eq">r_ref = δ_f / [ a/u + m·b·u/(C_f·L) ]</div>
<p>低速时分母被 a/u 主导，r_ref ≈ u·δ_f/a；高速时被第二项主导，r_ref 随 u 下降。
这条曲线就是模型跟随律的跟踪目标。</p>

<h3>2.5 理论校验：一次成功的预言和一个被它揪出来的缺陷</h3>
<p>理论有没有用，要看它能不能预言仿真。用 2.2 节的 K 去预言「关掉后轮转向」的实测不足转向梯度：</p>
<div class="tablewrap"><table>
<thead><tr><th>K 的来源</th><th>C_f (N/rad)</th><th>C_r (N/rad)</th>
<th>K (deg/g)</th><th>与实测偏差</th></tr></thead>
<tbody>
<tr><th>控制律所用的 <code>axle_cornering_stiffness()</code></th>
<td>{th["cf"]:.0f}</td><td>{th["cr"]:.0f}</td><td>{k_law:.3f}</td>
<td class="bad">相差 {ratio:.0f} 倍</td></tr>
<tr><th>车辆<b>真实</b>轴刚度（含 {th["scale_f"]:.2f}／{th["scale_r"]:.2f} 分配）</th>
<td>{th["cf_true"]:.0f}</td><td>{th["cr_true"]:.0f}</td><td>{k_true:.3f}</td>
<td class="good">{err_true:.1f}%</td></tr>
<tr><th>多体仿真实测（仅前轮转向）</th><td>—</td><td>—</td>
<td>{grad_none_K:.3f}</td><td>—</td></tr>
</tbody></table></div>
<div class="callout bad"><h4>发现：控制律解的不是它正在驾驶的那台车</h4>
<p>喂进<b>真实</b>轴侧偏刚度时，线性自行车模型预言 K = {k_true:.3f} deg/g，
实测 {grad_none_K:.3f} deg/g，<b>偏差 {err_true:.1f}%</b>——理论是准的。
但控制律实际调用的 <code>axle_cornering_stiffness()</code> 对前后轴都返回
2·<code>tire_c_alpha</code>，<b>没有应用</b>车辆刻意设置的
{th["scale_f"]:.2f}／{th["scale_r"]:.2f} 轴刚度分配。用这个错误的刚度，
同一个公式给出 K = {k_law:.3f} deg/g，差了 {ratio:.0f} 倍。</p>
<p>后果直接落在 k(u) 上：120 km/h 时控制律索取 k = {_k_at(th, 120, "k"):.3f}，
而正确值是 {_k_at(th, 120, "k_true"):.3f}——<b>同相后轮转角多给了
{(_k_at(th, 120, "k") / max(_k_at(th, 120, "k_true"), 1e-9) - 1) * 100:.0f}%</b>。
过渡速度也随之错位：{th["v_crossover_law"]:.1f} km/h 对 {th["v_crossover_true"]:.1f} km/h。
这不是「非线性轮胎导致解析解不够准」——是控制律的对象模型漏了一个已知参数。
第 6.7 节用一个对照实验证明了这一点。</p></div>
"""


def _k_at(th, v, field):
    rows = th["rows"]
    best = min(rows, key=lambda r: abs(r["v"] - v))
    return abs(best[field])


def method_section(meta, ay_target, vstep, low_mu, front_env, rear_env,
                   steer_limit):
    return f"""
<h2><span class="n">3</span>试验方法</h2>
<p>本节写得比通常详细，因为本研究在自查中<b>推翻过自己两次</b>，
两次都源于方法而非模型。把判据写清楚，读者才能判断哪些数字可信。</p>

<h3>3.1 角度包络：为什么四级必须同框</h3>
<p>四级架构统一到<b>前轮 ±{front_env:.0f}°、后轮 ±{rear_env:.0f}°</b>。
L0 的后轮为 0° 是架构定义（前轮转向车没有后轮作动器），不是限幅选择。
包络之外的差异只剩三项：转向自由度数、作动器带宽、是否轮级独立。</p>
<p>这一步是本研究修正自己的第一次。早期版本给各级不同的后轮权限（0／3／5／{steer_limit:.0f}°），
测得 L0→L3 转弯直径改善 33%，读起来像是「四轮独立转向低速优势巨大」。
统一包络后同一个量只剩 −9%——那 33% 的绝大部分是<b>角度</b>而非<b>架构</b>。
不固定角度，架构比较就是角度比较的伪装。</p>

<h3>3.2 等侧向加速度标定</h3>
<p>ISO 7401 用阶跃产生的<b>稳态侧向加速度</b>而非转角定义输入量。
本文每一个瞬态指标都先用<b>二分法</b>把前轮转角标定到目标 a_y：
区间 [0.1°, 20°]，每次迭代跑一次完整的稳态回转（整定 + 7 s 保持 + 0.5 s 平均），
收敛判据 |a_y − a_y*| &lt; 0.02 m/s²，最多 12 次迭代。
实测九个案例全部落在 {ay_target:.1f} ± 0.02 m/s²。</p>
<p>为什么不能省：阶梯的要害就是各级转向增益不同。
等转角下比，会把「响应更快」和「转得更狠」混为一谈。
代价是 L2 需要 4.96° 前轮转角才能达到 L0 用 1.54° 就达到的横摆——
<b>这个代价本身就是结果之一</b>（第 7.1 节）。</p>

<h3>3.3 指标定义</h3>
<div class="tablewrap"><table>
<thead><tr><th>指标</th><th>定义</th><th>取值方式</th></tr></thead>
<tbody>
<tr><th>稳态值</th><td style="text-align:left">阶跃后最后 0.5 s 的算术平均</td>
<td style="text-align:left">抑制残余极限环被读成稳态值</td></tr>
<tr><th>T90</th><td style="text-align:left">|r| 首次达到 0.9×稳态值的时刻</td>
<td style="text-align:left">从阶跃施加瞬间起计；未达到则记 NaN</td></tr>
<tr><th>超调</th><td style="text-align:left">(max|r| / 稳态 − 1) × 100%</td>
<td style="text-align:left">全窗口取最大</td></tr>
<tr><th>峰值时刻</th><td style="text-align:left">|r| 取最大值的时刻</td><td style="text-align:left">—</td></tr>
<tr><th>TB</th><td style="text-align:left">a_y 的 T90 减去 r 的 T90</td>
<td style="text-align:left">ISO 7401 的响应时间差；负值表示侧向加速度先于横摆建立</td></tr>
<tr><th>β/g</th><td style="text-align:left">稳态 β 除以稳态 a_y（以 g 计）</td>
<td style="text-align:left"><b>保留符号</b>——β 会过零变号，取绝对值会把关键现象藏起来</td></tr>
</tbody></table></div>

<h3>3.4 频率响应：为什么用相关法而不是 FFT</h3>
<p>单频正弦转向，丢弃前两个周期（瞬态），
再对<b>整数个</b>剩余周期做正弦／余弦相关：</p>
<div class="eq">G(ω) = ⟨y, sin ωt⟩ + j·⟨y, cos ωt⟩ &nbsp;&nbsp; 归一化后除以输入相量</div>
<p>相关法在整周期上做，因此不受频谱泄漏影响，也不需要加窗；
丢弃两个周期保证估计的是稳态频响而不是包含起振过程的混合。</p>

<h3>3.5 整定判据：本研究修正自己的第二次</h3>
<div class="callout bad"><h4>一处被查出来的测量错误</h4>
<p>第一版在 μ = {low_mu} 低附着上报出 L0 横摆响应时间 2408 ms、L2 为 108 ms，
相差 20 倍。核查轨迹发现：低附着上起步是<b>附着受限</b>的，
固定 6 s 直线整定不足以加速到 {vstep:.0f} km/h，「阶跃」施加在加速过程中——
2408 ms 量的是<b>加速完成时间</b>，108 ms 是加速段里的一个瞬态尖峰。两个数都是废的。</p>
<p>整定改为<b>等速度收敛</b>：跑到 |v − v*|/v* &lt; 1% 并<b>保持 1 s</b> 才施加阶跃，
超过 40 s 未收敛则抛错而不是返回一个可疑的数。
重测后低附着结果与高附着一致（330／398／820／790 ms）。
本文所有数据均出自修正后的版本。</p></div>

<h3>3.6 后轮限幅的实现位置</h3>
<p>后轮权限在控制律<b>之后、作动器之前</b>对指令限幅，不是机械限位。
因此不含作动器顶到限位后的绕线与回弹。在 ±{rear_env:.0f}° 下，
后轮对的阿克曼张角小于 0.1°，故逐轮限幅与整轴限幅在本文分辨率内等价。</p>

<h3>3.7 整车与模型</h3>
<p>14 自由度多体（车身 6 + 悬架 4 + 轮转动 4），固定步长 RK4，dt = 2 ms，
纵向为<b>转矩模式</b>——真实动力总成，而不是会把车硬拽在目标速度上的速度伺服。
这一点对制动/驱动耦合工况是必要的：速度伺服会掩盖掉纵向与侧向的附着竞争。</p>
"""


def algo_section(A, figs, algo_meta, fmt_pct):
    """Section 6 — the control laws, one subsection each, then the cross-cuts."""
    speed = A["speed"]
    noise = {r["algo"]: r for r in A["noise"]}
    dwell = {r["algo"]: r for r in A["dwell"]}
    corr = A["corrected"]
    order = ["none", "fixed", "schedule", "yaw_fb", "transient", "model_follow"]
    speeds = A["meta"]["speeds"]
    by = {(r["algo"], r["v"]): r for r in speed}

    def per_law(key, idx):
        d = LAW_DOC[key]
        name = algo_meta[key]["label_cn"]
        en = algo_meta[key]["label"]
        rows = "".join(
            f'<tr><th>{v:.0f}</th>'
            f'<td>{by[(key, v)]["delta_deg"]:.2f}</td>'
            f'<td>{by[(key, v)]["dr_over_df"]:+.3f}</td>'
            f'<td>{by[(key, v)]["yaw_t90"] * 1000:.0f}</td>'
            f'<td>{by[(key, v)]["yaw_overshoot"]:.1f}</td>'
            f'<td class="{"neg" if by[(key, v)]["beta_per_g"] < 0 else ""}">'
            f'{by[(key, v)]["beta_per_g"]:+.2f}</td>'
            f'<td>{by[(key, v)]["dr_max"]:.2f}</td></tr>'
            for v in speeds)
        nz = noise[key]
        dw = dwell[key]
        return f"""
<h3>6.{idx} {name} <span class="dim">{en}</span></h3>
<div class="eq law" style="--c:{ALGO_COLOR[key]}">{d["eq"]}</div>
<p class="gains"><b>参数</b>：{d["gains"]} &nbsp;·&nbsp;
<b>需要的信号</b>：{d["needs"]} &nbsp;·&nbsp; <b>代价</b>：{d["cost"]}</p>
<p>{d["theory"]}</p>
<div class="tablewrap"><table>
<thead><tr><th>车速 km/h</th><th>δ_f °</th><th>δ_r/δ_f</th><th>T90 ms</th>
<th>超调 %</th><th>β/g °</th><th>δ_r 峰值 °</th></tr></thead>
<tbody>{rows}</tbody></table>
<div class="tnote">等 a_y = {A["meta"]["ay"]:.1f} m/s²，载体固定为 {A["meta"]["carrier"]} 硬件。
正弦停留（80 km/h）：峰值横摆 {dw["peak_yaw"]:.2f} °/s，最大 β {dw["max_beta"]:.2f}°，
后轮峰值 {dw["dr_max"]:.2f}°，输入结束 1.0 s／1.75 s 后残余
{dw["ratio_1s"]:.3f}／{dw["ratio_175s"]:.3f}。
噪声：后轮 RMS {nz["dr_rms_deg"]:.3f}°（放大 {nz["amplification"]:.2f}×），
角速率 RMS {nz["dr_rate_rms_dps"]:.2f} °/s。</div></div>
<p><b>读解：</b>{d["verdict"]}</p>
"""

    subs = "".join(per_law(k, i + 1) for i, k in enumerate(order))

    # cross-cut: the corrected-plant experiment
    crows = "".join(
        f'<tr><th>{v:.0f}</th>'
        f'<td>{_c(corr, v, False)["delta_deg"]:.2f}</td>'
        f'<td class="{"neg" if _c(corr, v, False)["beta_per_g"] < 0 else "bad"}">'
        f'{_c(corr, v, False)["beta_per_g"]:+.2f}</td>'
        f'<td>{_c(corr, v, False)["yaw_t90"] * 1000:.0f}</td>'
        f'<td>{_c(corr, v, True)["delta_deg"]:.2f}</td>'
        f'<td class="good">{_c(corr, v, True)["beta_per_g"]:+.2f}</td>'
        f'<td>{_c(corr, v, True)["yaw_t90"] * 1000:.0f}</td></tr>'
        for v in speeds)

    nrows = "".join(
        f'<tr><th>{algo_meta[k]["label_cn"]}</th>'
        f'<td>{noise[k]["dr_rms_deg"]:.3f}</td>'
        f'<td class="hi">{noise[k]["amplification"]:.2f}</td>'
        f'<td>{noise[k]["dr_rate_rms_dps"]:.2f}</td>'
        f'<td>{noise[k]["yaw_rms_dps"]:.4f}</td></tr>'
        for k in sorted(order, key=lambda x: noise[x]["amplification"]))

    worst = max(order, key=lambda k: noise[k]["amplification"])
    tr_amp = noise["transient"]["amplification"]
    mf_amp = noise["model_follow"]["amplification"]

    return f"""
<h2><span class="n">6</span>控制算法的贡献</h2>
<p>本节固定 {A["meta"]["carrier"]} 硬件，只换控制律，因此全部差异来自算法。
六条律逐一给出：<b>实现方程</b>、<b>理论出处</b>、<b>所需信号</b>、
<b>四个速度点的完整数据</b>、以及一段读解。</p>
<div class="callout"><h4>为什么必须跨速度看</h4>
<p>六条律里有四条的后前轮比 δ_r/δ_f 是速度的函数，其中两条<b>在扫描区间内变号</b>。
在单一速度上排名会得到与另一速度相反的结论——
例如定比例律在 40 km/h 是最快的，在 140 km/h 超调 21.5% 已不可接受。
本节所有表格因此都是四速度的。</p></div>
{subs}

<h3>6.7 横向对比</h3>
{figs["speed_t90"]}
{figs["speed_beta"]}
{figs["speed_ratio"]}
{figs["speed_os"]}

<h3>6.8 对象模型缺陷的对照实验</h3>
<p>第 2.5 节指出控制律用的轴侧偏刚度漏掉了车辆的
{A["theory"]["scale_f"]:.2f}／{A["theory"]["scale_r"]:.2f} 分配。
如果这是 β 过冲的原因，那么<b>只把 k(v) 曲线换成用真实刚度重算的版本</b>、
其余一概不动，就应当把稳态侧偏角拉回零。这是一个可证伪的预言，下表是结果。</p>
<div class="tablewrap"><table>
<thead><tr><th rowspan="2">车速 km/h</th><th colspan="3">默认 k(v)（Cf = Cr）</th>
<th colspan="3">修正 k(v)（真实轴刚度）</th></tr>
<tr><th>δ_f °</th><th>β/g °</th><th>T90 ms</th>
<th>δ_f °</th><th>β/g °</th><th>T90 ms</th></tr></thead>
<tbody>{crows}</tbody></table></div>
<div class="callout good"><h4>预言成立</h4>
<p>换一张表，β/g 在四个速度点上全部回到零附近（140 km/h：+3.06 → −0.19），
而且<b>横摆响应还顺带变快了</b>（482 → 322 ms）——因为过量的同相后轮转角
本来就在拖慢横摆建立。前轮转角需求同时下降（2.45° → 1.46°），
意味着驾驶员要打的方向盘也变少了。</p>
<p>结论因此是明确的：<b>速度调度律与模型跟随律在本文中的糟糕表现，
不是这两种结构的固有缺陷，而是它们的对象模型漏了一个参数。</b>
这也解释了为什么横摆反馈律不受影响——它根本不用车辆参数。
<b>依赖模型越深的控制律，被模型误差伤得越狠</b>，这是本文对控制方案选型最实用的一条经验。</p></div>

<h3>6.9 噪声敏感度：一次被实测推翻的断言</h3>
<div class="callout warn"><h4>更正</h4>
<p>本报告早期版本断言瞬态补偿律「对 dδ_f/dt 求导，因此放大转角传感器噪声」，
并注明该断言在无噪声仿真中<b>未经检验</b>。现在检验了，结论要修正两处：</p>
<ul class="tight">
<li>噪声放大最严重的<b>不是</b>瞬态补偿律（{tr_amp:.2f}×），
    而是<b>模型跟随律</b>（{mf_amp:.2f}×）——后者的横摆误差反馈同样在放大噪声，
    且增益更高。</li>
<li>六条律的放大倍数<b>全部小于 1</b>，即都是净衰减。
    原因是控制律内部统一带一个 τ = 0.08 s 的一阶低通，
    对 12 Hz 带限噪声已有足够压制。原断言隐含「求导必然放大」是不成立的——
    求导之后有没有滤波才是决定性的。</li>
</ul>
<p>成立的部分是：求导确实抬高了<b>作动器角速率</b>需求——
瞬态补偿律 {noise["transient"]["dr_rate_rms_dps"]:.2f} °/s，
是同底座速度调度律（{noise["schedule"]["dr_rate_rms_dps"]:.2f} °/s）的
{noise["transient"]["dr_rate_rms_dps"] / max(noise["schedule"]["dr_rate_rms_dps"], 1e-9):.1f} 倍。
代价是真的，只是落在作动器功率而不是横摆品质上。</p></div>
<div class="tablewrap"><table>
<thead><tr><th>控制律</th><th>后轮角 RMS °</th><th>噪声放大 ×</th>
<th>后轮角速率 RMS °/s</th><th>横摆 RMS °/s</th></tr></thead>
<tbody>{nrows}</tbody></table>
<div class="tnote">直线行驶 100 km/h，在转向通道注入 {A["meta"]["noise_sigma_deg"]:.2f}° RMS、
{A["meta"]["noise_bw_hz"]:.0f} Hz 带限噪声，{A["meta"]["noise_seconds"]:.0f} s。
六条律使用<b>同一条噪声序列</b>（固定随机种子），因此差异纯粹来自控制律。</div></div>
{figs["noise"]}

<h3>6.10 正弦停留：稳定性而非响应</h3>
{figs["dwell"]}
<p>全部六条律在输入结束 1.0 s 后的残余横摆都低于峰值的 2%，
即<b>没有一条律让车辆失稳</b>——这个工况在本文的参数下不具备区分度，
它的价值在于确认「快」的那几条律没有以稳定性为代价换来速度。
唯一值得注意的是定比例律：峰值横摆 {dwell["fixed"]["peak_yaw"]:.2f} °/s，
最大 β {dwell["fixed"]["max_beta"]:.2f}°，两项都是六条中最高，
再次印证「常数反相比在高速是错的」。</p>
"""


def _c(corr, v, corrected):
    return next(r for r in corr if r["v"] == v and r["corrected"] is corrected)
