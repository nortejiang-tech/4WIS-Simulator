// Content model for the 4WIS theory page. Single-flow, progressively deeper.
// Each chapter renders as: number+title → diagram → intuition (plain language for
// PM/leadership) → key formula (KaTeX) → collapsible derivation (textbook-grade
// for engineers) → optional LS9 magnitude callout → optional interactive demo.
//
// All `*Html` strings may contain $...$ / $$...$$ — the page runs KaTeX over the
// rendered container (reusing components/load/latex.ts).

import type { DiagramKey } from "./diagrams";

export type DemoKey = "bicycleGain" | "tireCurve" | "kingpinBreakdown";

export interface Chapter {
  id: string;
  num: string;
  title: string;
  diagram?: DiagramKey;
  /** 现象 / 白话 — readable by non-specialists. */
  intuitionHtml: string;
  /** Key formula(s), KaTeX. */
  formulaHtml?: string;
  /** Textbook derivation, collapsible. */
  derivationHtml?: string;
  /** LS9 order-of-magnitude callout. */
  calloutHtml?: string;
  /** Interactive demo mounted under the chapter. */
  demo?: DemoKey;
}

export const CHAPTERS: Chapter[] = [
  {
    id: "intro",
    num: "0",
    title: "这套模型是什么 · 为谁服务",
    diagram: "hierarchy",
    intuitionHtml: `
<p><strong>一句话</strong>：这是整个 4WIS 仿真器背后的"数学发动机"。你在仿真工作台看到的车怎么动、在负载特性页看到的每条曲线，
都是这一套模型算出来的——这一页把它从头讲清楚。</p>
<p>四轮独立转向（4WIS）意味着<strong>四个车轮的转角和转速都能各自独立控制</strong>，没有传统转向那根机械连杆。
好处是能蟹行、能原地转、能高速稳态更好；代价是每个轮的受力、每个电机要出多大力，都得算准——这正是这套模型要回答的。</p>
<p>模型按 <strong>车 → 轴 → 轮</strong> 三层组织（见右图）：整车定平动和横摆，轴负责左右镜像，单轮算真正的轮胎力与主销力矩。
指令（转角 δ、转速 ω）自上而下分发，力和力矩自下而上汇总。</p>`,
    derivationHtml: `
<p>工程上这是一个<strong>分层多体的降阶模型</strong>：保留对转向负载和操稳分析最关键的自由度（整车平面 3-DOF + 四轮转向/转速），
舍弃对该目标低敏感度的高阶项（全六自由度车身、悬架柔度的细节动力学留给 research 级多体模型）。
这样既能做实时仿真，又能做工程量级的负载/选型分析，且参数集可控。</p>`,
  },
  {
    id: "frame",
    num: "1",
    title: "坐标系与符号约定",
    diagram: "bodyFrame",
    intuitionHtml: `
<p>先把"方向"定死，后面所有公式才有意义。车体坐标系原点放在<strong>前后轴中点</strong>，
$X$ 朝前、$Y$ 朝左、$Z$ 朝上。四个轮固定编号 FL / FR / RL / RR。</p>
<p>所有后端计算用 SI 单位、角度用弧度；只有界面显示时才换成 mm / 度 / km·h⁻¹。</p>`,
    formulaHtml: `
<p>核心符号：</p>
<table class="model-symtab">
<tr><td>$\\delta_i$</td><td>第 $i$ 轮转角（CCW 正）</td></tr>
<tr><td>$\\alpha_i$</td><td>第 $i$ 轮侧偏角</td></tr>
<tr><td>$\\kappa_i$</td><td>第 $i$ 轮纵滑率</td></tr>
<tr><td>$v_x, v_y$</td><td>车身原点纵/侧向速度</td></tr>
<tr><td>$r$</td><td>横摆角速度（yaw rate）</td></tr>
<tr><td>$\\beta$</td><td>车身侧偏角 $=\\arctan(v_y/v_x)$</td></tr>
<tr><td>$F_{z,i}$</td><td>第 $i$ 轮垂向载荷</td></tr>
</table>`,
  },
  {
    id: "wheel-kin",
    num: "2",
    title: "单轮运动学：从车身运动到滑移",
    diagram: "slipAngle",
    intuitionHtml: `
<p>轮胎产生多大力，取决于它"被拖着歪了多少"——也就是<strong>侧偏角 α</strong>：轮子自己的滚动方向，和它实际行进方向之间的夹角。
直行时 α=0、无侧向力；歪得越多，侧向力越大，直到打滑饱和。</p>
<p>怎么算 α？先把车身的平动 + 转动合成出<strong>每个轮心的速度</strong>，再旋转到轮自己的坐标系，取反正切。</p>`,
    formulaHtml: `
$$\\mathbf v_i = \\mathbf v_{\\text{origin}} + \\boldsymbol\\omega \\times \\mathbf r_i
\\;\\Rightarrow\\;
\\begin{cases} v_{x,i} = v_x - r\\,y_i \\\\ v_{y,i} = v_y + r\\,x_i \\end{cases}$$
$$\\alpha_i = \\arctan\\!\\frac{-\\sin\\delta_i\\,v_{x,i} + \\cos\\delta_i\\,v_{y,i}}{\\max(|\\cos\\delta_i\\,v_{x,i} + \\sin\\delta_i\\,v_{y,i}|,\\ \\varepsilon)}
\\qquad
\\kappa_i = \\frac{R\\,\\omega_i - v_{\\text{long},i}}{\\max(|v_{\\text{long},i}|,\\ \\varepsilon)}$$`,
    derivationHtml: `
<p>第一式是刚体平面运动学：轮心 $i$ 在车体系的速度等于原点速度加上横摆角速度叉乘臂长 $\\mathbf r_i=(x_i,y_i)$。
第二步把该速度旋转 $-\\delta_i$ 进入轮坐标系，得到沿滚动方向的 $v_{\\text{long}}$ 和垂直方向的 $v_{\\text{lat}}$，
侧偏角即 $\\alpha=\\arctan(v_{\\text{lat}}/|v_{\\text{long}}|)$。分母用 $\\varepsilon$（默认 0.5 m/s）兜底，避免低速除零。
实现见 <code>model_core.wheel_slip_kinematics</code>，时域模型与准静态 sweep 共用同一函数。</p>`,
  },
  {
    id: "tire",
    num: "3",
    title: "轮胎力：Pacejka + 摩擦椭圆",
    diagram: "tireCurve",
    intuitionHtml: `
<p>轮胎不是弹簧——小角度时力随 α 线性增长（斜率叫<strong>侧偏刚度 Cα</strong>），
但地面能给的力有上限 $\\mu F_z$（附着系数 × 垂载），到顶就<strong>饱和</strong>，再歪也不增反降。
这条"先直后弯再平"的曲线就是轮胎的灵魂。</p>
<p>纵向（驱动/制动）和侧向同时存在时，二者要共享同一份附着——这就是<strong>摩擦椭圆</strong>：合力不能超出椭圆边界。</p>`,
    formulaHtml: `
$$F_{y0} = -D\\,\\sin\\!\\big(C_y \\arctan(B_y\\alpha - E_y(B_y\\alpha - \\arctan B_y\\alpha))\\big),\\quad D=\\mu F_z,\\ B_y=\\tfrac{C_\\alpha}{C_y D}$$
$$\\Big(\\tfrac{F_x}{\\mu F_z}\\Big)^2 + \\Big(\\tfrac{F_y}{\\mu F_z}\\Big)^2 \\le 1 \\quad\\text{(摩擦椭圆裁剪)}$$`,
    derivationHtml: `
<p>采用简化魔术公式（Magic Formula）：$B$ 由小信号刚度反推（$B_y=C_\\alpha/(C_y D)$），保证<strong>原点斜率严格等于 Cα</strong>、
峰值等于 $\\mu F_z$。组合滑移用摩擦椭圆一次裁剪（friction ellipse），是线性 friction circle 的光滑版本，避免硬拐角。
载荷敏感性 $C_\\alpha(F_z)=C_{\\alpha 0}(F_z/F_{z,\\text{nom}})^{p}$（默认 $p=0.8$）：高速气动升力降低 $F_z$ 时，
线性区斜率和饱和峰值<strong>同时</strong>下降。唯一内核 <code>tire.pacejka_combined_forces</code>，时域与准静态共用。</p>`,
    calloutHtml: `<strong>LS9 量级</strong>：单轮静载约 7 kN，μ=0.85 → 峰值侧向力约 6 kN；Cα ≈ 1.2×10⁵ N/rad，
峰值出现在 α ≈ 6–8°。`,
    demo: "tireCurve",
  },
  {
    id: "load",
    num: "4",
    title: "垂向载荷：静态 + 转移 + 气动",
    diagram: "loadTransfer",
    intuitionHtml: `
<p>每个轮压在地上的力 $F_z$ 不是固定的：刹车时载荷往前压、加速往后坐、转弯往外侧压、高速时气动升力把整车"托轻"。
$F_z$ 一变，能给的轮胎力上限就变——所以它是一切力的"地基"。</p>`,
    formulaHtml: `
$$F_{z,i} = \\underbrace{F_{z,\\text{static}}}_{mgb/2L\\ \\text{或}\\ mga/2L}
\\;\\underbrace{\\mp\\, m a_x h / 2L}_{\\text{纵向转移}}
\\;\\underbrace{\\pm\\, m a_y h\\,(\\cdot)/t}_{\\text{横向转移}}
\\;\\underbrace{-\\, \\tfrac12\\rho C_l A v^2 / 2}_{\\text{气动升力}}$$`,
    derivationHtml: `
<p>准静态载荷转移（无悬架柔度的刚体近似）：纵向加速度 $a_x$ 经质心高度 $h$ 在前后轴间转移 $m a_x h / 2L$；
横向加速度 $a_y$ 按各轴质量份额在左右轮间转移。气动升力按前/后轴系数 $C_{l,f}, C_{l,r}$ 扣减。
实现 <code>load_transfer.vertical_loads</code> + <code>model_core.fz_with_aero_lift</code>。多体研究模型会用真实悬架自由度替代这个准静态式。</p>`,
  },
  {
    id: "kingpin",
    num: "5",
    title: "主销力矩：电机真正要克服的负载",
    diagram: "kingpinSide",
    intuitionHtml: `
<p>轮胎力并不直接顶在转向电机上，而是通过<strong>主销轴</strong>（车轮转动绕的那根轴）形成力矩。
主销有后倾（caster）、有偏置（scrub）——这些几何决定了同样的轮胎力会产生多大的转向阻力矩 $\\tau_{KP}$。
这就是电机选型最关心的量。</p>`,
    formulaHtml: `
$$\\tau_{KP} = \\underbrace{F_y\\,(s + t_m + t_p)}_{\\text{侧向力×拖距}}
+ \\underbrace{F_x\\,s}_{\\text{纵向力×偏置}}
+ \\underbrace{M_z}_{\\text{气胎回正}}
+ \\underbrace{F_z\\sin(\\text{KPI})\\,s\\sin\\delta}_{\\text{主销内倾抬升}}$$
<p>其中机械拖距 $t_m = R\\tan\\varepsilon$（$\\varepsilon$ 后倾角），$s$ 为主销偏置，$t_p$ 气胎拖距。</p>`,
    derivationHtml: `
<p>四项依 Reimpell §3.10 / Pacejka §9：①侧向力经"主销偏置+机械拖距+气胎拖距"的等效力臂回正（最大头）；
②纵向力经主销偏置（scrub）产生 torque-steer；③轮胎自身气胎回正力矩 $M_z$ 直通；④主销内倾（KPI）在转角下"把车顶起来"的回正，$\\propto\\sin\\delta$，直行时为 0。
注：$F_y(s+t_m+t_p)$ 是 Reimpell 的工程等效力臂写法（把倾斜主销的一阶 3D 耦合并进单一杠杆），EPS 选型行业标准用法；严格 3D 推导会略小并多出 $F_z$ 二阶项。
唯一来源 <code>kingpin.kingpin_torque</code>（<code>kingpin_torque_terms</code> 给分项，本页第 5 节交互演示就用它）。</p>`,
    demo: "kingpinBreakdown",
  },
  {
    id: "rack",
    num: "6",
    title: "齿条与电机：力链的最后一环",
    diagram: "linkage",
    intuitionHtml: `
<p>主销力矩经<strong>梯形臂 + 横拉杆</strong>转换成齿条的轴向力，再经齿轮和减速比变成电机扭矩。
机构几何效率 η 在转角大时会变差——同样的主销力矩，会被放大成更大的齿条力（这就是为什么极限转角处特别费劲）。</p>`,
    formulaHtml: `
$$F_{\\text{rack}} = \\frac{\\tau_{KP}}{L_{\\text{arm}}\\cdot\\eta},\\qquad
\\eta = |\\sin\\theta_{\\text{臂-杆}}|\\cdot\\cos\\theta_{\\text{杆-齿条}}\\cdot\\eta_{\\text{rack}},\\qquad
T_{\\text{motor}} = F_{\\text{rack}}\\,\\frac{r_p}{i}$$`,
    derivationHtml: `
<p>梯形机构是一个四连杆：主销转 δ → 梯形臂端点画弧 → 横拉杆推动齿条沿轴平移。
几何效率 η 由"臂-杆夹角"的正弦和"杆-齿条夹角"的余弦决定，乘上齿条机械效率 $\\eta_{\\text{rack}}$。
η 越低，相同 $\\tau_{KP}$ 需要越大的 $F_{\\text{rack}}$。实现 <code>geometry.wheel_rack_force_from_linkage</code>，硬点几何可在负载页编辑。</p>`,
  },
  {
    id: "vehicle",
    num: "7",
    title: "整车两层主线：运动学 vs 动力学",
    diagram: "forceChain",
    intuitionHtml: `
<p>整车有两种保真度：<strong>运动学</strong>假设轮胎不打滑、纯几何反解车身运动，适合验证转向策略、ICR 和几何；
<strong>动力学</strong>才算真实轮胎力、载荷转移和惯性响应，是工程负载分析与选型的主线。多体 14-DOF 模型保留作研究/回归参考。</p>`,
    formulaHtml: `
<p>运动学（最小二乘反解车身速度）：</p>
$$\\min_{v_x,v_y,r}\\ \\sum_i \\big\\| \\mathbf v_i - s_i\\,(\\cos\\delta_i,\\sin\\delta_i) \\big\\|^2$$
<p>动力学（Newton–Euler，车体系含 Coriolis）：</p>
$$m(\\dot v_x - r v_y) = \\textstyle\\sum F_{x,i},\\quad
m(\\dot v_y + r v_x) = \\textstyle\\sum F_{y,i},\\quad
I_z\\dot r = \\textstyle\\sum (x_i F_{y,i} - y_i F_{x,i} + M_{z,i})$$`,
    derivationHtml: `
<p>运动学层：四轮的滚动方向给出 8 个速度约束、3 个未知量（$v_x,v_y,r$），超定 → 最小二乘解（<code>kinematic.py</code>）。
动力学层：3-DOF 平面车身 + 4 个轮速自由度，固定步长 RK4 积分（<code>dynamic.py</code>），轮胎力来自上面的 Pacejka 内核，载荷来自准静态转移。
两层都通过 <code>model_core</code> 的滑移/旋转/力组装函数共用底层数学，互不耦合各自的高层职责。</p>`,
  },
  {
    id: "bicycle",
    num: "8",
    title: "稳态 bicycle 耦合：为什么高速更敏感",
    diagram: "bicycle",
    intuitionHtml: `
<p>有个常见误区：以为转角 δ 就等于侧偏角（α=−δ）。<strong>错。</strong>
实车一转向，车身会自己发展出侧偏 β 和横摆 r，每个轮真正的 α 是这三者的合成。
结果是：<strong>车速越高，同样的 δ 产生的 α 越大</strong>——线性区越窄、越容易饱和、回正越重。这就是为什么高速转向"发沉发贼"。</p>`,
    formulaHtml: `
$$\\textstyle\\sum F_{y,i} = m V r,\\qquad \\sum x_i F_{y,i} = 0
\\quad\\Longrightarrow\\quad (\\beta, r)$$
$$\\alpha_i = \\beta + \\frac{r\\,x_i}{V} - \\delta_i
\\qquad\\Rightarrow\\qquad
\\alpha_{FL} \\approx -\\delta\\Big(\\tfrac12 + \\frac{mV^2}{8 C_\\alpha L}\\Big)$$`,
    derivationHtml: `
<p>把四轮当广义线性 bicycle：侧向力平衡 $\\sum F_y=mVr$（向心力）+ 稳态横摆 $\\sum x_i F_{y,i}=0$（无角加速度），
代入 $F_{y,i}=-C_\\alpha\\alpha_i$ 与 $\\alpha_i=\\beta+rx_i/V-\\delta_i$，得 2×2 线性方程组解 $(\\beta,r)$。
对单轮转向的对称底盘可得闭式增益 $\\big(\\tfrac12+mV^2/(8C_\\alpha L)\\big)$，随 $V^2$ 增长。
实现 <code>model_core.solve_steady_state_body</code> / <code>steady_state_slip_angles</code>，负载页 sweep 用它把"台架口径 α=−δ"修正成"行驶口径"。下面的演示直接来自该求解器。</p>
<h4 style="margin-top:16px">v0.8.1：两种单轮分析口径开关</h4>
<p>负载页顶栏和本页第 5 章的主销力矩演示都有一个<strong>受力口径</strong>开关，让你在两种物理框架之间切换：</p>
<ul>
<li><strong>整车装载（vehicle，默认）</strong>：本节描述的稳态 bicycle 耦合。轮装在车身上、车身按 $(\\beta, r)$ 响应。
工程用途：<em>驾驶手感、$\\delta_{eq}$ 真实预测</em>。</li>
<li><strong>单轮台架（isolated）</strong>：把被分析轮视为独立台架上的单元，车身锁定直行 $(V, 0)$，
$\\alpha = -\\delta$ 与车速无关。轮自身物理（载荷敏感 $C_\\alpha(F_z)$、气动升力降 $F_z$、驱动力 $F_x$、camber thrust、toe、parking）<strong>全部照算</strong>，
但<strong>不发展车身响应</strong>。工程用途：<em>作动器/电机最差工况（worst-case）选型</em>——它给的是"无论车身怎么响应，单轮自己一定要扛住多少力"。</li>
</ul>
<p>两种口径下 $\\delta_{cmd}=0$ 处的残余力<strong>完全一致</strong>（无 forcing → 无 $\\beta, r$），但<strong>曲线斜率与饱和位置截然不同</strong>。
LS9 默认参数下 isolated 模式 $\\delta_{eq}$ 在 10→200 km/h 仅从 0.13° 降到 0.08°（载荷敏感二阶效应），vehicle 模式从 0.25° 显著降到 0.02°（bicycle 主导）——并排比较能给出立竿见影的物理直觉。</p>`,
    calloutHtml: `<strong>LS9 量级</strong>：vehicle 模式下增益从 v=10 km/h 的 ~0.5 涨到 v=200 km/h 的 ~3（同样转角下 α 大 6 倍，
饱和转角从 ~6° 收缩到 ~1°）；isolated 模式下增益基本恒定 0.5，仅 $C_\\alpha(F_z)$ 随气动升力小幅软化。`,
    demo: "bicycleGain",
  },
  {
    id: "deq",
    num: "9",
    title: "零输出自然转角 δ_eq 与工程应用",
    intuitionHtml: `
<p>负载页那条"δ_eq 随车速"的曲线问的是：<strong>电机不出力时，车轮会停在哪个角度？</strong>
理想对称车是 0°，但真车有 toe、camber、驱动力经主销偏置等"偏置源"，让这个角度偏离 0 并随车速漂移。</p>
<p>⚠ 注意口径：当前 δ_eq 是<strong>单轮台架 + bicycle 耦合</strong>下该轮齿条力的零点，用于<strong>作动器/电机选型</strong>是合适的；
它<strong>不是</strong>整车四轮联立的真自由稳态（那需要 3-DOF 耦合 + 四轮力矩平衡，是后续版本计划）。</p>`,
    formulaHtml: `
$$\\delta_{eq}(V) := \\arg\\min_{\\delta}\\ \\big|F_{\\text{rack}}(\\delta, V)\\big|_{\\text{single-wheel}}$$`,
    derivationHtml: `
<p>偏置源拆解：静态 toe 直接移轴（速度无关偏置）；camber thrust $C_\\gamma\\gamma F_z$ 在 α=0 也产生侧向力；
维持车速的驱动力 $F_x=C_{rr}mg+\\tfrac12\\rho C_d A V^2$ 经主销偏置 $s$ 产生力矩（随 $V^2$）；气动升力改变 $F_z$ 从而改变 $\\mu F_z$ 与 $C_\\alpha$。
工程用法：$F_{\\text{rack}}$ 在 $\\delta_{cmd}=0$ 的残值 = 直行位电机持续保持力 → 决定电机静态发热电流；峰值 $F_{\\text{rack}}$ → 齿条/丝杠选型上限（安全系数 ≥2）。</p>`,
  },
  {
    id: "bounds",
    num: "10",
    title: "参数边界与解耦约束",
    intuitionHtml: `
<p>模型参数分两层：<strong>核心参数</strong>（几何、质量惯量、轮胎小信号、主销几何、齿条传动）产品经理和底盘工程师都该直接看到；
<strong>高级参数</strong>（Pacejka 曲率、气动、停车经验项、伺服、多体研究项）默认折叠、保留可调性。</p>`,
    derivationHtml: `
<p>解耦约束（这次重构刻意守住的边界，避免再退化成"到处各写一份物理"）：</p>
<ul>
<li>控制器只输出目标 δ/ω，<strong>不</strong>拥有轮胎力或负载公式；</li>
<li>API router 只做请求/响应适配，<strong>不</strong>放物理推导；</li>
<li>前端图表只消费后端结果，<strong>不</strong>重算公式（本页交互演示也走后端 <code>/api/model/demo/*</code>）；</li>
<li>负载页可做 sweep 聚合，但物理内核必须复用 <code>vehicle</code> 层组件；</li>
<li>新增曲线优先挂 <code>dynamic</code> 主线，<code>multibody</code> 仅作研究验证口径。</li>
</ul>
<p>底层数学唯一来源：<code>model_core</code>（运动学/稳态）、<code>tire</code>（轮胎力）、<code>kingpin</code>（主销力矩）、<code>geometry</code>（齿条机构）。</p>`,
  },
];
