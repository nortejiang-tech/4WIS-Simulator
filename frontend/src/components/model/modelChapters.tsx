// Content model for the 4WIS theory page. Single-flow, progressively deeper.
// Each chapter renders as: number+title → diagram → intuition (plain language for
// PM/leadership) → key formula (KaTeX) → collapsible derivation (textbook-grade
// for engineers) → optional LS9 magnitude callout → optional interactive demo.
//
// All `*Html` strings may contain $...$ / $$...$$ — the page runs KaTeX over the
// rendered container (reusing components/load/latex.ts).

import type { DiagramKey } from "./diagrams";
import { KINGPIN_FORMULA, KINGPIN_DERIVATION, RACK_FORMULA, LINKAGE_INDICATOR, STEADY_BODY } from "./physicsExplanations";

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
<p>四轮独立转向（4WIS）意味着<strong>四个车轮的转角能各自独立控制</strong>。独立驱动是另一项能力，本仿真器也提供四轮独立驱动的抽象。
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
<tr><td>$\\beta$</td><td>原点侧偏角 $=\\operatorname{atan2}(v_y,v_x)$</td></tr>
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
峰值等于 $\\mu F_z$。组合滑移用摩擦椭圆一次裁剪（friction ellipse），是合力超限时的径向投影，边界处不保证导数光滑。
载荷敏感性 $C_\\alpha(F_z)=C_{\\alpha 0}(F_z/F_{z,\\text{nom}})^{p}$（默认 $p=0.8$）：高速气动升力降低 $F_z$ 时，
线性区斜率和饱和峰值<strong>同时</strong>下降。唯一内核 <code>tire.pacejka_combined_forces</code>，时域与准静态共用。</p>
<p>口径说明：更一般的摩擦椭圆允许纵横峰值不同 $\\big((F_x/\\mu_x F_z)^2+(F_y/\\mu_y F_z)^2\\le1\\big)$；本模型取 $\\mu_x=\\mu_y=\\mu$（圆），$\\mu$ 由场景逐轮给定。</p>`,
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
实现 <code>load_transfer.vertical_loads</code> + <code>model_core.fz_with_aero_lift</code>。多体研究模型使用垂向悬架自由度。四轮离地/翻滚超出该准静态模型边界。</p>`,
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
    formulaHtml: KINGPIN_FORMULA,
    derivationHtml: KINGPIN_DERIVATION,
    demo: "kingpinBreakdown",
  },
  {
    id: "rack",
    num: "6",
    title: "齿条与电机：力链的最后一环",
    diagram: "linkage",
    intuitionHtml: `<p>主销力矩通过梯形臂与横拉杆传到齿条，再经小齿轮和减速器传到电机。传动比由机构运动决定，必须同时满足力平衡与虚功。</p>`,
    formulaHtml: RACK_FORMULA,
    derivationHtml: LINKAGE_INDICATOR,
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
<p>动力学：公开速度在轴距中点 O，质心位于 $(e,0)$，$e=L/2-a$。</p>
$$I_z\dot r=M_O-eF_y,\qquad
\dot v_x=F_x/m+rv_y+er^2,\qquad
\dot v_y=F_y/m-rv_x-e\dot r$$`,
    derivationHtml: `<p>车身速度在旋转坐标系中表达；惯量定义在质心，先将外力矩从 O 平移到质心。
载荷转移使用质心加速度。平面车身用 RK4 子步，轮速使用实际非线性轮胎力的后向欧拉求解；
载荷、轮速与车身分步耦合，因此整个算法不具有四阶收敛保证。分析结果需要步长收敛检查。</p>
<p>运动学忽略力的可实现性；多体模型增加垂向悬架自由度，但仍是小角、低阶研究模型。</p>`,
  },
  {
    id: "bicycle",
    num: "8",
    title: "稳态 bicycle 耦合：为什么高速更敏感",
    diagram: "bicycle",
    intuitionHtml: `<p>车轮转角与轮胎侧偏角不同：车身的横向速度与横摆会改变各轮实际滑移。稳态 bicycle 近似用来解释这种耦合，并估计小信号响应。</p>`,
    formulaHtml: STEADY_BODY,
    derivationHtml: `<p>每轮侧偏刚度包括前/后轴比例。独立转角经侧向合力和质心横摆力矩平衡共同确定响应；正、反向转弯应在对称参数下互为镜像。交互演示直接调用后端求解器。</p>`,
    demo: "bicycleGain",
  },
  {
    id: "deq",
    num: "9",
    title: "零输出自然转角 δ_eq 与工程应用",
    intuitionHtml: `
<p>负载页那条"δ_eq 随车速"的曲线问的是：<strong>电机不出力时，车轮会停在哪个角度？</strong>
理想对称车是 0°，但真车有 toe、camber、驱动力经主销偏置等"偏置源"，让这个角度偏离 0 并随车速漂移。</p>
<p>⚠ 注意口径：当前 δ_eq 是<strong>单轮台架 + bicycle 耦合</strong>下该轮齿条力的零点，可为<strong>作动器概念设计</strong>提供参考；
它<strong>不是</strong>整车四轮联立的真自由稳态（那需要 3-DOF 耦合 + 四轮力矩平衡，是后续版本计划）。</p>`,
    formulaHtml: `
$$\\delta_{eq}(V) := \\arg\\min_{\\delta}\\ \\big|F_{\\text{rack}}(\\delta, V)\\big|_{\\text{single-wheel}}$$`,
    derivationHtml: `
<p>偏置源拆解：静态 toe 直接移轴（速度无关偏置）；camber thrust $C_\\gamma\\gamma F_z$ 在 α=0 也产生侧向力；
维持车速的驱动力 $F_x=C_{rr}mg+\\tfrac12\\rho C_d A V^2$ 经主销偏置 $s$ 产生力矩（随 $V^2$）；气动升力改变 $F_z$ 从而改变 $\\mu F_z$ 与 $C_\\alpha$。
工程用法：$F_{\\text{rack}}$ 在 $\\delta_{cmd}=0$ 的残值 = 直行位电机持续保持力 → 决定电机静态发热电流；峰值 $F_{\\text{rack}}$ → 齿条/丝杠选型上限（须另行定义任务谱、载荷包络与设计裕量）。</p>`,
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
