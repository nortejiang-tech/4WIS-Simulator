import {
  ADVANCED_PARAMETER_GROUPS,
  CORE_PARAMETER_GROUPS,
} from "@/vehicle/parameterGroups";

const coreNames = CORE_PARAMETER_GROUPS.flatMap((group) => group.fields.map(([, label]) => label));
const advancedNames = ADVANCED_PARAMETER_GROUPS.flatMap((group) => group.fields.map(([, label]) => label));

function VehicleDiagram() {
  return (
    <svg className="model-svg" viewBox="0 0 760 330" role="img" aria-label="四轮独立转向整车坐标与轮位">
      <defs>
        <marker id="arrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
          <path d="M0,0 L0,6 L7,3 z" fill="currentColor" />
        </marker>
      </defs>
      <rect x="260" y="70" width="240" height="170" rx="26" className="model-car" />
      <line x1="380" y1="155" x2="510" y2="155" className="model-axis" markerEnd="url(#arrow)" />
      <line x1="380" y1="155" x2="380" y2="45" className="model-axis" markerEnd="url(#arrow)" />
      <text x="520" y="160">X body</text>
      <text x="390" y="45">Y body</text>
      <circle cx="380" cy="155" r="5" className="model-dot" />
      <text x="390" y="174">body origin</text>
      {[
        [285, 70, -20, "FL"],
        [455, 70, 20, "FR"],
        [285, 225, 20, "RL"],
        [455, 225, -20, "RR"],
      ].map(([x, y, rot, label]) => (
        <g key={label as string} transform={`translate(${x} ${y}) rotate(${rot})`}>
          <rect x="-34" y="-13" width="68" height="26" rx="6" className="model-wheel" />
          <line x1="0" y1="0" x2="54" y2="0" className="model-wheel-axis" markerEnd="url(#arrow)" />
          <text x="-11" y="-24" transform={`rotate(${-rot})`}>{label}</text>
        </g>
      ))}
      <line x1="250" y1="70" x2="250" y2="240" className="model-measure" />
      <text x="198" y="160">wheelbase L</text>
      <line x1="285" y1="55" x2="455" y2="55" className="model-measure" />
      <text x="332" y="42">track</text>
      <path d="M110 270 C210 230, 285 250, 340 185" className="model-path" />
      <text x="84" y="286">ICR / path constraints</text>
    </svg>
  );
}

function ForceChainDiagram() {
  const items = [
    ["Wheel kinematics", "v_i, α_i, κ_i"],
    ["Tire forces", "Fx, Fy, Mz"],
    ["Kingpin torque", "τ_KP"],
    ["Linkage", "F_rack, η"],
    ["Motor sizing", "T_motor"],
  ];
  return (
    <svg className="model-svg model-chain" viewBox="0 0 880 170" role="img" aria-label="转向负载链路">
      {items.map(([title, body], index) => {
        const x = 30 + index * 170;
        return (
          <g key={title}>
            <rect x={x} y="44" width="132" height="70" rx="8" className="model-chain-box" />
            <text x={x + 14} y="74" className="model-chain-title">{title}</text>
            <text x={x + 14} y="96">{body}</text>
            {index < items.length - 1 && (
              <path d={`M${x + 136} 79 L${x + 166} 79`} className="model-chain-arrow" markerEnd="url(#chainArrow)" />
            )}
          </g>
        );
      })}
      <defs>
        <marker id="chainArrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
          <path d="M0,0 L0,6 L7,3 z" fill="currentColor" />
        </marker>
      </defs>
    </svg>
  );
}

function Formula({ children }: { children: React.ReactNode }) {
  return <div className="model-formula">{children}</div>;
}

export default function ModelTheoryPage() {
  return (
    <main className="model-page">
      <section className="model-hero">
        <div>
          <p className="model-kicker">4WIS foundation model</p>
          <h1>四轮独立转向整车数学模型</h1>
          <p>
            这一页描述项目统一后的底层物理口径：从车体坐标系、单轮运动学、轮胎力、
            主销阻力矩到齿条/电机负载。仿真工作台和负载特性页应挂在这些组件上，
            页面本身只讲模型边界，不复制后端计算。
          </p>
        </div>
        <VehicleDiagram />
      </section>

      <section className="model-band">
        <h2>1. 坐标系与轮位</h2>
        <div className="model-grid two">
          <article>
            <h3>整车坐标</h3>
            <p>
              车体系原点放在前后轴中点，X 向前，Y 向左，Z 向上。四轮编号固定为
              FL、FR、RL、RR。所有底层函数都使用 SI 单位，角度在后端使用 rad。
            </p>
            <Formula>
              r_i = [x_i, y_i],&nbsp;
              v_i = [v_x, v_y] + [-r y_i, r x_i]
            </Formula>
          </article>
          <article>
            <h3>四轮独立转向</h3>
            <p>
              控制器只负责输出四个轮的目标转角和轮速。轮胎力、主销力矩、齿条力不在控制器中计算，
              避免控制策略和物理模型过耦合。
            </p>
            <Formula>
              command = &#123;δ_cmd,i, ω_cmd,i&#125;, i ∈ &#123;FL, FR, RL, RR&#125;
            </Formula>
          </article>
        </div>
      </section>

      <section className="model-band">
        <h2>2. 单轮：运动学输入</h2>
        <div className="model-grid three">
          <article>
            <h3>轮心速度</h3>
            <p>
              每个轮心速度由车身平动速度和横摆角速度合成。这个步骤已经收敛到
              `wheel_center_velocities_body`。
            </p>
            <Formula>v_i = [v_x - r y_i, v_y + r x_i]</Formula>
          </article>
          <article>
            <h3>轮坐标变换</h3>
            <p>
              轮胎模型需要沿轮平面和侧向的速度分量，所以先把车体系速度旋转到轮坐标系。
            </p>
            <Formula>
              v_long = cosδ_i v_x + sinδ_i v_y<br />
              v_lat = -sinδ_i v_x + cosδ_i v_y
            </Formula>
          </article>
          <article>
            <h3>Slip angle / ratio</h3>
            <p>
              侧偏角和纵滑率是轮胎力的直接输入。低速用保护项避免除零，高速由真实速度决定。
            </p>
            <Formula>
              α_i = atan2(v_lat, |v_long|)<br />
              κ_i = (ω_i R - v_long) / max(|v_long|, ε)
            </Formula>
          </article>
        </div>
      </section>

      <section className="model-band">
        <h2>3. 单轴：载荷、对齐与轮胎力</h2>
        <div className="model-grid two">
          <article>
            <h3>垂向载荷</h3>
            <p>
              主线动力学使用准静态载荷转移：纵向加速度在前后轴转移载荷，横向加速度在左右轮转移载荷。
              负载特性页额外把气动升力按前/后轴扣减到 Fz。
            </p>
            <Formula>Fz_i = Fz_static + ΔFz_ax + ΔFz_ay - Fz_aero(v)</Formula>
          </article>
          <article>
            <h3>Toe / camber</h3>
            <p>
              toe 是每轴左右镜像的静态转角偏置；camber thrust 是 α=0 时仍可能产生的侧向力偏置。
              它们是 δ_eq 和 0° 保持齿条力的主要标定旋钮。
            </p>
            <Formula>
              α_eff = α - toe<br />
              Fy_camber = Cγ · γ · Fz
            </Formula>
          </article>
        </div>
        <div className="model-note">
          轮胎力内核统一使用 `pacejka_combined_forces`：small-slip 刚度由 Cα/Cκ 给出，
          峰值由 μFz 限制，再通过 friction ellipse 处理 combined slip。
        </div>
      </section>

      <section className="model-band">
        <h2>4. 整车：两层主线模型</h2>
        <div className="model-grid two">
          <article>
            <h3>Kinematic</h3>
            <p>
              运动学层把四个轮的滚动方向约束拟合成车身速度和横摆角速度，适合策略、几何和 ICR 验证。
              它不产生真实轮胎力。
            </p>
            <Formula>min || A · [v_x, v_y, r]^T - b ||²</Formula>
          </article>
          <article>
            <h3>Dynamic</h3>
            <p>
              动力学层是产品主线：3-DOF 平面车身 + 4 个轮速自由度 + 轮胎力 + 载荷转移。
              负载曲线、实时面板和工程选型应优先对齐这一层。
            </p>
            <Formula>
              m(v_dot + r × v) = ΣF_i<br />
              I_z r_dot = Σ(x_i Fy_i - y_i Fx_i)
            </Formula>
          </article>
        </div>
        <div className="model-note">
          `multibody` 仍作为研究模型保留，用于侧倾/俯仰/悬架自由度参考；它不再驱动所有页面必须支持的主线复杂度。
        </div>
      </section>

      <section className="model-band">
        <h2>5. 稳态 bicycle coupling</h2>
        <p>
          单轮负载 sweep 不能把 α 简化成 -δ。实车在高速下会发展出车身侧偏 β 和横摆角速度 r，
          然后每个轮的真实侧偏角变成速度相关量。这个修正解释了高速线性区收缩、斜率增大的现象。
        </p>
        <div className="model-grid two">
          <Formula>
            ΣFy = m V r<br />
            Σ(x_i Fy_i) = 0
          </Formula>
          <Formula>
            α_i = β + r x_i / V - δ_i<br />
            α_FL ≈ -δ · (1/2 + mV²/(8 Cα L))
          </Formula>
        </div>
      </section>

      <section className="model-band">
        <h2>6. 转向负载链路</h2>
        <ForceChainDiagram />
        <div className="model-grid two">
          <article>
            <h3>主销阻力矩</h3>
            <p>
              主销力矩由侧向力、纵向力、气胎回正力矩和低速接地斑扭转共同产生。
              工程选型主要看峰值、线性区斜率和 0° 保持负载。
            </p>
            <Formula>τ_KP = Fy(s + t_m + t_p) + Fx · s + Mz + τ_parking</Formula>
          </article>
          <article>
            <h3>齿条与电机</h3>
            <p>
              梯形臂和横拉杆把主销力矩转换为齿条轴向力。几何效率 η 过低时，同样的主销力矩会放大成更大的齿条力。
            </p>
            <Formula>
              F_rack = τ_KP / (L_arm · η)<br />
              T_motor = F_rack · r_p / i
            </Formula>
          </article>
        </div>
      </section>

      <section className="model-band">
        <h2>7. 参数边界</h2>
        <div className="model-grid two">
          <article>
            <h3>核心参数</h3>
            <p>
              这些参数决定整车几何、质量惯量、轮胎小信号响应、主销几何和齿条传动，是产品经理和底盘工程师都应直接看到的主参数。
            </p>
            <p className="model-chip-list">
              {coreNames.slice(0, 18).map((name) => <span key={name}>{name}</span>)}
            </p>
          </article>
          <article>
            <h3>高级参数</h3>
            <p>
              这些参数保留工程可调性，但默认折叠：Pacejka 曲率、气动、停车经验项、bump-steer、伺服和多体研究项。
            </p>
            <p className="model-chip-list">
              {advancedNames.slice(0, 18).map((name) => <span key={name}>{name}</span>)}
            </p>
          </article>
        </div>
      </section>

      <section className="model-band">
        <h2>8. 工程边界</h2>
        <ul className="model-list">
          <li>控制器只输出目标转角/轮速，不拥有轮胎力或负载公式。</li>
          <li>API router 只做请求响应适配，不放物理推导。</li>
          <li>前端图表只消费后端结果，不重新实现公式。</li>
          <li>负载页可做 sweep 聚合，但物理内核必须复用 `vehicle` 层组件。</li>
          <li>新增曲线优先挂到 `dynamic` 主线；`multibody` 作为研究验证口径。</li>
        </ul>
      </section>
    </main>
  );
}
