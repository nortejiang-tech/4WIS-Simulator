// Didactic SVG diagrams for the model-theory page.
// Pure SVG + CSS-variable colours (model-svg style family). Each diagram is a
// small component; DIAGRAMS maps a key → component so chapters can reference one
// by name.

import type { ReactNode } from "react";

function Arrow() {
  return (
    <marker id="md-arrow" markerWidth="9" markerHeight="9" refX="6.5" refY="3" orient="auto">
      <path d="M0,0 L0,6 L7,3 z" fill="currentColor" />
    </marker>
  );
}

function Svg({ vb, label, children }: { vb: string; label: string; children: ReactNode }) {
  return (
    <svg className="model-svg" viewBox={vb} role="img" aria-label={label}>
      <defs><Arrow /></defs>
      {children}
    </svg>
  );
}

// 0/2 — vehicle → axle → wheel hierarchy
function HierarchyDiagram() {
  return (
    <Svg vb="0 0 820 260" label="车-轴-轮层次">
      <rect x="30" y="100" width="150" height="60" rx="10" className="model-chain-box" />
      <text x="105" y="125" textAnchor="middle" className="model-chain-title">整车 Vehicle</text>
      <text x="105" y="145" textAnchor="middle">3-DOF：vx, vy, r</text>

      <rect x="300" y="40" width="150" height="56" rx="10" className="model-chain-box" />
      <text x="375" y="63" textAnchor="middle" className="model-chain-title">前轴 Front axle</text>
      <text x="375" y="82" textAnchor="middle">FL · FR（Y 镜像）</text>
      <rect x="300" y="164" width="150" height="56" rx="10" className="model-chain-box" />
      <text x="375" y="187" textAnchor="middle" className="model-chain-title">后轴 Rear axle</text>
      <text x="375" y="206" textAnchor="middle">RL · RR（Y 镜像）</text>

      {[["FL", 40], ["FR", 96], ["RL", 164], ["RR", 220]].map(([lbl, y]) => (
        <g key={lbl as string}>
          <rect x="580" y={(y as number)} width="150" height="44" rx="8" className="model-wheel" />
          <text x="655" y={(y as number) + 27} textAnchor="middle">{lbl} 单轮 δ,ω,Fz</text>
        </g>
      ))}

      <g className="model-axis" markerEnd="url(#md-arrow)">
        <line x1="182" y1="120" x2="298" y2="68" />
        <line x1="182" y1="140" x2="298" y2="192" />
        <line x1="452" y1="60" x2="578" y2="62" />
        <line x1="452" y1="78" x2="578" y2="118" />
        <line x1="452" y1="186" x2="578" y2="186" />
        <line x1="452" y1="200" x2="578" y2="242" />
      </g>
      <text x="410" y="252" textAnchor="middle" className="model-muted">力 / 力矩自下而上汇总，指令 δ/ω 自上而下分发</text>
    </Svg>
  );
}

// 1 — body frame + wheel positions
function BodyFrameDiagram() {
  const wheels: [number, number, number, string][] = [
    [285, 70, -18, "FL"], [455, 70, 18, "FR"], [285, 225, 18, "RL"], [455, 225, -18, "RR"],
  ];
  return (
    <Svg vb="0 0 720 320" label="车体坐标与轮位">
      <rect x="260" y="70" width="240" height="170" rx="26" className="model-car" />
      <g className="model-axis" markerEnd="url(#md-arrow)">
        <line x1="380" y1="155" x2="500" y2="155" />
        <line x1="380" y1="155" x2="380" y2="48" />
      </g>
      <text x="508" y="160">X（前）</text>
      <text x="388" y="48">Y（左）</text>
      <circle cx="380" cy="155" r="5" className="model-dot" />
      <text x="388" y="174">原点=轴距中点</text>
      {wheels.map(([x, y, rot, label]) => (
        <g key={label} transform={`translate(${x} ${y}) rotate(${rot})`}>
          <rect x="-32" y="-12" width="64" height="24" rx="6" className="model-wheel" />
          <line x1="0" y1="0" x2="50" y2="0" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
          <text x="-10" y="-22" transform={`rotate(${-rot})`}>{label}</text>
        </g>
      ))}
      <line x1="248" y1="70" x2="248" y2="240" className="model-measure" />
      <text x="150" y="160">轴距 L</text>
      <line x1="285" y1="56" x2="455" y2="56" className="model-measure" />
      <text x="338" y="44">轮距 t</text>
      <text x="300" y="285" className="model-muted">δᵢ 为各轮相对车体 X 的转角（CCW 正）</text>
    </Svg>
  );
}

// 3 — slip angle geometry
function SlipAngleDiagram() {
  return (
    <Svg vb="0 0 640 300" label="侧偏角几何">
      <rect x="250" y="120" width="150" height="50" rx="6" className="model-wheel" transform="rotate(-14 325 145)" />
      {/* rolling direction */}
      <line x1="325" y1="145" x2="520" y2="97" className="model-axis" markerEnd="url(#md-arrow)" transform="rotate(0 325 145)" />
      <text x="500" y="86">轮滚动方向 (cosδ,sinδ)</text>
      {/* velocity vector */}
      <line x1="325" y1="145" x2="520" y2="160" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
      <text x="500" y="182">轮心速度 v</text>
      {/* slip angle arc */}
      <path d="M 430 121 A 110 110 0 0 1 430 158" className="model-path" fill="none" />
      <text x="450" y="146" className="model-accent">α（侧偏角）</text>
      <circle cx="325" cy="145" r="4" className="model-dot" />
      <text x="120" y="250" className="model-muted">α = 滚动方向与实际速度方向的夹角；产生侧向力 Fy</text>
    </Svg>
  );
}

// 4a — tyre Fy-alpha curve
function TireCurveDiagram() {
  return (
    <Svg vb="0 0 560 320" label="轮胎 Fy-α 曲线">
      <line x1="60" y1="270" x2="520" y2="270" className="model-axis" markerEnd="url(#md-arrow)" />
      <line x1="60" y1="270" x2="60" y2="30" className="model-axis" markerEnd="url(#md-arrow)" />
      <text x="500" y="292">α</text>
      <text x="20" y="40">|Fy|</text>
      <line x1="60" y1="80" x2="520" y2="80" className="model-measure" />
      <text x="430" y="72" className="model-accent">μ·Fz（峰值）</text>
      <path d="M60 270 Q150 110 230 90 Q330 80 520 96" className="model-path-strong" fill="none" />
      <line x1="60" y1="270" x2="220" y2="95" className="model-dash" />
      <text x="120" y="200" className="model-muted">线性区斜率 = Cα</text>
      <circle cx="230" cy="90" r="4" className="model-dot" />
      <text x="232" y="76">峰值 α_peak</text>
    </Svg>
  );
}

// 4b — friction ellipse
function FrictionEllipseDiagram() {
  return (
    <Svg vb="0 0 400 320" label="摩擦椭圆">
      <line x1="40" y1="160" x2="360" y2="160" className="model-axis" markerEnd="url(#md-arrow)" />
      <line x1="200" y1="300" x2="200" y2="20" className="model-axis" markerEnd="url(#md-arrow)" />
      <ellipse cx="200" cy="160" rx="150" ry="120" className="model-path-strong" fill="none" />
      <text x="350" y="152">Fx</text>
      <text x="208" y="30">Fy</text>
      <line x1="200" y1="160" x2="305" y2="90" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
      <text x="300" y="80" className="model-accent">合力 ≤ μ·Fz</text>
      <text x="60" y="300" className="model-muted">(Fx/μFz)² + (Fy/μFz)² ≤ 1</text>
    </Svg>
  );
}

// 5 — load transfer
function LoadTransferDiagram() {
  return (
    <Svg vb="0 0 640 300" label="载荷转移">
      <rect x="180" y="90" width="280" height="90" rx="12" className="model-car" />
      <circle cx="320" cy="120" r="5" className="model-dot" />
      <text x="330" y="118">CG (h)</text>
      <line x1="180" y1="210" x2="460" y2="210" className="model-measure" />
      <line x1="200" y1="180" x2="200" y2="240" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
      <line x1="440" y1="180" x2="440" y2="240" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
      <text x="160" y="260">后轴 Fz</text>
      <text x="410" y="260">前轴 Fz</text>
      <line x1="320" y1="120" x2="420" y2="120" className="model-axis" markerEnd="url(#md-arrow)" />
      <text x="350" y="110" className="model-accent">a_x</text>
      <text x="120" y="285" className="model-muted">纵向加速度→前后转移；横向→左右转移；气动升力→整体减 Fz</text>
    </Svg>
  );
}

// 6a — kingpin side view (caster trail)
function KingpinSideDiagram() {
  return (
    <Svg vb="0 0 560 320" label="主销几何侧视">
      <line x1="60" y1="250" x2="520" y2="250" className="model-measure" />
      <text x="64" y="270">地面</text>
      <line x1="300" y1="60" x2="240" y2="250" className="model-axis-dash" />
      <text x="305" y="70" className="model-accent">主销轴（后倾 ε）</text>
      <circle cx="300" cy="250" r="4" className="model-dot" />
      <text x="306" y="244">接地点</text>
      <line x1="240" y1="250" x2="300" y2="250" className="model-wheel-axis" />
      <text x="244" y="290" className="model-muted">机械拖距 t_m = R·tanε</text>
      <ellipse cx="360" cy="250" rx="55" ry="14" className="model-wheel" />
    </Svg>
  );
}

// 6b — kingpin top view (scrub)
function KingpinTopDiagram() {
  return (
    <Svg vb="0 0 560 280" label="主销几何俯视">
      <rect x="250" y="60" width="70" height="160" rx="8" className="model-wheel" />
      <line x1="285" y1="20" x2="285" y2="260" className="model-axis-dash" />
      <text x="292" y="34" className="model-accent">主销轴线（俯视）</text>
      <line x1="350" y1="140" x2="285" y2="140" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
      <circle cx="350" cy="140" r="4" className="model-dot" />
      <text x="356" y="136">接地中心</text>
      <text x="300" y="160" className="model-muted">主销偏置 s（scrub）</text>
      <text x="120" y="255" className="model-muted">Fy 经 (s+t_m+t_p)、Fx 经 s 产生主销力矩</text>
    </Svg>
  );
}

// 7 — force chain
function ForceChainDiagram() {
  const items = [["轮运动学", "vᵢ,αᵢ,κᵢ"], ["轮胎力", "Fx,Fy,Mz"], ["主销力矩", "τ_KP"], ["齿条", "F_rack,η"], ["电机", "T_motor"]];
  return (
    <Svg vb="0 0 900 150" label="转向负载链路">
      {items.map(([t, b], i) => {
        const x = 20 + i * 178;
        return (
          <g key={t as string}>
            <rect x={x} y="40" width="140" height="64" rx="8" className="model-chain-box" />
            <text x={x + 70} y="68" textAnchor="middle" className="model-chain-title">{t}</text>
            <text x={x + 70} y="90" textAnchor="middle">{b}</text>
            {i < items.length - 1 && (
              <line x1={x + 144} y1="72" x2={x + 174} y2="72" className="model-axis" markerEnd="url(#md-arrow)" />
            )}
          </g>
        );
      })}
    </Svg>
  );
}

// 8 — bicycle model
function BicycleDiagram() {
  return (
    <Svg vb="0 0 640 340" label="bicycle 模型">
      <line x1="320" y1="80" x2="320" y2="280" className="model-car" />
      <rect x="305" y="60" width="30" height="30" rx="4" className="model-wheel" transform="rotate(-20 320 75)" />
      <rect x="305" y="270" width="30" height="30" rx="4" className="model-wheel" />
      <circle cx="320" cy="180" r="5" className="model-dot" />
      <text x="328" y="178">CG</text>
      {/* velocity & sideslip */}
      <line x1="320" y1="180" x2="470" y2="150" className="model-wheel-axis" markerEnd="url(#md-arrow)" />
      <text x="450" y="140" className="model-accent">v（β：与 X 夹角）</text>
      <line x1="320" y1="180" x2="470" y2="180" className="model-axis-dash" />
      {/* yaw */}
      <path d="M 360 180 A 40 40 0 0 1 350 215" className="model-path" fill="none" markerEnd="url(#md-arrow)" />
      <text x="366" y="210" className="model-accent">r（横摆角速度）</text>
      <text x="120" y="320" className="model-muted">前轮转 δ → 车身发展出 β 和 r → 各轮真实 αᵢ = β + r·xᵢ/V − δᵢ</text>
    </Svg>
  );
}

// 6c — steering linkage (trapezoid)
function LinkageDiagram() {
  return (
    <Svg vb="0 0 600 280" label="齿条梯形机构">
      <line x1="80" y1="200" x2="520" y2="200" className="model-chain-box" strokeWidth={6} />
      <text x="84" y="225">齿条（沿轴平移）</text>
      <circle cx="150" cy="80" r="6" className="model-dot" />
      <text x="110" y="72">主销 K</text>
      <line x1="150" y1="80" x2="240" y2="140" className="model-axis" />
      <text x="150" y="120" className="model-accent">梯形臂 L_arm</text>
      <line x1="240" y1="140" x2="330" y2="200" className="model-wheel-axis" />
      <text x="270" y="165" className="model-accent">横拉杆</text>
      <circle cx="240" cy="140" r="5" className="model-dot" />
      <circle cx="330" cy="200" r="5" className="model-dot" />
      <text x="120" y="260" className="model-muted">F_rack = τ_KP /(L_arm·η)，η = |sin(臂-杆)|·cos(杆-齿条)·η_rack</text>
    </Svg>
  );
}

export type DiagramKey =
  | "hierarchy" | "bodyFrame" | "slipAngle" | "tireCurve" | "frictionEllipse"
  | "loadTransfer" | "kingpinSide" | "kingpinTop" | "forceChain" | "bicycle" | "linkage";

export const DIAGRAMS: Record<DiagramKey, () => JSX.Element> = {
  hierarchy: HierarchyDiagram,
  bodyFrame: BodyFrameDiagram,
  slipAngle: SlipAngleDiagram,
  tireCurve: TireCurveDiagram,
  frictionEllipse: FrictionEllipseDiagram,
  loadTransfer: LoadTransferDiagram,
  kingpinSide: KingpinSideDiagram,
  kingpinTop: KingpinTopDiagram,
  forceChain: ForceChainDiagram,
  bicycle: BicycleDiagram,
  linkage: LinkageDiagram,
};
