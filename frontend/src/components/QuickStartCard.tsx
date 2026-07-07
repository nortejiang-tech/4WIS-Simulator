import "./QuickStartCard.css";

interface Step {
  n: number;
  title: string;
  desc: string;
  action?: { label: string; run: () => void };
}

interface Props {
  onClose: () => void;
  onGoTab: (tab: string) => void;
  onGoPage: (page: string) => void;
}

/**
 * First-visit onboarding card for the sim workbench. Lays out the recommended
 * workflow (选车型 → 选策略 → 看 2D → 切 3D / 验证) with one-click jumps so a
 * new user knows where to start instead of facing a "button sea". Dismissal is
 * persisted by the caller (localStorage); the header "?" re-opens it.
 */
export default function QuickStartCard({ onClose, onGoTab, onGoPage }: Props) {
  const steps: Step[] = [
    {
      n: 1,
      title: "选车型",
      desc: "在「场景」里调车辆/悬架参数，或到「负载特性」页选预置车型。",
      action: { label: "去负载特性", run: () => onGoPage("load") },
    },
    {
      n: 2,
      title: "选策略",
      desc: "在「驾驶」里选转向策略（前轮 / 后轮反相 / 4WIS…）与油门。",
      action: { label: "去驾驶", run: () => onGoTab("drive") },
    },
    {
      n: 3,
      title: "看 2D 轨迹",
      desc: "视图区默认 2D 俯视，用 W A S D / 方向键驾驶，观察四轮转角与轨迹。",
    },
    {
      n: 4,
      title: "切 3D / 验证",
      desc: "视图右上可切 3D；到「验证」做开环激励、评分与 A/B 对比。",
      action: { label: "去验证", run: () => onGoTab("validate") },
    },
  ];

  return (
    <div className="quickstart-card" role="region" aria-label="快速开始">
      <div className="quickstart-head">
        <span className="quickstart-title">快速开始</span>
        <span className="quickstart-sub">第一次用？按这四步走</span>
        <button className="quickstart-close" onClick={onClose} title="关闭（表头 ? 可重新打开）" aria-label="关闭快速开始">
          ✕
        </button>
      </div>
      <ol className="quickstart-steps">
        {steps.map((s) => (
          <li key={s.n} className="quickstart-step">
            <span className="quickstart-num">{s.n}</span>
            <div className="quickstart-body">
              <div className="quickstart-step-title">{s.title}</div>
              <div className="quickstart-step-desc">{s.desc}</div>
              {s.action && (
                <button className="quickstart-jump" onClick={s.action.run}>
                  {s.action.label} →
                </button>
              )}
            </div>
          </li>
        ))}
      </ol>
    </div>
  );
}
