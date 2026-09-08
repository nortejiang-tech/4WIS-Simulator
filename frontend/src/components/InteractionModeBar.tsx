import "./InteractionModeBar.css";

export type InteractionMode = "manual" | "script" | "agent";

interface InteractionModeBarProps {
  mode: InteractionMode | null;
  onChange: (mode: InteractionMode) => void;
  sourceLabel: string;
  connected: boolean;
  onStop?: () => void;
}

export default function InteractionModeBar({
  mode,
  onChange,
  sourceLabel,
  connected,
  onStop,
}: InteractionModeBarProps) {
  const modes: InteractionMode[] = ["manual", "script", "agent"];
  const modeLabels: Record<InteractionMode, string> = {
    manual: "手动驾驶",
    script: "脚本工况",
    agent: "AI Agent",
  };

  return (
    <nav
      className="interaction-bar"
      aria-label="交互方式"
    >
      <div className="interaction-bar__group interaction-bar__group--mode">
        {modes.map((m) => (
          <button
            key={m}
            type="button"
            className={`interaction-bar__button interaction-bar__button--${m}${
              mode === m ? " interaction-bar__button--active" : ""
            }`}
            aria-pressed={mode === m}
            onClick={() => onChange(m)}
          >
            {modeLabels[m]}
          </button>
        ))}
      </div>

      <div className="interaction-bar__group interaction-bar__group--status">
        <span className="interaction-bar__label">控制来源</span>
        <span className="interaction-bar__source">{sourceLabel}</span>
        <span
          className={`interaction-bar__status interaction-bar__status--${
            connected ? "connected" : "disconnected"
          }`}
        >
          {connected ? "已连接" : "未连接"}
        </span>
        {onStop && (
          <button
            type="button"
            className="interaction-bar__button interaction-bar__button--stop"
            onClick={onStop}
          >
            停止输入
          </button>
        )}
      </div>
    </nav>
  );
}
