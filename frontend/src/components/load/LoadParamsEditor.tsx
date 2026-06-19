import { numInput } from "@/ui/styles";
import {
  ADVANCED_PARAMETER_GROUPS,
  CORE_PARAMETER_GROUPS,
  displayNumber,
  getPath,
  setPath,
  type VehicleParams,
} from "./types";

interface Props {
  params: VehicleParams | null;
  onChange: (next: VehicleParams) => void;
  onResetDefault?: () => void;
  resetDisabled?: boolean;
}

const GROUP_HINTS: Record<string, string> = {
  "对齐 / 偏置标定":
    "把这三项清零 → 单轮 δ_eq 在所有车速下回到 0°。toe 与 Cγ 都是速度无关偏置，影响 0° 保持齿条力的基线。",
  "驱动 / 气动":
    "维持车速所需的 Fx 来自这里：Fx = Crr·m·g + ½ρ·Cd·A·v²，经主销 scrub_radius 进入转向力矩，是车速依赖项的主源。",
};

export function LoadParamsEditor({ params, onChange, onResetDefault, resetDisabled }: Props) {
  return (
    <aside className="load-param-pane">
      <div className="load-param-head">
        <h2>底盘参数</h2>
        {onResetDefault && (
          <button
            className="load-param-reset"
            onClick={onResetDefault}
            disabled={resetDisabled}
            title="重置为 LS9 出厂默认值"
          >
            重置 LS9
          </button>
        )}
      </div>
      {!params && <div className="small">读取中...</div>}
      {params && (
        <>
          <div className="load-param-section-label">核心参数</div>
          {CORE_PARAMETER_GROUPS.map((group) => (
            <details key={group.title} open={group.defaultOpen}>
              <summary>{group.title}</summary>
              {GROUP_HINTS[group.title] && (
                <div className="load-param-hint">{GROUP_HINTS[group.title]}</div>
              )}
              {group.fields.map(([path, label, step, scale = 1]) => (
                <label className="load-field" key={path}>
                  <span>{label}</span>
                  <input
                    type="number"
                    step={step}
                    value={displayNumber(getPath(params, path) * scale)}
                    style={numInput}
                    onChange={(e) => onChange(setPath(params, path, Number(e.target.value) / scale))}
                  />
                </label>
              ))}
            </details>
          ))}
          <div className="load-param-section-label">高级参数</div>
          {ADVANCED_PARAMETER_GROUPS.map((group) => (
            <details key={group.title}>
              <summary>{group.title}</summary>
              {GROUP_HINTS[group.title] && (
                <div className="load-param-hint">{GROUP_HINTS[group.title]}</div>
              )}
              {group.fields.map(([path, label, step, scale = 1]) => (
                <label className="load-field" key={path}>
                  <span>{label}</span>
                  <input
                    type="number"
                    step={step}
                    value={displayNumber(getPath(params, path) * scale)}
                    style={numInput}
                    onChange={(e) => onChange(setPath(params, path, Number(e.target.value) / scale))}
                  />
                </label>
              ))}
            </details>
          ))}
        </>
      )}
    </aside>
  );
}
