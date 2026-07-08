/**
 * ParamsPanel — numeric editor for vehicle / suspension / tire parameters.
 *
 * Reads and writes through the shared VehicleParamsContext, so edits here and
 * drags on the geometry diagrams feed one buffer and commit with one "应用".
 */

import { numInput, selectStyle } from "@/ui/styles";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";
import { useVehicleParams } from "@/components/vehicle/VehicleParamsContext";
import {
  ADVANCED_PARAMETER_GROUPS,
  CORE_PARAMETER_GROUPS,
  type ParameterGroup,
} from "@/vehicle/parameterGroups";

const displayNumber = (n: number) => {
  if (!Number.isFinite(n)) return "";
  return String(Number(n.toFixed(3)));
};

export default function ParamsPanel() {
  const { params, value, setValue, tireModel, setTireModel, dirty, busy, apply, reload } = useVehicleParams();

  if (!params) {
    return (
      <Panel title="车辆 / 悬架参数" help={HELP.params}>
        <div className="panel-small" style={{ color: "var(--muted)" }}>读取中…</div>
      </Panel>
    );
  }

  const renderGroup = (g: ParameterGroup) => (
    <details key={g.title} style={{ marginBottom: 4 }} open={g.defaultOpen}>
      <summary className="panel-small" style={{ cursor: "pointer", color: "var(--text)" }}>
        {g.title}
      </summary>
      <div style={{ padding: "4px 0 4px 8px" }}>
        {g.fields.map(([k, label, step, scale = 1]) => (
          <div key={k} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4 }}>
            <span className="panel-small" style={{ color: "var(--muted)", width: 130 }}>{label}</span>
            <input
              type="number"
              aria-label={label}
              data-param-key={k}
              step={step}
              value={displayNumber(value(k) * scale)}
              onChange={(e) => setValue(k, Number(e.target.value) / scale)}
              style={numInput}
            />
          </div>
        ))}
      </div>
    </details>
  );

  return (
    <Panel title="车辆 / 悬架参数" help={HELP.params}>
      <>
        <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 6 }}>
          <span className="panel-small" style={{ color: "var(--muted)", width: 110 }}>轮胎模型</span>
          <select aria-label="轮胎模型" value={tireModel} onChange={(e) => setTireModel(e.target.value)} style={selectStyle}>
            <option value="linear">线性 + 摩擦圆</option>
            <option value="pacejka">Pacejka (Magic Formula)</option>
          </select>
        </div>

        <div className="panel-small" style={{ color: "var(--muted)", margin: "4px 0" }}>核心参数</div>
        {CORE_PARAMETER_GROUPS.map((g) => renderGroup(g))}

        <div className="panel-small" style={{ color: "var(--muted)", margin: "8px 0 4px" }}>高级参数</div>
        {ADVANCED_PARAMETER_GROUPS.map((g) => renderGroup(g))}

        <div style={{ display: "flex", gap: 6, marginTop: 6, alignItems: "center" }}>
          <button onClick={apply} disabled={!dirty || busy}>应用参数</button>
          <button onClick={reload} disabled={busy}>还原</button>
          <span className="panel-small" style={{ color: "var(--muted)" }}>应用会重置动力学状态</span>
        </div>
        <div className="panel-small" style={{ color: "var(--muted)", marginTop: 4 }}>
          提示：也可直接拖动右侧示意图上的点来改几何参数
        </div>
      </>
    </Panel>
  );
}
