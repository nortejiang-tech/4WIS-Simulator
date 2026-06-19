/**
 * ScenarioPanel — pick a static driving scenario (plaza / city / tracks).
 *
 * Loading a scenario fetches its geometry once and teleports the vehicle to the
 * scenario spawn pose; "清除" removes it and returns to the empty grid.
 */

import { useEffect, useState } from "react";

import { fetchScenarios, loadScenario, clearScenario } from "@/api/ws";
import { useSimStore } from "@/store/sim";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

export default function ScenarioPanel() {
  const [list, setList] = useState<{ name: string; label: string }[]>([]);
  const active = useSimStore((s) => s.scenario?.name ?? null);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    fetchScenarios().then(setList).catch(() => setList([]));
  }, []);

  const pick = async (name: string) => {
    setBusy(true);
    try { await loadScenario(name); } finally { setBusy(false); }
  };
  const clear = async () => {
    setBusy(true);
    try { await clearScenario(); } finally { setBusy(false); }
  };

  return (
    <Panel title="场景路况" help={HELP.scenario}>
      <div className="strategy-buttons">
        {list.map((s) => (
          <button key={s.name} className={s.name === active ? "active" : ""}
            disabled={busy} onClick={() => pick(s.name)}>
            {s.label}
          </button>
        ))}
      </div>
      <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 6 }}>
        <span className="small" style={{ color: "var(--muted)" }}>
          当前：{active ? (list.find((s) => s.name === active)?.label ?? active) : "无（空网格）"}
        </span>
        <button style={{ marginLeft: "auto" }} disabled={busy || !active} onClick={clear}>清除场景</button>
      </div>
      <div className="small" style={{ color: "var(--muted)", marginTop: 4, lineHeight: 1.5 }}>
        加载后车辆会移到场景起点；2D / 3D 同步渲染。城市路口按中国标准画车道/停止线/斑马线/红绿灯。
      </div>
    </Panel>
  );
}
