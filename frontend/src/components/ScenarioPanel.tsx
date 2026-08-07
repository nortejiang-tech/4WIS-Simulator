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

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

export default function ScenarioPanel() {
  const [list, setList] = useState<{ name: string; label: string }[]>([]);
  const active = useSimStore((s) => s.scenario?.name ?? null);
  const spawns = useSimStore((s) => s.scenario?.spawns ?? []);
  const pushToast = useSimStore((s) => s.pushToast);
  const [busy, setBusy] = useState(false);
  const [listError, setListError] = useState<string | null>(null);

  const refreshList = () => fetchScenarios()
    .then((nextList) => {
      setList(nextList);
      setListError(null);
    })
    .catch((e) => {
      setList([]);
      const message = `读取场景列表失败：${formatError(e)}`;
      setListError(message);
      pushToast("error", message);
    });

  useEffect(() => {
    refreshList();
  }, []);

  const pick = async (name: string) => {
    setBusy(true);
    try {
      await loadScenario(name);
    } catch (e) {
      pushToast("error", `加载场景失败：${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };
  // Teleport to a named spawn of the already-loaded scenario.
  const pickSpawn = async (spawnName: string) => {
    if (!active) return;
    setBusy(true);
    try {
      await loadScenario(active, spawnName);
    } catch (e) {
      pushToast("error", `切换出生点失败：${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };
  const clear = async () => {
    setBusy(true);
    try {
      await clearScenario();
    } catch (e) {
      pushToast("error", `清除场景失败：${formatError(e)}`);
    } finally {
      setBusy(false);
    }
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
        {listError && (
          <div className="panel-small" role="alert" style={{ color: "var(--bad)", gridColumn: "1 / -1" }}>
            {listError}
          </div>
        )}
      </div>
      {spawns.length > 1 && (
        <div style={{ marginTop: 6 }}>
          <span className="panel-small" style={{ color: "var(--muted)", marginRight: 6 }}>出生点：</span>
          <div className="strategy-buttons" style={{ marginTop: 2 }}>
            {spawns.map((sp) => (
              <button key={sp.name} className="panel-small" disabled={busy}
                onClick={() => pickSpawn(sp.name)} title={`x=${sp.x.toFixed(0)} y=${sp.y.toFixed(0)}`}>
                {sp.name}
              </button>
            ))}
          </div>
        </div>
      )}
      <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 6 }}>
        <span className="panel-small" style={{ color: "var(--muted)" }}>
          当前：{active ? (list.find((s) => s.name === active)?.label ?? active) : "无（空网格）"}
        </span>
        <button style={{ marginLeft: "auto" }} disabled={busy || !active} onClick={clear}>清除场景</button>
        <button disabled={busy} onClick={refreshList}>刷新场景</button>
      </div>
      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 4, lineHeight: 1.5 }}>
        加载后车辆会移到场景起点；2D / 3D 同步渲染。城市路口按中国标准画车道/停止线/斑马线/红绿灯。
      </div>
    </Panel>
  );
}
