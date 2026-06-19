/**
 * DisturbancePanel — GUI editing of road disturbances.
 *
 * Workflow:
 *   1. Pick a type, click 放置 → click the 2D canvas to drop regions
 *      (Esc or the button exits place mode).
 *   2. Click a region on the canvas (or in the list) to select it; the form
 *      binds to it — edits PUT immediately on 应用. Drag the selected region
 *      to move it.
 *   3. Delete per-row or 清空全部. Save the project to persist the scene.
 */

import { useEffect, useState } from "react";

import { clearDisturbances, deleteDisturbance, updateDisturbance } from "@/api/scene";
import type { DistType } from "@/store/sim";
import { useSimStore } from "@/store/sim";
import type { DisturbanceMsg } from "@/types/sim";
import { activeBtn, numInput, selectStyle } from "@/ui/styles";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

const TYPE_LABELS: Record<DistType, string> = {
  ice_patch: "冰面 / 低附着",
  split_mu: "对开路面",
  speed_bump: "减速带",
  slope: "坡道",
};

// Editable numeric fields per type: [key, label, step]
const COMMON_FIELDS: [string, string, number][] = [
  ["x", "中心 X (m)", 1],
  ["y", "中心 Y (m)", 1],
  ["length", "长度 (m)", 1],
  ["width", "宽度 (m)", 1],
  ["heading", "朝向 (rad)", 0.05],
];
const TYPE_FIELDS: Record<DistType, [string, string, number][]> = {
  ice_patch: [["mu", "μ", 0.05]],
  split_mu: [["mu_left", "左侧 μ", 0.05], ["mu_right", "右侧 μ", 0.05]],
  speed_bump: [["height", "高度 (m)", 0.01], ["stiffness", "刚度 (N/m)", 10000]],
  slope: [["angle", "坡度角 (rad)", 0.01]],
};

export default function DisturbancePanel() {
  const state = useSimStore((s) => s.state);
  const placeType = useSimStore((s) => s.distPlaceType);
  const setPlaceType = useSimStore((s) => s.setDistPlaceType);
  const selectedId = useSimStore((s) => s.selectedDistId);
  const setSelectedId = useSimStore((s) => s.setSelectedDistId);
  const pushToast = useSimStore((s) => s.pushToast);

  const [type, setType] = useState<DistType>("ice_patch");
  const [form, setForm] = useState<Record<string, number>>({});

  const disturbances = state?.scene?.disturbances ?? [];
  const selected = disturbances.find((d) => d.id === selectedId) ?? null;

  // Bind the form to the selected disturbance.
  useEffect(() => {
    if (!selected) return;
    const f: Record<string, number> = {};
    for (const [k] of [...COMMON_FIELDS, ...(TYPE_FIELDS[selected.type as DistType] ?? [])]) {
      const v = (selected as unknown as Record<string, unknown>)[k];
      if (typeof v === "number") f[k] = v;
    }
    setForm(f);
  }, [selectedId, selected && JSON.stringify(selected)]); // eslint-disable-line react-hooks/exhaustive-deps

  const apply = async () => {
    if (!selected) return;
    try {
      await updateDisturbance(selected.id, form);
      pushToast("info", `已更新 ${selected.id}`);
    } catch (e: any) {
      pushToast("error", `更新失败：${e?.message ?? e}`);
    }
  };

  const remove = async (id: string) => {
    try {
      await deleteDisturbance(id);
      if (selectedId === id) setSelectedId(null);
    } catch (e: any) {
      pushToast("error", `删除失败：${e?.message ?? e}`);
    }
  };

  const clearAll = async () => {
    try {
      const r = await clearDisturbances();
      setSelectedId(null);
      pushToast("info", `已清空 ${r.removed} 个扰动`);
    } catch (e: any) {
      pushToast("error", `清空失败：${e?.message ?? e}`);
    }
  };

  const fields = selected
    ? [...COMMON_FIELDS, ...(TYPE_FIELDS[selected.type as DistType] ?? [])]
    : [];

  return (
    <Panel title="路面扰动编辑" help={HELP.disturbance}>
      <div style={{ display: "flex", gap: 6 }}>
        <select
          value={type}
          onChange={(e) => {
            const t = e.target.value as DistType;
            setType(t);
            if (placeType) setPlaceType(t);  // hot-swap while placing
          }}
          style={selectStyle}
        >
          {(Object.keys(TYPE_LABELS) as DistType[]).map((t) => (
            <option key={t} value={t}>{TYPE_LABELS[t]}</option>
          ))}
        </select>
        <button
          onClick={() => setPlaceType(placeType ? null : type)}
          style={placeType ? activeBtn : undefined}
        >
          {placeType ? "放置中…(Esc 退出)" : "放置"}
        </button>
      </div>

      {placeType && (
        <div className="small" style={{ color: "var(--muted)", marginTop: 4 }}>
          在 2D 画布上点击放置「{TYPE_LABELS[placeType]}」，可连续放置
        </div>
      )}

      {/* Existing disturbances */}
      {disturbances.length > 0 && (
        <div style={{ marginTop: 8 }}>
          {disturbances.map((d) => (
            <DistRow
              key={d.id}
              d={d}
              selected={d.id === selectedId}
              onSelect={() => setSelectedId(d.id === selectedId ? null : d.id)}
              onDelete={() => remove(d.id)}
            />
          ))}
          <button className="small" onClick={clearAll} style={{ marginTop: 4 }}>清空全部</button>
        </div>
      )}
      {disturbances.length === 0 && (
        <div className="small" style={{ color: "var(--muted)", marginTop: 6 }}>
          无扰动 — 选择类型后点「放置」在画布上添加
        </div>
      )}

      {/* Editor form for the selection */}
      {selected && (
        <div style={{ marginTop: 8, borderTop: "1px solid var(--border)", paddingTop: 8 }}>
          <div className="small" style={{ marginBottom: 6 }}>
            编辑 <span className="mono">{selected.id}</span>（可在画布上拖动）
          </div>
          {fields.map(([k, label, step]) => (
            <div key={k} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4 }}>
              <span className="small" style={{ color: "var(--muted)", width: 92 }}>{label}</span>
              <input
                type="number"
                step={step}
                value={form[k] ?? 0}
                onChange={(e) => setForm({ ...form, [k]: Number(e.target.value) })}
                style={numInput}
              />
            </div>
          ))}
          <button onClick={apply}>应用修改</button>
        </div>
      )}
    </Panel>
  );
}

function DistRow({
  d, selected, onSelect, onDelete,
}: { d: DisturbanceMsg; selected: boolean; onSelect: () => void; onDelete: () => void }) {
  const label = TYPE_LABELS[d.type as DistType] ?? d.type;
  return (
    <div
      style={{
        display: "flex", alignItems: "center", gap: 6, padding: "3px 6px",
        borderRadius: 6, cursor: "pointer", fontSize: 12,
        background: selected ? "rgba(56,189,248,0.15)" : undefined,
        border: selected ? "1px solid rgba(56,189,248,0.5)" : "1px solid transparent",
      }}
      onClick={onSelect}
    >
      <span>{label}</span>
      <span className="mono small" style={{ color: "var(--muted)" }}>
        ({d.x.toFixed(0)}, {d.y.toFixed(0)})
      </span>
      <button
        style={{ marginLeft: "auto", fontSize: 11 }}
        onClick={(e) => { e.stopPropagation(); onDelete(); }}
      >
        删除
      </button>
    </div>
  );
}
