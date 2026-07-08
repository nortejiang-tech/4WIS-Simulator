/**
 * TrajectoryPanel — generate standard maneuvers, draw custom waypoint paths,
 * and engage the pure-pursuit `follow_trajectory` strategy.
 *
 * The reference path lives on the backend (single source of truth); this panel
 * mutates it over REST. Ground cones for slalom / DLC / parking double as
 * manual-driving markers, so the path is useful whether you auto-follow it or
 * drive it by hand.
 */

import { useEffect, useState } from "react";

import {
  clearPath,
  setDriverMode,
  setPathTemplate,
  setPathWaypoints,
  setStrategy,
} from "@/api/ws";
import { fetchJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import { activeBtn, inputStyle, selectStyle } from "@/ui/styles";
import { fromKmh } from "@/ui/units";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

const TEMPLATE_LABELS: Record<string, string> = {
  straight: "直线",
  arc: "圆弧",
  slalom: "绕桩 (Slalom)",
  double_lane_change: "双移线 (DLC)",
  figure_eight: "八字",
  parking: "停车入位",
};

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

export default function TrajectoryPanel() {
  const [templates, setTemplates] = useState<string[]>([]);
  const [selected, setSelected] = useState("slalom");
  const [cruiseKmh, setCruiseKmh] = useState(20);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [templatesError, setTemplatesError] = useState<string | null>(null);

  const path = useSimStore((s) => s.path);
  const editMode = useSimStore((s) => s.editMode);
  const draft = useSimStore((s) => s.draftWaypoints);
  const setEditMode = useSimStore((s) => s.setEditMode);
  const clearDraft = useSimStore((s) => s.clearDraft);

  const refreshTemplates = () => {
    fetchJSON<{ templates: string[] }>("/api/path/templates")
      .then((d) => {
        const nextTemplates = d.templates ?? [];
        setTemplates(nextTemplates);
        setTemplatesError(null);
        setSelected((current) => nextTemplates.includes(current) ? current : nextTemplates[0] ?? "");
      })
      .catch((e) => {
        setTemplates([]);
        setSelected("");
        setTemplatesError(`读取路径模板失败：${formatError(e)}`);
      });
  };

  useEffect(() => {
    refreshTemplates();
  }, []);

  const wrap = async (fn: () => Promise<void>) => {
    setBusy(true);
    setError(null);
    try {
      await fn();
    } catch (e) {
      setError(formatError(e));
    } finally {
      setBusy(false);
    }
  };

  const onGenerate = () => wrap(() => setPathTemplate(selected, {}));
  const onClear = () => wrap(async () => { await clearPath(); });

  const onFollow = () => {
    setStrategy("follow_trajectory");
    setDriverMode({ cruise_speed: fromKmh(cruiseKmh) });
  };

  const onFinishDraw = () =>
    wrap(async () => {
      if (draft.length >= 2) await setPathWaypoints(draft, false);
      clearDraft();
      setEditMode(false);
    });

  const pointCount = path?.points.length ?? 0;
  const coneCount = path?.cones.length ?? 0;

  return (
    <Panel title="轨迹 / 路径" help={HELP.trajectory}>
      <div style={{ display: "flex", gap: 6 }}>
        <select
          value={selected}
          onChange={(e) => setSelected(e.target.value)}
          disabled={busy || templates.length === 0}
          style={selectStyle}
        >
          {templates.length === 0 && <option value="">（无模板）</option>}
          {templates.map((t) => (
            <option key={t} value={t}>{TEMPLATE_LABELS[t] ?? t}</option>
          ))}
        </select>
        <button onClick={onGenerate} disabled={busy || templates.length === 0}>生成</button>
        <button onClick={refreshTemplates} disabled={busy}>刷新模板</button>
      </div>

      <div style={{ display: "flex", gap: 6, marginTop: 6, alignItems: "center" }}>
        <span className="panel-small" style={{ color: "var(--muted)" }}>巡航速度</span>
        <input
          type="number" min={0} max={120} step={1}
          value={cruiseKmh}
          onChange={(e) => setCruiseKmh(Number(e.target.value))}
          style={{ ...inputStyle, width: 64, flex: "0 0 auto" }}
          disabled={busy}
        />
        <span className="panel-small" style={{ color: "var(--muted)" }}>km/h</span>
        <button onClick={onFollow} disabled={busy || pointCount < 2} style={{ marginLeft: "auto" }}>
          跟踪此路径
        </button>
      </div>

      <div style={{ display: "flex", gap: 6, marginTop: 8 }}>
        <button
          onClick={() => setEditMode(!editMode)}
          disabled={busy}
          style={editMode ? activeBtn : undefined}
        >
          {editMode ? "绘制中…点击画布" : "手动绘制"}
        </button>
        {editMode && (
          <>
            <button onClick={onFinishDraw} disabled={busy || draft.length < 2}>完成 ({draft.length})</button>
            <button onClick={clearDraft} disabled={busy || draft.length === 0}>撤销全部</button>
          </>
        )}
        <button onClick={onClear} disabled={busy || pointCount === 0} style={{ marginLeft: "auto" }}>
          清除路径
        </button>
      </div>

      <div className="panel-small" style={{ color: "var(--muted)", marginTop: 6 }}>
        {pointCount > 0
          ? `当前路径: ${path?.name || "custom"} · ${pointCount} 点 · ${coneCount} 桩`
          : "无路径"}
        {editMode && " · 在 2D 画布上点击放置航点"}
      </div>

      {error && (
        <div className="panel-small" role="alert" style={{ color: "var(--bad)", marginTop: 4 }}>{error}</div>
      )}
      {templatesError && (
        <div className="panel-small" role="alert" style={{ color: "var(--bad)", marginTop: 4 }}>
          {templatesError}
        </div>
      )}
    </Panel>
  );
}
