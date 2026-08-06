/**
 * TrajectoryPanel — generate standard maneuvers, draw custom waypoint paths,
 * and engage the pure-pursuit `follow_trajectory` strategy.
 *
 * The reference path lives on the backend (single source of truth); this panel
 * mutates it over REST. The cones and painted marks that come back with a plan
 * double as manual-driving markers, so a course is useful whether you auto-
 * follow it or drive it by hand with a wheel — which is why the panel echoes
 * the geometry the backend actually laid out (`notes`) instead of just a name.
 *
 * Tunable parameters come from `/api/path/templates` (`specs`), so adding a
 * maneuver on the backend gets it a UI for free. The vehicle-derived numbers
 * (ISO lane widths, parking-slot size) are injected server-side and are not
 * shown as knobs.
 */

import { useEffect, useMemo, useState } from "react";

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

interface ParamSpec {
  key: string;
  label: string;
  default: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
}

interface TemplateSpec {
  name: string;
  label: string;
  desc: string;
  params: ParamSpec[];
}

// Fallback labels for a backend that predates the `specs` payload.
const TEMPLATE_LABELS: Record<string, string> = {
  straight: "直线",
  arc: "圆弧",
  slalom: "绕桩 (Slalom)",
  lane_change: "单移线",
  double_lane_change: "双移线 · ISO 3888-2 (麋鹿)",
  iso3888_1: "双移线 · ISO 3888-1",
  skidpad: "定圆 (ISO 4138)",
  figure_eight: "八字",
  parking: "侧方停车",
};

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

export default function TrajectoryPanel() {
  const [templates, setTemplates] = useState<string[]>([]);
  const [specs, setSpecs] = useState<Record<string, TemplateSpec>>({});
  const [selected, setSelected] = useState("slalom");
  const [params, setParams] = useState<Record<string, number>>({});
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
    fetchJSON<{ templates: string[]; specs?: TemplateSpec[] }>("/api/path/templates")
      .then((d) => {
        const nextTemplates = d.templates ?? [];
        setTemplates(nextTemplates);
        setSpecs(Object.fromEntries((d.specs ?? []).map((s) => [s.name, s])));
        setTemplatesError(null);
        setSelected((current) => nextTemplates.includes(current) ? current : nextTemplates[0] ?? "");
      })
      .catch((e) => {
        setTemplates([]);
        setSpecs({});
        setSelected("");
        setTemplatesError(`读取路径模板失败：${formatError(e)}`);
      });
  };

  useEffect(() => {
    refreshTemplates();
  }, []);

  const spec = specs[selected];

  // Switching maneuver resets the knobs to that maneuver's own defaults.
  useEffect(() => {
    setParams(Object.fromEntries((spec?.params ?? []).map((p) => [p.key, p.default])));
  }, [spec]);

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

  const onGenerate = () => wrap(() => setPathTemplate(selected, params));
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
  const label = (t: string) => specs[t]?.label ?? TEMPLATE_LABELS[t] ?? t;
  const dirty = useMemo(
    () => (spec?.params ?? []).some((p) => params[p.key] !== undefined && params[p.key] !== p.default),
    [spec, params],
  );

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
            <option key={t} value={t}>{label(t)}</option>
          ))}
        </select>
        <button onClick={onGenerate} disabled={busy || templates.length === 0}>生成</button>
        <button onClick={refreshTemplates} disabled={busy}>刷新模板</button>
      </div>

      {spec?.desc && (
        <div className="panel-small" style={{ color: "var(--muted)", marginTop: 4, lineHeight: 1.5 }}>
          {spec.desc}
        </div>
      )}

      {/* Per-maneuver knobs. Vehicle-derived geometry (ISO lane widths, slot
          size) is filled in by the backend and deliberately not editable here. */}
      {(spec?.params?.length ?? 0) > 0 && (
        <div
          style={{
            display: "grid",
            gridTemplateColumns: "repeat(auto-fit, minmax(112px, 1fr))",
            gap: 6,
            marginTop: 6,
          }}
        >
          {spec!.params.map((p) => (
            <label key={p.key} style={{ display: "flex", alignItems: "center", gap: 4 }}>
              <span className="panel-small" style={{ color: "var(--muted)", flex: "1 1 auto" }}>
                {p.label}
              </span>
              <input
                type="number"
                aria-label={`${spec!.label} ${p.label}`}
                min={p.min} max={p.max} step={p.step}
                value={params[p.key] ?? p.default}
                onChange={(e) => setParams({ ...params, [p.key]: Number(e.target.value) })}
                style={{ ...inputStyle, width: 56, flex: "0 0 auto" }}
                disabled={busy}
              />
              {p.unit && (
                <span className="panel-small" style={{ color: "var(--muted)" }}>{p.unit}</span>
              )}
            </label>
          ))}
        </div>
      )}
      {dirty && (
        <button
          className="panel-small"
          style={{ marginTop: 4, alignSelf: "flex-start" }}
          disabled={busy}
          onClick={() =>
            setParams(Object.fromEntries((spec?.params ?? []).map((p) => [p.key, p.default])))
          }
        >
          恢复默认参数
        </button>
      )}

      <div style={{ display: "flex", gap: 6, marginTop: 6, alignItems: "center" }}>
        <span className="panel-small" style={{ color: "var(--muted)" }}>巡航速度</span>
        <input
          type="number" min={0} max={120} step={1}
          aria-label="巡航速度"
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
      {/* What the backend actually laid out — lane widths, spacings, offsets. */}
      {pointCount > 0 && path?.notes && (
        <div className="panel-small" style={{ color: "var(--muted)", marginTop: 2, lineHeight: 1.5 }}>
          {path.notes}
        </div>
      )}

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
