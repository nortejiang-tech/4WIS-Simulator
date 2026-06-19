/**
 * ParamsPanel — edit vehicle / suspension / tire parameters live.
 *
 * GET /api/params returns the full parameter dict; edits are collected
 * locally and POSTed as a partial update (the backend validates ranges and
 * returns 422 with details on bad values). Applying parameters rebuilds the
 * model — the vehicle's dynamic state resets in place.
 */

import { useEffect, useState } from "react";

import { fetchJSON, postJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import { numInput, selectStyle } from "@/ui/styles";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";
import {
  ADVANCED_PARAMETER_GROUPS,
  CORE_PARAMETER_GROUPS,
  type ParameterGroup,
} from "@/vehicle/parameterGroups";

type Params = Record<string, any> & {
  suspension?: Record<string, number>;
  steering_geometry?: Record<string, number>;
};

const displayNumber = (n: number) => {
  if (!Number.isFinite(n)) return "";
  return String(Number(n.toFixed(3)));
};

const getPath = (obj: Params | null, path: string): number => {
  if (!obj) return 0;
  return path.split(".").reduce<any>((acc, key) => acc?.[key], obj) ?? 0;
};

const setNested = (target: Record<string, any>, path: string, value: number) => {
  const keys = path.split(".");
  let cur = target;
  for (let i = 0; i < keys.length - 1; i++) {
    cur[keys[i]] = { ...(cur[keys[i]] ?? {}) };
    cur = cur[keys[i]];
  }
  cur[keys[keys.length - 1]] = value;
};

export default function ParamsPanel() {
  const pushToast = useSimStore((s) => s.pushToast);
  const [params, setParams] = useState<Params | null>(null);
  const [edits, setEdits] = useState<Record<string, number>>({});
  const [tireModel, setTireModel] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const load = () =>
    fetchJSON<Params>("/api/params")
      .then((p) => {
        setParams(p);
        setEdits({});
        setTireModel(null);
      })
      .catch((e) => pushToast("error", `读取参数失败：${e?.message ?? e}`));

  useEffect(() => { load(); }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const dirty = Object.keys(edits).length > 0 || tireModel != null;

  const apply = async () => {
    setBusy(true);
    try {
      const body: Record<string, unknown> = {};
      Object.entries(edits).forEach(([path, nextValue]) => setNested(body, path, nextValue));
      if (tireModel != null) body.tire_model = tireModel;
      const updated = await postJSON<Params>("/api/params", body);
      setParams(updated);
      setEdits({});
      setTireModel(null);
      pushToast("info", "参数已应用（车辆动力学状态已重置）");
    } catch (e: any) {
      pushToast("error", `参数被拒绝：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  };

  if (!params) {
    return (
      <Panel title="车辆 / 悬架参数" help={HELP.params} defaultOpen={false}>
        <div className="small" style={{ color: "var(--muted)" }}>读取中…</div>
      </Panel>
    );
  }

  const value = (path: string): number => edits[path] ?? getPath(params, path);
  const setValue = (path: string, v: number) => setEdits({ ...edits, [path]: v });

  const currentTireModel = tireModel ?? (params.tire_model as string) ?? "linear";
  const renderGroup = (g: ParameterGroup, defaultOpen = false) => (
    <details key={g.title} style={{ marginBottom: 4 }} open={defaultOpen || g.defaultOpen}>
      <summary className="small" style={{ cursor: "pointer", color: "var(--text)" }}>
        {g.title}
      </summary>
      <div style={{ padding: "4px 0 4px 8px" }}>
        {g.fields.map(([k, label, step, scale = 1]) => (
          <div key={k} style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 4 }}>
            <span className="small" style={{ color: "var(--muted)", width: 130 }}>{label}</span>
            <input
              type="number"
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
    <Panel title="车辆 / 悬架参数" help={HELP.params} defaultOpen={false}>
      <>
          {/* Tire model selector */}
          <div style={{ display: "flex", alignItems: "center", gap: 6, marginBottom: 6 }}>
            <span className="small" style={{ color: "var(--muted)", width: 110 }}>轮胎模型</span>
            <select
              value={currentTireModel}
              onChange={(e) => setTireModel(e.target.value)}
              style={selectStyle}
            >
              <option value="linear">线性 + 摩擦圆</option>
              <option value="pacejka">Pacejka (Magic Formula)</option>
            </select>
          </div>

          <div className="small" style={{ color: "var(--muted)", margin: "4px 0" }}>核心参数</div>
          {CORE_PARAMETER_GROUPS.map((g) => renderGroup(g))}

          <div className="small" style={{ color: "var(--muted)", margin: "8px 0 4px" }}>高级参数</div>
          {ADVANCED_PARAMETER_GROUPS.map((g) => renderGroup(g))}

          <div style={{ display: "flex", gap: 6, marginTop: 6, alignItems: "center" }}>
            <button onClick={apply} disabled={!dirty || busy}>应用参数</button>
            <button onClick={load} disabled={busy}>还原</button>
            <span className="small" style={{ color: "var(--muted)" }}>
              应用会重置动力学状态
            </span>
          </div>
          <div className="small" style={{ color: "var(--muted)", marginTop: 4 }}>
            提示：在「项目」面板保存，可将参数与扰动一起写入项目 YAML
          </div>
      </>
    </Panel>
  );
}
