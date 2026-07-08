/**
 * ProjectPanel — list/load/save YAML projects.
 *
 * Phase 1 keeps this minimal: a dropdown showing all projects under
 * <repo>/projects/, a "Load" button, and a save form with a name input.
 * Phase 2 will add description editing and import/export.
 */

import { useCallback, useEffect, useState } from "react";

import { resetSim } from "@/api/ws";
import { fetchJSON } from "@/api/http";
import { inputStyle as sharedInput, selectStyle } from "@/ui/styles";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

const inputStyle = { ...sharedInput, flex: 1 } as const;

export default function ProjectPanel() {
  const [projects, setProjects] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [saveName, setSaveName] = useState("");
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [info, setInfo] = useState<string | null>(null);

  const refresh = useCallback(() => {
    fetchJSON<{ projects: string[] }>("/api/projects")
      .then((d) => {
        setProjects(d.projects);
        setError(null);
        if (!selected && d.projects.length) setSelected(d.projects[0]);
      })
      .catch((e) => {
        setProjects([]);
        setSelected("");
        setError(`读取项目列表失败：${String(e?.message ?? e)}`);
      });
  }, [selected]);

  useEffect(() => { refresh() }, [refresh]);

  const onLoad = async () => {
    if (!selected) return;
    setBusy(true);
    setError(null);
    setInfo(null);
    try {
      const r = await fetchJSON<{ status: string; strategy: string }>(
        `/api/projects/${encodeURIComponent(selected)}/load`,
        { method: "POST" },
      );
      resetSim();
      setInfo(`已加载 ${selected} (策略: ${r.strategy})`);
    } catch (e: any) {
      setError(String(e?.message ?? e));
    } finally {
      setBusy(false);
    }
  };

  const onSave = async () => {
    const name = saveName.trim();
    if (!name) {
      setError("请输入项目名");
      return;
    }
    setBusy(true);
    setError(null);
    setInfo(null);
    try {
      await fetchJSON(`/api/projects/${encodeURIComponent(name)}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, description: "" }),
      });
      setInfo(`已保存 ${name}.yaml`);
      setSaveName("");
      refresh();
    } catch (e: any) {
      setError(String(e?.message ?? e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <Panel title="项目（YAML）" help={HELP.project}>
      <div style={{ display: "flex", gap: 6 }}>
        <select
          value={selected}
          onChange={(e) => setSelected(e.target.value)}
          disabled={projects.length === 0 || busy}
          style={selectStyle}
        >
          {projects.length === 0 && <option value="">（无）</option>}
          {projects.map((p) => (
            <option key={p} value={p}>{p}</option>
          ))}
        </select>
        <button onClick={onLoad} disabled={!selected || busy}>加载</button>
      </div>

      <div style={{ display: "flex", gap: 6, marginTop: 6 }}>
        <input
          value={saveName}
          onChange={(e) => setSaveName(e.target.value)}
          placeholder="新项目名"
          style={inputStyle}
          disabled={busy}
        />
        <button onClick={onSave} disabled={busy}>保存当前</button>
      </div>

      {(info || error) && (
        <div className="panel-small" style={{ color: error ? "var(--bad)" : "var(--good)", marginTop: 4 }}>
          {error ?? info}
        </div>
      )}
    </Panel>
  );
}
