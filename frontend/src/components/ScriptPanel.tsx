/**
 * ScriptPanel — load a YAML action sequence, run it, watch progress.
 *
 * Library dropdown calls /api/script/library to discover bundled scripts
 * under <repo>/scripts_lib/. The textarea is editable so users can tweak
 * before loading. Pressing Start posts the textarea contents to /api/script/load
 * followed by /api/script/start.
 */

import { useCallback, useEffect, useState } from "react";

interface ScriptStatus {
  running: boolean;
  current_action_idx: number;
  t_in_script: number;
  script_name: string;
  loop_count: number;
}

import { fetchJSON } from "@/api/http";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

const DEFAULT_TEMPLATE = `script:
  name: my_script
  description: ""
  loop: false
  actions:
    - { t: 0.0, action: set_strategy, name: ideal_ackermann }
    - { t: 0.0, action: drive,        throttle: 0.5, steering: 0.0 }
    - { t: 2.0, action: steer_ramp,   from_: 0.0, to: +0.3, duration: 0.5 }
    - { t: 5.0, action: brake,        duration: 2.0 }
`;

function formatError(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

export default function ScriptPanel() {
  const [library, setLibrary] = useState<string[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [scriptYaml, setScriptYaml] = useState<string>(DEFAULT_TEMPLATE);
  const [status, setStatus] = useState<ScriptStatus | null>(null);
  const [statusError, setStatusError] = useState<string | null>(null);
  const [info, setInfo] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const refreshLibrary = useCallback(() => {
    setInfo(null);
    fetchJSON<{ scripts: string[] }>("/api/script/library")
      .then((d) => {
        setLibrary(d.scripts);
        setError(null);
        setSelected(d.scripts.length ? d.scripts[0] : "");
      })
      .catch((e) => {
        setLibrary([]);
        setSelected("");
        setError(`读取脚本库失败：${formatError(e)}`);
      });
  }, []);

  // Load library on mount
  useEffect(() => {
    refreshLibrary();
  }, [refreshLibrary]);

  // Poll status while panel is mounted
  const pollStatus = useCallback(() => {
    fetchJSON<ScriptStatus>("/api/script/status")
      .then((nextStatus) => {
        setStatus(nextStatus);
        setStatusError(null);
      })
      .catch((e) => {
        setStatusError(`读取脚本状态失败：${formatError(e)}`);
      });
  }, []);
  useEffect(() => {
    pollStatus();
    const id = window.setInterval(pollStatus, 500);
    return () => window.clearInterval(id);
  }, [pollStatus]);

  const loadFromLibrary = async () => {
    if (!selected) return;
    setError(null);
    try {
      const r = await fetchJSON<{ name: string; yaml: string }>(
        `/api/script/library/${encodeURIComponent(selected)}`,
      );
      setScriptYaml(r.yaml);
      setInfo(`已载入: ${r.name}`);
    } catch (e) { setError(formatError(e)) }
  };

  const onStart = async () => {
    setError(null);
    setInfo(null);
    try {
      const loadResp = await fetchJSON<{ script_name: string; actions: number }>(
        "/api/script/load",
        { method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ yaml: scriptYaml }) },
      );
      await fetchJSON("/api/script/start", { method: "POST" });
      setInfo(`运行中: ${loadResp.script_name} (${loadResp.actions} 个动作)`);
    } catch (e) { setError(formatError(e)) }
  };

  const onStop = async () => {
    setError(null);
    try {
      await fetchJSON("/api/script/stop", { method: "POST" });
      setInfo("已停止");
    } catch (e) { setError(formatError(e)) }
  };

  const running = status?.running ?? false;

  return (
    <Panel title="动作脚本" help={HELP.script}>
      <div style={{ display: "flex", gap: 6 }}>
        <select
          value={selected}
          onChange={(e) => setSelected(e.target.value)}
          disabled={library.length === 0 || running}
          style={{ flex: 1, background: "var(--bg-2)", border: "1px solid var(--border)",
                   color: "var(--text)", borderRadius: 6, padding: "6px 8px", fontSize: 12 }}
        >
          {library.length === 0 && <option value="">（无）</option>}
          {library.map((s) => <option key={s} value={s}>{s}</option>)}
        </select>
        <button onClick={loadFromLibrary} disabled={!selected || running}>载入</button>
        <button onClick={refreshLibrary} disabled={running}>刷新库</button>
      </div>

      <textarea
        value={scriptYaml}
        onChange={(e) => setScriptYaml(e.target.value)}
        readOnly={running}
        rows={9}
        style={{
          marginTop: 6,
          width: "100%", background: "var(--bg-0)", color: "var(--text)",
          border: "1px solid var(--border)", borderRadius: 4, padding: 8,
          fontFamily: "ui-monospace, SFMono-Regular, Menlo, monospace",
          fontSize: 11, lineHeight: 1.4, resize: "vertical",
        }}
      />

      <div className="btn-row" style={{ marginTop: 6 }}>
        {running ? (
          <button className="warn" onClick={onStop}>■ 停止脚本</button>
        ) : (
          <button onClick={onStart}>▶ 启动脚本</button>
        )}
      </div>

      {status && (status.running || status.loop_count > 0) && (
        <div className="params-list" style={{ marginTop: 4 }}>
          <span>状态</span>
          <span className="value" style={{ color: running ? "var(--good)" : "var(--muted)" }}>
            {running ? "运行中" : "已结束"}
          </span>
          <span>脚本名</span>
          <span className="value">{status.script_name || "—"}</span>
          <span>动作索引</span>
          <span className="value">#{status.current_action_idx}</span>
          <span>已运行</span>
          <span className="value">{status.t_in_script.toFixed(1)} s</span>
        </div>
      )}
      {statusError && (
        <div
          className="small"
          role="alert"
          style={{ color: "var(--bad)", marginTop: 4 }}
        >
          {statusError}
        </div>
      )}
      {(info || error) && (
        <div
          className="small"
          role={error ? "alert" : "status"}
          style={{ color: error ? "var(--bad)" : "var(--good)", marginTop: 4 }}
        >
          {error ?? info}
        </div>
      )}
    </Panel>
  );
}
