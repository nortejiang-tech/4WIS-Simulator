import { useEffect, useRef, useState } from "react";
import { deleteJSON, fetchJSON, HttpError, postJSON } from "@/api/http";
import ResultArtifacts from "./ResultArtifacts";
import type { ArtifactManifest } from "./ResultArtifacts";
import AgentPlots from "./AgentPlots";
import type { Preview } from "./AgentPlots";
import "./InteractionWorkspace.css";

interface Session {
  session_id: string; label: string; status: string; revision: number; steps: number; samples: number;
  dt: number; remaining_steps: number; model_type: string; strategy: string; error: string | null;
  run_id: string | null;
  state: { t: number; velocity: { vx: number; vy: number; yaw_rate: number }; wheels: { name: string; delta: number; fz: number }[] };
}
const EXAMPLE = JSON.stringify({ label: "Agent 闭环试用", experiment: { name: "agent_interaction", model_type: "simplified_dynamic", strategy: "ideal_ackermann", dt: .005 } }, null, 2);
const API = "/api/agent/sessions";

export default function AgentWorkspace() {
  const [sessions, setSessions] = useState<Session[]>([]);
  const [selected, setSelected] = useState("");
  const [current, setCurrent] = useState<Session | null>(null);
  const [preview, setPreview] = useState<Preview | null>(null);
  const [config, setConfig] = useState(EXAMPLE);
  const [steps, setSteps] = useState(200);
  const [throttle, setThrottle] = useState(.1);
  const [steering, setSteering] = useState(0);
  const [error, setError] = useState("");
  const [pollError, setPollError] = useState("");
  const [busy, setBusy] = useState(false);
  const [runId, setRunId] = useState("");
  const [pending, setPending] = useState<{ session: string; body: object } | null>(null);
  const busyRef = useRef(false);
  const refresh = async () => {
    const data = await fetchJSON<{ sessions: Session[] }>(API);
    setSessions(data.sessions);
  };
  useEffect(() => { refresh().catch(err => setError(String(err))); }, []);
  useEffect(() => {
    setCurrent(null); setPreview(null); setRunId(""); setPollError("");
    if (!selected) return;
    let cancelled = false;
    let timer: number;
    const poll = async () => {
      try {
        const [state, trace] = await Promise.all([
          fetchJSON<Session>(`${API}/${selected}`), fetchJSON<Preview>(`${API}/${selected}/preview`),
        ]);
        if (!cancelled) {
          setCurrent(prev => !prev || state.revision >= prev.revision ? state : prev);
          setPreview(trace); setPollError("");
          if (state.run_id) setRunId(state.run_id);
        }
      } catch (err) { if (!cancelled) setPollError(String(err)); }
      if (!cancelled) timer = window.setTimeout(poll, 500);
    };
    poll();
    return () => { cancelled = true; window.clearTimeout(timer); };
  }, [selected]);
  const action = async (fn: () => Promise<void>) => {
    if (busyRef.current) return;
    busyRef.current = true; setBusy(true); setError("");
    try { await fn(); } catch (err) { setError(String(err)); }
    finally { busyRef.current = false; setBusy(false); }
  };
  const create = () => action(async () => {
    const response = await postJSON<Session>(API, JSON.parse(config));
    setSelected(response.session_id); await refresh();
  });
  const step = () => action(async () => {
    if (!current) return;
    const request = pending ?? { session: selected, body: { request_id: crypto.randomUUID(), expected_revision: current.revision,
      steps, control: { throttle, steering, brake: 0, gear: 1, handbrake: 0 } } };
    setPending(request);
    try {
      const response = await postJSON<Session>(`${API}/${request.session}/step`, request.body, 30000);
      setCurrent(response); setPending(null); setRunId("");
    } catch (err) {
      if (err instanceof HttpError && err.status > 0) setPending(null);
      throw err;
    }
  });
  const save = () => action(async () => {
    const result = await postJSON<ArtifactManifest>(`${API}/${selected}/export`, undefined, 30000);
    setRunId(result.run_id); await refresh();
  });
  const close = () => action(async () => {
    await deleteJSON(`${API}/${selected}`); setSelected(""); setPending(null); await refresh();
  });
  return <main className="interaction-workspace agent-workspace">
    <header className="interaction-workspace-head">
      <div><h1>AI Agent 工作台</h1><p>观察状态 → 提交控制 → 推进固定步数 → 保存结果。每个会话独立于人工驾驶台。</p></div>
      <a href="/docs#/agent-v1" target="_blank" rel="noreferrer">OpenAPI 接口</a>
    </header>
    <div className="agent-layout">
      <aside className="agent-config">
        <h2>会话配置</h2><label htmlFor="agent-config">创建请求 · JSON</label>
        <textarea id="agent-config" value={config} onChange={e => setConfig(e.target.value)} rows={12} spellCheck={false} disabled={busy} />
        <button onClick={create} disabled={busy || !!pending}>创建独立会话</button>
        <div className="interaction-section-title"><h2>活动会话</h2><button onClick={() => action(refresh)} disabled={busy}>刷新</button></div>
        <select aria-label="选择 Agent 会话" value={selected} onChange={e => setSelected(e.target.value)} disabled={busy || !!pending}>
          <option value="">选择会话</option>{sessions.map(s => <option key={s.session_id} value={s.session_id}>{s.label} · {s.session_id.slice(0,8)}</option>)}
        </select>
        <details className="agent-connection"><summary>MCP 与调用示例</summary>
          <p>使用本项目 Python 环境运行 <code>python -m sim4wis_mcp.server --base {window.location.origin}</code>。先调用 describe_capabilities，再调用 create_session / step_session / export_session。</p>
          <pre>{JSON.stringify({ tool: "step_session", arguments: { session_id: selected || "会话 ID", request_id: "step-001", expected_revision: current?.revision ?? 0, steps: 200, control: { throttle: .1, steering: .05 } } }, null, 2)}</pre>
          <a href="/api/agent/capabilities" target="_blank" rel="noreferrer">能力、单位与完整 Schema</a>
          <p>长研究用 start_study / get_study_job。会话最多 20,000 步，关闭或重启前请导出。</p>
        </details>
      </aside>
      <section className="agent-main">
        {current ? <>
          <div className="agent-session-status" aria-label="Agent 会话状态">
            <strong>{current.label}</strong><span>{current.model_type} · {current.strategy}</span>
            <span>状态 {current.status} · 修订 {current.revision}</span>
            <span>仿真 {current.state.t.toFixed(3)} s · {current.samples} 样本</span>
          </div>
          <div className="agent-command">
            <label>推进步数<input type="number" min="1" max="1000" value={steps} onChange={e => setSteps(Number(e.target.value))} disabled={busy || !!pending} /></label>
            <label>油门 [0,1]<input type="number" min="0" max="1" step="0.01" value={throttle} onChange={e => setThrottle(Number(e.target.value))} disabled={busy || !!pending} /></label>
            <label>转向 [-1,1]<input type="number" min="-1" max="1" step="0.01" value={steering} onChange={e => setSteering(Number(e.target.value))} disabled={busy || !!pending} /></label>
            <button onClick={step} disabled={busy || current.status !== "ready"}>{pending ? "重试同一请求" : `推进 ${(steps * current.dt).toFixed(3)} s`}</button>
          </div>
          {pending && <p className="interaction-note">请求尚未确认。重试会复用原请求 ID，不重复推进。</p>}
          <AgentPlots preview={preview} />
          <div className="agent-wheel-readback">{current.state.wheels.map(w => <span key={w.name}><b>{w.name}</b> {(w.delta * 180 / Math.PI).toFixed(2)}° · {w.fz.toFixed(0)} N</span>)}</div>
          <div className="interaction-actions">
            <button onClick={save} disabled={busy || !!pending || !current.samples}>保存完整结果</button>
            <button onClick={close} disabled={busy || !!pending}>关闭会话</button>
            <span className="interaction-note">关闭会丢弃未保存数据；已保存的结果保留。</span>
          </div>
          {current.error && <p className="interaction-error" role="alert">{current.error}</p>}
          {runId && <ResultArtifacts runId={runId} />}
        </> : <div className="agent-empty"><h2>先创建或选择一个会话</h2><p>接口与此工作台使用同一条执行路径。Agent 控制不会驱动人工驾驶台上的车辆。</p><p>图形预览、完整数据与分析图表均来自实际计算结果。</p></div>}
        {(error || pollError) && <p className="interaction-error" role="alert">{error || pollError}</p>}
      </section>
    </div>
  </main>;
}
