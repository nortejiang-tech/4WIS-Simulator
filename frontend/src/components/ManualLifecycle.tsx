import { useState } from "react";
import { postJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import "./InteractionWorkspace.css";

export default function ManualLifecycle() {
  const paused = useSimStore(s => s.state?.interaction?.paused ?? false);
  const armed = useSimStore(s => s.manualArmed);
  const page = useSimStore(s => s.page);
  const scripted = useSimStore(s => s.state?.interaction?.source === "script");
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const pause = async () => {
    setBusy(true); setError("");
    try { await postJSON("/api/interaction/control", { action: paused ? "resume" : "pause" }); }
    catch (err) { setError(String(err)); }
    finally { setBusy(false); }
  };
  return <div className="manual-lifecycle">
    <span>{paused ? "仿真已暂停" : scripted ? "实时脚本控制中" : page === "script" ? "等待脚本控制" : armed ? "驾驶输入已启用" : "驾驶输入已释放"}</span>
    <div className="interaction-actions">
      {!armed && page === "run" && <button onClick={() => useSimStore.getState().setManualArmed(true)}>启用手动输入</button>}
      <button disabled={busy} onClick={pause}>{paused ? "继续仿真" : "暂停仿真"}</button>
    </div>
    {error && <span className="interaction-error" role="alert">{error}</span>}
  </div>;
}
