import Viewport from "./Viewport";
import ScriptPanel from "./ScriptPanel";
import RecordingPanel from "./RecordingPanel";
import ManualLifecycle from "./ManualLifecycle";
import { useSimStore } from "@/store/sim";
import "./InteractionWorkspace.css";

export default function ScriptWorkspace() {
  return <div className="script-workspace">
    <div className="script-workspace-head"><strong>实时动作脚本</strong>
      <p>在驾驶台观察工况；动作按仿真时间执行。需要重复试验和参数扫描时使用离线批量。</p>
      <button onClick={() => useSimStore.getState().setPage("experiment")}>离线批量试验</button>
    </div>
    <main className="app-main">
      <section className="viewport-pane"><Viewport /></section>
      <aside className="side-pane"><ManualLifecycle /><div className="side-scroll"><ScriptPanel /><RecordingPanel /></div></aside>
    </main>
  </div>;
}
