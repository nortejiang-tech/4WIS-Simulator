/**
 * UserPythonPanel — status display and reload control for the hot-reload
 * Python strategy plugin (user_strategy.py in plugins/strategies/).
 *
 * Only fully visible when the active strategy is "user_python"; otherwise shows
 * a collapsed status badge.
 */

import { useEffect, useRef, useState } from "react";
import { fetchJSON, postJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

interface StatusResp {
  status: "ok" | "error" | "no_file";
  error: string | null;
  last_reload: number | null;
  file_path: string;
}

const STATUS_COLOR: Record<string, string> = {
  ok:      "#22c55e",
  error:   "#ef4444",
  no_file: "#f59e0b",
};
const STATUS_LABEL: Record<string, string> = {
  ok:      "已加载",
  error:   "错误",
  no_file: "文件不存在",
};

export default function UserPythonPanel() {
  const activeStrategy = useSimStore((s) => s.state?.strategy);
  const pushToast = useSimStore((s) => s.pushToast);

  const [status, setStatus] = useState<StatusResp | null>(null);
  const [busy, setBusy] = useState(false);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const refresh = () =>
    fetchJSON<StatusResp>("/api/user_python/status")
      .then(setStatus)
      .catch(() => {/* silent */});

  useEffect(() => {
    refresh();
    timerRef.current = setInterval(refresh, 2000);
    return () => { if (timerRef.current) clearInterval(timerRef.current); };
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const reload = async () => {
    setBusy(true);
    try {
      const r = await postJSON<StatusResp>("/api/user_python/reload");
      setStatus(r);
      pushToast("info", `Python 策略已重载：${r.status}`);
    } catch (e: any) {
      pushToast("error", `重载失败：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  };

  const color = STATUS_COLOR[status?.status ?? "no_file"];
  const label = STATUS_LABEL[status?.status ?? "no_file"];
  const isActive = activeStrategy === "user_python";

  return (
    <Panel
      title="Python 策略插件"
      help={HELP.userPython}
      badge={
        <>
          <span style={{
            fontSize: 10, padding: "1px 5px", borderRadius: 4,
            background: color, color: "#fff", marginLeft: "auto",
          }}>
            ● {label}
          </span>
          {isActive && (
            <span style={{ fontSize: 10, color: "#22d3ee", marginLeft: 4 }}>▶ 激活</span>
          )}
        </>
      }
    >

      {status && (
        <>
          <div className="small" style={{ color: "var(--muted)", marginBottom: 4, wordBreak: "break-all" }}>
            {status.file_path}
          </div>

          {status.status === "error" && status.error && (
            <div style={{
              fontSize: 11, color: "#ef4444", background: "rgba(239,68,68,0.1)",
              borderRadius: 4, padding: "4px 6px", marginBottom: 6,
              fontFamily: "monospace", whiteSpace: "pre-wrap", wordBreak: "break-all",
            }}>
              {status.error}
            </div>
          )}

          {status.status === "no_file" && (
            <div className="small" style={{ color: "#f59e0b", marginBottom: 6 }}>
              请在上方路径创建 user_strategy.py，仿真器会自动加载。
            </div>
          )}

          {status.last_reload && (
            <div className="small" style={{ color: "var(--muted)", marginBottom: 6 }}>
              上次重载：{new Date(status.last_reload * 1000).toLocaleTimeString()}
            </div>
          )}

          <button onClick={reload} disabled={busy || !isActive}>
            {busy ? "重载中…" : "强制重载"}
          </button>
          <div className="small" style={{ color: "var(--muted)", marginTop: 4 }}>
            保存文件后 ≈1s 自动热重载；或切换到 user_python 策略后点此手动触发。
          </div>
        </>
      )}
    </Panel>
  );
}
