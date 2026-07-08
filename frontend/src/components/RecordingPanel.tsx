/**
 * RecordingPanel — start/stop/export the backend recorder.
 *
 * Polls /api/recording/status every 1s while open. Exports CSV via an explicit
 * fetch so backend errors stay visible in this panel.
 */

import { useCallback, useEffect, useState } from "react";

interface RecorderStatus {
  recording: boolean;
  samples: number;
  from_t: number | null;
  to_t: number | null;
  channels: string[];
  buffer_seconds: number;
  available_channels: string[];
}

import { fetchBlob, fetchJSON } from "@/api/http";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";

function fmt(t: number | null): string {
  return t == null ? "—" : t.toFixed(2);
}
function duration(from: number | null, to: number | null): string {
  if (from == null || to == null) return "0.00";
  return (to - from).toFixed(2);
}

export default function RecordingPanel() {
  const [status, setStatus] = useState<RecorderStatus | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [exporting, setExporting] = useState(false);

  const refresh = useCallback(() => {
    fetchJSON<RecorderStatus>("/api/recording/status")
      .then(setStatus)
      .catch((e) => setError(String(e)));
  }, []);

  // Poll while panel is mounted.
  useEffect(() => {
    refresh();
    const id = window.setInterval(refresh, 1000);
    return () => window.clearInterval(id);
  }, [refresh]);

  const onStart = async () => {
    setError(null);
    try {
      const s = await fetchJSON<RecorderStatus>("/api/recording/start", { method: "POST" });
      setStatus(s);
    } catch (e: any) { setError(String(e?.message ?? e)) }
  };
  const onStop = async () => {
    setError(null);
    try {
      const s = await fetchJSON<RecorderStatus>("/api/recording/stop", { method: "POST" });
      setStatus(s);
    } catch (e: any) { setError(String(e?.message ?? e)) }
  };
  const onExport = async () => {
    setError(null);
    setExporting(true);
    try {
      const { blob, filename } = await fetchBlob("/api/recording/export.csv", undefined, 15000);
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = filename ?? `sim4wis_${Date.now()}.csv`;
      document.body.appendChild(link);
      link.click();
      link.remove();
      window.setTimeout(() => URL.revokeObjectURL(url), 0);
    } catch (e: any) {
      setError(`导出失败：${String(e?.message ?? e)}`);
    } finally {
      setExporting(false);
    }
  };

  const recording = status?.recording ?? false;
  const samples = status?.samples ?? 0;
  const dur = duration(status?.from_t ?? null, status?.to_t ?? null);

  return (
    <Panel title="数据录制" help={HELP.recording}>
      <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
        {recording ? (
          <button onClick={onStop} style={btnRecording}>
            ■ 停止录制
          </button>
        ) : (
          <button onClick={onStart} style={btnIdle}>
            ● 开始录制
          </button>
        )}
        <button data-testid="recording-export" onClick={onExport} disabled={samples === 0 || exporting}>
          {exporting ? "导出中..." : "导出 CSV"}
        </button>
      </div>
      <div className="params-list" style={{ marginTop: 6 }}>
        <span>状态</span>
        <span
          className="value"
          data-testid="recording-status"
          style={{ color: recording ? "var(--good)" : "var(--muted)" }}
        >
          {recording ? "录制中" : "空闲"}
        </span>
        <span>采样数</span>
        <span className="value" data-testid="recording-samples">{samples}</span>
        <span>时长</span>
        <span className="value">{dur} s</span>
        <span>时段</span>
        <span className="value">[{fmt(status?.from_t ?? null)}, {fmt(status?.to_t ?? null)}]</span>
        <span>通道数</span>
        <span className="value">{status?.channels.length ?? 0} / {status?.available_channels.length ?? 0}</span>
      </div>
      {error && (
        <div className="panel-small" style={{ color: "var(--bad)", marginTop: 4 }}>{error}</div>
      )}
    </Panel>
  );
}

const btnIdle: React.CSSProperties = {
  background: "rgba(74, 222, 128, 0.12)",
  borderColor: "rgba(74, 222, 128, 0.4)",
  color: "var(--good)",
  flex: 1,
};
const btnRecording: React.CSSProperties = {
  background: "rgba(239, 68, 68, 0.18)",
  borderColor: "rgba(239, 68, 68, 0.5)",
  color: "var(--bad)",
  flex: 1,
};
