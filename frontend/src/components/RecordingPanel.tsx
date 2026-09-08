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
  dropped_samples?: number;
  stop_reason?: string | null;
}

import { fetchBlob, fetchJSON, postJSON } from "@/api/http";
import Panel from "@/components/Panel";
import { HELP } from "@/ui/help";
import ResultArtifacts from "./ResultArtifacts";
import type { ArtifactManifest } from "./ResultArtifacts";

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
  const [full, setFull] = useState(true);
  const [runId, setRunId] = useState("");

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
      const channels = full ? (status?.available_channels ??
        (await fetchJSON<RecorderStatus>("/api/recording/status")).available_channels) : undefined;
      const s = await postJSON<RecorderStatus>("/api/recording/start", {
        channels, full_rate: full,
      });
      setRunId("");
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
  const onClear = async () => {
    setError(null);
    setExporting(true);
    try {
      const s = await postJSON<RecorderStatus>("/api/recording/clear");
      setRunId("");
      setStatus(s);
    } catch (e: any) { setError(`丢弃录制失败：${String(e?.message ?? e)}`) }
    finally { setExporting(false); }
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
      <label className="interaction-note"><input type="checkbox" checked={full} disabled={recording} onChange={e => setFull(e.target.checked)} /> 全部可用通道 · 每个积分步采样</label>
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
        <button
          data-testid="recording-clear"
          onClick={onClear}
          disabled={recording || samples === 0 || exporting}
          title="丢弃当前未保存的录制数据"
        >
          丢弃录制
        </button>
      </div>
      <button disabled={recording || samples === 0 || exporting} onClick={async () => {
        setError(null); setExporting(true);
        try {
          const saved = await postJSON<ArtifactManifest>("/api/recording/save", undefined, 60000);
          setRunId(saved.run_id);
        } catch (err) { setError(String(err)); }
        finally { setExporting(false); }
      }}>保存到结果库</button>
      {!!status?.dropped_samples && <p className="interaction-error" role="alert">缓冲区已丢弃 {status.dropped_samples} 个早期样本；当前录制不是完整起始记录。</p>}
      {status?.stop_reason && <p className="interaction-note">时间轴已重置，录制自动停止并保留此前数据。</p>}
      {runId && <ResultArtifacts runId={runId} />}
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
