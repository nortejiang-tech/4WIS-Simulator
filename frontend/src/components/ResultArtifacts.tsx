import { useEffect, useState } from "react";
import { fetchBlob, fetchJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import "./InteractionWorkspace.css";

export interface ArtifactManifest {
  run_id: string;
  samples: number;
  channels: { name: string; unit: string }[];
  sampling: string;
  csv_sha256: string;
  complete: boolean;
  execution_status: string;
  outputs: Record<string, string>;
}

export default function ResultArtifacts({ runId, showAnalysis = true }: { runId: string; showAnalysis?: boolean }) {
  const [manifest, setManifest] = useState<ArtifactManifest | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    let cancelled = false;
    setManifest(null); setError("");
    fetchJSON<ArtifactManifest>(`/api/runs/${encodeURIComponent(runId)}/artifacts`)
      .then(value => { if (!cancelled) setManifest(value); })
      .catch(err => { if (!cancelled) setError(String(err)); });
    return () => { cancelled = true; };
  }, [runId]);
  const download = async (format: string) => {
    if (!manifest) return;
    setBusy(true); setError("");
    try {
      const { blob, filename } = await fetchBlob(manifest.outputs[format], undefined, 60000);
      const url = URL.createObjectURL(blob);
      const anchor = document.createElement("a");
      anchor.href = url; anchor.download = filename ?? `${runId}.${format}`;
      anchor.click(); window.setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (err) { setError(String(err)); }
    finally { setBusy(false); }
  };
  const analyze = () => {
    const state = useSimStore.getState();
    state.setAnalysisPreselect([runId]); state.setPage("analysis");
  };
  return <section className="result-artifacts" aria-label="结果输出">
    <div className="interaction-section-title"><strong>结果输出</strong><code>{runId}</code></div>
    {manifest ? <>
      <p>{manifest.samples.toLocaleString()} 个已保存样本 · {manifest.channels.length - 1} 个数据通道 · 导出不降采样</p>
      {!manifest.complete && <p role="alert" className="interaction-error">本次运行或起始记录不完整；导出保留现有样本，不能据此声称完整工况成功。</p>}
      <div className="interaction-actions">
        <button disabled={busy} onClick={() => download("csv")}>完整 CSV</button>
        <button disabled={busy} onClick={() => download("json")}>完整 JSON</button>
        <button disabled={busy} onClick={() => download("bundle")}>数据与图表 ZIP</button>
        <a href={manifest.outputs.trajectory} target="_blank" rel="noreferrer">轨迹 SVG</a>
        <a href={manifest.outputs.chart} target="_blank" rel="noreferrer">横摆图表 SVG</a>
        {showAnalysis && <button onClick={analyze}>在分析页打开</button>}
      </div>
      <details><summary>采样说明与通道单位</summary><p>{manifest.sampling}</p>
        <p className="interaction-hash">CSV SHA-256: {manifest.csv_sha256}</p>
        <div className="interaction-channel-list">{manifest.channels.map(c => <span key={c.name}>{c.name} [{c.unit}]</span>)}</div>
      </details>
    </> : !error && <p role="status">读取结果清单…</p>}
    {error && <p className="interaction-error" role="alert">{error}</p>}
  </section>;
}
