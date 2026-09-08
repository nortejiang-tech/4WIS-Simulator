export interface Preview { total_samples: number; returned_samples: number; decimated: boolean; data: Record<string, (number | null)[]> }

function Plot({ x, y, title, equal = false }: { x: (number | null)[]; y: (number | null)[]; title: string; equal?: boolean }) {
  const pairs = x.flatMap((a, i) => a != null && y[i] != null && Number.isFinite(a) && Number.isFinite(y[i]) ? [[a, y[i] as number]] : []);
  if (!pairs.length) return <div className="agent-plot-empty">{title} · 推进会话后显示</div>;
  const xs = pairs.map(p => p[0]), ys = pairs.map(p => p[1]);
  let xmin = Math.min(...xs), ymin = Math.min(...ys);
  let dx = Math.max(Math.max(...xs) - xmin, .001), dy = Math.max(Math.max(...ys) - ymin, .001);
  if (equal) {
    const scale = Math.min(450 / dx, 200 / dy);
    const oldDx = dx, oldDy = dy;
    dx = 450 / scale; dy = 200 / scale;
    xmin -= (dx - oldDx) / 2; ymin -= (dy - oldDy) / 2;
  }
  const points = pairs.map(([a,b]) => `${(55 + (a-xmin) / dx*450).toFixed(2)},${(235-(b-ymin)/dy*200).toFixed(2)}`);
  const last = points.at(-1)!.split(",");
  return <svg viewBox="0 0 550 275" role="img" aria-label={title} className="agent-plot">
    <text x="55" y="20" className="agent-plot-title">{title}</text>
    {[0,1,2,3,4].map(i => <g key={i}>
      <line x1="55" x2="505" y1={235-i*50} y2={235-i*50} />
      <text x="47" y={238-i*50} textAnchor="end">{(ymin+dy*i/4).toFixed(2)}</text>
      <text x={55+i*112.5} y="255" textAnchor="middle">{(xmin+dx*i/4).toFixed(2)}</text>
    </g>)}
    <polyline points={points.join(" ")} />
    <circle cx={last[0]} cy={last[1]} r="4" />
  </svg>;
}

export default function AgentPlots({ preview }: { preview: Preview | null }) {
  const data = preview?.data ?? {};
  return <section aria-label="Agent 图形与图表">
    <div className="agent-plots">
      <Plot x={data.pose_x ?? []} y={data.pose_y ?? []} title="轨迹 · X / Y [m] · 等比例" equal />
      <Plot x={data.t ?? []} y={data.yaw_rate ?? []} title="横摆角速度 [rad/s] / 时间 [s]" />
    </div>
    <p className="interaction-note">预览 {preview?.returned_samples ?? 0} / {preview?.total_samples ?? 0} 样本{preview?.decimated ? "（已降采样）" : ""}。完整数据由结果导出提供。</p>
  </section>;
}
