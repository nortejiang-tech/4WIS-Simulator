/**
 * VehicleGeometryStudio — the vehicle-page "几何工作室": the numeric ParamsPanel
 * beside three parameter-driven, draggable geometry diagrams (whole-vehicle,
 * kingpin/wheel, rack hardpoints). All share one edit buffer + one Apply.
 */

import ParamsPanel from "@/components/ParamsPanel";
import ProjectPanel from "@/components/ProjectPanel";
import { VehicleParamsProvider, useVehicleParams } from "@/components/vehicle/VehicleParamsContext";
import VehicleTopDiagram from "@/components/vehicle/VehicleTopDiagram";
import WheelKingpinDiagram from "@/components/vehicle/WheelKingpinDiagram";
import AxleRackDiagram from "@/components/vehicle/AxleRackDiagram";
import "@/components/WorkflowPage.css";
import "./VehicleGeometryStudio.css";

function DirtyBar() {
  const { dirty, busy, apply, reload } = useVehicleParams();
  if (!dirty) return null;
  return (
    <div className="vg-dirtybar">
      <span>⚠ 有未应用的几何改动</span>
      <button className="wf-btn primary" onClick={apply} disabled={busy}>应用</button>
      <button className="wf-btn" onClick={reload} disabled={busy}>还原</button>
    </div>
  );
}

export default function VehicleGeometryStudio() {
  return (
    <VehicleParamsProvider>
      <div className="vg-page">
        <aside className="vg-side">
          <ParamsPanel />
          <ProjectPanel />
        </aside>
        <main className="vg-main">
          <DirtyBar />
          <div className="vg-grid">
            <VehicleTopDiagram />
            <WheelKingpinDiagram />
            <AxleRackDiagram />
          </div>
          <p className="vg-note">
            示意图随参数实时重建（2D 建模口径，非 3D 渲染）；拖动图上的圆点直接反写参数，
            与左侧数值表共用同一编辑缓冲，点「应用」写入模型。派生量（拖距/阿克曼误差/转弯半径/效率）
            与越界红旗均由真实几何算出。
          </p>
        </main>
      </div>
    </VehicleParamsProvider>
  );
}
