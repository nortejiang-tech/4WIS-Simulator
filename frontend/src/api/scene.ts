// Scene / disturbance CRUD — the sim pushes the updated scene in the next
// state frame, so no client-side cache invalidation is needed.

import { fetchJSON, postJSON } from "@/api/http";
import type { DisturbanceMsg } from "@/types/sim";
import type { DistType } from "@/store/sim";

export interface DisturbanceParams {
  x: number;
  y: number;
  width?: number;
  length?: number;
  heading?: number;
  mu?: number;
  mu_left?: number;
  mu_right?: number;
  height?: number;
  stiffness?: number;
  angle?: number;
}

export function createDisturbance(type: DistType, params: DisturbanceParams) {
  return postJSON<DisturbanceMsg>("/api/scene/disturbances", { type, ...params });
}

export function updateDisturbance(id: string, params: Partial<DisturbanceParams>) {
  return fetchJSON<DisturbanceMsg>(`/api/scene/disturbances/${encodeURIComponent(id)}`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(params),
  });
}

export function deleteDisturbance(id: string) {
  return fetchJSON<{ status: string; id: string }>(
    `/api/scene/disturbances/${encodeURIComponent(id)}`,
    { method: "DELETE" },
  );
}

export function clearDisturbances() {
  return postJSON<{ status: string; removed: number }>("/api/scene/clear");
}
