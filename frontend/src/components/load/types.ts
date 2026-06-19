// Shared types, constants, helpers for the 负载特性 page.

export {
  ADVANCED_PARAMETER_GROUPS,
  CORE_PARAMETER_GROUPS,
  MM,
  PARAMETER_GROUPS,
  type FieldSpec,
  type ParameterGroup,
} from "@/vehicle/parameterGroups";
export type VehicleParams = Record<string, any> & {
  suspension?: Record<string, number>;
  steering_geometry?: Record<string, number>;
};

export interface ProfileListItem {
  name: string;
  label: string;
  builtin: boolean;
}

export interface LoadRow {
  speed: number;
  requested_angle: number;
  wheel_index: number;
  wheel_label: string;
  delta: number;
  delta_cmd: number;
  torque_steer: number;
  torque_steer_ideal?: number;
  rack_force: number;
  rack_force_ideal?: number;
  motor_torque: number;
  motor_torque_ideal?: number;
  side_force_body_y: number;
  side_force_body_y_ideal?: number;
  fx_drive?: number;
  fy_camber?: number;
  arm_tie_angle: number;
  tie_rack_angle: number;
  geometry_efficiency: number;
  friction_utilization: number;
}

export interface PerSpeedEquilibrium {
  speed: number;
  delta_eq: number | null;
  delta_eq_deg: number | null;
  rack_at_zero: number | null;
  torque_at_zero: number | null;
  source: string;
  found: boolean;
}

export interface LoadSweepResponse {
  rows: LoadRow[];
  summary: {
    peak_abs_rack_force?: number;
    peak_abs_rack_force_at?: { speed: number; requested_angle: number; wheel_label: string };
    peak_abs_motor_torque?: number;
    min_geometry_efficiency?: number;
    max_friction_utilization?: number;
    warnings?: string[];
    per_speed_equilibrium?: PerSpeedEquilibrium[];
  };
}

export const MODE_OPTIONS = [
  ["single_wheel", "单轮强制转角"],
  ["ideal_ackermann", "理想阿克曼"],
  ["ackermann", "传统阿克曼"],
  ["rear_wheel_steer", "后轮转向"],
  ["crab", "蟹行"],
  ["zero_radius", "零半径"],
] as const;

export const deg = (rad: number) => (rad * 180) / Math.PI;
export const rad = (degValue: number) => (degValue * Math.PI) / 180;
export const fmt = (n: number | null | undefined, digits = 1): string =>
  n != null && Number.isFinite(n) ? (n as number).toFixed(digits) : "—";
export const displayNumber = (n: number): string => {
  if (!Number.isFinite(n)) return "";
  return String(Number(n.toFixed(3)));
};

export function getPath(obj: VehicleParams | null, path: string): number {
  if (!obj) return 0;
  return path.split(".").reduce<any>((acc, key) => acc?.[key], obj) ?? 0;
}

export function setPath(obj: VehicleParams, path: string, value: number): VehicleParams {
  const keys = path.split(".");
  const next: VehicleParams = { ...obj };
  let cur: Record<string, any> = next;
  for (let i = 0; i < keys.length - 1; i++) {
    const key = keys[i];
    cur[key] = { ...(cur[key] ?? {}) };
    cur = cur[key];
  }
  cur[keys[keys.length - 1]] = value;
  return next;
}

export function rangeValues(min: number, max: number, count: number): number[] {
  const n = Math.max(2, Math.min(161, Math.round(count)));
  const out: number[] = [];
  for (let i = 0; i < n; i++) out.push(min + ((max - min) * i) / (n - 1));
  return out;
}

export function uniqueSorted(values: number[]): number[] {
  const rounded = values.map((v) => Math.round(v * 1e6) / 1e6);
  return Array.from(new Set(rounded)).sort((a, b) => a - b);
}

export type NumericSeries = ArrayLike<number | null | undefined>;

export function interpolateAt(xs: NumericSeries, ys: NumericSeries, x: number): number | null {
  let prevX: number | null = null;
  let prevY: number | null = null;
  for (let i = 0; i < xs.length; i++) {
    const xi = Number(xs[i]);
    const yi = Number(ys[i]);
    if (!Number.isFinite(xi) || !Number.isFinite(yi)) continue;
    if (prevX == null) {
      if (x <= xi) return yi;
      prevX = xi;
      prevY = yi;
      continue;
    }
    if ((prevX <= x && x <= xi) || (xi <= x && x <= prevX)) {
      if (Math.abs(xi - prevX) < 1e-12) return yi;
      const t = (x - prevX) / (xi - prevX);
      return (prevY as number) + (yi - (prevY as number)) * t;
    }
    prevX = xi;
    prevY = yi;
  }
  return prevY;
}

export function paramToBody(params: VehicleParams | null): VehicleParams | undefined {
  return params ? JSON.parse(JSON.stringify(params)) : undefined;
}

/** Lightweight hash of the parameter object so chart-rebuild keys can react
 *  to *any* parameter change, not just the explicit control-bar inputs. */
export function paramsHash(params: VehicleParams | null): string {
  if (!params) return "0";
  const json = JSON.stringify(params);
  let h = 5381;
  for (let i = 0; i < json.length; i++) h = ((h << 5) + h + json.charCodeAt(i)) | 0;
  return h.toString(36);
}

export function equilibriumAt(
  per: PerSpeedEquilibrium[] | undefined,
  speedMs: number,
): PerSpeedEquilibrium | null {
  if (!per || per.length === 0) return null;
  return per.reduce((best, item) => (
    Math.abs(item.speed - speedMs) < Math.abs(best.speed - speedMs) ? item : best
  ), per[0]);
}
