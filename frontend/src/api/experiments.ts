// Typed client for the Phase-A experiment/batch/run REST API.
// Shapes mirror backend/src/sim4wis/experiment/schema.py.

import { deleteJSON, fetchJSON, postJSON } from "@/api/http";

export type SteerKind = "constant" | "step" | "ramp" | "sine" | "sweep" | "dlc";

export interface SteerProfileT {
  kind: SteerKind;
  amplitude: number;
  freq_hz: number;
  f0_hz: number;
  f1_hz: number;
  t_step: number;
  start: number;
}

export interface ManeuverStepT {
  name: string;
  duration: number;
  steer: SteerProfileT;
  speed_kmh: number | null;
  speed_ramp_s: number;
  mode_params: Record<string, unknown>;
}

export interface ExperimentT {
  name: string;
  description: string;
  vehicle: { profile: string | null; overrides: Record<string, unknown> };
  model_type: string;
  strategy: string;
  mode_params: Record<string, unknown>;
  scene: Record<string, unknown> | null;
  path: {
    template: string | null;
    params: Record<string, unknown>;
    waypoints: [number, number][] | null;
    closed: boolean;
  } | null;
  maneuver: { name: string; steps: ManeuverStepT[] };
  dt: number;
  record_hz: number;
  kpis: string[];
}

export function defaultSteer(): SteerProfileT {
  return { kind: "constant", amplitude: 0, freq_hz: 0.5, f0_hz: 0.1, f1_hz: 2.0, t_step: 0.5, start: 0 };
}

export function defaultStep(): ManeuverStepT {
  return { name: "", duration: 5, steer: defaultSteer(), speed_kmh: null, speed_ramp_s: 0, mode_params: {} };
}

export function defaultExperiment(): ExperimentT {
  return {
    name: "new_experiment",
    description: "",
    vehicle: { profile: null, overrides: {} },
    model_type: "simplified_dynamic",
    strategy: "ideal_ackermann",
    mode_params: {},
    scene: null,
    path: null,
    maneuver: {
      name: "maneuver",
      steps: [
        { ...defaultStep(), name: "加速", duration: 6, speed_kmh: 60, speed_ramp_s: 4 },
        { ...defaultStep(), name: "激励", duration: 6, steer: { ...defaultSteer(), kind: "step", amplitude: 0.05 } },
      ],
    },
    dt: 0.005,
    record_hz: 50,
    kpis: [],
  };
}

// ---- experiment definitions ----

export interface ExperimentListItem {
  name: string;
  description: string;
  strategy: string;
  model_type: string;
  maneuver: string;
}

export async function listExperiments(): Promise<ExperimentListItem[]> {
  const d = await fetchJSON<{ experiments: ExperimentListItem[] }>("/api/experiments");
  return d.experiments ?? [];
}

export function getExperiment(name: string): Promise<ExperimentT> {
  return fetchJSON<ExperimentT>(`/api/experiments/${encodeURIComponent(name)}`);
}

export function saveExperiment(exp: ExperimentT): Promise<unknown> {
  return postJSON(`/api/experiments/${encodeURIComponent(exp.name)}`, exp);
}

export function deleteExperiment(name: string): Promise<unknown> {
  return deleteJSON(`/api/experiments/${encodeURIComponent(name)}`);
}

export async function maneuverTemplates(): Promise<{ steer_kinds: string[]; path_templates: string[] }> {
  return fetchJSON("/api/maneuver-templates");
}

// ---- batch ----

export interface VariantT {
  label: string;
  overrides: Record<string, unknown>;
}

export interface BatchRunSummary {
  run_id: string;
  label: string;
  kpis: Record<string, number>;
}

export interface BatchStatus {
  job_id: string;
  status: "running" | "done" | "error" | "cancelled";
  total: number;
  done: number;
  current_label: string;
  error: string;
  elapsed_s: number;
  runs: BatchRunSummary[];
}

export async function startBatch(experiment: ExperimentT, variants: VariantT[]): Promise<string> {
  const d = await postJSON<{ job_id: string }>("/api/batch", { experiment, variants });
  return d.job_id;
}

export function getBatch(jobId: string): Promise<BatchStatus> {
  return fetchJSON<BatchStatus>(`/api/batch/${encodeURIComponent(jobId)}`);
}

// ---- runs ----

export interface RunListItem {
  run_id: string;
  label: string;
  created_at: string;
  duration_s: number | null;
  n_samples: number | null;
  strategy: string;
  model_type: string;
  experiment_name: string;
  kpis: Record<string, number>;
  job_id: string | null;
}

export async function listRuns(): Promise<RunListItem[]> {
  const d = await fetchJSON<{ runs: RunListItem[] }>("/api/runs");
  return d.runs ?? [];
}

export interface RunMeta extends RunListItem {
  channels: string[];
  experiment?: ExperimentT;
}

export function getRunMeta(runId: string): Promise<RunMeta> {
  return fetchJSON<RunMeta>(`/api/runs/${encodeURIComponent(runId)}`);
}

export type RunData = { t: number[] } & Record<string, (number | null)[]>;

export function getRunData(runId: string, channels: string[], decimate = 1): Promise<RunData> {
  const q = new URLSearchParams({ channels: channels.join(","), decimate: String(decimate) });
  return fetchJSON<RunData>(`/api/runs/${encodeURIComponent(runId)}/data?${q}`);
}

export function deleteRun(runId: string): Promise<unknown> {
  return deleteJSON(`/api/runs/${encodeURIComponent(runId)}`);
}
