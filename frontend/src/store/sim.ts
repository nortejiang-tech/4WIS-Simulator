import { create } from "zustand";
import type { PathPlan, Scenario, SimStateMessage } from "@/types/sim";

export type DistType = "ice_patch" | "split_mu" | "speed_bump" | "slope";

export interface ToastMsg {
  id: number;
  kind: "error" | "info";
  text: string;
}

export type RunSlot = "A" | "B";

// Top-level workflow pages (CarMaker-style rail): drive workbench, experiment
// composer, analysis workbench, vehicle datasets, scene editor, plus the two
// knowledge pages.
export type AppPage = "run" | "experiment" | "analysis" | "vehicle" | "scene" | "load" | "model";
export interface RunSnapshot {
  trajectory: number[];   // flat [x0,y0,x1,y1,...] world frame
  t: number[];
  vx: number[];
  yaw_rate: number[];
  icr_dev: number[];      // max |ICR deviation| across wheels per sample [m]
  motor_torque: number[]; // Σ |motor torque demand| across wheels per sample [N·m]
  strategy: string;
  label: string;          // e.g. "A · ideal_ackermann"
}

interface SimStore {
  online: boolean;
  state: SimStateMessage | null;
  strategies: string[];
  // Reference path (step 17) — fetched over REST, kept in sync via path_version.
  path: PathPlan | null;
  pathVersion: number;
  scenario: Scenario | null;
  scenarioVersion: number;
  // Waypoint editor draft (world-frame [x,y] points placed by clicking).
  editMode: boolean;
  draftWaypoints: [number, number][];
  // Disturbance editor: non-null type = "click canvas to place"; selection for
  // editing existing regions. Mutually exclusive with waypoint editMode.
  distPlaceType: DistType | null;
  selectedDistId: string | null;
  // 2D zoom + 3D camera pose, lifted here so the view survives 2D/3D switches.
  view2dPxm: number;
  camera3d: { position: [number, number, number]; target: [number, number, number] } | null;
  // Transient toasts (errors / confirmations).
  toasts: ToastMsg[];
  // Keyboard "hold speed" mode: when on, W/S nudge a *persistent* target speed
  // (no auto-return on release). Steering is independent (see steerReturn).
  holdSpeed: boolean;
  // Steering auto-return rate, 0..1 of max. 0 = hold (no return), 1 = 1.5× the
  // baseline return speed. Maps to an exponential decay rate in KeyboardInput.
  steerReturn: number;
  // Set by KeyboardInput so the UI can request a zero/stop without coupling.
  zeroRequest: number;
  // Fixed-speed cruise: when on, KeyboardInput holds throttle at cruiseSpeed.
  cruiseOn: boolean;
  cruiseSpeed: number;        // target speed [m/s]
  // Gamepad input (Web Gamepad API, standard mapping). `gamepadId` is set by
  // connect/disconnect events; when enabled, gamepad axes merge additively
  // with the keyboard axes in KeyboardInput's single-writer loop.
  gamepadEnabled: boolean;
  gamepadId: string | null;
  // Workflow page routing (in the store so any page can navigate, e.g.
  // "run batch → jump to analysis with these runs preselected").
  page: AppPage;
  analysisPreselect: string[];   // run ids the analysis page should auto-select
  // Measurement tool (2D canvas): click two world points to read a distance.
  measureMode: boolean;
  measurePts: [number, number][];   // world frame, length 0..2
  // UI theme.
  theme: "dark" | "light";
  // A/B run comparison: snapshot of a completed run for overlay.
  savedRuns: { A: RunSnapshot | null; B: RunSnapshot | null };
  showOverlay: boolean;
  // History for charts (capped FIFO buffers); written into on every state msg.
  // We pre-allocate per-channel arrays so chart components can subscribe.
  history: {
    t: number[];
    vx: number[];
    vy: number[];
    yaw_rate: number[];
    wheel_delta: [number[], number[], number[], number[]];
    wheel_torque: [number[], number[], number[], number[]];
    wheel_icr_dev: [number[], number[], number[], number[]];
    wheel_rack_force: [number[], number[], number[], number[]];
    wheel_motor_torque: [number[], number[], number[], number[]];
    wheel_slip_alpha: [number[], number[], number[], number[]];
  };
  // Trajectory (world frame): pairs of (x, y) — Konva-friendly flat array.
  trajectory: number[];

  setOnline: (b: boolean) => void;
  setStrategies: (s: string[]) => void;
  ingestState: (m: SimStateMessage) => void;
  clearHistory: () => void;
  setPath: (p: PathPlan | null, version: number) => void;
  setScenario: (s: Scenario | null, version: number) => void;
  setEditMode: (b: boolean) => void;
  addDraftWaypoint: (x: number, y: number) => void;
  clearDraft: () => void;
  setDistPlaceType: (t: DistType | null) => void;
  setSelectedDistId: (id: string | null) => void;
  setView2dPxm: (v: number) => void;
  setCamera3d: (c: SimStore["camera3d"]) => void;
  pushToast: (kind: ToastMsg["kind"], text: string) => void;
  dismissToast: (id: number) => void;
  setHoldSpeed: (b: boolean) => void;
  setSteerReturn: (v: number) => void;
  setCruiseOn: (b: boolean) => void;
  setCruiseSpeed: (ms: number) => void;
  setGamepadEnabled: (b: boolean) => void;
  setGamepadId: (id: string | null) => void;
  setPage: (p: AppPage) => void;
  setAnalysisPreselect: (runIds: string[]) => void;
  setMeasureMode: (b: boolean) => void;
  addMeasurePt: (x: number, y: number) => void;
  clearMeasure: () => void;
  setTheme: (t: "dark" | "light") => void;
  requestZero: () => void;
  saveRun: (slot: RunSlot) => void;
  clearRun: (slot: RunSlot) => void;
  setShowOverlay: (b: boolean) => void;
}

const HISTORY_LIMIT = 1800;     // ~30s at 60 Hz
const TRAJ_LIMIT = 2 * 4000;    // 4000 points × 2 floats
const DEFAULT_PXM = 35;         // 2D pixels per metre

function push<T>(arr: T[], v: T, limit: number): void {
  arr.push(v);
  if (arr.length > limit) arr.splice(0, arr.length - limit);
}

function emptyHistory(): SimStore["history"] {
  return {
    t: [],
    vx: [],
    vy: [],
    yaw_rate: [],
    wheel_delta: [[], [], [], []],
    wheel_torque: [[], [], [], []],
    wheel_icr_dev: [[], [], [], []],
    wheel_rack_force: [[], [], [], []],
    wheel_motor_torque: [[], [], [], []],
    wheel_slip_alpha: [[], [], [], []],
  };
}

let toastSeq = 1;

export const useSimStore = create<SimStore>((set, get) => ({
  online: false,
  state: null,
  strategies: [],
  path: null,
  pathVersion: -1,
  scenario: null,
  scenarioVersion: -1,
  editMode: false,
  draftWaypoints: [],
  distPlaceType: null,
  selectedDistId: null,
  view2dPxm: DEFAULT_PXM,
  camera3d: null,
  toasts: [],
  holdSpeed: false,
  steerReturn: 0.667,   // ≈ baseline return rate (slider midpoint feel)
  zeroRequest: 0,
  cruiseOn: false,
  cruiseSpeed: 5,       // m/s (≈ 18 km/h)
  gamepadEnabled: true,
  gamepadId: null,
  page: "run",
  analysisPreselect: [],
  measureMode: false,
  measurePts: [],
  theme: (typeof localStorage !== "undefined" && localStorage.getItem("sim4wis-theme") === "light")
    ? "light" : "dark",
  savedRuns: { A: null, B: null },
  showOverlay: true,
  history: emptyHistory(),
  trajectory: [],
  setOnline: (b) => set({ online: b }),
  setStrategies: (s) => set({ strategies: s }),
  ingestState: (m) => {
    const h = get().history;
    const traj = get().trajectory;
    push(h.t, m.t, HISTORY_LIMIT);
    push(h.vx, m.velocity.vx, HISTORY_LIMIT);
    push(h.vy, m.velocity.vy, HISTORY_LIMIT);
    push(h.yaw_rate, m.velocity.yaw_rate, HISTORY_LIMIT);
    for (let i = 0; i < 4; i++) {
      push(h.wheel_delta[i], m.wheels[i].delta, HISTORY_LIMIT);
      push(h.wheel_torque[i], m.wheels[i].torque_steer, HISTORY_LIMIT);
      push(h.wheel_icr_dev[i], m.wheels[i].icr_dev ?? NaN, HISTORY_LIMIT);
      push(h.wheel_rack_force[i], m.wheels[i].rack_force ?? 0, HISTORY_LIMIT);
      push(h.wheel_motor_torque[i], m.wheels[i].motor_torque_demand ?? 0, HISTORY_LIMIT);
      push(h.wheel_slip_alpha[i], m.wheels[i].slip_alpha ?? 0, HISTORY_LIMIT);
    }
    push(traj, m.pose.x, TRAJ_LIMIT);
    push(traj, m.pose.y, TRAJ_LIMIT);
    const patch: Partial<SimStore> = { state: m };
    if (typeof m.path_version === "number" && m.path_version !== get().pathVersion) {
      patch.pathVersion = m.path_version;
    }
    if (typeof m.scenario_version === "number" && m.scenario_version !== get().scenarioVersion) {
      patch.scenarioVersion = m.scenario_version;
    }
    set(patch);
  },
  clearHistory: () => set({ history: emptyHistory(), trajectory: [] }),
  setPath: (p, version) => set({ path: p, pathVersion: version }),
  setScenario: (s, version) => set({ scenario: s, scenarioVersion: version }),
  setEditMode: (b) => set({
    editMode: b,
    draftWaypoints: b ? get().draftWaypoints : [],
    // Mutually exclusive with the disturbance editor + measure tool.
    distPlaceType: b ? null : get().distPlaceType,
    measureMode: b ? false : get().measureMode,
  }),
  addDraftWaypoint: (x, y) => set({ draftWaypoints: [...get().draftWaypoints, [x, y]] }),
  clearDraft: () => set({ draftWaypoints: [] }),
  setDistPlaceType: (t) => set({
    distPlaceType: t,
    editMode: t != null ? false : get().editMode,
    draftWaypoints: t != null ? [] : get().draftWaypoints,
    measureMode: t != null ? false : get().measureMode,
  }),
  setSelectedDistId: (id) => set({ selectedDistId: id }),
  setView2dPxm: (v) => set({ view2dPxm: Math.max(4, Math.min(200, v)) }),
  setCamera3d: (c) => set({ camera3d: c }),
  pushToast: (kind, text) => {
    const id = toastSeq++;
    set({ toasts: [...get().toasts.slice(-3), { id, kind, text }] });
    window.setTimeout(() => get().dismissToast(id), kind === "error" ? 6000 : 3000);
  },
  dismissToast: (id) => set({ toasts: get().toasts.filter((t) => t.id !== id) }),
  setHoldSpeed: (b) => set({ holdSpeed: b }),
  setSteerReturn: (v) => set({ steerReturn: Math.max(0, Math.min(1, v)) }),
  setCruiseOn: (b) => set({ cruiseOn: b }),
  setCruiseSpeed: (ms) => set({ cruiseSpeed: Math.max(0, ms) }),
  setGamepadEnabled: (b) => set({ gamepadEnabled: b }),
  setGamepadId: (id) => set({ gamepadId: id }),
  setPage: (p) => set({ page: p }),
  setAnalysisPreselect: (runIds) => set({ analysisPreselect: runIds }),
  setMeasureMode: (b) => set({
    measureMode: b,
    measurePts: b ? get().measurePts : [],
    // Mutually exclusive with the waypoint / disturbance editors.
    editMode: b ? false : get().editMode,
    distPlaceType: b ? null : get().distPlaceType,
  }),
  addMeasurePt: (x, y) => {
    const pts = get().measurePts;
    // Third click starts a fresh measurement.
    const next: [number, number][] = pts.length >= 2 ? [[x, y]] : [...pts, [x, y]];
    set({ measurePts: next });
  },
  clearMeasure: () => set({ measurePts: [] }),
  setTheme: (t) => {
    if (typeof localStorage !== "undefined") localStorage.setItem("sim4wis-theme", t);
    set({ theme: t });
  },
  requestZero: () => set({ zeroRequest: get().zeroRequest + 1 }),
  saveRun: (slot) => {
    const s = get();
    const h = s.history;
    const n = h.t.length;
    const icr_dev = new Array<number>(n);
    const motor_torque = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      let maxDev = 0, sumTq = 0;
      for (let w = 0; w < 4; w++) {
        const d = h.wheel_icr_dev[w][i];
        if (d != null && !Number.isNaN(d)) maxDev = Math.max(maxDev, Math.abs(d));
        sumTq += Math.abs(h.wheel_motor_torque[w][i] ?? 0);
      }
      icr_dev[i] = maxDev;
      motor_torque[i] = sumTq;
    }
    const snap: RunSnapshot = {
      trajectory: [...s.trajectory],
      t: [...h.t],
      vx: [...h.vx],
      yaw_rate: [...h.yaw_rate],
      icr_dev,
      motor_torque,
      strategy: s.state?.strategy ?? "—",
      label: `${slot} · ${s.state?.strategy ?? "—"}`,
    };
    set({ savedRuns: { ...s.savedRuns, [slot]: snap } });
  },
  clearRun: (slot) => set({ savedRuns: { ...get().savedRuns, [slot]: null } }),
  setShowOverlay: (b) => set({ showOverlay: b }),
}));
