// Thin WebSocket wrapper — owns reconnect (exponential backoff), heartbeat
// (half-open detection), and routes messages into the Zustand store.

import type { ClientMessage, PathCone, PathMark, Scenario, SimStateMessage } from "@/types/sim";
import { useSimStore } from "@/store/sim";
import { fetchJSON, postJSON } from "@/api/http";

let ws: WebSocket | null = null;
let reconnectTimer: number | null = null;
let reconnectAttempt = 0;
let heartbeatTimer: number | null = null;
let lastRxWall = 0;
let pendingOutbox: ClientMessage[] = [];

const HEARTBEAT_INTERVAL_MS = 5000;
const STALE_AFTER_MS = 3000;   // no frames for this long after a ping → assume half-open

function url(): string {
  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${window.location.host}/ws/state`;
}

function flushOutbox() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  for (const m of pendingOutbox) ws.send(JSON.stringify(m));
  pendingOutbox = [];
}

function scheduleReconnect() {
  if (reconnectTimer != null) return;
  // 1s, 2s, 4s … capped at 15s, with jitter so multiple tabs don't sync up.
  const delay = Math.min(1000 * 2 ** reconnectAttempt, 15000) * (0.8 + Math.random() * 0.4);
  reconnectAttempt += 1;
  reconnectTimer = window.setTimeout(() => {
    reconnectTimer = null;
    connectSimSocket();
  }, delay);
}

function startHeartbeat() {
  stopHeartbeat();
  heartbeatTimer = window.setInterval(() => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    // The 60 Hz state stream is itself the liveness signal; if nothing has
    // arrived for a while, the link is half-open — force a reconnect.
    if (Date.now() - lastRxWall > HEARTBEAT_INTERVAL_MS + STALE_AFTER_MS) {
      try { ws.close(); } catch { /* triggers onclose → reconnect */ }
      return;
    }
    try { ws.send(JSON.stringify({ type: "ping", t: Date.now() })); } catch { /* ignore */ }
  }, HEARTBEAT_INTERVAL_MS);
}

function stopHeartbeat() {
  if (heartbeatTimer != null) {
    window.clearInterval(heartbeatTimer);
    heartbeatTimer = null;
  }
}

export function connectSimSocket() {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING))
    return;

  ws = new WebSocket(url());
  ws.onopen = () => {
    reconnectAttempt = 0;
    lastRxWall = Date.now();
    useSimStore.getState().setOnline(true);
    flushOutbox();
    startHeartbeat();
  };
  ws.onclose = () => {
    useSimStore.getState().setOnline(false);
    stopHeartbeat();
    scheduleReconnect();
  };
  ws.onerror = () => { /* close handler will retry */ };
  ws.onmessage = (e) => {
    lastRxWall = Date.now();
    try {
      const msg = JSON.parse(e.data) as SimStateMessage | { type: string };
      if (msg && msg.type === "state") {
        useSimStore.getState().ingestState(msg as SimStateMessage);
      }
      // "pong" and unknown types just refresh lastRxWall.
    } catch {
      /* ignore malformed */
    }
  };
}

export function sendMessage(m: ClientMessage) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(m));
  } else {
    pendingOutbox.push(m);
    if (pendingOutbox.length > 200) pendingOutbox.shift();
    connectSimSocket();
  }
}

export function setDriver(throttle: number, steering: number, mode_params?: Record<string, unknown>) {
  sendMessage({ type: "driver", throttle, steering, mode_params });
}

export function setStrategy(name: string) {
  sendMessage({ type: "strategy", name });
}

export function resetSim() {
  sendMessage({ type: "reset" });
  useSimStore.getState().clearHistory();
}

export function setDriverMode(mode_params: Record<string, unknown>) {
  sendMessage({ type: "driver", mode_params });
}

// ---- Reference path (step 17) ----

interface PathResponse {
  version: number;
  name: string;
  label?: string;
  notes?: string;
  closed: boolean;
  points: [number, number][];
  // Cones used to be bare [x, y] pairs; accept both so a stale backend (or a
  // recorded fixture) still renders instead of throwing.
  cones: (PathCone | [number, number])[];
  marks?: PathMark[];
}

function normalizeCone(c: PathCone | [number, number]): PathCone {
  return Array.isArray(c)
    ? { x: c[0], y: c[1], kind: "cone", color: "#f97316", height: 0.75 }
    : c;
}

function ingestPath(d: PathResponse) {
  useSimStore.getState().setPath(
    {
      name: d.name,
      label: d.label ?? d.name,
      notes: d.notes ?? "",
      closed: d.closed,
      points: d.points,
      cones: (d.cones ?? []).map(normalizeCone),
      marks: d.marks ?? [],
    },
    d.version,
  );
}

export async function fetchPath() {
  ingestPath(await fetchJSON<PathResponse>("/api/path"));
}

export async function setPathTemplate(name: string, params: Record<string, number> = {}) {
  ingestPath(await postJSON<PathResponse>("/api/path/template", { name, params }));
}

export async function setPathWaypoints(points: [number, number][], closed = false) {
  ingestPath(await postJSON<PathResponse>("/api/path/waypoints", { points, closed, name: "custom" }));
}

export async function clearPath() {
  await postJSON("/api/path/clear");
  useSimStore.getState().setPath(null, useSimStore.getState().pathVersion + 1);
}

// ---- Scenarios (static driving environments) ----

interface ScenarioResponse { version: number; scenario: Scenario | null }

function ingestScenario(d: ScenarioResponse) {
  useSimStore.getState().setScenario(d.scenario, d.version);
}

export async function fetchScenarios(): Promise<{ name: string; label: string }[]> {
  const d = await fetchJSON<{ scenarios: { name: string; label: string }[] }>("/api/scenarios");
  return d.scenarios ?? [];
}

export async function fetchScenario() {
  ingestScenario(await fetchJSON<ScenarioResponse>("/api/scenario"));
}

export async function loadScenario(name: string) {
  ingestScenario(await postJSON<ScenarioResponse>(`/api/scenarios/${encodeURIComponent(name)}/load`, {}));
}

export async function clearScenario() {
  ingestScenario(await postJSON<ScenarioResponse>("/api/scenario/clear", {}));
}

export async function setSceneMu(mu: number) {
  await postJSON("/api/scene/mu", { mu });
}

export async function setModel(model_type: string) {
  await postJSON("/api/model", { model_type });
  useSimStore.getState().clearHistory();
}
