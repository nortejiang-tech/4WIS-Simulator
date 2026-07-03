/**
 * VehicleParamsContext — one shared vehicle-parameter edit buffer for the whole
 * vehicle page, so the numeric ParamsPanel and the draggable geometry diagrams
 * write into the same pending edits and commit with one "应用" button.
 *
 * `value(path)` returns the live (edited-or-server) value; `setValue` records a
 * pending edit; `apply` POSTs the diff and rebuilds the model.
 */

import {
  createContext, useCallback, useContext, useEffect, useMemo, useState,
} from "react";

import { fetchJSON, postJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import type { Params } from "@/vehicle/geometryModel";

interface Ctx {
  params: Params | null;         // server truth + edits merged (live view)
  raw: Params | null;            // server truth only
  value: (path: string) => number;
  setValue: (path: string, v: number) => void;
  tireModel: string;
  setTireModel: (m: string) => void;
  dirty: boolean;
  busy: boolean;
  apply: () => Promise<void>;
  reload: () => void;
}

const VehicleParamsCtx = createContext<Ctx | null>(null);

const getPath = (obj: Params | null, path: string): number => {
  if (!obj) return 0;
  return path.split(".").reduce<any>((acc, k) => acc?.[k], obj) ?? 0;
};
const setNested = (target: Record<string, any>, path: string, v: number) => {
  const keys = path.split(".");
  let cur = target;
  for (let i = 0; i < keys.length - 1; i++) {
    cur[keys[i]] = { ...(cur[keys[i]] ?? {}) };
    cur = cur[keys[i]];
  }
  cur[keys[keys.length - 1]] = v;
};
const mergeEdits = (raw: Params | null, edits: Record<string, number>): Params | null => {
  if (!raw) return null;
  const next: Params = JSON.parse(JSON.stringify(raw));
  Object.entries(edits).forEach(([p, v]) => setNested(next, p, v));
  return next;
};

export function VehicleParamsProvider({ children }: { children: React.ReactNode }) {
  const pushToast = useSimStore((s) => s.pushToast);
  const [raw, setRaw] = useState<Params | null>(null);
  const [edits, setEdits] = useState<Record<string, number>>({});
  const [tireModel, setTireModelState] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  const reload = useCallback(() => {
    fetchJSON<Params>("/api/params")
      .then((p) => { setRaw(p); setEdits({}); setTireModelState(null); })
      .catch((e) => pushToast("error", `读取参数失败：${e?.message ?? e}`));
  }, [pushToast]);

  useEffect(() => { reload(); }, [reload]);

  const params = useMemo(() => mergeEdits(raw, edits), [raw, edits]);
  const value = useCallback((path: string) => edits[path] ?? getPath(raw, path), [edits, raw]);
  const setValue = useCallback((path: string, v: number) =>
    setEdits((e) => (e[path] === v ? e : { ...e, [path]: v })), []);
  const setTireModel = useCallback((m: string) => setTireModelState(m), []);
  const dirty = Object.keys(edits).length > 0 || tireModel != null;

  const apply = useCallback(async () => {
    setBusy(true);
    try {
      const body: Record<string, unknown> = {};
      Object.entries(edits).forEach(([p, v]) => setNested(body, p, v));
      if (tireModel != null) body.tire_model = tireModel;
      const updated = await postJSON<Params>("/api/params", body);
      setRaw(updated); setEdits({}); setTireModelState(null);
      pushToast("info", "参数已应用（车辆动力学状态已重置）");
    } catch (e: any) {
      pushToast("error", `参数被拒绝：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  }, [edits, tireModel, pushToast]);

  const ctx: Ctx = {
    params, raw, value, setValue,
    tireModel: tireModel ?? (raw?.tire_model as string) ?? "linear",
    setTireModel, dirty, busy, apply, reload,
  };
  return <VehicleParamsCtx.Provider value={ctx}>{children}</VehicleParamsCtx.Provider>;
}

export function useVehicleParams(): Ctx {
  const c = useContext(VehicleParamsCtx);
  if (!c) throw new Error("useVehicleParams must be used within VehicleParamsProvider");
  return c;
}
