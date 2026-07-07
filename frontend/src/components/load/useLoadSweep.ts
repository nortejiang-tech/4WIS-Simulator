import { useCallback, useEffect, useState } from "react";

import { fetchJSON, postJSON } from "@/api/http";
import { useSimStore } from "@/store/sim";
import {
  paramToBody,
  rad,
  rangeValues,
  uniqueSorted,
  type BodyCoupling,
  type LoadSweepResponse,
  type ProfileListItem,
  type VehicleParams,
} from "./types";

interface SweepInputs {
  wheelIndex: number;
  mode: string;
  mu: number;
  speedMaxKmh: number;
  speedSteps: number;
  profileSpeedKmh: number;
  angleMaxDeg: number;
  angleSteps: number;
  bodyCoupling: BodyCoupling;
}

export function useLoadSweep() {
  const pushToast = useSimStore((s) => s.pushToast);
  const [params, setParams] = useState<VehicleParams | null>(null);
  const [profiles, setProfiles] = useState<ProfileListItem[]>([]);
  const [result, setResult] = useState<LoadSweepResponse | null>(null);
  const [busy, setBusy] = useState(false);

  const refreshProfiles = useCallback(() => {
    fetchJSON<{ profiles: ProfileListItem[] }>("/api/vehicle-profiles")
      .then((d) => setProfiles(d.profiles ?? []))
      .catch((e) => pushToast("error", `读取车型库失败：${e?.message ?? e}`));
  }, [pushToast]);

  const loadCurrentParams = useCallback(() => {
    fetchJSON<VehicleParams>("/api/params")
      .then(setParams)
      .catch((e) => pushToast("error", `读取参数失败：${e?.message ?? e}`));
  }, [pushToast]);

  useEffect(() => {
    refreshProfiles();
    loadCurrentParams();
  }, [refreshProfiles, loadCurrentParams]);

  const runSweep = useCallback(async (inputs: SweepInputs) => {
    if (!params) return;
    setBusy(true);
    try {
      const profileSpeedMs = Math.max(0, Math.min(inputs.profileSpeedKmh, inputs.speedMaxKmh)) / 3.6;
      const speeds = uniqueSorted([
        ...rangeValues(0, inputs.speedMaxKmh / 3.6, inputs.speedSteps),
        profileSpeedMs,
      ]);
      const angles = rangeValues(-rad(inputs.angleMaxDeg), rad(inputs.angleMaxDeg), inputs.angleSteps);
      const body = {
        params: paramToBody(params),
        speeds,
        angles,
        wheel_index: inputs.wheelIndex,
        mode: inputs.mode,
        mu: inputs.mu,
        body_coupling: inputs.bodyCoupling,
      };
      setResult(await postJSON<LoadSweepResponse>("/api/load-analysis/sweep", body, 15000));
    } catch (e: any) {
      setResult((prev) => prev ?? {
        rows: [],
        body_coupling: inputs.bodyCoupling,
        summary: { warnings: [] },
      });
      pushToast("error", `负载扫图失败：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  }, [params, pushToast]);

  const loadProfile = useCallback(async (name: string) => {
    setBusy(true);
    try {
      const d = await fetchJSON<{ vehicle: VehicleParams }>(
        `/api/vehicle-profiles/${encodeURIComponent(name)}`,
      );
      setParams(d.vehicle);
      setResult(null);
      pushToast("info", `已载入车型 ${name}`);
    } catch (e: any) {
      pushToast("error", `载入车型失败：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  }, [pushToast]);

  const saveProfile = useCallback(async (name: string) => {
    if (!params) return;
    setBusy(true);
    try {
      await postJSON(`/api/vehicle-profiles/${encodeURIComponent(name)}`, {
        label: name,
        vehicle: paramToBody(params),
      });
      refreshProfiles();
      pushToast("info", `车型已保存：${name}`);
    } catch (e: any) {
      pushToast("error", `保存车型失败：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  }, [params, pushToast, refreshProfiles]);

  const applyProfileToSim = useCallback(async (name: string) => {
    setBusy(true);
    try {
      await postJSON(`/api/vehicle-profiles/${encodeURIComponent(name)}/apply`, {});
      loadCurrentParams();
      pushToast("info", `已应用到当前仿真：${name}`);
    } catch (e: any) {
      pushToast("error", `应用车型失败：${e?.message ?? e}`);
    } finally {
      setBusy(false);
    }
  }, [loadCurrentParams, pushToast]);

  const updateParams = useCallback((next: VehicleParams) => {
    setParams(next);
    setResult(null);
  }, []);

  const invalidate = useCallback(() => setResult(null), []);

  return {
    params,
    profiles,
    result,
    busy,
    setParams: updateParams,
    runSweep,
    loadProfile,
    saveProfile,
    applyProfileToSim,
    invalidate,
  };
}
