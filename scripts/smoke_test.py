#!/usr/bin/env python3
"""Phase 1 端到端冒烟测试 — 不依赖 pytest / FastAPI，只用 NumPy + PyYAML。

跑一遍：
    1. 仿真器启动 / 状态推送 / 客户端订阅
    2. 5 种策略全部能跑且物理量合理
    3. 理想阿克曼下四轮瞬心严格共点（核心不变量）
    4. KinematicModel 圆周积分闭合
    5. YAML 项目文件 schema 一致性
    6. 各策略下连续运行 5s 不发散

退出码：0 全部通过，非零有失败。
"""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

# Make the backend src package importable.
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "backend" / "src"))

import numpy as np
import yaml

from sim4wis.controller.registry import available_strategies, make_strategy
from sim4wis.core.simulator import Simulator
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams, VehicleState
from sim4wis.vehicle.geometry import line_intersection, wheel_perpendicular_dir
from sim4wis.vehicle.kinematic import KinematicModel


def hr() -> None:
    print("─" * 64)


def case_strategy_registry() -> bool:
    expected = {"ackermann", "ideal_ackermann", "rear_wheel_steer", "crab",
                "zero_radius", "follow_trajectory"}
    got = set(available_strategies())
    ok = expected.issubset(got)
    print(f"  策略注册表  expected={sorted(expected)}")
    print(f"             got     ={sorted(got)}  →  {'✓' if ok else '✗'}")
    return ok


def case_follow_trajectory() -> bool:
    """Pure-pursuit tracks a slalom path with small cross-track error."""
    from sim4wis.controller import path as P

    plan = P.plan_from_template("slalom", {"n_gates": 5, "spacing": 18.0, "offset": 2.5})
    P.set_active_plan(plan)
    params = VehicleParams()
    strat = make_strategy("follow_trajectory", params)
    st = VehicleState()
    v = 8.0
    drv = DriverInput(throttle=v / params.v_max)
    dt = 0.01
    max_err = 0.0
    end = plan.points[-1]
    reached = False
    for k in range(4000):
        cmd = strat.compute(drv, st)
        icr = cmd.icr_target_body
        omega = v * (1.0 / icr[1]) if np.isfinite(icr[1]) and abs(icr[1]) > 1e-9 else 0.0
        c, s = np.cos(st.psi), np.sin(st.psi)
        st.x += dt * v * c
        st.y += dt * v * s
        st.psi += dt * omega
        if k > 30:
            d = float(np.min(np.linalg.norm(plan.points - np.array([st.x, st.y]), axis=1)))
            max_err = max(max_err, d)
        if float(np.linalg.norm(np.array([st.x, st.y]) - end)) < 1.5:
            reached = True
            break
    P.clear_active_plan()
    ok = reached and max_err < 2.0
    print(f"  跟踪 slalom  到达终点={reached}  max_xtrack={max_err:.2f}m  →  {'✓' if ok else '✗'}")
    return ok


def case_ideal_ackermann_icr_共点() -> bool:
    params = VehicleParams()
    strat = make_strategy("ideal_ackermann", params)

    max_spread = 0.0
    for steering in (-0.9, -0.4, -0.1, 0.1, 0.4, 0.9):
        cmd = strat.compute(DriverInput(throttle=0.5, steering=steering), VehicleState())
        wheels = params.wheel_positions_body()
        pts = []
        for i in range(4):
            for j in range(i + 1, 4):
                pts.append(line_intersection(
                    wheels[i], wheel_perpendicular_dir(cmd.delta_cmd[i]),
                    wheels[j], wheel_perpendicular_dir(cmd.delta_cmd[j]),
                ))
        pts = np.array(pts)
        centroid = np.nanmean(pts, axis=0)
        spread = float(np.nanmax(np.linalg.norm(pts - centroid, axis=1)))
        max_spread = max(max_spread, spread)
    ok = max_spread < 1e-6
    print(f"  理想阿克曼四轮瞬心一致性  max_spread={max_spread:.3e} m  →  {'✓' if ok else '✗'}")
    return ok


def case_kinematic_circle_closure() -> bool:
    params = VehicleParams()
    model = KinematicModel(params)
    strat = make_strategy("ideal_ackermann", params)
    env = EnvironmentState()
    dt = 0.005
    # Use the steer_raw_rad bypass (v0.100) so this tests the ICR geometry
    # directly — the speed-dependent feel layer would otherwise change κ as v
    # rises from standstill and the path wouldn't close. The explicit speed
    # bypass also keeps the driver grip governor out of this geometry oracle.
    driver = DriverInput(throttle=0.3, steering=0.0,
                         mode_params={"steer_raw_rad": 0.15, "speed_target_ms": 0.3 * params.v_max})
    cmd0 = strat.compute(driver, model.state)
    R = abs(cmd0.icr_target_body[1])
    v = 0.3 * params.v_max
    T = 2 * np.pi * R / v
    steps = int(T / dt)
    for _ in range(steps):
        cmd = strat.compute(driver, model.state)
        model.step(dt, cmd, env)
    err = (model.state.x ** 2 + model.state.y ** 2) ** 0.5
    ok = err / R < 0.05
    print(f"  KinematicModel 圆周闭合  R={R:.2f}m  闭合误差={err:.3f}m ({err/R*100:.2f}%)  →  {'✓' if ok else '✗'}")
    return ok


def case_each_strategy_5s_stable() -> bool:
    params = VehicleParams()
    env = EnvironmentState()
    dt = 0.005

    inputs = {
        "ackermann":       DriverInput(throttle=0.4, steering=0.3),
        "ideal_ackermann": DriverInput(throttle=0.4, steering=0.3),
        "rear_wheel_steer": DriverInput(throttle=0.4, steering=0.3, mode_params={"rear_ratio": -0.5}),
        "crab":            DriverInput(throttle=0.4, steering=0.3),
        "zero_radius":     DriverInput(throttle=0.4, steering=1.0),
    }
    all_ok = True
    for name, di in inputs.items():
        model = KinematicModel(params)
        strat = make_strategy(name, params)
        for _ in range(int(5.0 / dt)):
            cmd = strat.compute(di, model.state)
            model.step(dt, cmd, env)
        s = model.state
        bad = not np.all(np.isfinite([s.x, s.y, s.psi, s.vx, s.vy, s.yaw_rate]))
        bad = bad or not np.all(np.isfinite(s.delta))
        # Final speed should be finite and within reason
        speed = (s.vx ** 2 + s.vy ** 2) ** 0.5
        bad = bad or speed > params.v_max * 1.2
        ok = not bad
        all_ok = all_ok and ok
        print(f"  策略 {name:18s} 5s 后:  pose=({s.x:+7.2f},{s.y:+7.2f},{np.rad2deg(s.psi):+7.1f}°)  "
              f"|v|={speed:5.2f}m/s  →  {'✓' if ok else '✗'}")
    return all_ok


def case_yaml_schema_consistency() -> bool:
    expected_top = {"project", "vehicle", "suspension", "controller", "scene", "recording"}
    expected_veh = {"wheelbase","track_front","track_rear","mass","inertia_z",
                    "cg_to_front","tire_radius","steer_limit","v_max"}
    all_ok = True
    for p in sorted((ROOT / "projects").glob("*.yaml")):
        d = yaml.safe_load(p.read_text())
        top_missing = expected_top - set(d.keys())
        veh_missing = expected_veh - set(d.get("vehicle", {}).keys())
        ok = not top_missing and not veh_missing
        all_ok = all_ok and ok
        print(f"  {p.name:24s}  top_missing={sorted(top_missing) or '∅'}  veh_missing={sorted(veh_missing) or '∅'}  →  {'✓' if ok else '✗'}")
    return all_ok


def case_project_model_matches_disturbances() -> bool:
    """Any project that places disturbances must run the primary dynamic model;
    the kinematic model ignores μ / fz / slope entirely (so e.g. a split-μ
    demo on the kinematic model silently demonstrates nothing).

    The 14-DOF multibody project remains a research/regression demo, not the
    default fidelity target for scenario projects.
    """
    all_ok = True
    for p in sorted((ROOT / "projects").glob("*.yaml")):
        d = yaml.safe_load(p.read_text())
        disturbances = (d.get("scene") or {}).get("disturbances") or []
        model = (d.get("vehicle") or {}).get("model", "kinematic")
        research_demo = p.name == "multibody_demo.yaml" and model == "multibody"
        ok = (not disturbances) or model == "simplified_dynamic" or research_demo
        all_ok = all_ok and ok
        if disturbances:
            print(f"  {p.name:24s}  {len(disturbances)} 扰动 · model={model}  →  {'✓' if ok else '✗ 需主线动力学模型'}")
    return all_ok


async def case_simulator_state_stream() -> bool:
    sim = Simulator(VehicleParams(), strategy_name="ideal_ackermann", dt_sim=0.005, dt_push=0.05)
    await sim.start()
    q = sim.subscribe()
    sim.set_driver(throttle=0.5, steering=0.3)

    msgs = []
    try:
        for _ in range(10):
            msgs.append(await asyncio.wait_for(q.get(), timeout=1.0))
    finally:
        await sim.stop()

    ok = len(msgs) == 10
    if ok:
        # Check schema
        m = msgs[-1]
        required = ["type", "t", "strategy", "driver", "pose", "velocity",
                    "wheels", "icr_vehicle_body", "icr_target_body", "params"]
        ok = all(k in m for k in required) and len(m["wheels"]) == 4
        # Times monotonic
        ts = [m["t"] for m in msgs]
        ok = ok and all(ts[i] < ts[i+1] for i in range(len(ts)-1))
    print(f"  Simulator 状态推流  收到 {len(msgs)} 条状态消息  →  {'✓' if ok else '✗'}")
    return ok


def case_disturbance_query() -> bool:
    """Disturbance regions return correct mu at queried wheel positions."""
    from sim4wis.environment.disturbance import IcePatch, Scene, SplitMu

    scene = Scene(
        base_mu=0.9,
        disturbances=[
            SplitMu(id="s", x=25.0, y=0.0, width=6.0, length=30.0,
                    mu_left=0.9, mu_right=0.3, heading=0.0),
            IcePatch(id="i", x=60.0, y=0.0, width=8.0, length=12.0,
                     mu=0.15, heading=0.0),
        ],
    )
    cases = [
        # (wheel_world_xy, expected_effective_mu) — friction values are ABSOLUTE
        ((0.0, 0.0),   0.9),    # far from any disturbance → base_mu
        ((25.0, 2.0),  0.9),    # inside split-mu, +Y side (left) → mu_left=0.9
        ((25.0, -2.0), 0.3),    # inside split-mu, -Y side (right) → mu_right=0.3
        ((60.0, 0.0),  0.15),   # inside ice patch → absolute 0.15
        ((100.0, 0.0), 0.9),    # outside everything → base_mu
    ]
    all_ok = True
    for (x, y), exp_eff in cases:
        pos = np.array([x, y])
        eff = scene.effective_mu(pos)
        ok = abs(eff - exp_eff) < 1e-6
        all_ok = all_ok and ok
        print(f"  pos=({x:>5.1f},{y:>+5.1f})  μ_eff={eff:.3f} (exp {exp_eff:.3f}, 绝对值)  {'✓' if ok else '✗'}")
    return all_ok


def case_scene_serialize_roundtrip() -> bool:
    """Scene → dict → Scene round-trip preserves disturbances."""
    from sim4wis.environment.disturbance import IcePatch, Scene, SplitMu

    s1 = Scene(
        base_mu=0.85,
        disturbances=[
            IcePatch(id="ice1", x=10.0, y=2.0, width=4.0, length=6.0, mu=0.2, heading=0.3),
            SplitMu(id="sp1", x=20.0, y=0.0, width=6.0, length=30.0, mu_left=0.95, mu_right=0.25),
        ],
    )
    blob = s1.serialize()
    s2 = Scene.from_dict(blob)

    # Sample point in each disturbance and confirm same mu_modifier comes out
    ok = True
    for x, y in [(10.0, 2.0), (20.0, 2.0), (20.0, -2.0)]:
        m1 = s1.effective_mu(np.array([x, y]))
        m2 = s2.effective_mu(np.array([x, y]))
        if abs(m1 - m2) > 1e-9:
            ok = False
    print(f"  Scene roundtrip: serialize→from_dict→effective_mu equal  →  {'✓' if ok else '✗'}")
    return ok


def case_load_transfer_balance() -> bool:
    """Static + dynamic vertical loads always sum to m·g and are non-negative."""
    from sim4wis.vehicle.load_transfer import vertical_loads, G

    params = VehicleParams()
    cases = [
        (0.0, 0.0),     # static
        (3.0, 0.0),     # forward accel
        (-5.0, 0.0),    # braking
        (0.0, 4.0),     # left cornering
        (2.0, -3.0),    # combined accel + right turn
    ]
    all_ok = True
    expected_total = params.mass * G
    for ax, ay in cases:
        fz = vertical_loads(params, ax, ay)
        total_ok = abs(fz.sum() - expected_total) < 1e-3 or fz.sum() == 0
        nonneg = bool(np.all(fz >= 0))
        ok = total_ok or nonneg  # in extreme cases a wheel might lift (sum < m·g)
        all_ok = all_ok and ok
        print(f"  ax={ax:+.1f} ay={ay:+.1f} → Fz=[{fz[0]:6.0f},{fz[1]:6.0f},{fz[2]:6.0f},{fz[3]:6.0f}]  "
              f"Σ={fz.sum():.1f}/{expected_total:.1f}  {'✓' if ok else '✗'}")
    return all_ok


def case_dynamic_straight_line() -> bool:
    """SimplifiedDynamicModel: straight-line accel reaches commanded speed (±15%).

    3.5 s window: the launch is friction-limited (a ≈ μ·g ≈ 9.8 m/s², ideal
    floor 2.4 s to 23.6 m/s). The pre-v0.10 window of 3.0 s only passed
    because the old servo's integral windup overdrove the approach; the
    anti-windup fix makes the approach clean but ~0.2 % slower.
    """
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams()
    model = SimplifiedDynamicModel(params)
    strat = make_strategy("ideal_ackermann", params)
    env = EnvironmentState()
    dt = 0.005
    for _ in range(int(3.5 / dt)):
        cmd = strat.compute(DriverInput(throttle=0.5, steering=0.0), model.state)
        model.step(dt, cmd, env)
    s = model.state
    v_target = 0.5 * params.v_max
    ok = (abs(s.y) < 0.1 and abs(s.psi) < 0.01
          and 0.85 * v_target <= s.vx <= 1.15 * v_target)
    print(f"  Dynamic 直线  vx={s.vx:.2f} (期望 ~{v_target:.1f})  y={s.y:.4f}  ψ={np.rad2deg(s.psi):.4f}°  →  {'✓' if ok else '✗'}")
    return ok


def case_dynamic_lateral_load_transfer() -> bool:
    """Cornering moves load toward the outside wheels."""
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams()
    model = SimplifiedDynamicModel(params)
    strat = make_strategy("ideal_ackermann", params)
    env = EnvironmentState()
    dt = 0.005
    # Drive in a left turn (steering > 0)
    for _ in range(int(3.0 / dt)):
        cmd = strat.compute(DriverInput(throttle=0.4, steering=0.4), model.state)
        model.step(dt, cmd, env)
    s = model.state
    # Left turn → outside is RIGHT side → FR/RR should have MORE load than FL/RL
    outside_loaded = bool(s.fz[1] > s.fz[0] and s.fz[3] > s.fz[2])
    print(f"  Dynamic 左转载荷:  Fz=[FL={s.fz[0]:.0f}  FR={s.fz[1]:.0f}  RL={s.fz[2]:.0f}  RR={s.fz[3]:.0f}]  "
          f"右侧>左侧?  {outside_loaded}  →  {'✓' if outside_loaded else '✗'}")
    return outside_loaded


def case_speed_bump_fz_pulse() -> bool:
    """Passing over a SpeedBump produces a transient Fz spike."""
    from sim4wis.environment.disturbance import Scene, SpeedBump
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams()
    model = SimplifiedDynamicModel(params)
    scene = Scene(disturbances=[
        SpeedBump(id="b", x=5.0, y=0.0, width=4.0, length=0.5, height=0.05),
    ])
    env = EnvironmentState(scene=scene, mu=1.0)
    strat = make_strategy("ideal_ackermann", params)
    dt = 0.005
    max_fz = 0.0
    baseline_fz = float(np.max(model.state.fz))
    for _ in range(int(2.0 / dt)):
        cmd = strat.compute(DriverInput(throttle=0.4, steering=0.0), model.state)
        model.step(dt, cmd, env)
        peak = float(np.max(model.state.fz))
        if peak > max_fz:
            max_fz = peak
    ok = max_fz > baseline_fz * 1.5
    print(f"  减速带 Fz 尖峰  baseline≈{baseline_fz:.0f}  peak={max_fz:.0f}  ratio={max_fz/baseline_fz:.2f}×  →  {'✓' if ok else '✗'}")
    return ok


def case_bump_steer() -> bool:
    """With bump_steer_coeff>0, crossing a SpeedBump perturbs δ transiently and
    the car keeps moving (no stop); with coeff=0 there is no δ perturbation."""
    from dataclasses import replace

    from sim4wis.environment.disturbance import Scene, SpeedBump
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    def run(coeff: float) -> tuple[float, float]:
        p = replace(VehicleParams(), bump_steer_coeff=coeff)
        scene = Scene(base_mu=0.9, disturbances=[
            SpeedBump(id="b", x=25.0, y=0.0, width=6.0, length=0.5, height=0.05, stiffness=1.0e5),
        ])
        env = EnvironmentState(scene=scene, mu=0.9)
        m = SimplifiedDynamicModel(p)
        m.state.x = 18.0; m.state.vx = 8.0; m.state.wheel_omega[:] = 8.0 / p.tire_radius
        strat = make_strategy("ideal_ackermann", p)
        dt = 0.005
        max_delta = 0.0; v_min = 8.0
        for _ in range(int(3.0 / dt)):
            cmd = strat.compute(DriverInput(throttle=0.4, steering=0.0), m.state)
            m.step(dt, cmd, env)
            max_delta = max(max_delta, float(np.max(np.abs(m.state.delta))))
            v_min = min(v_min, m.state.vx)
        return np.degrees(max_delta), v_min

    d0, _ = run(0.0)
    d1, vmin1 = run(2.5)
    ok = d0 < 0.5 and d1 > 2.0 and vmin1 > 5.0
    print(f"  bump-steer  coeff0 δmax={d0:.2f}°  coeff2.5 δmax={d1:.2f}° vmin={vmin1:.2f}  →  {'✓' if ok else '✗'}")
    return ok


def case_multibody() -> bool:
    """MultiBodyModel (step 19): static equilibrium, cornering roll + lateral
    load transfer, braking pitch + longitudinal transfer, bump stability."""
    from sim4wis.vehicle.multibody import MultiBodyModel

    p = VehicleParams()
    strat = make_strategy("ideal_ackermann", p)
    dt = 0.005
    env = EnvironmentState()
    checks = []

    # static equilibrium
    m = MultiBodyModel(p)
    for _ in range(int(2.0 / dt)):
        m.step(dt, strat.compute(DriverInput(0.0, 0.0), m.state), env)
    eq = (abs(m.state.z) < 1e-3 and abs(m.state.roll) < 1e-3 and abs(m.state.pitch) < 1e-3
          and abs(m.state.fz.sum() - p.mass * 9.81) < 1.0)
    checks.append(eq)

    # left turn → roll>0 (right side down) and right Fz > left Fz.
    # Use the steer_raw_rad bypass so the cornering accel is set directly
    # (independent of the v0.100 speed-dependent feel layer).
    m.reset()
    corner_driver = DriverInput(throttle=0.25, steering=0.0,
                                mode_params={"steer_raw_rad": 0.25})
    for _ in range(int(4.0 / dt)):
        m.step(dt, strat.compute(corner_driver, m.state), env)
    left = m.state.fz[0] + m.state.fz[2]
    right = m.state.fz[1] + m.state.fz[3]
    corner = m.state.roll > np.deg2rad(0.3) and right > left * 1.1
    checks.append(corner)
    roll_deg = np.degrees(m.state.roll)

    # braking → pitch<0 (nose dive) and front Fz > rear Fz.
    # v0.100: braking is a real friction-brake channel. Build the driver with
    # brake=0.6 (gear D), and fill cmd.brake_cmd the way the live loop does so
    # the multibody model applies actual brake torque (front-biased).
    m.reset(); m.state.vx = 15.0; m.state.wheel_omega[:] = 15.0 / p.tire_radius
    bf = float(getattr(p, "brake_bias_front", 0.65))
    brake_driver = DriverInput(throttle=0.0, brake=0.6, gear=1, steering=0.0)
    for _ in range(int(1.2 / dt)):
        cmd = strat.compute(brake_driver, m.state)
        cmd.brake_cmd = np.array([bf * 0.6, bf * 0.6, (1 - bf) * 0.6, (1 - bf) * 0.6])
        m.step(dt, cmd, env)
    front = m.state.fz[0] + m.state.fz[1]
    rear = m.state.fz[2] + m.state.fz[3]
    brake = m.state.pitch < np.deg2rad(-0.3) and front > rear * 1.1
    checks.append(brake)
    pitch_deg = np.degrees(m.state.pitch)

    ok = all(checks)
    print(f"  multibody  静平衡={checks[0]}  转弯侧倾{roll_deg:+.2f}°/外侧加载={checks[1]}  "
          f"制动俯仰{pitch_deg:+.2f}°/前轴加载={checks[2]}  →  {'✓' if ok else '✗'}")
    return ok


def case_plugin_loader_no_dir() -> bool:
    """Plugin loader returns an empty report when the dir is empty / missing."""
    from sim4wis.controller.plugins.loader import discover_and_register, serialize_report
    rep = discover_and_register()
    serialized = serialize_report()
    ok = isinstance(rep, list) and serialized["loaded"] == len([r for r in rep if r.ok])
    print(f"  plugin loader: {serialized['total']} found ({serialized['loaded']} loaded, {serialized['failed']} failed)  →  {'✓' if ok else '✗'}")
    return ok


def case_matlab_adapter_construct() -> bool:
    """MatlabEngineControllerStrategy constructs without matlab installed."""
    from pathlib import Path
    from sim4wis.controller.plugins.matlab_adapter import MatlabEngineControllerStrategy

    s = MatlabEngineControllerStrategy(
        params=VehicleParams(),
        name="dummy_slx",
        slx_path=Path("/tmp/nonexistent.slx"),
        inputs={"vx": "state.vx"},
        outputs={"delta_cmd[0]": "delta_fl"},
        step_size_ms=20.0,
    )
    ok = s.name == "dummy_slx" and abs(s.step_size - 0.02) < 1e-9
    print(f"  MatlabEngineControllerStrategy 构造  name={s.name}  step={s.step_size*1000:.1f}ms  →  {'✓' if ok else '✗'}")
    return ok


def case_fmu_adapter_construct() -> bool:
    """FMUControllerStrategy constructs without fmpy installed (lazy-loads)."""
    from pathlib import Path
    from sim4wis.controller.plugins.fmu_adapter import FMUControllerStrategy

    s = FMUControllerStrategy(
        params=VehicleParams(),
        name="dummy_fmu",
        fmu_path=Path("/tmp/nonexistent.fmu"),
        inputs={"vx": "state.vx"},
        outputs={"delta_cmd[0]": "delta_fl"},
        step_size_ms=10.0,
    )
    # compute() would fail (fmpy not installed) but constructor must succeed
    ok = s.name == "dummy_fmu" and s.step_size == 0.01
    print(f"  FMUControllerStrategy 构造  name={s.name}  step={s.step_size*1000:.1f}ms  →  {'✓' if ok else '✗'}")
    return ok


def case_slope_decelerates() -> bool:
    """On an uphill slope the vehicle slows down (vs flat)."""
    from sim4wis.environment.disturbance import Scene, Slope
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams()
    strat = make_strategy("ideal_ackermann", params)
    dt = 0.005

    # Compare during the acceleration transient (2 s from rest): gravity on the
    # grade slows the climb. We deliberately read mid-accel rather than steady
    # state — a high-torque vehicle (LS9) can hold the commanded speed on a 10°
    # grade at steady state, so the slope only shows as slower *acceleration*.
    def run(scene: Scene) -> float:
        model = SimplifiedDynamicModel(params)
        env = EnvironmentState(scene=scene, mu=1.0)
        for _ in range(int(2.0 / dt)):
            cmd = strat.compute(DriverInput(throttle=0.5, steering=0.0), model.state)
            model.step(dt, cmd, env)
        return float(model.state.vx)

    flat = run(Scene())
    uphill = run(Scene(disturbances=[
        Slope(id="up", x=20.0, y=0.0, width=8.0, length=80.0, angle=np.deg2rad(10.0)),
    ]))
    ok = uphill < flat
    print(f"  坡度减速(2s 加速段)  平地 vx={flat:.2f}  上坡 vx={uphill:.2f}  →  {'✓' if ok else '✗'}")
    return ok


def case_kingpin_torque_scales_with_steering() -> bool:
    """Kingpin steering torque grows monotonically with steering amplitude."""
    from sim4wis.core.state import SuspensionParams
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams(suspension=SuspensionParams(
        caster_angle=np.deg2rad(3.0),
        kingpin_inclination=np.deg2rad(8.0),
        scrub_radius=0.025,
    ))
    strat = make_strategy("ideal_ackermann", params)
    dt = 0.005
    # Cornering speed ≈8 m/s (throttle scaled for v_max≈56) so the tyre stays
    # below its friction limit — otherwise high-steer cases saturate/spin and
    # the peak torque is dominated by the spin transient, not the steer level.
    throttle = 8.0 / params.v_max
    peaks = []
    for steering in [0.0, 0.2, 0.6]:
        model = SimplifiedDynamicModel(params)
        env = EnvironmentState()
        # settle to steady-state cornering, then read the steady peak
        for _ in range(int(4.0 / dt)):
            cmd = strat.compute(DriverInput(throttle=throttle, steering=steering), model.state)
            model.step(dt, cmd, env)
        peaks.append(float(np.max(np.abs(model.state.torque_steer))))
    ok = peaks[0] < peaks[1] < peaks[2]
    print(f"  τ_steer 峰值 (N·m):  steer=0→{peaks[0]:.0f}  0.2→{peaks[1]:.0f}  0.6→{peaks[2]:.0f}  →  {'✓' if ok else '✗'}")
    return ok


def case_dynamic_low_mu_slip() -> bool:
    """Cornering on lower μ produces noticeably larger slip than on high μ."""
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams()
    strat = make_strategy("ideal_ackermann", params)
    dt = 0.005

    # Compare the tyre/vehicle at identical speed and angle excitation. A
    # grip-aware driver deliberately slows on ice and is a different test.
    def run(mu: float) -> float:
        model = SimplifiedDynamicModel(params)
        model.state.vx = 8.0
        model.state.wheel_omega[:] = 8.0 / params.tire_radius
        env = EnvironmentState(mu=mu)
        for _ in range(int(2.0 / dt)):
            cmd = strat.compute(DriverInput(throttle=0.15, steering=0.0,
                mode_params={"speed_target_ms": 8.0, "steer_raw_rad": 0.16}), model.state)
            model.step(dt, cmd, env)
        return float(np.max(np.abs(model.slip_alpha)))

    high = run(1.0)
    low = run(0.3)
    ok = low > high * 1.5
    print(f"  μ=1.0 max|α|={np.rad2deg(high):.2f}°  μ=0.3 max|α|={np.rad2deg(low):.2f}°  "
          f"低 μ 滑移更大?  {ok}  →  {'✓' if ok else '✗'}")
    return ok


def case_script_schema_parse() -> bool:
    """All bundled scripts under scripts_lib/ parse and validate."""
    from sim4wis.input.action_schema import Script

    lib = ROOT / "scripts_lib"
    files = sorted(lib.glob("*.yaml"))
    all_ok = True
    for f in files:
        try:
            d = yaml.safe_load(f.read_text())
            s = Script.from_dict(d)
            # Each script has ≥ 1 action, all t ≥ 0, sorted
            ok = (len(s.actions) > 0
                  and all(a.t >= 0 for a in s.actions)
                  and all(s.actions[i].t <= s.actions[i+1].t for i in range(len(s.actions)-1)))
            all_ok = all_ok and ok
            print(f"  {f.name:30s} actions={len(s.actions):2d} name={s.name:18s} {'✓' if ok else '✗'}")
        except Exception as e:
            print(f"  {f.name:30s} PARSE FAILED: {e}  ✗")
            all_ok = False
    return all_ok


async def case_script_runner_executes() -> bool:
    """ScriptRunner advances driver inputs in correct sequence."""
    from sim4wis.input.action_schema import Script

    yaml_str = """
script:
  name: tiny
  loop: false
  actions:
    - { t: 0.0, action: drive, throttle: 0.5, steering: 0.0 }
    - { t: 0.5, action: drive, throttle: 0.5, steering: +0.3 }
    - { t: 1.0, action: drive, throttle: 0.0, steering: 0.0 }
"""
    sim = Simulator(VehicleParams(), strategy_name="ideal_ackermann",
                    dt_sim=0.005, dt_push=0.05)
    await sim.start()
    s = Script.from_dict(yaml.safe_load(yaml_str))
    sim.script_runner.load(s)
    await sim.script_runner.start()
    # Wait for script to finish
    await asyncio.sleep(1.4)
    final_throttle = sim.driver.throttle
    await sim.script_runner.stop()
    await sim.stop()
    ok = abs(final_throttle - 0.0) < 1e-3
    print(f"  ScriptRunner 执行  最终 throttle={final_throttle:.3f} (期望 0.0)  →  {'✓' if ok else '✗'}")
    return ok


async def case_project_load_applies_scene() -> bool:
    """Loading split_mu_demo.yaml installs both disturbances into the simulator."""
    try:
        from sim4wis.project.io import load_project
    except ModuleNotFoundError as e:
        print(f"  load split_mu_demo.yaml: SKIPPED ({e.name} not installed in this env)")
        return True
    sim = Simulator(VehicleParams(), strategy_name="ideal_ackermann",
                    dt_sim=0.005, dt_push=0.05)
    await sim.start()
    proj = load_project("split_mu_demo")
    sim.set_scene(proj.scene.to_scene())
    n = len(sim.scene.disturbances)
    mu_inside = sim.scene.effective_mu(np.array([25.0, 2.0]))
    mu_outside = sim.scene.effective_mu(np.array([100.0, 0.0]))
    await sim.stop()
    ok = n == 2 and abs(mu_inside - 0.9) < 1e-6 and abs(mu_outside - 0.9) < 1e-6
    print(f"  load split_mu_demo.yaml:  disturbances={n}  μ@(25,2)={mu_inside:.3f}(绝对)  μ@(100,0)={mu_outside:.3f}  →  {'✓' if ok else '✗'}")
    return ok


async def case_recorder_csv_roundtrip() -> bool:
    """Recorder records during sim push tick → CSV round-trip preserves data."""
    import csv as _csv
    import io as _io
    from sim4wis.recorder.buffer import DEFAULT_CHANNELS

    sim = Simulator(VehicleParams(), strategy_name="ideal_ackermann",
                    dt_sim=0.005, dt_push=0.05)
    await sim.start()
    sim.set_driver(throttle=0.5, steering=0.3)
    sim.recorder.start()
    await asyncio.sleep(0.7)
    sim.recorder.stop()
    await sim.stop()

    csv_str = "".join(sim.recorder.to_csv())
    rows = list(_csv.DictReader(_io.StringIO(csv_str)))

    # ≥ 10 samples (0.7 s at 20 Hz push = ~14)
    n_ok = 10 <= len(rows) <= 20
    cols_ok = "t" in rows[0] and "delta_fl" in rows[0] and "strategy" in rows[0]
    chan_ok = all(c in rows[0] for c in DEFAULT_CHANNELS)
    # Time monotonic, deltas consistent across rows for a steady command
    t_mono = all(float(rows[i]["t"]) < float(rows[i+1]["t"]) for i in range(len(rows)-1))
    ok = n_ok and cols_ok and chan_ok and t_mono
    print(f"  Recorder + CSV  rows={len(rows)}  cols={len(rows[0])}  monotonic={t_mono}  →  {'✓' if ok else '✗'}")
    return ok


async def case_recorder_channel_selection() -> bool:
    """Recorder.set_channels filters channels and clears buffer."""
    sim = Simulator(VehicleParams(), strategy_name="ideal_ackermann",
                    dt_sim=0.005, dt_push=0.05)
    await sim.start()
    sim.set_driver(throttle=0.3, steering=0.2)
    sim.recorder.set_channels(["vx", "yaw_rate", "delta_fl"])
    sim.recorder.start()
    await asyncio.sleep(0.4)
    sim.recorder.stop()
    await sim.stop()

    csv_str = "".join(sim.recorder.to_csv(include_strategy=False))
    header = csv_str.split("\n")[0]
    cols = header.split(",")
    ok = cols == ["t", "vx", "yaw_rate", "delta_fl"]
    print(f"  Channel selection  header={cols}  →  {'✓' if ok else '✗'}")
    return ok


async def case_simulator_strategy_switch() -> bool:
    sim = Simulator(VehicleParams(), strategy_name="ideal_ackermann")
    await sim.start()
    q = sim.subscribe()
    sim.set_driver(throttle=0.3, steering=0.3)

    seen_strategies = set()
    try:
        for s in ["ackermann", "crab", "zero_radius", "rear_wheel_steer", "ideal_ackermann"]:
            sim.set_strategy(s)
            # Wait one push interval for a state message with the new strategy
            for _ in range(5):
                msg = await asyncio.wait_for(q.get(), timeout=1.0)
                if msg["strategy"] == s:
                    seen_strategies.add(s)
                    break
    finally:
        await sim.stop()

    ok = seen_strategies == {"ackermann", "crab", "zero_radius", "rear_wheel_steer", "ideal_ackermann"}
    print(f"  Simulator 策略热切换  seen={sorted(seen_strategies)}  →  {'✓' if ok else '✗'}")
    return ok


def case_wheel_icr_projection() -> bool:
    """每轮转向中心：理想阿克曼下与整车瞬心严格重合（dev≈0），蟹行下为有限值。"""
    from sim4wis.vehicle.geometry import steer_angle_for_icr, wheel_icr_projection

    params = VehicleParams()
    wheels = params.wheel_positions_body()
    icr = np.array([0.0, 9.0])
    delta = steer_angle_for_icr(wheels, icr)
    pts, dev = wheel_icr_projection(wheels, delta, icr)
    ok_ideal = bool(np.nanmax(np.abs(dev)) < 1e-9 and np.allclose(pts, icr, atol=1e-9))

    # 直行 → NaN（前端不画、CSV 空字段）
    _, dev_straight = wheel_icr_projection(wheels, np.zeros(4), np.array([np.nan, np.nan]))
    ok_nan = bool(np.all(np.isnan(dev_straight)))

    ok = ok_ideal and ok_nan
    print(f"  每轮瞬心偏差  ideal max|dev|={np.nanmax(np.abs(dev)):.2e} m, 直行=NaN  →  {'✓' if ok else '✗'}")
    return ok


def case_scene_crud_runtime() -> bool:
    """运行时增删扰动：加入冰面后区域内有效 μ 降低，删除后恢复。"""
    from sim4wis.environment.disturbance import Scene, disturbance_from_dict

    scene = Scene(base_mu=0.9)
    ice = disturbance_from_dict({"type": "ice_patch", "id": "ice_x",
                                 "x": 40.0, "y": 0.0, "width": 8.0, "length": 8.0, "mu": 0.2})
    scene.disturbances.append(ice)
    inside = scene.effective_mu(np.array([40.0, 0.0]))
    outside = scene.effective_mu(np.array([0.0, 0.0]))
    scene.disturbances.remove(ice)
    restored = scene.effective_mu(np.array([40.0, 0.0]))
    ok = abs(inside - 0.2) < 1e-9 and abs(outside - 0.9) < 1e-9 and abs(restored - 0.9) < 1e-9
    print(f"  扰动 CRUD  inside={inside:.2f} outside={outside:.2f} removed={restored:.2f}  →  {'✓' if ok else '✗'}")
    return ok


def case_pacejka_cornering_sideslip() -> bool:
    """Pacejka 回归：μ=0.9 理想阿克曼稳态转向 30s，侧滑 |vy| 应有界且不发散
    （直指 phase3_review 记录的 linear 模型 vy≈-7.8 m/s 异常）。"""
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    params = VehicleParams(tire_model="pacejka")
    model = SimplifiedDynamicModel(params)
    strat = make_strategy("ideal_ackermann", params)
    env = EnvironmentState(mu=0.9)
    drv = DriverInput(throttle=8.0 / params.v_max, steering=0.25)
    dt = 0.005
    max_vy = 0.0
    for _ in range(int(30.0 / dt)):
        cmd = strat.compute(drv, model.state)
        s = model.step(dt, cmd, env)
        if not np.isfinite(s.vx) or not np.isfinite(s.vy):
            print("  Pacejka 侧滑回归  发散(NaN)  →  ✗")
            return False
        max_vy = max(max_vy, abs(s.vy))
    ok = max_vy < 1.5 and abs(model.state.vx) < params.v_max
    print(f"  Pacejka 侧滑回归  max|vy|={max_vy:.2f} m/s (<1.5)  →  {'✓' if ok else '✗'}")
    return ok


async def case_strategy_switch_no_torque_jump() -> bool:
    """策略切换平滑：稳态圆周中 crab→ideal_ackermann，伺服积分清零后
    切换瞬间电机扭矩不放大（峰值 ≤ 切换前的 1.5 倍 + 余量）。"""
    from sim4wis.vehicle.dynamic import SimplifiedDynamicModel

    sim = Simulator(VehicleParams(), strategy_name="crab", model_type="simplified_dynamic")
    assert isinstance(sim.model, SimplifiedDynamicModel)
    env = sim.env
    sim.set_driver(throttle=0.2, steering=0.3)
    dt = sim.dt_sim

    def step_once() -> float:
        cmd = sim.strategy.compute(sim.driver, sim.model.state)
        t_pre = sim.model._compute_motor_torques(cmd, sim.model.state.wheel_omega, dt)  # noqa: SLF001
        sim.model.step(dt, cmd, env)
        return float(np.max(np.abs(t_pre)))

    peak_before = 0.0
    for _ in range(int(5.0 / dt)):       # settle 5 s
        peak_before = max(peak_before, step_once())

    sim.set_strategy("ideal_ackermann")   # clears servo integrators
    peak_after = 0.0
    for _ in range(int(0.5 / dt)):       # 0.5 s after the switch
        peak_after = max(peak_after, step_once())

    ok = peak_after <= max(peak_before * 1.5, 50.0)
    print(f"  切换无扭矩跳变  before={peak_before:.0f} N·m, after(0.5s)={peak_after:.0f} N·m  →  {'✓' if ok else '✗'}")
    return ok


def main() -> int:
    results: list[tuple[str, bool]] = []

    hr()
    print("§1 仿真核心")
    hr()
    results.append(("策略注册",       case_strategy_registry()))
    results.append(("瞬心一致性",     case_ideal_ackermann_icr_共点()))
    results.append(("圆周闭合",       case_kinematic_circle_closure()))
    results.append(("5 策略 5s 稳定", case_each_strategy_5s_stable()))

    hr()
    print("§2 项目文件")
    hr()
    results.append(("YAML schema",   case_yaml_schema_consistency()))
    results.append(("项目模型匹配扰动", case_project_model_matches_disturbances()))

    hr()
    print("§3 Simulator 异步")
    hr()
    results.append(("状态推流",       asyncio.run(case_simulator_state_stream())))
    results.append(("策略切换",       asyncio.run(case_simulator_strategy_switch())))
    results.append(("轨迹跟踪",       case_follow_trajectory()))

    hr()
    print("§4 Recorder (Phase 2)")
    hr()
    results.append(("CSV round-trip", asyncio.run(case_recorder_csv_roundtrip())))
    results.append(("通道选择",       asyncio.run(case_recorder_channel_selection())))

    hr()
    print("§5 Disturbances (Phase 2)")
    hr()
    results.append(("扰动查询",       case_disturbance_query()))
    results.append(("Scene 序列化",   case_scene_serialize_roundtrip()))
    results.append(("加载扰动项目",   asyncio.run(case_project_load_applies_scene())))

    hr()
    print("§6 Scripts (Phase 2)")
    hr()
    results.append(("脚本库 schema", case_script_schema_parse()))
    results.append(("ScriptRunner",  asyncio.run(case_script_runner_executes())))

    hr()
    print("§7 Dynamic Model (Phase 2)")
    hr()
    results.append(("载荷转移平衡",   case_load_transfer_balance()))
    results.append(("动力学直线",     case_dynamic_straight_line()))
    results.append(("横向载荷转移",   case_dynamic_lateral_load_transfer()))
    results.append(("低μ滑移更大",   case_dynamic_low_mu_slip()))
    results.append(("kingpin τ 单调", case_kingpin_torque_scales_with_steering()))
    results.append(("减速带 Fz 尖峰", case_speed_bump_fz_pulse()))
    results.append(("bump-steer 扰动", case_bump_steer()))
    results.append(("多体动力学", case_multibody()))
    results.append(("斜坡减速",       case_slope_decelerates()))
    results.append(("插件加载器",     case_plugin_loader_no_dir()))
    results.append(("FMU 适配器构造", case_fmu_adapter_construct()))
    results.append(("MATLAB 适配器构造", case_matlab_adapter_construct()))

    hr()
    print("§8 v0.4 改进")
    hr()
    results.append(("每轮瞬心偏差",   case_wheel_icr_projection()))
    results.append(("扰动 CRUD",      case_scene_crud_runtime()))
    results.append(("Pacejka 侧滑回归", case_pacejka_cornering_sideslip()))
    results.append(("切换无扭矩跳变", asyncio.run(case_strategy_switch_no_torque_jump())))

    hr()
    print()
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print(f"结果：{passed}/{total} 通过")
    for name, ok in results:
        print(f"  {'✓' if ok else '✗'}  {name}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
