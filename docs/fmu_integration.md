# Simulink → 4WIS Simulator 集成

算法同事在 Simulink 中开发的控制策略可以通过两种方式接入 sim4wis：

| 方式 | 速度 | 部署 | 推荐场景 |
|------|------|------|----------|
| **FMU 导出** | 接近实时 | 算法同事和使用者机器都不需要 MATLAB | 团队共享、回归测试 |
| **MATLAB Engine 直连** | 慢（5-15 s 启动 + 10-100 ms/步） | 仅自己机器需要 MATLAB | 快速调试、参数扫调 |

两种方式都使用同一个 `ControllerStrategy` 抽象基类，对 sim4wis 主循环透明。

---

## 方式 A：FMU 导出（推荐）

### 1. 在 Simulink 建模

约束：
- 模型需要有明确的 **Inport / Outport** 块
- 推荐使用 Sample-time 固定（例如 10 ms）以匹配 sim4wis 的 step_size_ms
- 不要使用 to-workspace 或 to-file 块（FMU 内部不可见）

### 2. 配置 FMU 导出

需要 **Simulink Coder** + **FMI Kit for Simulink** 工具箱：
```matlab
% Configuration Parameters → Code Generation → System Target File
% 选择: gensil_fmi_cs.tlc  (Co-Simulation FMI 2.0)
%
% 或者命令行：
configSet = getActiveConfigSet(modelName);
set_param(configSet, 'SystemTargetFile', 'gensil_fmi_cs.tlc');
rtwbuild(modelName);   % 生成 <modelName>.fmu
```

### 3. 准备 sidecar YAML

将 `<modelName>.fmu` 拷贝到 `<repo>/plugins/strategies/`，同名 `.fmu.yaml`：

```yaml
name: my_strategy             # 在策略选择器里显示的名字
description: "MPC + 理想阿克曼前馈"
inputs:                        # FMU 输入变量名 → sim4wis 状态路径
  vx:       state.vx
  vy:       state.vy
  yaw_rate: state.yaw_rate
  fz_fl:    state.fz[0]
  fz_fr:    state.fz[1]
  fz_rl:    state.fz[2]
  fz_rr:    state.fz[3]
  throttle: driver.throttle
  steering: driver.steering
outputs:                       # ControlCommand 字段 ← FMU 输出变量名
  delta_cmd[0]:       delta_fl
  delta_cmd[1]:       delta_fr
  delta_cmd[2]:       delta_rl
  delta_cmd[3]:       delta_rr
  wheel_speed_cmd[0]: omega_fl
  wheel_speed_cmd[1]: omega_fr
  wheel_speed_cmd[2]: omega_rl
  wheel_speed_cmd[3]: omega_rr
step_size_ms: 10
```

### 4. 启动 sim4wis

```bash
pip install fmpy            # 若尚未装
python scripts/start.py
```

启动后 `/api/plugins` 应当列出该 FMU：

```json
{
  "total": 1, "loaded": 1, "failed": 0,
  "items": [{"name": "my_strategy", "path": "...", "ok": true}]
}
```

策略选择器里也会出现 `my_strategy`，可以像内置策略一样选用。

---

## 方式 B：MATLAB Engine 直连

适合开发调试阶段，不想每次改完都重新导出 FMU。

### 1. 安装环境

- MATLAB R2020a 及以上（同机器，同 Python 体系）
- Python 包：
  ```bash
  pip install matlabengine
  ```

### 2. 准备 .slx + sidecar

将 `<modelName>.slx` 放到 `plugins/strategies/`，同名 `.slx.yaml`：

```yaml
name: my_simulink_dev
description: "本地 MATLAB Engine 直跑"
inputs:
  vx:       state.vx
  yaw_rate: state.yaw_rate
  throttle: driver.throttle
  steering: driver.steering
outputs:
  delta_cmd[0]: delta_fl
  delta_cmd[1]: delta_fr
  delta_cmd[2]: delta_rl
  delta_cmd[3]: delta_rr
step_size_ms: 20
```

输入会写到 MATLAB 的 base workspace 同名变量。输出从 base workspace 读同名变量。
确保模型用 `To Workspace` 块（或 set_param/get_param）把结果写到 workspace 变量。

### 3. 选用策略

切换到 `my_simulink_dev` 后，第一次 step 时 sim4wis 会自动 `start_matlab()`。
预期 5-15 秒延迟。后续每个 step 大约 10-100 ms。

---

## 表达式语法

`inputs` 中右侧表达式支持的最小子集：
- `state.<attr>` — 读 VehicleState 字段
- `driver.<attr>` — 读 DriverInput 字段
- `params.<attr>` — 读 VehicleParams 字段
- `[idx]` — 数组下标访问

例如：
```yaml
fz_fr: state.fz[1]
torque_fl: state.torque_steer[0]
mass: params.mass
```

不支持算术、函数调用、临时变量 —— 这是有意为之，避免任意代码执行。
如果需要复杂计算，在 Simulink 模型内部完成。

---

## 故障排除

| 现象 | 原因 |
|------|------|
| 启动日志 `Plugin discovery: 0 loaded, 0 failed` | `plugins/strategies/` 为空或不存在 |
| `0 loaded, 1 failed: missing sidecar yaml` | 有 .fmu 但缺 .fmu.yaml |
| `fmpy not installed` | `pip install fmpy` |
| FMU 加载报 GUID 不匹配 | FMU 损坏或 Simulink 端编译失败 — 重新导出 |
| MATLAB engine 启动报 `Engine couldn't start` | 检查 MATLAB 版本（≥ R2020a）和 matlabengine 兼容性 |
| 策略输出全为 0 | sidecar `outputs` 字段拼写错误 — 检查 FMU 实际变量名 |

可以从 `/api/plugins` 看每个 plugin 的 `error` 字段定位问题。
