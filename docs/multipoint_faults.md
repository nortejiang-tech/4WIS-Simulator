# 多点故障时间线（v0.105.0）

在「场景 → 故障注入」选择故障、多个车轮、相对延时、持续时间，点击「加入时间线」。重复添加可安排错时故障。点击「应用时间线」原子替换当前计划；同一延时的多轮故障同时触发。界面有双前轮同时卡滞、前后轮错时两个起点，支持调整自由转向的惯量/阻尼/摩擦/反驱效率。当前计划与本地草稿分开显示。时钟是仿真时钟，暂停即停止计时；重置从 t=0 重放计划。

## 支持的物理含义

|规范类型|旧实时 API 名称|物理/观测含义|
|---|---|---|
|stuck_zero|motor_stuck_zero|故障目标归零，保留原执行器滞后与速率限制；不是瞬时机械跳到零|
|stuck_value|motor_stuck_angle|故障固定目标角，value 为 rad；同样保留执行器动态|
|limited|motor_limited_range|指令权限缩小到 ±value，value≥0；不是突然把现有轮角裁切到限幅|
|stuck_hold|motor_stuck_hold|发生时锁住转向机构坐标；静态前束和 K&C 仍作用于真实轮角|
|free_caster|motor_free_caster|失电、非自锁机构的被动转向；道路主销力矩驱动，绕过有源伺服|
|sensor_bias|sensor_bias|观测偏差相加；value 单位 rad|
|sensor_noise|sensor_noise|固定 seed 下的零均值高斯观测噪声；value 为标准差 rad|
|sensor_dropout|sensor_dropout|保持故障前最后一次内部采样值；恢复后重新采样|

传感器故障只改观测，不进入转向控制器反馈，也不改轮胎受力。不能用此功能冒充真实闭环传感器故障响应。

卡滞/自由转向要求 `simplified_dynamic` 或 `multibody`。未启用详细转向系统时使用默认独立轮端模型；启用详细系统后，轮端执行器故障要求 `architecture: 4wis` 和 `angle_control.enabled: true`。共享齿条上的轮端独立机械故障会被拒绝，其约束反力需要另外建模。

`free_caster` 的降阶方程为 `J ω̇ = −ηrev τkingpin − c ω − τc Sign(ω)`，隐式阻尼/库仑摩擦步可在低于起动摩擦时静止；机械止挡吸收向外角速度。初始角/角速度来自实际机构，不从零开始。主销力矩沿用现有轮胎机械拖距、气动回正力矩、scrub 与简化 KPI 模型。忽略转向惯量对车体的反作用及完整三维主销轴、高频摆振；不声称实车相关性。

## 时间、组合与数据

- 所有 API 使用绝对仿真秒，窗口为 `[t_start,t_end)`；`t_end: null` 持续有效。GUI 的相对秒在提交时转为绝对秒。
- 积分遇到事件边界切分，同刻故障在同一物理状态触发。外层控制步/记录节拍不变，动力学内部步长上限 1.25 ms，降低车身/轮速分步误差。
- 同一车轮同一时段只能有一种执行器故障；相同类型观测故障也不能重复重叠。相邻窗口可以衔接。不同车轮不限制，同轮偏差+噪声相加，掉线优先保持。
- 负时间、非有限值、无效窗口、负标准差、超出车辆限位、重复 ID、冲突配置均拒绝。一次最多 64 条故障，失败不部分写入。
- 观测在接受的内部步采样；读取 WebSocket/REST/MCP 不消耗随机数。固定 seed、配置、外层步长和输入可重放；改变步长会改变白噪声采样序列。
- 导出 `delta_*` 为真实轮角，`delta_reported_*` 为观测；`fault_mode_*` 为 0 正常、1 故障目标、2 限幅、3 卡滞、4 自由转向；`sensor_fault_active_*` 为观测故障标志；`steer_rate_*` 为机构角速度 rad/s。
- `torque_steer_*` / `rack_force_*` / `motor_torque_*` 仍是道路负载/理论需求链，不能将自由转向时的理论电机需求解释成实际有源电机输出。
- run 元数据 `fault_timeline` 保存配置、实际事件时间和触发配置。实时录制还保存开始/结束快照。数值异常会保留状态并暂停实时仿真，不再自动重置后伪装继续。

## REST

实时驾驶的 `GET/POST /api/faults/schedule` 读取/原子替换时间线，body 为 `{faults:[...],seed:0}`。原 `POST /api/faults`、PATCH 启停、DELETE 单条/全部继续可用。运行中的计划替换会重新捕获立即生效的卡滞位置；需要可复现实验时优先使用隔离 Agent session 或实验 YAML。

实验 `faults` 数组使用同一 schema；`fault_seed` 默认为 0。`wheel` 为 0=FL、1=FR、2=RL、3=RR，也兼容名称。完整示例见 `experiments/multipoint_staggered.yaml`。

## MCP：Agent 从创建到报告

便携包的 `agent_mcp.command`（Mac）或 `agent_mcp.bat`（Windows）可用 `--print-config` 输出主机配置。Web UI 和 MCP 使用相同嵌入式 Python。先调用 `describe_capabilities` 获取完整 schema。

1. `create_session`：`{"experiment":{"name":"multi_example","model_type":"simplified_dynamic","strategy":"ideal_ackermann"}}`，取得 session_id 和 revision。
2. `set_fault_schedule`：如下配置同时与错时故障。`t_start` 必须不早于当前 session.state.t；创建时也可直接将故障放入 `experiment.faults`。

```json
{
  "session_id":"<create_session 返回的 ID>",
  "request_id":"fault-plan-1",
  "expected_revision":0,
  "seed":20261113,
  "faults":[
    {"id":"fl-jam","fault_type":"stuck_hold","wheel":0,"t_start":2.123,"t_end":6.123},
    {"id":"fr-jam","fault_type":"stuck_hold","wheel":1,"t_start":2.123,"t_end":6.123},
    {"id":"rr-free","fault_type":"free_caster","wheel":3,"t_start":4.321,"t_end":7.321},
    {"id":"rl-bias","fault_type":"sensor_bias","wheel":2,"value":0.02,"t_start":5.543}
  ]
}
```

3. 用返回的新 revision 调 `step_session`，每次 1..1000 步；例如 `{session_id,request_id:"drive-1",expected_revision:1,steps:1000,control:{speed_target_ms:8,front_angle_rad:0.03}}`。
4. `observe_session` 查看 `fault_timeline.events` 和状态。调度重试必须使用相同 request_id 和原 body；不同 body 或过期 revision 会拒绝。`faults:[]` 清除计划，也需要当前 revision。
5. `export_session` 获取完整 CSV/JSON/ZIP、SHA-256 和图表链接；`close_session` 释放资源。

样例分析见 [多点故障分析](reports/multipoint_v0.105.0/analysis.md)，可离线打开同目录 `analysis.html`。内部测试不能取代轮胎/机构标定、台架或实车验证。
