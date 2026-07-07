# 参考基准对照协议

本文定义 4WIS Simulator 与外部参考模型或实测数据对照的最小协议。当前仓库尚未包含 CarSim/CarMaker 或实物台架数据；不得把内部一致性测试包装成外部对照证据。

## 目标

把核心模型能力从“内部测试通过”推进到“可解释的参考对照”：

- L3：与理论解、公开基准、CarSim/CarMaker 或独立工具结果对照。
- L4：与实车、台架或缩比 4WIS K&C 平台数据对照。

## 推荐首批基准

| 基准 | 参考来源 | 目标指标 | 目的 |
|---|---|---|---|
| 稳态圆周 | 解析自行车模型或外部 MBS 工具 | 稳态横摆率、侧偏角、转弯半径 | 校核低频操稳口径。 |
| 阶跃转向 60 km/h | ISO 7401 风格外部结果 | 横摆增益、上升时间、超调、稳定时间 | 校核瞬态响应和轮胎/惯量组合。 |
| ISO 3888 双移线 | CarSim/CarMaker 或公开参考轨迹 | 峰值横摆率、横向偏差、速度误差、侧偏角峰值 | 校核机动工况与实验系统。 |
| 单轮转向失效 | 台架/缩比车或独立仿真 | TTLD、横摆扰动峰值、横向偏差、缓解后残余 | 校核安全研究脚本的核心结论。 |

## 数据目录约定

外部或实测数据放入 `validation_data/`，每个 benchmark 一个子目录：

```text
validation_data/
└── <benchmark_id>/
    ├── manifest.json
    ├── reference.csv
    ├── sim4wis_experiment.yaml
    └── notes.md
```

`manifest.json` 必须说明：

- `benchmark_id`
- `source_type`: `analytic` / `external_tool` / `bench` / `scaled_vehicle` / `full_vehicle`
- `source_name`: 例如 `CarMaker 14.0`、`K&C rig v1`、`scaled_4wis_platform`
- `source_version`
- `vehicle_mapping`: 外部参数如何映射到 `VehicleParams`
- `channels`: 每列单位、坐标系、采样率
- `metrics`: 对照指标、容差、理由
- `limitations`: 不可外推范围

## CSV 通道约定

`reference.csv` 至少包含：

| 列 | 单位 | 说明 |
|---|---|---|
| `t` | s | 时间，从工况开始计时。 |
| `vx` | m/s | 车体纵向速度。 |
| `vy` | m/s | 车体横向速度。 |
| `yaw_rate` | rad/s | 横摆角速度。 |
| `pose_x` | m | 世界系 X。 |
| `pose_y` | m | 世界系 Y。 |
| `driver_steering` | normalized 或 rad | 必须在 manifest 标明。 |

可选通道：`delta_fl/fr/rl/rr`、`slip_alpha_*`、`fz_*`、`rack_force_*`、`motor_torque_*`。

## 评审规则

1. 没有 `manifest.json` 的数据不能进入可信度矩阵。
2. 没有单位和坐标系说明的数据只能作为探索材料，不能作为 L3/L4 证据。
3. 外部工具对照必须记录工具版本、轮胎模型、求解步长和车辆参数映射。
4. 实测对照必须记录传感器、采样率、滤波、同步方式和数据裁剪窗口。
5. 更新 `docs/validation_matrix.md` 前，必须能复现对照脚本输出。

## 自动检查

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py
```

脚本会校验每个 benchmark 子目录的 `manifest.json`、`reference.csv`、`sim4wis_experiment.yaml` 和 `notes.md`，并在数据齐全时运行 Sim4WIS 实验对比 `manifest.metrics` 中声明的指标。默认没有 benchmark 时不失败，但会明确输出外部验证证据仍缺失；需要强制要求数据时使用 `--require-data`。

当前支持自动对照的指标包括：

- `yaw_rate_peak_dps`
- `vy_peak_kmh`
- `speed_error_rms_kmh`
- `pose_y_peak_abs_m`
- `trajectory_error_rms_m`
- `trajectory_error_peak_m`

## 当前状态

- 已有内部黄金实验：`docs/golden_experiments.json`。
- 已有内部发布门禁：`scripts/pre_release_check.py`。
- 已有参考数据结构 checker：`scripts/check_reference_benchmarks.py`。
- 尚缺真实外部工具或实测数据；该缺口仍然是 v1.0 前的关键风险。
