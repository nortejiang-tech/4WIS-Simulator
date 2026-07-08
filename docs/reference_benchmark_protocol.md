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
- `provenance`: 独立来源必须提供工具求解/导出口径或实测传感器/滤波/同步口径；解析 `analytic` benchmark 可省略
- `source_artifacts`: 独立来源必须提供至少一个原始/导出/测量/报告文件的 `path`、`role` 和 `sha256`；解析 `analytic` benchmark 可省略
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
3. 外部工具对照必须记录工具版本、轮胎模型、求解步长、车辆参数来源和导出流程。
4. 实测对照必须记录传感器、采样率、滤波、同步方式、数据裁剪窗口和标定口径。
5. 独立来源必须把原始导出、测量日志、工具报告或参数文件留存在 benchmark 目录内，并在 `manifest.source_artifacts` 中记录 SHA-256；不能用 `reference.csv`、`manifest.json`、`sim4wis_experiment.yaml` 或 `notes.md` 这些生成件冒充原始证据。
6. 更新 `docs/validation_matrix.md` 前，必须能复现对照脚本输出。

## 自动检查

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py
```

脚本会校验每个 benchmark 子目录的 `manifest.json`、`reference.csv`、`sim4wis_experiment.yaml` 和 `notes.md`，并在数据齐全时运行 Sim4WIS 实验对比 `manifest.metrics` 中声明的指标。默认没有 benchmark 时不失败，但会明确输出外部验证证据仍缺失；需要强制要求数据时使用 `--require-data`。

`--require-data` 只证明至少存在一个可复现 benchmark，解析参考也可以满足该门槛。需要防止把解析参考误报成独立外部/实测来源时，使用：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --require-independent-source
```

该模式只有在至少一个通过检查的 `external_tool`、`bench`、`scaled_vehicle` 或 `full_vehicle` benchmark 存在时才通过；当前两个 `analytic` benchmark 不满足该门槛。

需要给评审人留存审查材料时，生成 Markdown review report：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --report docs/reports/reference_benchmark_review.md
```

报告会汇总每个 benchmark 的来源、独立来源 provenance、limitations、warnings/failures、通过校验的 source artifacts 及 SHA-256、每项指标的 Sim/Reference/Delta/Tolerance 和 `notes.md` 摘要。仓库当前留存的审查件为 `docs/reports/reference_benchmark_review.md`；该报告只是复现性与人工评审材料，不能自动提升 `docs/validation_matrix.md` 的可信度等级。

pre-release 会校验已提交的 review report 是否与当前 benchmark 数据一致；单独检查时运行：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --check-report docs/reports/reference_benchmark_review.md
```

`--check-report` 只比较当前渲染结果与已有文件，发现缺失或过期会失败并提示重新运行 `--report`，不会在验证过程中静默改写审查件。

当前支持从 `reference.csv` 自动计算参考值的指标包括：

- `yaw_rate_peak_dps`
- `vy_peak_kmh`
- `speed_error_rms_kmh`
- `pose_y_peak_abs_m`
- `trajectory_error_rms_m`
- `trajectory_error_peak_m`

`manifest.metrics` 也可以为 Sim4WIS KPI 提供显式 `reference_value`；只要该指标名存在于 `compute_kpis` 输出中，checker 会直接比较仿真 KPI 与 manifest 参考值。

归一化 `reference.csv` 和实验 YAML 就绪后，可以先生成待审 `manifest.metrics` 候选片段：

```bash
backend/.venv/bin/python scripts/suggest_reference_metrics.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh
```

该脚本只计算 checker 已支持指标的候选 `reference_value`，并故意把 `abs_tol` 和 `reason` 留成 `TODO`；必须由 reviewer 按外部来源精度、测量噪声和验收目的替换后，benchmark 才能通过正式 checker。

## 独立数据接入脚手架

接入 CarSim/CarMaker、台架、缩比车或实车数据时，先生成待补齐模板：

```bash
backend/.venv/bin/python scripts/scaffold_reference_benchmark.py carmaker_iso3888_dlc_60kmh \
  --source-type external_tool \
  --source-name CarMaker \
  --source-version 14.0 \
  --template iso3888_dlc_60kmh
```

默认输出到 `validation_data/.incoming/<benchmark_id>/`。`.incoming` 不会被正式 `validation_data/` 扫描当作证据；模板中的 `manifest.metrics` 为空、`reference.csv` 只有表头，因此即使直接扫描 `.incoming` 也会失败。只有在真实样本、车辆参数映射、指标容差、采样/滤波/同步说明和 `notes.md` 都补齐，并清理所有 `TODO` / `TBD` / placeholder 文本后，才运行：

外部工具或台架导出的原始 CSV 可以先归一到标准 `reference.csv` 通道：

```bash
backend/.venv/bin/python scripts/normalize_reference_csv.py \
  --input raw_export.csv \
  --output validation_data/.incoming/carmaker_iso3888_dlc_60kmh/reference.csv \
  --map t=Time_ms --unit t=ms \
  --map vx=Vx_kmh --unit vx=km/h \
  --map vy=Vy_kmh --unit vy=km/h \
  --map yaw_rate=YawRate_deg_s --unit yaw_rate=deg/s \
  --map pose_x=X_mm --unit pose_x=mm \
  --map pose_y=Y_mm --unit pose_y=mm \
  --map driver_steering=Steer_deg --unit driver_steering=deg \
  --crop-start-s 1.5 --crop-end-s 9.5 --zero-time
```

该脚本只负责列名映射、单位换算、可复现时间窗裁剪、时间归零、数值合法性和时间单调性检查；它不会创建或修改 `manifest.json`，不会选择指标容差，也不会让 benchmark 自动具备独立证据资格。使用 `--crop-start-s` / `--crop-end-s` 时，应把原始时间窗记录到 `manifest.provenance.crop_window_s` 或 `notes.md`。`manifest.provenance`、`manifest.channels`、`vehicle_mapping`、`metrics` 和 `notes.md` 仍必须由接入者按真实来源补齐并复核。需要候选指标片段时运行：

```bash
backend/.venv/bin/python scripts/suggest_reference_metrics.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh
```

`suggest_reference_metrics.py` 输出的 `metrics` 片段保留了 `TODO` 容差和理由，不能直接作为通过门禁的证据。

原始来源文件清单也建议用同一套命令生成，避免重复手填 `sha256`：

```bash
backend/.venv/bin/python scripts/suggest_reference_artifacts.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh raw_source_export.csv --role "raw CarMaker CSV export before Sim4WIS normalization"
```

该脚本会校验文件路径是否在 benchmark 目录内、是否为文件、是否为原始/导出/测量来源而非生成件，并把路径按 `as_posix` 形式、角色和校验值输出成 `source_artifacts` JSON 片段。该片段仍必须由接入者确认并替换占位符后写入 `manifest.json`。

同时必须把原始导出或测量/报告文件放在同一个 incoming benchmark 目录内，并填写 `manifest.source_artifacts`。示例：

```json
{
  "source_artifacts": [
    {
      "path": "raw_source_export.csv",
      "role": "raw CarMaker CSV export before Sim4WIS normalization",
      "sha256": "..."
    }
  ]
}
```

checker 会拒绝缺失的 artifact、越界路径、checksum 不匹配，以及把 `reference.csv` 等生成件当作原始证据的写法。

对 `external_tool`，`manifest.provenance` 至少需要 `solver_step_s`、`tire_model`、`vehicle_parameter_source` 和 `export_pipeline`。对 `bench`、`scaled_vehicle` 或 `full_vehicle`，至少需要 `sensor_suite`、`sampling_rate_hz`、`filtering`、`time_sync`、`crop_window_s` 和 `calibration`。

完成补齐后运行：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --root validation_data/.incoming --require-independent-source
```

之后，用 promotion 门禁检查并移动到正式 `validation_data/<benchmark_id>/`：

```bash
backend/.venv/bin/python scripts/promote_reference_benchmark.py carmaker_iso3888_dlc_60kmh --dry-run
backend/.venv/bin/python scripts/promote_reference_benchmark.py carmaker_iso3888_dlc_60kmh
```

`scripts/check_reference_benchmarks.py` 会拒绝 manifest 或 notes 中残留的占位符；`scripts/promote_reference_benchmark.py` 会拒绝未通过 checker 的模板、非独立来源和已存在的目标目录。promotion 成功后，再重新生成 `docs/reports/reference_benchmark_review.md` 和 `docs/reports/v1_readiness.md`。

## 当前状态

- 已有内部黄金实验：`docs/golden_experiments.json`。
- 已有内部发布门禁：`scripts/pre_release_check.py`。
- 已有参考数据结构 checker 与 reviewer report 输出：`scripts/check_reference_benchmarks.py`。
- 已有独立 reference 接入脚手架、原始 CSV 归一工具、source artifact checksum 约束与 promotion 门禁：`scripts/scaffold_reference_benchmark.py` 默认输出到 `validation_data/.incoming/`，`scripts/normalize_reference_csv.py` 只生成标准 `reference.csv`，`scripts/check_reference_benchmarks.py` 会校验独立来源 artifact 的 SHA-256，`scripts/promote_reference_benchmark.py` 只允许通过检查的独立来源进入正式 `validation_data/`。
- 已有两个解析参考 benchmark：`validation_data/analytic_steady_circle_30kmh/` 和 `validation_data/analytic_step_steer_30kmh/`。
- 已有当前审查报告：`docs/reports/reference_benchmark_review.md`，记录 2/2 解析 benchmark 通过、独立外部/实测 benchmark 为 0。
- 尚缺真实外部工具或实测数据；该缺口仍然是 v1.0 前的关键风险。
