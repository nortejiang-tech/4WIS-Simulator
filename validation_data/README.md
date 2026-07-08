# validation_data

该目录用于保存解析参考、外部参考模型、台架、缩比车或实车验证数据。当前仓库已包含两个解析参考 benchmark；不得把未经说明的数据放入这里，也不得把解析数据包装成外部工具或实测证据。

目录结构：

```text
validation_data/
└── <benchmark_id>/
    ├── manifest.json
    ├── reference.csv
    ├── sim4wis_experiment.yaml
    └── notes.md
```

要求：

- `manifest.json` 说明来源、版本、车辆参数映射、通道单位、坐标系和容差。
- 独立来源的 `manifest.json` 还必须包含 `provenance` 和 `source_artifacts`，记录工具求解/导出口径或实测传感器/滤波/同步口径，以及原始/导出/测量/报告文件的相对路径、角色和 SHA-256。
- `reference.csv` 保持原始对照数据的最小可复现通道。
- `sim4wis_experiment.yaml` 是用于复现同一工况的实验定义。
- `notes.md` 记录人工判断、数据裁剪和已知问题。

详细协议见 `docs/reference_benchmark_protocol.md`。

## 独立数据接入

新的 CarSim/CarMaker、台架、缩比车或实车数据应先进入 `validation_data/.incoming/<benchmark_id>/`，不要直接放进正式 `validation_data/<benchmark_id>/`。可用脚手架创建待补齐目录：

```bash
backend/.venv/bin/python scripts/scaffold_reference_benchmark.py carmaker_iso3888_dlc_60kmh --source-type external_tool --source-name CarMaker --source-version 14.0 --template iso3888_dlc_60kmh
```

如果原始数据来自外部 CSV 导出，可用归一工具生成标准 `reference.csv`：

```bash
backend/.venv/bin/python scripts/normalize_reference_csv.py --input raw_export.csv --output validation_data/.incoming/carmaker_iso3888_dlc_60kmh/reference.csv --map t=Time_ms --unit t=ms --map vx=Vx_kmh --unit vx=km/h --map vy=Vy_kmh --unit vy=km/h --map yaw_rate=YawRate_deg_s --unit yaw_rate=deg/s --map pose_x=X_mm --unit pose_x=mm --map pose_y=Y_mm --unit pose_y=mm --map driver_steering=Steer_deg --unit driver_steering=deg --crop-start-s 1.5 --crop-end-s 9.5 --zero-time
```

`normalize_reference_csv.py` 只做列映射、单位换算、可复现时间窗裁剪、时间归零和基础数值检查；`manifest.json`、指标容差、车辆参数映射、采样/滤波说明和 `notes.md` 仍必须按真实来源人工补齐。使用裁剪选项时，要把原始裁剪时间窗记录进 `manifest.provenance.crop_window_s` 或 `notes.md`。独立来源还必须填写 `manifest.provenance`，并把原始导出、测量日志、工具报告或参数文件保存在 benchmark 目录内，在 `manifest.source_artifacts` 中记录 SHA-256；checker 会拒绝缺失 provenance、缺失 artifact、checksum 不匹配、越界路径，以及把 `reference.csv` 等生成件当作原始证据。补齐并清理占位符后，先 `--dry-run` promotion，再移动到正式目录：

```bash
backend/.venv/bin/python scripts/suggest_reference_metrics.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh
backend/.venv/bin/python scripts/suggest_reference_artifacts.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh raw_source_export.csv --role "raw source export for CarMaker normalization"
backend/.venv/bin/python scripts/check_reference_benchmarks.py --incoming-audit --incoming-root validation_data/.incoming
backend/.venv/bin/python scripts/promote_reference_benchmark.py carmaker_iso3888_dlc_60kmh --dry-run
backend/.venv/bin/python scripts/promote_reference_benchmark.py carmaker_iso3888_dlc_60kmh
```

`suggest_reference_metrics.py` 只生成可粘贴的 `manifest.metrics` 候选片段，并把容差和理由保留为 `TODO`；这些值必须经人工审查替换后才可能通过 checker。`suggest_reference_artifacts.py` 生成可复用的 `manifest.source_artifacts` 清单片段（`path`、`role`、`sha256`），用于把原始/导出/测量/报告文件的来源链路先行标准化，再交由 checker/promotion 做复核。

## 可运行检查

从仓库根目录运行：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py
```

默认情况下，如果没有任何 benchmark 子目录，脚本会通过并明确提示外部验证证据仍缺失。需要在发布或审查时强制要求至少一个对照数据集时，使用：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --require-data
```

`--require-data` 只要求存在可复现 benchmark，解析参考也满足该条件。需要强制要求至少一个通过检查的独立外部工具、台架、缩比车或实车来源时，使用：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --require-independent-source
```

当前 checker 支持以下从 `reference.csv` 自动计算参考值的指标：

- `yaw_rate_peak_dps`
- `vy_peak_kmh`
- `speed_error_rms_kmh`
- `pose_y_peak_abs_m`
- `trajectory_error_rms_m`
- `trajectory_error_peak_m`

每个 `manifest.json` 的 `metrics` 项至少需要给出 `abs_tol`、`rel_tol` 或 `tolerance` 之一；也可以提供 `reference_value` 覆盖从 `reference.csv` 自动计算出的参考值。对 Sim4WIS 已输出的 KPI，checker 会按同名 KPI 与 manifest 的 `reference_value` 直接比较。

可用以下命令从已有 incoming benchmark 生成待审 metrics 片段：

```bash
backend/.venv/bin/python scripts/suggest_reference_metrics.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh
```

需要生成给评审人看的 Markdown 摘要时：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --report docs/reports/reference_benchmark_review.md
```

如果要快速盘点 `.incoming` 目录里待接入的数据块是否可 promotion，可运行：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --incoming-audit --incoming-root validation_data/.incoming
backend/.venv/bin/python scripts/check_reference_benchmarks.py --incoming-audit --incoming-root validation_data/.incoming --incoming-report docs/reports/incoming_reference_benchmark_audit.md
```

该报告会包含指标误差表、通过校验的 provenance、source artifacts/SHA-256 和 `notes.md` 摘要，但不会自动把任何能力提升到 L3/L4；可信度等级仍需人工审查真实来源、限制和误差解释。

## 当前 benchmark

- `analytic_steady_circle_30kmh/`：解析稳态圆周参考，覆盖 kinematic ideal-Ackermann 在 30 km/h、归一化转向 0.05 下的横摆率、侧向速度、速度误差、横向位移和轨迹误差。该数据只构成解析 L3 参考证据，不是外部工具或实测证据。
- `analytic_step_steer_30kmh/`：解析阶跃转向参考，覆盖 kinematic ideal-Ackermann 在 30 km/h、1.0 s 归一化转向阶跃 0.05 下的横摆峰值、横摆增益、上升/稳定时间、侧向速度、速度误差、横向位移和轨迹误差。该数据只构成解析 L3 参考证据，不是外部工具或实测证据。
