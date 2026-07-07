# validation_data

该目录用于保存解析参考、外部参考模型、台架、缩比车或实车验证数据。当前仓库已包含一个解析参考 benchmark；不得把未经说明的数据放入这里，也不得把解析数据包装成外部工具或实测证据。

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
- `reference.csv` 保持原始对照数据的最小可复现通道。
- `sim4wis_experiment.yaml` 是用于复现同一工况的实验定义。
- `notes.md` 记录人工判断、数据裁剪和已知问题。

详细协议见 `docs/reference_benchmark_protocol.md`。

## 可运行检查

从仓库根目录运行：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py
```

默认情况下，如果没有任何 benchmark 子目录，脚本会通过并明确提示外部验证证据仍缺失。需要在发布或审查时强制要求至少一个对照数据集时，使用：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --require-data
```

当前 checker 支持以下自动对照指标：

- `yaw_rate_peak_dps`
- `vy_peak_kmh`
- `speed_error_rms_kmh`
- `pose_y_peak_abs_m`
- `trajectory_error_rms_m`
- `trajectory_error_peak_m`

每个 `manifest.json` 的 `metrics` 项至少需要给出 `abs_tol`、`rel_tol` 或 `tolerance` 之一；也可以提供 `reference_value` 覆盖从 `reference.csv` 自动计算出的参考值。

需要生成给评审人看的 Markdown 摘要时：

```bash
backend/.venv/bin/python scripts/check_reference_benchmarks.py --report docs/reports/reference_benchmark_review.md
```

该报告会包含指标误差表和 `notes.md` 摘要，但不会自动把任何能力提升到 L3/L4；可信度等级仍需人工审查真实来源、限制和误差解释。

## 当前 benchmark

- `analytic_steady_circle_30kmh/`：解析稳态圆周参考，覆盖 kinematic ideal-Ackermann 在 30 km/h、归一化转向 0.05 下的横摆率、侧向速度、速度误差、横向位移和轨迹误差。该数据只构成解析 L3 参考证据，不是外部工具或实测证据。
