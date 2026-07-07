# validation_data

该目录用于保存外部参考模型、台架、缩比车或实车验证数据。当前仓库只放协议和示例结构，不放未经说明的数据。

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
