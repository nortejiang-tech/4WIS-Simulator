# procedures/ — 客观试验模板

每个文件是一份可直接跑的 study spec，把一项标准试验的**工况、指标与目标带**
一次写定：

```bash
sim4wis study run procedures/iso13674_oncentre.yaml --dry-run
sim4wis study run procedures/iso13674_oncentre.yaml
```

| 模板 | 口径 | 量什么 | 需要 |
|---|---|---|---|
| `iso13674_oncentre.yaml` | ISO 13674-1 | 中心区力矩梯度、摩擦感、死区、灵敏度、横摆相位 | 转向系统被控对象层 |
| `friction_slow_ramp.yaml` | 频域分离协议 | 0.02 Hz、50 km/h 慢斜坡下的迟滞环宽与死区——摩擦分量 | 转向系统被控对象层 |
| `step_steer_golden.yaml` | ISO 7401 风格 | 角阶跃 @60 km/h 的横摆 KPI（黄金回归源） | 无 |
| `iso3888_dlc_golden.yaml` | ISO 3888-1/-2 | 开环双移线 @60 km/h 的 KPI（黄金回归源） | 无 |

两个 `*_golden.yaml` 是 `check_golden_experiments.py` 的回归源（S1：黄金并入
procedures）。它们与 GUI 实验库里的同名实验（`experiments/*.yaml`）必须逐字段一致，
检查器会校验（twin check）——改动时两边一起改。

## 为什么试验要成为模板，而不是每次现写

**工况就是测量的一半。** 中心区力矩梯度随扫掠幅值变化——这是试验的性质，
不是估计器的毛病，也正是 ISO 用侧向加速度来定幅值的原因。一个不带工况的
"力矩梯度 0.89 N·m/°"不是测量值，是一个数字。模板把工况钉死，并把实际达到的
工况（`onc_ay_amplitude_g`、`onc_frequency_hz`）一并记录进结果表。

模板自带 `criteria` 自检——跑出来的确实是一次中心区试验而不是一次操纵试验；
`targets:` 则指向产品级的要求集，见 [`docs/targets_guide.md`](../docs/targets_guide.md)。

复制一份改成自己的工况即可；改完记得同时改 `study:` 名字，否则两次研究会同名。
