# 轮胎模型 — Phase 2

## 当前实现：线性 + 摩擦圆饱和

`backend/src/sim4wis/vehicle/tire.py` 提供 `LinearTireModel`，是 Phase 2 唯一的轮胎模型。

### 数学描述

每个轮胎在 wheel-aligned 坐标系下：
- 纵向力：`Fx_0 = C_κ · κ` （κ 为纵向滑移率）
- 横向力：`Fy_0 = -C_α · α` （α 为侧偏角）
- 自回正力矩：`Mz = -Fy · t_p` （t_p 为气胎拖距）

摩擦圆饱和：
```
|F_max| = μ · Fz
若 sqrt(Fx_0² + Fy_0²) > F_max：
    scale = F_max / sqrt(Fx_0² + Fy_0²)
    Fx = Fx_0 · scale,  Fy = Fy_0 · scale
否则：
    Fx = Fx_0,  Fy = Fy_0
```

### 参数与默认值

| 参数 | 默认值 | 单位 | 物理意义 |
|------|--------|------|----------|
| `c_alpha` | 70000 | N/rad | 侧偏刚度（cornering stiffness）|
| `c_kappa` | 100000 | N/无量纲 | 纵向滑移刚度 |
| `t_pneumatic` | 0.03 | m | 气胎拖距（pneumatic trail）|

默认值适用于 ~1800 kg 中型轿车的标准乘用车轮胎。
对于轻型车（<1500 kg）：`c_alpha` 50-60 kN/rad；
重型车（>2200 kg）：80-100 kN/rad。

### 滑移定义（与 Pacejka / SAE 一致）

```
α = atan2(vy_wheel, vx_wheel)
κ = (r·ω_wheel - vx_wheel) / max(|vx_wheel|, vmin)
```

其中 `vmin = 0.5 m/s` 是低速分母下限，防止数值发散。
α 正值表示车轮在朝其滚动方向"右侧"移动；
κ 正值表示车轮在驱动（滚动速度 > 地面速度），负值表示在制动。

### 局限性

1. **线性段**：仅在小 slip（α < ~3°, |κ| < ~0.1）范围内精确。大滑移时摩擦圆裁断使力幅恒定，缺少真实轮胎的"过峰下降"行为。
2. **无组合滑移耦合**：横纵向力先独立计算再裁断，不考虑 Pacejka 1989 中的"weighting functions"。
3. **无温度 / 磨损依赖**：μ 是固定的（来自 Scene），不随热量变化。

### Phase 3 升级路线

Phase 3 将引入 Pacejka 1989 / 2002（Magic Formula）：
```
Fy = D · sin(C · atan(B · α - E · (B · α - atan(B · α))))
```
其中 B / C / D / E 是经验拟合参数（来自轮胎试验报告），与 Fz 相关。
保持 `TireModel` 接口不变，只新增 `MagicFormulaTireModel` 类即可。

### 参考文献

1. Hans B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012.
2. Jörnsen Reimpell, *The Automotive Chassis: Engineering Principles*, 2nd ed., Butterworth-Heinemann, 2001 — §3.10 关于主销力矩。
3. SAE J670 vehicle dynamics terminology.
