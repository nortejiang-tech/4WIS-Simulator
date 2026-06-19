# 路面扰动 — Phase 2

`Scene.disturbances` 是一个 `Disturbance` 实例列表；每个 Disturbance 是世界坐标系下的一片区域，对穿越其上的车轮局部环境（μ / Fz / 地面 z）做修正。
Disturbance 通过项目 YAML 配置，sim4wis 启动或加载项目时实例化。

## 类型

### `ice_patch`（冰面 / 湿滑路面）

矩形区域 + 单一 μ 修正系数。

```yaml
- type: ice_patch
  id: ice_1
  x: 60.0            # 世界坐标 X [m]
  y: 0.0             # 世界坐标 Y [m]
  width: 8.0         # 横向（垂直 heading）长度 [m]
  length: 12.0       # 纵向（沿 heading）长度 [m]
  mu_modifier: 0.15  # 与 base_mu 相乘
  heading: 0.0       # 旋转 [rad]
```

### `split_mu`（对开路面）

矩形区域；沿矩形局部 Y 轴对半分，左右各一个 μ。

```yaml
- type: split_mu
  id: split_main
  x: 25.0
  y: 0.0
  width: 6.0
  length: 30.0
  mu_left: 0.9       # 局部 +Y 一侧的 μ
  mu_right: 0.3      # 局部 -Y 一侧的 μ
  heading: 0.0
```

heading=0 时，左侧（+Y）对应世界 +Y（"左转方向"）；旋转 heading 会同步旋转左右划分。

### `speed_bump`（减速带）

沿 length 方向呈 raised-cosine 高度分布；通过 `stiffness × height_profile` 转换为 Fz 脉冲。

```yaml
- type: speed_bump
  id: bump1
  x: 25.0
  y: 0.0
  width: 6.0         # 横向覆盖
  length: 0.5        # 带宽（典型 0.3-0.8 m）
  height: 0.05       # 峰高 [m]
  stiffness: 1.0e6   # Fz = stiffness × profile_z [N/m]
```

效果：车轮经过 0.5m 宽 / 5cm 高的带子时 Fz 短暂跳升 ~3-4 倍静载，模拟悬架压缩。

### `slope`（斜坡）

矩形区域；坡度由 `angle`（弧度）决定，方向由 `heading` 决定。
动力学模型读取前后轴的 ground_z 差异，得到纵向坡度角，按 `-mg·sin(α)` 加到车身纵向力上。

```yaml
- type: slope
  id: ramp_up
  x: 60.0
  y: 0.0
  width: 8.0
  length: 15.0
  angle: 0.0873      # 5° (≈ 8.7% grade)
  heading: 0.0       # 0 = 上坡方向沿 +X
```

5° 上坡使车辆减速 ~5%。

---

## 与模型的交互

- **KinematicModel**（Phase 1）：完全忽略 disturbances（其工作目标是几何分析）
- **SimplifiedDynamicModel**（Phase 2）：
  - 每个仿真步对每个车轮调用 `scene.wheel_env(wheel_world_pos)`
  - μ 用于轮胎力的摩擦圆裁断
  - fz_offset 加到车轮垂直载荷
  - ground_z 用于估计纵向坡度

## 可视化

前端 `Canvas2D` 自动渲染所有扰动：
- `ice_patch` — 蓝/红渐变（按 μ 着色）
- `split_mu` — 上下半分色块（按各自 μ）
- `speed_bump` — 黄色斜条纹
- `slope` — 紫色虚线框 + 箭头标方向 + 坡度数字

切换不同项目能直接看到不同扰动布局。

## 程序化构造

如果不想写 YAML，也可以在测试 / 脚本中直接构造：
```python
from sim4wis.environment.disturbance import Scene, SplitMu, SpeedBump

scene = Scene(
    base_mu=0.9,
    disturbances=[
        SpeedBump(id="b1", x=10.0, y=0.0, width=5.0, length=0.5, height=0.04),
        SplitMu(id="s1", x=30.0, y=0.0, width=6.0, length=25.0,
                mu_left=0.9, mu_right=0.3),
    ],
)
sim.set_scene(scene)
```
