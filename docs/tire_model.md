# 轮胎内核与验证边界

2026-09-08 核对当前代码。`vehicle/tire.py` 同时提供 `LinearTireModel` 和 `PacejkaTireModel`；`VehicleParams` 默认选 **linear**。负载扫描使用简化 Pacejka 路径，所以扫描曲线与默认时域的轮胎类型不完全相同，比较前应统一模型。

## 坐标与滑移

本项目 X 前、Y 左、Z 上，轮序 FL / FR / RL / RR。轮坐标系绕车体 Z 轴转 δ：

```
vxi = vx − r yi; vyi = vy + r xi
u = cosδ vxi + sinδ vyi; v = −sinδ vxi + cosδ vyi
α = atan2(v, max(|u|, 0.5))
κ = (Rω − u) / max(|u|, 0.5)
```

α 正表示轮心向轮胎左侧滑动，Fy 反向抵抗；κ 正表示正向纵滑速度，在倒车时不能简单标为“驱动”。轮胎力返回在轮坐标系，Mz 正表示绕上方 Z 轴逆时针。与其他软件比较时必须转换各自轴向与正负约定。

## 力律

线性模型的未饱和力 `Fx=Cκκ`、`Fy=−Cαα`。简化 Magic Formula 用各轴 B/C/D/E 生成纯滑移力，令 `D=μFz`、`B=C_stiffness/(C_shape D)` 保证原点刚度。两种模型都对合力进行径向投影，使 `Fx²+Fy²≤(μFz)²`。这里是等向摩擦圆，不是完整 MF 组合滑移权重模型，也不保证饱和边界导数光滑。

`μ=0` 或 `Fz=0` 时严格返回零 Fx/Fy/Mz。无接触情况下不能靠数值下限生成虚假摩擦。

气胎自回正 `Mz=−Fy*t_p`，Pacejka 中有效拖距随侧偏变化。主销保持力矩再取 `−Mz`，只计一次；scrub 横向偏置不能加到 Fy 的纵向机械拖距上。

## 当前默认与参数来源

| 参数 | VehicleParams 默认 | 说明 |
|---|---:|---|
| tire_c_alpha | 120000 N/rad | 每轮名义小信号侧偏刚度 |
| 前/后轴刚度比例 | 0.80 / 1.20 | RWS 控制参考与时域需一致 |
| tire_c_kappa | 100000 N | 纵滑刚度 |
| tire_load_sensitivity_exp | 0.8 | 刚度随垂载的经验指数 |
| tire_relax_length | 0 m | 默认关闭侧偏松弛 |
| tire_cx / tire_cy | 1.65 / 1.30 | 简化 Magic Formula 形状参数 |
| tire_ex / tire_ey | −0.5 / −1.0 | 简化 Magic Formula 曲率参数 |

这些是工程估计，不是某一款量产轮胎的试验辨识结果。`make_tire(params)` 才代表车辆的完整当前配置；直接实例化 TireModel 的类默认值可能不同。

## 积分与适用范围

轮速求解实际轮胎力的后向欧拉方程：

```
Iw (ω_new − ω_old) = dt [T_drive + T_brake − R Fx(ω_new)]
```

采用摩擦上界构造根区间，使用带区间保护的 Newton 迭代。不能用线性 `Cκκ` 的近似隐式项替代实际 Pacejka 力，否则非线性平衡点会漂移。车身与轮速分步积分，整体仍存在一阶分裂误差，末状态轮胎力也不等于该步车身的平均受力。

0.5 m/s 的低速正则化用于避免分母奇异，不是静摩擦或接地印迹扭转模型。停车负载中的经验摩擦项与时域滚动轮胎属于不同近似，原地转向、反向过零、轮胎离地、高频松弛和极限饱和需要专门验证。温度、压力、磨损、有限接地印迹和完整 3D 接触尚未建模。

Project Chrono 的[轮胎模型说明](https://api.chrono.projectchrono.org/wheeled_tire.html)区分 handling 与刚性/可变形接触模型；这一分类也解释了为何当前低阶 handling 模型不能承担所有停车与复杂地形预测。

内部验证脚本：[review_physics.py](../scripts/review_physics.py)；评审结论：[ASTRA 科学模型评审](reports/astra_review_2026-09-08.md)。
