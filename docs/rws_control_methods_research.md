# 后轮转向（RWS）控制方法研究报告

> 状态：**研究报告，待审批后实现**（v0.7.0 ② 项）。
> 目的：把全网论文 / 专利里主流的后轮转向控制律梳理成一组**可在本仿真器里测试、演示、对比**的策略，
> 修正当前「任意车速都反相」的不真实行为。

---

## 0. 当前实现的问题

现有 `backend/src/sim4wis/controller/rear_steer.py` 用一个**与车速无关的静态比例** `rear_ratio`
（默认 −0.5，即恒定反相）：

```
δ_rear = rear_ratio · δ_front      (rear_ratio 为常数)
```

这只对应真实 RWS 的一个工作点。工程实践中后轮转向应**随车速、随转向输入的快慢、甚至随横摆目标闭环**地
改变同相/反相与幅值。下面把主流方法分 5 类，每类给出控制律、来源、在本仿真器里的可实现性评估。

本仿真器可用的反馈量（`VehicleState.velocity`）：纵向车速 `vx`、侧向车速 `vy`、横摆角速度 `yaw_rate`；
驾驶输入 `driver.steering / throttle / mode_params`。这决定了哪些方法**当下即可实现**、哪些需要先补状态估计。

---

## 1. 静态车速调度比例（speed-scheduled ratio，量产主流）

**控制律**

```
δ_rear = k(vx) · δ_front
```

`k(vx)` 是车速的单调函数：低速为负（反相，缩小转弯半径、提升机动性），高速为正（同相，提升高速稳定性），
中间某临界车速处过零。专利里典型的过零点约 **56 km/h**（≈15.6 m/s）。

**来源**
- US5224042A *Four wheel steering system with speed-dependent phase reversal*：高速反相、低速同相，按 ≈56 km/h 切换。
- US4953650 / US5341294：后轮转角比按车速连续变化，高速趋同相以保证稳定性。

**可实现性：★★★★★（立即可做）**
纯前馈、只需 `vx`。把 `rear_ratio` 常数替换为一条 `k(vx)` 曲线（折线或 `tanh`/分段），临界车速、低/高速增益做成 `mode_params`。
天然适合放进**策略设计器**做成可视曲线，与「开环激励 + 评分」闭环对比。

---

## 2. 稳态项 + 瞬态项（前馈动态，含「转得快就先反相」）

**控制律**

```
δ_rear = δ_rear,ss(vx, δ_front)  +  δ_rear,tr(vx, dδ_front/dt)
```

- 稳态项 `δ_rear,ss` ＝车速 × 前轮角的函数（≈ 方法 1）。
- 瞬态项 `δ_rear,tr` ＝前轮角**变化率**的函数：方向盘**快打**时后轮先**反相**（迅速建立横摆、响应更快），
  稳态或慢打时回到同相。等价于给后轮一个相位超前/带通环节。

这正对应用户提到的「根据前轮转角频率输入动态调整后轮转角」。

**来源**
- US4842089 *Four wheel steering system with closed-loop feedback and open-loop feedforward*：稳态+瞬态分量合成。
- US5627754 *Method for controlling a front and rear wheel steering vehicle*：稳态项 f(车速, 前轮角)，瞬态项 f(前轮角变化率, 车速)。
- 频域视角：「方向盘快打时后轮初始反相、慢打/稳态时同相；相位超前随频率降低或车速升高而减小。」

**可实现性：★★★★☆（可做，需数值微分）**
需要 `dδ_front/dt`——对驾驶 `steering` 做一阶差分或在控制器里存上一帧值即可（仿真步长已知）。
瞬态项用一阶高通/超前环节，参数（增益、时间常数）入 `mode_params`。无需新状态估计。

---

## 3. 横摆角速度反馈（yaw-rate feedback，闭环）

**控制律**

```
δ_rear = g1 · δ_front  +  g2 · yaw_rate
```

`g1` 前馈、`g2` 对横摆角速度反馈；增益常按「零质心侧偏角」假设推导。通过让后轮在反相/同相间按
方向盘角与横摆角速度的配比变化，闭环保持稳定、行进方向被稳定修正。

**来源**
- US9469339 *Apparatus for controlling rear wheel steering using driving behavior signal feedback*。
- US5964819 *Vehicle yawing behavior control apparatus*。
- 综述式表述：`δ_rear = (gain1×δ_front) + (gain2×yaw_rate)`，增益按零侧偏推导。

**可实现性：★★★★★（立即可做）**
`yaw_rate` 已在状态里。这是第一个真正「闭环」策略，能直观演示对扰动/激励的抑制；
配合「评分」面板的横摆峰值/侧偏角指标对比前馈方案，演示价值高。

---

## 4. 模型跟踪 / 零侧偏解耦（model-following，前馈+反馈，进阶）

**控制律（结构）**

前后轮同时控制，跟踪一个线性参考模型的**期望横摆角速度**与**期望侧偏角（常取 β=0）**：

```
[δ_front; δ_rear] = FeedForward(参考模型, vx)  +  FeedBack(yaw_rate误差, β误差)
```

反馈常用最优控制（LQR）或 PI；目标是让侧向速度与横摆动态**渐近解耦**——零侧偏时侧向加速度与横摆相位差减小、舒适性更好。

**来源**
- *Optimal Model Following Control of Four-wheel Active Steering Vehicle*（前馈+LQR 反馈跟踪期望 yaw rate 与 β）。
- *Asymptotic sideslip angle and yaw rate decoupling control in four-wheel steering vehicles*（Veh. Sys. Dyn.）：PI 主动前/后转向 + β 前馈，渐近解耦。
- US5402341 *…four wheel steering control utilizing tire characteristics*（零侧偏前馈传函）。

**可实现性：★★★☆☆（可做，但需侧偏角 β）**
β = atan2(vy, vx) 可由现有 `vy/vx` 直接算（动力学模型下有意义；运动学模型 vy 由几何给出）。
参考模型 = 单轨（bicycle）模型，参数已有（轴距、质量、侧偏刚度）。属于「能做、但要先搭参考模型 + 增益整定」的中等工作量。

---

## 5. 现代鲁棒 / 预测控制（H∞ / 滑模 / MPC，研究级）

**思路**
- **H∞**：针对高速瞬态/不确定性优化，改善高速稳定与安全。
- **滑模（SMC）**：直接跟踪 yaw rate 与 β 的参考，强鲁棒。
- **MPC**：显式约束（转角/速率限幅、轮胎力饱和、载荷转移、轮胎松弛），运动性能最佳但算力最高。

**来源**
- *Active Rear Wheel Steering Control Strategy Research Based on H∞*。
- *Yaw Rate and Sideslip Tracking for 4-Wheel Steering Cars Using Sliding Mode Control*。
- *Design and Implementation of a MPC-based Rear-Wheel Steering*（Politecnico di Milano）。
- *Disturbance Observer Based Control for Four Wheel Steering Vehicles With Model Reference*（IEEE/CAA JAS）。

**可实现性：★★☆☆☆（演示可选，工作量大）**
SMC 相对轻量，可作为「鲁棒闭环」演示；H∞/MPC 需要离线综合或求解器，建议**仅做 1 个**作为「现代方法」代表，或留作 v0.8+。

---

## 6. 实现映射（落到本仿真器的方式）

每种方法 = 一个新的 `ControllerStrategy`（仿照 `rear_steer.py`）：
1. 由 `driver.steering` 得 `δ_front = steer_limit · steering`；
2. 按所选控制律算 `δ_rear`（用到 `vx` / `yaw_rate` / `vy` / `dδ_front/dt`）；
3. 由前后轴垂线交点定 ICR → 调用 `compute_commands(...)` 得四轮角+轮速（现有基建已支持）；
4. 所有可调参数走 `driver.mode_params`（临界车速、各增益、时间常数），便于在界面整定；
5. 在 `registry.py` 注册，自动出现在「控制策略」面板。

演示与验证：复用本版的**开环激励**（阶跃/扫频/双移线）+ **策略评分**（横摆峰值、侧偏角、瞬心偏差、能耗）
+ **A/B 对比**，把每种 RWS 与「仅前轮 / 现有恒定反相」量化对比。方法 1 还可在**策略设计器**里直接画 `k(vx)` 曲线。

---

## 7. 建议实现集（待你审批）

| 优先 | 方法 | 工作量 | 价值 | 备注 |
|---|---|---|---|---|
| P0 | ① 车速调度比例 k(vx) | 小 | 高 | 直接修正「恒定反相」；可视曲线 |
| P0 | ③ 横摆角速度反馈 | 小 | 高 | 首个闭环，演示抗扰 |
| P1 | ② 稳态+瞬态前馈 | 中 | 高 | 体现「转得快先反相」的频率特性 |
| P1 | ④ 模型跟踪 / 零侧偏 | 中 | 中高 | 需 bicycle 参考模型+整定 |
| P2 | ⑤ SMC（鲁棒代表） | 中 | 中 | H∞/MPC 留 v0.8+ |

**推荐 v0.7.0 落地：① + ③ + ②**（三者都只依赖现有状态量，能立刻跑激励/评分对比）；④ 视精力，⑤ 顺延。

### 待确认问题
1. 实现范围按推荐的 **①+③+②** 吗？还是要把 ④ 也纳入 v0.7.0？
2. 参考车辆标定沿用默认（智己 LS9）吗？方法 ①/④ 的临界车速、增益是否要按它整定一组默认值？
3. 是否需要把方法 ① 的 `k(vx)` 曲线也接进**策略设计器**（曲线编辑器），让用户可视化调度曲线？

---

## 来源

- [US5224042A — Speed-dependent phase reversal](https://patents.google.com/patent/US5224042A/en)
- [US4953650 — Rear wheel steering control system](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4953650)
- [US5341294 — Four-wheel steering system for vehicle](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5341294)
- [US4842089 — Closed-loop feedback + open-loop feedforward](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4842089)
- [US5627754 — Method for controlling a front and rear wheel steering vehicle](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5627754)
- [US9469339 — RWS using driving behavior signal feedback](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9469339)
- [US5964819 — Vehicle yawing behavior control apparatus](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5964819)
- [US5402341 — 4WS control utilizing tire characteristics](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5402341)
- [Optimal Model Following Control of Four-wheel Active Steering Vehicle](https://www.researchgate.net/publication/316280638_Optimal_Model_Following_Control_of_Four-wheel_Active_Steering_Vehicle)
- [Asymptotic sideslip & yaw rate decoupling in 4WS (Veh. Sys. Dyn.)](https://www.tandfonline.com/doi/full/10.1080/00423110903248686)
- [Yaw Rate & Sideslip Tracking for 4WS using Sliding Mode Control](https://www.researchgate.net/publication/224381653_Yaw_Rate_and_Sideslip_Tracking_For_4-Wheel_Steering_Cars_Using_Sliding_Mode_Control)
- [Active Rear Wheel Steering Control Based on H∞](https://www.ingentaconnect.com/content/asp/jctn/2016/00000013/00000003/art00074)
- [MPC-based Rear-Wheel Steering (PoliMi)](https://re.public.polimi.it/retrieve/e0c31c12-a217-4599-e053-1705fe0aef77/RWS_MPC_CCTA.pdf)
- [Disturbance-Observer Based Control for 4WS (IEEE/CAA JAS)](https://www.ieee-jas.net/article/doi/10.1109/JAS.2016.7510220)
- [Rear-Wheel Steering Control for Steady-State & Transient Handling (ResearchGate)](https://www.researchgate.net/publication/343489563_Rear-Wheel_Steering_Control_for_Enhanced_Steady-State_and_Transient_Vehicle_Handling_Characteristics)
