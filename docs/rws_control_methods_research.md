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
中间某临界车速处过零。

**过零点不是通用常数，也不该抄专利数值**——它由整车参数解析决定。取线性单轨模型的零侧偏比
（实现见 `backend/src/sim4wis/controller/rws_common.py::zero_sideslip_ratio`）：

```
k(u) = (−b + a·m·u²/(Cr·L)) / (a + b·m·u²/(Cf·L))        过零点  u₀² = b·Cr·L/(a·m)
```

按本项目默认标定（LS9：L=3.160 m、a=1.550 m、m=2900 kg、轴侧偏刚度 240 kN/rad）得
**u₀ ≈ 16.5 m/s ≈ 59 km/h**；k(0)=−1.04、k(90 km/h)=+0.39、k(200 km/h)=+0.81。换车型、换轮胎，过零点跟着变。

**来源**（专利原文已于 2026-07-26 逐条核对，核对记录见 §8）

- **US5224042A** *Four wheel steering system with speed-dependent phase reversal*（GM，1991 申请／1993 授权）——
  FIG. 3 的稳态查表为**低速反相、高速同相**：原文 "At low vehicle speeds V1, V2, the rear steering is primarily
  out-of-phase with the front wheel steering. However, at increasing vehicle speeds V3, V4, the rear steering is
  primarily in-phase"；权利要求 1 亦载明高速时稳态后轮转向与前轮同相。
  ⚠️ 两点提醒：① 该专利的主发明点是**瞬态项**，整体归属方法 2，此处只引其稳态部分作为相位方向的佐证；
  ② 原文**未给出任何相位过零车速**——全文唯一的速度阈值 40 mph（≈64 km/h）管的是求导采样点数 n 的拐点，与相位切换无关。

**可实现性：★★★★★（立即可做）**
纯前馈、只需 `vx`。把 `rear_ratio` 常数替换为一条 `k(vx)` 曲线（折线或 `tanh`/分段），临界车速、低/高速增益做成 `mode_params`。
天然适合放进**策略设计器**做成可视曲线，与「开环激励 + 评分」闭环对比。

---

## 1b. 变体：按横向加速度调度相位（scheduling on a_y）

**控制律**

调度自变量从车速换成横向加速度，按双阈值切换相位：

```
|a_y| ∈ (a_y1, a_y2]  →  δ_rear 反相   （抵消不足转向，保机动）
|a_y| >  a_y2         →  δ_rear 同相   （抑制后轮侧偏角过大，保稳定）
```

与方法 1 的区别：车速调度是**开环按工况**切换，a_y 调度是**按实际轮胎工作点**切换——同一车速下大转角/低附着会更早进入同相保护区。
两者可叠加（k(vx) 定基线，a_y 做限幅/修正）。

**来源**
- US5627754 *Method for controlling a front and rear wheel steering vehicle*（Honda，1995 申请／1997 授权）：
  a_y 介于第一、第二阈值之间时反相以抵消不足转向；超过第二阈值时转同相，防止后轮产生过大侧偏角。

**可实现性：★★★★☆（可做，需 a_y）**
`a_y` 在动力学模型下可由 `vy` 微分 + `vx·yaw_rate` 得到，或直接取模型输出的侧向加速度；运动学模型下用 `vx·yaw_rate` 近似。
两个阈值入 `mode_params`。价值在于它是**唯一以轮胎饱和为切换依据**的量产级策略，和现有轮胎模型/载荷页联动演示效果好。

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

**来源**（专利原文已于 2026-07-26 逐条核对，核对记录见 §8）

- **US5224042A**（GM，1991／1993）——**本方法的主引专利**。摘要即 `θr = θss + θtr`：
  稳态项 `θss(vx, δf)` 查三维表（低速反相/高速同相，见 §1），瞬态项 `θtr = K(vx)·dδf/dt`。
  其独有发明点是**求导所用采样点数 n 随车速递减**（拐点 40 mph ≈ 64 km/h）——高速取样少 → 相位滞后小、后轮响应快；
  低速取样多 → 指令平滑、不过度追随方向盘。
  原文明确 `θr = θss − θtr` 取**减号**："θtr is subtracted because the rear wheel steady state steering command
  θss will be in phase with the front wheels over the range of speeds where it is desired that the rear wheels steer
  temporarily out of phase"——即"瞬时反相"是在同相稳态基线上**减出来**的，回轮时导数变号又自动帮助回正。
- US5341294 *Four-wheel steering system for vehicle*（Mazda，1991／1994）——同一思想的**机械式**实现：
  转角比变换机构使中速区间打方向的瞬间转角比"先负后正"（先反相、随即转同相）。
- US4842089 *…with closed-loop feedback and open-loop feedforward*（GM，1989）——US5224042 引用的在先技术，同一 GM 谱系。［摘要未核原文］
- 术语出处：SAE 891978 *Development of "Super HICAS", a New Rear Wheel Steering System with Phase-reversal Control*
  (Eguchi et al., 1989)，US5224042 的 Other References 之一——"phase reversal（相位反转）"一词即源出于此，指的**始终是瞬态那一下**。
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

## 8. 来源核对记录（2026-07-26）

建库过程中发现本文 §1 的来源描述与正文自相矛盾（知识库 OPEN-001），遂逐条打开专利原文核对。
**结论：§1／§2 原有的四条专利描述全部有误，已在上文订正。** 核对经由 freepatentsonline 全文
（Google Patents 反爬拦截）。

| 公开号 | 权利人／年份 | 原文实际内容 | 订正前本文的写法 | 性质 |
|---|---|---|---|---|
| US5224042A | GM，1991 申请／1993 授权 | 稳态 θss(vx,δf) **低速反相、高速同相**；瞬态 θtr=K(vx)·dδf/dt，采样点数随速递减（拐点 40 mph） | 「高速反相、低速同相，按 ≈56 km/h 切换」 | **相位方向写反**；56 km/h 原文无此数；且归错方法（应属 §2） |
| US4953650 | Mazda，1989／1990 | 后轮转向机构的**锁止／释放装置**（目标角恒定时锁住机构，特定条件释放） | 「后轮转角比按车速连续变化，高速趋同相」 | **与控制律完全无关**，已从来源中移除 |
| US5341294 | Mazda，1991／1994 | **机械式**转角比变换机构，中速区间打方向瞬间比值「先负后正」 | 同上 | 属瞬态反相（§2 的机械实现），非随速连续调度 |
| US5627754 | Honda，1995／1997 | 按**横向加速度双阈值**切换相位 | 「稳态项 f(车速,前轮角)，瞬态项 f(前轮角变化率,车速)」 | 该描述实为 US5224042A 的摘要，两条串位；本专利另立为 §1b |

**影响面复查（结论：无外溢）**
- 代码正确：`rear_wheel_steer.py` + `rws_common.zero_sideslip_ratio` 实现的即低速反相／高速同相，
  默认过零 ≈59 km/h 由整车参数解析导出，与 US5224042A FIG. 3 方向一致。
- 前端提示（`ControlPanel.tsx`、`help.tsx`、策略设计器图例）、CHANGELOG、外部讲义正文与配图，
  方向均为「低速反相、高速同相」，未被污染。
- 讲义配图中的「示例过零 ≈56 km/h」标注为示例值且已注明「随车型/标定变」，可保留；
  但**不得**再表述为「专利里的典型值」。

**教训**：本文初版的专利来源行是检索摘要转写，未逐条回原文，出现了方向反转、描述串位和数值杜撰三类错误。
后续新增来源行一律标注是否已核原文（本文中未核者已用「［…未核原文］」显式标出）。

---

## 来源

> 标 ✅ 者已于 2026-07-26 核对专利全文（见 §8）；未标者仅有检索摘要，引用前须回原文。
> Google Patents 会对自动访问返回 503／反爬页，改用 freepatentsonline.com/<号码>.html 或下方 USPTO PDF 直链。

- ✅ [US5224042A — Speed-dependent phase reversal](https://patents.google.com/patent/US5224042A/en)（GM 1991／1993；§2 主引，稳态部分佐证 §1）
- ~~US4953650 — Rear wheel steering control system~~（Mazda 1989／1990：**锁止/释放机构，与控制律无关**，已从 §1 剔除）
- ✅ [US5341294 — Four-wheel steering system for vehicle](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5341294)（Mazda 1991／1994；§2 机械式实现）
- [SAE 891978 — Development of "Super HICAS" with Phase-reversal Control](https://www.sae.org/publications/technical-papers/content/891978/)（Eguchi et al. 1989；"phase reversal" 术语出处）
- [US4842089 — Closed-loop feedback + open-loop feedforward](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4842089)
- ✅ [US5627754 — Method for controlling a front and rear wheel steering vehicle](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5627754)（Honda 1995／1997；§1b 按 a_y 调度）
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
