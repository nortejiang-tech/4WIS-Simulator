/** Shared scientific definitions for theory and load help. SI units, X forward/Y left. */
export const KINGPIN_FORMULA = String.raw`
$$\tau_{KP}=F_y t_m+\sigma F_x s-M_z+F_z\sin(\mathrm{KPI})s\sin\delta$$
<p>$t_m=R\tan\varepsilon$，$\sigma=+1$ 为左轮、$-1$ 为右轮。正力矩表示作动器需沿正转角方向出力。</p>`;
export const KINGPIN_DERIVATION = String.raw`
<p>以主销地面交点为参考，接地点的平面力臂为 $\mathbf r=(-t_m,\sigma s)$。
作动器保持力矩取轮胎外力矩的反号：$\tau=-(\mathbf r\times\mathbf F)_z-M_z$。
因此 scrub 只为纵向力提供平面力臂；不能把横向偏置 $s$ 加到 $F_y$ 的纵向拖距上。
气胎拖距已包含在 $M_z\approx-F_y t_p$ 中，只计一次。</p>
<p>末项 KPI 抬升是未经过实测标定的低阶近似。这里没有完整的空间主销轴投影、接地印迹扭转和悬架约束求解；
用于机理与量级分析，不能单独作为实际作动器选型验收依据。</p>`;
export const RACK_FORMULA = String.raw`
$$J=\frac{ds}{d\delta},\qquad |F_{rack}|=\frac{|\tau_{KP}|}{|J|},\qquad
T_{motor}=\frac{F_{rack}r_p}{i\eta_m}$$
<p>$s$ 为齿条行程，$J$ 由硬点约束隐式微分得到，满足虚功 $\tau\,d\delta=F_{rack}\,ds$。
界面的齿条力符号按各轮正转角方向统一；它不是左右轮共同物理坐标中的推拉符号。
电机效率 $\eta_m$ 只在电机输入端计入一次。</p>`;
export const LINKAGE_INDICATOR = String.raw`
<p>兼容字段 $\eta_{linkage}=|\sin\theta_{arm-tie}|\,|\cos\theta_{tie-rack}|\,\eta_m$
是机构几何状态的提示指标，<strong>不是能量效率，也不是力比</strong>。
齿条力使用真实行程导数 $ds/d\delta$；投影角改变力和位移的比例，本身不消耗能量。
指标接近零时需检查是哪一项退化。超行程或硬点不可达的负载是外推值，不能用于选型验收。</p>`;
export const STEADY_BODY = String.raw`
$$\sum F_{y,i}=mVr,\qquad \sum(x_i-e)F_{y,i}=0,\qquad e=L/2-a$$
$$\alpha_i\simeq\beta_O+rx_i/V-\delta_i,\qquad F_{y,i}=-C_{\alpha,i}\alpha_i$$
<p>在线性、小角、前进且准稳态的假设下解 $2\times2$ 方程组得到 $\beta_O,r$。
$x_i$ 以轴距中点 O 为基准，力矩必须绕质心 G 计算。不同轴荷和前后轴刚度会改变增益与稳定性，
高速增益并非普适单调关系。低速、轮胎饱和或临界速度附近应改看时域结果。</p>
<p>单轮台架模式固定车身速度方向；整车装载模式允许上述车身响应。
两者是不同边界条件，台架结果不是所有车辆运动下的严格负载上界，且都不等同于四轮自由转向平衡。</p>`;
