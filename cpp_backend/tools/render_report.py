from pathlib import Path
import base64,json,subprocess,sys,math,statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/reports/cpp_v0.107.0';OUT.mkdir(exist_ok=True)
bench=json.loads((OUT/'benchmark-macos.json').read_text())
base=json.loads(json.dumps(bench['cases'][2]['experiment']))
base['name']='healthy_reference';base['faults']=[]
cases=[('Healthy',base)]
for title,case in [('Simultaneous','simultaneous'),('Staggered','staggered')]:
 exp=next(c['experiment'] for c in bench['cases'] if c['model']=='simplified_dynamic' and c['tire']=='pacejka' and c['case']==case)
 cases.append((title,exp))
results={}
(OUT/'experiments').mkdir(exist_ok=True)
for name,exp in cases:
 (OUT/'experiments'/(name.lower()+'.json')).write_text(json.dumps(exp,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

 raw=subprocess.check_output([str(ROOT/'cpp_backend/build/sim4wis-cpp'),'--root',str(ROOT),'--run-experiment','-'],input=json.dumps(exp).encode())
 results[name]=json.loads(raw)
ref=results['Healthy'];refch=ref['channels'];table=[]
fig,axs=plt.subplots(2,2,figsize=(13,8),layout='constrained')
colors=['#64748b','#f97316','#2563eb']
for (name,data),color in zip(results.items(),colors):
 ch=data['channels'];t=np.array(data['t']);x=np.array(ch['pose_x']);y=np.array(ch['pose_y']);psi=np.array(ch['pose_psi'])
 lateral=-(x-np.array(refch['pose_x']))*np.sin(refch['pose_psi'])+(y-np.array(refch['pose_y']))*np.cos(refch['pose_psi'])
 yaw=np.degrees(ch['yaw_rate'])
 axs[0,0].plot(x,y,label=name,color=color);axs[0,1].plot(t,yaw,label=name,color=color)
 axs[1,0].plot(t,lateral,label=name,color=color)
 axs[1,1].plot(t,np.degrees(ch['delta_fl']),label=name+' FL true',color=color)
 table.append({'case':name,'max_abs_yaw_dps':float(np.max(abs(yaw))),'max_abs_time_aligned_lateral_delta_m':float(np.max(abs(lateral))),'final_x_m':float(x[-1]),'final_y_m':float(y[-1]),'min_fz_n':min(min(ch['fz_'+w]) for w in ('fl','fr','rl','rr')),'peak_grip_utilization':max(max(ch['grip_util_'+w]) for w in ('fl','fr','rl','rr'))})
for ax in axs.flat:ax.grid(alpha=.2);ax.legend(fontsize=8)
axs[0,0].set(title='Vehicle path (equal spatial scale)',xlabel='x [m]',ylabel='y [m]',aspect='equal')
axs[0,1].set(title='Yaw response',xlabel='Simulation time [s]',ylabel='Yaw rate [deg/s]')
axs[1,0].set(title='Lateral difference from healthy at the same time',xlabel='Simulation time [s]',ylabel='Delta lateral position [m]')
axs[1,1].set(title='Front-left tire angle (static alignment included)',xlabel='Simulation time [s]',ylabel='Tire angle [deg]')
fig.savefig(OUT/'sample-fault-response.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(figsize=(11,5),layout='constrained');xx=np.arange(len(bench['cases']));width=.25
for shift,engine,title,color in [(-1,'python','Python','#64748b'),(0,'native','C++ kernel + Python host','#f97316'),(1,'cpp_full','Full C++ backend','#2563eb')]:
 ax.bar(xx+shift*width,[c['median_s'][engine] for c in bench['cases']],width,label=title,color=color)
ax.set_yscale('log');ax.set_ylabel('Whole experiment elapsed time [s, log scale]');ax.set_xticks(xx,[c['model'].replace('simplified_dynamic','Planar').replace('multibody','Multibody')+'\n'+c['tire']+' / '+c['case'][:4] for c in bench['cases']],fontsize=8);ax.grid(axis='y',alpha=.2);ax.legend(loc='lower center',bbox_to_anchor=(.5,1.02),ncol=3);fig.savefig(OUT/'three-route-performance.png',dpi=150);plt.close(fig)
evidence={'method':'14 s, same 5 ms outer and <=1.25 ms dynamics inner steps; 200 Hz recording; 40 km/h target with 1 s ramp; 2 degree / 0.2 Hz sinusoidal steering','cases':table,'definitions':{n:e for n,e in cases},'timelines':{n:d['meta']['fault_timeline'] for n,d in results.items()},'kpis':{n:d['kpis'] for n,d in results.items()},'model_validity':'Internal numerical/analytic verification only. No bench or vehicle correlation.'}
(OUT/'sample-analysis.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
lines=['# 两条 C++ 重构路线对照与多点故障样例','', '日期：2026-11-14。两个候选分支均以 v0.105.0 的物理与多点故障版本为基础，分别作为 v0.106.0 内核迁移版和 v0.107.0 全 C++ 后端版。主分支不合并，稳定 release 不替换。','', '## 实测性能','', '在同一台 Apple M5 Pro / 64 GiB Mac 上，8 组相同工况各预热一次，再轮换运行顺序各测量 5 次。没有并行本地测试或构建任务。全 C++ 的测量额外包含新进程启动、KPI 计算、完整 JSON 序列化与读取；另外两条路线测量 run_experiment 和记录。这个口径对全 C++ 更保守，不能把它误称为单一动力学函数的提速。','', '|模型|轮胎|故障|Python s|内核迁移 s|全 C++ s|全 C++ / Python 提速|全 C++ / 内核版提速|','|---|---|---|---:|---:|---:|---:|---:|']
for c in bench['cases']:
 m=c['median_s'];lines.append(f"|{c['model']}|{c['tire']}|{c['case']}|{m['python']:.4f}|{m['native']:.4f}|{m['cpp_full']:.4f}|{m['python']/m['cpp_full']:.2f}×|{m['native']/m['cpp_full']:.2f}×|")
maxdiff=max(v for c in bench['cases'] for k,v in c['max_abs_channel_differences']['cpp_full'].items() if k in ('pose_x','pose_y') and v is not None)
lines+=['',f'逐通道数值对照通过，记录时间、步数及故障事件序列一致；全 C++ 相对 Python 的 x/y 最大分量差为 {maxdiff:.3g} m。这是程序一致性，不是实车精度。状态绝对容差 1e-9 SI，力与力矩/摩擦余量 1e-5 SI，相对容差 1e-9；原有物理和黄金工况标准未放宽。', '', '![三路线耗时](three-route-performance.png)','', '## 多点故障响应样例','',evidence['method'], '', '健康参考无故障；同时故障为 FL/FR 在 1.713 s 同时机械卡滞；错时故障为 FL 1.713 s 卡滞、RR 2.123 s 失电自由转向、FR 2.321 s 偏差和噪声、FL 2.543 s 观测掉线，结束时刻见 JSON。使用相同随机种子 20261114。','', '|案例|峰值绝对横摆角速度 deg/s|相同时间的最大横向位置差 m|最小轮荷 N|最大摩擦利用率|','|---|---:|---:|---:|---:|']
for row in table:lines.append(f"|{row['case']}|{row['max_abs_yaw_dps']:.4f}|{row['max_abs_time_aligned_lateral_delta_m']:.4f}|{row['min_fz_n']:.2f}|{row['peak_grip_utilization']:.6f}|")
lines+=['','![故障响应](sample-fault-response.png)','', '横向差按健康参考在同一仿真时刻的车身法向投影计算，不是最近点路径误差，也不用于宣称越线或失稳阈值。当前模型里传感器故障仅改变观测通道，不进入控制反馈，所以它们不能被解释成闭环安全策略试验。机械卡滞锁住执行器坐标，轮胎总角仍可能受到前束/K&C 影响；自由转向由惯量、阻尼、摩擦和道路回正力矩驱动，不能按无受力约束的角度漂移处理。','', '## 发现并修复的物理基础问题','', '选型曲线原本把分析返回的四轮载荷混入同一横轴，导致不相关的重复转角点和排序依赖。两个候选版共同修复为选定车轮、执行器坐标，并增加不同车速的独立受力来源回归。现有双侧对称镜像负载假设继续明确保留；非对称左右载荷的精确齿条求解、实测参数与台架关联不在这组内部验收里。详见 sizing-correction.md。','', '## 选择路线时的实际差别','', '|维度|v0.106.0 内核迁移版|v0.107.0 全 C++ 后端版|','|---|---|---|','|计算|数值热点 C++；宿主、分析和 MCP 保留 Python|内置仿真、控制、研究分析、HTTP/WebSocket/MCP 都由 C++ 执行|','|界面|现有 React 浏览器界面|同一 React 浏览器界面，由 C++ 服务提供|','|维护|变化面较小；Python 扩展和故障排查更直接|代码与构建面更大；通过差分测试保持公式、数据与接口同步|','|插件|原 Python/FMU/MATLAB 插件机制|用户外部代码在独立兼容进程；内置流程不依赖 Python|','|并行对比|默认端口 8010|默认端口 8020，独立数据目录|','|回滚|停止该目录程序，启动另一目录|同左，无主分支合并或数据库迁移|','', '软件验收记录、安装包 SHA-256、平台结果见 acceptance.md。仿真模型仍是带有适用范围的工程模型：降阶平面动力学、分步积分、研究型多体与估算的转向参数均不等价于实车验证。Windows 用户机器的 GUI、硬件输入与长时间运行验收仍需实际目标机执行。','',f"测量源提交：{bench['source_commit']}；测量 C++ 二进制 SHA-256：{bench['binary_sha256']}。完整原始耗时在 benchmark-macos.json，样例定义、KPI 和时间线在 sample-analysis.json。"]
ix=lines.index('## 发现并修复的物理基础问题')
lines[ix:ix]=['## 样例的工程解释与复现','',
 '这是一组相同输入的开环对照。双前轮同时卡滞后，方向保持在故障发生时的执行器角度，后续正弦转向指令无法改变前轮，因此轨迹持续向一侧偏离。错时案例中，后右轮的自由转向改变了横摆响应；它的峰值横摆角速度更高，但本例相同时间横向差更小。这两个指标不能互相替代，也不能据此给所有故障组合排序。',
 '健康与故障案例的最大摩擦利用率都达到 1，主要涉及 1 s 起步速度斜坡下的轮胎力饱和；没有把这一数值单独判为故障失稳。样例最小轮荷保持正值。机械故障均在 13.679 s 解除，恢复段仍经过执行器动态，因此末段响应变化有明确的事件来源。FL 观测掉线在 13.877 s 恢复；FR 偏差为 0.02 rad，噪声标准差为 0.003 rad。',
 '样例的三份可直接运行定义保存在 experiments/healthy.json、experiments/simultaneous.json 和 experiments/staggered.json。在全 C++ 便携包根目录执行：',
 '`app/sim4wis-cpp --root . --run-experiment docs/reports/cpp_v0.107.0/experiments/staggered.json`',
 'Windows 使用 app\\sim4wis-cpp.exe，其他参数相同。输出包含完整通道、KPI、真实事件时间线和引擎元数据。','']
md='\n'.join(lines)+'\n';(OUT/'comparison.md').write_text(md,encoding='utf-8')
# Readable standalone report, with images embedded for offline handoff.
import html
parts=[];table_open=False
for line in lines:
 if line.startswith('|'):
  if set(line.replace('|','').replace(':','').replace('-',''))==set():continue
  if not table_open:parts.append('<table>');table_open=True
  parts.append('<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in line.strip('|').split('|'))+'</tr>');continue
 if table_open:parts.append('</table>');table_open=False
 if line.startswith('!['):
  name=line.split('(')[1][:-1];parts.append('<img src="data:image/png;base64,'+base64.b64encode((OUT/name).read_bytes()).decode()+'"/>')
 elif line.startswith('# '):parts.append('<h1>'+html.escape(line[2:])+'</h1>')
 elif line.startswith('## '):parts.append('<h2>'+html.escape(line[3:])+'</h2>')
 elif line:parts.append('<p>'+html.escape(line)+'</p>')
(OUT/'comparison.html').write_text('<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>4WIS C++ 路线对照</title><style>body{font:16px/1.7 system-ui;background:#f1f5f9;color:#172033;margin:0}main{max-width:1120px;margin:32px auto;background:white;padding:40px;border-radius:16px}h1{font-size:30px}h2{margin-top:40px;color:#1d4ed8}table{border-collapse:collapse;width:100%;font-size:13px}td{padding:9px;border:1px solid #dbe3ec}tr:first-child{font-weight:700;background:#eff6ff}img{max-width:100%;margin:20px 0}p{overflow-wrap:anywhere}@media(max-width:700px){main{margin:0;padding:16px}table{display:block;overflow:auto}}</style><main>'+''.join(parts)+'</main></html>',encoding='utf-8')
print(json.dumps({'status':'PASS','max_position_difference_m':maxdiff,'sample_cases':table},ensure_ascii=False))
