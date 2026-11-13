#!/usr/bin/env python3
"""Generate the synthetic multipoint example and step-size study, without network or vehicle I/O."""
from __future__ import annotations
import base64
import csv
import gzip
import hashlib
import html
import io
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend/src'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sim4wis import __version__
from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import run_experiment

OUT = ROOT / 'docs/reports/multipoint_v0.105.0'
OUT.mkdir(parents=True, exist_ok=True)

CASES = {
    'baseline': [],
    'simultaneous': [dict(id='fl-jam',fault_type='stuck_hold',wheel=0,t_start=6.123,t_end=10.123),
                     dict(id='fr-jam',fault_type='stuck_hold',wheel=1,t_start=6.123,t_end=10.123)],
    'staggered': [dict(id='fl-jam',fault_type='stuck_hold',wheel=0,t_start=6.123,t_end=10.123),
                  dict(id='rr-free',fault_type='free_caster',wheel=3,t_start=7.321,t_end=11.321),
                  dict(id='fr-bias',fault_type='sensor_bias',wheel=1,value=.02,t_start=8.543,t_end=12.123)],
    'four_corner': [dict(id='fl-jam',fault_type='stuck_hold',wheel=0,t_start=6.123,t_end=10.123),
                    dict(id='fr-limit',fault_type='limited',wheel=1,value=.015,t_start=6.123,t_end=10.123),
                    dict(id='rl-target',fault_type='stuck_value',wheel=2,value=-.03,t_start=7.321,t_end=11.321),
                    dict(id='rr-free',fault_type='free_caster',wheel=3,t_start=8.543,t_end=12.123),
                    dict(id='fr-bias',fault_type='sensor_bias',wheel=1,value=.02,t_start=8.543),
                    dict(id='rl-noise',fault_type='sensor_noise',wheel=2,value=.003,t_start=8.543),
                    dict(id='fl-dropout',fault_type='sensor_dropout',wheel=0,t_start=9.123,t_end=11.123)],
}
NAMES = {'baseline':'无故障基准','simultaneous':'双前轮同时卡滞','staggered':'前轮卡滞 → 后轮自由转向 → 观测偏差',
         'four_corner':'四轮混合故障 + 观测叠加'}

def config(case, model, dt=.005):
    return Experiment.model_validate(dict(name=f'multi_{case}',model_type=model,strategy='ideal_ackermann',
        fault_seed=20261113, faults=CASES[case], dt=dt,record_hz=200,
        maneuver={'steps':[{'name':'ramp','duration':4,'speed_kmh':40,'speed_ramp_s':3},
                           {'name':'sine','duration':10,'speed_kmh':40,
                            'steer':{'kind':'sine','amplitude':2,'unit':'front_deg','freq_hz':.2}}]}))

def metrics(run, baseline):
    c=run.channels; t=np.array(run.t)
    refx=np.interp(t,baseline.t,baseline.channels['pose_x']); refy=np.interp(t,baseline.t,baseline.channels['pose_y'])
    dev=np.hypot(np.array(c['pose_x'])-refx,np.array(c['pose_y'])-refy)
    grip=np.array([c[f'grip_util_{w}'] for w in ('fl','fr','rl','rr')])
    fz=np.array([c[f'fz_{w}'] for w in ('fl','fr','rl','rr')])
    return {'peak_yaw_deg_s':float(np.max(np.abs(np.rad2deg(c['yaw_rate'])))),
            'peak_ay_m_s2':float(np.max(np.abs(c['ay']))),
            'max_position_difference_m':float(np.max(dev)),
            'end_position_difference_m':float(dev[-1]),
            'peak_grip_util':float(grip.max()),'minimum_fz_n':float(fz.min()),
            'max_observation_error_deg':float(max(np.max(np.abs(np.rad2deg(np.array(c[f'delta_reported_{w}'])-c[f'delta_{w}']))) for w in ('fl','fr','rl','rr')))}

runs={}; rows=[]; files={}
for model in ('simplified_dynamic','multibody'):
    for case in CASES:
        exp=config(case,model); run=run_experiment(exp); runs[(model,case)]=run
        if case=='baseline': base=run
        m=metrics(run,base)
        assert m['minimum_fz_n']>=0 and m['peak_grip_util']<=1+1e-8
        raw=io.StringIO(); writer=csv.writer(raw); fields=['t',*run.channels]
        writer.writerow(fields)
        writer.writerows(zip(run.t,*run.channels.values()))
        data=gzip.compress(raw.getvalue().encode(),mtime=0)
        name=f'{model}_{case}.csv.gz'; (OUT/name).write_bytes(data)
        files[name]=hashlib.sha256(data).hexdigest()
        rows.append({'model':model,'case':case,'metrics':m,'events':run.meta['fault_timeline']['events'],
                     'experiment':exp.model_dump(mode='json'),'samples':len(run.t),'steps':run.n_steps})
        print(model,case,json.dumps(m),flush=True)

# Numerics acceptance is deliberately separate from scenario severity.
# Compare maximum planar position error and yaw trace RMS on the common interval;
# omit the sub-ms initial sample where the coarse grid has no sample to compare.
convergence=[]
for model in ('simplified_dynamic','multibody'):
    for case in ('simultaneous','four_corner'):
        series=[]
        for dt in (.005,.0025,.00125,.000625):
            r=runs[(model,case)] if dt==.005 else run_experiment(config(case,model,dt))
            series.append((dt,r))
        fine=series[-1][1]
        for dt,r in series[:-1]:
            ts=np.asarray(r.t); c=r.channels
            pos=np.hypot(np.asarray(c['pose_x'])-np.interp(ts,fine.t,fine.channels['pose_x']),
                         np.asarray(c['pose_y'])-np.interp(ts,fine.t,fine.channels['pose_y']))
            yaw=np.rad2deg(np.asarray(c['yaw_rate'])-np.interp(ts,fine.t,fine.channels['yaw_rate']))
            m={'model':model,'case':case,'dt_s':dt,'reference_dt_s':.000625,
               'max_position_difference_m':float(pos.max()),'yaw_rms_difference_deg_s':float(np.sqrt(np.mean(yaw*yaw)))}
            m['status']='PASS' if m['max_position_difference_m']<=.15 and m['yaw_rms_difference_deg_s']<=.1 else 'FAIL'
            convergence.append(m);print('convergence',json.dumps(m),flush=True)

fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained')
for case in CASES:
    r=runs[('simplified_dynamic',case)]; c=r.channels
    axes[0,0].plot(c['pose_x'],c['pose_y'],label=case)
    axes[0,1].plot(r.t,np.rad2deg(c['yaw_rate']),label=case)
r=runs[('simplified_dynamic','four_corner')]
for w in ('fl','fr','rl','rr'):
    axes[1,0].plot(r.t,np.rad2deg(r.channels[f'delta_{w}']),label=w.upper())
axes[1,1].plot(r.t,np.rad2deg(r.channels['delta_fl']),label='FL actual')
axes[1,1].plot(r.t,np.rad2deg(r.channels['delta_reported_fl']),label='FL reported',linestyle='--')
for ax in axes.flat: ax.grid(alpha=.25);ax.legend(fontsize=8)
axes[0,0].set(xlabel='x [m]',ylabel='y [m]',title='Trajectory / same-time positions, no path controller')
axes[0,1].set(xlabel='Simulation time [s]',ylabel='Yaw rate [deg/s]',title='Fault timing changes vehicle response')
axes[1,0].set(xlabel='Simulation time [s]',ylabel='Physical wheel angle [deg]',title='Four-corner mixed mechanical faults')
axes[1,1].set(xlabel='Simulation time [s]',ylabel='Wheel angle [deg]',title='Dropout holds the last observation only')
for ax in (axes[0,1],axes[1,0],axes[1,1]):
    for t in (6.123,7.321,8.543,9.123): ax.axvline(t,color='grey',alpha=.25,linestyle=':')
fig.savefig(OUT/'overview.png',dpi=150);plt.close(fig)
summary={'version':__version__,'date':'2026-11-13','kind':'synthetic internal verification',
    'scope':'reduced independent-corner mechanics; no vehicle/bench correlation',
    'scenario_metrics':rows,'convergence':convergence,'data_sha256':files,
    'numerical_criteria':{'max_position_difference_m':.15,'yaw_rms_difference_deg_s':.1},
    'limitations':['first-order partitioned integration; event/control grids differ',
                   'passive steering ignores steering-inertia reaction on chassis and 3D kingpin projection',
                   'sensor faults affect observation only','no shared-rack faults or high-frequency shimmy validation']}
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
lines=['# v'+__version__+' 多点故障样例与动力学复核','',
'日期：2026-11-13。全部数据来自本仓库新内核的合成仿真；模型为默认车辆参数、40 km/h 速度目标、2° / 0.2 Hz 开环正弦前轮输入、14 s 工况。实际速度由驱动/轮胎动力学产生。固定噪声种子 20261113。', '',
'本报告的轨迹差是相同仿真时刻相对无故障工况的位置差，不是路径跟踪误差，也不是安全阈值。没有启用多故障容错控制器。', '',
'## 配置与物理含义','',
'- 同时：FL/FR 在 6.123 s 卡住当时机构角，10.123 s 恢复。',
'- 错时：FL 6.123 s 卡滞；RR 7.321 s 失电自由转向；FR 8.543 s 出现 0.02 rad 观测偏差。',
'- 四轮混合：FL 卡滞、FR 限幅同时开始；RL 固定目标角和 RR 自由转向随后触发；另叠加偏差、噪声、掉线。详见 summary.json 的完整实验。',
'- 锁死不注入任意角度跳变；静态前束/K&C 是机构外的轮角贡献。自由转向使用 J=3 kg·m²、c=80 N·m·s/rad、τc=5 N·m、ηrev=0.6，受道路主销力矩驱动，具有摩擦静止区与非弹性止挡。',
'- 固定目标角/限幅故障保留伺服动态；传感器故障只改观测，不影响轮胎的真实角度。', '',
'## 实际结果','',
'|模型|工况|横摆峰值 °/s|横向加速度峰值 m/s²|最大位置差 m|末端位置差 m|最大摩擦利用率|最小 Fz N|',
'|---|---|---:|---:|---:|---:|---:|---:|']
for row in rows:
    m=row['metrics'];lines.append(f"|{row['model']}|{NAMES[row['case']]}|{m['peak_yaw_deg_s']:.3f}|{m['peak_ay_m_s2']:.3f}|{m['max_position_difference_m']:.3f}|{m['end_position_difference_m']:.3f}|{m['peak_grip_util']:.4f}|{m['minimum_fz_n']:.1f}|")
lines+=['','![仿真曲线](overview.png)','','## 步长敏感性','','本版共用执行链将动力学内部积分步限制为 1.25 ms；外层控制/记录节拍不变。修订前的 5 ms 最大位置差约 0.60 m，见 before_substeps.json。以下同一配置比较外层 5 / 2.5 / 1.25 ms 与 0.625 ms。此项预设工程门槛为最大位置差 ≤0.15 m、横摆差 RMS ≤0.1°/s，仅适用于本例，不等于车辆安全门槛。参考解也属于当前模型，并非真实值。','','|模型|工况|dt ms|最大位置差 m|横摆差 RMS °/s|状态|','|---|---|---:|---:|---:|---|']
for c in convergence:lines.append(f"|{c['model']}|{c['case']}|{c['dt_s']*1000:g}|{c['max_position_difference_m']:.5f}|{c['yaw_rms_difference_deg_s']:.5f}|{c['status']}|")
lines+=['','## 物理复核证据与解释','',
'独立测试使用已知外力验证两种动力学模型在非中心质心配置下的 Newton–Euler 力/矩平衡；验证支承载荷总和与未饱和俯仰/侧倾力矩、组合滑移摩擦圆和非线性轮速角冲量。故障回归覆盖 8 类故障 × 4 轮 × 2 种动力学模型，以及卡滞、自由转向耗散/静摩擦/止挡、同时/错时/结束事件、传感器叠加和跨宿主重放。完整命令和数量见发布验收记录。',
'', '车辆平面方程与坐标约定的参考是 [MathWorks Vehicle Body 3DOF](https://www.mathworks.com/help/vdynblks/ref/vehiclebody3dof.html)，轮胎与车轮转动受力参考 [Combined Slip Wheel 2DOF](https://www.mathworks.com/help/vdynblks/ref/combinedslipwheel2dof.html)。本轮使用这些一手资料核对方程类别，不声称与该软件已完成数值对标。',
'', '相同时刻不同车轮的故障会改变合力与偏航力矩；错时触发发生在已受前序故障影响的状态上，因此不能用单故障结果线性相加。具体严重程度依赖当时轮角、车速、摩擦、前束、机构惯量和控制器。本例观测故障本身不改变受力。',
'', '## 可信度边界与复现','',
'内部验证与本例数值门槛按各自证据判定；台架/实车与独立外部模型相关性为 PENDING。当前降阶模型不支持共享齿条上相互独立的轮端机械卡滞；忽略转向惯量对车体的反作用、高频摆振、完整三维主销轴投影。准静态模型的离地载荷饱和不是翻滚预测。没有这些参数与外部证据，不能保证任意实车故障的绝对精度。',
'', '`backend/.venv/bin/python scripts/analyze_multipoint_faults.py` 生成本报告、summary.json、overview.png 及 8 份 gzip CSV；每份数据的 SHA-256 记录在 summary.json。事件时刻在 JSON 中，可逐项核查。旧单轮报告保留为历史版本证据。',
'', '回滚：保留 v0.104.0 标签与发行包；本轮未执行回滚。']
md='\n'.join(lines)+'\n';(OUT/'analysis.md').write_text(md)
# Self-contained, offline report: render the small Markdown subset without dependencies.
blocks=[];table=False
for line in lines:
    if line.startswith('|'):
        if set(line.replace('|','').replace(':','').replace('-','').strip())==set():continue
        if not table:blocks.append('<table>');table=True
        cells=line.strip('|').split('|');blocks.append('<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells)+'</tr>');continue
    if table:blocks.append('</table>');table=False
    if line.startswith('!['):
        image=base64.b64encode((OUT/'overview.png').read_bytes()).decode();blocks.append('<img alt="Scenario charts" src="data:image/png;base64,'+image+'">')
    elif line.startswith('# '):blocks.append('<h1>'+html.escape(line[2:])+'</h1>')
    elif line.startswith('## '):blocks.append('<h2>'+html.escape(line[3:])+'</h2>')
    elif line:blocks.append('<p>'+html.escape(line)+'</p>')
if table:blocks.append('</table>')
(OUT/'analysis.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>4WIS 多点故障分析 v'+__version__+'</title><style>body{font:16px/1.7 system-ui,sans-serif;max-width:1120px;margin:auto;padding:32px;background:#f5f7fa;color:#18273a}h1,h2{color:#174c77}table{border-collapse:collapse;width:100%;font-size:13px;background:white}td{border:1px solid #ccd6e0;padding:8px}tr:first-child{font-weight:bold;background:#e3edf5}img{width:100%;margin:20px 0}p{overflow-wrap:anywhere}</style>'+''.join(blocks)+'</html>')
print('REPORT',OUT,flush=True)
