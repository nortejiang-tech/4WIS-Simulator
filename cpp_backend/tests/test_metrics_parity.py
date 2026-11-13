import json,os,pathlib,subprocess,sys
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'backend/src'))
from sim4wis.study.metrics import evaluate_expression
from sim4wis.study.tracking_step import analyse_tracking_step
from sim4wis.study.tracking_disturbance import analyse_tracking_disturbance
from sim4wis.study.tracking_sweep import analyse_tracking_sweep
from sim4wis.study.oncentre import analyse_weave
from sim4wis.study.metrics import collect
EXE=os.environ.get('CPP_PROBE',str(ROOT/'cpp_backend/build/sim4wis-cpp-probe'))
def run(t,ch,**kw):
 data={'action':'metrics','t':t.tolist(),'channels':{k:v.tolist() for k,v in ch.items()},**kw}
 p=subprocess.run([EXE],input=json.dumps(data)+'\n',capture_output=True,text=True,check=True)
 return json.loads(p.stdout)
@pytest.mark.parametrize('expr',['mean(vx)','rms(vy)','steady(vx)','steady(vx,0.3)','max(abs(vy))','peak(vy)','first(t)','last(t)','sum(vy)','median(vx)','sqrt(mean(vy**2))','degrees(atan2(steady(vy),steady(vx)))','radians(90)','max(clip(vy,-0.1,0.2))','mean(sign(vy))','sin(pi/2)+cos(pi)+tan(0)','-2**2','2**-2','2**3**2','-5%3','-5//3','3 if last(t)>2 else 4','1 < 2 < 3','not (2>3)','(0 or 3) and 4','mean([1,2,3])','max((1,2,3))'])
def test_expressions(expr):
 t=np.arange(0,5,.01);ch={'vx':10+t,'vy':np.sin(t)}
 expected=evaluate_expression(expr,t.tolist(),{k:v.tolist() for k,v in ch.items()});actual=run(t,ch,expr=expr)
 assert 'error' not in actual,actual
 np.testing.assert_allclose(actual['value'],expected,rtol=1e-11,atol=1e-11)
@pytest.mark.parametrize('expr',["__import__('os')",'vx[0]','vx.mean()','unknown(vx)','mean(unknown)','mean(vx,axis=0)','[x for x in vx]','vx'])
def test_expression_rejects_unsafe_or_nonscalar(expr):
 t=np.arange(10.);assert 'error' in run(t,{'vx':t},expr=expr)
def test_step_procedure():
 t=np.arange(0,4,.005);cmd=np.where(t<.5,0,.06);resp=np.where(t<.5,0,.06*(1-np.exp(-(t-.5)/.1)))
 ch={f'{c}_{w}':v for w in ('fl','rl') for c,v in [('delta_cmd',cmd),('delta',resp)]}
 want=analyse_tracking_step(t,ch);got=run(t,ch,procedure='tracking_step');assert 'error' not in got,got
 for w in ('fl','rl'):
  for metric in ('rise_s','overshoot_pct','settle_s','ss_err_rad','peak_dev_rad'):
   np.testing.assert_allclose(got['trk_'+metric+'_'+w],getattr(want.get(w),metric),rtol=1e-11,atol=1e-11)
def test_disturbance_procedure():
 t=np.arange(0,5,.005);rack=np.where(t<1,0,2500.);cmd=np.full_like(t,.02);response=cmd+np.where(t<1,0,.1*np.exp(-(t-1)/.3))
 ch={'rack_force_fl':rack,'delta_cmd_fl':cmd,'delta_fl':response};want=analyse_tracking_disturbance(t,ch);got=run(t,ch,procedure='tracking_disturbance');assert 'error' not in got,got
 for key,field in [('onset_s','onset_s'),('peak_dev_rad','peak_dev_rad'),('recover_s','recover_s'),('rack_step_n','rack_step_n')]:np.testing.assert_allclose(got['trk_dist_'+key],getattr(want,field),atol=1e-10)
def test_weave_procedure():
 t=np.arange(0,30,.01);angle=.2*np.sin(2*np.pi*.2*t);rate=np.gradient(angle,t);torque=8*angle+.25*np.tanh(rate/.01);ay=.9*np.sin(2*np.pi*.2*t-.02);yaw=.03*np.sin(2*np.pi*.2*t-.05)
 ch={'steer_plant_active':np.ones_like(t),'steer_hand_angle':angle,'steer_hand_torque':torque,'ay':ay,'yaw_rate':yaw}
 names=['onc_torque_gradient_nm_per_deg','onc_torque_gradient_nm_per_g','onc_torque_at_0_1g_nm','onc_torque_hysteresis_nm','onc_torque_deadband_deg','onc_angle_gradient_deg_per_g','onc_yaw_phase_lag_deg','onc_ay_amplitude_g','onc_sw_amplitude_deg','onc_frequency_hz']
 want=collect(names,[],{},t,ch);got=run(t,ch,procedure='weave');assert 'error' not in got,got
 assert not want.errors,want.errors
 for k,v in want.values.items():np.testing.assert_allclose(got[k],v,rtol=1e-9,atol=1e-9,err_msg=k)
def test_stepped_sweep_procedure():
 t=np.arange(0,40,.002);cmd=np.zeros_like(t);response=np.zeros_like(t)
 for i,f in enumerate([.5,1,2,5,10]):
  mask=(t>=i*8)&(t<(i+1)*8);phase=2*np.pi*f*(t[mask]-i*8);cmd[mask]=.03*np.sin(phase);response[mask]=.03/np.sqrt(1+(f/3)**2)*np.sin(phase-np.arctan(f/3))
 ch={'delta_cmd_fl':cmd,'delta_fl':response};want=analyse_tracking_sweep(t,ch);got=run(t,ch,procedure='tracking_sweep');assert 'error' not in got,got
 for f,key in [(.5,'0_5'),(1,'1'),(2,'2'),(5,'5'),(10,'10')]:np.testing.assert_allclose(got['trk_amp_ratio_'+key+'hz'],want.get(f)[0],rtol=1e-8,atol=1e-9)
 np.testing.assert_allclose(got['trk_phase_lag_2hz'],want.get(2)[1],rtol=1e-8,atol=1e-8)
 np.testing.assert_allclose(got['trk_bw_hz'],want.bandwidth_hz,rtol=1e-8,atol=1e-8)
