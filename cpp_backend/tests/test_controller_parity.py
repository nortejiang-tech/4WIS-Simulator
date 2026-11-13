import dataclasses,json,os,pathlib,subprocess,sys
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'backend/src'))
from sim4wis.core.state import VehicleParams,VehicleState,DriverInput
from sim4wis.controller.registry import make_strategy
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.controller.longitudinal import apply_drive_command,apply_brake_command
EXE=pathlib.Path(os.environ.get('CPP_PROBE',ROOT/'cpp_backend/build/sim4wis-cpp-probe'))
@pytest.mark.parametrize('strategy',['ackermann','ideal_ackermann','rear_wheel_steer','crab','zero_radius','manual_body','manual_wheel','user_js','follow_trajectory','fault_reconfig'])
@pytest.mark.parametrize('rws_mode',['fixed_ratio','speed_schedule','yaw_feedback','transient','model_following'])
def test_controller_sequence(strategy,rws_mode):
 p=VehicleParams(longitudinal_mode='torque',cg_to_front=1.2)
 rng=np.random.default_rng(447);steps=[];expected=[];ctrl=make_strategy(strategy,p)
 for i in range(24):
  s=VehicleState(t=i*.005,vx=float(rng.uniform(-1,20)),vy=float(rng.normal(0,.2)),yaw_rate=float(rng.normal(0,.1)))
  d=DriverInput(throttle=.35,brake=.2 if i>16 else 0,steering=float(rng.uniform(-.3,.3)),mode_params={'rws_mode':rws_mode,'fault_time':0,'detect_delay':.025,'fault_wheel':i%4,'wheel_norm':[.1,-.2,.3,.05],'vx_frac':.1,'vy_frac':.02,'yaw_frac':.01})
  if i%3==0:d.mode_params['steer_raw_rad']=.08
  if i%4==0:d.mode_params['speed_target_ms']=12
  c=ctrl.compute(d,s,.005);apply_brake_command(c,d,p);apply_drive_command(c,d,p,s)
  expected.append(c)
  steps.append({'state':{k:getattr(s,k) for k in ('t','vx','vy','yaw_rate')},'driver':dataclasses.asdict(d)})
 req={'action':'controller','params':dataclasses.asdict(p),'strategy':strategy,'steps':steps}
 out=json.loads(subprocess.run([str(EXE)],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True).stdout)
 assert 'error' not in out,out
 for i,c in enumerate(expected):
  for key in ('delta_cmd','wheel_speed_cmd','icr_target_body','drive_torque_cmd','brake_cmd'):
   np.testing.assert_allclose(np.asarray(out['commands'][i][key],dtype=float),getattr(c,key),atol=1e-10,rtol=1e-10,equal_nan=True,err_msg=f'{key} step {i}')
