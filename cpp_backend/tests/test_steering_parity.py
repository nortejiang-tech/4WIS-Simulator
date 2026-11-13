"""Controller, actuator and sensor chronology is independently compared with Python."""
import copy,dataclasses,json,math,os,pathlib,subprocess,sys
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backend/src'))
from sim4wis.steering.tracking.controller import make_controller
from sim4wis.core.state import VehicleParams,ControlCommand,EnvironmentState
from sim4wis.vehicle.model_registry import make_vehicle_model
from sim4wis.environment.disturbance import Scene
from sim4wis.core.derived import update_derived_outputs
from sim4wis.native import use_engine
from sim4wis.project.params_codec import params_from_dict
EXE=pathlib.Path(os.environ.get('CPP_PROBE',ROOT/'cpp_backend/build/sim4wis-cpp-probe'))
def probe(req):
 p=subprocess.run([str(EXE)],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True)
 r=json.loads(p.stdout);assert 'error' not in r,r
 return r
@pytest.mark.parametrize('name',['open_loop','pid_single','pid_cascade','lqr','dob','adrc','smc','mpc','h_inf'])
@pytest.mark.parametrize('feed',[False,True])
def test_tracking_controller(name,feed):
 kw={}
 if feed and name!='open_loop':kw['ff']={'blocks':[{'type':'velocity','damping':3.5,'inertia':.63}] if name in ('dob','adrc') else [{'type':'rack_force','lowpass_hz':12},{'type':'velocity','accel_term':False},{'type':'friction','friction_nm':.8,'eps_rad_s':.03}]}
 control=make_controller(name,**copy.deepcopy(kw));steps=[];expected=[]
 for i in range(240):
  t=i*.0005
  x=dict(dt=.0005,target_angle=.001*math.sin(t*5),target_rate=.005*math.cos(t*5),feedback_angle=.0007*math.sin(t*4.8),plant_angle=.00075*math.sin(t*4.8),load_torque=.1*math.cos(t),speed_ms=12)
  steps.append(x);o=control.step(**x);expected.append(dataclasses.asdict(o))
 actual=probe(dict(action='tracking',controller=name,kwargs=kw,steps=steps))['rows']
 for key in ('angle_set','torque_cmd'):
  np.testing.assert_allclose(np.array([r[key] for r in actual],float),np.array([r[key] for r in expected],float),rtol=2e-8,atol=1e-8,equal_nan=True,err_msg=name+'/'+key)
 for key in expected[-1]['diagnostics']:
  np.testing.assert_allclose(np.array([r['diagnostics'][key] for r in actual],float),np.array([r['diagnostics'][key] for r in expected],float),rtol=2e-8,atol=1e-8,equal_nan=True,err_msg=name+'/'+key)
@pytest.mark.parametrize('model',['simplified_dynamic','multibody'])
@pytest.mark.parametrize('arch',['c_eps','p_eps','dp_eps','r_eps','eps_rws','sbw','sbw_rws','4wis'])
def test_vehicle_steering(model,arch):
 p=params_from_dict({'steering_system':{'enabled':True,'architecture':arch}},VehicleParams())
 defs=[{'command':{'delta_cmd':[.004,.0035,-.002,-.0018],'wheel_speed_cmd':[12]*4},'dt':.00125,'count':600}]
 expected=[]
 with use_engine('python'):
  m=make_vehicle_model(p,model);env=EnvironmentState(scene=Scene(base_mu=.85),mu=.85)
  for st in defs:
   c=ControlCommand.zero()
   for k,v in st['command'].items():setattr(c,k,np.array(v,dtype=float))
   for i in range(st['count']):
    m.step(st['dt'],c,env);update_derived_outputs(m.state,p);m.state.delta_reported[:]=m.state.delta
    if (i+1)%20==0:
     row=dataclasses.asdict(m.state)
     for k in ('slip_alpha','slip_kappa','tire_fx','tire_fy'):row[k]=getattr(m,k).copy()
     row['steering_channels']=dict(m.steering_channels);expected.append(row)
 actual=probe(dict(params=dataclasses.asdict(p),model=model,steps=defs,record_every=20,steering_channels=True))['rows']
 for key in actual[0]:
  if key=='steering_channels':
   for sk in expected[0][key]:
    np.testing.assert_allclose(np.array([r[key].get(sk) for r in actual],float),np.array([r[key][sk] for r in expected],float),rtol=1e-8,atol=1e-7,equal_nan=True,err_msg=arch+'/'+sk)
  else:
   atol=1e-5 if any(s in key for s in ('force','torque','fz','grip_margin','tire_f')) else 1e-9
   np.testing.assert_allclose(np.array([r[key] for r in actual],float),np.array([r[key] for r in expected],float),rtol=1e-8,atol=atol,equal_nan=True,err_msg=arch+'/'+key)
