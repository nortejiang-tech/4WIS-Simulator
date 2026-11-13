"""Executable-level differential checks against the retained Python oracle."""
import dataclasses,json,os,pathlib,subprocess,sys
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backend/src'))
from sim4wis.core.state import VehicleParams,ControlCommand,EnvironmentState
from sim4wis.vehicle.model_registry import make_vehicle_model
from sim4wis.environment.disturbance import Scene
from sim4wis.core.derived import update_derived_outputs
from sim4wis.native import use_engine
from sim4wis.project.params_codec import params_from_dict
EXE=pathlib.Path(os.environ.get('CPP_PROBE',ROOT/'cpp_backend/build/sim4wis-cpp-probe'))
@pytest.mark.parametrize('model',['kinematic','simplified_dynamic','multibody'])
@pytest.mark.parametrize('tire',['linear','pacejka'])
@pytest.mark.parametrize('case',['baseline','offset_kc_brake'])
def test_all_accepted_states(model,tire,case):
    raw={'tire_model':tire}
    if case!='baseline': raw.update(cg_to_front=1.1,tire_relax_length=.45,tire_mu_load_sensitivity=.15,bump_steer_coeff=.04)
    p=params_from_dict(raw,VehicleParams())
    scene=Scene(base_mu=.67)
    defs=[{'command':{'delta_cmd':[.04,.035,-.02,-.018],'wheel_speed_cmd':[20]*4},'dt':.00125,'count':800},{'command':{'delta_cmd':[-.025,-.03,.015,.02],'wheel_speed_cmd':[0]*4,'brake_cmd':[.3,.3,.15,.15]},'dt':.00125,'count':400}]
    req={'params':dataclasses.asdict(p),'model':model,'scene':scene.serialize(),'steps':defs,'record_every':20}
    proc=subprocess.run([str(EXE)],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True)
    out=json.loads(proc.stdout);assert 'error' not in out,out
    actual=out['rows'];expected=[]
    with use_engine('python'):
      m=make_vehicle_model(p,model);env=EnvironmentState(scene=scene,mu=.67)
      for st in defs:
        cmd=ControlCommand.zero()
        for k,v in st['command'].items():setattr(cmd,k,np.array(v,dtype=float))
        for i in range(st['count']):
          m.step(st['dt'],cmd,env);update_derived_outputs(m.state,p);m.state.delta_reported[:]=m.state.delta
          if (i+1)%20==0:
            row=dataclasses.asdict(m.state)
            for f in ('slip_alpha','slip_kappa','tire_fx','tire_fy'):row[f]=getattr(m,f,np.zeros(4)).copy()
            expected.append(row)
    assert len(actual)==len(expected)
    for k in actual[0]:
      a=np.asarray([r[k] for r in actual],dtype=float);b=np.asarray([r[k] for r in expected],dtype=float)
      atol=1e-5 if any(n in k for n in ('force','torque','fz','grip_margin','tire_f')) else 1e-9
      np.testing.assert_allclose(a,b,rtol=1e-9,atol=atol,equal_nan=True,err_msg=k)
