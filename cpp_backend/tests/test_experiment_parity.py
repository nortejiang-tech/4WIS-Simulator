import dataclasses,json,os,pathlib,subprocess,sys
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'backend/src'),str(ROOT/'scripts')]
from benchmark_native_core import config
from sim4wis.experiment.session import run_experiment,resolve_vehicle_params
from sim4wis.native import use_engine
EXE=pathlib.Path(os.environ.get('CPP_PROBE',ROOT/'cpp_backend/build/sim4wis-cpp-probe'))
@pytest.mark.parametrize('model',['simplified_dynamic','multibody'])
@pytest.mark.parametrize('tire',['linear','pacejka'])
@pytest.mark.parametrize('case',['simultaneous','staggered'])
def test_faults_all_channels_and_event_timeline(model,tire,case):
 exp=config(model,tire,case,3.5)
 req={'action':'experiment','params':dataclasses.asdict(resolve_vehicle_params(exp)),'experiment':exp.model_dump(mode='json')}
 out=json.loads(subprocess.run([str(EXE)],input=json.dumps(req)+'\n',text=True,capture_output=True,check=True).stdout)
 assert 'error' not in out,out
 with use_engine('python'):ref=run_experiment(exp)
 assert out['t']==ref.t
 assert out['n_steps']==ref.n_steps
 assert out['meta']['fault_timeline']==ref.meta['fault_timeline']
 assert set(out['channels'])==set(ref.channels)
 for name,values in ref.channels.items():
  atol=1e-5 if name.startswith(('fz_','rack_force_','motor_torque_','torque_steer_','grip_margin_')) else 1e-9
  np.testing.assert_allclose(np.asarray(out['channels'][name],dtype=float),values,rtol=1e-9,atol=atol,equal_nan=True,err_msg=name)
