"""Actual C++ API responses compared with independent Python analysis functions."""
import dataclasses,json,os,pathlib,subprocess,sys
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backend/src'))
from sim4wis.core.state import VehicleParams
from sim4wis.vehicle.load_analysis import sweep_load_analysis,sweep_sensitivity
from sim4wis.vehicle.geometry import wheel_angle_from_rack_travel
from sim4wis.project.params_codec import params_from_dict
EXE=os.environ.get('CPP_SERVER',str(ROOT/'cpp_backend/build/sim4wis-cpp'))
@pytest.fixture
def api(tmp_path):
 tmp_path=tmp_path/'中文数据 目录';tmp_path.mkdir()
 env=os.environ.copy()
 for name in ('runs','studies','vehicle_profiles','projects','experiments','targets'):
  env['SIM4WIS_'+name.upper()+'_DIR']=str(tmp_path/name)
 p=subprocess.Popen([EXE,'--root',str(ROOT),'--probe-api'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,env=env)
 def call(path,body=None,method=None,query=None):
  p.stdin.write(json.dumps(dict(path=path,body=body,method=method or ('POST' if body is not None else 'GET'),query=query or {}))+'\n');p.stdin.flush()
  line=p.stdout.readline();assert line,p.stderr.read();r=json.loads(line)
  return r
 yield call
 p.stdin.close();p.wait(timeout=10);assert p.returncode==0,p.stderr.read()
def assert_tree(actual,expected,path='root'):
 if isinstance(expected,dict):
  assert set(actual)==set(expected),path
  for k,v in expected.items():assert_tree(actual[k],v,path+'.'+k)
 elif isinstance(expected,(list,tuple)):
  assert len(actual)==len(expected),path
  for i,(a,b) in enumerate(zip(actual,expected)):assert_tree(a,b,f'{path}[{i}]')
 elif isinstance(expected,(int,float,np.number)) and not isinstance(expected,(bool,np.bool_)):
  if not np.isfinite(expected):assert actual is None or not np.isfinite(actual),path
  else:np.testing.assert_allclose(actual,expected,rtol=2e-9,atol=2e-7,err_msg=path)
 else:assert actual==expected,path
@pytest.mark.parametrize('wheel',range(4))
@pytest.mark.parametrize('coupling',['vehicle','isolated'])
@pytest.mark.parametrize('mode',['single_wheel','ideal_ackermann'])
def test_load_analysis(api,wheel,coupling,mode):
 patch={'cg_to_front':1.36,'tire_c_alpha_front_scale':.9,'tire_c_alpha_rear_scale':1.1}
 params=params_from_dict(patch,VehicleParams())
 request=dict(speeds=[0,1,2,5,17,35,60],angles=np.linspace(-.64,.64,21).tolist(),wheel_index=wheel,mode=mode,mu=.62,body_coupling=coupling)
 expected=sweep_load_analysis(params,**request)
 response=api('/api/load-analysis/sweep',{**request,'params':patch});assert response['status']==200,response
 assert_tree(response['body'],expected)
@pytest.mark.parametrize('wheel',range(4))
def test_rack_inverse(api,wheel):
 travels=np.linspace(-.11,.11,35);hint=.03;expected=[]
 for travel in travels:
  sol=wheel_angle_from_rack_travel(travel,VehicleParams().steering_geometry,wheel,hint);expected.append(sol)
  if sol['valid']:hint=sol['delta']
 response=api('/api/load-analysis/rack-solve',dict(rack_travels=travels.tolist(),wheel_index=wheel,hint_delta=.03))
 assert response['status']==200,response
 assert_tree(response['body'],dict(wheel_index=wheel,solutions=expected))
@pytest.mark.parametrize('vary,values',[('suspension.caster_angle',[.02,.06,.12]),('static_toe_front',[-.003,0,.003]),('cg_to_front',[1.2,1.6,1.9]),('mass_typo',[1000,2000]),('suspension.caster_typo',[.02,.06])])
def test_sensitivity(api,vary,values):
 req=dict(vary_param=vary,values=values,speed=12,angles=np.linspace(-.5,.5,41).tolist(),wheel_index=2,mu=.8,body_coupling='vehicle')
 expected=sweep_sensitivity(VehicleParams(),**req);response=api('/api/load-analysis/sensitivity',req)
 assert response['status']==200,response
 assert_tree(response['body'],expected)
