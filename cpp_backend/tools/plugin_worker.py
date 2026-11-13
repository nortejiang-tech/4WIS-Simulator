"""Optional user-code adapter. Built-in simulation never starts this process.

Not a security sandbox: plugins are trusted user programs, as in the Python
route. A separate process contains crashes and enforces the C++ deadline.
"""
import argparse, contextlib, dataclasses, json, math, os, pathlib, sys, time
for stream in (sys.stdin,sys.stdout,sys.stderr):
 if hasattr(stream,'reconfigure'):stream.reconfigure(encoding='utf-8')
args=argparse.ArgumentParser();args.add_argument('--root',required=True);root=pathlib.Path(args.parse_args().root)
sys.path.insert(0,str(root/'backend/src'))
sys.path.insert(0,str(root/'app/backend'))
sys.path.insert(0,str(root/'vendor'))
output=sys.stdout
sys.stdout=sys.stderr
import numpy as np
from sim4wis.core.state import VehicleParams,VehicleState,DriverInput
from sim4wis.project.params_codec import params_from_dict
from sim4wis.controller.registry import make_strategy
from sim4wis.controller.plugins.loader import discover_and_register
strategy=None
last_poll=0

def clean(x):
 if isinstance(x,np.ndarray):return clean(x.tolist())
 if isinstance(x,np.generic):return clean(x.item())
 if isinstance(x,float) and not math.isfinite(x):return None
 if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [clean(v) for v in x]
 return x

def handle(request):
 global strategy,last_poll
 operation=request['operation']
 if operation=='init':
  os.environ['SIM4WIS_DATA_DIR']=str(root)
  os.environ['SIM4WIS_PLUGINS_DIR']=request['plugins_dir']
  # HotReloadStrategy historically resolves through paths.plugins_dir, while
  # FMU and MATLAB use loader.plugin_root. Respect the same selected folder.
  import sim4wis.controller.user_python as hot
  hot.plugins_dir=lambda:pathlib.Path(request['plugins_dir'])
  discover_and_register()
  strategy=make_strategy(request['strategy'],params_from_dict(request['params'],VehicleParams()))
  return {'ready':True}
 if strategy is None:raise RuntimeError('worker not initialized')
 if operation=='status':return strategy.status_dict() if hasattr(strategy,'status_dict') else {'status':'ok'}
 if operation=='reload':return strategy.force_reload()
 if operation!='compute':raise ValueError('unknown plugin operation')
 if hasattr(strategy,'_path') and time.monotonic()-last_poll>1:
  last_poll=time.monotonic()
  try:modified=strategy._path.stat().st_mtime
  except FileNotFoundError:modified=-1
  if modified!=strategy._mtime:strategy.force_reload()
 state=VehicleState()
 for name,value in request['state'].items():
  if hasattr(state,name):setattr(state,name,np.asarray(value,dtype=getattr(state,name).dtype) if isinstance(getattr(state,name),np.ndarray) else value)
 command=strategy.compute(DriverInput(**request['driver']),state,request['dt'])
 return clean(dataclasses.asdict(command))

for line in sys.stdin:
 try:response={'result':handle(json.loads(line))}
 except Exception as exc:response={'error':str(exc)}
 output.write(json.dumps(response,allow_nan=False,separators=(',',':'))+'\n');output.flush()
