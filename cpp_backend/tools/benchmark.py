#!/usr/bin/env python3
"""Paired whole-experiment comparison: Python, native kernel, full C++ service."""
import argparse,importlib.util,json,os,pathlib,platform,statistics,subprocess,sys,time
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backend/src'));sys.path.insert(0,str(ROOT/'scripts'))
# A development-only optional extension location allows comparison without
# installing the Python service into the full C++ runtime.
location=os.environ.get('NATIVE_REFERENCE_EXTENSION')
if location:
 spec=importlib.util.spec_from_file_location('sim4wis._native',location)
 module=importlib.util.module_from_spec(spec);sys.modules['sim4wis._native']=module;spec.loader.exec_module(module)
from benchmark_native_core import config
from sim4wis.native import use_engine,engine_info
from sim4wis.experiment.session import run_experiment
import numpy as np

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--binary',type=pathlib.Path,required=True);parser.add_argument('--output',type=pathlib.Path,required=True);parser.add_argument('--repeat',type=int,default=5);parser.add_argument('--duration',type=float,default=14);args=parser.parse_args()
 cases=[]
 for model in ('simplified_dynamic','multibody'):
  for tire in ('linear','pacejka'):
   for scenario in ('simultaneous','staggered'):
    exp=config(model,tire,scenario,args.duration);timings={k:[] for k in ('python','native','cpp_full')};outputs={}
    def run(engine):
     start=time.perf_counter()
     if engine=='cpp_full':
      result=subprocess.run([str(args.binary.resolve()),'--root',str(ROOT),'--run-experiment','-'],input=exp.model_dump_json(),text=True,encoding='utf-8',capture_output=True,check=True)
      out=json.loads(result.stdout)
     else:
      with use_engine(engine):r=run_experiment(exp)
      out={'t':r.t,'channels':r.channels,'n_steps':r.n_steps,'meta':r.meta}
     elapsed=time.perf_counter()-start;return elapsed,out
    engines=list(timings)
    for engine in engines:_,outputs[engine]=run(engine)
    differences={}
    ref=outputs['python']
    for engine in engines[1:]:
     actual=outputs[engine];assert actual['t']==ref['t'] and actual['n_steps']==ref['n_steps']
     assert actual['meta']['fault_timeline']==ref['meta']['fault_timeline']
     differences[engine]={}
     for name,values in ref['channels'].items():
      x=np.array(values,float);y=np.array(actual['channels'][name],float)
      tolerance=1e-5 if name.startswith(('fz_','rack_force_','motor_torque_','torque_steer_','grip_margin_')) else 1e-9
      np.testing.assert_allclose(x,y,rtol=1e-9,atol=tolerance,equal_nan=True,err_msg=engine+'/'+name)
      finite=np.isfinite(x)&np.isfinite(y);differences[engine][name]=float(np.max(abs(x[finite]-y[finite]))) if finite.any() else None
    for repeat in range(args.repeat):
     order=engines[repeat%3:]+engines[:repeat%3]
     for engine in order:elapsed,_=run(engine);timings[engine].append(elapsed)
    medians={key:statistics.median(values) for key,values in timings.items()}
    row={'model':model,'tire':tire,'case':scenario,'experiment':exp.model_dump(mode='json'),'times_s':timings,'median_s':medians,'speedup_over_python':{k:medians['python']/v for k,v in medians.items()},'cpp_full_over_native':medians['native']/medians['cpp_full'],'parity':'PASS','max_abs_channel_differences':differences,'fault_timeline':ref['meta']['fault_timeline']}
    cases.append(row);print(model,tire,scenario,json.dumps(medians),flush=True)
 output={'status':'PASS','date':'2026-11-14','source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'binary_sha256':__import__('hashlib').sha256(args.binary.read_bytes()).hexdigest(),'python':sys.version,'platform':platform.platform(),'machine':platform.machine(),'processor':platform.processor(),'repeat':args.repeat,'duration_s':args.duration,'method':'one warm-up for each route; rotating order; no concurrent local test/build workers; full C++ includes new-process startup, KPI calculation, full JSON serialization and parsing (Python/native timings only include run_experiment and recording)','native_reference':engine_info(),'cases':cases}
 args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
