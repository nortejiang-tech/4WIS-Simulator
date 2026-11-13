"""Development-only export of versioned data/schema; C++ runtime does not invoke Python."""
from __future__ import annotations
import ast, dataclasses, json, pathlib, sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backend/src'))
from sim4wis.core.state import VehicleParams
from sim4wis.main import app
from sim4wis.core.telemetry import AVAILABLE_CHANNELS
from sim4wis.experiment.session import SCALAR_CHANNELS,WHEEL_CHANNELS
from sim4wis.vehicle.steering_link import STEERING_CHANNELS
OUT=ROOT/'cpp_backend/data'
OUT.mkdir(parents=True,exist_ok=True)
def write(name,x): (OUT/name).write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
write('default_params.json',dataclasses.asdict(VehicleParams()))
# Include Pydantic factory/None defaults omitted by OpenAPI, so the native
# validator has the same construction semantics as the Python contract.
from pydantic import BaseModel
from pydantic_core import to_jsonable_python
contract=app.openapi()
seen=set()
def model_classes(cls):
 for c in cls.__subclasses__():
  if c in seen: continue
  seen.add(c)
  yield c
  yield from model_classes(c)
for cls in model_classes(BaseModel):
 component=contract['components']['schemas'].get(cls.__name__)
 if not component: continue
 for name,field in cls.model_fields.items():
  if name not in component.get('properties',{}) or field.is_required(): continue
  try:
   default=field.get_default(call_default_factory=True)
   component['properties'][name]['default']=to_jsonable_python(default)
  except (TypeError,ValueError): pass
write('openapi.json',contract)
write('channels.json',{'available':list(AVAILABLE_CHANNELS),'experiment_scalar':SCALAR_CHANNELS,'experiment_wheel':WHEEL_CHANNELS,'steering':STEERING_CHANNELS})
tree=ast.parse((ROOT/'backend/src/sim4wis/core/state.py').read_text())
classes={n.name:n for n in tree.body if isinstance(n,ast.ClassDef)}
s=['// Generated from core/state.py by tools/export_contract.py. Do not edit defaults by hand.','#pragma once','#include "common.hpp"','namespace fourwis {']
for name in ('SuspensionParams','SteeringGeometryParams','VehicleParams'):
 fields=[]
 for n in classes[name].body:
  if not isinstance(n,ast.AnnAssign):continue
  t=ast.unparse(n.annotation); field=n.target.id
  cpp={'float':'double','bool':'bool','str':'std::string','Any':'json','SuspensionParams':'SuspensionParams','SteeringGeometryParams':'SteeringGeometryParams','SteeringSystemParams':'json'}[t]
  if isinstance(n.value,ast.Constant): value="nullptr" if n.value.value is None else json.dumps(n.value.value)
  else:value='{}'
  fields.append((field,cpp,value))
 s.append(f'struct {name} {{')
 for field,cpp,value in fields:s.append(f'    {cpp} {field} = {value};')
 s.append('};')
 s.append(f'inline void to_json(json& j, const {name}& p) {{ j=json::object();')
 for field,cpp,value in fields:s.append(f'    j["{field}"]=p.{field};')
 s.append('}')
 s.append(f'inline void from_json(const json& j, {name}& p) {{')
 for field,cpp,value in fields:s.append(f'    if(j.contains("{field}")) j.at("{field}").get_to(p.{field});')
 s.append('}')
s.append('}')
(ROOT/'cpp_backend/include/params.hpp').write_text('\n'.join(s)+'\n')
from sim4wis.api.routers.agent import capabilities as agent_caps
from sim4wis.api.routers.study import capabilities as study_caps
from sim4wis.controller.path import list_templates,template_specs
from sim4wis.controller.registry import available_strategies
from sim4wis.vehicle.model_registry import model_ids,model_infos,serialize_model_info
from sim4wis.project.vehicle_profiles import _builtin_ls9
from sim4wis.recorder.buffer import DEFAULT_CHANNELS
from sim4wis_mcp.server import TOOL_DESCRIPTIONS
import asyncio
write('catalog.json',{'agent_capabilities':agent_caps(),'study_capabilities':asyncio.run(study_caps()),'path_templates':{'templates':list_templates(),'specs':template_specs()},'strategies':available_strategies(),'models':[serialize_model_info(i) for i in model_infos()],'primary_models':model_ids(include_research=False),'builtin_profile':_builtin_ls9(),'recording_default_channels':DEFAULT_CHANNELS,'available_channels':list(AVAILABLE_CHANNELS)})
write('mcp_tools.json',[t.model_dump(mode='json',by_alias=True,exclude_none=True) for t in TOOL_DESCRIPTIONS])

from sim4wis.environment.scenario import _BUILDERS
write('scenarios.json',{n:f().serialize() for n,f in _BUILDERS.items()})
from sim4wis.project.schema import ProjectFile
write('project_defaults.json',ProjectFile().model_dump(mode='json'))
write('project_schema.json',ProjectFile.model_json_schema())

from sim4wis.targets.library import BUILTIN as TARGETS
write('targets.json',{ref:{**t.to_dict(),'ref':t.ref,'digest':t.digest()} for ref,t in TARGETS.items()})
from sim4wis.steering.sizing import DEFAULT_SCENARIOS
write('sizing_scenarios.json',[dataclasses.asdict(s) for s in DEFAULT_SCENARIOS])

sys.path.insert(0,str(ROOT/'scripts'))
import check_golden_experiments as golden
import study_single_wheel_failure as swf
import math
cases=[]
for name,cfg in golden.PROCEDURE_GOLDENS.items():
 cases.append({'name':name,'kind':'procedure','path':cfg['path'],'library':cfg.get('library')})
for name,cfg in golden.SINGLE_WHEEL_GOLDENS.items():
 _,tf,_=swf.scenario_steps(cfg['scenario'])
 fault=swf.make_fault(cfg['wheel'],cfg['fault'],tf)
 ref=swf.build_exp(cfg['scenario'],None)
 exp=swf.build_exp(cfg['scenario'],fault)
 if cfg['mitigation']=='mitigated':
  kind='free' if cfg['fault']=='free_caster' else 'stuck'
  angle=math.radians(swf.STUCK_DEG) if cfg['fault']=='stuck_value' else 0.0
  exp=swf.build_exp(cfg['scenario'],fault,'fault_reconfig',swf.mitigation_params(cfg['wheel'],kind,angle,tf))
 cases.append({'name':name,'kind':'single_wheel','reference':ref.model_dump(mode='json'),'experiment':exp.model_dump(mode='json'),'t_fault':tf})
write('golden_cases.json',cases)
