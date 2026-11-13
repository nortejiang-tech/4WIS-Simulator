"""Re-use the shipped HTTP contract checks against an actual standalone binary."""
import asyncio,csv,hashlib,io,json,os,pathlib,subprocess,time,zipfile
import numpy as np
import pytest
from test_study_api import TestCapabilities,TestDryRun,TestRun,TestTrace
from test_agent_interaction import test_discovery_and_openapi_describe_the_contract,config,create,step
from conftest import ROOT,SERVER

def test_agent_isolation_retry_artifacts(client):
 client.post('/api/interaction/control',json={'action':'pause'})
 before=client.get('/api/status').json()
 sid=create(client);original=step(client,sid,steps=101).json()
 replay=step(client,sid,steps=101).json();assert replay['replayed'] and replay['state']==original['state']
 assert step(client,sid,steps=99).json()['detail']['code']=='request_conflict'
 assert step(client,sid,'x',0).json()['detail']['code']=='revision_conflict'
 assert client.get('/api/status').json()['t']==before['t']
 exported=client.post(f'/api/agent/sessions/{sid}/export').json()
 assert exported['samples']==101 and exported['full_data_decimated'] is False
 assert client.post(f'/api/agent/sessions/{sid}/export').json()['run_id']==exported['run_id']
 raw=client.get(exported['outputs']['csv']).content
 assert hashlib.sha256(raw).hexdigest()==exported['csv_sha256']
 rows=list(csv.DictReader(io.StringIO(raw.decode())))
 full=client.get(exported['outputs']['json']).json()['rows']
 assert len(rows)==len(full)==101
 for a,b in zip(rows,full):
  for key,value in a.items():assert b[key]==(float(value) if value else None)
 archive=zipfile.ZipFile(io.BytesIO(client.get(exported['outputs']['bundle']).content))
 assert archive.read('data.csv')==raw
 assert {'trajectory.svg','chart.svg','meta.json','manifest.json'}<=set(archive.namelist())
 assert step(client,sid,'b',1,steps=10).status_code==200
 assert client.get(exported['outputs']['csv']).content==raw
 preview=client.get(f'/api/agent/sessions/{sid}/preview?max_points=9').json()
 assert preview['returned_samples']==9 and preview['decimated']
 assert client.delete(f'/api/agent/sessions/{sid}').status_code==200
 assert client.get(f'/api/agent/sessions/{sid}').status_code==404
 assert client.get(exported['outputs']['csv']).status_code==200

@pytest.mark.parametrize('model',['kinematic','simplified_dynamic','multibody'])
def test_chunking(client,model):
 a,b=create(client,model_type=model),create(client,model_type=model)
 assert step(client,a).status_code==200
 assert step(client,b,steps=40).status_code==200
 assert step(client,b,'b',1,steps=60).status_code==200
 ar=client.get(f'/api/agent/sessions/{a}/preview?max_points=1000').json()['data']
 br=client.get(f'/api/agent/sessions/{b}/preview?max_points=1000').json()['data']
 assert ar==br

@pytest.mark.parametrize('overrides',[{'mass_typo':5},{'mass':0},{'inertia_z':0},{'tire_radius':0}])
def test_invalid_vehicle(client,overrides):
 assert client.post('/api/agent/sessions',json=config(vehicle={'overrides':overrides})).status_code==422
 assert client.get('/api/agent/sessions').json()['sessions']==[]

@pytest.mark.parametrize('control',[{'throttle':2},{'steering':-2},{'gear':3},{'unknown_control':1},{'wheel_norm':[0]*4},{'body_fraction':[0]*3}])
def test_invalid_controls(client,control):
 sid=create(client);assert step(client,sid,**control).status_code==422
 assert client.get(f'/api/agent/sessions/{sid}').json()['steps']==0

def test_fault_atomicity_and_event_timing(client):
 sid=create(client)
 req={'request_id':'faults','expected_revision':0,'faults':[{'id':'a','fault_type':'stuck_hold','wheel':'FL','t_start':.023,'t_end':.067},{'id':'b','fault_type':'free_caster','wheel':'RR','t_start':.023,'t_end':.091}], 'seed':123}
 res=client.post(f'/api/agent/sessions/{sid}/faults',json=req);assert res.status_code==200,res.text
 assert client.post(f'/api/agent/sessions/{sid}/faults',json=req).json()['replayed']
 bad={**req,'request_id':'bad','expected_revision':1,'faults':req['faults']+[{'fault_type':'stuck_value','wheel':0,'t_start':.02,'value':.1}]}
 assert client.post(f'/api/agent/sessions/{sid}/faults',json=bad).status_code==422
 result=step(client,sid,'drive',1,steps=40).json()
 events=result['fault_timeline']['events']
 activations=[e for e in events if e['event']=='activated']
 assert len(activations)==2 and all(e['time']==pytest.approx(.023,abs=1e-12) for e in activations)
 endings=[e for e in events if e['event']=='deactivated']
 assert sorted(e['time'] for e in endings)==pytest.approx([.067,.091],abs=1e-12)
 assert client.get(f'/api/agent/sessions/{sid}').json()['revision']==2

def test_origin_static_and_safety(client):
 assert client.get('/health').headers['connection']=='close'
 assert client.get('/api/params',headers={'Origin':'https://evil.example'}).status_code==403
 assert client.options('/api/faults/x',headers={'Origin':'http://localhost:8000'}).status_code==204
 assert 'PATCH' in client.options('/api/faults/x',headers={'Origin':'http://localhost:8000'}).headers['access-control-allow-methods']
 assert client.get('/').status_code==200
 assert client.get('/api/study/trace/nope?channels=vx&max_points=10000000').status_code==422

def test_custom_python_reload(cpp_http):
 client,port,env,path,process=cpp_http
 assert client.post('/api/strategy',json={'name':'user_python'}).status_code==200
 status=client.get('/api/user_python/status').json();assert status['status']=='ok',status
 (path/'plugins/user_strategy.py').write_text('def compute(driver,state):\n print("plugin log belongs on stderr")\n return {"delta_cmd":[.03]*4}\n')
 assert client.post('/api/user_python/reload').json()['status']=='ok'
 time.sleep(.1)
 assert process.poll() is None
 assert client.get('/api/interaction/status').json()['paused'] is False
 assert client.post('/api/strategy',json={'name':'ideal_ackermann'}).status_code==200

@pytest.mark.asyncio
async def test_official_mcp_client(cpp_http):
 from mcp import ClientSession,StdioServerParameters
 from mcp.client.stdio import stdio_client
 _,port,env,_,_=cpp_http
 args=['--mcp','--root',str(ROOT),'--port',str(port)]
 async with stdio_client(StdioServerParameters(command=SERVER,args=args,env=env)) as (read,write):
  async with ClientSession(read,write) as session:
   initialized=await session.initialize();assert initialized.server_info.version=='0.107.0'
   tools=await session.list_tools();assert len(tools.tools)==18
   created=await session.call_tool('create_session',config())
   assert not created.is_error
   result=created.structured_content or json.loads(created.content[0].text)
   sid=result['session_id']
   faults=await session.call_tool('set_fault_schedule',dict(session_id=sid,request_id='f',expected_revision=0,faults=[dict(wheel=0,fault_type='sensor_bias',value=.02,t_start=.01),dict(wheel=3,fault_type='sensor_bias',value=-.03,t_start=.025)],seed=5))
   assert not faults.is_error,faults
   stepped=await session.call_tool('step_session',dict(session_id=sid,request_id='s',expected_revision=1,steps=50,control={'throttle':.1}))
   assert not stepped.is_error,stepped
   exported=await session.call_tool('export_session',{'session_id':sid});assert not exported.is_error,exported
   closed=await session.call_tool('close_session',{'session_id':sid});assert not closed.is_error


def test_completed_batch_registry_is_bounded(client):
 first=None
 for index in range(53):
  body={'experiment':{'name':f'bounded_{index}','maneuver':{'steps':[{'duration':.01,'speed_kmh':0}]}}}
  response=client.post('/api/batch',json=body)
  assert response.status_code==200,response.text
  jid=response.json()['job_id'];first=first or jid
  for _ in range(200):
   queried=client.get('/api/batch/'+jid)
   assert queried.status_code==200,(index,jid,queried.text)
   job=queried.json()
   if job['status']!='running':break
   time.sleep(.01)
  assert job['status']=='done',job
 assert len(client.get('/api/batch').json()['jobs'])==50
 assert client.get('/api/batch/'+first).status_code==404


def test_unicode_named_resources(cpp_http):
 client,_,_,folder,_=cpp_http
 name='中文车型_测试'
 assert client.post('/api/vehicle-profiles/'+name,json={'label':'中文标定'}).status_code==200
 assert client.get('/api/vehicle-profiles/'+name).status_code==200
 assert client.post('/api/vehicle-profiles/'+name+'/apply').status_code==200
 assert client.delete('/api/vehicle-profiles/'+name).status_code==200
 assert client.get('/api/vehicle-profiles/'+name).status_code==404
 experiment={'name':name,'maneuver':{'steps':[{'duration':.01,'speed_kmh':0}]}}
 assert client.post('/api/experiments/'+name,json=experiment).status_code==200
 assert client.get('/api/experiments/'+name).status_code==200
 assert client.delete('/api/experiments/'+name).status_code==200
 assert client.get('/api/experiments/'+name).status_code==404
 assert client.post('/api/projects/'+name,json={'name':name,'description':'中文工程'}).status_code==200
 assert client.get('/api/projects/'+name).json()['project']['name']==name
 assert client.post('/api/projects/'+name+'/load').status_code==200
 scripts=folder/'scripts_lib';scripts.mkdir(exist_ok=True)
 (scripts/(name+'.yaml')).write_text('name: 中文脚本\nactions: []\n',encoding='utf-8')
 response=client.get('/api/script/library/'+name)
 assert response.status_code==200,response.text
 assert '中文脚本' in response.json()['yaml']
