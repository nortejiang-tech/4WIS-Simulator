#!/usr/bin/env python3
"""Extract and exercise the shipped binary, its MCP and optional plugin runtime."""
import argparse,asyncio,hashlib,io,json,os,pathlib,socket,subprocess,tempfile,time,zipfile
import httpx
async def mcp_check(binary,root,port):
 from mcp import ClientSession,StdioServerParameters
 from mcp.client.stdio import stdio_client
 async with stdio_client(StdioServerParameters(command=str(binary),args=['--root',str(root),'--port',str(port),'--mcp'])) as (read,write):
  async with ClientSession(read,write) as session:
   result=await session.initialize()
   value=result.model_dump(by_alias=True);assert value['serverInfo']['version']=='0.107.0'
   assert len((await session.list_tools()).tools)==18
   response=await session.call_tool('describe_capabilities',{})
   assert not response.model_dump(by_alias=True).get('isError'),response

def main():
 parser=argparse.ArgumentParser();parser.add_argument('archive',type=pathlib.Path);parser.add_argument('--output',type=pathlib.Path);args=parser.parse_args();checks=[]
 with tempfile.TemporaryDirectory(prefix='4wis C++ 解压 ') as folder:
  dest=pathlib.Path(folder)
  with zipfile.ZipFile(args.archive) as archive:
   assert all(not pathlib.PurePosixPath(n).is_absolute() and '..' not in pathlib.PurePosixPath(n).parts for n in archive.namelist())
   assert archive.testzip() is None;archive.extractall(dest)
  root=next(p for p in dest.iterdir() if p.is_dir());manifest=json.loads((root/'build-manifest.json').read_text());binary=root/'app'/('sim4wis-cpp.exe' if os.name=='nt' else 'sim4wis-cpp');binary.chmod(0o755)
  if os.name!='nt':
   for p in (root/'app/python/bin').glob('*'):
    if p.is_file():p.chmod(p.stat().st_mode|0o111)
  assert hashlib.sha256(binary.read_bytes()).hexdigest()==manifest['binary_sha256'];checks.append('archive_crc_and_binary_hash')
  env=os.environ.copy();env['SIM4WIS_PLUGIN_PYTHON']=str(dest/'missing-python')
  result=subprocess.run([str(binary),'--root',str(root),'--self-test'],env=env,capture_output=True,text=True,encoding='utf-8',timeout=60)
  assert result.returncode==0,result.stderr;golden=json.loads(result.stdout);checks.append('six_golden_cases_without_python')
  with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
  env.pop('SIM4WIS_PLUGIN_PYTHON',None)
  for key in list(env):
   if key.startswith('SIM4WIS_'):env.pop(key)
  with (dest/'server.log').open('w',encoding='utf-8') as log:
   proc=subprocess.Popen([str(binary),'--root',str(root),'--port',str(port)],env=env,stdout=log,stderr=log)
   try:
    with httpx.Client(base_url=f'http://127.0.0.1:{port}',timeout=30) as client:
     for _ in range(200):
      assert proc.poll() is None,'packaged server exited'
      try:
       if client.get('/health').status_code==200:break
      except httpx.ConnectError:pass
      time.sleep(.03)
     version=client.get('/api/version').json();assert version['engine']['service_runtime']=='cpp17';checks.append('native_service_identity')
     assert client.get('/').status_code==200;checks.append('production_frontend')
     for model in ('simplified_dynamic','multibody'):
      for tire in ('linear','pacejka'):
       cfg={'label':'portable','experiment':{'name':'portable','model_type':model,'vehicle':{'overrides':{'tire_model':tire}},'faults':[{'wheel':0,'fault_type':'stuck_hold','t_start':.023,'t_end':.067},{'wheel':3,'fault_type':'free_caster','t_start':.023,'t_end':.091},{'wheel':1,'fault_type':'sensor_bias','value':.03,'t_start':.051}]}}
       res=client.post('/api/agent/sessions',json=cfg);assert res.status_code==201,res.text;sid=res.json()['session_id']
       req={'request_id':'a','expected_revision':0,'steps':100,'control':{'throttle':.15,'steering':.08}}
       res=client.post(f'/api/agent/sessions/{sid}/step',json=req);assert res.status_code==200,res.text
       replay=client.post(f'/api/agent/sessions/{sid}/step',json=req).json();assert replay['replayed']
       exported=client.post(f'/api/agent/sessions/{sid}/export').json();raw=client.get(exported['outputs']['csv']).content
       assert hashlib.sha256(raw).hexdigest()==exported['csv_sha256']
       full=client.get(exported['outputs']['json']).json();assert len(full['rows'])==100
       bundle=zipfile.ZipFile(io.BytesIO(client.get(exported['outputs']['bundle']).content));assert bundle.read('data.csv')==raw
       assert client.delete(f'/api/agent/sessions/{sid}').status_code==200
       checks.append(model+'_'+tire+'_fault_retry_artifacts')
     asyncio.run(mcp_check(binary,root,port));checks.append('official_mcp_initialize_18_tools_and_call')
     if manifest['plugin_runtime_bundled']:
      assert client.post('/api/strategy',json={'name':'user_python'}).status_code==200
      status=client.get('/api/user_python/status').json();assert status['status'] in ('ok','no_file'),status
      assert client.get('/api/interaction/status').json()['paused'] is False;checks.append('bundled_optional_plugin_runtime')
   finally:
    proc.terminate()
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
  report={'status':'PASS','checks':checks,'count':len(checks),'archive_sha256':hashlib.sha256(args.archive.read_bytes()).hexdigest(),'manifest':manifest,'engine':version['engine'],'golden':golden}
  if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
  print(json.dumps({'status':'PASS','count':len(checks),'checks':checks}))
if __name__=='__main__':main()
