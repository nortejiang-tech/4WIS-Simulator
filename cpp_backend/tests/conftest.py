import os,pathlib,subprocess,sys,socket,time
import httpx,pytest
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backend/src'))
sys.path.insert(0,str(ROOT/'backend/tests'))
SERVER=os.environ.get('CPP_SERVER',str(ROOT/'cpp_backend/build/sim4wis-cpp'))
@pytest.fixture
def cpp_http(tmp_path):
 tmp_path=tmp_path/'中文数据 目录';tmp_path.mkdir()
 with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
 env=os.environ.copy()
 for name in ('runs','studies','vehicle_profiles','projects','experiments','targets','plugins'):
  env['SIM4WIS_'+name.upper()+'_DIR']=str(tmp_path/name)
 plugin=tmp_path/'plugins';plugin.mkdir(exist_ok=True)
 (plugin/'user_strategy.py').write_text('def compute(driver,state):\n return {"delta_cmd":[driver["steering"]*.2]*4}\n')
 env['SIM4WIS_PLUGIN_PYTHON']=sys.executable
 log=(tmp_path/'server.log').open('w')
 process=subprocess.Popen([SERVER,'--root',str(ROOT),'--port',str(port)],env=env,stdout=log,stderr=log)
 client=httpx.Client(base_url=f'http://127.0.0.1:{port}',timeout=30)
 try:
  for _ in range(150):
   if process.poll() is not None:pytest.fail((tmp_path/'server.log').read_text())
   try:
    if client.get('/health').status_code==200:break
   except httpx.ConnectError:pass
   time.sleep(.05)
  else:pytest.fail('server not ready')
  yield client,port,env,tmp_path,process
 finally:
  client.close();process.terminate()
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:process.kill();process.wait()
  log.close()
@pytest.fixture
def client(cpp_http):return cpp_http[0]
