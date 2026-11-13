"""Exercise the packaged MCP process against the packaged backend, on either OS."""
import argparse
import asyncio
import json
import math
import os
import sys
import urllib.request
from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client

async def smoke(base, require_native=False):
    # The SDK's safe default child environment omits PYTHONPATH. A portable
    # install deliberately uses bundled sources/vendor rather than site-packages.
    package_env={key:os.environ[key] for key in ('PYTHONPATH','SIM4WIS_DATA_DIR') if key in os.environ}
    params=StdioServerParameters(command=sys.executable,args=['-m','sim4wis_mcp.server','--base',base],env=package_env)
    async with stdio_client(params) as (read,write):
        async with ClientSession(read,write) as session:
            await session.initialize()
            names={t.name for t in (await session.list_tools()).tools}
            assert 'set_fault_schedule' in names
            async def call(name,args):
                response=await session.call_tool(name,args)
                if response.is_error:raise RuntimeError(response.content)
                return json.loads(response.content[0].text)
            for model in ('simplified_dynamic','multibody'):
                opened=await call('create_session',{'experiment':{'name':'portable_multipoint','model_type':model}})
                sid=opened['session_id']
                body={'session_id':sid,'request_id':'timeline','expected_revision':0,'faults':[
                    {'fault_type':'stuck_hold','wheel':0,'t_start':.023,'t_end':.083},
                    {'fault_type':'stuck_hold','wheel':1,'t_start':.023,'t_end':.083},
                    {'fault_type':'free_caster','wheel':3,'t_start':.047,'t_end':.091},
                    {'fault_type':'sensor_bias','wheel':2,'value':.02,'t_start':.037}]}
                configured=await call('set_fault_schedule',body)
                assert configured['revision']==1
                assert (await call('set_fault_schedule',body))['replayed']
                result=await call('step_session',{'session_id':sid,'request_id':'run','expected_revision':1,
                    'steps':24,'control':{'front_angle_rad':.1,'speed_target_ms':5}})
                assert result['status']=='ready' and result['steps']==24
                if require_native:
                    assert result['engine']['body_core']=='cpp17',result['engine']
                events=result['fault_timeline']['events']
                starts=[e['time'] for e in events if e['event']=='activated']
                assert len(starts)==4 and all(abs(a-b)<1e-9 for a,b in zip(starts,[.023,.023,.037,.047]))
                artifact=await call('export_session',{'session_id':sid})
                with urllib.request.urlopen(artifact['outputs']['json']) as r:data=json.load(r)
                assert len(data['rows'])==24
                row=data['rows'][-1]
                assert math.isclose(row['delta_reported_rl']-row['delta_rl'],.02,abs_tol=1e-10)
                await call('close_session',{'session_id':sid})
                print(json.dumps({'model':model,'status':'PASS','samples':24,'activation_times':starts}),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--base',required=True)
    parser.add_argument('--require-native-core',action='store_true')
    args=parser.parse_args()
    asyncio.run(smoke(args.base,args.require_native_core))
