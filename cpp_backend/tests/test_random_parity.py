import hashlib,json,os,pathlib,subprocess
import numpy as np
import pytest
ROOT=pathlib.Path(__file__).resolve().parents[2]
EXE=pathlib.Path(os.environ.get('CPP_PROBE',ROOT/'cpp_backend/build/sim4wis-cpp-probe'))
@pytest.mark.parametrize('seed',[0,1,1234,20261114,2**32-1,2**63+42,2**64-1])
def test_pcg64_seed_sequence_and_normal(seed):
 text=f'{seed}:1:sensor_noise:2.321'
 q={'action':'random','seed':seed,'count':12000,'text':text}
 out=json.loads(subprocess.run([str(EXE)],input=json.dumps(q)+'\n',text=True,capture_output=True,check=True).stdout)
 assert out['sha256']==list(hashlib.sha256(text.encode()).digest())
 assert out['raw']==np.random.PCG64(seed).random_raw(20).tolist()
 np.testing.assert_allclose(out['normal'],np.random.default_rng(seed).normal(size=12000),rtol=0,atol=2e-15)
