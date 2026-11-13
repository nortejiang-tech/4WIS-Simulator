import copy,dataclasses,json,sys
import pytest
from test_analysis_parity import api,assert_tree
from sim4wis.study.spec import StudySpec
from sim4wis.study.runner import dry_run
from sim4wis.targets import library,compliance,measure
from sim4wis.steering.sizing import size_actuator
from sim4wis.core.state import VehicleParams
from test_study_api import SPEC

@pytest.mark.parametrize('model',['kinematic','simplified_dynamic','multibody'])
@pytest.mark.parametrize('explicit',[False,True])
def test_dry_study_parity(api,model,explicit):
 spec=copy.deepcopy(SPEC);spec['model']=model
 if explicit:spec['baseline']['model_type']='simplified_dynamic'
 if explicit and model!='simplified_dynamic':
  with pytest.raises(ValueError):StudySpec.model_validate(spec)
  assert api('/api/study/dry-run',spec)['status']==422
  return
 expected=dry_run(StudySpec.model_validate(spec))
 result=api('/api/study/dry-run',spec);assert result['status']==200,result
 assert_tree(result['body'],expected)

def test_target_catalog(api):
 response=api('/api/targets');assert response['status']==200
 assert_tree(response['body']['sets'],library.catalogue())
 for item in library.catalogue():
  ref=item['ref'];ts=library.get(ref)
  result=api('/api/targets/'+ref);assert result['status']==200,result
  assert_tree(result['body'],{**ts.to_dict(),'ref':ts.ref,'digest':ts.digest()})

def test_sizing_and_compliance(api):
 p=VehicleParams();p=dataclasses.replace(p,steering_system=dataclasses.replace(p.steering_system,enabled=True))
 result=size_actuator(p);target=library.catalogue()[0]['ref']
 expected={'sizing':result.to_dict(),'compliance':compliance.evaluate(library.get(target),measure.from_sizing(result)).to_dict()}
 response=api('/api/targets/'+target+'/check',{});assert response['status']==200,response
 assert_tree(response['body'],expected)
