import io
import pytest
from experiments.run_scheduled_swing_up import validate_start_slot, bounded_energy_gain, bounded_alignment_lag, bounded_center_gain
from experiments.run_upright_direction_trial import request

def test_bounded_future_slot():
 validate_start_slot(1120,1000)

@pytest.mark.parametrize('slot',[999,1000,1004.9,1180.1,float('nan'),float('inf')])
def test_no_late_or_unbounded_slot(slot):
 with pytest.raises(RuntimeError):
  validate_start_slot(slot,1000)

@pytest.mark.parametrize('path',['/api/control/arm','/api/control/start'])
def test_expired_deadline_blocks_activation_before_network(monkeypatch,path):
 monkeypatch.setenv('CARTPOLE_START_DEADLINE_UNIX','0')
 def forbidden(*args,**kwargs):raise AssertionError('network activation must not occur')
 monkeypatch.setattr('urllib.request.urlopen',forbidden)
 with pytest.raises(RuntimeError,match='deadline expired'):
  request(path,{})

def test_expired_deadline_still_allows_stop(monkeypatch):
 monkeypatch.setenv('CARTPOLE_START_DEADLINE_UNIX','0')
 monkeypatch.setattr('urllib.request.urlopen',lambda *a,**kw:io.BytesIO(b'{"ok":true}'))
 assert request('/api/control/stop',{})=={'ok':True}

def test_nonfinite_deadline_cannot_bypass_activation(monkeypatch):
 monkeypatch.setenv('CARTPOLE_START_DEADLINE_UNIX','nan')
 with pytest.raises(RuntimeError):
  request('/api/control/arm',{})

@pytest.mark.parametrize('value',['nan','inf','-inf','0','0.099','64.001','invalid'])
def test_invalid_energy_gain_exits_before_any_hardware_request(monkeypatch,value):
 from experiments import run_scheduled_swing_up as scheduled
 monkeypatch.setattr('sys.argv',['scheduled','--start-at-unix','1120','--energy-gain',value])
 def forbidden(*args,**kwargs):raise AssertionError('invalid candidate must not contact hardware')
 monkeypatch.setattr(scheduled,'request',forbidden)
 with pytest.raises(SystemExit) as stopped:
  scheduled.main()
 assert stopped.value.code==2

@pytest.mark.parametrize('value',[.1,32.,48.,64.])
def test_energy_gain_accepts_only_commissioned_api_range(value):
 assert bounded_energy_gain(str(value))==value

@pytest.mark.parametrize('value',['nan','inf','-inf','-0.001','0.061','invalid'])
def test_invalid_alignment_lag_exits_before_any_hardware_request(monkeypatch,value):
 from experiments import run_scheduled_swing_up as scheduled
 monkeypatch.setattr('sys.argv',['scheduled','--start-at-unix','1120','--energy-rate-lag',value])
 def forbidden(*args,**kwargs):raise AssertionError('invalid alignment must not contact hardware')
 monkeypatch.setattr(scheduled,'request',forbidden)
 with pytest.raises(SystemExit) as stopped:
  scheduled.main()
 assert stopped.value.code==2

@pytest.mark.parametrize('value',[0.,.03,.06])
def test_alignment_lag_stays_within_validated_controller_range(value):
 assert bounded_alignment_lag(str(value))==value

@pytest.mark.parametrize('option',['--cart-position-gain','--cart-velocity-gain'])
@pytest.mark.parametrize('value',['nan','inf','-0.01','5.01','invalid'])
def test_invalid_centering_candidate_exits_before_hardware_request(monkeypatch,option,value):
 from experiments import run_scheduled_swing_up as scheduled
 monkeypatch.setattr('sys.argv',['scheduled','--start-at-unix','1120',option,value])
 def forbidden(*args,**kwargs):raise AssertionError('invalid centering must not contact hardware')
 monkeypatch.setattr(scheduled,'request',forbidden)
 with pytest.raises(SystemExit) as stopped:
  scheduled.main()
 assert stopped.value.code==2

@pytest.mark.parametrize('value',[0.,.8,1.5,3.2,5.])
def test_centering_candidates_stay_within_validated_api_range(value):
 assert bounded_center_gain(str(value))==value
