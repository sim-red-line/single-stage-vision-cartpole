"""One bounded swing-up trial at an explicit UTC slot; no interactive GO."""
from __future__ import annotations
import argparse
from datetime import datetime,timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from experiments.run_upright_direction_trial import request,preflight,TRIAL_ENVELOPES,DEFAULT_COMMISSIONING_PROFILE
from experiments.run_upright_direction_trial import bounded_balance_hold,swing_trial_hard_limit

def validate_start_slot(slot:float,now:float)->None:
 if not math.isfinite(slot) or not 5 <= slot-now <= 180:
  raise RuntimeError('start slot must arrive 5..180 seconds in advance')

def bounded_energy_gain(value:str)->float:
 gain=float(value)
 if not math.isfinite(gain) or not .1<=gain<=64:
  raise argparse.ArgumentTypeError('energy gain must be finite and within 0.1..64')
 return gain

def bounded_alignment_lag(value:str)->float:
 lag=float(value)
 if not math.isfinite(lag) or not 0<=lag<=.06:
  raise argparse.ArgumentTypeError('energy alignment lag must be finite and within 0..0.06 seconds')
 return lag

def bounded_center_gain(value:str)->float:
 gain=float(value)
 if not math.isfinite(gain) or not 0<=gain<=5:
  raise argparse.ArgumentTypeError('centering gain must be finite and within 0..5')
 return gain

def bounded_kick_acceleration(value:str)->float:
 acceleration=float(value)
 if not math.isfinite(acceleration) or not 0<=acceleration<=1500:
  raise argparse.ArgumentTypeError('startup acceleration must be finite and within 0..1500')
 return acceleration

def interrupted(signum,frame):
 raise KeyboardInterrupt('scheduled trial interrupted')

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--start-at-unix',type=float,required=True)
 p.add_argument('--energy-margin',type=float,default=.03)
 p.add_argument('--energy-gain',type=bounded_energy_gain,default=32.)
 p.add_argument('--energy-rate-lag',type=bounded_alignment_lag,default=0.)
 p.add_argument('--cart-position-gain',type=bounded_center_gain,default=3.2)
 p.add_argument('--cart-velocity-gain',type=bounded_center_gain,default=3.)
 p.add_argument('--kick-acceleration',type=bounded_kick_acceleration,default=200.)
 p.add_argument('--balance-hold-seconds',type=bounded_balance_hold,default=5.)
 p.add_argument('--commissioning-profile',choices=tuple(TRIAL_ENVELOPES),default=DEFAULT_COMMISSIONING_PROFILE)
 a=p.parse_args()
 hard_runtime=swing_trial_hard_limit(30.,a.balance_hold_seconds)
 validate_start_slot(a.start_at_unix,time.time())
 if not math.isfinite(a.energy_margin) or not 0 <= a.energy_margin <= .05:
  raise ValueError('energy surplus must remain in 0..0.05')
 import fcntl
 lock_stream=open('/tmp/cartpole-scheduled-trial.lock','a')
 try:
  fcntl.flock(lock_stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:
  raise RuntimeError('another scheduled hardware trial already owns the slot')
 root=Path(__file__).resolve().parents[1]
 request('/api/control/swing-up/parameters',{'cart_position_gain_s2':a.cart_position_gain,'cart_velocity_gain_s':a.cart_velocity_gain},method='PUT')
 initial=request('/api/status')
 preflight(initial,energy_swing_up=True,commissioning_profile=a.commissioning_profile)
 remaining=a.start_at_unix-time.time()
 if not 0 <= remaining <= 180:
  raise RuntimeError('start slot expired during preparation; motor remains disarmed')
 print('SCHEDULED_READY '+datetime.fromtimestamp(a.start_at_unix,timezone.utc).isoformat()+' profile='+a.commissioning_profile,flush=True)
 monotonic_slot=time.monotonic()+remaining
 child=None
 output=''
 result={'scheduled_utc':datetime.fromtimestamp(a.start_at_unix,timezone.utc).isoformat(),'commissioning_profile':a.commissioning_profile,'energy_gain':a.energy_gain,'energy_rate_lag_s':a.energy_rate_lag,'cart_position_gain_s2':a.cart_position_gain,'cart_velocity_gain_s':a.cart_velocity_gain,'kick_acceleration_mm_s2':a.kick_acceleration}
 result.update(balance_hold_target_s=a.balance_hold_seconds,hard_runtime_limit_s=hard_runtime)
 try:
  while time.monotonic()<monotonic_slot:
   time.sleep(min(.1,max(0,monotonic_slot-time.monotonic())))
  if abs(time.time()-a.start_at_unix)>1:
   raise RuntimeError('clock or deadline mismatch; motor remains disarmed')
  preflight(request('/api/status'),energy_swing_up=True,commissioning_profile=a.commissioning_profile)
  if time.time()>a.start_at_unix+1:
   raise RuntimeError('slot missed during final preflight; motor remains disarmed')
  env=dict(os.environ)
  env['CARTPOLE_START_DEADLINE_UNIX']=str(a.start_at_unix+2)
  cmd=[str(root/'.venv/bin/python'),str(root/'experiments/run_upright_direction_trial.py'),
       '--duration','30','--swing-up','--commissioning-profile',a.commissioning_profile,
       '--balance-hold-seconds',str(a.balance_hold_seconds),
       '--energy-gain',str(a.energy_gain),'--energy-margin',str(a.energy_margin),
       '--energy-rate-lag',str(a.energy_rate_lag),'--kick-acceleration',str(a.kick_acceleration),'--kick-duration','.3',
       '--notes','Scheduled supervised '+a.commissioning_profile+' swing-up; gain '+str(a.energy_gain)+'; surplus '+str(a.energy_margin)+'; alignment '+str(a.energy_rate_lag)+'; stable hold '+str(a.balance_hold_seconds)+'s']
  child=subprocess.Popen(cmd,cwd=root,env=env,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,text=True)
  output,_=child.communicate(timeout=hard_runtime+9.5)
  result['return_code']=child.returncode
  if child.returncode==0:
   full=json.loads(output)
   keys=['abort_reason','stabilization_success','observed_modes','max_deviation_from_down_deg',
         'start_position_mm','end_position_mm','final_safety','final_motor_target_mm_s',
         'final_motor_velocity_mm_s','experiment_id','monitor_file',
         'balance_hold_target_s','observed_continuous_stable_s','hard_runtime_limit_s']
   result.update({k:full.get(k) for k in keys})
  else:
   result['error']=output[-1800:]
 except BaseException as exc:
  result['error']=str(exc)
 finally:
  if child is not None:
   if child.poll() is None:
    child.terminate()
   try:
    request('/api/control/stop',{})
   except Exception as exc:
    result['stop_error']=str(exc)
   if child.poll() is None:
    try:
     extra,_=child.communicate(timeout=3)
     output+=extra or ''
    except subprocess.TimeoutExpired:
     child.kill()
     extra,_=child.communicate()
     output+=extra or ''
  log=root/'experiments/maintenance'/('scheduled-'+str(int(a.start_at_unix))+'.stdout.txt')
  log.parent.mkdir(parents=True,exist_ok=True)
  log.write_text(output,encoding='utf-8')
  result['stdout_file']=str(log)
  print('SCHEDULED_RESULT '+json.dumps(result),flush=True)
 if result.get('error') or result.get('stop_error'):
  raise SystemExit(1)

if __name__=='__main__':
 signal.signal(signal.SIGTERM,interrupted)
 signal.signal(signal.SIGHUP,interrupted)
 main()
