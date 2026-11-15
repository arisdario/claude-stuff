"""Closed-loop test of the REAL tesla/ACC_module.py (update_acc, unchanged code) with
  * openpilot 0.7.5's longitudinal MPC compiled from its own generated C code (libmpc1.so) + planner.py logic (replay.py),
    fed through the REAL _gap_distance() from tools/patch_planner_gap.py
  * a radar model with the errors measured on this car (radarState vRel ~3 km/h low, +-2 km/h, blips, distance noise,
    far-track jumps, drop-outs)
  * a Tesla cruise model from the logs (a = 0.9 x set error, +-0.6 m/s2, jerk 0.8 m/s3; set change shows 0.2 s after the
    press; RES = previous set speed; cancel = regen -1.5 m/s2) ; the plan reaches the module one cycle (50 ms) late.
Nobody brakes in these runs (the worst case: the driver does nothing)."""
import sys, os, io, contextlib, math, random, types, importlib.util, json
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H)
REPO = os.path.abspath(os.path.join(H, '..', '..'))
def mod(name, **kw):
  m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m
class CV: MPH_TO_MS=0.44704; MS_TO_KPH=3.6; KPH_TO_MS=1/3.6; MS_TO_MPH=2.23694; KPH_TO_MPH=0.621371; MPH_TO_KPH=1.609344
mod('selfdrive.car', dbc_dict=lambda *a, **k: {})
for n in ('selfdrive', 'selfdrive.car.tesla', 'selfdrive.car.tesla.speed_utils', 'cereal'): mod(n)
mod('selfdrive.config', Conversions=CV)
class FS:
  def __init__(self, *a): pass
  def adjust(self, CS, v, frame): return v
  def reset_averager(self): pass
  def is_active(self, f): return False
mod('selfdrive.car.tesla.speed_utils.fleet_speed', FleetSpeed=FS)
BOX = {}
def recv(sock):
  return BOX.pop(sock, None)
mod('cereal.messaging', sub_sock=lambda name, **k: name, recv_one_or_none=recv); sys.modules['cereal'].messaging = sys.modules['cereal.messaging']
spec = importlib.util.spec_from_file_location('values', REPO + '/values.py'); vals = importlib.util.module_from_spec(spec); spec.loader.exec_module(vals)
sys.modules['selfdrive.car.tesla.values'] = vals
CB, CSt = vals.CruiseButtons, vals.CruiseState
from replay import Planner
spec = importlib.util.spec_from_file_location('ppg', REPO + '/tools/patch_planner_gap.py'); ppg = importlib.util.module_from_spec(spec); spec.loader.exec_module(ppg)
GAPNS = {}; exec(ppg.HELPER, GAPNS); gap_distance, gap_s = GAPNS['_gap_distance'], GAPNS['_gap_s']

clock = [1000.0]
def load(path, tag):
  spec = importlib.util.spec_from_file_location('ACC_' + tag, path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
  m._current_time_millis = lambda: int(round(clock[0] * 1000))
  m.os = types.SimpleNamespace(path=types.SimpleNamespace(exists=lambda p: p == m.ACC_LOG_OFF), makedirs=os.makedirs, listdir=os.listdir, remove=os.remove)
  return m

class NS(types.SimpleNamespace): pass
class Btns:
  def get_button_label2(self, n): return 'follow'
class UE:
  def custom_alert_message(self, *a): pass

def scenarios():
  S = {}
  S['constant'] = dict(v=lambda t, v0: v0)
  S['slower by 2 km/h'] = dict(v=lambda t, v0: v0 - 2)
  S['gradual slow -0.3'] = dict(v=lambda t, v0: v0 if t < 20 else max(v0 * 0.75, v0 - 1.08 * (t - 20)))
  S['gradual accel +0.3'] = dict(v=lambda t, v0: v0 if t < 20 else min(v0 * 1.2, v0 + 1.08 * (t - 20)))
  S['sudden brake -3'] = dict(v=lambda t, v0: v0 if t < 30 else max(v0 * 0.5, v0 - 10.8 * (t - 30)), brake_t=30.)
  S['hard brake -5'] = dict(v=lambda t, v0: v0 if t < 30 else max(v0 * 0.5, v0 - 18. * (t - 30)), brake_t=30.)
  S['accelerates away +1.5'] = dict(v=lambda t, v0: v0 if t < 20 else min(v0 + 30, v0 + 5.4 * (t - 20)))
  S['cut-in 15 slower at 0.6 gap'] = dict(v=lambda t, v0: v0 if t < 25 else v0 - 15, cut=(25., 0.6), brake_t=25.)
  S['slower car ahead from 150 m'] = dict(v=lambda t, v0: v0 - 30, d0=150.)
  S['lead leaves at 30 s'] = dict(v=lambda t, v0: v0, gone=30.)
  S['noisy + drop-outs'] = dict(v=lambda t, v0: v0 + 1.5 * math.sin(t / 9.), noisy=True)
  S['your -5 press at 40 s'] = dict(v=lambda t, v0: v0, human=(40., -5))
  S['lead brakes to a stop -4'] = dict(v=lambda t, v0: v0 if t < 30 else max(0., v0 - 14.4 * (t - 30)), brake_t=30., beyond=True)
  return S

def run(path, scen, v0, seed=1, dur=90., tag='new', user_set=None):
  random.seed(seed); BOX.clear()
  A = load(path, tag)
  class P:
    def curve_speed_for_acc(self, c): return 255.
  CC = NS(PCC=P(), alca_enabled=False, HSO=NS(human_control=False))
  acc = A.ACCController(CC); acc.enable_adaptive_cruise = True; acc.prev_enable_adaptive_cruise = True; acc.adaptive = True
  acc.acc_speed_kph = user_set if user_set is not None else v0 + 15
  v = v0 / 3.6; a = 0.
  cs = NS(UE=UE(), cstm_btns=Btns(), imperial_speed_units=False, turn_signal_blinking=False, angle_steers=0., apFollowTimeInS=2.5,
          pcm_acc_status=CSt.ENABLED, cruise_buttons=CB.IDLE, v_ego=v, v_ego_raw=v, a_ego=0., v_cruise_actual=float(v0),
          brake_pressed=False, user_gas_pressed=False)
  lead_v = scen['v'](0, v0) / 3.6; prev_lv = lead_v
  d = scen['d0'] if 'd0' in scen else gap_s(v) * v
  planner = Planner(); pending_plan = None
  set_kph = float(v0); prev_set = float(v0); cc_on = True; queue = []           # (apply time, action)
  t = 0.; dt = 0.05; drop_until = -1.; gone = False
  out = dict(t=[], a=[], v=[], d=[], lv=[], la=[], set=[], want=[], press=[], cancel=[], viol=[], cc=[])
  la_hist = []
  while t < dur:
    clock[0] = 1000 + t
    lv = scen['v'](t, v0) / 3.6; la = (lv - prev_lv) / dt; prev_lv = lv; lead_v = lv
    la_hist = (la_hist + [la])[-10:]
    if 'cut' in scen and abs(t - scen['cut'][0]) < dt / 2: d = scen['cut'][1] * gap_s(v) * v
    if 'gone' in scen and t >= scen['gone']: gone = True
    heavy = scen.get('noisy', False)
    if random.random() < (0.08 if heavy else 0.02) * dt and t > drop_until: drop_until = t + random.uniform(0.3, 1.5 if heavy else 0.8)
    if gone or t < drop_until:
      lead = NS(dRel=0., vRel=0., aRel=0., aLeadK=0., status=False, vLead=0., aLeadTau=1.5)
    else:
      dm = d + random.gauss(0, 0.8 if heavy else 0.3) + (28. if d > 70 and random.random() < (0.3 if heavy else 0.05) else 0.)
      vr = lead_v - v - 3 / 3.6 + random.gauss(0, 2 / 3.6) + (25 / 3.6 if random.random() < 1 / 60. else 0.)
      lead = NS(dRel=dm, vRel=vr, aRel=0., aLeadK=la + random.gauss(0, 0.3), status=True, vLead=v + vr, aLeadTau=1.5)
    BOX['radarState'] = NS(radarState=NS(leadOne=lead))
    if pending_plan is not None: BOX['plan'] = pending_plan
    # your stalk press
    cs.cruise_buttons = CB.IDLE
    if 'human' in scen and abs(t - scen['human'][0]) < dt / 2:
      step = scen['human'][1]
      cs.cruise_buttons = CB.DECEL_2ND if step == -5 else CB.RES_ACCEL_2ND
      acc.acc_speed_kph += step; queue.append((t + 0.2, ('human', step))); out['press'].append((t, step, 'YOU'))
    cs.v_ego = cs.v_ego_raw = v; cs.a_ego = a; cs.v_cruise_actual = set_kph; cs.pcm_acc_status = CSt.ENABLED if cc_on else CSt.STANDBY
    with contextlib.redirect_stdout(io.StringIO()):
      btn = acc.update_acc(True, cs, int(t * 100), None, 0., 0., False, 0.)
    # the planner (runs on every radarState, ~20 Hz) with the gap patch; its result reaches the module next cycle
    L = None if not lead.status else dict(d=gap_distance(lead.dRel, v), v=lead.vLead, a=lead.aLeadK)
    vs, as_ = (planner.v_start, planner.a_start) if planner.v_start is not None else (v, 0.)
    p = planner.step(v, a, L, acc.acc_speed_kph / 3.6)
    pending_plan = NS(valid=True, plan=NS(vStart=vs, aStart=as_, vTarget=p['v_target'], aTarget=p['a_target'], hasLead=L is not None,
                                          longitudinalPlanSource=p['src'], vCruise=p['v_target'], aCruise=p['a_target']))
    if btn:
      out['press'].append((t, btn, acc.last_follow_msg or ''))
      queue.append((t + 0.2, btn))
      if CB.is_accel(btn):
        b_true = max(0., -sum(la_hist) / len(la_hist)); closing = max(0., v - lead_v); room = d - 6.
        need = b_true + (closing ** 2 / (2 * room) if room > 0.5 else 99.)
        if not gone and (sum(la_hist) / len(la_hist) < -0.3 or need > 0.6):
          out['viol'].append((round(t, 2), btn, round(sum(la_hist) / len(la_hist), 2), round(need, 2)))
    for item in [q for q in queue if q[0] <= t + 1e-9]:
      queue.remove(item); b = item[1]
      if isinstance(b, tuple):
        set_kph += b[1]
      elif b == CB.CANCEL:
        if cc_on: prev_set = set_kph
        cc_on = False; out['cancel'].append(t)
      elif not cc_on and b == CB.DECEL_SET:
        cc_on = True; set_kph = float(round(v * 3.6))
      elif not cc_on and b in (CB.RES_ACCEL, CB.RES_ACCEL_2ND):
        cc_on = True; set_kph = prev_set                       # Tesla RES: the PREVIOUS set speed
      elif cc_on:
        set_kph += {CB.DECEL_2ND: -5, CB.DECEL_SET: -1, CB.RES_ACCEL: 1, CB.RES_ACCEL_2ND: 5}.get(b, 0)
    if cc_on:
      a_cmd = max(-0.6, min(0.6, 0.9 * (set_kph / 3.6 - v))); jl = 0.8
    else:
      a_cmd = -1.5 if v > 0.5 else 0.; jl = 1.5
    a += max(-jl * dt, min(jl * dt, a_cmd - a))
    v = max(0., v + a * dt)
    if v == 0.: a = max(a, 0.)
    if not gone: d += (lead_v - v) * dt
    t += dt
    for k, x in (('t', t), ('a', a), ('v', v), ('d', d if not gone else float('nan')), ('lv', lead_v), ('la', la), ('set', set_kph),
                 ('want', getattr(acc, 'follow_target_kph', None) or float('nan')), ('cc', cc_on)):
      out[k].append(x)
  return out

def pct(xs, q):
  xs = sorted(xs); return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else 0.

def metrics(o, scen, v0):
  dt = 0.05; skip = int(5 / dt); n = len(o['t'])
  a = o['a'][skip:]; jerk = [(y - x) / dt for x, y in zip(a, a[1:])]
  ds = [x for x in o['d'] if x == x]
  dmin = min(ds) if ds else float('nan')
  gaps = [d / max(v, .5) for d, v in zip(o['d'][skip:], o['v'][skip:]) if d == d]
  mins = (o['t'][-1] - 5) / 60.
  unnec = 0
  autos = [p for p in o['press'] if p[0] > 5 and p[2] != 'YOU']
  for (tp, b, why) in autos:
    i = min(n - 1, int(tp / dt))
    v, lv, d = o['v'][i], o['lv'][i], o['d'][i]
    la = (o['lv'][i] - o['lv'][max(0, i - 20)])
    if d == d and abs(lv - v) * 3.6 < 2 and 0.8 * gap_s(v) <= d / max(v, .5) <= 1.25 * gap_s(v) and la > -0.3 and b != CB.CANCEL: unnec += 1
  # hunting: a set-speed press in the opposite direction of the previous one within 3 s
  rev = 0; last = None
  for (tp, b, why) in autos:
    s = 1 if CB.is_accel(b) else -1 if CB.is_decel(b) else 0
    if s and last and last[1] == -s and tp - last[0] < 3.: rev += 1
    if s: last = (tp, s)
  resp = None
  if 'brake_t' in scen:
    tb = scen['brake_t']
    for i in range(int(tb / dt), n):
      if o['a'][i] < -0.3: resp = round(o['t'][i] - tb, 2); break
  tail = [d / max(v, .5) for d, v in zip(o['d'][-int(30 / dt):], o['v'][-int(30 / dt):]) if d == d]
  ups = [x for x in a if x > 0.05]
  return dict(collision=dmin < 1., dmin=round(dmin, 1), gap_min=round(min(gaps), 2) if gaps else None,
              gap_end=round(sum(tail) / len(tail), 2) if tail else None, gap_want=round(gap_s(v0 / 3.6), 2),
              viol=len(o['viol']), viol_list=o['viol'][:5], cancels=len(o['cancel']),
              presses_min=round(len(autos) / mins, 1), unnec_min=round(unnec / mins, 2), reversals_min=round(rev / mins, 2),
              a_max=round(max(a), 2), a_min=round(min(a), 2), a_up_p90=round(pct(ups, .9), 2) if ups else 0.,
              jerk_p90=round(pct([abs(j) for j in jerk], .9), 3), jerk_rms=round((sum(j * j for j in jerk) / len(jerk)) ** .5, 3), resp=resp)

USER = {'town': dict(up90=1.25, jerk90=0.54), 'hwy': dict(up90=0.76, jerk90=0.39)}

def failures(name, scen, v0, m):
  f = []; band = 'town' if v0 < 70 else 'hwy'; U = USER[band]
  if m['collision']: f.append(('SAFETY', 'collision (min distance %.1f m)%s' % (m['dmin'], ' - beyond what cruise + regen can do: the driver must brake' if scen.get('beyond') else '')))
  elif m['dmin'] == m['dmin'] and m['dmin'] < 4.: f.append(('SAFETY', 'min distance %.1f m < 4 m' % m['dmin']))
  if m['viol']: f.append(('SAFETY', '%d speed-up press(es) while the lead decelerated / braking was needed: %s' % (m['viol'], m['viol_list'])))
  if m['jerk_p90'] > U['jerk90']: f.append(('COMFORT', 'jerk p90 %.2f > your %.2f m/s3 (%s)' % (m['jerk_p90'], U['jerk90'], band)))
  if m['a_up_p90'] > U['up90']: f.append(('COMFORT', 'acceleration p90 %.2f > your %.2f m/s2 (%s)' % (m['a_up_p90'], U['up90'], band)))
  if m['unnec_min'] > 1.: f.append(('COMFORT', '%.1f unnecessary presses/min (steady lead, gap fine)' % m['unnec_min']))
  if m['reversals_min'] > 1.: f.append(('COMFORT', '%.1f +/- reversals/min (hunting)' % m['reversals_min']))
  if name in ('constant', 'slower by 2 km/h', 'noisy + drop-outs', 'your -5 press at 40 s') and m['gap_end'] and abs(m['gap_end'] / m['gap_want'] - 1) > 0.15:
    f.append(('GAP', 'gap at the end %.2f s vs your %.2f s' % (m['gap_end'], m['gap_want'])))
  return f

if __name__ == '__main__':
  which = sys.argv[1] if len(sys.argv) > 1 else 'new'
  path = REPO + '/ACC_module.py' if which == 'new' else os.path.join(H, 'ACC_before.py')   # git show d15539e:tesla/ACC_module.py > ACC_before.py
  res = {}
  for name, sc in scenarios().items():
    for v0 in (60, 90, 120):
      fails, ms = [], []
      for seed in (1, 2, 3):
        o = run(path, sc, v0, seed=seed, tag=which)
        m = metrics(o, sc, v0); ms.append(m)
        for f in failures(name, sc, v0, m): fails.append((seed,) + f)
      res['%s @%d' % (name, v0)] = dict(metrics=ms, failures=fails)
      worst = max(ms, key=lambda m: (m['collision'], m['viol'], -m['dmin'] if m['dmin'] == m['dmin'] else 0))
      print('%-34s %3d  dmin %6s gapEnd %5s/%4s cancels %d press/min %5.1f unnec %4.2f rev %4.2f jerk90 %.2f aUp90 %.2f resp %s  FAIL %d' % (
        name, v0, worst['dmin'], worst['gap_end'], worst['gap_want'], worst['cancels'], worst['presses_min'], max(m['unnec_min'] for m in ms),
        max(m['reversals_min'] for m in ms), max(m['jerk_p90'] for m in ms), max(m['a_up_p90'] for m in ms), worst['resp'], len(fails)), flush=True)
      for fl in fails: print('      seed %d %s: %s' % fl)
  json.dump(res, open(os.path.join(H, 'results_%s.json' % which), 'w'), indent=1, default=str)
