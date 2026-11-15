#!/usr/bin/env python3
"""patch_radard_lead.py - keep the car ahead at long range: radar-only lead when the camera model cannot confirm it.

Why: on this car (Tinkla 0.7.5.a radard) a radar object only becomes "the car ahead" (radarState.leadOne, which the
planner / MPC and the ACC follow) when the camera model ALSO sees a car there with > 50 % confidence at a matching
distance. At 120-135 m a car is a few pixels for the C920, so the model drops it - while the Bosch radar keeps tracking
it to 160 m+. The lead then vanished for seconds at highway speed, the planner planned for an empty road and the ACC
accelerated toward the car (drive 15 Nov 19:45-19:48, 130 km/h).

What the patch changes in selfdrive/controls/radard.py (nothing else):
  * the predicted path used to place radar objects reaches 200 m instead of 160 m
  * leadOne only (leadTwo, FCW, the low-speed override and the side-lane display are unchanged): when no radar object
    matches the camera's car, the closest radar object that is
      - MOVING forward (not stationary, absolute speed > 5 m/s, not oncoming),
      - within 1.5 m of the predicted path, beyond 35 m,
      - in the path for 0.5 s (10 cycles) before it is taken,
    becomes the lead (only with the Tesla radar integration on: without it radar objects have no path distance).
    It is then kept by its radar track id while it stays within 2.2 m of the path (no flicker); a
    confirmed closer car in the path replaces it. A camera+radar match always wins, and a camera-only car that is much
    closer than the radar-only one is kept (the radar may have missed it).
  * leadTwo (openpilot's second MPC, mpc2): the next MOVING radar car in the path BEYOND the lead (8 m+ farther,
    confirmed 0.5 s) when there is one; otherwise the camera model's estimate as before. The planner takes the slower of
    the two plans, so when the lead changes lane in front of a slower car the car has already been easing off for it.
  * radar-only leads carry model probability 0, so they never raise a forward-collision warning.
Not changed: stopped cars far ahead are still not taken from radar alone (signs / bridges look the same to the radar).

  python3 patch_radard_lead.py /data/openpilot/selfdrive/controls/radard.py            # dry run: shows the diff
  python3 patch_radard_lead.py FILE --apply     # backup FILE.bak_radarlead, patch, py_compile (restores itself on failure)
  python3 patch_radard_lead.py FILE --check     # says whether FILE is patched
  python3 patch_radard_lead.py FILE --restore   # puts FILE.bak_radarlead back
Refuses (changes nothing) if radard.py does not look exactly like the expected Tinkla 0.7.5.a code.
Loaded when radard starts: turn the car off and on (no EON reboot needed).
"""
import difflib
import py_compile
import shutil
import sys

MARK = '# RADARLEAD (tesla/tools/patch_radard_lead.py)'
HELPER = MARK + ''': radar-only lead at long range, see the patch script's docstring
RL_PATH_M = 1.5        # a new radar-only lead must be this close to the predicted path (m) ...
RL_KEEP_PATH_M = 2.2   # ... and is kept while it stays within this
RL_MIN_D = 35.         # radar-only beyond this distance only (closer, the camera confirms cars reliably)
RL_MIN_SPEED = 5.      # moving forward, absolute m/s: no stationary objects, no oncoming traffic
RL_CONFIRM = 10        # cycles (0.5 s at 20 Hz) a new car must stay in the path before it becomes the lead
RL_CLOSER_M = 5.       # a confirmed car this much closer in the path replaces the kept one
_RL = {'id': None, 'cand': None, 'cnt': 0}


def _rl_ok(c, v_ego, path_m):
  return bool(c.measured) and not c.stationary and abs(c.dPath) < path_m and (v_ego + c.vRel) > RL_MIN_SPEED


def _radar_only_lead(v_ego, clusters, matched):
  """the radar-only lead (a Cluster) or None. Called every cycle for leadOne; keeps its own small state."""
  if matched is not None:                      # camera + radar agree: remember that car for when the camera loses it
    _RL['id'], _RL['cand'], _RL['cnt'] = matched.track_id, None, 0
    return None
  cands = [c for c in clusters if c.dRel > RL_MIN_D and _rl_ok(c, v_ego, RL_PATH_M)]
  best = min(cands, key=lambda c: c.dRel) if cands else None
  if best is None:
    _RL['cand'], _RL['cnt'] = None, 0
  elif best.track_id == _RL['cand']:
    _RL['cnt'] += 1
  else:
    _RL['cand'], _RL['cnt'] = best.track_id, 1
  confirmed = best if (best is not None and _RL['cnt'] >= RL_CONFIRM) else None
  kept = None
  if _RL['id'] is not None:
    for c in clusters:
      if c.track_id == _RL['id'] and _rl_ok(c, v_ego, RL_KEEP_PATH_M):
        kept = c
        break
  if kept is not None and not (confirmed is not None and confirmed.dRel < kept.dRel - RL_CLOSER_M):
    return kept
  if confirmed is not None:
    _RL['id'] = confirmed.track_id
    return confirmed
  _RL['id'] = None
  return None


RL2_GAP_M = 8.         # the car beyond must be at least this much farther than the lead (not the lead's own echo)
_RL2 = {'cand': None, 'cnt': 0}


def _radar_beyond_lead(v_ego, clusters, lead):
  """the next MOVING radar car in the path BEYOND the lead (a Cluster), or None; confirmed 0.5 s like the lead.
  Goes to leadTwo, so openpilot's second MPC keeps a safe plan for it too: when the lead changes lane in front of a
  slower car, the car is already slowing for that one (a driver looks through / past the car ahead)."""
  if not lead.get('status'):
    _RL2['cand'], _RL2['cnt'] = None, 0
    return None
  cands = [c for c in clusters if c.dRel > lead['dRel'] + RL2_GAP_M and _rl_ok(c, v_ego, RL_PATH_M)]
  best = min(cands, key=lambda c: c.dRel) if cands else None
  if best is None:
    _RL2['cand'], _RL2['cnt'] = None, 0
    return None
  if best.track_id == _RL2['cand']:
    _RL2['cnt'] += 1
  else:
    _RL2['cand'], _RL2['cnt'] = best.track_id, 1
  return best if _RL2['cnt'] >= RL_CONFIRM else None


'''
ANCHORS = [
  ("    self.path_x = np.arange(0.0, 160.0, 0.1)    # 160 meters is max\n",
   "    self.path_x = np.arange(0.0, 200.0, 0.1)    # RADARLEAD: 200 m (radar objects are tracked beyond 160 m)\n"),
  ("def get_lead(v_ego, ready, clusters, lead_msg, low_speed_override=True,use_tesla_radar=False):\n",
   HELPER + "def get_lead(v_ego, ready, clusters, lead_msg, low_speed_override=True,use_tesla_radar=False, radar_only=False):\n"),
  ("""  if cluster is not None:
    lead_dict,lead_dict_ext = cluster.get_RadarState(lead_msg.prob)
  elif (cluster is None) and ready and (lead_msg.prob > .5):
    lead_dict = Cluster(use_tesla_radar).get_RadarState_from_vision(lead_msg, v_ego)
""", """  # RADARLEAD - only with the Tesla radar integration: otherwise tracks have no path distance (dPath is 0 for all)
  rl = _radar_only_lead(v_ego, clusters, cluster) if (radar_only and use_tesla_radar and ready) else None
  if rl is not None and lead_msg.prob > .5 and rl.dRel > 1.25 * (lead_msg.dist - RADAR_TO_CAMERA) + 5.:
    rl = None                                  # the camera sees a car much closer than the radar-only one: keep that
  if cluster is not None:
    lead_dict,lead_dict_ext = cluster.get_RadarState(lead_msg.prob)
  elif rl is not None:
    lead_dict,lead_dict_ext = rl.get_RadarState()      # RADARLEAD: model probability 0 -> never an FCW
  elif (cluster is None) and ready and (lead_msg.prob > .5):
    lead_dict = Cluster(use_tesla_radar).get_RadarState_from_vision(lead_msg, v_ego)
"""),
  ("      l1d,l1x = get_lead(self.v_ego, self.ready, clusters, sm['model'].lead, low_speed_override=True,use_tesla_radar=self.use_tesla_radar)\n",
   "      l1d,l1x = get_lead(self.v_ego, self.ready, clusters, sm['model'].lead, low_speed_override=True,use_tesla_radar=self.use_tesla_radar, radar_only=True)\n"),
  ("      l2d,l2x = get_lead(self.v_ego, self.ready, clusters, sm['model'].leadFuture, low_speed_override=False, use_tesla_radar=self.use_tesla_radar)\n",
   "      l2d,l2x = get_lead(self.v_ego, self.ready, clusters, sm['model'].leadFuture, low_speed_override=False, use_tesla_radar=self.use_tesla_radar)\n"
   "      r2 = _radar_beyond_lead(self.v_ego, clusters, l1d) if (self.use_tesla_radar and self.ready) else None   # RADARLEAD\n"
   "      if r2 is not None:\n"
   "        l2d,l2x = r2.get_RadarState()          # leadTwo = the next car beyond the lead (second MPC plans for it too)\n"),
]


def patch(text):
  if MARK in text:
    return text, 'already patched'
  for old, _ in ANCHORS:
    if text.count(old) != 1:
      raise SystemExit('REFUSED: %r not found exactly once - not the expected radard.py; nothing changed' % old.strip()[:90])
  for old, new in ANCHORS:
    text = text.replace(old, new, 1)
  return text, 'patched'


def main(a):
  if not a:
    print(__doc__); return 2
  path, bak = a[0], a[0] + '.bak_radarlead'
  if '--restore' in a:
    shutil.copy2(bak, path); print('restored from', bak, '- turn the car off and on to load it'); return 0
  text = open(path).read()
  if '--check' in a:
    print('patched' if MARK in text else 'not patched'); return 0
  new, msg = patch(text)
  if msg == 'already patched':
    print(msg); return 0
  if '--apply' not in a:
    print(''.join(difflib.unified_diff(text.splitlines(True), new.splitlines(True), 'before', 'after', n=1)))
    print('dry run only; add --apply'); return 0
  shutil.copy2(path, bak)
  open(path, 'w').write(new)
  try:
    py_compile.compile(path, doraise=True)
  except py_compile.PyCompileError as e:
    shutil.copy2(bak, path); print('compile failed, original restored:', e); return 1
  print('patched; backup at', bak, '- turn the car off and on to load it'); return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
