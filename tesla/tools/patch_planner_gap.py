#!/usr/bin/env python3
"""patch_planner_gap.py - make openpilot's longitudinal planner (0.7.5 MPC) keep YOUR following gap, not its fixed 1.8 s.

Why: ACC_module (ACCPLAN) now follows openpilot's own plan. The 0.7.5 MPC has the gap compiled in: 4 m + 1.8 s. Your logs
(15 Nov) show ~2.5 s in town and ~3.6 s on the highway (median), so the plan alone would follow ~45 % closer than you do.

What the patch changes in selfdrive/controls/lib/planner.py (nothing else):
  * adds a small helper (_GapLead) before 'class Planner'
  * the two MPC calls get the lead through it:  self.mpc1.update(pm, sm['carState'], _GapLead(lead_1, v_ego), ...)
    _GapLead shows the MPC a shorter distance, so its 1.8 s equals your gap (2.5 s up to 50 km/h, 3.6 s from 90 km/h,
    in between linear). Above your gap the distance is shifted by (gap - 1.8 s) x speed; below it scaled by
    (4 + 1.8 v) / (4 + gap v) - both are equal at your gap, and the distance the MPC sees never becomes negative.
  * radarState, the FCW distance / speed checks and everything else keep the REAL distance.
Risk: FCW's 'how hard does the plan brake' test uses mpc1's solution, which brakes earlier with the shorter distance. FCW
also needs a real time-to-collision under 2.5 s, so it can fire somewhat earlier, not without a real closing car.

  python3 patch_planner_gap.py /data/openpilot/selfdrive/controls/lib/planner.py            # dry run: shows the diff
  python3 patch_planner_gap.py FILE --apply     # backup FILE.bak_gap, patch, py_compile (restores itself on failure)
  python3 patch_planner_gap.py FILE --check     # says whether FILE is patched
  python3 patch_planner_gap.py FILE --restore   # puts FILE.bak_gap back
Refuses (changes nothing) if the planner does not look exactly like the expected 0.7.5 code.
"""
import difflib
import py_compile
import shutil
import sys

MARK = '# GAPSHIFT (tesla/tools/patch_planner_gap.py)'
HELPER = MARK + ''': the MPC is shown a distance at which its fixed 4 m + 1.8 s equals the driver's gap
GAP_TOWN_S, GAP_HWY_S, GAP_TOWN_KPH, GAP_HWY_KPH, MPC_GAP_S = 2.5, 3.6, 50., 90., 1.8


def _gap_s(v_ego):
  kph = v_ego * 3.6
  if kph <= GAP_TOWN_KPH:
    return GAP_TOWN_S
  if kph >= GAP_HWY_KPH:
    return GAP_HWY_S
  return GAP_TOWN_S + (GAP_HWY_S - GAP_TOWN_S) * (kph - GAP_TOWN_KPH) / (GAP_HWY_KPH - GAP_TOWN_KPH)


def _gap_distance(d, v_ego):
  v = max(0., v_ego)
  g = _gap_s(v)
  if d <= 0. or g <= MPC_GAP_S:
    return d
  return max(d - (g - MPC_GAP_S) * v, d * (4. + MPC_GAP_S * v) / (4. + g * v))


class _GapLead(object):
  """the radar lead with only dRel changed (for the MPC); every other field is the lead's own"""
  def __init__(self, lead, v_ego):
    self._lead = lead
    self.dRel = _gap_distance(lead.dRel, v_ego)

  def __getattr__(self, name):
    return getattr(self._lead, name)


'''
ANCHORS = [
  ("    self.mpc1.update(pm, sm['carState'], lead_1, v_cruise_setpoint)\n",
   "    self.mpc1.update(pm, sm['carState'], _GapLead(lead_1, v_ego), v_cruise_setpoint)   # GAPSHIFT\n"),
  ("    self.mpc2.update(pm, sm['carState'], lead_2, v_cruise_setpoint)\n",
   "    self.mpc2.update(pm, sm['carState'], _GapLead(lead_2, v_ego), v_cruise_setpoint)   # GAPSHIFT\n"),
  ("\nclass Planner():\n", "\n" + HELPER + "class Planner():\n"),
]
NEEDED = ["    v_ego = sm['carState'].vEgo\n", "    lead_1 = sm['radarState'].leadOne\n", "    lead_2 = sm['radarState'].leadTwo\n"]


def patch(text):
  if MARK in text:
    return text, 'already patched'
  for old, _ in ANCHORS:
    if text.count(old) != 1:
      raise SystemExit('REFUSED: %r not found exactly once - not the expected 0.7.5 planner; nothing changed' % old.strip())
  for line in NEEDED:
    if line not in text:
      raise SystemExit('REFUSED: %r missing - not the expected 0.7.5 planner; nothing changed' % line.strip())
  for old, new in ANCHORS:
    text = text.replace(old, new, 1)
  return text, 'patched'


def main(a):
  if not a:
    print(__doc__); return 2
  path, bak = a[0], a[0] + '.bak_gap'
  if '--restore' in a:
    shutil.copy2(bak, path); print('restored from', bak, '- reboot to load it'); return 0
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
  print('patched; backup at', bak, '- reboot to load it'); return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
