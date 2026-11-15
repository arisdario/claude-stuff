#!/usr/bin/env python3
"""tools/patch_planner_gap.py: patches only the expected 0.7.5 planner, once; the distance shown to the MPC equals the
driver's gap at equilibrium, is continuous and never negative; every other lead field passes through unchanged."""
import os, sys, tempfile, unittest
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import patch_planner_gap as P  # noqa: E402
G = {}; exec(P.HELPER, G)
PLANNER = ("import math\n\nclass Planner():\n  def update(self, sm, pm, CP, VM, PP):\n    v_ego = sm['carState'].vEgo\n"
           "    lead_1 = sm['radarState'].leadOne\n    lead_2 = sm['radarState'].leadTwo\n"
           "    self.mpc1.update(pm, sm['carState'], lead_1, v_cruise_setpoint)\n"
           "    self.mpc2.update(pm, sm['carState'], lead_2, v_cruise_setpoint)\n")

class T(unittest.TestCase):
  def test_patch_once_and_refuse_unknown(self):
    new, msg = P.patch(PLANNER); self.assertEqual(msg, 'patched')
    self.assertIn('_GapLead(lead_1, v_ego)', new); self.assertIn('_GapLead(lead_2, v_ego)', new)
    self.assertEqual(P.patch(new)[1], 'already patched')
    compile(new, 'planner.py', 'exec')
    with self.assertRaises(SystemExit):
      P.patch(PLANNER.replace('lead_1, v_cruise_setpoint', 'lead_1, v_cruise'))

  def test_apply_and_restore(self):
    d = tempfile.mkdtemp(); f = os.path.join(d, 'planner.py'); open(f, 'w').write(PLANNER)
    self.assertEqual(P.main([f, '--apply']), 0); self.assertIn(P.MARK, open(f).read())
    self.assertEqual(P.main([f, '--restore']), 0); self.assertEqual(open(f).read(), PLANNER)

  def test_gap_distance(self):
    gd, gs = G['_gap_distance'], G['_gap_s']
    self.assertEqual(gs(10.), 2.5); self.assertEqual(gs(30.), 3.6); self.assertAlmostEqual(gs(70 / 3.6), 3.05)
    for v in (5., 15., 20., 25., 33.):
      eq = 4. + gs(v) * v                                           # the driver's gap ...
      self.assertAlmostEqual(gd(eq, v), 4. + 1.8 * v, places=6)     # ... is the MPC's own 4 m + 1.8 s
      prev = None
      for d in [x * 0.5 for x in range(1, 400)]:
        y = gd(d, v); self.assertGreater(y, 0.)
        if prev is not None:
          self.assertGreaterEqual(y, prev)                          # monotonic: closer stays closer
          self.assertLess(y - prev, 0.5 + 1e-9)                     # continuous, no jump (the MPC re-inits above 2.5 m)
        prev = y
    lead = type('L', (), {'dRel': 80., 'vLead': 20., 'aLeadK': -0.3, 'status': True})()
    g = G['_GapLead'](lead, 25.)
    self.assertLess(g.dRel, 80.); self.assertEqual((g.vLead, g.aLeadK, g.status), (20., -0.3, True))

if __name__ == '__main__':
  unittest.main()
