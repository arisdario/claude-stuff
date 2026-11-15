#!/usr/bin/env python3
"""tools/patch_radard_lead.py: patches only the expected Tinkla 0.7.5.a radard, once; radar-only lead / car-beyond logic:
moving + in path + confirmed 0.5 s, kept by track id, never stationary / oncoming / next lane / too close, camera wins."""
import os, sys, tempfile, unittest
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import patch_radard_lead as P  # noqa: E402
NS = {}; exec(P.HELPER, NS)
RADARD = ("import numpy as np\n"
          "class RadarD():\n  def __init__(self):\n"
          "    self.path_x = np.arange(0.0, 160.0, 0.1)    # 160 meters is max\n"
          "def get_lead(v_ego, ready, clusters, lead_msg, low_speed_override=True,use_tesla_radar=False):\n"
          "  cluster, lead_dict = None, {}\n"
          "  if cluster is not None:\n    lead_dict,lead_dict_ext = cluster.get_RadarState(lead_msg.prob)\n"
          "  elif (cluster is None) and ready and (lead_msg.prob > .5):\n"
          "    lead_dict = Cluster(use_tesla_radar).get_RadarState_from_vision(lead_msg, v_ego)\n"
          "  return lead_dict\n"
          "class Upd():\n  def update(self, sm, clusters):\n    if True:\n"
          "      l1d,l1x = get_lead(self.v_ego, self.ready, clusters, sm['model'].lead, low_speed_override=True,use_tesla_radar=self.use_tesla_radar)\n"
          "      l2d,l2x = get_lead(self.v_ego, self.ready, clusters, sm['model'].leadFuture, low_speed_override=False, use_tesla_radar=self.use_tesla_radar)\n")


class C(object):
  def __init__(self, tid, d, vrel, dpath, stationary=False, measured=True):
    self.track_id, self.dRel, self.vRel, self.dPath, self.stationary, self.measured = tid, d, vrel, dpath, stationary, measured


class T(unittest.TestCase):
  def setUp(self):
    NS['_RL'].update(id=None, cand=None, cnt=0); NS['_RL2'].update(cand=None, cnt=0)

  def lead(self, clusters, n=1, matched=None):
    r = None
    for _ in range(n):
      r = NS['_radar_only_lead'](36., clusters, matched)
    return r

  def test_patch_once_and_refuse_unknown(self):
    new, msg = P.patch(RADARD); self.assertEqual(msg, 'patched')
    self.assertIn('radar_only=True', new); self.assertIn('_radar_beyond_lead', new); self.assertIn('200.0', new)
    self.assertEqual(P.patch(new)[1], 'already patched')
    with self.assertRaises(SystemExit):
      P.patch(RADARD.replace('160.0', '150.0'))

  def test_apply_restore(self):
    d = tempfile.mkdtemp(); f = os.path.join(d, 'radard.py'); open(f, 'w').write(RADARD)
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
      self.assertEqual(P.main([f, '--apply']), 0); self.assertIn(P.MARK, open(f).read())
      self.assertEqual(P.main([f, '--restore']), 0)
    self.assertEqual(open(f).read(), RADARD)

  def test_radar_only_lead(self):
    far = C(7, 130., -2., 0.3)
    self.assertIsNone(self.lead([far]))                              # not yet confirmed
    self.assertIs(self.lead([far], n=10), far)                       # confirmed after 0.5 s
    drifted = C(7, 132., -2., 2.0); self.assertIs(self.lead([drifted]), drifted)   # kept up to 2.2 m from the path
    self.assertIsNone(self.lead([C(7, 133., -2., 3.0)]))             # left the lane

  def test_never_these(self):
    for c in (C(9, 140., -36., 0.1, stationary=True), C(10, 150., -60., 0.2), C(11, 120., 0., 3.5), C(12, 25., -1., 0.2),
              C(13, 120., 0., 0.2, measured=False)):
      self.setUp(); self.assertIsNone(self.lead([c], n=20), c.track_id)

  def test_kept_after_camera_match_and_closer_car_wins(self):
    m = C(7, 100., -2., 0.2)
    self.assertIsNone(self.lead([m], matched=m))                     # camera+radar: nothing to do, car remembered
    k = C(7, 130., -2., 0.4); self.assertIs(self.lead([k]), k)        # camera lost it: kept at once
    cut = C(15, 90., -3., 0.4)
    self.assertIs(self.lead([k, cut]), k)                            # a cut-in must be confirmed first
    self.assertIs(self.lead([k, cut], n=10), cut)

  def test_car_beyond_the_lead(self):
    L1 = {'status': True, 'dRel': 120.}
    b = NS['_radar_beyond_lead']
    slow = C(21, 170., -8.3, 0.4)
    for _ in range(9): self.assertIsNone(b(36., [C(20, 120., 0., 0.2), slow], L1))
    self.assertIs(b(36., [C(20, 120., 0., 0.2), slow], L1), slow)
    self.setUp()
    for c in (C(23, 170., -8.3, 3.4), C(24, 123., 0., 0.3)):
      r = None
      for _ in range(12): r = b(36., [C(20, 120., 0., 0.2), c], L1)
      self.assertIsNone(r)
    self.assertIsNone(b(36., [slow], {'status': False}))


if __name__ == '__main__':
  unittest.main()
