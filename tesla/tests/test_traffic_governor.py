"""Traffic-awareness governor: fail-safe behaviour (pure logic) and the isolation rule (no vehicle-control imports)."""
import ast
import glob
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'traffic'))
import governor as G  # noqa: E402

LIM = {'others_cpu_limit_pct': 40.0, 'own_cpu_limit_core_pct': 40.0, 'temp_limit_c': 70.0, 'mem_avail_min_mb': 200.0,
       'warmup_s': 5.0, 'sustain_s': 3.0, 'hold_s': 20.0, 'recover_s': 10.0}


def ok(**kw):
  s = {'others_cpu_pct': 30.0, 'self_cpu_core_pct': 10.0, 'temp_max_c': 60.0, 'mem_avail_mb': 900.0, 'thermal_status': 'green', 'model_fps': 20.0}
  s.update(kw)
  return s


def run(g, seconds, start, **kw):
  d = None
  for i in range(int(seconds)):
    d = g.update(ok(**kw), start + i)
  return d, start + seconds


class TestGovernor(unittest.TestCase):
  def test_refuses_to_exist_without_limits(self):
    for missing in G.REQUIRED:
      lim = dict(LIM)
      del lim[missing]
      with self.assertRaises(ValueError):
        G.Governor(lim)

  def test_off_until_warmed_up(self):
    g = G.Governor(LIM)
    d, t = run(g, 4, 0)
    self.assertFalse(d.allowed)
    d, t = run(g, 4, t)
    self.assertTrue(d.allowed)

  def test_switches(self):
    g = G.Governor(LIM)
    run(g, 10, 0)
    self.assertFalse(g.update(ok(), 11, switch_off=True).allowed)
    self.assertFalse(g.update(ok(), 12, switch_on=False).allowed)

  def test_missing_readings_fail_safe(self):
    g = G.Governor(LIM)
    for k in ('temp_max_c', 'others_cpu_pct', 'mem_avail_mb', 'model_fps'):
      g = G.Governor(LIM)
      run(g, 10, 0)
      s = ok()
      s[k] = None
      d = None
      for i in range(10):
        d = g.update(s, 100 + i)
      self.assertFalse(d.allowed, k)

  def test_thermal_status_is_instant(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    self.assertTrue(g.update(ok(), t).allowed)
    d = g.update(ok(thermal_status='yellow'), t + 1)
    self.assertFalse(d.allowed)

  def test_hard_temperature_is_instant(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    self.assertFalse(g.update(ok(temp_max_c=76.0), t).allowed)

  def test_soft_limits_need_to_persist(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    self.assertTrue(g.update(ok(others_cpu_pct=55.0), t).allowed)       # a one-second blip does not trip
    self.assertTrue(g.update(ok(), t + 1).allowed)
    d, t2 = run(g, 5, t + 2, others_cpu_pct=55.0)                       # sustained: trips
    self.assertFalse(d.allowed)

  def test_model_fps_drop_trips_on_the_mean(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    d, t = run(g, 8, t, model_fps=15.0)
    self.assertFalse(d.allowed)
    self.assertIn('driving model', g.last_trip)

  def test_one_jittery_second_does_not_trip(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    seq = [15.5, 23.9, 23.0, 17.0, 24.0, 22.5, 19.5, 20.5]       # bunched message counts like the real run; mean is about 20.7
    for i, f in enumerate(seq):
      d = g.update(ok(model_fps=f), t + i)
    self.assertTrue(d.allowed)

  def test_missed_model_frames_trip(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    d = None
    for i in range(8):
      d = g.update(ok(model_gaps=2), t + i)
    self.assertFalse(d.allowed)
    self.assertIn('missed', g.last_trip)

  def test_hold_then_recover_with_margin(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    run(g, 5, t, others_cpu_pct=60.0)                                   # trip at t+3
    d, t = run(g, 15, t + 5)                                            # healthy but still holding
    self.assertFalse(d.allowed)
    d, t = run(g, 30, t)
    self.assertTrue(d.allowed)

  def test_no_recovery_right_at_the_limit(self):
    g = G.Governor(LIM)
    _, t = run(g, 10, 0)
    run(g, 5, t, others_cpu_pct=60.0)
    d, t = run(g, 60, t + 5, others_cpu_pct=39.0)                       # under the limit but inside the 3-point margin
    self.assertFalse(d.allowed)

  def test_latches_after_repeated_trips(self):
    g = G.Governor(dict(LIM, hold_s=1.0, recover_s=1.0, warmup_s=1.0, max_trips_per_hour=2))
    t = 0
    for _ in range(4):
      _, t = run(g, 5, t)
      _, t = run(g, 5, t, others_cpu_pct=90.0)
    d, t = run(g, 100, t)
    self.assertFalse(d.allowed)
    self.assertTrue(g.latched)

  def test_never_imports_vehicle_control(self):
    bad = {'carcontroller', 'carstate', 'PCC_module', 'ACC_module', 'HSO_module', 'ALCA_module', 'AHB_module', 'selfdrive', 'teslacan', 'interface', 'tesla_csa'}
    for fn in glob.glob(os.path.join(HERE, '..', 'traffic', '*.py')):
      for node in ast.walk(ast.parse(open(fn).read())):
        names = [a.name.split('.')[0] for a in node.names] if isinstance(node, ast.Import) else [node.module.split('.')[0]] if isinstance(node, ast.ImportFrom) and node.module else []
        for n in names:
          # benchmark.py imports cereal.messaging lazily only to READ message rates; selfdrive/control modules are never allowed
          self.assertNotIn(n, bad, '%s imports %s' % (os.path.basename(fn), n))


if __name__ == '__main__':
  unittest.main()
