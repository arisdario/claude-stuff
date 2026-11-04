"""Resource sampling on a fake /proc and /sys tree (no EON needed)."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'traffic'))
import benchmark as B  # noqa: E402
import sysstats as S  # noqa: E402

STAT1 = "cpu  100 0 100 800 0 0 0 0\ncpu0 50 0 50 400 0 0 0 0\ncpu1 50 0 50 400 0 0 0 0\nintr 1\n"
STAT2 = "cpu  200 0 200 1000 0 0 0 0\ncpu0 150 0 150 500 0 0 0 0\ncpu1 50 0 50 500 0 0 0 0\nintr 1\n"


def w(path, text):
  os.makedirs(os.path.dirname(path), exist_ok=True)
  open(path, 'w').write(text)


class TestParsers(unittest.TestCase):
  def test_proc_stat_and_pct(self):
    t1, c1 = S.parse_proc_stat(STAT1)
    t2, c2 = S.parse_proc_stat(STAT2)
    self.assertAlmostEqual(S.pct(t1, t2), 100.0 * 200 / 400)
    self.assertAlmostEqual(S.pct(c1[0], c2[0]), 100.0 * 200 / 300)
    self.assertAlmostEqual(S.pct(c1[1], c2[1]), 0.0 + 100.0 * 0 / 100)      # idle core: 0 busy of 100
    self.assertIsNone(S.pct(c1[0], c1[0]))                                  # no time passed

  def test_meminfo(self):
    m = S.parse_meminfo('MemTotal: 2048000 kB\nMemFree: 100000 kB\nMemAvailable: 512000 kB\n')
    self.assertEqual(m['total_mb'], 2000.0)
    self.assertEqual(m['avail_mb'], 500.0)
    m = S.parse_meminfo('MemTotal: 1024 kB\nMemFree: 100 kB\nBuffers: 50 kB\nCached: 150 kB\n')
    self.assertAlmostEqual(m['avail_mb'], 300 / 1024.0)

  def test_pid_stat_with_awkward_comm(self):
    st = '123 (my (odd) proc) S 1 1 1 0 -1 4194560 100 0 0 0 40 25 0 0 20 0 1 0 1 1 1'
    self.assertEqual(S.parse_pid_stat(st), ('my (odd) proc', 65))
    self.assertEqual(S.parse_pid_stat('garbage'), (None, None))

  def test_labels(self):
    self.assertEqual(S.proc_label('python3\0/data/radarweb.py\0' + '8088\0', 'python3'), 'radarweb.py')
    self.assertEqual(S.proc_label('./modeld\0', 'modeld'), 'modeld')
    self.assertEqual(S.proc_label('', 'kworker'), 'kworker')

  def test_thermal_units_and_duplicates(self):
    d = tempfile.mkdtemp()
    w(os.path.join(d, 'thermal_zone0/type'), 'cpu\n'); w(os.path.join(d, 'thermal_zone0/temp'), '61500\n')
    w(os.path.join(d, 'thermal_zone1/type'), 'cpu\n'); w(os.path.join(d, 'thermal_zone1/temp'), '48\n')
    w(os.path.join(d, 'thermal_zone2/type'), 'bad\n'); w(os.path.join(d, 'thermal_zone2/temp'), '-273000\n')
    t = S.read_thermal(d)
    self.assertEqual(t['cpu'], 61.5)
    self.assertEqual(t['cpu#1'], 48.0)
    self.assertNotIn('bad', t)

  def test_qualcomm_tsens_are_tenths_of_a_degree(self):
    d = tempfile.mkdtemp()
    w(os.path.join(d, 'thermal_zone0/type'), 'tsens_tz_sensor9\n'); w(os.path.join(d, 'thermal_zone0/temp'), '699\n')
    w(os.path.join(d, 'thermal_zone1/type'), 'msm_therm\n'); w(os.path.join(d, 'thermal_zone1/temp'), '53\n')
    w(os.path.join(d, 'thermal_zone2/type'), 'pm8994_tz\n'); w(os.path.join(d, 'thermal_zone2/temp'), '51447\n')
    t = S.read_thermal(d)
    self.assertAlmostEqual(t['tsens_tz_sensor9'], 69.9)
    self.assertEqual(t['msm_therm'], 53.0)
    self.assertAlmostEqual(t['pm8994_tz'], 51.447)

  def test_gpu(self):
    d = tempfile.mkdtemp()
    w(os.path.join(d, 'gpubusy'), '30 120\n'); w(os.path.join(d, 'devfreq/cur_freq'), '510000000\n')
    self.assertEqual(S.read_gpu(d), (25.0, 510.0))
    self.assertEqual(S.read_gpu(os.path.join(d, 'nothing')), (None, None))


class TestSampler(unittest.TestCase):
  def test_two_samples_on_a_fake_tree(self):
    proc, sys_ = tempfile.mkdtemp(), tempfile.mkdtemp()
    w(os.path.join(proc, 'stat'), STAT1)
    w(os.path.join(proc, 'meminfo'), 'MemTotal: 2048000 kB\nMemAvailable: 1024000 kB\n')
    pid = str(os.getpid())
    for p, comm, cmd, ticks in ((pid, 'python3', 'python3\0benchmark.py\0', 10), ('77', 'modeld', './modeld\0', 100)):
      w(os.path.join(proc, p, 'stat'), '%s (%s) S 1 1 1 0 -1 0 0 0 0 0 %d 0 0 0 20 0 1 0 1 1 1' % (p, comm, ticks))
      w(os.path.join(proc, p, 'cmdline'), cmd)
    w(os.path.join(sys_, 'class/thermal/thermal_zone0/type'), 'cpu\n'); w(os.path.join(sys_, 'class/thermal/thermal_zone0/temp'), '55000\n')
    s = S.Sampler(proc, sys_, own_pid=int(pid))
    self.assertIsNone(s.sample(now=100.0))
    w(os.path.join(proc, 'stat'), STAT2)
    w(os.path.join(proc, '77', 'stat'), '77 (modeld) S 1 1 1 0 -1 0 0 0 0 0 %d 0 0 0 20 0 1 0 1 1 1' % (100 + int(S.CLK_TCK * 0.5)))
    w(os.path.join(proc, pid, 'stat'), '%s (python3) S 1 1 1 0 -1 0 0 0 0 0 %d 0 0 0 20 0 1 0 1 1 1' % (pid, 10 + int(S.CLK_TCK * 0.1)))
    r = s.sample(now=101.0)
    self.assertAlmostEqual(r['cpu_total_pct'], 50.0)
    self.assertAlmostEqual(r['procs']['modeld'], 50.0, places=3)
    self.assertAlmostEqual(r['self_cpu_core_pct'], 10.0, places=3)
    self.assertAlmostEqual(r['others_cpu_pct'], 50.0 - 10.0 / 2, places=3)
    self.assertEqual(r['temp_max_c'], 55.0)
    self.assertEqual(r['mem_avail_mb'], 1000.0)


class TestSummary(unittest.TestCase):
  def rows(self, n, secs):
    return [{'t': i * secs / (n - 1), 'others_cpu_pct': 20.0 + (i % 5), 'cpu_total_pct': 25.0, 'temp_max_c': 55.0 + (i % 7), 'mem_avail_mb': 800.0, 'model_fps': 20.0,
             'procs_json': '{"modeld": 80.0}', 'thermal_status': 'green', 'model_gaps': 0.0} for i in range(n)]

  def test_short_run_is_flagged(self):
    r = B.summarize(self.rows(30, 29))
    self.assertTrue(r['warnings'])

  def test_long_run_gives_limits_from_the_data(self):
    r = B.summarize(self.rows(1200, 1199))
    self.assertFalse(r['warnings'])
    lim = r['suggested_limits']
    self.assertAlmostEqual(lim['others_cpu_limit_pct'], 24.0 + 5.0, places=1)
    self.assertAlmostEqual(lim['temp_limit_c'], 61.0 + 3.0, places=1)
    self.assertEqual(lim['mem_avail_min_mb'], 700.0)
    self.assertEqual(r['model_frame_gaps_total'], 0.0)


if __name__ == '__main__':
  unittest.main()
