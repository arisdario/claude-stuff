import os, subprocess, sys, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import cpu_layout as C

LOADS = {'controlsd': 61, 'modeld': 62, 'camerad': 45, 'plannerd': 16, 'radard': 14, 'boardd': 3}   # measured on the EON with the C920 (MJPEG), percent of one core


class T(unittest.TestCase):
  def test_stock083_overloads_core2_with_a_c920(self):
    core = C.project(LOADS, C.LAYOUTS['stock083'], other=60)
    self.assertGreater(core[2], C.OVERLOAD)
    self.assertEqual(core[2] - 60 / 4.0, 62 + 45 + 16 + 14)

  def test_adapted_stays_under_the_limit(self):
    core = C.project(LOADS, C.LAYOUTS['adapted'], other=60)
    self.assertTrue(all(c < C.OVERLOAD for c in core), core)
    self.assertGreaterEqual(core[2], 62); self.assertGreaterEqual(core[3], 61 + 3)

  def test_totals_are_conserved(self):
    for name in C.LAYOUTS:
      self.assertAlmostEqual(sum(C.project(LOADS, C.LAYOUTS[name], other=60)), sum(LOADS.values()) + 60, places=6)

  def test_reset_floats_everything(self):
    core = C.project(LOADS, C.LAYOUTS['reset'], other=0)
    self.assertLess(max(core) - min(core), 62)                    # nothing pinned: no single core takes everything


IRQ = """           CPU0       CPU1       CPU2       CPU3
 121:          0          0          0          0  qpnp-int 4250  usbin-src-det
 193:       5093          0          0          0     GICv3 332  kgsl-3d0
 736:      80536          0          0       8927     GICv3 163  dwc3, xhci-hcd:usb1
 IPI0:       100        100        100        100  Rescheduling interrupts
"""


class Discover(unittest.TestCase):
  def test_parse_interrupts(self):
    r = C.parse_interrupts(IRQ)
    self.assertEqual([x['irq'] for x in r], [121, 193, 736])               # IPI lines are not numbered IRQs
    self.assertEqual(r[2]['counts'], [80536, 0, 0, 8927]); self.assertIn('xhci', r[2]['name'])

  def test_parse_cpulist(self):
    self.assertEqual(C.parse_cpulist('0-1,3'), [0, 1, 3]); self.assertEqual(C.parse_cpulist('2'), [2])


class Recommend(unittest.TestCase):
  def test_recommends_no_change_when_nothing_beats_floating(self):
    v, w, rows = C.recommend({'modeld': 30, 'controlsd': 30, 'camerad': 10}, 20, [3, 0, 0, 0])
    self.assertIn('NO CHANGE', v)

  def test_flags_overload_and_c920_warning(self):
    v, w, rows = C.recommend(LOADS, 60, [11, 0, 0, 0])
    s = '\n'.join(w)
    self.assertIn('stock083 would overload', s); self.assertIn('software JPEG decode', s)
    e = dict(rows)['stock083: modeld, camerad, plannerd, radard->2; controlsd, boardd->3']
    self.assertEqual(e['overloaded'], [2]); self.assertLess(e['headroom'], 0)

  def test_irq_overhead_is_added_per_core(self):
    a = C.evaluate(LOADS, C.LAYOUTS['adapted'], 0, [0, 0, 0, 0]); b = C.evaluate(LOADS, C.LAYOUTS['adapted'], 0, [0, 0, 0, 20])
    self.assertAlmostEqual(b['cores'][3] - a['cores'][3], 20)


class Rollback(unittest.TestCase):
  @unittest.skipIf(len(os.sched_getaffinity(0)) < 2, 'needs at least 2 cpus')
  def test_snapshot_apply_restore_puts_back_the_original(self):
    p = subprocess.Popen(['sleep', '30'])
    try:
      before = sorted(os.sched_getaffinity(p.pid))
      snap = C.snapshot({'modeld': p.pid})
      os.sched_setaffinity(p.pid, {before[0]})
      self.assertEqual(sorted(os.sched_getaffinity(p.pid)), [before[0]])
      ok, bad = C.restore(snap)
      self.assertEqual(sorted(os.sched_getaffinity(p.pid)), before); self.assertEqual((ok, bad), (1, 0))
    finally:
      p.kill(); p.wait()

  def test_restore_ignores_gone_processes(self):
    self.assertEqual(C.restore({'99999999': {'name': 'x', 'threads': {'99999999': [0, 1]}}}), (0, 1))


class Watch(unittest.TestCase):
  def run_watch(self, alive, busy, secs=20):
    return C.watch(alive, busy, secs, interval=1, sleep=lambda s: None)

  def test_clean(self):
    self.assertIsNone(self.run_watch(lambda: None, lambda: [60, 70, 80, 90]))

  def test_process_exit_rolls_back_at_once(self):
    self.assertIn('modeld exited', self.run_watch(lambda: 'modeld', lambda: [10] * 4))

  def test_sustained_overload_rolls_back_but_a_spike_does_not(self):
    seq = iter([[99, 10, 10, 10], [10, 10, 10, 10]] * 20)
    self.assertIsNone(self.run_watch(lambda: None, lambda: next(seq)))                        # one hot sample, then cool: streak resets
    self.assertIn('above 95%', self.run_watch(lambda: None, lambda: [10, 10, 97, 10]))        # three hot samples in a row


if __name__ == '__main__':
  unittest.main()
