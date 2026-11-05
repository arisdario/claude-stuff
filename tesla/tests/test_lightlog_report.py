#!/usr/bin/env python3
"""lightlog_report on synthetic logs with known answers.  python3 tesla/tests/test_lightlog_report.py"""
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'diag'))
import lightlog_report as R  # noqa: E402
import lightlog as L  # noqa: E402

HZ = 20.


def synth(delay=0.3, offset=1.0, wig_hz=0.4, wig_amp=3.0, n_s=120, sR=22.93, engaged=1):
  n = int(n_s * HZ); t = np.arange(n) / HZ + 100.
  rng = np.random.RandomState(1)
  cmd = wig_amp * np.sin(2 * np.pi * wig_hz * t) + 0.05 * rng.randn(n)
  k = int(round(delay * HZ))
  meas = np.concatenate([np.zeros(k), cmd[:n - k]]) + offset
  cols = {c: np.full(n, np.nan) for c in L.COLS}
  cols.update({'t': t, 'v': np.full(n, 25.), 'ang': meas, 'ang_cmd': cmd, 'yaw': meas * 0 + 0.0, 'pressed': np.zeros(n),
               'enabled': np.full(n, float(engaged)), 'active': np.full(n, float(engaged)), 'pp_ang': cmd, 'sR': np.full(n, sR),
               'sF': np.full(n, 0.504), 'ao': np.full(n, -0.3), 'lp_valid': np.ones(n)})
  return cols


class T(unittest.TestCase):
  def test_delay_wiggle_offset(self):
    d = synth()
    lag, c = R.xcorr_lag(d['ang_cmd'][:600], d['ang'][:600], HZ)
    self.assertAlmostEqual(lag, 0.3, delta=0.06); self.assertGreater(c, 0.9)
    rms, f, snr = R.band_stats(d['t'][:600], d['ang'][:600])
    self.assertAlmostEqual(f, 0.4, delta=0.07); self.assertGreater(snr, 4); self.assertGreater(rms, 1.5)

  def test_param_sets_and_report(self):
    a = synth(sR=22.93); b = synth(sR=13.85, wig_amp=0.3)
    d = {k: np.concatenate([a[k], b[k] + (np.arange(len(b[k])) * 0 + 500. if k == 't' else 0)]) for k in a}
    sets = R.param_sets(d)
    self.assertEqual([round(s[2][0], 2) for s in sets], [22.93, 13.85])
    out = []
    R.report(d, [], out=out.append)
    txt = '\n'.join(out)
    self.assertIn('gain=1.53', txt); self.assertIn('gain=0.92', txt)
    self.assertIn('DELAY', txt)

  def test_manual_shadow_and_ratio(self):
    d = synth(engaged=0, wig_amp=0.0)
    v = 20.; ang_deg = 30.; yaw = v * np.radians(ang_deg) / (R.WHEELBASE * 15.)
    d['v'][:] = v; d['ang'][:] = ang_deg; d['yaw'][:] = yaw
    r = R.ratio_check(d, np.arange(len(d['t'])))
    self.assertAlmostEqual(r[0], 15., delta=0.5)
    out = []; R.report(d, [], out=out.append)
    self.assertIn('SHADOW', '\n'.join(out))

  def test_load_roundtrip(self):
    d = synth(n_s=5)
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
      f.write('# LiveParameters {}\n' + ','.join(L.COLS) + '\n')
      for i in range(len(d['t'])):
        f.write(','.join('%.4f' % d[c][i] if d[c][i] == d[c][i] else '' for c in L.COLS) + '\n')
    try:
      got, meta = R.load([f.name])
      self.assertEqual(len(got['t']), len(d['t'])); self.assertTrue(meta[0].startswith('# LiveParameters'))
    finally:
      os.remove(f.name)


if __name__ == '__main__':
  unittest.main()
