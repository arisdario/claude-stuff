#!/usr/bin/env python3
"""tools/pyprof.py: the CPU split between busy functions is right and waits are not counted; tools/patch_controlsd_profile.py
patches only controlsd's expected main(), once, and does nothing without the trigger file."""
import os, sys, tempfile, threading, time, unittest
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import pyprof  # noqa: E402
import patch_controlsd_profile as PP  # noqa: E402


def heavy():
  s = 0
  for i in range(30000):
    s += i * i
  return s


def light():
  return sum(range(3000))


def wait():
  time.sleep(0.004)


class T(unittest.TestCase):
  def test_split_and_waits(self):
    d = tempfile.mkdtemp(); out = os.path.join(d, 'r.txt'); trig = os.path.join(d, 'trig'); open(trig, 'w').close()
    th = pyprof.start(out, 2.0, trigger=trig)
    t0 = time.time()
    while time.time() - t0 < 2.4:
      heavy(); light(); wait()
    th.join()
    rep = open(out).read()
    self.assertTrue(os.path.exists(trig + '.done')); self.assertFalse(os.path.exists(trig))
    self_sec = rep.split('== SELF')[1].split('==')[0]
    first = [l for l in self_sec.splitlines() if '%' in l][0]
    self.assertIn('heavy', first)                              # the hot spot is ranked first
    self.assertNotIn(':wait', self_sec)                        # waiting is not counted as CPU

  def test_patch(self):
    src = "import os\n\ndef controlsd_thread(sm, pm, logcan):\n  pass\n\n\ndef main(sm=None, pm=None, logcan=None):\n  controlsd_thread(sm, pm, logcan)\n"
    new, msg = PP.patch(src); self.assertEqual(msg, 'patched'); compile(new, 'controlsd.py', 'exec')
    self.assertIn("os.path.exists('/data/profile_controlsd')", new); self.assertEqual(PP.patch(new)[1], 'already patched')
    with self.assertRaises(SystemExit):
      PP.patch(src.replace('controlsd_thread(sm, pm, logcan)\n', 'controlsd_thread(sm, pm)\n'))


if __name__ == '__main__':
  unittest.main()
