#!/usr/bin/env python3
"""Tests for tools/gps_boot_time_sync.sh.

This can't exercise the real GPS chip or the real device's `date` binary (no hardware here), so it
stubs both DATE_CMD and PY_CMD out with small fake scripts and drives the real shell script's control
flow through its env-var overrides. What IS tested for real: the floor/ceiling sanity-check logic,
the already-plausible no-op path, the GPS-success path, the GPS-timeout path, the corrupt-lastgood-
file fallback, the clock-set-failure path, and the date-format fallback when `date -u -d` isn't
available. What is NOT verified here (needs the real device): that NEOS's actual `date` binary accepts
`-u -d "@epoch" +FORMAT` and `-u -s "SPEC"` the way assumed, and that gpsd actually publishes gpsNMEA.

    python3 -m unittest tesla/tests/test_gps_boot_time_sync.py
"""
import os
import stat
import subprocess
import tempfile
import textwrap
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'gps_boot_time_sync.sh')

FLOOR = 1000000          # arbitrary, just needs to be a fixed reference point for the tests
DAY = 86400


def _make_exe(path, body):
  with open(path, 'w') as f:
    f.write(body)
  os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
  return path


def _date_stub(d, now, support_dash_d=True):
  """Fake `date`: `+%s` prints `now`; `-u -d @E +FMT` formats E (if support_dash_d); `-u -s SPEC`
  records the call and succeeds. Pure python so we don't depend on the test host's own date flavor."""
  body = textwrap.dedent('''\
    #!/usr/bin/env python3
    import datetime, sys
    args = sys.argv[1:]
    if args == ['+%%s']:
      print(%d)
    elif args[:2] == ['-u', '-d'] and %s:
      epoch = int(args[2].lstrip('@'))
      print(datetime.datetime.utcfromtimestamp(epoch).strftime('%%Y-%%m-%%d %%H:%%M:%%S'))
    elif args[:2] == ['-u', '-d']:
      sys.exit(1)  # simulate a date binary with no -d support
    elif args[:2] == ['-u', '-s']:
      with open('%s/date_set_calls.log', 'a') as f:
        f.write(args[2] + '\\n')
      sys.exit(0)
    else:
      sys.exit(2)
  ''' % (now, support_dash_d, d))
  return _make_exe(os.path.join(d, 'fake_date.py'), body)


def _py_stub(d, gps_result):
  """Fake `python3`: handles the poll_gps_epoch heredoc call (argv[1] == '-') per gps_result
  ('TIMEOUT' or an epoch int), and the date-format fallback call (argv[1] == '-c')."""
  body = textwrap.dedent('''\
    #!/usr/bin/env python3
    import datetime, sys
    args = sys.argv[1:]
    if args and args[0] == '-':
      result = %r
      if result == 'TIMEOUT':
        sys.exit(1)
      print(int(result))
      sys.exit(0)
    elif args and args[0] == '-c':
      epoch = int(args[2])
      print(datetime.datetime.utcfromtimestamp(epoch).strftime('%%Y-%%m-%%d %%H:%%M:%%S'))
      sys.exit(0)
    else:
      sys.exit(2)
  ''' % (gps_result,))
  return _make_exe(os.path.join(d, 'fake_python3.py'), body)


class TestGpsBootTimeSync(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.lastgood = os.path.join(self.d, 'lastgood')

  def run_script(self, now, gps_result='TIMEOUT', support_dash_d=True, lastgood_contents=None):
    if lastgood_contents is not None:
      with open(self.lastgood, 'w') as f:
        f.write(lastgood_contents)
    env = dict(os.environ)
    env.update({
        'FLOOR_EPOCH_S': str(FLOOR),
        'LASTGOOD_FILE': self.lastgood,
        'TIMEOUT_S': '1',
        'MAX_SKEW_YEARS': '5',
        'DATE_CMD': _date_stub(self.d, now, support_dash_d),
        'PY_CMD': _py_stub(self.d, gps_result),
        'NOW_CMD': _date_stub(self.d, now, support_dash_d) + ' +%s',
    })
    return subprocess.run(['sh', SCRIPT], env=env, capture_output=True, text=True, timeout=15)

  def date_set_calls(self):
    p = os.path.join(self.d, 'date_set_calls.log')
    return open(p).read().splitlines() if os.path.exists(p) else []

  def test_already_plausible_is_a_noop(self):
    r = self.run_script(now=FLOOR + 100)
    self.assertEqual(r.returncode, 0)
    self.assertIn('already plausible', r.stdout)
    self.assertEqual(self.date_set_calls(), [])
    self.assertEqual(open(self.lastgood).read().strip(), str(FLOOR + 100))

  def test_reset_clock_is_set_from_gps(self):
    picked = FLOOR + 3 * DAY
    r = self.run_script(now=0, gps_result=picked)
    self.assertEqual(r.returncode, 0)
    self.assertIn('waiting up to', r.stdout)
    self.assertIn('system clock set', r.stdout)
    calls = self.date_set_calls()
    self.assertEqual(len(calls), 1)
    self.assertEqual(open(self.lastgood).read().strip(), str(picked))

  def test_gps_timeout_leaves_clock_untouched(self):
    r = self.run_script(now=0, gps_result='TIMEOUT')
    self.assertEqual(r.returncode, 1)
    self.assertIn('no usable GPS time', r.stdout)
    self.assertEqual(self.date_set_calls(), [])
    self.assertFalse(os.path.exists(self.lastgood))

  def test_corrupt_lastgood_file_falls_back_to_hardcoded_floor(self):
    r = self.run_script(now=FLOOR - 10, gps_result='TIMEOUT', lastgood_contents='not-a-number\n')
    # now (FLOOR-10) is below the hardcoded floor either way, so this should still try GPS,
    # not silently treat the corrupt file's garbage as a trustworthy floor and misbehave.
    self.assertEqual(r.returncode, 1)
    self.assertIn('waiting up to', r.stdout)

  def test_saved_lastgood_raises_the_floor_above_hardcoded_default(self):
    saved = FLOOR + 30 * DAY
    # "now" sits between the hardcoded floor and the saved lastgood - without the saved value
    # this would look plausible; the saved value should make it look reset instead.
    r = self.run_script(now=FLOOR + DAY, gps_result='TIMEOUT', lastgood_contents=str(saved))
    self.assertEqual(r.returncode, 1)
    self.assertIn('waiting up to', r.stdout)

  def test_gps_value_beyond_skew_ceiling_is_rejected_even_if_python_misbehaves(self):
    # bash's own is_sane_epoch is a second, independent check - simulate a misbehaving/compromised
    # python stub handing back something absurdly far in the future and confirm bash still rejects it.
    picked = FLOOR + 50 * 365 * DAY  # far past MAX_SKEW_YEARS=5
    r = self.run_script(now=0, gps_result=picked)
    self.assertEqual(r.returncode, 1)
    self.assertIn('no usable GPS time', r.stdout)
    self.assertEqual(self.date_set_calls(), [])

  def test_date_minus_d_unavailable_falls_back_to_python_formatting(self):
    picked = FLOOR + 3 * DAY
    r = self.run_script(now=0, gps_result=picked, support_dash_d=False)
    self.assertEqual(r.returncode, 0)
    self.assertEqual(len(self.date_set_calls()), 1)

  def test_clock_set_failure_does_not_update_lastgood(self):
    picked = FLOOR + 3 * DAY
    d2 = tempfile.mkdtemp()
    # a date stub whose -u -s call always fails (simulates "not root")
    failing_date = os.path.join(d2, 'failing_date.py')
    with open(failing_date, 'w') as f:
      f.write(textwrap.dedent('''\
        #!/usr/bin/env python3
        import datetime, sys
        args = sys.argv[1:]
        if args == ['+%s']:
          print(0)
        elif args[:2] == ['-u', '-d']:
          epoch = int(args[2].lstrip('@'))
          print(datetime.datetime.utcfromtimestamp(epoch).strftime('%Y-%m-%d %H:%M:%S'))
        elif args[:2] == ['-u', '-s']:
          sys.exit(1)
        else:
          sys.exit(2)
      '''))
    os.chmod(failing_date, os.stat(failing_date).st_mode | stat.S_IEXEC)
    env = dict(os.environ)
    env.update({
        'FLOOR_EPOCH_S': str(FLOOR),
        'LASTGOOD_FILE': self.lastgood,
        'TIMEOUT_S': '1',
        'MAX_SKEW_YEARS': '5',
        'DATE_CMD': failing_date,
        'PY_CMD': _py_stub(self.d, picked),
        'NOW_CMD': failing_date + ' +%s',
    })
    r = subprocess.run(['sh', SCRIPT], env=env, capture_output=True, text=True, timeout=15)
    self.assertEqual(r.returncode, 1)
    self.assertIn('failed', r.stdout)
    self.assertFalse(os.path.exists(self.lastgood))


if __name__ == '__main__':
  unittest.main()
