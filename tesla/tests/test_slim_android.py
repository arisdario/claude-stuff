"""slim_android.sh with fake getprop/stop/free: dry run changes nothing; --yes stops only running services from the list; --telephony adds the phone daemons."""
import os, subprocess, tempfile, unittest
SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'slim_android.sh')


def run(args, running):
  d = tempfile.mkdtemp(prefix='slim_')
  log = os.path.join(d, 'calls')
  def w(name, body):
    p = os.path.join(d, name); open(p, 'w').write('#!/bin/sh\n' + body); os.chmod(p, 0o755)
  w('getprop', 'case "$1" in %s) echo running;; *) echo stopped;; esac\n' % '|'.join('init.svc.%s' % s for s in running))
  w('stop', 'echo "stop $1" >> "%s"\n' % log)
  w('free', 'echo "              total  used  free"; echo "Mem:   5740  1318  3192"\n')
  r = subprocess.run(['sh', SH] + args, env=dict(os.environ, PATH=d + ':' + os.environ['PATH']), stdout=subprocess.PIPE, universal_newlines=True)
  calls = open(log).read().split('\n') if os.path.exists(log) else []
  return r.stdout, [c for c in calls if c]


class T(unittest.TestCase):
  def test_dry_run_stops_nothing(self):
    out, calls = run([], ['qfp-daemon', 'perfprofd', 'ril-daemon'])
    self.assertEqual(calls, []); self.assertIn('would stop', out); self.assertIn('dry run only', out)

  def test_yes_stops_only_the_small_set_that_is_running(self):
    out, calls = run(['--yes'], ['qfp-daemon', 'perfprofd', 'ifaadaemon', 'ril-daemon', 'sshd'])
    self.assertEqual(sorted(calls), ['stop ifaadaemon', 'stop perfprofd', 'stop qfp-daemon'])      # drm was not running: skipped; ril and sshd untouched
    self.assertIn('drm: not running', out)

  def test_telephony_is_opt_in(self):
    out, calls = run(['--yes', '--telephony'], ['qfp-daemon', 'ril-daemon', 'ril-daemon2', 'imsqmidaemon', 'sshd'])
    self.assertEqual(sorted(calls), ['stop imsqmidaemon', 'stop qfp-daemon', 'stop ril-daemon', 'stop ril-daemon2']); self.assertIn('WARNING', out)

  def test_never_touches_what_the_car_needs(self):
    body = open(SH).read()
    for needed in ('sshd', 'surfaceflinger', 'cnss-daemon', 'sensors', 'audiod', 'adsprpcd', 'qseecomd', 'netd', 'healthd', 'logd', 'zygote', 'servicemanager'):
      self.assertNotIn(' ' + needed + ' ', body.split('SMALL=')[1].split('YES=')[0] + ' ')


if __name__ == '__main__':
  unittest.main()
