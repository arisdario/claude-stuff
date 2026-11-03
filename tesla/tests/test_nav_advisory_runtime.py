"""Gate for the curve advisory: shadow by default, notification only when enabled and every check passes. Advisory only."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from nav import advisory_runtime as AR  # noqa: E402


def msg(conf='high', dist=150.0, curve_id=1, sev='sharp', text='Sharp left ahead, slow to 30', age=1.0, vc=9.0):
    return {'route_progress_m': 1000.0, 'announce': {'active': [{'id': curve_id, 'distance_m': dist, 'speed_ms': vc, 'severity': sev, 'text': text}]},
            'advisory': {'confidence': conf, 'target_speed_ms': 20.0, 'position_age_s': age}}


class Base(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.off, self.en, self.log = (os.path.join(self.d, n) for n in ('off', 'enable', 'log'))
        self.a = AR.CurveAdvisory(self.off, self.en, self.log)

    def touch(self, p):
        open(p, 'w').write('x')


class TestModes(Base):
    def test_default_is_shadow_and_never_notifies_but_logs(self):
        r = self.a.step(msg(), 20.0)
        self.assertEqual((r['mode'], r['notify']), ('shadow', None))
        rec = json.loads(open(self.log).read().splitlines()[-1])
        self.assertEqual(rec['would_notify'], 'Sharp left ahead, slow to 30')
        self.assertIsNone(rec['notified'])

    def test_enable_file_turns_notifications_on(self):
        self.touch(self.en)
        r = self.a.step(msg(), 20.0)
        self.assertEqual(r['mode'], 'enabled')
        self.assertEqual(r['notify']['text'], 'Sharp left ahead, slow to 30')
        self.assertEqual(r['notify']['level'], 'warn')

    def test_off_file_overrides_enable_and_logs_nothing(self):
        self.touch(self.en)
        self.touch(self.off)
        r = self.a.step(msg(), 20.0)
        self.assertEqual((r['mode'], r['notify']), ('off', None))
        self.assertFalse(os.path.exists(self.log))


class TestGating(Base):
    def setUp(self):
        Base.setUp(self)
        self.touch(self.en)

    def test_each_curve_is_notified_once(self):
        self.assertIsNotNone(self.a.step(msg(), 20.0)['notify'])
        self.assertIsNone(self.a.step(msg(dist=120.0), 20.0)['notify'])
        self.assertIsNotNone(self.a.step(msg(curve_id=2), 20.0)['notify'])

    def test_low_confidence_never_notifies(self):
        for c in ('low', 'none'):
            self.assertIsNone(self.a.step(msg(conf=c), 20.0)['notify'])

    def test_stale_gps_slow_speed_and_far_curves_do_not_notify(self):
        self.assertIsNone(self.a.step(msg(age=45.0), 20.0)['notify'])
        self.assertIsNone(self.a.step(msg(), 3.0)['notify'])
        self.assertIsNone(self.a.step(msg(dist=780.0), 20.0)['notify'])           # 780 m away at 20 m/s: well outside braking + reaction distance

    def test_notification_distance_grows_with_speed(self):
        self.assertIsNone(self.a.step(msg(dist=400.0, curve_id=7), 15.0)['notify'])
        self.assertIsNotNone(self.a.step(msg(dist=400.0, curve_id=8), 30.0)['notify'])

    def test_missing_switch_files_directory_is_safe(self):
        a = AR.CurveAdvisory('/nonexistent/off', '/nonexistent/enable', None)
        self.assertEqual(a.step(msg(), 20.0)['mode'], 'shadow')


if __name__ == '__main__':
    unittest.main()
