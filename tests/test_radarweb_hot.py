"""radarweb's HOT set (IDs handled on every frame): the radar object range and 0x30A must really be in it.
A comment once swallowed '| set(range(0x310, 0x36F))' and radar objects were handled only twice a second."""
import ast
import os
import unittest

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'radarweb.py')


def hot_set():
    tree = ast.parse(open(SRC).read())
    for n in tree.body:
        if isinstance(n, ast.Assign) and any(getattr(t, 'id', None) == 'HOT' for t in n.targets):
            return eval(compile(ast.Expression(n.value), 'HOT', 'eval'), {'set': set, 'range': range, 'MUXINFO': {0x999: 0}})
    raise AssertionError('HOT not found')


class T(unittest.TestCase):
    def test_radar_range_and_lights_are_hot(self):
        h = hot_set()
        for a in (0x310, 0x340, 0x36E, 0x370, 0x155, 0x30A, 0x999):
            self.assertIn(a, h, hex(a))


class Beam(unittest.TestCase):
    def test_event_sample_carries_high_beam_only_when_fresh(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(SRC)))
        import radarevents
        st = {'v': 20.0, 'beam': {'hi': 1, 'fl': 0, 't': 100.0}}
        self.assertEqual(radarevents.sample(st, 100.5).get('hb'), 1)
        self.assertNotIn('hb', radarevents.sample(st, 105.0))     # stale light status: unknown, not "off"

    def test_trip_high_beam_column(self):
        import json, sys, tempfile
        sys.path.insert(0, os.path.dirname(SRC))
        import radartrip
        fn = os.path.join(tempfile.mkdtemp(), 'map_20261101_120000.jsonl')
        with open(fn, 'w') as f:
            for i, b in enumerate(('80', '88')):
                f.write(json.dumps({'t': i, 'v': 10, 'raw': {'0x30a': b + ' 00'}, 'msgs': {'MCU_locationStatus': {'MCU_latitude': 52.0, 'MCU_longitude': 4.0}}}) + '\n')
        t = radartrip.trip(fn)
        i = t['cols']['points'].index('high_beam')
        self.assertEqual([p[i] for p in t['points']], [0, 1])


if __name__ == '__main__':
    unittest.main()
