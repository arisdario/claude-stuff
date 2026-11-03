"""The Python port (tesla/nav/curvecall_engine.py) must reproduce the ORIGINAL CurveCue engine (data/curvecall_engine.js = web-demo/engine.js, MIT) on real routes.
Golden files were produced by running the original engine under node (data/run_golden.mjs); regenerate:  node run_golden.mjs route_bf.json gold_bf.json"""
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from nav import curvecall_engine as E  # noqa: E402


class TestPortMatchesOriginalEngine(unittest.TestCase):
    def check(self, name):
        pts = [tuple(p) for p in json.load(open(os.path.join(HERE, 'data', 'route_%s.json' % name)))]
        gold = json.load(open(os.path.join(HERE, 'data', 'gold_%s.json' % name)))['curves']
        mine = E.analyze_route(pts)
        self.assertEqual(len(mine), len(gold))
        for a, b in zip(mine, gold):
            self.assertEqual((a['severity'], a['direction'], a['start_index'], a['end_index']), (b['severity'], b['direction'], b['startIndex'], b['endIndex']))
            self.assertAlmostEqual(a['min_radius'], b['minRadius'], delta=1e-5 * max(1.0, b['minRadius']))
            self.assertAlmostEqual(a['distance_from_start'], b['distanceFromStart'], delta=1e-4)
            self.assertEqual(a['advisory_speed_kmh'], b['advisorySpeedKmh'])
            self.assertEqual(a['modifiers'], b['modifiers'])

    def test_bucuresti_to_fundulea(self):
        self.check('bf')

    def test_fundulea_to_bucuresti(self):
        self.check('fb')


if __name__ == '__main__':
    unittest.main()
