#!/usr/bin/env python3
"""check_pairing verdicts.  python3 tesla/tests/test_panda_pairing.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'panda'))
import check_pairing as C  # noqa: E402

OLD = """
const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {25., 25., 25.}};
const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {25., 26., 25.}};
int delta_angle_up = (int)(tesla_interpolate(TESLA_LOOKUP_ANGLE_RATE_UP, tesla_speed) * 25. + 1.);
"""
NEW = OLD + """
const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP_MSG = {
    {2., 7., 17.},
    {10., 8., 1.5}};
const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN_MSG = {
    {2., 7., 17.},
    {10., 11., 4.8}};
int delta_angle_up = (int)(tesla_interpolate(TESLA_LOOKUP_ANGLE_RATE_UP_MSG, tesla_speed) + 1.);
"""
TIGHT = NEW.replace('{10., 8., 1.5}', '{2., 1., 0.2}')


class T(unittest.TestCase):
  def test_unpatched(self):
    v, _ = C.analyse(OLD, 0.0); self.assertEqual(v, 'UNPATCHED')
    v, _ = C.analyse(OLD, 1.0); self.assertEqual(v, 'UNPATCHED')

  def test_paired(self):
    v, lines = C.analyse(NEW, 1.0); self.assertEqual(v, 'PAIRED', '\n'.join(lines))

  def test_patched_but_car_side_partial_or_off(self):
    self.assertEqual(C.analyse(NEW, 0.25)[0], 'WILL TRIP')
    self.assertEqual(C.analyse(NEW, 0.0)[0], 'WILL TRIP')

  def test_firmware_tighter_than_car(self):
    self.assertEqual(C.analyse(TIGHT, 1.0)[0], 'MISMATCH')

  def test_shipped_patch_is_paired(self):
    patch = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'panda', 'angle_rate_per_message.patch')).read()
    added = '\n'.join(l[1:] for l in patch.splitlines() if l.startswith('+') and not l.startswith('+++'))
    self.assertEqual(C.analyse(OLD + added, 1.0)[0], 'PAIRED')


if __name__ == '__main__':
  unittest.main()
