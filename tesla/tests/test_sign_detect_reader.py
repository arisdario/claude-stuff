#!/usr/bin/env python3
"""Tests for sign_detect_reader.py (no cereal/car/camera needed):
    python3 -m unittest tesla/tests/test_sign_detect_reader.py
"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import sign_detect_reader as SD  # noqa: E402


def write(path, obj):
  with open(path, 'w') as f:
    json.dump(obj, f)


class TestReadDetection(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.path = os.path.join(self.d, 'sign_detect.json')

  def test_missing_file_is_empty(self):
    self.assertEqual(SD.read_detection(self.path, now=1000.0), SD.EMPTY)

  def test_malformed_json_is_empty(self):
    with open(self.path, 'w') as f:
      f.write('not json{{{')
    self.assertEqual(SD.read_detection(self.path, now=1000.0), SD.EMPTY)

  def test_fresh_detection_is_read(self):
    write(self.path, {'t': 1000.0, 'stop_sign': True, 'stop_sign_dist_m': 12.5, 'stop_sign_confidence': 0.8,
                       'light_present': False, 'light_color': -1, 'light_confidence': 0.0})
    d = SD.read_detection(self.path, now=1000.3)
    self.assertTrue(d['stop_sign']); self.assertEqual(d['stop_sign_dist_m'], 12.5)

  def test_stale_detection_is_empty(self):
    write(self.path, {'t': 1000.0, 'stop_sign': True, 'stop_sign_dist_m': 12.5, 'stop_sign_confidence': 0.8})
    d = SD.read_detection(self.path, now=1002.0, max_age_s=1.0)   # 2s old, max 1s
    self.assertEqual(d, SD.EMPTY)

  def test_future_timestamp_is_empty(self):
    # a clock jump backward (or a bogus future 't') should not be trusted as "fresh"
    write(self.path, {'t': 2000.0, 'stop_sign': True})
    d = SD.read_detection(self.path, now=1000.0)
    self.assertEqual(d, SD.EMPTY)

  def test_missing_t_field_is_empty(self):
    write(self.path, {'stop_sign': True, 'stop_sign_dist_m': 5.0})
    self.assertEqual(SD.read_detection(self.path, now=1000.0), SD.EMPTY)

  def test_missing_optional_fields_default_sensibly(self):
    write(self.path, {'t': 1000.0})   # minimal, no stop_sign/light_* keys at all
    d = SD.read_detection(self.path, now=1000.1)
    self.assertFalse(d['stop_sign']); self.assertFalse(d['light_present'])


class TestPickIcSign(unittest.TestCase):
  def test_no_stop_sign_is_none(self):
    d = dict(SD.EMPTY)
    self.assertIsNone(SD.pick_ic_sign(d))

  def test_confident_stop_sign_returns_a_tuple(self):
    d = {'stop_sign': True, 'stop_sign_dist_m': 15.0, 'stop_sign_confidence': 0.9,
         'light_present': False, 'light_color': None}
    r = SD.pick_ic_sign(d)
    self.assertEqual(r, (SD.ROAD_SIGN_TYPE_STOP, 15.0, 0, 0))

  def test_low_confidence_is_rejected(self):
    d = {'stop_sign': True, 'stop_sign_dist_m': 15.0, 'stop_sign_confidence': 0.1}
    self.assertIsNone(SD.pick_ic_sign(d))

  def test_missing_distance_is_rejected(self):
    d = {'stop_sign': True, 'stop_sign_dist_m': None, 'stop_sign_confidence': 0.9}
    self.assertIsNone(SD.pick_ic_sign(d))

  def test_negative_distance_is_rejected(self):
    d = {'stop_sign': True, 'stop_sign_dist_m': -1.0, 'stop_sign_confidence': 0.9}
    self.assertIsNone(SD.pick_ic_sign(d))

  def test_light_only_never_returns_a_sign(self):
    # see module docstring: lights have no reliable distance estimate, so never sent to the IC
    d = {'stop_sign': False, 'light_present': True, 'light_color': 0, 'light_confidence': 0.9}
    self.assertIsNone(SD.pick_ic_sign(d))

  def test_custom_min_confidence_threshold(self):
    d = {'stop_sign': True, 'stop_sign_dist_m': 10.0, 'stop_sign_confidence': 0.6}
    self.assertIsNotNone(SD.pick_ic_sign(d, min_confidence=0.5))
    self.assertIsNone(SD.pick_ic_sign(d, min_confidence=0.7))


if __name__ == '__main__':
  unittest.main()
