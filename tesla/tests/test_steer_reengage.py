#!/usr/bin/env python3
"""Tests for carcontroller.py's STEER-REENGAGE hold-off (post-lane-change steady-in-lane check).

carcontroller.py itself needs the full openpilot environment to import (CANPacker, ALCA/PCC/ACC
modules, etc.), same situation as test_sign_detect_carcontroller_wiring.py - so this does NOT import
the module. Instead it extracts the actual inserted source block by text anchors and exec()s it
against fake state each step, so the tests run the REAL code (not a hand-copied reimplementation that
could silently drift from it) while only needing the handful of attributes that block touches.
"""
import os
import re
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
CC = open(os.path.join(ROOT, 'carcontroller.py')).read()

START = '    # STEER-REENGAGE (see the module-level comment above STEER_REENGAGE_SMALL_ANGLE_DEG): a human-'
END_MARKER = 'or self.steer_reengage_active)'


def extract_block():
  a = CC.index(START)
  b = CC.index(END_MARKER, a) + len(END_MARKER)
  block = CC[a:b]
  # dedent: every line in this block is indented 4 spaces (inside CarController.update())
  lines = block.splitlines()
  dedented = '\n'.join(l[4:] if l.startswith('    ') else l for l in lines)
  return dedented


def extract_constant(name):
  m = re.search(r'^%s\s*=\s*([0-9.]+)' % re.escape(name), CC, re.MULTILINE)
  assert m, '%s not found in carcontroller.py' % name
  return float(m.group(1))


BLOCK_SRC = extract_block()
SMALL_ANGLE_DEG = extract_constant('STEER_REENGAGE_SMALL_ANGLE_DEG')
STABLE_FRAMES = int(extract_constant('STEER_REENGAGE_STABLE_FRAMES'))


class FakeSelf:
  def __init__(self):
    self.steer_reengage_frame_end = -1
    self.steer_reengage_stable_since = None
    self.steer_reengage_active = False
    self.alca_enabled = False


class FakeUE:
  def __init__(self):
    self.calls = []

  def custom_alert_message(self, *args):
    self.calls.append(args)


class FakeCS:
  def __init__(self, steerReengageNumbPeriod=12.0):
    self.prev_turn_signal_stalk_state = 0
    self.turn_signal_stalk_state = 0
    self.steerReengageNumbPeriod = steerReengageNumbPeriod
    self.UE = FakeUE()


class FakeActuators:
  def __init__(self, steerAngle=0.0):
    self.steerAngle = steerAngle


class Sim:
  """Runs the real extracted carcontroller.py block, frame by frame."""

  def __init__(self, numb_period=12.0):
    self.s = FakeSelf()
    self.cs = FakeCS(numb_period)
    self.frame = 0

  def step(self, stalk_state, steer_angle, frames=1):
    """Advances `frames` ticks with the stalk held at stalk_state and a given commanded angle.
    Returns the final human_lane_changing value."""
    result = None
    for _ in range(frames):
      self.cs.prev_turn_signal_stalk_state = self.cs.turn_signal_stalk_state
      self.cs.turn_signal_stalk_state = stalk_state
      ns = {'self': self.s, 'CS': self.cs, 'actuators': FakeActuators(steer_angle), 'frame': self.frame,
            'STEER_REENGAGE_SMALL_ANGLE_DEG': SMALL_ANGLE_DEG, 'STEER_REENGAGE_STABLE_FRAMES': STABLE_FRAMES}
      exec(BLOCK_SRC, ns)
      result = ns['human_lane_changing']
      self.frame += 1
    return result


class TestExtractionSanity(unittest.TestCase):
  def test_block_was_actually_found_and_is_nonempty(self):
    self.assertIn('steer_reengage_frame_end', BLOCK_SRC)
    self.assertIn('human_lane_changing', BLOCK_SRC)

  def test_constants_were_actually_found(self):
    self.assertEqual(SMALL_ANGLE_DEG, 3.0)
    self.assertEqual(STABLE_FRAMES, 100)


class TestDuringTheSignaledLaneChange(unittest.TestCase):
  def test_blocked_while_stalk_is_active(self):
    sim = Sim()
    result = sim.step(stalk_state=1, steer_angle=0.0, frames=50)
    self.assertTrue(result)


class TestHoldAfterBlinkerOff(unittest.TestCase):
  def test_does_not_release_the_instant_blinker_goes_off_if_angle_is_not_small(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)   # signaling, still correcting into the lane
    result = sim.step(stalk_state=0, steer_angle=10.0, frames=1)   # blinker JUST went off, still drifting
    self.assertTrue(result, 'should still be held - commanded angle is not small yet')

  def test_releases_early_once_angle_has_been_small_for_the_stable_window(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=10.0, frames=5)   # blinker off, still correcting briefly
    result = sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES + 2)  # now steady and small
    self.assertFalse(result, 'should have released once steady for the stable window')

  def test_does_not_release_after_almost_but_not_quite_the_stable_window(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=10.0, frames=1)
    result = sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES - 1)
    self.assertTrue(result, 'one frame short of the stable window - must still be held')

  def test_a_brief_wobble_above_threshold_resets_the_stable_counter(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=10.0, frames=1)
    sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES - 5)  # almost steady long enough
    sim.step(stalk_state=0, steer_angle=10.0, frames=1)                  # one wobble - should reset the streak
    result = sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES - 5)  # same almost-there length again
    self.assertTrue(result, 'the wobble should have reset the stability streak, not been forgiven')

  def test_backstop_releases_even_if_never_steady(self):
    sim = Sim(numb_period=1.0)  # 1 second -> 100 frames, small enough to test quickly
    sim.step(stalk_state=1, steer_angle=10.0, frames=5)
    sim.step(stalk_state=0, steer_angle=10.0, frames=1)
    # keep wobbling (never steady) right up to and past the 1s backstop
    result = sim.step(stalk_state=0, steer_angle=10.0, frames=101)
    self.assertFalse(result, 'the backstop period must release control even if never measured as steady')

  def test_a_new_lane_change_restarts_the_hold_even_mid_countdown(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES - 10)  # almost released
    # driver signals again for a second lane change before settling
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    result = sim.step(stalk_state=0, steer_angle=10.0, frames=1)
    self.assertTrue(result, 'a fresh lane change must restart the hold, not inherit the old countdown')


class TestNoRecentLaneChangeIsUnaffected(unittest.TestCase):
  def test_never_signaled_means_never_held_by_this_mechanism(self):
    sim = Sim()
    result = sim.step(stalk_state=0, steer_angle=10.0, frames=500)
    self.assertFalse(result)


class TestResumeAlertFiresOnceOnRelease(unittest.TestCase):
  def test_alert_fires_exactly_once_when_control_hands_back(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=10.0, frames=5)
    sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES)   # releases partway through this
    sim.step(stalk_state=0, steer_angle=0.5, frames=50)              # stays released for a while longer
    self.assertEqual(len(sim.cs.UE.calls), 1)
    self.assertIn('Lane Centering Resumed', sim.cs.UE.calls[0])

  def test_no_alert_while_still_held(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=10.0, frames=20)  # still drifting - never releases
    self.assertEqual(sim.cs.UE.calls, [])

  def test_no_alert_on_a_fresh_lane_change_restart(self):
    sim = Sim()
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)
    sim.step(stalk_state=0, steer_angle=0.5, frames=STABLE_FRAMES + 2)  # releases, fires once
    sim.step(stalk_state=1, steer_angle=10.0, frames=10)             # signals again - restart, not a release
    self.assertEqual(len(sim.cs.UE.calls), 1, 'restarting the hold must not fire a second "resumed" alert')

  def test_no_alert_if_this_mechanism_never_held_anything(self):
    sim = Sim()
    sim.step(stalk_state=0, steer_angle=10.0, frames=50)
    self.assertEqual(sim.cs.UE.calls, [])


if __name__ == '__main__':
  unittest.main()
