#!/usr/bin/env python3
"""can_capture.py analysis: finds a lamp bit that follows the lever with a delay, ignores noise and the lever itself."""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import can_capture as C  # noqa: E402


def lever_frame(t, lever, cnt):
  d = bytearray(8)
  d[2] = (lever & 3) << 2
  d[6] = (cnt % 16) << 4
  return (t, 0, 0x45, bytes(d))


class T(unittest.TestCase):
  def scenario(self):
    """10 Hz lever; normal 0-8 s, out 8-18 s, normal, flash 26-27 s. A lamp bit (bus 2, 0x3f5 byte 1 bit 4) follows
    'out' 0.3 s late; another bit (0x3f5 byte 1 bit 5) is set only during the flash; 0x100 is random noise."""
    rnd = random.Random(1)
    lev = lambda t: 1 if 8 <= t < 18 else (2 if 26 <= t < 27 else 0)
    frames, cnt = [], 0
    for i in range(400):
      t = i * 0.1
      frames.append(lever_frame(t, lev(t), cnt)); cnt += 1
      lamp = 1 if 8.3 <= t < 18.3 else 0
      flash = 1 if 26.3 <= t < 27.3 else 0
      frames.append((t + 0.01, 2, 0x3f5, bytes([0, (lamp << 4) | (flash << 5), 0, 0])))
      frames.append((t + 0.02, 0, 0x100, bytes([rnd.randrange(256) for _ in range(4)])))
      frames.append((t + 0.03, 1, 0x219, lever_frame(t, lev(t), cnt)[3]))      # panda's radar copy: ignored
    return frames

  def test_finds_the_lamp_and_flash_bits_only(self):
    lines = []
    found = C.analyse(self.scenario(), out=lines.append)
    keys = {(f[0], f[1]) for f in found}
    self.assertIn(('follows OUT (high on)', (2, 0x3f5, 1, 4)), keys)
    self.assertIn(('follows PULL (flash)', (2, 0x3f5, 1, 5)), keys)
    self.assertFalse([k for k in keys if k[1][1] in (0x100, 0x219, 0x45)])
    text = '\n'.join(lines)
    self.assertIn('10.0 messages/s', text)
    self.assertIn("counter steps {1: 399}", text)
    self.assertIn('OUT (high on) 10.0 s', text)

  def test_lever_decoding(self):
    self.assertEqual(C.lever_of(lever_frame(0, 1, 0)[3]), 1)
    self.assertEqual(C.lever_of(lever_frame(0, 2, 0)[3]), 2)
    self.assertIsNone(C.lever_of(b'\x00'))


if __name__ == '__main__':
  unittest.main()
