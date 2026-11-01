"""canwatch: finds bits that follow ON/OFF phases, ignores counters, noise, the panda's own frames and settle time"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import canwatch as cw  # noqa: E402


def run(phases=6, seed=1):
  rnd = random.Random(seed); w = cw.Watch(); t = 0.0; cnt = 0
  for p in range(phases):
    if p:
      w.toggle(t)
    on = w.segments[-1]['on']
    for i in range(40):                          # 4 s at 10 Hz
      t += 0.1; cnt = (cnt + 1) & 0xF
      state = on if (i >= 5 or p == 0) else not on   # the light reacts 0.5 s after Enter (inside SETTLE)
      w.add(0, 0x2A0, bytes([0x10 | cnt, (0x02 if state else 0x00) | 0x80, 0]), t)      # byte 1 bit 1 follows
      w.add(1, 0x123, bytes([rnd.randrange(256), 0x55]), t)                               # noise + constant
      w.add(128, 0x3E9, bytes([0xFF if on else 0]), t)                                    # our own frame: ignored
      w.add(0, 0x400, bytes([0xA0 if on else 0x05]), t)                                   # multi-bit field
  return w.analyse()


class T(unittest.TestCase):
  def test_finds_following_bit(self):
    r = run()
    found = [(b['bus'], b['id'], b['byte'], b['bit'], b['on']) for b in r['bits']]
    self.assertIn((0, '0x2A0', 1, 1, 1), found)
    self.assertFalse(any(b['id'] in ('0x123', '0x3E9') for b in r['bits']))
    self.assertFalse(any(b['id'] == '0x2A0' and b['byte'] == 0 for b in r['bits']))     # counter bits
    self.assertIn({'bus': 0, 'id': '0x400', 'byte': 0, 'on_values': ['a0'], 'off_values': ['05']}, r['bytes'])

  def test_needs_enough_phases(self):
    self.assertIn('error', run(phases=3))


if __name__ == '__main__':
  unittest.main()
