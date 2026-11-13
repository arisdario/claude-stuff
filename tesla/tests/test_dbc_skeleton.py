#!/usr/bin/env python3
"""tools/dbc_skeleton.py: logs -> per-bus DBC with known definitions merged, placeholders for unknown IDs (only the bytes that
change), excluded IDs left out, extended IDs marked, and the output loads in cantools (when installed)."""
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import dbc_skeleton as D  # noqa: E402

KNOWN = '''VERSION ""
BO_ 264 DI_torque1: 8 DI
 SG_ DI_motorRPM : 32|16@1- (1,0) [-17000|17000] "RPM" X
VAL_ 264 DI_motorRPM 0 "ZERO" ;
VAL_ 264 NOT_A_SIGNAL_HERE 1 "x" ;
'''


class T(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()

  def tearDown(self):
    shutil.rmtree(self.d)

  def test_build(self):
    log = os.path.join(self.d, 'map_1.jsonl')
    with open(log, 'w') as f:
      for i in range(5):
        f.write(json.dumps({'raw': {'0x108': '00 00 00 00 %02x 10 00 00' % i, '0x123': '11 %02x 33' % (i * 3), '0x239': '00' * 8,
                                    'r0x310': '00' * 8}}) + '\n')
      f.write('[CAN->CEREAL] ID=0x18DAF1 DLC=2 BUS=0,2 DATA=0102\n')
    dbc = os.path.join(self.d, 'k.dbc'); open(dbc, 'w').write(KNOWN)
    out = os.path.join(self.d, 'car')
    files = D.build(D.read_logs([log]), D.read_dbc_blocks([dbc]), out)
    t0 = open(files[0]).read()
    self.assertIn('BO_ 264 DI_torque1', t0); self.assertIn('VAL_ 264 DI_motorRPM', t0); self.assertNotIn('NOT_A_SIGNAL_HERE', t0)
    self.assertIn('BO_ 291 UNKNOWN_123: 3', t0)
    self.assertIn('SG_ B1_123 : 8|8@1+', t0); self.assertNotIn('B0_123', t0)          # byte 0 never changes: no placeholder
    self.assertNotIn('BO_ 569 ', t0)                                                 # 0x239 left out
    self.assertIn('BO_ %d UNKNOWN_18DAF1' % (0x18DAF1 | 0x80000000), t0)             # extended id marked
    self.assertIn('radar', files)
    self.assertIn('| 0x239 | 5 | 8 | (left out) |', open(out + '_report.md').read())
    try:
      import cantools
    except ImportError:
      return
    db = cantools.database.load_file(files[0], strict=False)
    self.assertEqual(db.get_message_by_frame_id(264).decode(bytes([0, 0, 0, 0, 0x10, 0x27, 0, 0]))['DI_motorRPM'], 10000)


if __name__ == '__main__':
  unittest.main()
