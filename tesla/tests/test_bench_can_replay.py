"""bench_can_replay: log parsing, speed decoding and pacing (no openpilot needed). The replay only publishes to the local cereal socket; the script never opens a CAN interface."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'model083'))
import bench_can_replay as B  # noqa: E402

LOG = """nohup: ignoring input
[+] Waiting for Arduino CAN frames...
[CAN->CEREAL] ID=0x118 DLC=6 BUS=0,2 DATA=0090F4119C48
[CAN->CEREAL] ID=0x118 DLC=6 BUS=0,2 DATA=004015438D3C
[CAN->CEREAL] ID=0x45 DLC=8 BUS=0,2 DATA=40FF000000006154
[CAN->CEREAL] ID=0x6D DLC=4 BUS=0,2 DATA=40C070
[CAN->CEREAL] ID=0xZZ DLC=2 BUS=0,2 DATA=0102
[CAN->CEREAL] ID=0x101 DLC=3 BUS=0 DATA=050A11
"""


class TestReplay(unittest.TestCase):
    def setUp(self):
        self.p = os.path.join(tempfile.mkdtemp(), 'bridge.log')
        open(self.p, 'w').write(LOG)

    def test_parse_skips_noise_and_dlc_mismatches(self):
        frames, bad = B.parse_log(self.p)
        self.assertEqual([hex(a) for a, _, _ in frames], ['0x118', '0x118', '0x45', '0x101'])
        self.assertEqual(bad, 1)                                    # 0x6D: DLC 4 but 3 data bytes
        self.assertEqual(frames[3][2], [0])

    def test_vehicle_speed_decodes_like_the_dbc(self):
        frames, _ = B.parse_log(self.p)
        self.assertAlmostEqual(B.di_speed_mph(*frames[0][:2]), 0.0, places=2)         # raw 500 -> 0.0 mph (standing)
        self.assertAlmostEqual(B.di_speed_mph(*frames[1][:2]), 0x315 * 0.05 - 25, places=2)   # bytes 15 43 -> raw 0x315 = 789 -> 14.45 mph
        self.assertIsNone(B.di_speed_mph(*frames[2][:2]))
        mn, mx, n = B.speed_summary(frames)
        self.assertEqual(n, 2)
        self.assertLess(mn, mx)

    def test_pacing_keeps_file_order_and_rate(self):
        frames = [(i, b'\x00', [0]) for i in range(1000)]
        chunks = list(B.batches(frames, 100.0))                       # 100 frames/s at 10 ms ticks = 1 frame per tick
        self.assertEqual(len(chunks), 1000)
        flat = [f[0] for c in chunks for f in c]
        self.assertEqual(flat, list(range(1000)))
        self.assertEqual(len(list(B.batches(frames, 1000.0))), 100)   # 10 frames per tick

    def test_script_never_touches_a_can_interface_or_sendcan(self):
        src = open(os.path.join(os.path.dirname(B.__file__), 'bench_can_replay.py')).read()
        code = [l for l in src.splitlines() if not l.strip().startswith(('#', '"""'))]
        text = '\n'.join(code).replace('never publishes `sendcan`', '')
        for forbidden in ("PubMaster(['sendcan'", "pub_sock('sendcan'", 'socketcan', 'import can\n', 'usb1', 'panda import'):
            self.assertNotIn(forbidden, text)


if __name__ == '__main__':
    unittest.main()
