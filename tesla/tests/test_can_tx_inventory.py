import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from collections import Counter
import can_tx_inventory as T


class Tx(unittest.TestCase):
  def test_suspect_bus_and_consumed(self):
    sent = Counter({(0, 0x488): 500, (1, 0x2b9): 200, (0, 0x659): 1000, (0, 0x557): 100})
    rx = Counter({(0, 0x118): 3000})
    s = '\n'.join(T.summarize(sent, rx, 100, 600, 10))
    self.assertIn('SUSPECT bus 1', s)
    self.assertNotIn('SUSPECT bus 0', s)
    self.assertIn('consumed inside the panda firmware', s)
    self.assertIn('+500 in 10 s = 50/s', s)
    self.assertIn('could not queue for the host', s)
    self.assertNotIn('0x659  ', s.split('consumed')[0])          # the consumed addresses are not counted as bus traffic

  def test_echoes_are_the_panda_own_transmits(self):
    rx = Counter({(0, 0x118): 3000, (128, 0x399): 200, (129, 0x2a9): 100})
    s = '\n'.join(T.summarize(Counter({(0, 0x488): 10}), rx, 1, 1, 10))
    self.assertIn('TRANSMITTED successfully', s); self.assertIn('bus 0  0x399', s); self.assertIn('bus 1  0x2a9', s)
    self.assertNotIn('bus 128:', s)                              # not listed as a bus of its own

  def test_not_rising(self):
    s = '\n'.join(T.summarize(Counter({(0, 0x488): 10}), Counter({(0, 0x118): 10}), 5, 5, 10))
    self.assertIn('not rising now', s)

  def test_fwd_counter(self):
    s = '\n'.join(T.summarize(Counter({(0, 0x488): 10}), Counter({(0, 0x118): 10}), 5, 5, 10, 100, 130))
    self.assertIn('canFwdErrs', s); self.assertIn('+30 in 10 s', s); self.assertIn('transmit queue is full', s)

  def test_no_health(self):
    self.assertIn('no health message', '\n'.join(T.summarize(Counter(), Counter(), None, None, 10)))


if __name__ == '__main__':
  unittest.main()
