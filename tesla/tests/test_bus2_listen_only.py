#!/usr/bin/env python3
"""tesla/panda/bus2_listen_only.py: patches the two places in panda board/drivers/can.h, refuses on unknown code."""
import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'panda'))
import bus2_listen_only as B  # noqa: E402

CAN_H = """#define ALL_CAN_SILENT 0xFF
#define ALL_CAN_LIVE 0

int can_live = 0, pending_can_live = 0, can_loopback = 0, can_silent = ALL_CAN_SILENT;

void can_set_speed(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  if (!llcan_set_speed(CAN, can_speed[bus_number], can_loopback, (unsigned int)(can_silent) & (1U << can_number))) {
    puts("CAN init FAILED!!!!!\\n");
  }
}

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number, bool skip_tx_hook) {
  if (skip_tx_hook || safety_tx_hook(to_push) != 0) {
    queue(to_push, bus_number);
  }
}
"""


class T(unittest.TestCase):
  def test_patch_both_layers(self):
    t, msg = B.patch(CAN_H)
    self.assertIn('applied', msg)
    self.assertIn('#define LISTEN_ONLY_BUS 2U', t)
    self.assertIn('| ((bus_number == LISTEN_ONLY_BUS) ? 1U : 0U))', t)                        # silent controller for bus 2
    body = t[t.index('void can_send('):]
    self.assertLess(body.index('if (bus_number == LISTEN_ONLY_BUS)'), body.index('safety_tx_hook'))   # dropped before anything else

  def test_idempotent(self):
    t, _ = B.patch(CAN_H)
    t2, msg = B.patch(t)
    self.assertEqual(t, t2); self.assertIn('already', msg)

  def test_refuses_unknown_code(self):
    for broken in (CAN_H.replace('#define ALL_CAN_LIVE 0', ''), CAN_H.replace('can_loopback, (unsigned int)', 'can_loopback, (int)'),
                   CAN_H.replace('bool skip_tx_hook) {', 'bool skip) {')):
      with self.assertRaises(SystemExit):
        B.patch(broken)

  def test_semantics_compiled(self):
    # the patched expressions, compiled and run: bus 2 -> silent and every send dropped; other buses unchanged
    src = r'''#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#define LISTEN_ONLY_BUS 2U
static int can_silent = 0, queued[4];
static bool silent_arg(uint8_t can_number, uint8_t bus_number) { return ((unsigned int)(can_silent) & (1U << can_number)) | ((bus_number == LISTEN_ONLY_BUS) ? 1U : 0U); }
static void can_send(uint8_t bus_number) { if (bus_number == LISTEN_ONLY_BUS) { return; } queued[bus_number]++; }
int main(void) { for (int b = 0; b < 3; b++) { can_send(b); can_send(b); }
  printf("%d%d%d %d%d%d\n", silent_arg(0,0), silent_arg(1,1), silent_arg(2,2), queued[0], queued[1], queued[2]); return 0; }'''
    d = tempfile.mkdtemp(); open(os.path.join(d, 't.c'), 'w').write(src)
    subprocess.check_call(['gcc', '-Wall', '-Wextra', '-Werror', '-o', os.path.join(d, 't'), os.path.join(d, 't.c')])
    self.assertEqual(subprocess.check_output([os.path.join(d, 't')]).decode().strip(), '001 220')

  def test_apply_and_restore_on_a_file(self):
    d = tempfile.mkdtemp(); f = os.path.join(d, 'can.h'); open(f, 'w').write(CAN_H)
    env = dict(os.environ, FILE=f); script = os.path.join(os.path.dirname(B.__file__), 'bus2_listen_only.py')
    subprocess.check_call([sys.executable, script, '--apply'], env=env, stdout=subprocess.DEVNULL)
    self.assertEqual(subprocess.call([sys.executable, script, '--check'], env=env, stdout=subprocess.DEVNULL), 0)
    subprocess.check_call([sys.executable, script, '--restore'], env=env, stdout=subprocess.DEVNULL)
    self.assertEqual(open(f).read(), CAN_H)


if __name__ == '__main__':
  unittest.main()
