#!/usr/bin/env python3
"""can_tx_inventory.py - READ-ONLY: which CAN buses does openpilot SEND to, which buses ANSWER, and is the panda's canSendErrs counter still rising?
Subscribes to messages only (sendcan, can, health). It never opens the panda or USB, never sends anything. Run on the EON with openpilot running:

  cd /data/openpilot && python3 /data/can_tx_inventory.py [seconds]        (default 10)

canSendErrs is ONE counter for every frame the panda failed to put on a bus; it does not say which. The usual cause is a bus where no device acknowledges frames
(the transmit queue fills). So the table compares, per bus: frames openpilot asks the panda to send vs frames the panda receives. A bus we send to but never hear
from is the suspect. The four DAS-emulation addresses are consumed inside the panda firmware (they are not put on any bus) and are listed apart."""
import sys
import time
from collections import Counter, defaultdict

CONSUMED = {0x557: 'DAS lanes+lead', 0x559: 'DAS objects', 0x659: 'DAS status+steering', 0x554: 'DAS switches'}   # tesla firmware: expanded inside the panda


def summarize(sent, rx, errs0, errs1, secs):
  """sent: Counter{(bus, addr): n}; rx: Counter{(bus, addr): n}; -> list of text lines"""
  out = []
  secs = max(secs, 1e-6)
  consumed = {k: v for k, v in sent.items() if k[1] in CONSUMED}
  real = {k: v for k, v in sent.items() if k[1] not in CONSUMED}
  rxbus = defaultdict(int); rxaddr = defaultdict(set)
  for (b, a), n in rx.items():
    rxbus[b] += n; rxaddr[b].add(a)
  txbus = defaultdict(int)
  for (b, a), n in real.items():
    txbus[b] += n
  out.append('frames openpilot asks the panda to put on a bus (per second), by bus:')
  for b in sorted(set(txbus) | set(rxbus)):
    out.append('  bus %d: sent %6.1f/s   received %7.1f/s (%d addresses)' % (b, txbus[b] / secs, rxbus[b] / secs, len(rxaddr[b])))
  out.append('')
  out.append('what is sent, busiest first (bus, address, per second):')
  for (b, a), n in sorted(real.items(), key=lambda kv: -kv[1])[:20]:
    out.append('  bus %d  0x%03x  %6.1f/s' % (b, a, n / secs))
  if consumed:
    out.append('consumed inside the panda firmware (not sent on any bus): ' + ', '.join('0x%03x %s %.0f/s' % (a, CONSUMED[a], n / secs) for (b, a), n in sorted(consumed.items(), key=lambda kv: kv[0][1])))
  out.append('')
  for b in sorted(txbus):
    if rxbus[b] == 0:
      out.append('SUSPECT bus %d: %.1f frames/s are sent to it but NOTHING is received from it - no device is acknowledging, so its transmit queue fills.' % (b, txbus[b] / secs))
  if errs0 is not None and errs1 is not None:
    d = errs1 - errs0
    out.append('canSendErrs: %d -> %d  (+%d in %.0f s = %.0f/s)%s' % (errs0, errs1, d, secs, d / secs, '   <- not rising now' if d == 0 else ''))
  else:
    out.append('canSendErrs: no health message received')
  return out


def main():
  secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
  import cereal.messaging as messaging
  sc = messaging.sub_sock('sendcan', timeout=50)
  cn = messaging.sub_sock('can', timeout=50)
  hl = messaging.sub_sock('health', conflate=True, timeout=50)
  sent, rx = Counter(), Counter()
  errs0 = errs1 = None
  t0 = time.time()
  while time.time() - t0 < secs:
    for ev in messaging.drain_sock(sc):
      for c in ev.sendcan:
        sent[(c.src, c.address)] += 1
    for ev in messaging.drain_sock(cn):
      for c in ev.can:
        rx[(c.src, c.address)] += 1
    for ev in messaging.drain_sock(hl):
      e = ev.health.canSendErrs
      if errs0 is None:
        errs0 = e
      errs1 = e
    time.sleep(0.02)
  print('\n'.join(summarize(sent, rx, errs0, errs1, time.time() - t0)))
  return 0


if __name__ == '__main__':
  sys.exit(main())
