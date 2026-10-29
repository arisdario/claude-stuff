#!/usr/bin/env python3
"""check_model.py - on the EON: print what modeld publishes, twice a second, to check the 0.8.3 model before engaging.
Read-only (only listens to the 'model' message).

  cd /data/openpilot && PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/check_model.py [seconds]

What to expect on a normal road, car centred in its lane:
  left lane  offset about +1.5..+2.0 m, right lane about -1.5..-2.0 m (left is positive), probs near 1 with clear lines
  path offset near 0 and staying near 0;  lead prob > 0.5 only with a car ahead, dist close to radarweb's lead
"""
import sys
import time
import cereal.messaging as messaging

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


def main():
  sock = messaging.sub_sock('model')
  t0 = time.time(); nxt = t0; n = 0; last = None
  print('listening to "model" for %.0f s ...' % SECS)
  while time.time() - t0 < SECS:
    for ev in messaging.drain_sock(sock, wait_for_one=True):
      last = ev.model; n += 1
    if last is not None and time.time() >= nxt:
      nxt = time.time() + 0.5
      m = last; lp, rp, pp = list(m.leftLane.poly), list(m.rightLane.poly), list(m.path.poly)
      if len(lp) < 4 or len(rp) < 4 or len(pp) < 4:
        print('no polys in the message - is the new driving.cc built and running?'); continue
      ld = m.lead
      print('L %+5.2f m p%.2f | R %+5.2f m p%.2f | width %.2f | path %+5.2f m, at 30 m %+5.2f | lead p%.2f %5.1f m vrel %+5.1f | %d pts | %.0f Hz' % (
        lp[3], m.leftLane.prob, rp[3], m.rightLane.prob, lp[3] - rp[3], pp[3],
        ((pp[0] * 30 + pp[1]) * 30 + pp[2]) * 30 + pp[3], ld.prob, ld.dist, ld.relVel, len(m.path.points), n / max(time.time() - t0, 1e-3)))
  if n == 0:
    print('no "model" messages: modeld is not running (crashed?) - see README "If modeld does not start"')


if __name__ == '__main__':
  main()
