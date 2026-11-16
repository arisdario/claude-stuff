"""AHBSTALK - can the high beams be switched through the light stalk, the way the ACC presses the cruise stalk?

What is known (radarweb map logs of 25 Oct, 1 Nov, 4 Nov): while the high beams were on, the stalk message's high-beam
field STW_ACTN_RQ.HiBmLvr_Stat stayed at 1 ("pushed") for 12-75 s at a time. The lever is LATCHED: it stays in the
high-beam position. If the light controller follows the lever's POSITION, openpilot could only hold the beams on (or
dip them) by contradicting the real steering-column module in every message, all the time, on the message that also
carries the turn signals, wipers, horn and cruise lever. That is not built. If the light controller instead LATCHES on
a short push, a short push / pull is enough - exactly like a cruise click - and automatic high beams become possible.

Tesla's own automatic high beams (what the driver wants here): YOU push the lever out (high beams on); the system DIPS
for oncoming / preceding traffic and restores them when the road is clear - only while the lever is out. With the
lever latched out, a dip means the light controller must see "normal" while the column keeps reporting "pushed".

This ONE-SHOT, PARKED test answers how the light controller reacts. It sends the real stalk message (every other field
copied from the steering column, as the ACC does) with the high-beam lever changed for 0.5 s, once, then logs 4 s more.

  Car in P, headlights on, openpilot not engaged. Watch the blue high-beam symbol on the cluster, then:
    lever OUT (high beams on):  echo dip > /data/ahb_stalk_test     # 0.5 s "normal": do they dip - and STAY dipped?
    lever normal:               touch /data/ahb_stalk_test          # 0.5 s PUSH: do they come on - and STAY on?
    lever normal:               echo pull > /data/ahb_stalk_test    # 0.5 s PULL (flash)
  Result: /data/ahb_stalk_test.log; the trigger file is renamed to .done (or .skipped / .aborted), so it never repeats.

It sends nothing unless: the car stands still, openpilot is not engaged, the light lever is where the test needs it
(out for dip, normal otherwise) and the turn-signal and cruise levers are untouched. It stops at once if the car moves, openpilot engages or you touch any of those levers (you win).
It waits up to 60 s for those conditions, then gives up (.skipped).
"""
import os
import time

TEST_FILE = '/data/ahb_stalk_test'
LOG_FILE = '/data/ahb_stalk_test.log'
PRESS_S = 0.5           # how long the lever is "held" - like one cruise click (the ACC holds a click ~0.4 s)
OBSERVE_S = 4.0         # logging after the press: do the beams stay on or drop back?
WAIT_MAX_S = 60.        # conditions not met within this -> give up
CHECK_EVERY_S = 1.0     # the trigger file is looked for once a second (controlsd CPU)
LEVER = {'push': 1, 'pull': 2, 'dip': 0}   # HiBmLvr_Stat: 0 IDLE (normal), 1 HIBM_ON_PSD (pushed out), 2 HIBM_FLSH_ON_PSD (pulled)
NEEDS = {'push': 0, 'pull': 0, 'dip': 1}    # where YOUR lever must be for each test


def _lever(stalk, name):
  try:
    return int(round(stalk[name]))
  except Exception:
    return None


class StalkHighBeamTest(object):
  def __init__(self, path=TEST_FILE, log=LOG_FILE, clock=time.monotonic):
    self.path, self.logp, self.clock = path, log, clock
    self.state, self.kind, self.lines = 'idle', None, []
    self.next_check = 0.
    self.t_wait = self.t0 = None
    self.levers0 = None

  # ------------------------------------------------------------------ helpers
  def _why_not(self, CS, enabled):
    why = []
    if enabled:
      why.append('openpilot engaged')
    if (getattr(CS, 'v_ego', 0.) or 0.) > 0.1:
      why.append('car moving')
    stalk = getattr(CS, 'steering_wheel_stalk', None)
    if not stalk:
      why.append('no stalk message from the car yet')
    else:
      if _lever(stalk, 'HiBmLvr_Stat') != NEEDS[self.kind]:
        why.append('light lever must be %s for this test' % ('pushed out (high beams on)' if NEEDS[self.kind] else 'in its normal position'))
      if _lever(stalk, 'TurnIndLvr_Stat') != 0: why.append('turn-signal lever moved')
      if _lever(stalk, 'SpdCtrlLvr_Stat') != 0: why.append('cruise lever moved')
    return why

  def _levers(self, CS):
    stalk = getattr(CS, 'steering_wheel_stalk', None) or {}
    return tuple(_lever(stalk, n) for n in ('HiBmLvr_Stat', 'TurnIndLvr_Stat', 'SpdCtrlLvr_Stat'))

  def _log(self, now, phase, sent, CS):
    stalk = getattr(CS, 'steering_wheel_stalk', None) or {}
    self.lines.append('%6.2f %-9s sent_lever=%s real_lever=%s body:HiBm_On=%s LoBm_On_Rq=%s LgtSens_Night=%s' % (
      now - (self.t0 if self.t0 is not None else now), phase, sent, _lever(stalk, 'HiBmLvr_Stat'),
      getattr(CS, 'ahbHiBeamOn', None), getattr(CS, 'ahbLoBeamOn', None), getattr(CS, 'ahbNightMode', None)))

  def _finish(self, result, note=''):
    head = ['AHBSTALK test (%s): %s %s' % (self.kind, result, note),
            time.strftime('%Y-%m-%d %H:%M:%S'),
            'what to report: did the beams change and STAY changed after the 0.5 s (latch), change only for those 0.5 s',
            '(the controller follows the lever position), or not react at all? body:* = what the car reports (may read 0)']
    try:
      with open(self.logp, 'w') as f:
        f.write('\n'.join(head + self.lines) + '\n')
    except Exception:
      pass
    try:
      os.rename(self.path, self.path + '.' + result)
    except Exception:
      pass
    self.state, self.lines, self.t0, self.t_wait = 'idle', [], None, None

  # ------------------------------------------------------------------ main
  def update(self, CS, enabled, busy=False):
    """called at 20 Hz from carcontroller. Returns the HiBmLvr_Stat value to inject in THIS frame's stalk message, or
    None (nothing to send). busy = a cruise press goes out in this frame (then this frame is skipped)."""
    now = self.clock()
    if self.state == 'idle':
      if now < self.next_check:
        return None
      self.next_check = now + CHECK_EVERY_S
      if not os.path.exists(self.path):
        return None
      try:
        word = open(self.path).read().strip().lower()
      except Exception:
        word = ''
      self.kind = 'pull' if word.startswith('pull') else ('dip' if word.startswith('dip') else 'push')
      self.state, self.t_wait, self.lines = 'waiting', now, []
    if self.state == 'waiting':
      why = self._why_not(CS, enabled)
      if why:
        if now - self.t_wait > WAIT_MAX_S:
          self._finish('skipped', '(' + ', '.join(why) + ')')
        return None
      self.state, self.t0, self.levers0 = 'pressing', now, self._levers(CS)
    if self.state in ('pressing', 'observing'):
      if enabled or (getattr(CS, 'v_ego', 0.) or 0.) > 0.1:
        self._finish('aborted', '(car moved or openpilot engaged)')
        return None
    if self.state == 'pressing':
      if self._levers(CS) != self.levers0:           # you touched a lever: stop sending, you win
        self._log(now, 'driver', None, CS)
        self._finish('aborted', '(a lever was moved during the test)')
        return None
      if now - self.t0 < PRESS_S - 1e-3:            # (tolerance: the 20 Hz clock is not exact)
        if busy:
          self._log(now, 'skip', None, CS)
          return None
        self._log(now, 'press', LEVER[self.kind], CS)
        return LEVER[self.kind]
      self.state = 'observing'
    if self.state == 'observing':
      self._log(now, 'observe', None, CS)
      if now - self.t0 >= PRESS_S + OBSERVE_S:
        self._finish('done')
      return None
    return None
