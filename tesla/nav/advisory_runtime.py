"""Run-time gate for the curve advisory: shadow logging by default, a driver-visible TEXT notification only when you switch it on and everything checks out.
ADVISORY ONLY: the output has no steering/braking/acceleration command and nothing here imports vehicle-control code. A notification is a short text (and a level) for the display layer to show.

SWITCHES (files on the EON, same convention as tesla_csa):
  default (no files)              SHADOW: the advisory is computed and logged to /data/curveadvice_shadow.log; NO notification is produced.
  /data/tesla_curveadvice_off     overrides everything: no notification, no computation of one, ever. (Nothing else changes.)
  /data/tesla_curveadvice_enable  notifications ON, but only while every check passes:
        - road confidence is 'high' (the road ahead is identified; never on a guess),
        - the curve is moderate or tighter, and was not already announced,
        - the GPS position is fresh (age <= MAX_POSITION_AGE_S) and the speed is >= MIN_SPEED_MS,
        - the curve is within the notification distance (braking distance + reaction time at the current speed, never below MIN_NOTIFY_M).
  Any failed check -> no notification (the reason is logged). A curve is notified ONCE."""
import json
import math
import os
import time

OFF_FILE = '/data/tesla_curveadvice_off'
ENABLE_FILE = '/data/tesla_curveadvice_enable'
SHADOW_LOG = '/data/curveadvice_shadow.log'
SHADOW_LOG_MAX = 5 * 1024 * 1024
MAX_POSITION_AGE_S = 30.0
MIN_SPEED_MS = 8.0           # no notifications when crawling (about 29 km/h)
MIN_NOTIFY_M = 200.0
REACTION_S = 3.0             # reading time + reaction before the braking distance starts to matter
DECEL = 1.0


def _exists(path):
    try:
        return os.path.exists(path)
    except Exception:
        return False


def mode(off_file=OFF_FILE, enable_file=ENABLE_FILE):
    """'off' | 'enabled' | 'shadow' (the default; also the answer if the files cannot be read)"""
    if _exists(off_file):
        return 'off'
    return 'enabled' if _exists(enable_file) else 'shadow'


class CurveAdvisory(object):
    """Wraps a CurveAhead (or anything with update(...) -> its advisory dict). step(msg) -> {'mode','notify','reason','target_speed_ms'}; `notify` is None or {'text','level','curve_id','distance_m'}."""
    def __init__(self, off_file=OFF_FILE, enable_file=ENABLE_FILE, log_path=SHADOW_LOG, decel=DECEL, clock=time.time):
        self.off_file, self.enable_file, self.log_path, self.decel, self.clock = off_file, enable_file, log_path, decel, clock
        self.notified = set()

    def _log(self, rec):
        if not self.log_path:
            return
        try:
            if os.path.exists(self.log_path) and os.path.getsize(self.log_path) > SHADOW_LOG_MAX:
                os.replace(self.log_path, self.log_path + '.1')
            with open(self.log_path, 'a') as f:
                f.write(json.dumps(rec) + '\n')
        except Exception:
            pass                                                    # logging must never affect anything else

    def step(self, msg, speed_ms):
        m = mode(self.off_file, self.enable_file)
        ann = (msg.get('announce') or {}).get('active') or []
        adv = msg.get('advisory') or {}
        notify, reason = None, None
        if m == 'off':
            reason = 'off switch present'
        elif adv.get('confidence') != 'high':
            reason = 'road not confidently identified (%s)' % adv.get('confidence')
        elif speed_ms is None or speed_ms < MIN_SPEED_MS:
            reason = 'speed below %.0f m/s' % MIN_SPEED_MS
        elif (adv.get('position_age_s') or 0.0) > MAX_POSITION_AGE_S:
            reason = 'GPS position older than %.0f s' % MAX_POSITION_AGE_S
        else:
            for a in ann:
                if a['id'] in self.notified or a.get('distance_m') is None:
                    continue
                brake = max(0.0, (speed_ms ** 2 - a['speed_ms'] ** 2) / (2.0 * self.decel))
                lead = max(MIN_NOTIFY_M, brake + speed_ms * REACTION_S)
                if a['distance_m'] <= lead:
                    notify = {'text': a['text'], 'level': 'warn' if a['severity'] in ('sharp', 'hairpin') else 'info', 'curve_id': a['id'], 'distance_m': a['distance_m']}
                    break
            if notify is None:
                reason = 'no curve inside the notification distance'
        would = notify
        if m != 'enabled':
            notify = None                                           # shadow / off: computed (shadow only), never delivered
            if m == 'shadow' and would is not None:
                reason = 'shadow mode: would notify'
        elif notify is not None:
            self.notified.add(notify['curve_id'])
        if m != 'off':
            self._log({'t': round(self.clock(), 2), 'mode': m, 'conf': adv.get('confidence'), 'v': speed_ms, 'progress': msg.get('route_progress_m'),
                       'target': adv.get('target_speed_ms'), 'curve': (ann[0]['text'], round(ann[0]['distance_m'] or 0.0)) if ann else None,
                       'would_notify': None if would is None else would['text'], 'notified': None if notify is None else notify['text'], 'reason': reason})
        return {'mode': m, 'notify': notify, 'reason': reason, 'target_speed_ms': adv.get('target_speed_ms')}
