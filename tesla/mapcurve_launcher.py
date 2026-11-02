#!/usr/bin/env python3
"""
mapcurve_launcher.py - start mapcurved.py (the road-aware map curve-speed daemon) together with the car stack,
WITHOUT needing an entry in openpilot's process manager (which this repo's deploy_car.sh doesn't touch).

carcontroller.__init__ calls ensure_running() once at startup. It:
  - does nothing if there is no map DB at /data/mapcurve.sqlite (nothing uploaded -> no daemon, no error);
  - does nothing if disabled by the kill switch /data/mapcurve_off;
  - does nothing if a mapcurved is already alive (pidfile + /proc cmdline check), so a controlsd restart never
    spawns a second copy;
  - otherwise launches `python3 selfdrive/car/tesla/mapcurved.py` detached, with PYTHONPATH/cwd set so its
    cereal + pcc_curves imports resolve, logging to /data/mapcurved.log, and records its pid.

So the whole on-car workflow becomes: upload mapcurve.sqlite to /data/, set use_tesla_gps = True once, reboot.
No process to start by hand.

Every path is wrapped so a failure here can NEVER take down the car stack: worst case the map daemon simply doesn't
start and PCC keeps running on vision + radar exactly as before. Read-only: this starts a process that only reads the
sqlite and GPS and publishes liveMapData; it touches no CAN, UDS, or vehicle state.
"""
import os
import subprocess
import sys

DB_PATH = '/data/mapcurve.sqlite'
PID_PATH = '/data/mapcurved.pid'
OFF_FLAG = '/data/mapcurve_off'
LOG_PATH = '/data/mapcurved.log'
OP_DIR = '/data/openpilot'
SCRIPT = 'selfdrive/car/tesla/mapcurved.py'
PROC_TAG = 'mapcurved.py'


def _alive(pid):
    """True only if that pid is running AND is actually a mapcurved (guards against a reused pid)."""
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as f:
            return PROC_TAG in f.read().decode('utf-8', 'replace')
    except Exception:
        return False


def _running_pid(pid_path=PID_PATH):
    try:
        with open(pid_path) as f:
            pid = int(f.read().strip())
    except Exception:
        return None
    return pid if _alive(pid) else None


def ensure_running(db_path=DB_PATH, op_dir=OP_DIR, pid_path=PID_PATH, off_flag=OFF_FLAG, log_path=LOG_PATH):
    """Start mapcurved once if a map DB exists, it's not disabled, and it isn't already running.

    Returns True if it launched a new process, False otherwise. Never raises.
    """
    try:
        if os.path.exists(off_flag):
            return False                      # kill switch
        if not os.path.exists(db_path):
            return False                      # no map uploaded -> nothing to do
        if not os.path.isfile(os.path.join(op_dir, SCRIPT)):
            return False                      # not deployed into the openpilot tree
        if _running_pid(pid_path) is not None:
            return False                      # already up
        env = dict(os.environ)
        env['PYTHONPATH'] = op_dir + ((os.pathsep + env['PYTHONPATH']) if env.get('PYTHONPATH') else '')
        log = open(log_path, 'ab', buffering=0)
        p = subprocess.Popen([sys.executable, SCRIPT], cwd=op_dir, env=env,
                             stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                             start_new_session=True)
        try:
            with open(pid_path, 'w') as f:
                f.write(str(p.pid))
        except Exception:
            pass
        return True
    except Exception:
        return False


if __name__ == '__main__':
    # Manual use / debugging: report what it would do and (unless --dry) start it.
    dry = '--dry' in sys.argv
    print('db=%s exists=%s  off=%s  running_pid=%s'
          % (DB_PATH, os.path.exists(DB_PATH), os.path.exists(OFF_FLAG), _running_pid()))
    if not dry:
        print('launched' if ensure_running() else 'did not launch (see reasons above)')
