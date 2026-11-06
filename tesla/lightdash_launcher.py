#!/usr/bin/env python3
"""
lightdash_launcher.py - start the lightdash web dashboard together with the car stack, WITHOUT needing an
entry in openpilot's process manager (same approach as mapcurve_launcher.py, which this mirrors).

carcontroller.__init__ calls ensure_running() once at startup. It:
  - does nothing if the binary at /data/lightdash doesn't exist (nothing built -> no daemon, no error);
  - does nothing if disabled by the kill switch /data/lightdash_off;
  - does nothing if a lightdash is already alive (pidfile + /proc cmdline check), so a controlsd restart
    never spawns a second copy;
  - otherwise launches /data/lightdash detached, with LD_LIBRARY_PATH set so its shared libs resolve,
    logging to /data/lightdash.log, and records its pid.

So the on-car workflow becomes: build it once (build_lightdash.sh), reboot. No process to start by hand,
and /data/lightdash_start.sh is no longer needed day to day (still fine to use by hand too).

Every path is wrapped so a failure here can NEVER take down the car stack: worst case the dashboard simply
doesn't start and nothing else changes. Read-only with respect to the car: lightdash only subscribes to
openpilot's own messages (sendcan, can, model, carState, controlsState); it never opens the panda, never
sends anything, and this launcher does not touch CAN, UDS, or vehicle state either - it only starts a process.
"""
import os
import subprocess
import sys

BIN_PATH = '/data/lightdash'
PID_PATH = '/data/lightdash.pid'
OFF_FLAG = '/data/lightdash_off'
LOG_PATH = '/data/lightdash.log'
LIB_DIR = '/system/comma/usr/lib'
PROC_TAG = 'lightdash'


def _alive(pid):
    """True only if that pid is running AND is actually a lightdash (guards against a reused pid)."""
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


def ensure_running(bin_path=BIN_PATH, pid_path=PID_PATH, off_flag=OFF_FLAG, log_path=LOG_PATH, lib_dir=LIB_DIR):
    """Start lightdash once if it's built, not disabled, and not already running.

    Returns True if it launched a new process, False otherwise. Never raises.
    """
    try:
        if os.path.exists(off_flag):
            return False                      # kill switch
        if not os.path.isfile(bin_path):
            return False                      # not built yet
        if _running_pid(pid_path) is not None:
            return False                      # already up
        env = dict(os.environ)
        if os.path.isdir(lib_dir):
            env['LD_LIBRARY_PATH'] = lib_dir + ((os.pathsep + env['LD_LIBRARY_PATH']) if env.get('LD_LIBRARY_PATH') else '')
        log = open(log_path, 'ab', buffering=0)
        p = subprocess.Popen([bin_path], env=env, stdout=log, stderr=log, stdin=subprocess.DEVNULL,
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
    print('bin=%s exists=%s  off=%s  running_pid=%s'
          % (BIN_PATH, os.path.isfile(BIN_PATH), os.path.exists(OFF_FLAG), _running_pid()))
    if not dry:
        print('launched' if ensure_running() else 'did not launch (see reasons above)')
