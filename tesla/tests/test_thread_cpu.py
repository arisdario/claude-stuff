import os, sys, subprocess, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import thread_cpu as t


def test_finds_busy_thread():
  code = "import threading\ndef b():\n  x = 0\n  while True: x += 1\nthreading.Thread(target=b, daemon=True).start()\nimport time; time.sleep(6)"
  p = subprocess.Popen([sys.executable, '-c', code]); time.sleep(0.5)
  try:
    a = t.snap(p.pid); time.sleep(1.5); b = t.snap(p.pid)
    top = max((((b[k][1] - a.get(k, ('', 0, 0))[1]) + (b[k][2] - a.get(k, ('', 0, 0))[2])) for k in b))
    assert top >= 0.5 * os.sysconf('SC_CLK_TCK')            # some thread used at least ~1/3 of a core during 1.5 s
    assert t.find_pid(str(p.pid)) == p.pid
  finally:
    p.kill()
