#!/usr/bin/env python3
"""launcher.py - tiny on-demand switchboard for the EON (stdlib only; idle cost ~0: it blocks in accept()).

Purpose: keep heavy helpers (radarweb, mapcurved, the diagnostics logger) OFF while driving unless you want them, and start/stop them from a phone page:

  python3 launcher.py [port]          # default 8087; open http://<EON-IP>:8087/

Buttons: radarweb start/stop, diagnostics start/stop (start_diagnostics.sh / stop_diagnostics.sh), FCW shadow logger start/stop/report
(fcw_shadow.py - logs only, never alerts), live per-process CPU + temperature.
It never touches openpilot's own processes (controlsd, modeld, camerad ...): stopping is done only on the processes matching the whitelisted
names in /data/launcher.conf, and only via SIGTERM to that exact PID. No shell is exposed: the commands are fixed in the config file, not typed in the page.

/data/launcher.conf (key=value, optional; defaults below). Put your real start command here instead of me knowing it (it may hold ngrok tokens):
  radarweb_cmd=sh /data/startweb.sh
  radarweb_match=radarweb.py          # comma separated substrings of /proc/<pid>/cmdline that belong to the web stack (e.g. radarweb.py,ngrok)
  diag_dir=/data/claude-stuff/tesla/diag
Auth: set LAUNCHER_AUTH=user:password in the environment to require HTTP basic auth (do this before exposing it beyond your own network).
"""
import base64
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

CONF = os.environ.get('LAUNCHER_CONF', '/data/launcher.conf')
DEFAULTS = {'radarweb_cmd': 'sh /data/startweb.sh', 'radarweb_match': 'radarweb.py', 'diag_dir': '/data/claude-stuff/tesla/diag'}
FCW_SHADOW = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fcw_shadow.py')
AUTH = os.environ.get('LAUNCHER_AUTH', '')
PROTECTED = ('controlsd', 'modeld', 'camerad', 'plannerd', 'boardd', 'manager', 'radard', 'locationd', 'paramsd', 'calibrationd', 'sensord', 'thermald')


def conf():
  c = dict(DEFAULTS)
  try:
    for ln in open(CONF):
      ln = ln.split('#')[0].strip()
      if '=' in ln:
        k, v = ln.split('=', 1)
        c[k.strip()] = v.strip()
  except OSError:
    pass
  return c


def find(match, proc='/proc'):
  """pids whose cmdline contains one of the substrings (never this process, never a protected openpilot process)."""
  keys = [m.strip() for m in match.split(',') if m.strip()]
  out = []
  for d in os.listdir(proc):
    if not d.isdigit() or int(d) == os.getpid():
      continue
    try:
      cmd = open('%s/%s/cmdline' % (proc, d), 'rb').read().replace(b'\0', b' ').decode('utf8', 'replace')
    except OSError:
      continue
    if any(k in cmd for k in keys) and not any(p in cmd.split(' ')[0].split('/')[-1] for p in PROTECTED) and 'launcher.py' not in cmd:
      out.append(int(d))
  return out


def start_web(c):
  if find(c['radarweb_match']):
    return 'radarweb already running'
  subprocess.Popen(c['radarweb_cmd'], shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL, start_new_session=True)
  return 'radarweb started'


def stop_web(c):
  pids = find(c['radarweb_match'])
  for p in pids:
    try:
      os.kill(p, signal.SIGTERM)
    except OSError:
      pass
  time.sleep(1.5)
  left = find(c['radarweb_match'])
  return 'radarweb stopped (%d process%s)' % (len(pids), '' if len(pids) == 1 else 'es') if not left else 'still running: %s (stop it by hand)' % left


def find_fcw(proc='/proc'):
  """pids of the RUNNING FCW logger only: a python process whose script argument is fcw_shadow.py, not in
  --report mode. (A plain substring match would also hit an ssh shell or editor whose command line
  merely mentions the file name - and stop_fcw would SIGTERM it.)"""
  out = []
  for d in os.listdir(proc):
    if not d.isdigit() or int(d) == os.getpid():
      continue
    try:
      argv = open('%s/%s/cmdline' % (proc, d), 'rb').read().split(b'\0')
    except OSError:
      continue
    argv = [a.decode('utf8', 'replace') for a in argv if a]
    if not argv or 'python' not in os.path.basename(argv[0]) or '--report' in argv:
      continue
    if any(os.path.basename(a) == 'fcw_shadow.py' for a in argv[1:3]):
      out.append(int(d))
  return out


def start_fcw(c=None):
  """FCW shadow logger (logs only, never alerts). Runs from /data/openpilot so cereal imports."""
  if find_fcw():
    return 'FCW logger already running'
  if not os.path.exists(FCW_SHADOW):
    return 'not found: %s' % FCW_SHADOW
  env = dict(os.environ)
  env['PYTHONPATH'] = '/data/openpilot' + ((':' + env['PYTHONPATH']) if env.get('PYTHONPATH') else '')
  subprocess.Popen(['nice', '-n', '10', 'python3', FCW_SHADOW], cwd='/data/openpilot', env=env, stdout=subprocess.DEVNULL,
                   stderr=open('/tmp/fcw_shadow.err', 'w'), stdin=subprocess.DEVNULL, start_new_session=True)
  return 'FCW logger started'


def stop_fcw(c=None):
  pids = find_fcw()
  for p in pids:
    try:
      os.kill(p, signal.SIGTERM)
    except OSError:
      pass
  return 'FCW logger stopped' if pids else 'FCW logger was not running'


def fcw_report(c=None):
  r = subprocess.run(['python3', FCW_SHADOW, '--report'], capture_output=True, text=True, timeout=30)
  return (r.stdout + r.stderr)[-4000:] or 'no report yet'


def diag(c, what):
  sh = os.path.join(c['diag_dir'], 'start_diagnostics.sh' if what == 'start' else 'stop_diagnostics.sh')
  if not os.path.exists(sh):
    return 'not found: %s' % sh
  if what == 'stop':                                   # can take minutes: run detached, the page shows progress via status
    subprocess.Popen(['sh', sh], stdout=open('/tmp/launcher_diag_stop.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    return 'stopping diagnostics (analysis + zip can take minutes, see status)'
  r = subprocess.run(['sh', sh], capture_output=True, text=True, timeout=60)
  return (r.stdout + r.stderr)[-600:]


def status(c):
  sysd = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'traffic')
  sys.path.insert(0, sysd)
  out = {'radarweb': bool(find(c['radarweb_match'])), 'fcw': bool(find_fcw()), 'diag': None, 'procs': [], 'temp_c': None, 'load': open('/proc/loadavg').read().split()[:3]}
  try:
    r = subprocess.run(['sh', os.path.join(c['diag_dir'], 'start_diagnostics.sh'), 'status'], capture_output=True, text=True, timeout=10)
    out['diag'] = r.stdout.strip().splitlines()[0][:200] if r.stdout.strip() else None
  except Exception:
    pass
  try:
    import sysstats
    S = sysstats.Sampler(proc_every=1)
    S.sample()
    time.sleep(1.0)
    s = S.sample()
    if s:
      out['temp_c'] = s.get('temp_max_c')
      out['cpu_total_pct'] = s.get('cpu_total_pct')
      out['procs'] = sorted(((round(v), k) for k, v in s['procs'].items()), reverse=True)[:10]
  except Exception as e:
    out['err'] = str(e)
  try:
    out['stop_log'] = open('/tmp/launcher_diag_stop.log').read()[-300:]
  except OSError:
    pass
  return out


PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>EON launcher</title>
<style>body{font-family:system-ui;background:#0d0f12;color:#cfd6dd;margin:16px}button{font:600 16px system-ui;padding:12px 18px;margin:4px;border-radius:10px;border:0;background:#2a313a;color:#fff}
.on{background:#178644}.off{background:#7a2b2b}pre{background:#161a1f;padding:10px;border-radius:8px;white-space:pre-wrap}h3{margin:16px 0 4px}</style>
<h2>EON launcher</h2><div id=st>loading...</div>
<h3>radarweb (web UI, CAN decode)</h3><button class=on onclick="act('web_start')">Start</button><button class=off onclick="act('web_stop')">Stop</button>
<h3>FCW shadow logger (logs only, never warns)</h3><button class=on onclick="act('fcw_start')">Start</button><button class=off onclick="act('fcw_stop')">Stop</button><button onclick="act('fcw_report')">Report</button>
<h3>steering diagnostics</h3><button class=on onclick="act('diag_start')">Start recording</button><button class=off onclick="act('diag_stop')">Stop + build zip</button>
<pre id=msg></pre><h3>Busiest processes (% of one core)</h3><pre id=pr></pre>
<script>
function act(a){document.getElementById('msg').textContent='...';fetch('/api/'+a,{method:'POST'}).then(function(r){return r.json()}).then(function(d){document.getElementById('msg').textContent=d.msg;upd()})}
function upd(){fetch('/api/status').then(function(r){return r.json()}).then(function(d){
 document.getElementById('st').innerHTML='radarweb: <b>'+(d.radarweb?'RUNNING':'stopped')+'</b> &middot; FCW logger: <b>'+(d.fcw?'RUNNING':'stopped')+'</b> &middot; diagnostics: <b>'+(d.diag||'?')+'</b><br>CPU total '+(d.cpu_total_pct!=null?d.cpu_total_pct.toFixed(0)+'%':'?')+' &middot; temp '+(d.temp_c!=null?d.temp_c+' C':'?')+' &middot; load '+d.load.join(' ')+(d.stop_log?'<pre>'+d.stop_log+'</pre>':'');
 document.getElementById('pr').textContent=d.procs.map(function(p){return ('   '+p[0]).slice(-4)+'  '+p[1]}).join('\\n')})}
upd();setInterval(upd,5000);
</script>"""


class H(BaseHTTPRequestHandler):
  def log_message(self, *a):
    pass

  def _ok(self):
    if not AUTH:
      return True
    h = self.headers.get('Authorization', '')
    if h.startswith('Basic ') and base64.b64decode(h[6:]).decode('utf8', 'replace') == AUTH:
      return True
    self.send_response(401); self.send_header('WWW-Authenticate', 'Basic realm="launcher"'); self.end_headers()
    return False

  def _send(self, body, ct='application/json'):
    b = body if isinstance(body, bytes) else body.encode()
    self.send_response(200); self.send_header('Content-Type', ct); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)

  def do_GET(self):
    if not self._ok():
      return
    if self.path.split('?')[0] == '/api/status':
      self._send(json.dumps(status(conf())))
    else:
      self._send(PAGE, 'text/html')

  def do_POST(self):
    if not self._ok():
      return
    c = conf()
    p = self.path.split('?')[0]
    try:
      msg = {'/api/web_start': lambda: start_web(c), '/api/web_stop': lambda: stop_web(c),
             '/api/diag_start': lambda: diag(c, 'start'), '/api/diag_stop': lambda: diag(c, 'stop'),
             '/api/fcw_start': lambda: start_fcw(c), '/api/fcw_stop': lambda: stop_fcw(c), '/api/fcw_report': lambda: fcw_report(c)}[p]()
    except KeyError:
      msg = 'unknown action'
    except Exception as e:
      msg = 'error: %s' % e
    self._send(json.dumps({'msg': msg}))


if __name__ == '__main__':
  port = int(sys.argv[1]) if len(sys.argv) > 1 else 8087
  print('launcher on port %d (conf %s)' % (port, CONF))
  HTTPServer(('0.0.0.0', port), H).serve_forever()
