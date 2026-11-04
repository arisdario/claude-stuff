import os, sys, tempfile, time, subprocess, json, threading, urllib.request
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import launcher


def test_start_stop_only_matching_process_and_never_protected():
  d = tempfile.mkdtemp()
  conf = os.path.join(d, 'l.conf')
  tag = 'fakeweb_%d' % os.getpid()
  open(conf, 'w').write('radarweb_cmd=sleep 300 # %s\nradarweb_match=sleep 300\ndiag_dir=%s\n' % (tag, d))
  os.environ['LAUNCHER_CONF'] = conf; launcher.CONF = conf
  c = launcher.conf()
  assert 'started' in launcher.start_web(c)
  time.sleep(0.3)
  assert launcher.find(c['radarweb_match'])
  assert 'already' in launcher.start_web(c)
  assert 'stopped' in launcher.stop_web(c)
  assert not launcher.find(c['radarweb_match'])
  # a protected openpilot name in the match list is never returned
  p = subprocess.Popen(['sh', '-c', 'exec -a controlsd sleep 5'])
  time.sleep(0.3)
  assert p.pid not in launcher.find('sleep,controlsd')
  p.kill()


def test_http_status_and_unknown_action():
  from http.server import HTTPServer
  d = tempfile.mkdtemp(); conf = os.path.join(d, 'l.conf'); open(conf, 'w').write('diag_dir=%s\n' % d); launcher.CONF = conf
  srv = HTTPServer(('127.0.0.1', 0), launcher.H); port = srv.server_address[1]
  threading.Thread(target=srv.serve_forever, daemon=True).start()
  s = json.load(urllib.request.urlopen('http://127.0.0.1:%d/api/status' % port))
  assert 'radarweb' in s and 'load' in s
  r = urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:%d/api/nope' % port, method='POST'))
  assert 'unknown' in json.load(r)['msg']
  srv.shutdown()
