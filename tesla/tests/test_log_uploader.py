#!/usr/bin/env python3
"""tools/log_uploader.py: what counts as finished, Wi-Fi/route detection, GPS stripping, and a full pass against a small fake
GitHub API server (release created once per month, assets uploaded, state saved, a drive aborts the pass)."""
import gzip
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import log_uploader as U  # noqa: E402

ROUTE_WIFI = 'Iface\tDestination\tGateway\nwlan0\t0001A8C0\t00000000\nwlan0\t00000000\t0101A8C0\n'
ROUTE_LTE = 'Iface\tDestination\tGateway\nrmnet_data0\t00000000\t01020304\n'
DUMPSYS = 'Wi-Fi is enabled\nmWifiInfo SSID: "HomeNet", BSSID: 11:22:33:44:55:66, Supplicant state: COMPLETED, RSSI: -50\n'


class Fake(BaseHTTPRequestHandler):
  releases = {}; assets = []; calls = []

  def log_message(self, *a):
    pass

  def _send(self, code, obj):
    b = json.dumps(obj).encode()
    self.send_response(code); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b)))
    self.end_headers(); self.wfile.write(b)

  def do_GET(self):
    Fake.calls.append(('GET', self.path))
    tag = self.path.rsplit('/', 1)[-1]
    if '/releases/tags/' in self.path and tag in Fake.releases:
      return self._send(200, Fake.releases[tag])
    self._send(404, {'message': 'Not Found'})

  def do_POST(self):
    n = int(self.headers.get('Content-Length', 0)); body = self.rfile.read(n)
    Fake.calls.append(('POST', self.path))
    assert self.headers['Authorization'] == 'Bearer tok'
    if self.path.endswith('/releases'):
      tag = json.loads(body)['tag_name']
      Fake.releases[tag] = {'id': len(Fake.releases) + 1, 'tag_name': tag, 'assets': [],
                            'upload_url': 'http://127.0.0.1:%d/upload/%s{?name,label}' % (self.server.server_port, tag)}
      return self._send(201, Fake.releases[tag])
    if self.path.startswith('/upload/'):
      name = self.path.split('name=')[1]
      Fake.assets.append((name, gzip.decompress(body)))
      return self._send(201, {'id': len(Fake.assets), 'name': name})
    self._send(404, {})

  def do_DELETE(self):
    Fake.calls.append(('DELETE', self.path)); self._send(204, {})


class T(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.orig = {k: getattr(U, k) for k in ('SOURCES', 'GROWING', 'STATE_FILE', 'TMP_DIR', 'LOG_FILE', 'STRIP', 'DELETE', 'API')}
    U.SOURCES = [os.path.join(self.d, 'map_*.jsonl')]; U.GROWING = [os.path.join(self.d, 'fcw_shadow.jsonl')]
    U.STATE_FILE = os.path.join(self.d, 'state.json'); U.TMP_DIR = os.path.join(self.d, 'tmp'); U.LOG_FILE = os.path.join(self.d, 'log')
    U.STRIP = os.path.join(self.d, 'strip'); U.DELETE = os.path.join(self.d, 'delete')

  def tearDown(self):
    for k, v in self.orig.items():
      setattr(U, k, v)
    shutil.rmtree(self.d)

  def mk(self, name, text, age=3600):
    p = os.path.join(self.d, name)
    with open(p, 'w') as f:
      f.write(text)
    t = time.time() - age; os.utime(p, (t, t))
    return p

  def test_network_detection(self):
    self.assertEqual(U.default_iface(ROUTE_WIFI), 'wlan0')
    self.assertEqual(U.default_iface(ROUTE_LTE), 'rmnet_data0')
    self.assertIsNone(U.default_iface('Iface\tDestination\n'))
    self.assertEqual(U.current_ssid(DUMPSYS), 'HomeNet')
    self.assertIsNone(U.current_ssid('nothing here'))

  def test_only_finished_files_once(self):
    self.mk('map_1.jsonl', '{"t":1}\n')
    self.mk('map_2.jsonl', '{"t":2}\n', age=30)                       # still being written
    self.mk('map_3.jsonl', '', age=3600)                              # empty
    todo = U.pending({})
    self.assertEqual([os.path.basename(p) for p, _, _ in todo], ['map_1.jsonl'])
    p = todo[0][0]
    self.assertEqual(U.pending({p: {'size': os.path.getsize(p)}}), [])          # already sent
    self.assertEqual(U.pending({p: {'size': 1}})[0][1], 'map_1_%d.jsonl.gz' % os.path.getsize(p))   # grew: new name

  def test_strip_gps(self):
    row = {'t': 1, 'gps': {'lat': 44.1}, 'msgs': {'MCU_locationStatus': {'MCU_latitude': 44.1}, 'DI_state': {'x': 1}},
           'raw': {'0x3d8': 'aa', '0x108': 'bb'}, 'o': [[1, 2.0]]}
    s = U.strip_gps(row)
    self.assertNotIn('gps', s); self.assertNotIn('MCU_locationStatus', s['msgs']); self.assertNotIn('0x3d8', s['raw'])
    self.assertEqual(s['raw']['0x108'], 'bb'); self.assertEqual(s['o'], [[1, 2.0]])

  def test_full_pass_against_fake_github(self):
    srv = HTTPServer(('127.0.0.1', 0), Fake); threading.Thread(target=srv.serve_forever, daemon=True).start()
    Fake.releases, Fake.assets, Fake.calls = {}, [], []
    U.API = 'http://127.0.0.1:%d' % srv.server_port
    try:
      a = self.mk('map_1.jsonl', '{"t":1,"gps":{"lat":44.1},"v":3}\n' * 100)
      self.mk('fcw_shadow.jsonl', '{"ev":"session"}\n')
      open(U.STRIP, 'w').close(); open(U.DELETE, 'w').close()
      st = {}
      sent, failed = U.run_pass(U.GitHub('me/logs', 'tok', ctx=None), st, lambda: False)
      self.assertEqual((sent, failed), (2, 0))
      names = dict(Fake.assets)
      self.assertEqual(names['map_1.jsonl.gz'].decode().splitlines()[0], '{"t":1,"v":3}')   # GPS stripped
      self.assertTrue(any(n.startswith('fcw_shadow_') for n in names))
      self.assertEqual(len(Fake.releases), 1)                                          # one release for the month
      self.assertFalse(os.path.exists(a))                                              # deleted after upload (option)
      self.assertTrue(os.path.exists(os.path.join(self.d, 'fcw_shadow.jsonl')))         # growing file never deleted
      self.assertEqual(json.load(open(U.STATE_FILE)), st)
      self.assertEqual(U.run_pass(U.GitHub('me/logs', 'tok', ctx=None), st, lambda: False), (0, 0))   # nothing new
      self.mk('map_2.jsonl', 'x' * 100)
      with self.assertRaises(U.Abort):                                                 # car started: stop at once
        U.run_pass(U.GitHub('me/logs', 'tok', ctx=None), st, lambda: True)
      self.assertNotIn(os.path.join(self.d, 'map_2.jsonl'), st)
    finally:
      srv.shutdown(); srv.server_close()


if __name__ == '__main__':
  unittest.main()
