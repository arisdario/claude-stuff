#!/usr/bin/env python3
"""log_uploader.py - send finished drive logs from the EON to a PRIVATE GitHub repository by itself, so nobody has to pull
them off the EON with a laptop. Files go up as compressed GitHub *release assets* (one release per month, e.g. logs-2026-11),
NOT as git commits: no 100 MB file limit, nothing piles up in git history, and an asset can be deleted any time.

  when:   only while openpilot is OFF-ROAD (controlsd not running for 2 min = car off / parked) and the EON's internet goes
          over Wi-Fi (default route on wlan0). The moment controlsd starts (you start driving) the running upload is
          aborted and retried next time. Runs at the lowest CPU and disk priority.
  what:   finished files only (not written for 10 min):
            /data/maplog/map_*.jsonl          radarweb map logs (the biggest: ~10x smaller compressed)
            /data/drivelog_*.jsonl            driving_logger.py
            /data/interventions/ev_*.json     radarevents.py
            /data/lightlog/ll_*.csv           diag/lightlog.py
            /data/fcw_shadow.jsonl            grows forever: a dated snapshot is sent when it changed
          each file once (state in /data/log_upload_state.json); a file that grew is sent again under a new name.
  setup (once, see the chat):
            /data/github_log_repo   one line: owner/name of a PRIVATE repo that has at least one commit (e.g. a README)
            /data/github_token      a fine-grained token with access to ONLY that repo, permission Contents: read and write
                                    (chmod 600). Lost EON = revoke the token on github.com, nothing else is exposed.
  options (create the file to switch on):
            /data/log_upload_off        stop uploading
            /data/log_upload_strip_gps  remove GPS (latitude/longitude, MCU_locationStatus, 0x3D8 raw frames) before upload
            /data/log_upload_delete     delete a log from the EON after it was uploaded (never fcw_shadow.jsonl)
            /data/log_upload_ssids      Wi-Fi names, one per line: only upload on these (e.g. home), never on a phone hotspot
  run:      nohup nice -n 19 python3 /data/claude-stuff/tesla/tools/log_uploader.py > /dev/null 2>&1 &
            (auto-start: add  (python3 /data/claude-stuff/tesla/tools/log_uploader.py > /dev/null 2>&1 &)  to
             /data/openpilot/launch_chffrplus.sh next to the lightdash_tunnel line)
  check:    python3 log_uploader.py --status     what is waiting / sent / why it is not uploading now
            python3 log_uploader.py --once       one pass now, ignoring the off-road/Wi-Fi wait (still aborts when driving)
  log:      /data/log_upload.log

It never touches openpilot, its processes or its messages: it reads files and /proc only. Python 3.7 standard library.
"""
import glob
import gzip
import json
import os
import shutil
import ssl
import subprocess
import sys
import time
import urllib.error
import urllib.request

REPO_FILE = '/data/github_log_repo'
TOKEN_FILE = '/data/github_token'
STATE_FILE = '/data/log_upload_state.json'
LOG_FILE = '/data/log_upload.log'
TMP_DIR = '/data/log_upload_tmp'
OFF = '/data/log_upload_off'
STRIP = '/data/log_upload_strip_gps'
DELETE = '/data/log_upload_delete'
SSIDS = '/data/log_upload_ssids'
SOURCES = ['/data/maplog/map_*.jsonl', '/data/drivelog_*.jsonl', '/data/interventions/ev_*.json', '/data/lightlog/ll_*.csv']
GROWING = ['/data/fcw_shadow.jsonl']          # appended forever: snapshots, never deleted
QUIET_S = 600                                 # a file not written for this long is finished
OFFROAD_S = 120                               # controlsd gone for this long = parked
PASS_EVERY_S = 300
API = 'https://api.github.com'
GPS_KEYS = {'gps', 'lat', 'lon', 'latitude', 'longitude', 'MCU_locationStatus', 'MCU_locationStatus2', '0x3d8', '0x3D8', 'r0x3d8'}


class Abort(Exception):
  pass


def log(msg):
  line = '%s %s' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg)
  try:
    with open(LOG_FILE, 'a') as f:
      f.write(line + '\n')
  except Exception:
    pass
  if sys.stdout.isatty():
    print(line)


# --------------------------------------------------------------------------- when is it OK to upload
def onroad():
  """controlsd only runs while openpilot is started (car on). Read /proc: `ps` from Python on NEOS may list only this terminal."""
  for pid in os.listdir('/proc'):
    if not pid.isdigit():
      continue
    try:
      c = open('/proc/%s/cmdline' % pid, 'rb').read().split(b'\0')[0].decode('utf-8', 'replace')
    except Exception:
      continue
    if os.path.basename(c).endswith('controlsd'):
      return True
  return False


def default_iface(route_text=None):
  """the interface of the default route from /proc/net/route (wlan0 = Wi-Fi, rmnet* = mobile data)"""
  try:
    text = route_text if route_text is not None else open('/proc/net/route').read()
  except Exception:
    return None
  for line in text.splitlines()[1:]:
    f = line.split()
    if len(f) > 2 and f[1] == '00000000':
      return f[0]
  return None


def current_ssid(dumpsys_text=None):
  """the connected Wi-Fi name from `dumpsys wifi` (Android); None if it cannot be read"""
  try:
    text = dumpsys_text if dumpsys_text is not None else subprocess.check_output(['dumpsys', 'wifi'], timeout=10).decode('utf-8', 'replace')
  except Exception:
    return None
  for line in text.splitlines():
    if 'mWifiInfo' in line and 'SSID:' in line:
      s = line.split('SSID:', 1)[1].split(',')[0].strip().strip('"')
      return s or None
  return None


def network_ok():
  """(ok, why). Wi-Fi only; with /data/log_upload_ssids also only those Wi-Fi names (fails closed if the name can't be read)"""
  iface = default_iface()
  if iface != 'wlan0':
    return False, 'no Wi-Fi (default route: %s)' % iface
  if os.path.exists(SSIDS):
    allowed = {l.strip() for l in open(SSIDS) if l.strip()}
    ssid = current_ssid()
    if ssid not in allowed:
      return False, 'Wi-Fi %r is not in %s' % (ssid, SSIDS)
  return True, 'Wi-Fi'


# --------------------------------------------------------------------------- what to upload
def load_state():
  try:
    return json.load(open(STATE_FILE))
  except Exception:
    return {}


def save_state(st):
  tmp = STATE_FILE + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(st, f, indent=0, sort_keys=True)
  os.rename(tmp, STATE_FILE)


def pending(st, now=None, sources=None, growing=None):
  """[(path, asset_name, growing)] of finished files not uploaded in their current size"""
  now = time.time() if now is None else now
  out = []
  for pats, grow in ((SOURCES if sources is None else sources, False), (GROWING if growing is None else growing, True)):
    for pat in pats:
      for p in sorted(glob.glob(pat)):
        try:
          s = os.stat(p)
        except OSError:
          continue
        if s.st_size == 0 or now - s.st_mtime < QUIET_S:
          continue
        prev = st.get(p)
        if prev and prev.get('size') == s.st_size:
          continue
        base = os.path.basename(p)
        if grow:
          stem, ext = os.path.splitext(base)
          name = '%s_%s%s.gz' % (stem, time.strftime('%Y%m%d_%H%M', time.localtime(s.st_mtime)), ext)
        elif prev:                                   # a finished file that grew after it was sent: send the new size too
          stem, ext = os.path.splitext(base)
          name = '%s_%d%s.gz' % (stem, s.st_size, ext)
        else:
          name = base + '.gz'
        out.append((p, name, grow))
  return out


def strip_gps(obj):
  if isinstance(obj, dict):
    return {k: strip_gps(v) for k, v in obj.items() if k not in GPS_KEYS}
  if isinstance(obj, list):
    return [strip_gps(v) for v in obj]
  return obj


def compress(src, dst, strip, stop):
  """gzip src -> dst (optionally without GPS for .json/.jsonl). stop() is checked every few MB: raises Abort"""
  n = 0
  with open(src, 'rb') as fi, gzip.open(dst, 'wb', compresslevel=6) as fo:
    if strip and src.endswith('.jsonl'):
      for line in fi:
        try:
          line = (json.dumps(strip_gps(json.loads(line)), separators=(',', ':')) + '\n').encode()
        except Exception:
          pass                                       # not JSON: keep the line as it is
        fo.write(line); n += len(line)
        if n > 4 << 20:
          n = 0
          if stop():
            raise Abort()
    elif strip and src.endswith('.json'):
      try:
        fo.write(json.dumps(strip_gps(json.load(fi))).encode())
      except Exception:
        fi.seek(0); shutil.copyfileobj(fi, fo)
    else:
      while True:
        b = fi.read(1 << 20)
        if not b:
          break
        fo.write(b); n += len(b)
        if n > 4 << 20:
          n = 0
          if stop():
            raise Abort()


# --------------------------------------------------------------------------- GitHub
def ssl_context():
  for f in ('/data/cacert.pem', '/data/data/com.termux/files/usr/etc/tls/cert.pem'):
    if os.path.exists(f) and os.path.getsize(f) > 0:
      return ssl.create_default_context(cafile=f)
  try:
    import certifi
    return ssl.create_default_context(cafile=certifi.where())
  except Exception:
    return ssl.create_default_context()


class StopReader(object):
  """file wrapper for the upload body: checks stop() every 4 MB so a drive aborts the upload"""
  def __init__(self, f, stop):
    self.f, self.stop, self.n = f, stop, 0

  def read(self, size=-1):
    b = self.f.read(size if size and size > 0 else 1 << 16)
    self.n += len(b)
    if self.n > 4 << 20:
      self.n = 0
      if self.stop():
        raise Abort()
    return b


class GitHub(object):
  def __init__(self, repo, token, ctx=None):
    self.repo, self.token, self.ctx = repo, token, ctx or ssl_context()

  def call(self, method, url, body=None, headers=None, length=None, timeout=60):
    h = {'Authorization': 'Bearer ' + self.token, 'Accept': 'application/vnd.github+json', 'User-Agent': 'eon-log-uploader',
         'X-GitHub-Api-Version': '2022-11-28'}
    h.update(headers or {})
    data = body
    if isinstance(body, (dict, list)):
      data = json.dumps(body).encode(); h['Content-Type'] = 'application/json'
    if length is not None:
      h['Content-Length'] = str(length)
    req = urllib.request.Request(url if url.startswith('http') else API + url, data=data, headers=h, method=method)
    try:
      with urllib.request.urlopen(req, context=self.ctx, timeout=timeout) as r:
        t = r.read()
        return r.status, (json.loads(t) if t else None)
    except urllib.error.HTTPError as e:
      try:
        t = json.loads(e.read() or b'null')
      except Exception:
        t = None
      return e.code, t

  def release(self, tag):
    code, r = self.call('GET', '/repos/%s/releases/tags/%s' % (self.repo, tag))
    if code == 200:
      return r
    code, r = self.call('POST', '/repos/%s/releases' % self.repo,
                        {'tag_name': tag, 'name': tag, 'body': 'Drive logs uploaded by the EON (tesla/tools/log_uploader.py).'})
    if code != 201:
      raise RuntimeError('cannot create release %s: HTTP %s %s (does the repo have a first commit, e.g. a README?)' % (tag, code, (r or {}).get('message')))
    return r

  def upload(self, rel, name, path, stop):
    for a in rel.get('assets') or []:               # same name already there (e.g. an earlier upload broke off): replace it
      if a.get('name') == name:
        self.call('DELETE', '/repos/%s/releases/assets/%d' % (self.repo, a['id']))
    url = rel['upload_url'].split('{')[0] + '?name=' + urllib.request.quote(name)
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
      code, r = self.call('POST', url, StopReader(f, stop), {'Content-Type': 'application/gzip'}, length=size, timeout=600)
    if code != 201:
      raise RuntimeError('upload %s: HTTP %s %s' % (name, code, (r or {}).get('message')))
    return r


def read_config():
  try:
    repo = open(REPO_FILE).read().strip()
    token = open(TOKEN_FILE).read().strip()
  except Exception:
    return None, None
  if repo.count('/') != 1 or not token:
    return None, None
  return repo, token


# --------------------------------------------------------------------------- one pass
def run_pass(gh, st, stop, now=None):
  """uploads what is pending; returns (sent, failed). Aborts (raises Abort) when stop() turns true."""
  sent = failed = 0
  todo = pending(st, now)
  if not todo:
    return 0, 0
  os.makedirs(TMP_DIR, exist_ok=True)
  rels = {}
  for path, name, grow in todo:
    if stop():
      raise Abort()
    tmp = os.path.join(TMP_DIR, name)
    try:
      size, mtime = os.path.getsize(path), os.path.getmtime(path)
      compress(path, tmp, os.path.exists(STRIP), stop)
      tag = time.strftime('logs-%Y-%m', time.localtime(mtime))
      if tag not in rels:
        rels[tag] = gh.release(tag)
      a = gh.upload(rels[tag], name, tmp, stop)
      rels[tag].setdefault('assets', []).append(a)
      st[path] = {'size': size, 'asset': name, 'tag': tag, 't': round(time.time())}
      save_state(st); sent += 1
      log('sent %s -> %s/%s (%d kB -> %d kB)' % (path, tag, name, size // 1024, os.path.getsize(tmp) // 1024))
      if not grow and os.path.exists(DELETE):
        os.remove(path); log('deleted %s from the EON (%s)' % (path, DELETE))
    except Abort:
      raise
    except Exception as e:
      failed += 1; log('FAILED %s: %s' % (path, e))
    finally:
      try:
        os.remove(tmp)
      except OSError:
        pass
  return sent, failed


def status():
  repo, token = read_config()
  print('repo:    %s' % (repo or 'NOT SET (%s)' % REPO_FILE))
  print('token:   %s' % ('set' if token else 'NOT SET (%s)' % TOKEN_FILE))
  print('off:     %s' % ('YES (%s)' % OFF if os.path.exists(OFF) else 'no'))
  print('driving: %s' % ('yes (controlsd running): waits' if onroad() else 'no'))
  ok, why = network_ok(); print('network: %s%s' % (why, '' if ok else ': waits'))
  print('options: strip GPS %s, delete after upload %s, Wi-Fi names %s' % (
      'on' if os.path.exists(STRIP) else 'off', 'on' if os.path.exists(DELETE) else 'off', 'limited' if os.path.exists(SSIDS) else 'any'))
  st = load_state(); todo = pending(st)
  print('sent:    %d files' % len(st))
  print('waiting: %d files, %.1f MB' % (len(todo), sum(os.path.getsize(p) for p, _, _ in todo) / 1e6))
  for p, n, _ in todo[:20]:
    print('   %s' % p)


def main(argv):
  if '--status' in argv:
    status(); return 0
  try:
    os.nice(19)
  except Exception:
    pass
  try:
    subprocess.call(['ionice', '-c', '3', '-p', str(os.getpid())], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  except Exception:
    pass
  once = '--once' in argv
  offroad_since = None if not once else 0.0
  log('started%s' % (' (one pass)' if once else ''))
  while True:
    if os.path.exists(OFF) and not once:
      time.sleep(PASS_EVERY_S); continue
    now = time.time()
    if onroad():
      offroad_since = None
    elif offroad_since is None:
      offroad_since = now
    repo, token = read_config()
    ok, why = network_ok()
    ready = repo and offroad_since is not None and (once or now - offroad_since >= OFFROAD_S) and (ok or once)
    if ready:
      try:
        sent, failed = run_pass(GitHub(repo, token), load_state(), onroad)
        if sent or failed:
          log('pass done: %d sent, %d failed' % (sent, failed))
      except Abort:
        log('car started: upload stopped, will continue when parked')
      except Exception as e:
        log('pass failed: %s' % e)
    if once:
      return 0
    time.sleep(PASS_EVERY_S)


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
