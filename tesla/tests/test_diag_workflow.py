"""End-to-end: the real shell scripts + the real collector process against a time-driven fake cereal (no EON needed)."""
import io, json, os, signal, subprocess, sys, tempfile, threading, time, zipfile, glob, shutil
HERE = os.path.dirname(os.path.abspath(__file__))
DIAG = os.path.join(HERE, '..', 'diag')


def _env(base, jpg, extra=None):
  e = dict(os.environ, DIAG_BASE=base, DIAG_PYTHON=sys.executable, DIAG_PARAMS=base, DIAG_LIVE_JPG=jpg,
           PYTHONPATH=os.path.join(HERE, 'fake_cereal'))
  e.update(extra or {})
  return e


def _jpeg_writer(path, stop):
  from PIL import Image
  import random
  n = 0
  while not stop.is_set():
    im = Image.effect_noise((320, 180), 40 + (n % 5) * 10).convert('RGB')
    b = io.BytesIO(); im.save(b, 'JPEG')
    open(path + '.t', 'wb').write(b.getvalue()); os.rename(path + '.t', path)
    n += 1
    time.sleep(0.1)


def _sh(script, env, timeout=200):
  return subprocess.run(['sh', os.path.join(DIAG, script)], env=env, capture_output=True, text=True, timeout=timeout, cwd='/')


def test_start_run_stop_zip():
  base = tempfile.mkdtemp(prefix='diagwf_')
  jpg = os.path.join(base, 'live.jpg')
  json.dump({'pre_s': 5, 'post_s': 5, 'seg_s': 5, 'keep_s': 30, 'frame_keep_s': 30, 'min_free_mb': 10, 'hard_free_mb': 5},
            open(os.path.join(base, 'config.json'), 'w'))
  stop = threading.Event()
  th = threading.Thread(target=_jpeg_writer, args=(jpg, stop)); th.daemon = True; th.start()
  env = _env(base, jpg, {'FAKE_OSC': '10,28', 'FAKE_FRAME_DROP': '5,5.3', 'FAKE_CAN_GAP': '32,33'})
  try:
    r = _sh('start_diagnostics.sh', env)
    assert 'RECORDING ACTIVE' in r.stdout, r.stdout + r.stderr
    st = _sh('start_diagnostics.sh', env)               # second start refuses
    assert st.returncode != 0 or 'already' in st.stdout
    s2 = subprocess.run(['sh', os.path.join(DIAG, 'start_diagnostics.sh'), 'status'], env=env, capture_output=True, text=True)
    assert 'RECORDING ACTIVE' in s2.stdout
    time.sleep(42)
    r = _sh('stop_diagnostics.sh', env)
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'SEND ME THIS FILE' in r.stdout, r.stdout
  finally:
    stop.set()
  sessions = sorted(glob.glob(os.path.join(base, 'drive_*')))
  sessions = [s for s in sessions if os.path.isdir(s)]
  assert len(sessions) == 1
  s = sessions[0]
  for d in ('metadata', 'camera', 'model', 'pose', 'calibration', 'can', 'localization', 'steering', 'performance', 'events', 'plots'):
    assert os.path.isdir(os.path.join(s, d)), d
  for f in ('summary.txt', 'README.txt', 'manifest.json', 'root_cause_report.txt'):
    assert os.path.exists(os.path.join(s, f)), f
  z = zipfile.ZipFile(s + '.zip')
  assert z.testzip() is None
  names = set(z.namelist())
  man = json.load(open(os.path.join(s, 'manifest.json')))
  listed = {os.path.join(os.path.basename(s), f['path']) for f in man['files']}
  assert listed <= names and len(listed) > 20
  undescribed = [f['path'] for f in man['files'] if not f['what']]
  assert not undescribed, undescribed
  ev = glob.glob(os.path.join(s, 'events', 'event_*'))
  kinds = set()
  for e in ev:
    kinds |= set(json.load(open(os.path.join(e, 'summary.json')))['kinds'])
  assert 'steering_oscillation' in kinds, kinds
  assert 'camera_frame_drop' in kinds, kinds
  assert 'can_timeout' in kinds or 'can_gap' in kinds, kinds
  assert any(glob.glob(os.path.join(e, 'frames', '*.jpg')) for e in ev), 'no camera frames preserved around events'
  rep = open(os.path.join(s, 'root_cause_report.txt')).read()
  assert 'downstream' in rep, rep
  summ = open(os.path.join(s, 'summary.txt')).read()
  assert 'logger CPU' in summ and 'dropped records 0' in summ, summ
  assert open(os.path.join(base, 'DiagEnabled')).read().strip() == '0'
  shutil.rmtree(base)


def test_crash_then_finalize():
  base = tempfile.mkdtemp(prefix='diagwf2_')
  jpg = os.path.join(base, 'live.jpg')
  env = _env(base, jpg)
  r = _sh('start_diagnostics.sh', env)
  assert 'RECORDING ACTIVE' in r.stdout, r.stdout + r.stderr
  pid = int(open(os.path.join(base, '.collector.pid')).read())
  time.sleep(6)
  os.kill(pid, signal.SIGKILL)                      # power loss / crash
  time.sleep(1)
  r = _sh('stop_diagnostics.sh', env)
  assert r.returncode == 0 and 'not running' in r.stdout, r.stdout + r.stderr
  s = glob.glob(os.path.join(base, 'drive_*.zip'))
  assert len(s) == 1 and zipfile.ZipFile(s[0]).testzip() is None
  shutil.rmtree(base)
