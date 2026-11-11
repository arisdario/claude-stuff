import os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import param_switch as ps


def test_toggles_drive_actions_once_per_change():
  d = tempfile.mkdtemp()
  calls = []
  fake = dict(start_web=lambda c: calls.append('web_start') or 'ok', stop_web=lambda c: calls.append('web_stop') or 'ok',
              diag=lambda c, w: calls.append('diag_' + w) or 'ok')
  sw = ps.Switch({'switch_param': 'IsUploadRawEnabled'}, d)
  assert sw.step(**fake) == [] and calls == []                       # no params yet: nothing happens
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('0')
  sw.step(**fake); assert calls == []                                # a 0 at startup does not stop anything
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('1')
  sw.step(**fake); sw.step(**fake); assert calls == ['web_start']    # acts once per change, not every poll
  open(os.path.join(d, 'RecordFront'), 'w').write('0'); sw.step(**fake)
  open(os.path.join(d, 'RecordFront'), 'w').write('1'); sw.step(**fake)
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('0'); sw.step(**fake)
  open(os.path.join(d, 'RecordFront'), 'w').write('0'); sw.step(**fake)
  assert calls == ['web_start', 'diag_start', 'web_stop', 'diag_stop'], calls


def test_toggle_left_on_at_boot_starts_radarweb_and_names_are_configurable():
  d = tempfile.mkdtemp()
  open(os.path.join(d, 'MyToggle'), 'w').write('1')
  calls = []
  sw = ps.Switch({'switch_param': 'MyToggle'}, d)
  sw.step(start_web=lambda c: calls.append('web_start') or 'ok', stop_web=lambda c: 'x', diag=lambda c, w: 'x')
  assert calls == ['web_start']


def test_binary_params_do_not_crash():
  d = tempfile.mkdtemp()
  open(os.path.join(d, 'Blob'), 'wb').write(b'\x12\x9f\xff\x00binary')
  os.mkdir(os.path.join(d, 'subdir'))
  assert isinstance(ps.rd('Blob', d), str) and ps.rd('subdir', d) is None and ps.rd('missing', d) is None
  assert ps.Switch({'switch_param': 'IsUploadRawEnabled'}, d).step(start_web=lambda c: 'x', stop_web=lambda c: 'x', diag=lambda c, w: 'x') == []


def test_default_switch_is_the_on_screen_web_button_file():
  d = tempfile.mkdtemp()
  f = os.path.join(d, 'web_switch')
  calls = []
  sw = ps.Switch({'switch_param': f}, d)
  open(f, 'w').write('1\n')
  sw.step(start_web=lambda c: calls.append('on') or 'ok', stop_web=lambda c: calls.append('off') or 'ok', diag=lambda c, w: 'x')
  open(f, 'w').write('0\n')
  sw.step(start_web=lambda c: calls.append('on') or 'ok', stop_web=lambda c: calls.append('off') or 'ok', diag=lambda c, w: 'x')
  assert calls == ['on', 'off']


def test_c920_audio_unbind_touches_only_audio_interfaces():
  root = tempfile.mkdtemp()
  usb, drvdir = os.path.join(root, 'devices'), os.path.join(root, 'drivers')
  os.makedirs(os.path.join(drvdir, 'snd-usb-audio')); os.makedirs(os.path.join(drvdir, 'uvcvideo'))
  def mk(dev, vid, pid, itfs):
    p = os.path.join(usb, dev); os.makedirs(p)
    open(os.path.join(p, 'idVendor'), 'w').write(vid); open(os.path.join(p, 'idProduct'), 'w').write(pid)
    for name, cls, drv in itfs:
      ip = os.path.join(p, name); os.makedirs(ip)
      open(os.path.join(ip, 'bInterfaceClass'), 'w').write(cls)
      if drv: os.symlink(os.path.join(drvdir, drv), os.path.join(ip, 'driver'))
  mk('1-1.1', '046d', '082d', [('1-1.1:1.0', '0e', 'uvcvideo'), ('1-1.1:1.1', '0e', 'uvcvideo'), ('1-1.1:1.2', '01', 'snd-usb-audio'), ('1-1.1:1.3', '01', 'snd-usb-audio')])
  mk('1-1.2', '1234', '5678', [('1-1.2:1.0', '01', 'snd-usb-audio')])            # some other USB sound card: must not be touched
  unbind = os.path.join(drvdir, 'snd-usb-audio', 'unbind')
  done = ps.unbind_c920_audio(usb, unbind)
  assert sorted(done) == ['1-1.1:1.2', '1-1.1:1.3']
  assert '1-1.2:1.0' not in done and not any('1.0' in x or '1.1' == x[-3:] for x in done)
  assert ps.unbind_c920_audio(usb, os.path.join(root, 'nodir', 'unbind')) == []       # no unbind file: no crash, nothing reported


def _fake(calls):
  return dict(start_web=lambda c: calls.append('web_start') or 'ok', stop_web=lambda c: calls.append('web_stop') or 'ok',
              diag=lambda c, w: calls.append('diag_' + w) or 'ok',
              start_fcw=lambda c: calls.append('fcw_start') or 'ok', stop_fcw=lambda c: calls.append('fcw_stop') or 'ok')


def test_fcw_logger_follows_its_toggle():
  d = tempfile.mkdtemp(); calls = []
  sw = ps.Switch({}, d)                                               # defaults: web=/data/web_switch, fcw=IsUploadRawEnabled
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('1'); sw.step(**_fake(calls)); sw.step(**_fake(calls))
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('0'); sw.step(**_fake(calls))
  assert calls == ['fcw_start', 'fcw_stop'], calls


def test_fcw_toggle_left_on_at_boot_starts_the_logger():
  d = tempfile.mkdtemp(); calls = []
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('1')
  ps.Switch({}, d).step(**_fake(calls))
  assert calls == ['fcw_start']


def test_fcw_never_shares_a_toggle_with_radarweb_or_diag():
  d = tempfile.mkdtemp(); calls = []
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('1')
  ps.Switch({'switch_param': 'IsUploadRawEnabled'}, d).step(**_fake(calls))
  assert calls == ['web_start'], calls                               # same toggle as radarweb: FCW switch is skipped


def test_fcw_switch_can_be_disabled():
  d = tempfile.mkdtemp(); calls = []
  open(os.path.join(d, 'IsUploadRawEnabled'), 'w').write('1')
  ps.Switch({'fcw_param': 'none'}, d).step(**_fake(calls))
  assert calls == []


def test_find_fcw_matches_only_the_running_logger():
  import launcher
  root = tempfile.mkdtemp()
  def mk(pid, *argv):
    os.makedirs(os.path.join(root, str(pid)))
    open(os.path.join(root, str(pid), 'cmdline'), 'wb').write(b'\0'.join(a.encode() for a in argv) + b'\0')
  mk(101, 'python3', '/data/claude-stuff/tesla/tools/fcw_shadow.py')                 # the logger
  mk(102, 'nice', '-n', '10', 'python3', 'fcw_shadow.py')                            # nice wrapper exec's python: this pid is not python
  mk(103, 'python3', '/data/claude-stuff/tesla/tools/fcw_shadow.py', '--report')      # a report run
  mk(104, '-bash', '-c', 'python3 fcw_shadow.py --report; vi fcw_shadow.py')           # an ssh shell mentioning it
  mk(105, 'vi', 'fcw_shadow.py')
  assert launcher.find_fcw(root) == [101]
