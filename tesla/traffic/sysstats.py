"""Resource sampling for the EON (stdlib only, Python 3.6+). Reads /proc and /sys; writes nothing.

Everything takes its root directories as parameters so it can be tested on a fake tree. No openpilot/cereal import here.
Advisory tooling only: nothing in the traffic package may import vehicle-control code (a test checks it)."""
import glob
import os
import time

try:
  CLK_TCK = os.sysconf('SC_CLK_TCK')
except (AttributeError, ValueError, OSError):
  CLK_TCK = 100


def parse_proc_stat(text):
  """-> (total (busy, all) over all cpus, {core: (busy, all)}). busy = all - idle - iowait."""
  total, cores = None, {}
  for line in text.splitlines():
    if not line.startswith('cpu'):
      continue
    p = line.split()
    try:
      v = [int(x) for x in p[1:9]]
    except ValueError:
      continue
    v += [0] * (8 - len(v))
    allv = sum(v)
    busy = allv - v[3] - v[4]
    if p[0] == 'cpu':
      total = (busy, allv)
    else:
      try:
        cores[int(p[0][3:])] = (busy, allv)
      except ValueError:
        pass
  return total, cores


def pct(prev, cur):
  """percent busy between two (busy, all) pairs; None when no time passed (offline core)"""
  if prev is None or cur is None:
    return None
  d_all = cur[1] - prev[1]
  if d_all <= 0:
    return None
  return max(0.0, min(100.0, 100.0 * (cur[0] - prev[0]) / d_all))


def parse_meminfo(text):
  kb = {}
  for line in text.splitlines():
    if ':' in line:
      k, v = line.split(':', 1)
      try:
        kb[k.strip()] = int(v.split()[0])
      except (ValueError, IndexError):
        pass
  avail = kb.get('MemAvailable')
  if avail is None:
    avail = kb.get('MemFree', 0) + kb.get('Buffers', 0) + kb.get('Cached', 0)
  return {'total_mb': kb.get('MemTotal', 0) / 1024.0, 'avail_mb': avail / 1024.0}


def parse_pid_stat(text):
  """/proc/<pid>/stat -> (comm, utime+stime in clock ticks); comm may contain spaces and parentheses"""
  try:
    head, rest = text.rsplit(')', 1)
    comm = head.split('(', 1)[1]
    f = rest.split()
    return comm, int(f[11]) + int(f[12])          # fields 14 and 15 of the file
  except (ValueError, IndexError):
    return None, None


def proc_label(cmdline, comm):
  """short stable name: the script/binary a process runs (python processes are told apart by their script)"""
  parts = [p for p in cmdline.split('\0') if p]
  if not parts:
    return comm
  for p in parts[:3]:
    base = os.path.basename(p)
    if base.endswith('.py'):
      return base
  base = os.path.basename(parts[0])
  if base.startswith('python') and len(parts) > 1:
    return os.path.basename(parts[1])
  return base or comm


def _rd(path):
  with open(path) as f:
    return f.read()


def read_thermal(root='/sys/class/thermal'):
  """{zone type (+index if duplicated): degrees C}; Android reports milli-degrees on most zones, degrees on some"""
  out = {}
  for z in sorted(glob.glob(os.path.join(root, 'thermal_zone*'))):
    try:
      typ = _rd(os.path.join(z, 'type')).strip()
      raw = float(_rd(os.path.join(z, 'temp')).strip())
    except (OSError, ValueError):
      continue
    if typ.startswith('tsens_tz_sensor'):                  # Qualcomm SoC sensors on this kernel: tenths of a degree (openpilot's thermald divides by 10 too)
      c = raw / 1000.0 if abs(raw) > 3000 else raw / 10.0
    else:
      c = raw / 1000.0 if abs(raw) > 200 else raw
    if not -40 < c < 200:
      continue
    name = typ if typ not in out else '%s#%s' % (typ, os.path.basename(z)[len('thermal_zone'):])
    out[name] = c
  return out


def read_gpu(root='/sys/class/kgsl/kgsl-3d0'):
  """(busy percent since the last read of gpubusy, clock MHz) - either may be None. The Adreno driver resets gpubusy on read."""
  busy = mhz = None
  try:
    a = _rd(os.path.join(root, 'gpubusy')).split()
    if len(a) >= 2 and int(a[1]) > 0:
      busy = max(0.0, min(100.0, 100.0 * int(a[0]) / int(a[1])))
  except (OSError, ValueError):
    pass
  for name in ('devfreq/cur_freq', 'gpuclk'):
    try:
      v = float(_rd(os.path.join(root, name)).split()[0])
      mhz = v / 1e6 if v > 1e5 else v
      break
    except (OSError, ValueError, IndexError):
      continue
  return busy, mhz


def read_cpu_mhz(root='/sys/devices/system/cpu'):
  out = {}
  for d in sorted(glob.glob(os.path.join(root, 'cpu[0-9]*'))):
    try:
      out[int(os.path.basename(d)[3:])] = float(_rd(os.path.join(d, 'cpufreq', 'scaling_cur_freq')).split()[0]) / 1000.0
    except (OSError, ValueError, IndexError):
      pass
  return out


def read_battery_c(root='/sys/class/power_supply/battery'):
  try:
    return float(_rd(os.path.join(root, 'temp')).split()[0]) / 10.0
  except (OSError, ValueError, IndexError):
    return None


class Sampler(object):
  """sample() every >= 1 s. First call only primes the counters and returns None."""

  def __init__(self, proc='/proc', sysroot='/sys', own_pid=None, proc_every=1):
    self.proc, self.sys = proc, sysroot
    self.proc_every = max(1, int(proc_every))      # scan the process list only every N samples (a full scan costs real CPU on the EON)
    self.n = 0
    self.labels = {}                               # pid -> label, read once
    self.kthreads = set()                          # pids with an empty cmdline (kernel threads): never looked at again
    self.scan_prev = None
    self.per, self.own = {}, 0.0
    self.own_pid = own_pid if own_pid is not None else os.getpid()
    self.prev = None

  def _read(self, path):
    try:
      with open(path) as f:
        return f.read()
    except OSError:
      return ''

  def _procs(self):
    out = {}
    for d in glob.glob(os.path.join(self.proc, '[0-9]*')):
      try:
        pid = int(os.path.basename(d))
      except ValueError:
        continue
      if pid in self.kthreads:
        continue
      st = self._read(os.path.join(d, 'stat'))
      comm, ticks = parse_pid_stat(st) if st else (None, None)
      if comm is None:
        continue
      if pid not in self.labels:
        cmd = self._read(os.path.join(d, 'cmdline'))
        if not cmd:
          self.kthreads.add(pid)
          continue
        self.labels[pid] = proc_label(cmd, comm)
      out[pid] = (self.labels[pid], ticks)
    return out

  def sample(self, now=None):
    now = time.time() if now is None else now
    total, cores = parse_proc_stat(self._read(os.path.join(self.proc, 'stat')))
    self.n += 1
    cur = {'t': now, 'total': total, 'cores': cores}
    prev, self.prev = self.prev, cur
    if self.proc_every == 1 or self.n % self.proc_every == 1:      # a process scan (the expensive part); the first sample always scans
      snap = {'t': now, 'procs': self._procs()}
      old = self.scan_prev
      if old is not None and now > old['t']:
        per, own = {}, 0.0
        for pid, (label, ticks) in snap['procs'].items():
          if pid in old['procs'] and old['procs'][pid][1] <= ticks:
            p = 100.0 * (ticks - old['procs'][pid][1]) / (CLK_TCK * (now - old['t']))      # percent of ONE core, averaged since the previous scan
            per[label] = per.get(label, 0.0) + p
            if pid == self.own_pid:
              own = p
        self.per, self.own = per, own
      self.scan_prev = snap
    if prev is None or now <= prev['t']:
      return None
    ncore = max(1, len(cores))
    mem = parse_meminfo(self._read(os.path.join(self.proc, 'meminfo')))
    temps = read_thermal(os.path.join(self.sys, 'class/thermal'))
    gpu_busy, gpu_mhz = read_gpu(os.path.join(self.sys, 'class/kgsl/kgsl-3d0'))
    s = {
      't': now, 'dt': now - prev['t'],
      'cpu_total_pct': pct(prev['total'], cur['total']),
      'cpu_core_pct': [pct(prev['cores'].get(i), cur['cores'].get(i)) for i in sorted(cur['cores'])],
      'cpu_mhz': read_cpu_mhz(os.path.join(self.sys, 'devices/system/cpu')),
      'mem_avail_mb': mem['avail_mb'], 'mem_total_mb': mem['total_mb'],
      'temps': temps, 'temp_max_c': max(temps.values()) if temps else None,
      'temp_soc_max_c': max([v for k, v in temps.items() if k.startswith('tsens_tz_sensor')] or [None]) if any(k.startswith('tsens_tz_sensor') for k in temps) else None,
      'temp_board_max_c': max([v for k, v in temps.items() if not k.startswith('tsens_tz_sensor') and not k.startswith('bms') and not k.startswith('bq') and k != 'battery'] or [None]) if temps else None,
      'gpu_busy_pct': gpu_busy, 'gpu_mhz': gpu_mhz,
      'batt_c': read_battery_c(os.path.join(self.sys, 'class/power_supply/battery')),
      'procs': dict(self.per), 'self_cpu_core_pct': self.own, 'ncores': ncore,
    }
    # the rest of the system, in percent of the whole CPU, without this process
    if s['cpu_total_pct'] is not None:
      s['others_cpu_pct'] = max(0.0, s['cpu_total_pct'] - self.own / ncore)
    else:
      s['others_cpu_pct'] = None
    return s
