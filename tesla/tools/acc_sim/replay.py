"""openpilot 0.7.5 longitudinal MPC (compiled from its own generated code) replayed on recorded ACC lines."""
import os, sys, gzip, json, glob, math, time
import importlib.util
from cffi import FFI
H = os.path.dirname(os.path.abspath(__file__))
ffi = FFI(); ffi.cdef("""
typedef struct { double x_ego, v_ego, a_ego, x_l, v_l, a_l; } state_t;
typedef struct { double x_ego[21]; double v_ego[21]; double a_ego[21]; double j_ego[20]; double x_l[21]; double v_l[21]; double a_l[21]; double t[21]; double cost; } log_t;
void init(double ttcCost, double distanceCost, double accelerationCost, double jerkCost);
void init_with_simulation(double v_ego, double x_l, double v_l, double a_l, double l);
int run_mpc(state_t * x0, log_t * solution, double l, double a_l_0);
""")
lib = ffi.dlopen(os.path.join(H, 'libmpc1.so'))
spec = importlib.util.spec_from_file_location('ss', os.path.join(H, 'speed_smoother.py')); ss = importlib.util.module_from_spec(spec); spec.loader.exec_module(ss)
TTC, DIST, ACCEL, JERK = 5.0, 0.1, 10.0, 20.0       # MPC_COST_LONG in 0.7.5 drive_helpers
def interp(x, xp, fp):
  if x <= xp[0]: return fp[0]
  if x >= xp[-1]: return fp[-1]
  for i in range(len(xp) - 1):
    if xp[i] <= x <= xp[i + 1]: return fp[i] + (fp[i + 1] - fp[i]) * (x - xp[i]) / (xp[i + 1] - xp[i])
def cruise_limits(v, following):
  amin = interp(v, [0., 5., 10., 20., 40.], [-1.0, -.8, -.67, -.5, -.30])
  amax = interp(v, [0., 6.4, 22.5, 40.], [1.6, 1.6, 0.65, .4] if following else [1.2, 1.2, 0.65, .4])
  return amin, amax
class Planner:
  def __init__(self):
    lib.init(TTC, DIST, ACCEL, JERK)
    self.st = ffi.new('state_t *'); self.sol = ffi.new('log_t *')
    self.v_mpc = self.a_mpc = self.v_mpc_future = 0.; self.prev_lead = False; self.prev_x = 0.
    self.v_start = self.a_start = None
  def step(self, v_ego, a_ego, lead, v_set, reset_to_ego=False):
    if self.v_start is None or reset_to_ego: self.v_start, self.a_start = v_ego, min(a_ego, 0.) if self.v_start is None else a_ego
    following = lead is not None and lead['d'] < 45. and lead['v'] > v_ego and lead['a'] > 0.
    amin, amax = cruise_limits(v_ego, following)
    v_cr, a_cr = ss.speed_smoother(self.v_start, self.a_start, v_set, amax, amin, max(0.1, amax), min(-0.1, amin), 0.2)
    self.st.x_ego = 0.; self.st.v_ego = self.v_start; self.st.a_ego = self.a_start
    if lead is not None:
      v_l = max(0., lead['v']); a_l = lead['a']
      if v_l < 0.1 or -a_l / 2. > v_l: v_l = a_l = 0.
      if not self.prev_lead or abs(lead['d'] - self.prev_x) > 2.5: lib.init_with_simulation(self.v_mpc, lead['d'], v_l, a_l, 1.5)
      self.prev_lead, self.prev_x = True, lead['d']; self.st.x_l, self.st.v_l = lead['d'], v_l
    else:
      self.prev_lead = False; self.st.x_l, self.st.v_l = 50., v_ego + 10.; a_l = 0.
    lib.run_mpc(self.st, self.sol, 1.5, a_l)
    self.v_mpc, self.a_mpc, self.v_mpc_future = self.sol.v_ego[1], self.sol.a_ego[1], self.sol.v_ego[10]
    if any(math.isnan(x) for x in self.sol.v_ego):
      lib.init(TTC, DIST, ACCEL, JERK); self.v_mpc, self.a_mpc = v_ego, a_ego
    if lead is not None and self.v_mpc < v_cr: v_acc, a_acc, src = self.v_mpc, self.a_mpc, 'mpc'
    else: v_acc, a_acc, src = v_cr, a_cr, 'cruise'
    a_sol = self.a_start + (0.05 / 0.2) * (a_acc - self.a_start)
    v_sol = self.v_start + 0.05 * (a_sol + self.a_start) / 2.
    self.v_start, self.a_start = v_sol, a_sol
    return dict(v_target=v_acc, a_target=a_acc, v_future=min(self.v_mpc_future, v_set), src=src, v_pid=v_sol)
def load_acc(drive):
  acc = []
  for f in sorted(glob.glob(drive + '/c*.jsonl.gz')):
    for l in gzip.open(f, 'rt'):
      try: r = json.loads(l)
      except Exception: continue
      if r.get('src') == 'acc' and isinstance(r.get('r'), list): acc.append(r['r'])
  return sorted(acc, key=lambda a: a[0])
