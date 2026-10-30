// model083_adapter.h - turns the raw output of the openpilot 0.8.3 driving model (supercombo) into the fields the
// 0.7.5 "model" message carries, so 0.7.5's planners, radard and the Tesla code keep working unchanged.
// Plain C++11, no openpilot / capnp / Eigen / OpenCL: driving.cc copies these structs into the message, and
// test_adapter.cc unit-tests the conversion on any PC (g++ -std=c++11 test_adapter.cc && ./a.out).
//
// The output layout below is copied from openpilot v0.8.3 selfdrive/modeld/models/driving.{h,cc} and
// selfdrive/common/modeldata.h; the conversions from that version's fill_model / fill_lead_v2 / fill_meta /
// posenet_publish and from 0.7.5's fill_path / fill_lead / fill_meta / poly_fit.
//
// Conventions that differ between the two versions (all handled here):
//   - side: 0.8.3 is device frame (y positive = RIGHT); 0.7.5 polys and lead relY are positive = LEFT -> negated
//   - path: 0.8.3 gives a time-indexed plan (x,y at 33 times); 0.7.5 wants y at x = 0..191 m -> interpolated
//   - lanes: 0.8.3 has 4 lines at 33 distances (X_IDXS); the inner two (1 = left, 2 = right) are 0.7.5's lanes
//   - stds: 0.8.3 outputs log-std (exp); 0.7.5 used softplus; lead x/y no longer scaled by 10
//   - desireState: 0.7.5 code reads desireState[laneChange* - 1], the 0.8.3 model is indexed [laneChange*] -> shifted
//   - pose rotation: already in radians in 0.8.3 (0.7.5 converted degrees)
#pragma once
#include <math.h>
#include <string.h>

namespace m083 {

const int TRAJECTORY_SIZE = 33;
const double T_IDXS[TRAJECTORY_SIZE] = {0., 0.00976562, 0.0390625, 0.08789062, 0.15625, 0.24414062, 0.3515625,
  0.47851562, 0.625, 0.79101562, 0.9765625, 1.18164062, 1.40625, 1.65039062, 1.9140625, 2.19726562, 2.5,
  2.82226562, 3.1640625, 3.52539062, 3.90625, 4.30664062, 4.7265625, 5.16601562, 5.625, 6.10351562, 6.6015625,
  7.11914062, 7.65625, 8.21289062, 8.7890625, 9.38476562, 10.};
const double X_IDXS[TRAJECTORY_SIZE] = {0., 0.1875, 0.75, 1.6875, 3., 4.6875, 6.75, 9.1875, 12., 15.1875, 18.75,
  22.6875, 27., 31.6875, 36.75, 42.1875, 48., 54.1875, 60.75, 67.6875, 75., 82.6875, 90.75, 99.1875, 108.,
  117.1875, 126.75, 136.6875, 147., 157.6875, 168.75, 180.1875, 192.};

const int DESIRE_N = 8;
const int TRAFFIC_N = 2;
const int DESIRE_PRED_N = 32;
const int OTHER_META_N = 4;
const int PLAN_MHP_N = 5;
const int PLAN_MHP_COLUMNS = 30;
const int PLAN_MHP_VALS = 30 * 33;
const int PLAN_MHP_SELECTION = 1;
const int PLAN_MHP_GROUP_SIZE = 2 * PLAN_MHP_VALS + PLAN_MHP_SELECTION;
const int LEAD_MHP_N = 5;
const int LEAD_MHP_VALS = 4;
const int LEAD_MHP_SELECTION = 3;
const int LEAD_MHP_GROUP_SIZE = 2 * LEAD_MHP_VALS + LEAD_MHP_SELECTION;
const int POSE_N = 12;

const int PLAN_IDX = 0;
const int LL_IDX = PLAN_IDX + PLAN_MHP_N * PLAN_MHP_GROUP_SIZE;
const int LL_PROB_IDX = LL_IDX + 4 * 2 * 2 * 33;
const int RE_IDX = LL_PROB_IDX + 4;
const int LEAD_IDX = RE_IDX + 2 * 2 * 2 * 33;
const int LEAD_PROB_IDX = LEAD_IDX + LEAD_MHP_N * LEAD_MHP_GROUP_SIZE;
const int DESIRE_STATE_IDX = LEAD_PROB_IDX + 3;
const int META_IDX = DESIRE_STATE_IDX + DESIRE_N;
const int POSE_IDX = META_IDX + OTHER_META_N + DESIRE_PRED_N;
const int OUTPUT_SIZE = POSE_IDX + POSE_N;
const int TEMPORAL_SIZE = 512;              // 0.8.3 recurrent state (0.7.5's model used 1024)

const int PATH_N = 192;                     // 0.7.5 MODEL_PATH_DISTANCE: one point per metre
const float NO_DATA_STD = 1.0e3f;           // 0.7.5 marks points past the valid length with this std
const int LANE_LEFT = 1, LANE_RIGHT = 2;    // inner lane lines of the 4

struct Line { float points[PATH_N]; float stds[PATH_N]; float poly[4]; float prob; float std; };
struct Lead { float prob, dist, std, rel_y, rel_y_std, rel_vel, rel_vel_std, rel_a, rel_a_std; };
struct Longi { float dist[10], speed[10], accel[10]; };
struct Meta { float desire_state[DESIRE_N]; float engaged, gas, brake, steer; float desire_pred[DESIRE_PRED_N]; };
struct Pose { float trans[3], trans_std[3], rot[3], rot_std[3]; };

inline float sigm(float x) { return 1.0f / (1.0f + expf(-x)); }

inline void softmax(const float *in, float *out, int n) {
  float mx = in[0];
  for (int i = 1; i < n; i++) if (in[i] > mx) mx = in[i];
  float den = 0;
  for (int i = 0; i < n; i++) { out[i] = expf(in[i] - mx); den += out[i]; }
  for (int i = 0; i < n; i++) out[i] /= den;
}

// 0.8.3 get_best_data: the hypothesis whose selection logit (at offset from the END of its group) is largest
inline const float *best(const float *data, int n, int group_size, int offset) {
  int mi = 0;
  for (int i = 1; i < n; i++)
    if (data[(i + 1) * group_size + offset] > data[(mi + 1) * group_size + offset]) mi = i;
  return &data[mi * group_size];
}

// linear interpolation of ys over increasing xs, clamped at both ends
inline float interp(double x, const double *xs, const float *ys, int n) {
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  for (int i = 1; i < n; i++) {
    if (x <= xs[i]) {
      double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
      return (float)(ys[i - 1] + t * (ys[i] - ys[i - 1]));
    }
  }
  return ys[n - 1];
}

// 0.7.5 poly_fit: weighted least squares of (y - y0) on [x^3, x^2, x] for x = 0..191, weights 1/std; poly[3] = y0.
// Normal equations with column scaling, in double (3x3, solved by Gaussian elimination with partial pivoting).
inline void poly_fit(const float *pts, const float *stds, float *poly) {
  const double y0 = pts[0];
  double A[3][3] = {{0}}, b[3] = {0}, sc[3] = {0};
  for (int i = 0; i < PATH_N; i++) {
    const double x = i, w = 1.0 / ((double)stds[i] * stds[i]);
    const double v[3] = {x * x * x, x * x, x};
    for (int j = 0; j < 3; j++) sc[j] += v[j] * v[j] * w;
  }
  for (int j = 0; j < 3; j++) sc[j] = sc[j] > 0 ? 1.0 / sqrt(sc[j]) : 1.0;
  for (int i = 0; i < PATH_N; i++) {
    const double x = i, w = 1.0 / ((double)stds[i] * stds[i]), r = pts[i] - y0;
    const double v[3] = {x * x * x * sc[0], x * x * sc[1], x * sc[2]};
    for (int j = 0; j < 3; j++) { b[j] += v[j] * r * w; for (int k = 0; k < 3; k++) A[j][k] += v[j] * v[k] * w; }
  }
  for (int c = 0; c < 3; c++) {
    int p = c;
    for (int r = c + 1; r < 3; r++) if (fabs(A[r][c]) > fabs(A[p][c])) p = r;
    if (p != c) { for (int k = 0; k < 3; k++) { double t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; } double t = b[c]; b[c] = b[p]; b[p] = t; }
    if (fabs(A[c][c]) < 1e-300) { poly[0] = poly[1] = poly[2] = 0; poly[3] = (float)y0; return; }
    for (int r = c + 1; r < 3; r++) {
      const double f = A[r][c] / A[c][c];
      for (int k = c; k < 3; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  double s[3];
  for (int c = 2; c >= 0; c--) {
    double acc = b[c];
    for (int k = c + 1; k < 3; k++) acc -= A[c][k] * s[k];
    s[c] = acc / A[c][c];
  }
  for (int j = 0; j < 3; j++) poly[j] = (float)(s[j] * sc[j]);
  poly[3] = (float)y0;
}

// predicted path from the most likely plan: y (left positive) at every metre; points past where the plan reaches
// get NO_DATA_STD so the fit ignores them (like 0.7.5's valid_len), but the first 5 always count
inline void path_from_plan(const float *out, Line &L) {
  const float *pl = best(out + PLAN_IDX, PLAN_MHP_N, PLAN_MHP_GROUP_SIZE, -1);
  double px[TRAJECTORY_SIZE]; float py[TRAJECTORY_SIZE], ps[TRAJECTORY_SIZE];
  double mx = 0;
  for (int i = 0; i < TRAJECTORY_SIZE; i++) {
    double x = pl[i * PLAN_MHP_COLUMNS + 0];
    if (i > 0 && x < px[i - 1] + 1e-3) x = px[i - 1] + 1e-3;     // keep x increasing (standstill / noise)
    px[i] = x; if (x > mx) mx = x;
    py[i] = -pl[i * PLAN_MHP_COLUMNS + 1];
    ps[i] = expf(pl[PLAN_MHP_VALS + i * PLAN_MHP_COLUMNS + 1]);
  }
  const bool short_plan = mx < 5.0;          // standing / creeping: the plan says nothing about the road ahead
  for (int d = 0; d < PATH_N; d++) {
    L.points[d] = short_plan ? py[0] : interp(d, px, py, TRAJECTORY_SIZE);
    L.stds[d] = (d < 5 || d <= mx) ? (short_plan ? ps[0] : interp(d, px, ps, TRAJECTORY_SIZE)) : NO_DATA_STD;
    if (!(L.stds[d] > 1e-3f)) L.stds[d] = 1e-3f;
  }
  L.prob = 1.0f;
  L.std = ps[0];
  poly_fit(L.points, L.stds, L.poly);
}

// one of the 4 lane lines (k = 1 left inner, 2 right inner), y left positive at every metre
inline void lane_line(const float *out, int k, Line &L) {
  const float *ln = out + LL_IDX + k * TRAJECTORY_SIZE * 2;
  const float *sd = out + LL_IDX + 2 * TRAJECTORY_SIZE * (4 + k);
  float ly[TRAJECTORY_SIZE], ls[TRAJECTORY_SIZE];
  for (int i = 0; i < TRAJECTORY_SIZE; i++) { ly[i] = -ln[2 * i]; ls[i] = expf(sd[2 * i]); }
  for (int d = 0; d < PATH_N; d++) {
    L.points[d] = interp(d, X_IDXS, ly, TRAJECTORY_SIZE);
    L.stds[d] = interp(d, X_IDXS, ls, TRAJECTORY_SIZE);
    if (!(L.stds[d] > 1e-3f)) L.stds[d] = 1e-3f;
  }
  L.prob = sigm(out[LL_PROB_IDX + k]);
  L.std = expf(sd[0]);
  poly_fit(L.points, L.stds, L.poly);
}

// lead now (t = 0) or in 2 s (t = 1): distance from the camera like 0.7.5 (radard subtracts RADAR_TO_CAMERA)
inline void lead(const float *out, int t, Lead &R) {
  const float *d = best(out + LEAD_IDX, LEAD_MHP_N, LEAD_MHP_GROUP_SIZE, t - LEAD_MHP_SELECTION);
  R.prob = sigm(out[LEAD_PROB_IDX + t]);
  R.dist = d[0];            R.std = expf(d[LEAD_MHP_VALS + 0]);
  R.rel_y = -d[1];          R.rel_y_std = expf(d[LEAD_MHP_VALS + 1]);
  R.rel_vel = d[2];         R.rel_vel_std = expf(d[LEAD_MHP_VALS + 2]);
  R.rel_a = d[3];           R.rel_a_std = expf(d[LEAD_MHP_VALS + 3]);
}

// 0.7.5 "longitudinal": distance / speed / acceleration along the plan at t = 0..9 s
inline void longi(const float *out, Longi &G) {
  const float *pl = best(out + PLAN_IDX, PLAN_MHP_N, PLAN_MHP_GROUP_SIZE, -1);
  float x[TRAJECTORY_SIZE], v[TRAJECTORY_SIZE], a[TRAJECTORY_SIZE];
  for (int i = 0; i < TRAJECTORY_SIZE; i++) {
    x[i] = pl[i * PLAN_MHP_COLUMNS + 0]; v[i] = pl[i * PLAN_MHP_COLUMNS + 3]; a[i] = pl[i * PLAN_MHP_COLUMNS + 6];
  }
  for (int s = 0; s < 10; s++) {
    G.dist[s] = interp(s, T_IDXS, x, TRAJECTORY_SIZE);
    G.speed[s] = interp(s, T_IDXS, v, TRAJECTORY_SIZE);
    G.accel[s] = interp(s, T_IDXS, a, TRAJECTORY_SIZE);
  }
}

inline void meta(const float *out, Meta &M) {
  const float *m = out + DESIRE_STATE_IDX;
  float st[DESIRE_N];
  softmax(m, st, DESIRE_N);
  for (int i = 0; i < DESIRE_N - 1; i++) M.desire_state[i] = st[i + 1];   // 0.7.5 reads [desire - 1]
  M.desire_state[DESIRE_N - 1] = 0.0f;
  M.engaged = sigm(m[DESIRE_N]);
  M.gas = sigm(m[DESIRE_N + 1]);
  M.brake = sigm(m[DESIRE_N + 2]);
  M.steer = sigm(m[DESIRE_N + 3]);
  for (int i = 0; i < 4; i++)
    softmax(&m[DESIRE_N + OTHER_META_N + i * DESIRE_N], &M.desire_pred[i * DESIRE_N], DESIRE_N);
}

inline void pose(const float *out, Pose &P) {
  const float *p = out + POSE_IDX;
  for (int i = 0; i < 3; i++) {
    P.trans[i] = p[i];      P.trans_std[i] = expf(p[6 + i]);
    P.rot[i] = p[3 + i];    P.rot_std[i] = expf(p[9 + i]);
  }
}

// 0.7.5's modeld feeds the desire one-hot shifted by one (index = desire - 1); the 0.8.3 model wants index = desire
inline void desire_to_083(const float *desire_075, float *desire_083) {
  desire_083[0] = 0.0f;
  for (int i = 1; i < DESIRE_N; i++) desire_083[i] = desire_075[i - 1];
}

// Road edges (curbs, walls, end of the drivable surface) have no field in the 0.7.5 message; they travel in the
// unused ModelData.freePath list:  [ROAD_EDGE_FORMAT, left edge y x33, right edge y x33, left std, right std]
// y is left-positive at the X_IDXS distances, like the lanes. radarweb reads them (empty list = old model).
const float ROAD_EDGE_FORMAT = 1.0f;
const int ROAD_EDGE_PACK_N = 1 + 2 * TRAJECTORY_SIZE + 2;
inline void road_edges(const float *out, float *pack) {
  pack[0] = ROAD_EDGE_FORMAT;
  for (int k = 0; k < 2; k++) {
    const float *e = out + RE_IDX + k * TRAJECTORY_SIZE * 2;       // 0.8.3 fill_xyzt(road_edges[k], ..., 2, -1)
    for (int i = 0; i < TRAJECTORY_SIZE; i++) pack[1 + k * TRAJECTORY_SIZE + i] = -e[2 * i];
    pack[1 + 2 * TRAJECTORY_SIZE + k] = expf(out[RE_IDX + 2 * TRAJECTORY_SIZE * (2 + k)]);
  }
}

}  // namespace m083
