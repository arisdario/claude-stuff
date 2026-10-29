// Unit test for model083_adapter.h:   g++ -std=c++11 -O2 -Wall -o /tmp/t test_adapter.cc && /tmp/t
// The fake 0.8.3 output is written with the READ formulas of openpilot v0.8.3 driving.cc (fill_xyzt, fill_lead_v2,
// get_best_data, fill_meta, posenet_publish), not with the adapter's own expressions, so an index mistake in the
// adapter shows up here instead of being mirrored.
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "model083_adapter.h"
using namespace m083;

static int fails = 0;
static void check(bool ok, const char *what, double got, double want) {
  printf("%s %-46s got %10.5f  want %10.5f\n", ok ? "ok  " : "FAIL", what, got, want);
  if (!ok) fails++;
}
static void near(const char *what, double got, double want, double tol) { check(fabs(got - want) <= tol, what, got, want); }

int main() {
  std::vector<float> out(OUTPUT_SIZE + TEMPORAL_SIZE, 0.0f);
  float *o = out.data();
  printf("OUTPUT_SIZE %d (+ recurrent %d = %d floats)\n", OUTPUT_SIZE, TEMPORAL_SIZE, OUTPUT_SIZE + TEMPORAL_SIZE);

  // --- plan: hypothesis 2 is the best (selection logit = last value of its group); 20 m/s, curving RIGHT ---
  const int best_plan = 2;
  for (int h = 0; h < PLAN_MHP_N; h++) {
    float *g = o + PLAN_IDX + h * PLAN_MHP_GROUP_SIZE;
    for (int i = 0; i < TRAJECTORY_SIZE; i++) {
      double x = 20.0 * T_IDXS[i], y = (h == best_plan ? 0.002 : 0.05) * x * x;    // device frame: + = right
      g[i * PLAN_MHP_COLUMNS + 0] = x; g[i * PLAN_MHP_COLUMNS + 1] = y;          // fill_xyzt column_offset 0
      g[i * PLAN_MHP_COLUMNS + 3] = 20.0; g[i * PLAN_MHP_COLUMNS + 6] = 0.5;     // velocity / acceleration x
      g[PLAN_MHP_VALS + i * PLAN_MHP_COLUMNS + 1] = logf(0.3f);                  // log std of y
    }
    g[PLAN_MHP_GROUP_SIZE - 1] = (h == best_plan) ? 5.0f : -1.0f;                // get_best_data(..., -1)
  }
  // --- lane lines: fill_xyzt(lane_lines[k], &ll[k*33*2], columns 2, offset -1) reads y = data[i*2 + 0] ---
  const float lane_y[4] = {-5.4f, -1.8f, 1.7f, 5.3f};                              // device frame, + = right
  for (int k = 0; k < 4; k++) {
    float *d = o + LL_IDX + k * TRAJECTORY_SIZE * 2;
    for (int i = 0; i < TRAJECTORY_SIZE; i++) d[i * 2 + 1 + (-1)] = lane_y[k] + 0.001 * X_IDXS[i] * X_IDXS[i];
    float *s = o + LL_IDX + 2 * TRAJECTORY_SIZE * (4 + k);
    for (int i = 0; i < TRAJECTORY_SIZE; i++) s[i * 2] = logf(0.2f);
    o[LL_PROB_IDX + k] = (k == 1 || k == 2) ? 2.0f : -3.0f;
  }
  // --- leads: get_best_data(lead, 5, 11, t_offset - 3) compares element 8 + t of each group; hyp 3 best now ---
  for (int h = 0; h < LEAD_MHP_N; h++) {
    float *g = o + LEAD_IDX + h * LEAD_MHP_GROUP_SIZE;
    g[0] = 35.0f + h; g[1] = 0.5f; g[2] = -2.0f; g[3] = 0.3f;                       // x y v a (y + = right)
    for (int j = 0; j < 4; j++) g[LEAD_MHP_VALS + j] = logf(1.0f + j);
    g[8 + 0] = (h == 3) ? 4.0f : 0.0f;                                              // t = 0 selection
    g[8 + 1] = (h == 1) ? 4.0f : 0.0f;                                              // t = 2 s selection
  }
  o[LEAD_PROB_IDX + 0] = 1.5f; o[LEAD_PROB_IDX + 1] = -0.5f;
  // --- meta: laneChangeLeft (enum 3) most likely in the 0.8.3 model's indexing ---
  for (int i = 0; i < DESIRE_N; i++) o[DESIRE_STATE_IDX + i] = (i == 3) ? 6.0f : 0.0f;
  o[DESIRE_STATE_IDX + DESIRE_N + 0] = 2.0f;                                      // engaged logit
  // --- pose: rotation in rad already in 0.8.3 ---
  o[POSE_IDX + 3] = 0.01f; o[POSE_IDX + 9] = logf(0.002f); o[POSE_IDX + 0] = 19.9f;

  Line p, l, r; Lead ld, lf; Longi g; Meta m; Pose ps;
  path_from_plan(o, p); lane_line(o, LANE_LEFT, l); lane_line(o, LANE_RIGHT, r);
  lead(o, 0, ld); lead(o, 1, lf); longi(o, g); meta(o, m); pose(o, ps);

  // path: device y = 0.002 x^2 to the right -> 0.7.5 (left positive) y = -0.002 x^2
  near("path y at 50 m (left +)", p.points[50], -0.002 * 50 * 50, 0.05);
  near("path poly x^2 coefficient", p.poly[1], -0.002, 2e-4);
  near("path poly x^3 coefficient", p.poly[0], 0.0, 2e-6);
  near("path poly offset", p.poly[3], 0.0, 0.01);
  check(p.stds[150] < 1.0 && p.stds[199 > PATH_N ? 0 : 191] < 1.0, "path covered to 191 m (plan reaches 200 m)", p.stds[191], 0.3);
  near("left lane offset (left +)", l.poly[3], 1.8, 0.01);
  near("right lane offset (left +)", r.poly[3], -1.7, 0.01);
  near("left lane x^2 coefficient", l.poly[1], -0.001, 1e-4);
  near("lane width from polys", l.poly[3] - r.poly[3], 3.5, 0.02);
  near("left lane prob", l.prob, 1.0 / (1.0 + exp(-2.0)), 1e-5);
  near("left lane std", l.std, 0.2, 1e-5);
  near("lead dist (hyp 3)", ld.dist, 38.0, 1e-4);
  near("lead relY (left +)", ld.rel_y, -0.5, 1e-5);
  near("lead relVel", ld.rel_vel, -2.0, 1e-5);
  near("lead relA", ld.rel_a, 0.3, 1e-5);
  near("lead std", ld.std, 1.0, 1e-5);
  near("lead prob", ld.prob, 1.0 / (1.0 + exp(-1.5)), 1e-5);
  near("lead in 2 s dist (hyp 1)", lf.dist, 36.0, 1e-4);
  near("lead in 2 s prob", lf.prob, 1.0 / (1.0 + exp(0.5)), 1e-5);
  near("longitudinal dist at 5 s", g.dist[5], 100.0, 0.5);
  near("longitudinal speed at 5 s", g.speed[5], 20.0, 1e-4);
  // 0.7.5 lane_planner reads desireState[laneChangeLeft - 1] = [2]
  check(m.desire_state[2] > 0.9, "desireState[laneChangeLeft - 1] (0.7.5 read)", m.desire_state[2], 0.98);
  near("engaged prob", m.engaged, 1.0 / (1.0 + exp(-2.0)), 1e-5);
  near("pose rot (rad, unchanged)", ps.rot[0], 0.01, 1e-7);
  near("pose rot std", ps.rot_std[0], 0.002, 1e-6);
  near("pose trans", ps.trans[0], 19.9, 1e-5);

  // desire input: 0.7.5 modeld sets vec[desire - 1]; laneChangeLeft (3) -> index 2 -> 0.8.3 index 3
  float d75[DESIRE_N] = {0}, d83[DESIRE_N]; d75[2] = 1.0f; desire_to_083(d75, d83);
  check(d83[3] == 1.0f && d83[2] == 0.0f && d83[0] == 0.0f, "desire one-hot shifted to the 0.8.3 index", d83[3], 1.0);

  // poly_fit alone: an exact cubic must come back
  float pts[PATH_N], sd[PATH_N], pf[4];
  for (int i = 0; i < PATH_N; i++) { double x = i; pts[i] = 0.5 + 0.01 * x - 2e-4 * x * x + 1e-6 * x * x * x; sd[i] = 1.0f; }
  poly_fit(pts, sd, pf);
  near("poly_fit x^3", pf[0], 1e-6, 1e-9); near("poly_fit x^2", pf[1], -2e-4, 1e-7); near("poly_fit x", pf[2], 0.01, 1e-5); near("poly_fit y0", pf[3], 0.5, 1e-6);

  // standstill: all plan x = 0 must not blow up the fit
  for (int i = 0; i < TRAJECTORY_SIZE; i++) o[PLAN_IDX + best_plan * PLAN_MHP_GROUP_SIZE + i * PLAN_MHP_COLUMNS] = 0.0f;
  path_from_plan(o, p);
  check(fabs(p.poly[0]) < 1e-3 && fabs(p.poly[1]) < 1e-2 && p.poly[0] == p.poly[0], "standstill path stays finite and flat", p.poly[1], 0.0);

  printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
  return fails ? 1 : 0;
}
