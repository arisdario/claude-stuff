// Unit test for model0812_adapter.h:   g++ -std=c++17 -O2 -Wall -o /tmp/t test_adapter0812.cc && /tmp/t
// The fake 0.8.12 output is written THROUGH comma's own v0.8.12 ModelOutput structs (tools/openpilot_0812_structs.h,
// copied verbatim), not with the adapter's offsets, so a layout mistake in the adapter shows up here.
#include <stdio.h>
#include <math.h>
#include <vector>
#include "tools/openpilot_0812_structs.h"
#include "model0812_adapter.h"
using namespace m0812;

static int fails = 0;
static void check(bool ok, const char *what, double got, double want) {
  printf("%s %-50s got %10.5f  want %10.5f\n", ok ? "ok  " : "FAIL", what, got, want);
  if (!ok) fails++;
}
static void near(const char *what, double got, double want, double tol) { check(fabs(got - want) <= tol, what, got, want); }
static float logit(float p) { return logf(p / (1 - p)); }

int main() {
  check(op0812::OUTPUT_SIZE == OUTPUT_SIZE, "output size = comma's sizeof(ModelOutput)/4", OUTPUT_SIZE, op0812::OUTPUT_SIZE);
  std::vector<float> buf(OUTPUT_SIZE + TEMPORAL_SIZE, 0.0f);
  float *o = buf.data();
  auto *mo = reinterpret_cast<op0812::ModelOutput *>(o);
  auto &plans = const_cast<op0812::ModelOutputPlans &>(mo->plans);
  auto &lanes = const_cast<op0812::ModelOutputLaneLines &>(mo->lane_lines);
  auto &edges = const_cast<op0812::ModelOutputRoadEdges &>(mo->road_edges);
  auto &leads = const_cast<op0812::ModelOutputLeads &>(mo->leads);
  auto &meta_ = const_cast<op0812::ModelOutputMeta &>(mo->meta);
  auto &pose_ = const_cast<op0812::ModelOutputPose &>(mo->pose);

  // --- plan: hypothesis 2 is best; 20 m/s, curving RIGHT (device frame y + = right) ---
  for (int h = 0; h < 5; h++) {
    for (int i = 0; i < 33; i++) {
      double x = 20.0 * T_IDXS[i];
      plans.prediction[h].mean[i].position.x = x;
      plans.prediction[h].mean[i].position.y = (h == 2 ? 0.002 : 0.05) * x * x;
      plans.prediction[h].mean[i].velocity.x = 20.0f;
      plans.prediction[h].mean[i].acceleration.x = 0.5f;
      plans.prediction[h].std[i].position.y = logf(0.3f);
    }
    plans.prediction[h].prob = (h == 2) ? 5.0f : -1.0f;
  }
  // --- lanes (device frame), probs: val_deprecated deliberately different from val ---
  const float ly[4] = {-5.4f, -1.8f, 1.7f, 5.3f};
  op0812::ModelOutputYZ *lm[4] = {lanes.mean.left_far.data(), lanes.mean.left_near.data(), lanes.mean.right_near.data(), lanes.mean.right_far.data()};
  op0812::ModelOutputYZ *ls[4] = {lanes.std.left_far.data(), lanes.std.left_near.data(), lanes.std.right_near.data(), lanes.std.right_far.data()};
  for (int k = 0; k < 4; k++) for (int i = 0; i < 33; i++) { lm[k][i].y = ly[k]; ls[k][i].y = logf(0.10f + 0.05f * k); }   // near lanes: 0.15, 0.20
  lanes.prob.left_near = {logit(0.01f), logit(0.9f)};
  lanes.prob.right_near = {logit(0.99f), logit(0.3f)};
  // --- road edges ---
  for (int i = 0; i < 33; i++) { edges.mean.left[i].y = -6.0f; edges.mean.right[i].y = 4.5f; }
  edges.std.left[0].y = logf(0.4f); edges.std.right[0].y = logf(0.6f);
  // --- leads (v/a ABSOLUTE in 0.8.12): hypothesis 1 best NOW = a STOPPED car 30 m ahead, 1 m right; hypothesis 0
  //     best for 2 s = a car at 15 m/s, 45 m ahead NOW (comma's leadTwo reads point 0). Ego (plan) = 20 m/s, 0.5 m/s2.
  for (int h = 0; h < 2; h++) for (int i = 0; i < 6; i++) {
    leads.prediction[h].mean[i] = {(h == 1 ? 30.0f : 45.0f) + 7.5f * i, h == 1 ? 1.0f : -2.0f, h == 1 ? 0.0f : 15.0f, 0.2f};
    leads.prediction[h].std[i] = {logf(1.5f + i), logf(0.4f), logf(0.8f), logf(0.5f)};
  }
  leads.prediction[1].prob = {3.0f, -2.0f, 0.0f}; leads.prediction[0].prob = {-1.0f, 4.0f, 0.0f};
  leads.prob = {logit(0.8f), logit(0.6f), logit(0.5f)};
  // --- meta / pose ---
  meta_.desire_state_prob.array = {0, 0, 0, 3.0f, 0, 0, 0, 0};                  // lane_change_left most likely
  meta_.engaged_prob = logit(0.7f);
  meta_.disengage_prob[0] = {logit(0.1f), logit(0.2f), logit(0.3f), 0, 0, 0, 0};
  meta_.disengage_prob[1] = {logit(0.9f), logit(0.9f), logit(0.9f), 0, 0, 0, 0};   // a later horizon: must NOT be used
  pose_.velocity_mean = {20.0f, 0.1f, 0.0f}; pose_.rotation_mean = {0.0f, 0.0f, 0.05f};
  pose_.velocity_std = {logf(0.2f), logf(0.2f), logf(0.2f)}; pose_.rotation_std = {logf(0.01f), logf(0.01f), logf(0.01f)};

  Line p, l, r; Lead ld, lf; Longi g; Meta mt; Pose ps; float re[ROAD_EDGE_PACK_N];
  path_from_plan(o, p); lane_line(o, LANE_LEFT, l); lane_line(o, LANE_RIGHT, r);
  lead(o, 0, ld); lead(o, 1, lf); longi(o, g); meta(o, mt); pose(o, ps); road_edges(o, re);

  near("path picks best plan, right curve -> y negative @40m", p.points[40], -0.002 * 40 * 40, 0.05);
  near("path std from plan std (exp)", p.stds[10], 0.3, 1e-3);
  near("left lane = -left_near (left positive)", l.points[20], 1.8, 1e-4);
  near("right lane = -right_near", r.points[20], -1.7, 1e-4);
  near("left lane prob uses .val (std 0.15: no discount)", l.prob, 0.9, 1e-4);
  near("right lane prob .val x comma std discount (0.20)", r.prob, 0.3 * (0.3 - 0.2) / 0.15, 1e-4);
  near("left lane std (exp of std[0].y)", l.std, 0.15, 1e-4);
  near("lead now: best-for-t0 hypothesis, mean[0].x", ld.dist, 30.0, 1e-4);
  near("lead now: y negated", ld.rel_y, -1.0, 1e-4);
  near("lead now: STOPPED car at 20 m/s ego -> relVel -20", ld.rel_vel, -20.0, 1e-4);
  near("lead now: relative accel = 0.2 - plan 0.5", ld.rel_a, -0.3, 1e-4);
  near("lead now: x std = exp(std[0].x)", ld.std, 1.5, 1e-4);
  near("lead now prob = sigmoid(leads.prob[0])", ld.prob, 0.8, 1e-4);
  near("leadFuture: best-for-t1 hypothesis, point 0 x", lf.dist, 45.0, 1e-4);
  near("leadFuture: x std = exp(std[0].x)", lf.std, 1.5, 1e-4);
  near("leadFuture: relVel = 15 - 20", lf.rel_vel, -5.0, 1e-4);
  near("lead 2 s prob = sigmoid(leads.prob[1])", lf.prob, 0.6, 1e-4);
  near("longitudinal speed @3 s", g.speed[3], 20.0, 1e-3);
  near("longitudinal accel @3 s", g.accel[3], 0.5, 1e-3);
  near("desire_state shifted for 0.7.5 ([lcl-1] = idx 2)", mt.desire_state[2], expf(3) / (expf(3) + 7), 1e-4);
  near("engaged prob", mt.engaged, 0.7, 1e-4);
  near("gas from disengage horizon 0", mt.gas, 0.1, 1e-4);
  near("brake from disengage horizon 0", mt.brake, 0.2, 1e-4);
  near("steer override from disengage horizon 0", mt.steer, 0.3, 1e-4);
  near("pose trans x", ps.trans[0], 20.0, 1e-4);
  near("pose rot z", ps.rot[2], 0.05, 1e-4);
  near("pose rot std (exp)", ps.rot_std[2], 0.01, 1e-5);
  near("road edge left = +6 (left positive)", re[1 + 5], 6.0, 1e-4);
  near("road edge right = -4.5", re[1 + 33 + 5], -4.5, 1e-4);
  near("road edge left std", re[1 + 66], 0.4, 1e-4);

  // --- model FCW: comma's rule needs P(5 m/s2) > .05/.05/.15/.15/.15 over the last 5 frames AND P(3 m/s2) > .7 x3 ---
  {
    std::vector<float> b(OUTPUT_SIZE + TEMPORAL_SIZE, 0.0f); float *q = b.data();
    auto &m2 = const_cast<op0812::ModelOutputMeta &>(reinterpret_cast<op0812::ModelOutput *>(q)->meta);
    FcwHistory hist; float pk[MODEL_FCW_PACK_N]; int first_on = -1;
    for (int f = 0; f < 8; f++) {
      m2.disengage_prob[0].brake_5ms2 = logit(0.30f); m2.disengage_prob[0].brake_3ms2 = logit(0.80f);
      model_fcw(q, hist, pk);
      if (pk[0] == 1.0f && first_on < 0) first_on = f;
    }
    check(first_on == 4, "model FCW fires after 5 frames of high P (not before)", first_on, 4);
    near("model FCW pack: P(5 m/s2)", pk[1], 0.30, 1e-4);
    near("model FCW pack: P(3 m/s2)", pk[2], 0.80, 1e-4);
    m2.disengage_prob[0].brake_3ms2 = logit(0.50f); model_fcw(q, hist, pk);
    check(pk[0] == 0.0f, "model FCW drops when P(3 m/s2) < 0.7", pk[0], 0);
    m2.disengage_prob[1].brake_5ms2 = logit(0.99f); m2.disengage_prob[0].brake_5ms2 = logit(0.01f);
    m2.disengage_prob[0].brake_3ms2 = logit(0.99f);
    for (int f = 0; f < 6; f++) model_fcw(q, hist, pk);
    check(pk[0] == 0.0f, "model FCW uses horizon 0 only (like comma)", pk[0], 0);
  }
  // --- lane width ahead (exits / merges): lines 6 m apart -> no weight; 4.5 m -> half; 3.5 m -> untouched ---
  {
    for (float wide : {3.5f, 4.5f, 6.0f}) {
      Line a, c; for (int d = 0; d < PATH_N; d++) { a.points[d] = wide / 2; c.points[d] = -wide / 2; }
      a.prob = c.prob = 0.9f;
      lane_width_check(o, a, c);
      near(wide == 3.5f ? "lane width 3.5 m: probs unchanged" : wide == 4.5f ? "lane width 4.5 m: probs halved" : "lane width 6 m: probs zero",
           a.prob, wide == 3.5f ? 0.9 : wide == 4.5f ? 0.45 : 0.0, 1e-4);
    }
    Line a, c; for (int d = 0; d < PATH_N; d++) { a.points[d] = d < 40 ? 1.75f : 3.5f; c.points[d] = -1.75f; }   // left line veers away at 40 m (exit)
    a.prob = c.prob = 0.9f; lane_width_check(o, a, c);                  // plan speed 20 m/s: checks 0, 40.5, 81 m ahead
    near("left line veering off ahead (exit): probs zero", a.prob, 0.0, 1e-4);
  }
  float d075[8] = {0, 0, 1, 0, 0, 0, 0, 0}, d0812[8];
  desire_to_083(d075, d0812);
  check(d0812[3] == 1.0f && d0812[0] == 0.0f, "desire in: 0.7.5 index 2 -> model index 3", d0812[3], 1.0);
  check(MODEL_DLC_BYTES == 56684955L, "model file size = the 0.8.12 supercombo.dlc", MODEL_DLC_BYTES, 56684955);

  printf("\n%s\n", fails ? "FAILURES" : "all passed");
  return fails ? 1 : 0;
}
