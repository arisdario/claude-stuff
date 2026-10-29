// reads raw_out.bin (one 0.8.3 output, 11327 floats) and prints what the adapter would publish as 0.7.5 fields
#include <stdio.h>
#include <vector>
#include "../model083_adapter.h"
using namespace m083;
int main() {
  std::vector<float> o(OUTPUT_SIZE + TEMPORAL_SIZE); FILE *f = fopen("raw_out.bin", "rb");
  if (!f || fread(o.data(), sizeof(float), o.size(), f) != o.size()) { printf("bad file\n"); return 1; } fclose(f);
  Line p, l, r; Lead ld; Meta m; path_from_plan(o.data(), p); lane_line(o.data(), LANE_LEFT, l); lane_line(o.data(), LANE_RIGHT, r); lead(o.data(), 0, ld); meta(o.data(), m);
  printf("left  lane: prob %.2f  y at 0/20/40 m: %+.2f %+.2f %+.2f  poly [%.2e %.2e %.2e %+.2f]\n", l.prob, l.points[0], l.points[20], l.points[40], l.poly[0], l.poly[1], l.poly[2], l.poly[3]);
  printf("right lane: prob %.2f  y at 0/20/40 m: %+.2f %+.2f %+.2f  poly [%.2e %.2e %.2e %+.2f]\n", r.prob, r.points[0], r.points[20], r.points[40], r.poly[0], r.poly[1], r.poly[2], r.poly[3]);
  printf("lane width at 0 m: %.2f m\n", l.points[0] - r.points[0]);
  printf("path      : y at 0/20/40 m: %+.2f %+.2f %+.2f\n", p.points[0], p.points[20], p.points[40]);
  printf("lead      : prob %.2f (no car drawn, should be low)\n", ld.prob);
  printf("engagedProb %.2f\n", m.engaged);
  return 0;
}
