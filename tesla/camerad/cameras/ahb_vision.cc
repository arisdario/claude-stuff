// ahb_vision.cc - see ahb_vision.h. Port of the Python prototype measured on 368 night pictures of 15 Nov 2026
// (AHB_VISION.md): a headlight is a saturated, compact core WITH a glow halo around it; lane markings, signs and
// guardrail reflectors lit by our own beam are reflections: elongated and/or without a halo. Only a band around the
// horizon is searched: a car's lights sit 0.6-1 m above the road and the C920 ~1.2 m, so at any distance they appear
// at or a little below the horizon, while street lamps (6-12 m up) appear clearly above it.
#include "ahb_vision.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <vector>

namespace {
const int DS = 2;                 // work at half resolution
const int ABOVE = 18;             // full-res px above the horizon still searched (pitch / slope error)
const int BELOW = 90;             // ... and below it: oncoming cars' lights from ~6 m outward
const int BOTTOM = 560;           // never below (dash / hood reflections)
const int LAMPS_SPAN = 330;       // street lamps are counted up to this far above the horizon
const int CORE = 250;             // saturated
const int GLOW = 110;             // halo pixel
const float HALO_MIN = 0.45f;
const int MAX_COMPONENTS = 64;    // worst-case bound per run

inline int luma(const uint8_t *rgb, int w, int x2, int y2) {
  const uint8_t *p = rgb + ((size_t)(y2 * DS) * w + (size_t)(x2 * DS)) * 3;
  return (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8;
}

struct Comp { int area, x0, x1, y0, y1; long sx, sy; };

// 4-connected components of (luma >= CORE) in half-res rows [r0, r1). Visits each bright pixel once.
void components(const uint8_t *rgb, int w, int w2, int r0, int r1, std::vector<Comp> &out) {
  out.clear();
  if (r1 <= r0) return;
  const int bh = r1 - r0;
  static std::vector<uint8_t> seen;
  static std::vector<int> stack;
  seen.assign((size_t)w2 * bh, 0);
  for (int y = r0; y < r1; y++) {
    for (int x = 0; x < w2; x++) {
      size_t i = (size_t)(y - r0) * w2 + x;
      if (seen[i] || luma(rgb, w, x, y) < CORE) continue;
      Comp c = {0, x, x, y, y, 0, 0};
      stack.clear(); stack.push_back((int)i); seen[i] = 1;
      while (!stack.empty()) {
        int j = stack.back(); stack.pop_back();
        int cy = j / w2 + r0, cx = j % w2;
        c.area++; c.sx += cx; c.sy += cy;
        if (cx < c.x0) c.x0 = cx;
        if (cx > c.x1) c.x1 = cx;
        if (cy < c.y0) c.y0 = cy;
        if (cy > c.y1) c.y1 = cy;
        const int nx[4] = {cx - 1, cx + 1, cx, cx}, ny[4] = {cy, cy, cy - 1, cy + 1};
        for (int k = 0; k < 4; k++) {
          if (nx[k] < 0 || nx[k] >= w2 || ny[k] < r0 || ny[k] >= r1) continue;
          size_t q = (size_t)(ny[k] - r0) * w2 + nx[k];
          if (seen[q] || luma(rgb, w, nx[k], ny[k]) < CORE) continue;
          seen[q] = 1; stack.push_back((int)q);
        }
      }
      out.push_back(c);
    }
  }
}

float halo_of(const uint8_t *rgb, int w, int w2, int h2, float cx, float cy, int area) {
  float r = std::sqrt(area / 3.14159265f);                  // (no M_PI: not every toolchain defines it)
  if (r < 2.f) r = 2.f;
  const float rin = 1.5f * r, rout = 3.f * r + 2.f;
  int n = 0, lit = 0;
  for (int y = (int)std::floor(cy - rout); y <= (int)std::ceil(cy + rout); y++) {
    if (y < 0 || y >= h2) continue;
    for (int x = (int)std::floor(cx - rout); x <= (int)std::ceil(cx + rout); x++) {
      if (x < 0 || x >= w2) continue;
      float d2 = (y - cy) * (y - cy) + (x - cx) * (x - cx);
      if (d2 < rin * rin || d2 > rout * rout) continue;
      n++;
      if (luma(rgb, w, x, y) >= GLOW) lit++;
    }
  }
  return n ? (float)lit / n : 0.f;
}
}  // namespace

AhbLights detect_lights(const uint8_t *rgb, int w, int h, int horizon) {
  AhbLights R;
  memset(&R, 0, sizeof(R));
  const int w2 = w / DS, h2 = h / DS;
  std::vector<Comp> comps;
  // headlights: the band around the horizon
  int b0 = (horizon - ABOVE) / DS, b1 = (horizon + BELOW < BOTTOM ? horizon + BELOW : BOTTOM) / DS;
  if (b0 < 0) b0 = 0;
  if (b1 > h2) b1 = h2;
  components(rgb, w, w2, b0, b1, comps);
  int done = 0;
  for (const Comp &c : comps) {
    if (c.area < 2) continue;
    if (++done > MAX_COMPONENTS) break;
    const int bw = c.x1 - c.x0 + 1, bh = c.y1 - c.y0 + 1;
    if (bw > 3 * bh || bh > 3 * bw) continue;                 // thin / elongated: a lane marking or a sign edge
    const float cx = (float)c.sx / c.area, cy = (float)c.sy / c.area;
    const float halo = halo_of(rgb, w, w2, h2, cx, cy, c.area);
    if (halo < HALO_MIN) continue;                            // no glow around it: a reflection, not a lamp
    if (R.n < AHB_MAX_LIGHTS) R.l[R.n] = {(int)(cx * DS), (int)(cy * DS), c.area * DS * DS, halo};
    R.n++;
  }
  // street lamps: saturated sources clearly above the horizon
  int a0 = (horizon - LAMPS_SPAN) / DS, a1 = (horizon - ABOVE) / DS;
  if (a0 < 0) a0 = 0;
  if (a1 > h2) a1 = h2;
  components(rgb, w, w2, a0, a1, comps);
  for (const Comp &c : comps) if (c.area >= 2 && R.lamps_above < 99) R.lamps_above++;
  // how dark the band is (sparse)
  long sum = 0, n = 0;
  for (int y = b0; y < b1; y += 2)
    for (int x = 0; x < w2; x += 4) { sum += luma(rgb, w, x, y); n++; }
  R.band_luma = n ? (float)sum / n : 0.f;
  return R;
}

namespace {
const char *FLAG_FILE = "/data/ahb_vision_on";
const char *HORIZON_FILE = "/data/c920_horizon";      // optional: the horizon row, if the mount differs (default 325)
const char *OUT_FILE = "/dev/c920_lights.json";
const char *OUT_TMP = "/dev/c920_lights.json.tmp";
const int AHB_EVERY_N_FRAMES = 4;                      // 5 Hz at 20 fps
const double FLAG_RECHECK_S = 2.0;
int g_horizon = 325;
bool g_on = false;
double g_checked = -1e9;
long g_tick = 0;

double mono_s() { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
double wall_s() { struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }

void recheck() {
  struct stat st;
  g_on = stat(FLAG_FILE, &st) == 0;
  if (!g_on) return;
  FILE *f = fopen(HORIZON_FILE, "r");
  if (f) {
    int v = 0;
    if (fscanf(f, "%d", &v) == 1 && v >= 150 && v <= 570) g_horizon = v;
    fclose(f);
  }
}

void write_result(const AhbLights &r, int horizon) {
  FILE *f = fopen(OUT_TMP, "w");
  if (!f) return;
  fprintf(f, "{\"t\":%.3f,\"horizon\":%d,\"n\":%d,\"lamps_above\":%d,\"band_luma\":%.1f,\"lights\":[",
          wall_s(), horizon, r.n, r.lamps_above, r.band_luma);
  const int k = r.n < AHB_MAX_LIGHTS ? r.n : AHB_MAX_LIGHTS;
  for (int i = 0; i < k; i++)
    fprintf(f, "%s[%d,%d,%d,%.2f]", i ? "," : "", r.l[i].x, r.l[i].y, r.l[i].area, r.l[i].halo);
  fprintf(f, "]}\n");
  fclose(f);
  rename(OUT_TMP, OUT_FILE);
}
}  // namespace

void ahb_vision_tick(const uint8_t *rgb, int w, int h) {
  const double t = mono_s();
  if (t - g_checked >= FLAG_RECHECK_S) { recheck(); g_checked = t; }
  if (!g_on || !rgb || w < 64 || h < 64) return;
  if (g_tick++ % AHB_EVERY_N_FRAMES) return;
  write_result(detect_lights(rgb, w, h, g_horizon), g_horizon);
}
