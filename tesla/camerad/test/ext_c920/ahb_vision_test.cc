// ahb_vision_test.cc - synthetic-scene checks for the shadow auto-high-beam light finder (cameras/ahb_vision.cc),
// plus a raw mode used to compare it with the Python prototype on real frames.
//   g++ -std=c++11 -O2 -I../../cameras ahb_vision_test.cc ../../cameras/ahb_vision.cc -o /tmp/av && /tmp/av
//   /tmp/av --raw frame.rgb 1280 720 325     -> prints the lights of one raw RGB frame (x y area halo per line)
#include "ahb_vision.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::cerr << "FAIL: " << msg << " (" << #cond << ")\n"; failures++; } \
                               else { std::cerr << "ok:   " << msg << "\n"; } } while (0)

const int W = 1280, H = 720, HOR = 325;

struct Img {
  std::vector<uint8_t> px;
  Img(int v = 12) : px((size_t)W * H * 3, (uint8_t)v) {}
  void set(int x, int y, int v) {
    if (x < 0 || y < 0 || x >= W || y >= H) return;
    size_t i = ((size_t)y * W + x) * 3; px[i] = px[i + 1] = px[i + 2] = (uint8_t)v;
  }
  void disk(int cx, int cy, double r, int v) {
    for (int y = (int)(cy - r); y <= (int)(cy + r); y++)
      for (int x = (int)(cx - r); x <= (int)(cx + r); x++)
        if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r) set(x, y, v);
  }
  void lamp(int cx, int cy, double core, double glow) { disk(cx, cy, glow, 170); disk(cx, cy, core, 255); }  // source + halo
  void rect(int x0, int y0, int w, int h, int v) { for (int y = y0; y < y0 + h; y++) for (int x = x0; x < x0 + w; x++) set(x, y, v); }
  AhbLights run() const { return detect_lights(px.data(), W, H, HOR); }
};

int main(int argc, char **argv) {
  if (argc >= 6 && !strcmp(argv[1], "--raw")) {
    int w = atoi(argv[3]), h = atoi(argv[4]), hor = atoi(argv[5]);
    std::vector<uint8_t> px((size_t)w * h * 3);
    FILE *f = fopen(argv[2], "rb");
    if (!f || fread(px.data(), 1, px.size(), f) != px.size()) { std::cerr << "cannot read " << argv[2] << "\n"; return 2; }
    fclose(f);
    AhbLights r = detect_lights(px.data(), w, h, hor);
    printf("n %d lamps %d luma %.1f\n", r.n, r.lamps_above, r.band_luma);
    for (int i = 0; i < r.n && i < AHB_MAX_LIGHTS; i++) printf("%d %d %d %.2f\n", r.l[i].x, r.l[i].y, r.l[i].area, r.l[i].halo);
    return 0;
  }

  { Img im; AhbLights r = im.run(); CHECK(r.n == 0 && r.lamps_above == 0, "dark empty road: nothing"); CHECK(r.band_luma < 20, "dark band"); }
  { Img im; im.lamp(500, HOR + 12, 6, 18); AhbLights r = im.run();
    CHECK(r.n == 1, "one oncoming headlight (core + glow) at the horizon");
    CHECK(r.n >= 1 && std::abs(r.l[0].x - 500) <= 2 && std::abs(r.l[0].y - (HOR + 12)) <= 2, "... at the right place"); }
  { Img im; im.lamp(470, HOR + 30, 9, 26); im.lamp(560, HOR + 30, 9, 26); AhbLights r = im.run(); CHECK(r.n == 2, "a pair of headlights"); }
  { Img im; im.rect(600, HOR + 50, 70, 6, 255); AhbLights r = im.run(); CHECK(r.n == 0, "lit lane marking (long, no glow): ignored"); }
  { Img im; im.disk(900, HOR + 20, 3, 255); AhbLights r = im.run(); CHECK(r.n == 0, "guardrail reflector (small, no glow): ignored"); }
  { Img im; im.lamp(300, HOR - 150, 7, 22); im.lamp(700, HOR - 80, 7, 22); AhbLights r = im.run();
    CHECK(r.n == 0, "street lamps above the horizon: not a car"); CHECK(r.lamps_above == 2, "... counted as street lamps"); }
  { Img im; im.lamp(640, HOR + 200, 8, 24); AhbLights r = im.run(); CHECK(r.n == 0, "far below the band (hood / dash reflection): ignored"); }
  { Img im; im.lamp(5, HOR, 6, 18); im.lamp(1275, HOR + 4, 6, 18); AhbLights r = im.run(); CHECK(r.n == 2, "lamps at the picture edges: no crash, found"); }
  { Img im(255); AhbLights r = im.run(); CHECK(r.n <= 1, "all-white picture (camera blinded): bounded, no crash"); }
  {   // worst case speed: a busy, lit scene
    Img im(40);
    for (int i = 0; i < 60; i++) im.lamp(20 + i * 21, HOR - 20 + (i % 7) * 15, 4, 12);
    for (int i = 0; i < 40; i++) im.lamp(30 + i * 31, HOR - 300 + (i % 5) * 50, 5, 15);
    auto t0 = std::chrono::steady_clock::now();
    const int N = 50;
    AhbLights r;
    for (int k = 0; k < N; k++) r = im.run();
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / N;
    std::cerr << "busy scene: " << r.n << " lights, " << r.lamps_above << " lamps, " << ms << " ms per run on this PC\n";
    CHECK(ms < 5.0, "busy scene under 5 ms on a PC");
  }
  std::cerr << (failures ? "FAILED: " : "all passed: ") << failures << "\n";
  return failures ? 1 : 0;
}
