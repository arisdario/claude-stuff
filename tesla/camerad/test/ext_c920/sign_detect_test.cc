// sign_detect_test.cc - synthetic-image checks for the plain-C++ (no OpenCV) sign/light detector.
//   g++ -std=c++14 -O2 -I../../cameras sign_detect_test.cc ../../cameras/sign_detect.cc -o /tmp/sd && /tmp/sd
#include "sign_detect.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::cerr << "FAIL: " << msg << " (" << #cond << ")\n"; failures++; } \
                               else { std::cerr << "ok:   " << msg << "\n"; } } while (0)

struct Img {
  int w, h;
  std::vector<uint8_t> px;
  Img(int w_, int h_, int r, int g, int b) : w(w_), h(h_), px((size_t)w_ * h_ * 3) {
    for (size_t i = 0; i < (size_t)w * h; i++) { px[3 * i] = r; px[3 * i + 1] = g; px[3 * i + 2] = b; }
  }
  void set(int x, int y, int r, int g, int b) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    size_t i = ((size_t)y * w + x) * 3; px[i] = r; px[i + 1] = g; px[i + 2] = b;
  }
  SignResult detect() const { return detect_signs(px.data(), w, h); }
};

static void fill_rect(Img &im, int x, int y, int rw, int rh, int r, int g, int b) {
  for (int yy = y; yy < y + rh; yy++) for (int xx = x; xx < x + rw; xx++) im.set(xx, yy, r, g, b);
}
static void fill_circle(Img &im, int cx, int cy, int rad, int r, int g, int b) {
  for (int yy = cy - rad; yy <= cy + rad; yy++)
    for (int xx = cx - rad; xx <= cx + rad; xx++)
      if ((xx - cx) * (xx - cx) + (yy - cy) * (yy - cy) <= rad * rad) im.set(xx, yy, r, g, b);
}
// regular octagon of "radius" rad, flat top/bottom (vertices at 22.5 + k*45 degrees)
static void fill_octagon(Img &im, int cx, int cy, int rad, int r, int g, int b) {
  double vx[8], vy[8];
  for (int i = 0; i < 8; i++) { double a = M_PI / 8 + i * M_PI / 4; vx[i] = cx + rad * cos(a); vy[i] = cy + rad * sin(a); }
  for (int yy = cy - rad; yy <= cy + rad; yy++)
    for (int xx = cx - rad; xx <= cx + rad; xx++) {
      bool inside = true;
      for (int i = 0; i < 8 && inside; i++) {
        int j = (i + 1) % 8;
        double c = (vx[j] - vx[i]) * (yy + 0.5 - vy[i]) - (vy[j] - vy[i]) * (xx + 0.5 - vx[i]);
        if (c < 0) inside = false;
      }
      if (inside) im.set(xx, yy, r, g, b);
    }
}

#define SCENE Img img(1280, 720, 60, 90, 60)   // greenish "road scene" background

int main() {
  {  // 1. a clean, large stop sign: detected, plausible distance and confidence
    SCENE; fill_octagon(img, 640, 360, 120, 190, 20, 20);
    auto r = img.detect();
    CHECK(r.stop_sign, "large clean stop sign detected");
    CHECK(r.stop_sign_dist_m > 1.0 && r.stop_sign_dist_m < 10.0, "distance estimate in a plausible range");
    CHECK(r.stop_sign_confidence > 0.5, "confidence reasonably high for a clean sign");
  }
  {  // 2. small far stop sign still found, estimated far
    SCENE; fill_octagon(img, 640, 360, 20, 190, 20, 20);
    auto r = img.detect();
    CHECK(r.stop_sign, "small far stop sign still detected");
    if (r.stop_sign) CHECK(r.stop_sign_dist_m > 15.0, "far sign estimated as far");
  }
  {  // 3. empty scene: nothing
    SCENE;
    auto r = img.detect();
    CHECK(!r.stop_sign, "empty scene: no stop sign false positive");
    CHECK(!r.light_present, "empty scene: no traffic light false positive");
  }
  {  // 4. red rectangle (truck/wall) is not a stop sign
    SCENE; fill_rect(img, 500, 250, 300, 220, 190, 20, 20);
    CHECK(!img.detect().stop_sign, "red rectangle (truck/wall) is not called a stop sign");
  }
  {  // 5. wide red strip (taillight bar) is not a stop sign
    SCENE; fill_rect(img, 400, 400, 500, 40, 190, 20, 20);
    CHECK(!img.detect().stop_sign, "wide red strip (taillight bar) is not called a stop sign");
  }
  {  // 6. small red circle high up = red light, not a stop sign
    SCENE; fill_circle(img, 640, 150, 14, 200, 30, 30);
    auto r = img.detect();
    CHECK(r.light_present, "red circular light detected");
    CHECK(r.light_color == 0, "detected light color is red");
    CHECK(!r.stop_sign, "a small red circle is not mistaken for a stop sign");
  }
  {  // 7. green light
    SCENE; fill_circle(img, 700, 140, 13, 30, 190, 60);
    auto r = img.detect();
    CHECK(r.light_present, "green circular light detected");
    CHECK(r.light_color == 2, "detected light color is green");
  }
  {  // 8. yellow light
    SCENE; fill_circle(img, 700, 140, 13, 220, 200, 30);
    auto r = img.detect();
    CHECK(r.light_present, "yellow circular light detected");
    CHECK(r.light_color == 1, "detected light color is yellow");
  }
  {  // 9. red round blob near the bottom (brake-light position) is suppressed
    SCENE; fill_circle(img, 640, 690, 14, 200, 30, 30);
    CHECK(!img.detect().light_present, "a round red blob near the bottom (brake-light position) is not read as a signal");
  }
  {  // 10. stop sign and a green light together
    SCENE; fill_octagon(img, 300, 450, 80, 190, 20, 20); fill_circle(img, 900, 150, 13, 30, 190, 60);
    auto r = img.detect();
    CHECK(r.stop_sign, "stop sign found alongside a light");
    CHECK(r.light_present && r.light_color == 2, "green light found alongside a stop sign");
  }
  {  // 11. a real sign has white "STOP" letters: holes inside the red must not break detection
    SCENE; fill_octagon(img, 640, 360, 120, 190, 20, 20);
    for (int k = 0; k < 4; k++) fill_rect(img, 545 + k * 50, 335, 30, 50, 240, 240, 240);
    CHECK(img.detect().stop_sign, "stop sign with white letters (holes) still detected");
  }
  {  // 12. a green square (not round) high up is not a light
    SCENE; fill_rect(img, 700, 120, 28, 8, 30, 190, 60);
    CHECK(!img.detect().light_present, "a flat green bar is not read as a light");
  }
  {  // 13. speed on a full 1280x720 frame (runs every 8th frame on the EON; report, generous bound)
    SCENE; fill_octagon(img, 300, 450, 80, 190, 20, 20); fill_circle(img, 900, 150, 13, 30, 190, 60);
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 5; i++) img.detect();
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 5;
    std::cerr << "      detect_signs 1280x720: " << ms << " ms/frame on this machine\n";
    CHECK(ms < 1000, "runs in well under a second per frame");
  }
  {  // 14. bad input does not crash
    auto r = detect_signs(nullptr, 1280, 720);
    CHECK(!r.stop_sign && !r.light_present, "null frame returns an empty result");
  }

  std::cerr << "\n" << (failures == 0 ? "ALL PASSED" : std::to_string(failures) + " FAILURE(S)") << "\n";
  return failures == 0 ? 0 : 1;
}
