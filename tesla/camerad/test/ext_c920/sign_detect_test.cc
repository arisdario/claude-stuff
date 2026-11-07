#include "sign_detect.h"
#include <iostream>
#include <cmath>
using namespace cv;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::cerr << "FAIL: " << msg << " (" << #cond << ")\n"; failures++; } \
                               else { std::cerr << "ok:   " << msg << "\n"; } } while (0)

// draws a red octagon of the given "radius" centered at (cx,cy) on a plain background image
static void draw_octagon(Mat &img, int cx, int cy, int r, Scalar color) {
  std::vector<Point> pts;
  for (int i = 0; i < 8; i++) {
    double a = CV_PI / 8 + i * CV_PI / 4;
    pts.push_back(Point(cx + r * cos(a), cy + r * sin(a)));
  }
  std::vector<std::vector<Point>> polys{pts};
  fillPoly(img, polys, color);
}

int main() {
  // --- 1. a clean, large stop sign should be detected, with a plausible distance ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));  // greenish "road scene" background
    draw_octagon(img, 640, 360, 120, Scalar(190, 20, 20));  // RGB-order Scalar: ~red
    auto r = detect_signs(img);
    CHECK(r.stop_sign, "large clean stop sign detected");
    // width ~ 2*120*sin(~67.5deg)=~222px box width roughly; distance = 0.75*907/222 ~= 3.1m - just
    // sanity-range check, not an exact pixel-perfect expectation (box width isn't exactly 2r):
    CHECK(r.stop_sign_dist_m > 1.0 && r.stop_sign_dist_m < 10.0, "distance estimate in a plausible range");
    CHECK(r.stop_sign_confidence > 0.5, "confidence reasonably high for a clean sign");
  }

  // --- 2. a small, far-away stop sign should still be found, with a larger estimated distance ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    draw_octagon(img, 640, 360, 20, Scalar(190, 20, 20));
    auto r = detect_signs(img);
    CHECK(r.stop_sign, "small far stop sign still detected");
    if (r.stop_sign) CHECK(r.stop_sign_dist_m > 15.0, "far sign estimated as far");
  }

  // --- 3. no red at all: no false positive ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    auto r = detect_signs(img);
    CHECK(!r.stop_sign, "empty scene: no stop sign false positive");
    CHECK(!r.light_present, "empty scene: no traffic light false positive");
  }

  // --- 4. a large red RECTANGLE (e.g. a red truck/wall) should NOT be called a stop sign ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    rectangle(img, Rect(500, 250, 300, 220), Scalar(190, 20, 20), FILLED);
    auto r = detect_signs(img);
    CHECK(!r.stop_sign, "red rectangle (truck/wall) is not called a stop sign");
  }

  // --- 5. a long red horizontal strip (e.g. brake lights / tail light bar) should NOT be a stop sign ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    rectangle(img, Rect(400, 400, 500, 40), Scalar(190, 20, 20), FILLED);
    auto r = detect_signs(img);
    CHECK(!r.stop_sign, "wide red strip (taillight bar) is not called a stop sign");
  }

  // --- 6. a red circle (traffic light lens, round) in the upper frame should be a RED light, not a stop sign ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    circle(img, Point(640, 150), 14, Scalar(200, 30, 30), FILLED);
    auto r = detect_signs(img);
    CHECK(r.light_present, "red circular light detected");
    CHECK(r.light_color == 0, "detected light color is red");
    CHECK(!r.stop_sign, "a small red circle is not mistaken for a stop sign");
  }

  // --- 7. green light ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    circle(img, Point(700, 140), 13, Scalar(30, 190, 60), FILLED);
    auto r = detect_signs(img);
    CHECK(r.light_present, "green circular light detected");
    CHECK(r.light_color == 2, "detected light color is green");
  }

  // --- 8. yellow light ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    circle(img, Point(700, 140), 13, Scalar(220, 200, 30), FILLED);
    auto r = detect_signs(img);
    CHECK(r.light_present, "yellow circular light detected");
    CHECK(r.light_color == 1, "detected light color is yellow");
  }

  // --- 9. a red circular blob in the LOWER part of the frame (plausible brake light position) should
  //         be suppressed by the upper-2/3 restriction ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    circle(img, Point(640, 690), 14, Scalar(200, 30, 30), FILLED);   // near the bottom edge
    auto r = detect_signs(img);
    CHECK(!r.light_present, "a round red blob near the bottom (brake-light position) is not read as a signal");
  }

  // --- 10. both a stop sign AND a light in frame at once - both should be found independently ---
  {
    Mat img(720, 1280, CV_8UC3, Scalar(60, 90, 60));
    draw_octagon(img, 300, 450, 80, Scalar(190, 20, 20));
    circle(img, Point(900, 150), 13, Scalar(30, 190, 60), FILLED);
    auto r = detect_signs(img);
    CHECK(r.stop_sign, "stop sign found alongside a light");
    CHECK(r.light_present && r.light_color == 2, "green light found alongside a stop sign");
  }

  std::cerr << "\n" << (failures == 0 ? "ALL PASSED" : std::to_string(failures) + " FAILURE(S)") << "\n";
  return failures == 0 ? 0 : 1;
}
