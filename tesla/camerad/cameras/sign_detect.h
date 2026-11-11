#pragma once
// sign_detect.h - classical CV (no neural net, NO OpenCV) stop-sign / traffic-light detection, for the
// C920's native 1280x720 RGB frame (JCS_RGB, matches camera_ext_c920.cc's decode - see that file for the
// 907px focal length this distance estimate uses). Pure function: takes a frame, returns a result
// struct. No file I/O, no CAN, no globals - the camerad integration (sign_detect_io.cc) is the only
// part that touches anything outside this translation unit.
//
// Plain C++ on purpose: the EON's NEOS has no OpenCV, so the earlier OpenCV version could never be
// built there. Everything it used (RGB->HSV, inRange, morphology, external contours, convex hull,
// polygon simplification, circularity) is reimplemented in sign_detect.cc with the same thresholds.
#include <cstdint>

struct SignResult {
  bool stop_sign = false;
  double stop_sign_dist_m = -1.0;   // -1 = not detected
  double stop_sign_confidence = 0.0; // 0..1, see SIGN_DETECT.md for what this is built from

  bool light_present = false;
  int light_color = -1;              // -1 none, 0 red, 1 yellow, 2 green
  double light_confidence = 0.0;
};

// focal length of the native C920 1280x720 capture, per camera_ext_c920.cc's own comment ("~907 px").
constexpr double NATIVE_FOCAL_PX = 907.0;
// real-world stop sign width. ADJUST FOR YOUR REGION: this default is a generic placeholder, not a
// verified regional standard - sign size varies by country and road class. Get this right before
// trusting the distance number; a wrong constant only skews distance, never a false positive/negative.
constexpr double STOP_SIGN_WIDTH_M = 0.75;

// detect_signs: rgb = w*h*3 bytes, 8-bit RGB (not BGR), row-major, no padding. Ideally the native
// 1280x720 C920 frame (before the crop/pad to the model canvas - keeps the focal length valid).
SignResult detect_signs(const uint8_t *rgb, int w, int h);
