#include "sign_detect.h"
#include <cmath>

using namespace cv;

namespace {

// red wraps hue 0, so two ranges; "high sat/val" cuts out dull reds (rust, brick, brake-light glare
// at a distance) that would otherwise flood the mask.
void red_mask(const Mat &hsv, Mat &out) {
  Mat m1, m2;
  inRange(hsv, Scalar(0, 120, 90), Scalar(10, 255, 255), m1);
  inRange(hsv, Scalar(170, 120, 90), Scalar(180, 255, 255), m2);
  out = m1 | m2;
}
void yellow_mask(const Mat &hsv, Mat &out) { inRange(hsv, Scalar(18, 110, 120), Scalar(35, 255, 255), out); }
void green_mask(const Mat &hsv, Mat &out) { inRange(hsv, Scalar(45, 90, 80), Scalar(85, 255, 255), out); }

double contour_circularity(const std::vector<Point> &c, double area) {
  double peri = arcLength(c, true);
  if (peri <= 1e-6) return 0.0;
  return 4.0 * CV_PI * area / (peri * peri);   // 1.0 = a perfect circle
}

// stop sign: red octagon. Convex, roughly as wide as tall, 6-10 vertices once simplified (an octagon
// under perspective and approxPolyDP noise rarely comes back as a clean 8).
bool find_stop_sign(const Mat &rgb, SignResult &out) {
  Mat hsv, mask;
  cvtColor(rgb, hsv, COLOR_RGB2HSV);
  red_mask(hsv, mask);
  morphologyEx(mask, mask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

  std::vector<std::vector<Point>> contours;
  findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  double best_area = 0; bool found = false; Rect best_box; double best_solidity = 0;
  const double min_area = rgb.cols * rgb.rows * 0.0008;   // ~0.08% of frame: filters sensor noise / tiny red specs
  const double max_area = rgb.cols * rgb.rows * 0.35;     // implausibly close / a red wall, not a sign

  for (auto &c : contours) {
    double area = contourArea(c);
    if (area < min_area || area > max_area) continue;
    std::vector<Point> hull;
    convexHull(c, hull);
    double hull_area = contourArea(hull);
    if (hull_area <= 1e-6) continue;
    double solidity = area / hull_area;
    if (solidity < 0.85) continue;   // octagon is close to convex; rejects jagged/irregular red blobs

    std::vector<Point> approx;
    approxPolyDP(hull, approx, 0.03 * arcLength(hull, true), true);
    if ((int)approx.size() < 6 || (int)approx.size() > 10) continue;

    Rect box = boundingRect(c);
    double aspect = (double)box.width / std::max(1, box.height);
    if (aspect < 0.7 || aspect > 1.43) continue;   // a sign viewed near head-on is close to square

    if (area > best_area) { best_area = area; found = true; best_box = box; best_solidity = solidity; }
  }

  if (!found) return false;
  out.stop_sign = true;
  out.stop_sign_dist_m = (STOP_SIGN_WIDTH_M * NATIVE_FOCAL_PX) / std::max(1, best_box.width);
  // confidence: solidity (shape match) scaled down if the box is small (noisier estimate far away)
  double size_factor = std::min(1.0, best_area / (rgb.cols * rgb.rows * 0.01));
  out.stop_sign_confidence = std::min(1.0, best_solidity * (0.5 + 0.5 * size_factor));
  return true;
}

// traffic light: a small, highly circular colored blob, restricted to the upper part of the frame
// (lights are mounted up high; this is the single biggest practical filter against brake lights and
// red signage, which this classical approach cannot otherwise reliably tell apart from a lit signal).
bool find_light_of_color(const Mat &hsv, int color_id, void (*mask_fn)(const Mat &, Mat &),
                          int frame_h, double &best_circularity, Rect &best_box) {
  Mat mask;
  mask_fn(hsv, mask);
  Mat roi_mask = Mat::zeros(mask.size(), mask.type());
  Rect roi(0, 0, mask.cols, frame_h * 2 / 3);   // top two-thirds only
  mask(roi).copyTo(roi_mask(roi));
  morphologyEx(roi_mask, roi_mask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

  std::vector<std::vector<Point>> contours;
  findContours(roi_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  bool found = false;
  const double min_area = 12, max_area = mask.cols * mask.rows * 0.01;   // a light lens is small in frame
  for (auto &c : contours) {
    double area = contourArea(c);
    if (area < min_area || area > max_area) continue;
    double circ = contour_circularity(c, area);
    if (circ < 0.65) continue;
    if (circ > best_circularity) { best_circularity = circ; best_box = boundingRect(c); found = true; }
  }
  return found;
}

bool find_traffic_light(const Mat &rgb, SignResult &out) {
  Mat hsv;
  cvtColor(rgb, hsv, COLOR_RGB2HSV);
  double red_c = 0, yellow_c = 0, green_c = 0;
  Rect red_box, yellow_box, green_box;
  bool r = find_light_of_color(hsv, 0, red_mask, rgb.rows, red_c, red_box);
  bool y = find_light_of_color(hsv, 1, yellow_mask, rgb.rows, yellow_c, yellow_box);
  bool g = find_light_of_color(hsv, 2, green_mask, rgb.rows, green_c, green_box);
  if (!r && !y && !g) return false;

  // pick whichever color's best candidate is the most circular (least likely a false positive)
  double best = -1; int color = -1;
  if (r && red_c > best) { best = red_c; color = 0; }
  if (y && yellow_c > best) { best = yellow_c; color = 1; }
  if (g && green_c > best) { best = green_c; color = 2; }

  out.light_present = true;
  out.light_color = color;
  out.light_confidence = best;   // circularity doubles as the confidence score (0..1)
  return true;
}

}  // namespace

SignResult detect_signs(const Mat &rgb) {
  SignResult out;
  if (rgb.empty() || rgb.channels() != 3) return out;
  find_stop_sign(rgb, out);
  find_traffic_light(rgb, out);
  return out;
}
