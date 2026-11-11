// sign_detect.cc - plain C++ (no OpenCV) port of the stop-sign / traffic-light detector. Same pipeline
// and thresholds as the original OpenCV version; see SIGN_DETECT.md.
#include "sign_detect.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace {

typedef std::vector<uint8_t> Mask;   // w*h, 0 or 1

struct Hsv {   // OpenCV 8-bit convention: H 0..180, S 0..255, V 0..255
  std::vector<uint8_t> h, s, v;
  int w = 0, ht = 0;
};

void to_hsv(const uint8_t *rgb, int w, int ht, Hsv &out) {
  const size_t n = (size_t)w * ht;
  out.w = w; out.ht = ht;
  out.h.resize(n); out.s.resize(n); out.v.resize(n);
  for (size_t i = 0; i < n; i++) {
    int r = rgb[3 * i], g = rgb[3 * i + 1], b = rgb[3 * i + 2];
    int mx = std::max(r, std::max(g, b)), mn = std::min(r, std::min(g, b));
    int diff = mx - mn;
    out.v[i] = (uint8_t)mx;
    out.s[i] = mx == 0 ? 0 : (uint8_t)((255 * diff + mx / 2) / mx);
    double hh = 0;
    if (diff != 0) {
      if (mx == r) hh = 60.0 * (g - b) / diff;
      else if (mx == g) hh = 120.0 + 60.0 * (b - r) / diff;
      else hh = 240.0 + 60.0 * (r - g) / diff;
      if (hh < 0) hh += 360.0;
    }
    int h2 = (int)std::lround(hh / 2.0);
    out.h[i] = (uint8_t)(h2 >= 180 ? h2 - 180 : h2);
  }
}

inline bool in_range(const Hsv &p, size_t i, int h0, int s0, int v0, int h1, int s1, int v1) {
  return p.h[i] >= h0 && p.h[i] <= h1 && p.s[i] >= s0 && p.s[i] <= s1 && p.v[i] >= v0 && p.v[i] <= v1;
}

// red wraps hue 0, so two ranges; "high sat/val" cuts out dull reds (rust, brick, brake-light glare
// at a distance) that would otherwise flood the mask.
void red_mask(const Hsv &p, Mask &m) {
  m.assign(p.h.size(), 0);
  for (size_t i = 0; i < m.size(); i++)
    m[i] = in_range(p, i, 0, 120, 90, 10, 255, 255) || in_range(p, i, 170, 120, 90, 180, 255, 255);
}
void yellow_mask(const Hsv &p, Mask &m) {
  m.assign(p.h.size(), 0);
  for (size_t i = 0; i < m.size(); i++) m[i] = in_range(p, i, 18, 110, 120, 35, 255, 255);
}
void green_mask(const Hsv &p, Mask &m) {
  m.assign(p.h.size(), 0);
  for (size_t i = 0; i < m.size(); i++) m[i] = in_range(p, i, 45, 90, 80, 85, 255, 255);
}

// structuring elements identical to OpenCV's getStructuringElement(MORPH_ELLIPSE, 3x3 / 5x5)
typedef std::vector<std::pair<int, int>> Kernel;
Kernel ellipse3() { return {{0, -1}, {-1, 0}, {0, 0}, {1, 0}, {0, 1}}; }
Kernel ellipse5() {
  Kernel k;
  for (int dy = -2; dy <= 2; dy++)
    for (int dx = -2; dx <= 2; dx++)
      if (std::abs(dy) < 2 || dx == 0) k.push_back({dx, dy});
  return k;
}

// erode/dilate; pixels outside the image are ignored (same effect as OpenCV's default border value)
void morph(const Mask &in, Mask &out, int w, int h, const Kernel &k, bool dilate) {
  out.assign(in.size(), 0);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      bool v = !dilate;
      for (auto &o : k) {
        int xx = x + o.first, yy = y + o.second;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        uint8_t p = in[(size_t)yy * w + xx];
        if (dilate && p) { v = true; break; }
        if (!dilate && !p) { v = false; break; }
      }
      out[(size_t)y * w + x] = v;
    }
  }
}
void close_op(Mask &m, int w, int h, const Kernel &k) { Mask t; morph(m, t, w, h, k, true); morph(t, m, w, h, k, false); }
void open_op(Mask &m, int w, int h, const Kernel &k) { Mask t; morph(m, t, w, h, k, false); morph(t, m, w, h, k, true); }

struct P { double x, y; };

double cross(const P &o, const P &a, const P &b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

std::vector<P> convex_hull(std::vector<P> pts) {   // Andrew's monotone chain, counter-clockwise
  std::sort(pts.begin(), pts.end(), [](const P &a, const P &b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  pts.erase(std::unique(pts.begin(), pts.end(), [](const P &a, const P &b) { return a.x == b.x && a.y == b.y; }), pts.end());
  if (pts.size() < 3) return pts;
  std::vector<P> hull(2 * pts.size());
  size_t k = 0;
  for (size_t i = 0; i < pts.size(); i++) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) k--;
    hull[k++] = pts[i];
  }
  for (size_t i = pts.size() - 1, t = k + 1; i > 0; i--) {
    while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i - 1]) <= 0) k--;
    hull[k++] = pts[i - 1];
  }
  hull.resize(k - 1);
  return hull;
}

double poly_area(const std::vector<P> &p) {
  double a = 0;
  for (size_t i = 0; i < p.size(); i++) { const P &u = p[i], &v = p[(i + 1) % p.size()]; a += u.x * v.y - v.x * u.y; }
  return std::fabs(a) / 2.0;
}
double poly_perimeter(const std::vector<P> &p) {
  double s = 0;
  for (size_t i = 0; i < p.size(); i++) { const P &u = p[i], &v = p[(i + 1) % p.size()]; s += std::hypot(u.x - v.x, u.y - v.y); }
  return s;
}

double seg_dist(const P &p, const P &a, const P &b) {
  double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
  if (len2 <= 1e-12) return std::hypot(p.x - a.x, p.y - a.y);
  return std::fabs(dx * (a.y - p.y) - dy * (a.x - p.x)) / std::sqrt(len2);
}
// Douglas-Peucker on the open chain pts[i..j] (indices into a closed ring of size n); marks kept points
void dp(const std::vector<P> &r, size_t i, size_t j, double eps, std::vector<bool> &keep) {
  const size_t n = r.size();
  size_t span = (j + n - i) % n;
  if (span < 2) return;
  double best = -1; size_t bi = i;
  for (size_t s = 1; s < span; s++) {
    size_t k = (i + s) % n;
    double d = seg_dist(r[k], r[i], r[j]);
    if (d > best) { best = d; bi = k; }
  }
  if (best > eps) { keep[bi] = true; dp(r, i, bi, eps, keep); dp(r, bi, j, eps, keep); }
}
// approxPolyDP(closed) equivalent: number of vertices left after simplifying the ring
int simplified_vertex_count(const std::vector<P> &ring, double eps) {
  const size_t n = ring.size();
  if (n < 3) return (int)n;
  size_t far = 0; double fd = -1;   // split the ring at point 0 and the point farthest from it
  for (size_t k = 1; k < n; k++) {
    double d = std::hypot(ring[k].x - ring[0].x, ring[k].y - ring[0].y);
    if (d > fd) { fd = d; far = k; }
  }
  std::vector<bool> keep(n, false);
  keep[0] = keep[far] = true;
  dp(ring, 0, far, eps, keep);
  dp(ring, far, 0, eps, keep);
  return (int)std::count(keep.begin(), keep.end(), true);
}

struct Blob {
  double filled_area = 0;   // pixel area with interior holes filled (= OpenCV external-contour area)
  int x0, y0, x1, y1;       // inclusive bbox
  std::vector<P> hull;
  double hull_area = 0, hull_perim = 0;
  int bw() const { return x1 - x0 + 1; }
  int bh() const { return y1 - y0 + 1; }
};

// 8-connected components of `m`. For each component: bbox, hole-filled area, convex hull of its
// pixel squares. Components with fewer than min_px pixels are dropped early (cheap noise filter).
std::vector<Blob> find_blobs(const Mask &m, int w, int h, int min_px) {
  std::vector<Blob> blobs;
  std::vector<int> label(m.size(), 0);
  std::vector<int> stack;
  int next = 0;
  for (int sy = 0; sy < h; sy++) {
    for (int sx = 0; sx < w; sx++) {
      size_t si = (size_t)sy * w + sx;
      if (!m[si] || label[si]) continue;
      ++next;
      Blob b; b.x0 = b.x1 = sx; b.y0 = b.y1 = sy;
      std::vector<int> pix;
      stack.assign(1, (int)si); label[si] = next;
      while (!stack.empty()) {
        int i = stack.back(); stack.pop_back();
        pix.push_back(i);
        int x = i % w, y = i / w;
        b.x0 = std::min(b.x0, x); b.x1 = std::max(b.x1, x); b.y0 = std::min(b.y0, y); b.y1 = std::max(b.y1, y);
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            int xx = x + dx, yy = y + dy;
            if ((dx || dy) && xx >= 0 && yy >= 0 && xx < w && yy < h) {
              size_t j = (size_t)yy * w + xx;
              if (m[j] && !label[j]) { label[j] = next; stack.push_back((int)j); }
            }
          }
      }
      if ((int)pix.size() < min_px) continue;

      // fill holes: flood the background from the border of a 1px-padded bbox; whatever the flood
      // can't reach (blob pixels + enclosed holes) is the filled area
      int bw = b.bw() + 2, bh = b.bh() + 2;
      std::vector<uint8_t> local((size_t)bw * bh, 0);   // 1 = blob, 2 = reached background
      for (int i : pix) local[(size_t)(i / w - b.y0 + 1) * bw + (i % w - b.x0 + 1)] = 1;
      std::vector<int> q(1, 0); local[0] = 2;
      size_t reached = 1;
      while (!q.empty()) {
        int i = q.back(); q.pop_back();
        int x = i % bw, y = i / bw;
        const int nx[4] = {x - 1, x + 1, x, x}, ny[4] = {y, y, y - 1, y + 1};
        for (int k = 0; k < 4; k++) {
          if (nx[k] < 0 || ny[k] < 0 || nx[k] >= bw || ny[k] >= bh) continue;
          size_t j = (size_t)ny[k] * bw + nx[k];
          if (local[j] == 0) { local[j] = 2; reached++; q.push_back((int)j); }
        }
      }
      b.filled_area = (double)((size_t)bw * bh - reached);

      // hull of the pixel squares: only each row's leftmost/rightmost pixel corners can be on it
      std::vector<int> rmin(b.bh(), w), rmax(b.bh(), -1);
      for (int i : pix) { int r = i / w - b.y0, x = i % w; rmin[r] = std::min(rmin[r], x); rmax[r] = std::max(rmax[r], x); }
      std::vector<P> pts;
      for (int r = 0; r < b.bh(); r++) {
        if (rmax[r] < 0) continue;
        double y = b.y0 + r;
        pts.push_back({(double)rmin[r], y}); pts.push_back({(double)rmin[r], y + 1});
        pts.push_back({(double)rmax[r] + 1, y}); pts.push_back({(double)rmax[r] + 1, y + 1});
      }
      b.hull = convex_hull(pts);
      b.hull_area = poly_area(b.hull);
      b.hull_perim = poly_perimeter(b.hull);
      blobs.push_back(std::move(b));
    }
  }
  return blobs;
}

// stop sign: red octagon. Convex, roughly as wide as tall, 6-10 vertices once simplified (an octagon
// under perspective and simplification noise rarely comes back as a clean 8).
bool find_stop_sign(const Hsv &hsv, int scale, SignResult &out) {
  const int w = hsv.w, h = hsv.ht;
  Mask mask;
  red_mask(hsv, mask);
  close_op(mask, w, h, ellipse5());
  open_op(mask, w, h, ellipse3());

  const double min_area = w * h * 0.0008;   // ~0.08% of frame: filters sensor noise / tiny red specs
  const double max_area = w * h * 0.35;     // implausibly close / a red wall, not a sign

  double best_area = 0, best_solidity = 0; int best_w = 0; bool found = false;
  for (auto &b : find_blobs(mask, w, h, (int)(min_area * 0.5))) {
    double area = b.filled_area;
    if (area < min_area || area > max_area) continue;
    if (b.hull_area <= 1e-6) continue;
    double solidity = std::min(1.0, area / b.hull_area);
    if (solidity < 0.85) continue;   // octagon is close to convex; rejects jagged/irregular red blobs

    int verts = simplified_vertex_count(b.hull, 0.03 * b.hull_perim);
    if (verts < 6 || verts > 10) continue;

    double aspect = (double)b.bw() / std::max(1, b.bh());
    if (aspect < 0.7 || aspect > 1.43) continue;   // a sign viewed near head-on is close to square

    if (area > best_area) { best_area = area; best_solidity = solidity; best_w = b.bw(); found = true; }
  }
  if (!found) return false;
  out.stop_sign = true;
  out.stop_sign_dist_m = (STOP_SIGN_WIDTH_M * NATIVE_FOCAL_PX) / std::max(1, best_w * scale);   // width in native px
  // confidence: solidity (shape match) scaled down if the blob is small (noisier estimate far away)
  double size_factor = std::min(1.0, best_area / (w * h * 0.01));
  out.stop_sign_confidence = std::min(1.0, best_solidity * (0.5 + 0.5 * size_factor));
  return true;
}

// traffic light: a small, highly circular colored blob, restricted to the upper part of the frame
// (lights are mounted up high; this is the single biggest practical filter against brake lights and
// red signage, which this classical approach cannot otherwise reliably tell apart from a lit signal).
bool find_light_of_color(const Hsv &hsv, void (*mask_fn)(const Hsv &, Mask &), double &best_circ) {
  const int w = hsv.w, h = hsv.ht;
  Mask mask;
  mask_fn(hsv, mask);
  std::fill(mask.begin() + (size_t)(h * 2 / 3) * w, mask.end(), 0);   // top two-thirds only
  open_op(mask, w, h, ellipse3());

  // a light lens is small in frame. 12 px was tuned at the native 1280x720; scaled to the working size.
  const double min_area = std::max(3.0, 12.0 * w * h / (1280.0 * 720.0)), max_area = w * h * 0.01;
  bool found = false;
  for (auto &b : find_blobs(mask, w, h, 2)) {
    double area = b.filled_area;
    if (area < min_area || area > max_area || b.hull_perim <= 1e-6 || b.hull_area <= 1e-6) continue;
    if (area / b.hull_area < 0.85) continue;   // hull perimeter is only a fair measure for a convex blob
    double circ = std::min(1.0, 4.0 * M_PI * area / (b.hull_perim * b.hull_perim));   // 1.0 = perfect circle
    if (circ < 0.65) continue;
    if (circ > best_circ) { best_circ = circ; found = true; }
  }
  return found;
}

bool find_traffic_light(const Hsv &hsv, SignResult &out) {
  double red_c = 0, yellow_c = 0, green_c = 0;
  bool r = find_light_of_color(hsv, red_mask, red_c);
  bool y = find_light_of_color(hsv, yellow_mask, yellow_c);
  bool g = find_light_of_color(hsv, green_mask, green_c);
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

SignResult detect_signs(const uint8_t *rgb, int w, int h) {
  SignResult out;
  if (!rgb || w <= 0 || h <= 0) return out;
  // Work at half resolution for big frames: 4x less CPU on the EON, and a stop sign / light lens is
  // still many pixels wide at 640x360. Distances are converted back to native pixels.
  int scale = (w >= 960 && h >= 540) ? 2 : 1;
  std::vector<uint8_t> small;
  const uint8_t *src = rgb;
  int sw = w / scale, sh = h / scale;
  if (scale == 2) {
    small.resize((size_t)sw * sh * 3);
    for (int y = 0; y < sh; y++)
      for (int x = 0; x < sw; x++)
        for (int c = 0; c < 3; c++) {
          const uint8_t *a = rgb + ((size_t)(2 * y) * w + 2 * x) * 3 + c, *b = a + (size_t)w * 3;
          small[((size_t)y * sw + x) * 3 + c] = (uint8_t)((a[0] + a[3] + b[0] + b[3] + 2) / 4);
        }
    src = small.data();
  }
  Hsv hsv;
  to_hsv(src, sw, sh, hsv);
  find_stop_sign(hsv, scale, out);
  find_traffic_light(hsv, out);
  return out;
}
