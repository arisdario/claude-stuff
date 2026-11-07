#include "sign_detect_io.h"
#include "sign_detect.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>

namespace {
const char *FLAG_FILE = "/data/sign_detect_on";
const char *OUT_FILE = "/data/sign_detect.json";
const char *OUT_TMP = "/data/sign_detect.json.tmp";
const int SIGN_DETECT_EVERY_N_FRAMES = 8;    // ~2.5 Hz at the C920's 20 fps - plenty for a sign/light, not every-frame CPU cost
const double FLAG_RECHECK_S = 2.0;           // how often to stat() the flag file, not every frame

long frame_counter = 0;
bool flag_cached = false;
double flag_checked_at = -1e9;

double now_s() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

bool file_exists(const char *path) {
  struct stat st;
  return stat(path, &st) == 0;
}

bool detection_enabled() {
  double t = now_s();
  if (t - flag_checked_at >= FLAG_RECHECK_S) {
    flag_cached = file_exists(FLAG_FILE);
    flag_checked_at = t;
  }
  return flag_cached;
}

void write_result(const SignResult &r) {
  FILE *f = fopen(OUT_TMP, "w");
  if (!f) return;
  fprintf(f,
          "{\"t\":%.3f,\"stop_sign\":%s,\"stop_sign_dist_m\":%.2f,\"stop_sign_confidence\":%.2f,"
          "\"light_present\":%s,\"light_color\":%d,\"light_confidence\":%.2f}\n",
          now_s(), r.stop_sign ? "true" : "false", r.stop_sign_dist_m, r.stop_sign_confidence,
          r.light_present ? "true" : "false", r.light_color, r.light_confidence);
  fclose(f);
  rename(OUT_TMP, OUT_FILE);   // atomic: a reader never sees a partial write
}

}  // namespace

void sign_detect_tick(const uint8_t *rgb, int w, int h) {
  if (!detection_enabled()) return;
  frame_counter++;
  if (frame_counter % SIGN_DETECT_EVERY_N_FRAMES != 0) return;
  if (!rgb || w <= 0 || h <= 0) return;
  cv::Mat frame(h, w, CV_8UC3, const_cast<uint8_t *>(rgb));   // wraps the existing buffer, no copy
  SignResult r = detect_signs(frame);
  write_result(r);
}
