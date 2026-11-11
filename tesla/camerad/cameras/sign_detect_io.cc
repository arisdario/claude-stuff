#include "sign_detect_io.h"
#include "sign_detect.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <pthread.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <vector>

namespace {
const char *FLAG_FILE = "/data/sign_detect_on";
const char *OUT_FILE = "/data/sign_detect.json";
const char *OUT_TMP = "/data/sign_detect.json.tmp";
const int SIGN_DETECT_EVERY_N_FRAMES = 8;    // ~2.5 Hz at the C920's 20 fps - plenty for a sign/light, not every-frame CPU cost
const double FLAG_RECHECK_S = 2.0;           // how often to stat() the flag file, not every frame

long frame_counter = 0;
bool flag_cached = false;
double flag_checked_at = -1e9;

double now_s() {   // monotonic: internal rate-limiting/flag-recheck only, robust to wall-clock jumps
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

double wall_now_s() {   // CLOCK_REALTIME: for the JSON "t" field specifically - sign_detect_reader.py
  struct timespec ts;   // compares it against Python's time.time() (also wall-clock/Unix-epoch). Using
  clock_gettime(CLOCK_REALTIME, &ts);   // CLOCK_MONOTONIC here would make every detection look billions
  return ts.tv_sec + ts.tv_nsec * 1e-9;   // of seconds old to the Python side and silently reject all of them.
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
          wall_now_s(), r.stop_sign ? "true" : "false", r.stop_sign_dist_m, r.stop_sign_confidence,
          r.light_present ? "true" : "false", r.light_color, r.light_confidence);
  fclose(f);
  rename(OUT_TMP, OUT_FILE);   // atomic: a reader never sees a partial write
}

// Detection runs on its own background thread so camerad's capture thread (which feeds the driving
// model) only ever pays for one memcpy of the frame. If the worker is still busy with the previous
// frame, the new one is simply skipped.
pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_cv = PTHREAD_COND_INITIALIZER;
std::vector<uint8_t> g_buf;
int g_w = 0, g_h = 0;
bool g_pending = false;     // a frame is waiting for / being processed by the worker
bool g_worker_up = false;

void *worker(void *) {
  setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);   // lower priority than camerad/model (this thread only)
  std::vector<uint8_t> frame;
  for (;;) {
    int w, h;
    pthread_mutex_lock(&g_mu);
    while (!g_pending) pthread_cond_wait(&g_cv, &g_mu);
    frame.swap(g_buf);
    w = g_w; h = g_h;
    pthread_mutex_unlock(&g_mu);

    SignResult r = detect_signs(frame.data(), w, h);
    write_result(r);

    pthread_mutex_lock(&g_mu);
    g_pending = false;
    pthread_mutex_unlock(&g_mu);
  }
  return nullptr;
}

}  // namespace

void sign_detect_tick(const uint8_t *rgb, int w, int h) {
  if (!detection_enabled()) return;
  frame_counter++;
  if (frame_counter % SIGN_DETECT_EVERY_N_FRAMES != 0) return;
  if (!rgb || w <= 0 || h <= 0) return;

  if (pthread_mutex_trylock(&g_mu) != 0) return;   // never block the capture thread
  if (!g_worker_up) {
    pthread_t t;
    if (pthread_create(&t, NULL, worker, NULL) == 0) { pthread_detach(t); g_worker_up = true; }
  }
  if (g_worker_up && !g_pending) {
    const size_t n = (size_t)w * h * 3;
    g_buf.resize(n);
    memcpy(g_buf.data(), rgb, n);
    g_w = w; g_h = h;
    g_pending = true;
    pthread_cond_signal(&g_cv);
  }
  pthread_mutex_unlock(&g_mu);
}
