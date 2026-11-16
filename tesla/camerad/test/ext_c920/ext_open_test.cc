// End-to-end test of the format choice in camera_ext_c920.cc against a FAKE camera (the uvc_* calls are replaced with --wrap, everything else is the real file).
//   ext_open_test SCENARIO CLIP.h264 FRAME.jpg      prints "RESULT ok=<selftest> started=<mjpeg|h264|none>"
// scenarios: normal (camera offers both) | no_h264 (camera has no H.264 mode) | start_fails (H.264 mode negotiates but will not start) | garbage (H.264 frames are junk)
// RESTART IN PLACE: restart_call (a failed try restarts) | restart_hang (watchdog restart, then a second hang must restart again) | restart_stopping (openpilot stopping: exit, no restart)
//   these print "RESULT restarted gen=N same_pid=0|1 alarm_blocked=0|1 inherited_fd_open=0|1" from the last generation
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include "camera_ext_c920.h"
#include "libuvc/libuvc.h"

extern "C" {
static const char *g_scn = "normal";
static std::vector<uint8_t> g_clip, g_jpg;
static std::vector<std::pair<size_t, size_t>> g_aus;
static uvc_frame_callback_t *g_cb = NULL;
static volatile int g_run = 0;
static pthread_t g_th;
static int g_fmt = 0;                       // the format asked for in the last get_stream_ctrl_format_size
static const char *g_started = "none";
volatile sig_atomic_t do_exit = 0;

static void *feeder(void *) {
  size_t i = 0;
  while (g_run) {
    uvc_frame_t f; memset(&f, 0, sizeof f);
    if (g_fmt == UVC_FRAME_FORMAT_H264) {
      f.frame_format = UVC_FRAME_FORMAT_H264;
      if (!strcmp(g_scn, "garbage")) { static uint8_t junk[3000]; for (size_t k = 0; k < sizeof junk; k++) junk[k] = (uint8_t)(k * 131 + 7); f.data = junk; f.data_bytes = sizeof junk; }
      else { auto a = g_aus[i++ % g_aus.size()]; f.data = &g_clip[a.first]; f.data_bytes = a.second; }
    } else {
      f.frame_format = UVC_FRAME_FORMAT_MJPEG; f.data = g_jpg.data(); f.data_bytes = g_jpg.size();
    }
    g_cb(&f, NULL);
    usleep(8000);
  }
  return NULL;
}

uvc_error_t __wrap_uvc_init(uvc_context_t **ctx, struct libusb_context *) { *ctx = (uvc_context_t *)1; return UVC_SUCCESS; }
uvc_error_t __wrap_uvc_find_device(uvc_context_t *, uvc_device_t **dev, int, int, const char *) { *dev = (uvc_device_t *)1; return UVC_SUCCESS; }
uvc_error_t __wrap_uvc_open(uvc_device_t *, uvc_device_handle_t **h) {
  if (!strcmp(g_scn, "hang_open")) for (;;) pause();             // 15 Nov 2026: the C920 stopped answering in the middle of opening
  *h = (uvc_device_handle_t *)1; return UVC_SUCCESS;
}
void __wrap_uvc_close(uvc_device_handle_t *) {}
void __wrap_uvc_unref_device(uvc_device_t *) {}
void __wrap_uvc_exit(uvc_context_t *) {}
const uvc_extension_unit_t *__wrap_uvc_get_extension_units(uvc_device_handle_t *) { return NULL; }
uvc_error_t __wrap_uvc_set_ae_priority(uvc_device_handle_t *, uint8_t) { return UVC_SUCCESS; }
uvc_error_t __wrap_uvc_set_focus_auto(uvc_device_handle_t *, uint8_t) { return UVC_SUCCESS; }
uvc_error_t __wrap_uvc_set_focus_abs(uvc_device_handle_t *, uint16_t) { return UVC_SUCCESS; }
uvc_error_t __wrap_uvc_get_stream_ctrl_format_size(uvc_device_handle_t *, uvc_stream_ctrl_t *ctrl, enum uvc_frame_format fmt, int, int, int) {
  if (fmt == UVC_FRAME_FORMAT_H264 && !strcmp(g_scn, "no_h264")) return UVC_ERROR_INVALID_MODE;
  memset(ctrl, 0, sizeof *ctrl); g_fmt = fmt; return UVC_SUCCESS;
}
uvc_error_t __wrap_uvc_start_streaming(uvc_device_handle_t *, uvc_stream_ctrl_t *, uvc_frame_callback_t *cb, void *, uint8_t) {
  if (g_fmt == UVC_FRAME_FORMAT_H264 && !strcmp(g_scn, "start_fails")) return UVC_ERROR_IO;
  g_cb = cb; g_run = 1; g_started = g_fmt == UVC_FRAME_FORMAT_H264 ? "h264" : "mjpeg";
  pthread_create(&g_th, NULL, feeder, NULL);
  return UVC_SUCCESS;
}
void __wrap_uvc_stop_streaming(uvc_device_handle_t *) { if (g_run) { g_run = 0; pthread_join(g_th, NULL); } }
}

static std::vector<uint8_t> slurp(const char *p) { std::vector<uint8_t> v; FILE *f = fopen(p, "rb"); uint8_t b[65536]; size_t n; while (f && (n = fread(b, 1, sizeof b, f)) > 0) v.insert(v.end(), b, b + n); if (f) fclose(f); return v; }

int main(int argc, char **argv) {
  if (argc < 4) return 2;
  g_scn = argv[1]; g_clip = slurp(argv[2]); g_jpg = slurp(argv[3]);
  std::vector<size_t> st;
  for (size_t i = 0; i + 4 < g_clip.size(); i++)
    if (g_clip[i] == 0 && g_clip[i + 1] == 0 && g_clip[i + 2] == 1 && (g_clip[i + 3] & 31) == 9) { st.push_back((i > 0 && g_clip[i - 1] == 0) ? i - 1 : i); i += 3; }
  st.push_back(g_clip.size());
  for (size_t i = 0; i + 1 < st.size(); i++) g_aus.push_back({st[i], st[i + 1] - st[i]});
  if (!strncmp(g_scn, "restart_", 8)) {                           // RESTART IN PLACE (camera_ext_c920.cc)
    const char *gs = getenv("EXT_RESTART_GEN");
    int gen = gs ? atoi(gs) : 1;
    if (!ext_c920_enable_restart()) { printf("RESULT enable_failed\n"); return 5; }   // as camerad: every generation sets it up
    if (gen == 1) {
      int p[2];
      if (pipe(p) != 0) return 3;                                  // an inherited descriptor (no close-on-exec), like a USB handle
      char b[32];
      snprintf(b, sizeof b, "%d", p[0]); setenv("EXT_TEST_FD", b, 1);
      snprintf(b, sizeof b, "%d", (int)getpid()); setenv("EXT_FIRST_PID", b, 1);
      setenv("EXT_RESTART_GEN", "2", 1);
      if (!strcmp(g_scn, "restart_hang")) { g_scn = "hang_open"; ext_c920_selftest(1500); }   // hangs in uvc_open: only the watchdog gets out
      if (!strcmp(g_scn, "restart_stopping")) do_exit = 1;
      ext_c920_restart();
      printf("RESULT not_restarted\n"); return 4;
    }
    if (gen == 2 && !strcmp(g_scn, "restart_hang")) {           // a second hang after the restart: the watchdog must work again
      setenv("EXT_RESTART_GEN", "3", 1);
      alarm(2);
      for (;;) pause();
    }
    sigset_t cur;
    pthread_sigmask(SIG_BLOCK, NULL, &cur);
    const char *fs = getenv("EXT_TEST_FD"), *ps = getenv("EXT_FIRST_PID");
    int fd = fs ? atoi(fs) : -1;
    printf("RESULT restarted gen=%d same_pid=%d alarm_blocked=%d inherited_fd_open=%d\n", gen, ps && atoi(ps) == (int)getpid(),
           sigismember(&cur, SIGALRM), fd >= 0 && fcntl(fd, F_GETFD) != -1);
    return 0;
  }
  if (!strcmp(g_scn, "settle")) {                                 // SETTLE: returns once the CPU is calm (or after the limit)
    time_t t0 = time(NULL); ext_c920_wait_settled(8000);
    printf("RESULT settled_in=%ld\n", (long)(time(NULL) - t0)); return 0;
  }
  int ok = ext_c920_selftest(1500);
  printf("RESULT ok=%d started=%s\n", ok, g_started);
  return 0;
}
