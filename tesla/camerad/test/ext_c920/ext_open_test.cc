// End-to-end test of the format choice in camera_ext_c920.cc against a FAKE camera (the uvc_* calls are replaced with --wrap, everything else is the real file).
//   ext_open_test SCENARIO CLIP.h264 FRAME.jpg      prints "RESULT ok=<selftest> started=<mjpeg|h264|none>"
// scenarios: normal (camera offers both) | no_h264 (camera has no H.264 mode) | start_fails (H.264 mode negotiates but will not start) | garbage (H.264 frames are junk)
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
uvc_error_t __wrap_uvc_open(uvc_device_t *, uvc_device_handle_t **h) { *h = (uvc_device_handle_t *)1; return UVC_SUCCESS; }
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
  int ok = ext_c920_selftest(1500);
  printf("RESULT ok=%d started=%s\n", ok, g_started);
  return 0;
}
