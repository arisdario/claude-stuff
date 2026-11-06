#include "camera_ext_c920.h"

#include <dirent.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include <jpeglib.h>
#include <sys/stat.h>
#include "camera_profile.h"
#include "camera_h264.h"      // optional H.264 input: compiled in only when the build defines EXT_WANT_AVCODEC (see SConscript); libavcodec is loaded at run time, never linked
#include "libuvc/libuvc.h"

#ifndef EXT_C920_TEST
#include "common/buffering.h"
#include "common/timing.h"
#include "common/visionbuf.h"
#include "common/visionimg.h"
extern volatile sig_atomic_t do_exit;
#else
// off-EON test build: the openpilot parts are stubbed in the test file
extern volatile sig_atomic_t do_exit;
#endif

#define EXT_VID 0x046d
#define EXT_PID 0x082d
#define SRC_W 1280          // the C920 mode used: MJPEG 1280x720. Its focal length is ~907 px (78 deg diagonal), nearly the EON's 910 px, so the picture is NOT scaled:
#define SRC_H 720           // 58 columns are cropped on each side (1164 wide) and the 720 rows sit in the middle of a black 874-row canvas (the model's 1164x874 frame).
#define SRC_FPS 20          // the EON's rear camera runs at 20 fps
#define OUT_W 1164
#define OUT_H 874
#define CROP_X 58
#define PAD_Y 77
#define SOURCE_FILE "/data/camera_source"
#define FOCUS_FILE "/data/camera_focus"
#define FLIP_FILE "/data/camera_flip"   // 1 = camera mounted upside down: the picture is rotated 180 degrees
#define FORMAT_FILE "/data/camera_format"   // "h264" = ask the C920 for H.264 and decode it here; anything else / no file = MJPEG (the default). Read at every camera (re)start.

// ---------------------------------------------------------------------------------------- pure helpers
void ext_c920_pack_bgr(const uint8_t *rgb, int src_w, int src_h, uint8_t *dst, int dst_stride, int dst_w, int dst_h, int x0, int y0) {
  for (int y = 0; y < src_h && y + y0 < dst_h; y++) {
    const uint8_t *s = rgb + (size_t)y * src_w * 3 + (size_t)x0 * 3;
    uint8_t *d = dst + (size_t)(y + y0) * dst_stride;
    for (int x = 0; x < dst_w; x++) {
      d[3 * x] = s[3 * x + 2];
      d[3 * x + 1] = s[3 * x + 1];
      d[3 * x + 2] = s[3 * x];
    }
  }
}

static int read_int_file(const char *path, int dflt) {
  FILE *f = fopen(path, "r");
  if (!f) return dflt;
  char buf[64] = {0};
  int v = dflt;
  if (fgets(buf, sizeof(buf), f)) v = atoi(buf);
  fclose(f);
  return v;
}

int ext_c920_wanted(void) {
  const char *e = getenv("CAMERA_SOURCE");
  char buf[64] = {0};
  if (e && *e) {
    strncpy(buf, e, sizeof(buf) - 1);
  } else {
    FILE *f = fopen(SOURCE_FILE, "r");
    if (!f) return 0;
    if (!fgets(buf, sizeof(buf), f)) buf[0] = 0;
    fclose(f);
  }
  return strncmp(buf, "external", 8) == 0;
}

static int read_hex_file(const char *path) {
  FILE *f = fopen(path, "r");
  if (!f) return -1;
  unsigned v = 0;
  int ok = fscanf(f, "%x", &v);
  fclose(f);
  return ok == 1 ? (int)v : -1;
}

int ext_c920_present(void) {
  DIR *d = opendir("/sys/bus/usb/devices");
  if (!d) return 0;
  struct dirent *e;
  int found = 0;
  while ((e = readdir(d)) != NULL) {
    if (e->d_name[0] == '.') continue;
    char p[512];
    snprintf(p, sizeof(p), "/sys/bus/usb/devices/%s/idVendor", e->d_name);
    int v = read_hex_file(p);
    snprintf(p, sizeof(p), "/sys/bus/usb/devices/%s/idProduct", e->d_name);
    int pid = read_hex_file(p);
    if (v == EXT_VID && pid == EXT_PID) { found = 1; break; }
  }
  closedir(d);
  return found;
}

// ---------------------------------------------------------------------------------------- the camera
namespace {

struct Ext {
  uvc_context_t *ctx = NULL;
  uvc_device_t *dev = NULL;
  uvc_device_handle_t *devh = NULL;
  volatile int streaming = 0;
};

Ext g;
pthread_t g_thread;
volatile int g_stop = 0, g_started = 0;
volatile unsigned long g_frames = 0;
volatile int g_flip = 0;
volatile float g_luma = -1.f;         // mean luma of the road region of the picture the model gets (profiles / AUTO switching), measured about once a second
volatile int64_t g_stream_start_ns = 0;
volatile int64_t g_last_frame_ns = 0;
CameraState *g_rear = NULL;
VisionBuf *g_bufs = NULL;
uint32_t g_frame_id = 0;
unsigned g_cleared = 0;              // bit per rear buffer that has been blanked once (the black bars above and below the picture are never overwritten)
uint8_t *g_rgb = NULL;               // decode target, SRC_W x SRC_H x 3
volatile int g_h264 = 0;             // the running stream is H.264 (0 = MJPEG, always 0 in a build without H.264 support)
volatile int g_h264_try = 0;         // this ext_open chose H.264 (stays set after a failed start, for the fallback)
volatile int g_force_mjpeg = 0;      // H.264 failed in this camerad run: MJPEG until the next camerad start
bool g_dispatch = false;             // false during the startup self-test: decode and count, but do not touch openpilot's buffers

int64_t now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

struct JErr {
  struct jpeg_error_mgr pub;
  jmp_buf jb;
};
void jerr_exit(j_common_ptr cinfo) { longjmp(((JErr *)cinfo->err)->jb, 1); }

bool decode_jpeg(const uint8_t *data, size_t len, uint8_t *rgb) {
  struct jpeg_decompress_struct ci;
  JErr je;
  ci.err = jpeg_std_error(&je.pub);
  je.pub.error_exit = jerr_exit;
  if (setjmp(je.jb)) { jpeg_destroy_decompress(&ci); return false; }
  jpeg_create_decompress(&ci);
  jpeg_mem_src(&ci, (unsigned char *)data, (unsigned long)len);
  jpeg_read_header(&ci, TRUE);
  ci.out_color_space = JCS_RGB;
  ci.dct_method = JDCT_IFAST;          // speed: the phone's CPU decodes ~20 frames of 1280x720 per second
  ci.do_fancy_upsampling = FALSE;
  jpeg_start_decompress(&ci);
  if (ci.output_width != SRC_W || ci.output_height != SRC_H || ci.output_components != 3) {
    jpeg_destroy_decompress(&ci);
    return false;
  }
  while (ci.output_scanline < ci.output_height) {
    JSAMPROW row = rgb + (size_t)ci.output_scanline * SRC_W * 3;
    jpeg_read_scanlines(&ci, &row, 1);
  }
  jpeg_finish_decompress(&ci);
  jpeg_destroy_decompress(&ci);
  return true;
}

// live view: the camera's own MJPEG frame (full 1280x720, no re-encode) is published for radarweb's /video page, ~10 fps
void publish_live(const uint8_t *data, size_t len) {
  static int64_t last = 0;
  int64_t t = now_ns();
  if (t - last < 90000000LL) return;
  last = t;
  FILE *fp = fopen("/dev/c920_live.tmp", "wb");
  if (!fp) return;
  size_t w = fwrite(data, 1, len, fp);
  fclose(fp);
  if (w == len) rename("/dev/c920_live.tmp", "/dev/c920_live.jpg");
}

void on_frame(uvc_frame_t *f, void *) {
  if (f->frame_format != UVC_FRAME_FORMAT_MJPEG) return;
  publish_live((const uint8_t *)f->data, f->data_bytes);
  if (!g_rgb) return;
  if (!decode_jpeg((const uint8_t *)f->data, f->data_bytes, g_rgb)) return;     // a corrupt frame is skipped
  if (g_flip) {                                                                 // 180 degrees = reverse the pixel order
    uint8_t *a = g_rgb, *z = g_rgb + (size_t)SRC_W * SRC_H * 3 - 3;
    for (; a < z; a += 3, z -= 3) { for (int k = 0; k < 3; k++) { uint8_t t = a[k]; a[k] = z[k]; z[k] = t; } }
  }
  if (g_frames % 20 == 0) {                                                     // ~1 per second at 20 fps: a sparse sample of the lower 55% (the road), after the flip
    long sum = 0, n = 0;
    for (int y = SRC_H * 45 / 100; y < SRC_H * 90 / 100; y += 12)
      for (int x = 0; x < SRC_W; x += 16) { const uint8_t *p = g_rgb + ((size_t)y * SRC_W + x) * 3; sum += (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8; n++; }
    if (n) g_luma = (float)sum / (float)n;
  }
  g_last_frame_ns = now_ns();
  g_frames++;
#ifndef EXT_C920_TEST
  if (!g_dispatch || !g_rear || !g_bufs) return;
  const int idx = tbuffer_select(&g_rear->camera_tb);
  VisionBuf *b = &g_bufs[idx];
  if (!(g_cleared & (1u << idx))) { memset(b->addr, 0, b->len); g_cleared |= (1u << idx); }
  ext_c920_pack_bgr(g_rgb, SRC_W, SRC_H, (uint8_t *)b->addr, g_rear->ci.frame_stride, OUT_W, OUT_H, CROP_X, PAD_Y);
  visionbuf_sync(b, VISIONBUF_SYNC_TO_DEVICE);
  FrameMetadata m;
  memset(&m, 0, sizeof(m));
  m.frame_id = g_frame_id++;
  m.timestamp_eof = (uint64_t)nanos_since_boot();
  g_rear->camera_bufs_metadata[idx] = m;
  tbuffer_dispatch(&g_rear->camera_tb, idx);
#endif
}

#ifdef EXT_HAVE_AVCODEC
// ---- H.264 input (opt-in). Its own callback: on_frame above (MJPEG) is untouched. Everything after the decode is the same as in on_frame
// (tests/test_h264_tail.py compares the two tails text for text, so they cannot drift apart).
h264::Decoder g_dec;
volatile int64_t g_dec_ns_sum = 0;
volatile long g_dec_n = 0;

bool wants_h264() {
  char b[32] = {0};
  const char *e = getenv("CAMERA_FORMAT");
  if (e && *e) strncpy(b, e, sizeof(b) - 1);
  else {
    FILE *f = fopen(FORMAT_FILE, "r");
    if (!f) return false;
    if (!fgets(b, sizeof(b), f)) b[0] = 0;
    fclose(f);
  }
  return strncasecmp(b, "h264", 4) == 0;
}

// the live page normally gets the camera's own MJPEG frame; with H.264 there is none, so one decoded picture (unflipped, as the camera sees it) is JPEG-encoded about twice a second
void publish_live_rgb(const uint8_t *rgb) {
  static int64_t last = 0;
  int64_t t = now_ns();
  if (t - last < 500000000LL) return;
  last = t;
  FILE *fp = fopen("/dev/c920_live.tmp", "wb");
  if (!fp) return;
  struct jpeg_compress_struct co;
  JErr je;
  co.err = jpeg_std_error(&je.pub);
  je.pub.error_exit = jerr_exit;
  if (setjmp(je.jb)) { jpeg_destroy_compress(&co); fclose(fp); return; }
  jpeg_create_compress(&co);
  jpeg_stdio_dest(&co, fp);
  co.image_width = SRC_W; co.image_height = SRC_H; co.input_components = 3; co.in_color_space = JCS_RGB;
  jpeg_set_defaults(&co);
  jpeg_set_quality(&co, 80, TRUE);
  jpeg_start_compress(&co, TRUE);
  while (co.next_scanline < co.image_height) { JSAMPROW row = (JSAMPROW)(rgb + (size_t)co.next_scanline * SRC_W * 3); jpeg_write_scanlines(&co, &row, 1); }
  jpeg_finish_compress(&co);
  jpeg_destroy_compress(&co);
  if (fclose(fp) == 0) rename("/dev/c920_live.tmp", "/dev/c920_live.jpg");
}

void on_frame_h264(uvc_frame_t *f, void *) {
  if (f->frame_format != UVC_FRAME_FORMAT_H264 || !g_rgb) return;
  const int64_t t_dec0 = now_ns();
  if (!g_dec.decode((const uint8_t *)f->data, f->data_bytes, SRC_W, SRC_H, g_rgb)) return;   // undecodable, or waiting for the next keyframe: skipped, never shown to the model
  g_dec_ns_sum += now_ns() - t_dec0; g_dec_n++;
  publish_live_rgb(g_rgb);
  //TAIL-BEGIN
  if (g_flip) {                                                                 // 180 degrees = reverse the pixel order
    uint8_t *a = g_rgb, *z = g_rgb + (size_t)SRC_W * SRC_H * 3 - 3;
    for (; a < z; a += 3, z -= 3) { for (int k = 0; k < 3; k++) { uint8_t t = a[k]; a[k] = z[k]; z[k] = t; } }
  }
  if (g_frames % 20 == 0) {                                                     // ~1 per second at 20 fps: a sparse sample of the lower 55% (the road), after the flip
    long sum = 0, n = 0;
    for (int y = SRC_H * 45 / 100; y < SRC_H * 90 / 100; y += 12)
      for (int x = 0; x < SRC_W; x += 16) { const uint8_t *p = g_rgb + ((size_t)y * SRC_W + x) * 3; sum += (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8; n++; }
    if (n) g_luma = (float)sum / (float)n;
  }
  g_last_frame_ns = now_ns();
  g_frames++;
#ifndef EXT_C920_TEST
  if (!g_dispatch || !g_rear || !g_bufs) return;
  const int idx = tbuffer_select(&g_rear->camera_tb);
  VisionBuf *b = &g_bufs[idx];
  if (!(g_cleared & (1u << idx))) { memset(b->addr, 0, b->len); g_cleared |= (1u << idx); }
  ext_c920_pack_bgr(g_rgb, SRC_W, SRC_H, (uint8_t *)b->addr, g_rear->ci.frame_stride, OUT_W, OUT_H, CROP_X, PAD_Y);
  visionbuf_sync(b, VISIONBUF_SYNC_TO_DEVICE);
  FrameMetadata m;
  memset(&m, 0, sizeof(m));
  m.frame_id = g_frame_id++;
  m.timestamp_eof = (uint64_t)nanos_since_boot();
  g_rear->camera_bufs_metadata[idx] = m;
  tbuffer_dispatch(&g_rear->camera_tb, idx);
#endif
  //TAIL-END
}
#endif  // EXT_HAVE_AVCODEC

void ext_close() {
  if (g.streaming) { uvc_stop_streaming(g.devh); g.streaming = 0; }
#ifdef EXT_HAVE_AVCODEC
  g_dec.close();                                                    // the callback thread has stopped; decoder state never survives a camera restart
#endif
  g_h264 = 0;
  if (g.devh) { uvc_close(g.devh); g.devh = NULL; }
  if (g.dev) { uvc_unref_device(g.dev); g.dev = NULL; }
  if (g.ctx) { uvc_exit(g.ctx); g.ctx = NULL; }
}

// Status LED: the Logitech peripheral XU (GUID ffe52d21-8030-4e2c-82d9-f587d00540bd, USB byte order 212de5ff-...) selector 9, 5 bytes,
// "LED1 Mode" = low 2 bits of byte 1 (0 off, 1 on, 2 blink, 3 auto). Verified on the user's C920 with tesla/usbcam/ledtool: set 0 -> VERIFIED.
// Applied at every start (the camera forgets it at power loss) unless /data/camera_led_auto exists. Same strict checks as ledtool: the control must
// be readable and settable, 5 bytes long; only the 2 mode bits change; the result is read back. Any failure is logged and streaming goes on.
void apply_led_off(uvc_device_handle_t *devh, bool quiet = false) {
  if (access("/data/camera_led_auto", F_OK) == 0) { if (!quiet) fprintf(stderr, "external camera: LED left on auto (/data/camera_led_auto)\n"); return; }
  static const uint8_t GUID[16] = {0xff, 0xe5, 0x2d, 0x21, 0x30, 0x80, 0x2c, 0x4e, 0x82, 0xd9, 0xf5, 0x87, 0xd0, 0x05, 0x40, 0xbd};   // as written in the xml
  uint8_t le[16]; memcpy(le, GUID, 16);                                                                                            // USB descriptor order
  le[0] = GUID[3]; le[1] = GUID[2]; le[2] = GUID[1]; le[3] = GUID[0]; le[4] = GUID[5]; le[5] = GUID[4]; le[6] = GUID[7]; le[7] = GUID[6];
  int unit = -1;
  for (const uvc_extension_unit_t *u = uvc_get_extension_units(devh); u; u = u->next)
    if (!memcmp(u->guidExtensionCode, le, 16) || !memcmp(u->guidExtensionCode, GUID, 16)) { unit = u->bUnitID; break; }
  if (unit < 0) { if (!quiet) fprintf(stderr, "external camera: LED control unit not found, LED unchanged\n"); return; }
  uint8_t info = 0, cur[5] = {0}, nw[5], back[5] = {0};
  if (uvc_get_ctrl(devh, (uint8_t)unit, 9, &info, 1, UVC_GET_INFO) < 0 || (info & 3) != 3 || uvc_get_ctrl_len(devh, (uint8_t)unit, 9) != 5 ||
      uvc_get_ctrl(devh, (uint8_t)unit, 9, cur, 5, UVC_GET_CUR) < 0) { if (!quiet) fprintf(stderr, "external camera: LED control not as expected, LED unchanged\n"); return; }
  if ((cur[1] & 3) == 0) { if (!quiet) fprintf(stderr, "external camera: LED already off\n"); return; }
  memcpy(nw, cur, 5); nw[1] = (uint8_t)(cur[1] & ~3);
  int w = uvc_set_ctrl(devh, (uint8_t)unit, 9, nw, 5), rb = uvc_get_ctrl(devh, (uint8_t)unit, 9, back, 5, UVC_GET_CUR);
  fprintf(stderr, "external camera: LED off %s\n", (w >= 0 && rb >= 0 && (back[1] & 3) == 0 && back[0] == cur[0] && back[2] == cur[2] && back[3] == cur[3] && back[4] == cur[4]) ? "verified" : "NOT verified");
}

bool ext_open() {
  uvc_stream_ctrl_t ctrl;
  if (uvc_init(&g.ctx, NULL) < 0) { g.ctx = NULL; return false; }
  if (uvc_find_device(g.ctx, &g.dev, EXT_VID, EXT_PID, NULL) < 0) { g.dev = NULL; ext_close(); return false; }
  if (uvc_open(g.dev, &g.devh) < 0) { g.devh = NULL; ext_close(); return false; }
  apply_led_off(g.devh);
  uvc_set_ae_priority(g.devh, 0);                                   // frame rate stays at 20 fps in the dark
  uvc_set_focus_auto(g.devh, 0);                                    // no focus hunting
  uvc_set_focus_abs(g.devh, (uint16_t)read_int_file(FOCUS_FILE, 0)); // 0 = infinity (the road)
  uvc_frame_callback_t *frame_cb = on_frame;
  g_h264_try = 0;
#ifdef EXT_HAVE_AVCODEC
  bool use264 = false;
  if (wants_h264() && !g_force_mjpeg) {                              // any problem below leaves use264 false: the MJPEG lines that follow run exactly as before
    char why[256] = "";
    if (!g_dec.open(why, sizeof why)) fprintf(stderr, "external camera: H.264 not available (%s): using MJPEG\n", why);
    else if (uvc_get_stream_ctrl_format_size(g.devh, &ctrl, UVC_FRAME_FORMAT_H264, SRC_W, SRC_H, SRC_FPS) < 0) { fprintf(stderr, "external camera: the camera offers no H.264 %dx%d @ %d fps: using MJPEG\n", SRC_W, SRC_H, SRC_FPS); g_dec.close(); }
    else { use264 = true; frame_cb = on_frame_h264; }
  }
  if (!use264)
#else
  if (access(FORMAT_FILE, F_OK) == 0) fprintf(stderr, "external camera: %s exists, but this camerad was built without H.264 support: MJPEG\n", FORMAT_FILE);
#endif
  if (uvc_get_stream_ctrl_format_size(g.devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, SRC_W, SRC_H, SRC_FPS) < 0) { ext_close(); return false; }
  if (!g_rgb) g_rgb = (uint8_t *)malloc((size_t)SRC_W * SRC_H * 3);
  g_last_frame_ns = now_ns();
#ifdef EXT_HAVE_AVCODEC
  g_h264 = use264 ? 1 : 0; g_h264_try = use264 ? 1 : 0; g_dec_ns_sum = 0; g_dec_n = 0;
#endif
  if (uvc_start_streaming(g.devh, &ctrl, frame_cb, NULL, 0) < 0) {
#ifdef EXT_HAVE_AVCODEC
    if (use264) { g_force_mjpeg = 1; fprintf(stderr, "external camera: the H.264 stream would not start: MJPEG from now on\n"); }
#endif
    ext_close(); return false;
  }
  g.streaming = 1;
  g_stream_start_ns = now_ns();
  apply_led_off(g.devh);                                            // again once streaming: the firmware may switch the LED with the stream
  return true;
}

// ---- DAY / NIGHT profiles (logic and unit tests: camera_profile.h, tests/test_camera_profile.cc). Off unless /data/camera_profile says day/night/auto.
struct UvcIface : camprof::Iface {
  uvc_device_handle_t *h;
  explicit UvcIface(uvc_device_handle_t *d) : h(d) {}
  bool get(int c, int *v) override {
    uint8_t b8 = 0; uint16_t b16 = 0; int16_t s16 = 0; uint32_t b32 = 0;
    switch (c) {
      case camprof::AE_MODE: if (uvc_get_ae_mode(h, &b8, UVC_GET_CUR) < 0) return false; *v = b8; return true;
      case camprof::EXPOSURE: if (uvc_get_exposure_abs(h, &b32, UVC_GET_CUR) < 0) return false; *v = (int)b32; return true;
      case camprof::GAIN: if (uvc_get_gain(h, &b16, UVC_GET_CUR) < 0) return false; *v = b16; return true;
      case camprof::BRIGHTNESS: if (uvc_get_brightness(h, &s16, UVC_GET_CUR) < 0) return false; *v = s16; return true;
      case camprof::CONTRAST: if (uvc_get_contrast(h, &b16, UVC_GET_CUR) < 0) return false; *v = b16; return true;
      case camprof::SATURATION: if (uvc_get_saturation(h, &b16, UVC_GET_CUR) < 0) return false; *v = b16; return true;
      case camprof::SHARPNESS: if (uvc_get_sharpness(h, &b16, UVC_GET_CUR) < 0) return false; *v = b16; return true;
      case camprof::BACKLIGHT: if (uvc_get_backlight_compensation(h, &b16, UVC_GET_CUR) < 0) return false; *v = b16; return true;
      case camprof::WB_AUTO: if (uvc_get_white_balance_temperature_auto(h, &b8, UVC_GET_CUR) < 0) return false; *v = b8; return true;
      case camprof::WB_KELVIN: if (uvc_get_white_balance_temperature(h, &b16, UVC_GET_CUR) < 0) return false; *v = b16; return true;
      case camprof::POWER_LINE: if (uvc_get_power_line_frequency(h, &b8, UVC_GET_CUR) < 0) return false; *v = b8; return true;
    }
    return false;
  }
  bool set(int c, int v) override {
    switch (c) {
      case camprof::AE_MODE: return uvc_set_ae_mode(h, (uint8_t)v) >= 0;
      case camprof::EXPOSURE: return uvc_set_exposure_abs(h, (uint32_t)v) >= 0;
      case camprof::GAIN: return uvc_set_gain(h, (uint16_t)v) >= 0;
      case camprof::BRIGHTNESS: return uvc_set_brightness(h, (int16_t)v) >= 0;
      case camprof::CONTRAST: return uvc_set_contrast(h, (uint16_t)v) >= 0;
      case camprof::SATURATION: return uvc_set_saturation(h, (uint16_t)v) >= 0;
      case camprof::SHARPNESS: return uvc_set_sharpness(h, (uint16_t)v) >= 0;
      case camprof::BACKLIGHT: return uvc_set_backlight_compensation(h, (uint16_t)v) >= 0;
      case camprof::WB_AUTO: return uvc_set_white_balance_temperature_auto(h, (uint8_t)v) >= 0;
      case camprof::WB_KELVIN: return uvc_set_white_balance_temperature(h, (uint16_t)v) >= 0;
      case camprof::POWER_LINE: return uvc_set_power_line_frequency(h, (uint8_t)v) >= 0;
    }
    return false;
  }
};

bool read_text(const char *path, char *buf, size_t n) {
  FILE *f = fopen(path, "r");
  if (!f) { buf[0] = 0; return false; }
  size_t k = fread(buf, 1, n - 1, f);
  buf[k] = 0;
  fclose(f);
  return true;
}

camprof::Controller g_prof;
camprof::Conf g_conf;
time_t g_conf_mtime = 0, g_mode_mtime = 0;
int g_prof_tick = 0;

void profile_tick(uvc_device_handle_t *devh, double now_s) {
  char mb[32];
  struct stat st1, st2;
  bool changed = false, do_factory = false;
  char cmd[32];
  if (access("/data/camera_cmd", F_OK) == 0) {                                // one-shot commands from the radarweb page
    read_text("/data/camera_cmd", cmd, sizeof cmd);
    unlink("/data/camera_cmd");
    if (!strncmp(cmd, "factory", 7)) do_factory = true;
  }
  if (stat("/data/camera_profile.conf", &st1) == 0 && st1.st_mtime != g_conf_mtime) {
    char buf[8192];
    read_text("/data/camera_profile.conf", buf, sizeof buf);
    std::string warn;
    g_conf = camprof::parse_conf(buf, &warn);
    g_conf_mtime = st1.st_mtime; changed = true;
    if (!warn.empty()) fprintf(stderr, "external camera: profile file: %s", warn.c_str());
  } else if (stat("/data/camera_profile.conf", &st1) != 0 && g_conf_mtime != 0) {   // file removed (restore defaults): back to empty profiles
    g_conf = camprof::Conf(); g_conf_mtime = 0; changed = true;
  }
  if (stat("/data/camera_profile", &st2) == 0 && st2.st_mtime != g_mode_mtime) { g_mode_mtime = st2.st_mtime; changed = true; }
  read_text("/data/camera_profile", mb, sizeof mb);
  camprof::Mode want = camprof::parse_mode(mb);
  bool idle = want == camprof::M_OFF && g_prof.mode == camprof::M_OFF && !do_factory;
  ++g_prof_tick;
  if (idle && g_prof_tick % 5) return;                                           // default: nothing is written; the status (read-only) is refreshed every 5 s
  UvcIface cam(devh);
  camprof::Measure m;
  m.luma = g_luma;
  cam.get(camprof::EXPOSURE, &m.exposure);
  cam.get(camprof::GAIN, &m.gain);
  m.frames_ok = (now_ns() - g_last_frame_ns) < 1000000000LL;
  m.stream_age_s = (now_ns() - g_stream_start_ns) / 1e9;
  g_prof.log.clear();
  if (!idle) g_prof.tick(now_s, want, changed, g_conf, m, cam);
  if (do_factory && m.frames_ok) g_prof.factory(now_s, cam);
  for (size_t i = 0; i < g_prof.log.size(); i++) {
    fprintf(stderr, "external camera: %s\n", g_prof.log[i].c_str());
    FILE *lf = fopen("/data/camera_profile.log", "a");
    if (lf) { fprintf(lf, "%ld %s\n", (long)time(NULL), g_prof.log[i].c_str()); fclose(lf); }
  }
  if (!idle || g_prof_tick % 5 == 0) {                                           // status for the radarweb page (read-only requests)
    int v[camprof::N_CTL];
    for (int i = 0; i < camprof::N_CTL; i++) if (!cam.get(i, &v[i])) v[i] = -1;
    uint8_t aep = 255; uvc_get_ae_priority(devh, &aep, UVC_GET_CUR);
    char js[1024];
    snprintf(js, sizeof js, "{\"mode\":\"%s\",\"active\":\"%s\",\"luma\":%.1f,\"ae_mode\":%d,\"exposure\":%d,\"gain\":%d,\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,\"sharpness\":%d,"
             "\"backlight\":%d,\"wb_auto\":%d,\"wb_kelvin\":%d,\"power_line\":%d,\"ae_priority\":%d,\"format\":\"%s\",\"apply_count\":%d,\"apply_ok\":%d,\"reason\":\"%s\",\"error\":\"%s\",\"t\":%ld}\n",
             camprof::mode_name(g_prof.mode), g_prof.active < 0 ? "none" : (g_prof.active ? "night" : "day"), m.luma, v[camprof::AE_MODE], v[camprof::EXPOSURE], v[camprof::GAIN],
             v[camprof::BRIGHTNESS], v[camprof::CONTRAST], v[camprof::SATURATION], v[camprof::SHARPNESS], v[camprof::BACKLIGHT], v[camprof::WB_AUTO], v[camprof::WB_KELVIN],
             v[camprof::POWER_LINE], aep == 255 ? -1 : (int)aep, g_h264 ? "h264" : "mjpeg", g_prof.apply_count, g_prof.apply_ok ? 1 : 0, g_prof.last_reason.c_str(), g_prof.last_error.c_str(), (long)time(NULL));
    FILE *f = fopen("/data/camera_status.json.tmp", "w");
    if (f) { fputs(js, f); fclose(f); rename("/data/camera_status.json.tmp", "/data/camera_status.json"); }
  }
}

void *capture_thread(void *) {
  g_dispatch = true;
  while (!g_stop && !do_exit) {
    if (!ext_open()) { fprintf(stderr, "external camera: not available, retrying\n"); sleep(1); continue; }
    g_flip = read_int_file(FLIP_FILE, 0) ? 1 : 0;
    g_prof.reopened();
    fprintf(stderr, "ROAD CAMERA: external C920 streaming %dx%d %s @ %d fps\n", SRC_W, SRC_H, g_h264 ? "H.264 (decoded here)" : "MJPEG", SRC_FPS);
    int last_focus = read_int_file(FOCUS_FILE, 0), tick = 0;
    const unsigned long frames_at_open = g_frames;
    while (!g_stop && !do_exit && (now_ns() - g_last_frame_ns) < 3000000000LL) {
      usleep(100000);
#ifdef EXT_HAVE_AVCODEC
      if (g_h264 && tick % 300 == 299 && g_dec_n > 0) {                                  // every 30 s while H.264 runs: what decoding costs and how the decoder is doing
        fprintf(stderr, "external camera: H.264 decode %.1f ms per frame on average (%ld frames), decoder errors %ld, frames dropped waiting for a keyframe %ld\n", g_dec_ns_sum / 1e6 / (double)g_dec_n, g_dec_n, g_dec.errors(), g_dec.dropped());
        g_dec_ns_sum = 0; g_dec_n = 0;
      }
#endif
      if (++tick % 10 == 0) {
        if (g.devh) profile_tick(g.devh, now_ns() / 1e9);
        if (g.devh) {                                                                   // LED: on request from the radarweb camera page (/data/camera_led_now), and re-checked every 5 s
          if (access("/data/camera_led_now", F_OK) == 0) { unlink("/data/camera_led_now"); apply_led_off(g.devh); }
          else if (tick % 50 == 0) apply_led_off(g.devh, true);                         // quiet: logs only if it had to switch the LED off again
        }
        g_flip = read_int_file(FLIP_FILE, 0) ? 1 : 0;                                   // once a second: a focus change made in the radarweb camera page (writes /data/camera_focus) is applied live
        int f = read_int_file(FOCUS_FILE, 0);
        if (f < 0) f = 0;
        if (f > 250) f = 250;
        if (f != last_focus && g.devh) {
          uvc_set_focus_abs(g.devh, (uint16_t)f);
          last_focus = f;
          fprintf(stderr, "external camera: focus -> %d\n", f);
        }
      }
    }
    if (!g_stop && !do_exit) fprintf(stderr, "external camera: no frames for 3 s, restarting the camera\n");
#ifdef EXT_HAVE_AVCODEC
    if (g_h264_try && g_frames == frames_at_open && !g_force_mjpeg) {            // H.264 never produced one picture: do not loop on it
      g_force_mjpeg = 1;
      fprintf(stderr, "external camera: H.264 gave no picture at all: MJPEG from now on\n");
    }
#endif
    ext_close();
  }
  ext_close();
  return NULL;
}

}  // namespace

static int selftest_once(int timeout_ms) {
  g_dispatch = false;
  g_frames = 0;
  if (!ext_open()) return 0;
  int waited = 0;
  while (g_frames < 3 && waited < timeout_ms) { usleep(50000); waited += 50; }
  int ok = g_frames >= 3;
  ext_close();
  return ok;
}

int ext_c920_selftest(int timeout_ms) {
  fprintf(stderr, "external camera: self-test (opening the C920 ...)\n");
  int ok = selftest_once(timeout_ms);
#ifdef EXT_HAVE_AVCODEC
  if (!ok && g_h264_try) {                                          // H.264 was chosen and failed (no stream, or no picture): the camera itself may be fine, so MJPEG gets one try before the C920 is given up on
    g_force_mjpeg = 1;
    fprintf(stderr, "external camera: H.264 self-test failed: trying MJPEG\n");
    ok = selftest_once(timeout_ms);
  }
#endif
  fprintf(stderr, "external camera: self-test %s (%lu frames)\n", ok ? "PASSED" : "FAILED", (unsigned long)g_frames);
  return ok;
}

// Be patient at startup: after a cold boot the C920 can show up on USB, and deliver its first frames, later than camerad starts.
// Wait up to present_wait_ms for the camera to appear, then try the self-test up to `attempts` times. Called only when "external" is selected.
int ext_c920_acquire(int present_wait_ms, int attempts, int timeout_ms) {
  int waited = 0;
  while (!ext_c920_present() && waited < present_wait_ms) {
    if (do_exit) return 0;
    usleep(500000);
    waited += 500;
  }
  if (!ext_c920_present()) {
    fprintf(stderr, "external camera: C920 not found on USB after %d ms, using the phone camera\n", waited);
    return 0;
  }
  if (waited > 0) fprintf(stderr, "external camera: C920 appeared after %d ms\n", waited);
  for (int i = 0; i < attempts; i++) {
    if (do_exit) return 0;
    if (ext_c920_selftest(timeout_ms)) return 1;
    fprintf(stderr, "external camera: attempt %d of %d failed%s\n", i + 1, attempts, i + 1 < attempts ? ", retrying" : ", using the phone camera");
    if (i + 1 < attempts) sleep(1);
  }
  return 0;
}

void ext_c920_configure(CameraState *rear) {
  int aw = OUT_W, ah = OUT_H;
#ifndef EXT_C920_TEST
  visionimg_compute_aligned_width_and_height(OUT_W, OUT_H, &aw, &ah);     // the same rule main.cc uses for the rgb buffer
#endif
  rear->ci.name = "external C920";
  rear->ci.frame_width = OUT_W;
  rear->ci.frame_height = OUT_H;
  rear->ci.frame_stride = aw * 3;
  rear->ci.bayer = false;
  rear->ci.bayer_flip = 0;
  rear->ci.hdr = false;
  rear->frame_size = aw * ah * 3;
  rear->fps = SRC_FPS;
}

int ext_c920_start(CameraState *rear, VisionBuf *bufs) {
  g_rear = rear;
  g_bufs = bufs;
  g_stop = 0;
  if (pthread_create(&g_thread, NULL, capture_thread, NULL) != 0) return -1;
  g_started = 1;
  return 0;
}

void ext_c920_stop(void) {
  g_stop = 1;
  if (g_started) { pthread_join(g_thread, NULL); g_started = 0; }
}
