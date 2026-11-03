#include "camera_ext_c920.h"

#include <dirent.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <jpeglib.h>
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
volatile int64_t g_last_frame_ns = 0;
CameraState *g_rear = NULL;
VisionBuf *g_bufs = NULL;
uint32_t g_frame_id = 0;
unsigned g_cleared = 0;              // bit per rear buffer that has been blanked once (the black bars above and below the picture are never overwritten)
uint8_t *g_rgb = NULL;               // decode target, SRC_W x SRC_H x 3
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

void ext_close() {
  if (g.streaming) { uvc_stop_streaming(g.devh); g.streaming = 0; }
  if (g.devh) { uvc_close(g.devh); g.devh = NULL; }
  if (g.dev) { uvc_unref_device(g.dev); g.dev = NULL; }
  if (g.ctx) { uvc_exit(g.ctx); g.ctx = NULL; }
}

// Status LED: the Logitech peripheral XU (GUID ffe52d21-8030-4e2c-82d9-f587d00540bd, USB byte order 212de5ff-...) selector 9, 5 bytes,
// "LED1 Mode" = low 2 bits of byte 1 (0 off, 1 on, 2 blink, 3 auto). Verified on the user's C920 with tesla/usbcam/ledtool: set 0 -> VERIFIED.
// Applied at every start (the camera forgets it at power loss) unless /data/camera_led_auto exists. Same strict checks as ledtool: the control must
// be readable and settable, 5 bytes long; only the 2 mode bits change; the result is read back. Any failure is logged and streaming goes on.
void apply_led_off(uvc_device_handle_t *devh) {
  if (access("/data/camera_led_auto", F_OK) == 0) { fprintf(stderr, "external camera: LED left on auto (/data/camera_led_auto)\n"); return; }
  static const uint8_t GUID[16] = {0xff, 0xe5, 0x2d, 0x21, 0x30, 0x80, 0x2c, 0x4e, 0x82, 0xd9, 0xf5, 0x87, 0xd0, 0x05, 0x40, 0xbd};   // as written in the xml
  uint8_t le[16]; memcpy(le, GUID, 16);                                                                                            // USB descriptor order
  le[0] = GUID[3]; le[1] = GUID[2]; le[2] = GUID[1]; le[3] = GUID[0]; le[4] = GUID[5]; le[5] = GUID[4]; le[6] = GUID[7]; le[7] = GUID[6];
  int unit = -1;
  for (const uvc_extension_unit_t *u = uvc_get_extension_units(devh); u; u = u->next)
    if (!memcmp(u->guidExtensionCode, le, 16) || !memcmp(u->guidExtensionCode, GUID, 16)) { unit = u->bUnitID; break; }
  if (unit < 0) { fprintf(stderr, "external camera: LED control unit not found, LED unchanged\n"); return; }
  uint8_t info = 0, cur[5] = {0}, nw[5], back[5] = {0};
  if (uvc_get_ctrl(devh, (uint8_t)unit, 9, &info, 1, UVC_GET_INFO) < 0 || (info & 3) != 3 || uvc_get_ctrl_len(devh, (uint8_t)unit, 9) != 5 ||
      uvc_get_ctrl(devh, (uint8_t)unit, 9, cur, 5, UVC_GET_CUR) < 0) { fprintf(stderr, "external camera: LED control not as expected, LED unchanged\n"); return; }
  if ((cur[1] & 3) == 0) { fprintf(stderr, "external camera: LED already off\n"); return; }
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
  if (uvc_get_stream_ctrl_format_size(g.devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, SRC_W, SRC_H, SRC_FPS) < 0) { ext_close(); return false; }
  if (!g_rgb) g_rgb = (uint8_t *)malloc((size_t)SRC_W * SRC_H * 3);
  g_last_frame_ns = now_ns();
  if (uvc_start_streaming(g.devh, &ctrl, on_frame, NULL, 0) < 0) { ext_close(); return false; }
  g.streaming = 1;
  return true;
}

void *capture_thread(void *) {
  g_dispatch = true;
  while (!g_stop && !do_exit) {
    if (!ext_open()) { fprintf(stderr, "external camera: not available, retrying\n"); sleep(1); continue; }
    g_flip = read_int_file(FLIP_FILE, 0) ? 1 : 0;
    fprintf(stderr, "ROAD CAMERA: external C920 streaming %dx%d MJPEG @ %d fps\n", SRC_W, SRC_H, SRC_FPS);
    int last_focus = read_int_file(FOCUS_FILE, 0), tick = 0;
    while (!g_stop && !do_exit && (now_ns() - g_last_frame_ns) < 3000000000LL) {
      usleep(100000);
      if (++tick % 10 == 0) {
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
    ext_close();
  }
  ext_close();
  return NULL;
}

}  // namespace

int ext_c920_selftest(int timeout_ms) {
  fprintf(stderr, "external camera: self-test (opening the C920 ...)\n");
  g_dispatch = false;
  g_frames = 0;
  if (!ext_open()) return 0;
  int waited = 0;
  while (g_frames < 3 && waited < timeout_ms) { usleep(50000); waited += 50; }
  int ok = g_frames >= 3;
  ext_close();
  fprintf(stderr, "external camera: self-test %s (%lu frames)\n", ok ? "PASSED" : "FAILED", (unsigned long)g_frames);
  return ok;
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
