// uvctest - standalone USB webcam test for the EON. Talks to the camera through libusb + libuvc (the EON's kernel has
// no webcam driver; this is the same raw-USB path boardd uses for the panda). Touches nothing in openpilot.
//
// Opens a UVC camera, lists its modes, reports whether it has the controls a road camera needs, streams for a few
// seconds, reports the frame rate actually received and saves one frame (frame.jpg for MJPEG, frame.ppm otherwise).
//
//   ./uvctest                        first UVC camera, 640x480 @ 30 fps, 10 s
//   ./uvctest 0c45:6422              a given camera (vendor:product from the USB list)
//   ./uvctest 0c45:6422 640 480 30   a given mode (pick one from the list it prints)
//   ./uvctest 0c45:6422 640 480 30 20   ... for 20 seconds
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include "libuvc/libuvc.h"
#include <libusb.h>

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static int frames = 0, short_frames = 0;
static long bytes = 0;
static double t_first = 0, t_last = 0;
static uvc_frame_t *keep = NULL;          // a copy of one frame, saved after streaming
static size_t expect_bytes = 0;           // for uncompressed modes: the size a complete frame must have

static double now_s(void) {
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// libuvc calls this on its own thread for every frame received
static void on_frame(uvc_frame_t *f, void *user) {
  (void)user;
  double t = now_s();
  pthread_mutex_lock(&lock);
  if (frames == 0) t_first = t;
  t_last = t; frames++; bytes += (long)f->data_bytes;
  if (expect_bytes && f->data_bytes != expect_bytes) short_frames++;
  if (frames == 10 && keep == NULL) {     // skip the first frames (exposure still settling)
    keep = uvc_allocate_frame(0);
    if (keep && uvc_duplicate_frame(f, keep) != UVC_SUCCESS) { uvc_free_frame(keep); keep = NULL; }
  }
  pthread_mutex_unlock(&lock);
}

static void hexdump(const unsigned char *b, int n) {
  printf("      ");
  for (int i = 0; i < n; i++) printf("%02x%s", b[i], (i % 16 == 15 && i + 1 < n) ? "\n      " : " ");
  printf("\n");
}

// Raw USB requests, each with libusb's exact error name: tells a cabling/power problem (everything fails) from a
// claiming problem (only interface requests fail) from a camera that rejects UVC requests (only those fail).
static void usb_diag(uvc_device_handle_t *devh) {
  libusb_device_handle *h = uvc_get_libusb_handle(devh);
  const struct libusb_version *v = libusb_get_version();
  unsigned char b[64];
  int r;
  printf("\n=== raw USB requests (diagnosis), libusb %d.%d.%d ===\n", v->major, v->minor, v->micro);
  r = libusb_control_transfer(h, 0x80, 0x06, 0x0100, 0, b, 18, 1000);          // GET_DESCRIPTOR(device), live
  printf("standard GET_DESCRIPTOR(device)  : %s\n", r >= 0 ? "OK" : libusb_error_name(r));
  r = libusb_control_transfer(h, 0x80, 0x00, 0, 0, b, 2, 1000);                // GET_STATUS(device)
  printf("standard GET_STATUS(device)      : %s\n", r >= 0 ? "OK" : libusb_error_name(r));
  for (int i = 0; i < 2; i++) {
    int k = libusb_kernel_driver_active(h, i);
    printf("interface %d kernel driver        : %s\n", i, k == 1 ? "YES (attached)" : k == 0 ? "none" : libusb_error_name(k));
  }
  r = libusb_claim_interface(h, 1);
  printf("claim video-streaming interface 1: %s\n", r == 0 ? "OK" : libusb_error_name(r));
  memset(b, 0, sizeof(b));
  r = libusb_control_transfer(h, 0xA1, 0x81, 0x01 << 8, 1, b, 26, 1000);         // UVC GET_CUR, VS_PROBE_CONTROL
  printf("UVC GET_CUR  probe (interface 1) : %s\n", r >= 0 ? "OK" : libusb_error_name(r));
  if (r > 0) hexdump(b, r);
  memset(b, 0, sizeof(b));
  r = libusb_control_transfer(h, 0xA1, 0x87, 0x01 << 8, 1, b, 26, 1000);         // UVC GET_DEF, VS_PROBE_CONTROL
  printf("UVC GET_DEF  probe (interface 1) : %s\n", r >= 0 ? "OK" : libusb_error_name(r));
  if (r > 0) hexdump(b, r);
  // the negotiation libuvc does before streaming starts: SET_CUR on the probe control (a WRITE request), here for YUY2 640x480 @ 30 fps (format 1, frame 1, interval 333333 x 100 ns)
  {
    unsigned char sb[26], gb[26];
    memset(sb, 0, sizeof(sb));
    sb[0] = 1; sb[1] = 0;                  // bmHint = dwFrameInterval is fixed
    sb[2] = 1; sb[3] = 1;                  // bFormatIndex 1 (YUY2), bFrameIndex 1 (640x480)
    sb[4] = 333333 & 0xff; sb[5] = (333333 >> 8) & 0xff; sb[6] = (333333 >> 16) & 0xff; sb[7] = 0;
    r = libusb_control_transfer(h, 0x21, 0x01, 0x01 << 8, 1, sb, 26, 1000);       // UVC SET_CUR, VS_PROBE_CONTROL
    printf("UVC SET_CUR  probe (WRITE)       : %s\n", r >= 0 ? "OK" : libusb_error_name(r));
    memset(gb, 0, sizeof(gb));
    r = libusb_control_transfer(h, 0xA1, 0x81, 0x01 << 8, 1, gb, 26, 1000);       // read back
    printf("UVC GET_CUR  probe after SET_CUR : %s\n", r >= 0 ? "OK" : libusb_error_name(r));
    if (r > 0) { hexdump(gb, r); printf("      camera chose: format %d frame %d interval %u max payload %u bytes\n", gb[2], gb[3], gb[4] | (gb[5] << 8) | (gb[6] << 16), gb[22] | (gb[23] << 8) | (gb[24] << 16)); }
  }
  r = libusb_control_transfer(h, 0xA1, 0x81, 0x02 << 8, 0, b, 1, 1000);          // UVC GET_CUR, VC_REQUEST_ERROR_CODE
  printf("UVC GET_CUR  error code (if 0)   : %s", r >= 0 ? "OK" : libusb_error_name(r));
  if (r > 0) printf("  (last request error code %d)", b[0]);
  printf("\n");
}

static void save_frame(uvc_frame_t *f) {
  if (f->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    FILE *o = fopen("frame.jpg", "wb");
    if (o) { fwrite(f->data, 1, f->data_bytes, o); fclose(o); printf("saved frame.jpg (%zu bytes)\n", f->data_bytes); }
    return;
  }
  uvc_frame_t *rgb = uvc_allocate_frame(f->width * f->height * 3);
  if (rgb && uvc_any2rgb(f, rgb) == UVC_SUCCESS) {
    FILE *o = fopen("frame.ppm", "wb");
    if (o) {
      fprintf(o, "P6\n%u %u\n255\n", f->width, f->height); fwrite(rgb->data, 1, (size_t)f->width * f->height * 3, o); fclose(o);
      printf("saved frame.ppm (%ux%u)\n", f->width, f->height);
    }
  } else {
    printf("could not convert the frame to RGB to save it (format %d)\n", f->frame_format);
  }
  if (rgb) uvc_free_frame(rgb);
}

// libuvc's own mode matching (uvc_get_stream_ctrl_format_size) answered "invalid mode" on the EON although the descriptors it printed list the mode. Show what it sees, and pick the mode ourselves
// from the same parsed descriptors (format GUID + width + height + frame interval), then let libuvc negotiate it with the camera (uvc_probe_stream_ctrl).
static void show_candidates(uvc_device_handle_t *devh, int w, int h) {
  const uvc_format_desc_t *f;
  fprintf(stderr, "\nwhat libuvc parsed for %dx%d:\n", w, h);
  for (f = uvc_get_format_descs(devh); f; f = f->next) {
    const uvc_frame_desc_t *fr;
    for (fr = f->frame_descs; fr; fr = fr->next) {
      if (fr->wWidth != w || fr->wHeight != h) continue;
      fprintf(stderr, "  format #%d guid %02x%02x%02x%02x.. frame #%d intervals(x100ns):", f->bFormatIndex, f->guidFormat[0], f->guidFormat[1], f->guidFormat[2], f->guidFormat[3], fr->bFrameIndex);
      if (fr->intervals) { const uint32_t *iv; for (iv = fr->intervals; *iv; iv++) fprintf(stderr, " %u", *iv); }
      else fprintf(stderr, " (continuous %u..%u step %u)", fr->dwMinFrameInterval, fr->dwMaxFrameInterval, fr->dwFrameIntervalStep);
      fprintf(stderr, "\n");
    }
  }
}

// returns 0 = YUYV, 1 = MJPEG (chosen), -1 = none; fills ctrl and negotiates
static int pick_mode_manually(uvc_device_handle_t *devh, uvc_stream_ctrl_t *ctrl, int w, int h, int fps, uvc_error_t *neg) {
  const uvc_format_desc_t *f;
  for (int want = 1; want >= 0; want--) {                         // MJPEG first (smaller on the bus), then YUYV
    for (f = uvc_get_format_descs(devh); f; f = f->next) {
      int is_mjpeg = !memcmp(f->guidFormat, "MJPG", 4), is_yuyv = !memcmp(f->guidFormat, "YUY2", 4);
      if ((want == 1 && !is_mjpeg) || (want == 0 && !is_yuyv)) continue;
      for (const uvc_frame_desc_t *fr = f->frame_descs; fr; fr = fr->next) {
        if (fr->wWidth != w || fr->wHeight != h || !fr->intervals) continue;
        for (const uint32_t *iv = fr->intervals; *iv; iv++) {
          if (10000000u / *iv != (unsigned)fps) continue;
          memset(ctrl, 0, sizeof(*ctrl));
          ctrl->bmHint = 1;
          ctrl->bFormatIndex = f->bFormatIndex;
          ctrl->bFrameIndex = fr->bFrameIndex;
          ctrl->dwFrameInterval = *iv;
          ctrl->bInterfaceNumber = 1;
          *neg = uvc_probe_stream_ctrl(devh, ctrl);
          if (*neg == UVC_SUCCESS) return want;
          fprintf(stderr, "  negotiating %s %dx%d @ %d fps: %s (%d)\n", want ? "MJPEG" : "YUYV", w, h, fps, uvc_strerror(*neg), (int)*neg);
        }
      }
    }
  }
  return -1;
}

int main(int argc, char **argv) {
  int vid = 0, pid = 0, w = 640, h = 480, fps = 30, secs = 10;
  int a = 1;
  if (argc > a && strchr(argv[a], ':')) { sscanf(argv[a], "%x:%x", &vid, &pid); a++; }
  if (argc > a + 2) { w = atoi(argv[a]); h = atoi(argv[a + 1]); fps = atoi(argv[a + 2]); a += 3; }
  if (argc > a) secs = atoi(argv[a]);
  if (secs < 1) secs = 1;

  uvc_context_t *ctx; uvc_device_t *dev; uvc_device_handle_t *devh; uvc_stream_ctrl_t ctrl; uvc_error_t r;
  if ((r = uvc_init(&ctx, NULL)) < 0) { uvc_perror(r, "uvc_init (libusb could not start)"); return 1; }
  if ((r = uvc_find_device(ctx, &dev, vid, pid, NULL)) < 0) {
    uvc_perror(r, "no UVC camera found - is it in the USB list? (for d in /sys/bus/usb/devices/*; ...)");
    uvc_exit(ctx); return 2;
  }
  if ((r = uvc_open(dev, &devh)) < 0) {
    uvc_perror(r, "uvc_open - run as root; is another program using the camera?");
    uvc_unref_device(dev); uvc_exit(ctx); return 3;
  }

  printf("=== camera and its modes ===\n");
  uvc_print_diag(devh, stdout);
  usb_diag(devh);

  // the two controls a road camera needs: steady frame rate at night, and no focus hunting
  printf("\n=== road-camera controls ===\n");
  r = uvc_set_ae_priority(devh, 0);
  printf("exposure priority off (keeps the frame rate in the dark): %s\n", r == UVC_SUCCESS ? "OK" : uvc_strerror(r));
  r = uvc_set_focus_auto(devh, 0);
  printf("autofocus off: %s\n", r == UVC_SUCCESS ? "OK" : uvc_strerror(r));

  enum uvc_frame_format fmts[] = {UVC_FRAME_FORMAT_MJPEG, UVC_FRAME_FORMAT_YUYV};
  const char *names[] = {"MJPEG", "YUYV"};
  int k;
  memset(&ctrl, 0, sizeof(ctrl));
  uvc_error_t rr[2] = {UVC_ERROR_INVALID_MODE, UVC_ERROR_INVALID_MODE};
  for (k = 0; k < 2; k++) {
    rr[k] = uvc_get_stream_ctrl_format_size(devh, &ctrl, fmts[k], w, h, fps);
    if (rr[k] == UVC_SUCCESS) break;
  }
  if (k == 2) {
    // INVALID_MODE = the camera does not list that mode; anything else = the mode IS listed but the negotiation with the camera failed
    for (int j = 0; j < 2; j++)
      fprintf(stderr, "\n%s %dx%d @ %d fps: %s (%d)%s", names[j], w, h, fps, uvc_strerror(rr[j]), (int)rr[j],
              rr[j] == UVC_ERROR_INVALID_MODE ? "  <- mode not listed" : "  <- listed, but the NEGOTIATION with the camera failed");
    show_candidates(devh, w, h);
    uvc_error_t neg = UVC_SUCCESS;
    int m = pick_mode_manually(devh, &ctrl, w, h, fps, &neg);
    if (m >= 0) {
      fprintf(stderr, "\nlibuvc's matching failed but picking the mode from the parsed descriptors WORKED: %s %dx%d @ %d fps\n", m ? "MJPEG" : "YUYV", w, h, fps);
      k = m ? 0 : 1;                                             // fmts[0] = MJPEG, fmts[1] = YUYV: carry on to the streaming test below
    }
  }
  if (k == 2) {
    fprintf(stderr, "\nstill no usable mode: try  ./uvctest %04x:%04x WIDTH HEIGHT FPS  with another size from the list above\n", vid, pid);
    uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx); return 4;
  }
  expect_bytes = fmts[k] == UVC_FRAME_FORMAT_YUYV ? (size_t)w * h * 2 : 0;

  printf("\n=== streaming %s %dx%d @ %d fps for %d s ===\n", names[k], w, h, fps, secs);
  uvc_print_stream_ctrl(&ctrl, stdout);
  if ((r = uvc_start_streaming(devh, &ctrl, on_frame, NULL, 0)) < 0) {
    uvc_perror(r, "uvc_start_streaming - USB bandwidth, or the camera refused the mode");
    uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx); return 5;
  }
  for (int i = 1; i <= secs; i++) {
    sleep(1);
    pthread_mutex_lock(&lock); int n = frames; pthread_mutex_unlock(&lock);
    printf("  %2d s: %d frames\n", i, n); fflush(stdout);
  }
  uvc_stop_streaming(devh);

  double span = t_last - t_first, got = (frames > 1 && span > 0) ? (frames - 1) / span : 0;
  printf("\n=== result ===\n");
  printf("frames: %d in %.1f s -> %.1f fps (asked for %d)\n", frames, span, got, fps);
  if (frames) printf("average frame: %ld bytes\n", bytes / frames);
  if (expect_bytes) printf("incomplete frames: %d of %d\n", short_frames, frames);
  if (keep) { save_frame(keep); uvc_free_frame(keep); }
  if (frames == 0)
    printf("VERDICT: NO FRAMES - the camera opened but nothing streamed (isochronous USB transfers failing?)\n");
  else if (got < 0.8 * fps || (expect_bytes && short_frames > frames / 10))
    printf("VERDICT: STREAMS BUT SLOW/INCOMPLETE - try a smaller mode, or check the light (exposure priority)\n");
  else
    printf("VERDICT: OK - video streams over raw USB on this phone\n");

  uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx);
  return 0;
}
