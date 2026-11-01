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
  for (k = 0; k < 2; k++)
    if (uvc_get_stream_ctrl_format_size(devh, &ctrl, fmts[k], w, h, fps) == UVC_SUCCESS) break;
  if (k == 2) {
    fprintf(stderr, "\nno MJPEG or YUYV mode at %dx%d @ %d fps - pick one from the list above:\n"
                    "  ./uvctest %04x:%04x WIDTH HEIGHT FPS\n", w, h, fps, vid, pid);
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
