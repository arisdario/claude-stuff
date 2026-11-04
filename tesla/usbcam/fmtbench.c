/* fmtbench - READ-ONLY benchmark of the video formats the Logitech C920 offers, for 1280x720 at 20 fps. Nothing is changed in openpilot.
 *
 *   ./fmtbench list                              formats / sizes / frame rates the camera advertises (as parsed by libuvc)
 *   ./fmtbench run <mjpeg|yuyv|h264> [seconds]   stream that format alone (default 20 s) and report:
 *        negotiated frame rate and the USB bandwidth the camera RESERVES (dwMaxPayloadTransferSize) vs the bytes actually delivered per second
 *        delivered fps, frame-interval jitter, late frames (>1.5 intervals), sequence gaps (dropped frames), frame sizes
 *        decode time per frame (MJPEG: libjpeg like camerad; YUYV: conversion to RGB; H.264: not decoded - no decoder on the EON image, delivery numbers only)
 *        age of each frame when it reaches the callback (now - the camera's capture time estimate)
 *        CPU of this process (user+system) as % of one core
 *   ./fmtbench all [seconds]                     the three runs one after the other + a comparison table
 *
 * Only one program can own the camera: stop openpilot first (tmux kill-session -t comma), run, then reboot or restart it.
 * The model input, camera geometry and driving pipeline are not touched; nothing is written except the report on stdout. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <setjmp.h>
#include "libuvc/libuvc.h"
#if defined(__has_include)
#if __has_include(<jpeglib.h>)
#include <stdio.h>
#include <jpeglib.h>
#define HAVE_JPEG 1
#endif
#endif

#define W 1280
#define H 720
#define MAXF 4000

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static double wall_s(void) { struct timeval t; gettimeofday(&t, NULL); return t.tv_sec + t.tv_usec * 1e-6; }

typedef struct { int fmt; double t0, nominal; long n, bytes, sizemin, sizemax, gaps, late; uint32_t lastseq; int haveseq; double last_arrival;
  double iv[MAXF], age[MAXF], dec[MAXF]; int niv, nage, ndec; uint8_t *rgb; int decode; } Stats;
static Stats S;

#ifdef HAVE_JPEG
struct jerr { struct jpeg_error_mgr pub; jmp_buf jb; };
static void jexit(j_common_ptr c) { longjmp(((struct jerr *)c->err)->jb, 1); }
static int dec_mjpeg(const uint8_t *d, size_t len, uint8_t *rgb) {
  struct jpeg_decompress_struct ci; struct jerr je;
  ci.err = jpeg_std_error(&je.pub); je.pub.error_exit = jexit;
  if (setjmp(je.jb)) { jpeg_destroy_decompress(&ci); return 0; }
  jpeg_create_decompress(&ci); jpeg_mem_src(&ci, (unsigned char *)d, len); jpeg_read_header(&ci, TRUE);
  ci.out_color_space = JCS_RGB; ci.dct_method = JDCT_IFAST; ci.do_fancy_upsampling = FALSE;   /* same settings as camerad */
  jpeg_start_decompress(&ci);
  if (ci.output_width != W || ci.output_height != H) { jpeg_destroy_decompress(&ci); return 0; }
  while (ci.output_scanline < ci.output_height) { JSAMPROW row = rgb + (size_t)ci.output_scanline * W * 3; jpeg_read_scanlines(&ci, &row, 1); }
  jpeg_finish_decompress(&ci); jpeg_destroy_decompress(&ci); return 1;
}
#endif
static inline uint8_t clip(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
static void dec_yuyv(const uint8_t *s, uint8_t *o) {
  for (int i = 0; i < W * H / 2; i++) { int y0 = s[0], u = s[1] - 128, y1 = s[2], v = s[3] - 128; s += 4;
    int rr = (359 * v) >> 8, gg = (88 * u + 183 * v) >> 8, bb = (454 * u) >> 8;
    o[0] = clip(y0 + rr); o[1] = clip(y0 - gg); o[2] = clip(y0 + bb); o[3] = clip(y1 + rr); o[4] = clip(y1 - gg); o[5] = clip(y1 + bb); o += 6; }
}

static void cb(uvc_frame_t *f, void *ptr) {
  double t = now_s();
  Stats *s = &S;
  if (s->t0 == 0) s->t0 = t;
  if (s->last_arrival > 0 && s->niv < MAXF) {
    double iv = t - s->last_arrival; s->iv[s->niv++] = iv; if (iv > 1.5 * s->nominal) s->late++;
  }
  s->last_arrival = t;
  s->n++; s->bytes += f->data_bytes;
  if (s->sizemin == 0 || (long)f->data_bytes < s->sizemin) s->sizemin = f->data_bytes;
  if ((long)f->data_bytes > s->sizemax) s->sizemax = f->data_bytes;
  if (s->haveseq && f->sequence > s->lastseq + 1) s->gaps += f->sequence - s->lastseq - 1;
  s->lastseq = f->sequence; s->haveseq = 1;
  double age = wall_s() - (f->capture_time.tv_sec + f->capture_time.tv_usec * 1e-6);
  if (f->capture_time.tv_sec && age > -1.0 && age < 10.0 && s->nage < MAXF) s->age[s->nage++] = age;
  if (s->decode && s->ndec < MAXF) {
    double a = now_s(); int ok = 0;
    if (s->fmt == UVC_FRAME_FORMAT_MJPEG) {
#ifdef HAVE_JPEG
      ok = dec_mjpeg(f->data, f->data_bytes, s->rgb);
#endif
    } else if (s->fmt == UVC_FRAME_FORMAT_YUYV && f->data_bytes >= (size_t)W * H * 2) { dec_yuyv(f->data, s->rgb); ok = 1; }
    if (ok) s->dec[s->ndec++] = now_s() - a;
  }
}

static int cmpd(const void *a, const void *b) { double x = *(double *)a, y = *(double *)b; return x < y ? -1 : x > y; }
static double pct(double *v, int n, double p) { if (!n) return -1; qsort(v, n, sizeof(double), cmpd); return v[(int)(p * (n - 1))]; }
static double mean(double *v, int n) { double s = 0; for (int i = 0; i < n; i++) s += v[i]; return n ? s / n : -1; }

typedef struct { char name[8]; int ok; double fps, mbit, reserved_mbit, ivp99, ivmax, late_pct, gaps, dec_mean, dec_p99, age_mean, age_p99, cpu, size_kb; } Row;

static int run(uvc_device_handle_t *devh, const char *name, double seconds, Row *row) {
  int fmt = !strcmp(name, "mjpeg") ? UVC_FRAME_FORMAT_MJPEG : !strcmp(name, "yuyv") ? UVC_FRAME_FORMAT_YUYV : !strcmp(name, "h264") ? UVC_FRAME_FORMAT_H264 : -1;
  if (fmt < 0) { fprintf(stderr, "format must be mjpeg, yuyv or h264\n"); return 2; }
  strncpy(row->name, name, 7); row->ok = 0;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t r = uvc_get_stream_ctrl_format_size(devh, &ctrl, fmt, W, H, 20);
  printf("\n=== %s 1280x720 @ 20 fps ===\n", name);
  if (r < 0) { printf("NOT AVAILABLE: libuvc found no %s mode at 1280x720/20 fps (%s)\n", name, uvc_strerror(r)); return 0; }
  printf("negotiated: frame interval %u (x100 ns) = %.2f fps, dwMaxVideoFrameSize %u, dwMaxPayloadTransferSize %u bytes per USB (micro)frame\n",
         ctrl.dwFrameInterval, 1e7 / ctrl.dwFrameInterval, ctrl.dwMaxVideoFrameSize, ctrl.dwMaxPayloadTransferSize);
  memset(&S, 0, sizeof S);
  S.fmt = fmt; S.nominal = ctrl.dwFrameInterval * 1e-7; S.decode = 1;
  S.rgb = (uint8_t *)malloc((size_t)W * H * 3);
  struct rusage a, b; getrusage(RUSAGE_SELF, &a); double w0 = now_s();
  r = uvc_start_streaming(devh, &ctrl, cb, NULL, 0);
  if (r < 0) { printf("start failed: %s\n", uvc_strerror(r)); free(S.rgb); return 0; }
  while (now_s() - w0 < seconds) usleep(100000);
  uvc_stop_streaming(devh);
  double wall = now_s() - w0; getrusage(RUSAGE_SELF, &b);
  double cpu = ((b.ru_utime.tv_sec - a.ru_utime.tv_sec) + (b.ru_utime.tv_usec - a.ru_utime.tv_usec) * 1e-6 + (b.ru_stime.tv_sec - a.ru_stime.tv_sec) + (b.ru_stime.tv_usec - a.ru_stime.tv_usec) * 1e-6) / wall * 100;
  double fps = S.n / wall, mbit = S.bytes * 8.0 / wall / 1e6;
  /* high-speed isochronous: the camera reserves dwMaxPayloadTransferSize bytes in every 125 us microframe (8000 per second) */
  double reserved = ctrl.dwMaxPayloadTransferSize * 8000.0 * 8 / 1e6;
  printf("delivered: %ld frames in %.1f s = %.2f fps | payload %.1f Mbit/s (%.2f MB/s) | frame size min %.0f / mean %.0f / max %.0f KB\n",
         S.n, wall, fps, mbit, mbit / 8, S.sizemin / 1024.0, S.bytes / 1024.0 / (S.n ? S.n : 1), S.sizemax / 1024.0);
  printf("reserved USB bandwidth (worst case if the camera used every slot): %.0f Mbit/s of 480\n", reserved);
  double ivp99 = pct(S.iv, S.niv, 0.99), ivmax = pct(S.iv, S.niv, 1.0);
  printf("frame interval: mean %.1f ms, p99 %.1f ms, max %.1f ms | late (>1.5x nominal): %ld (%.1f%%) | sequence gaps (dropped): %ld\n",
         mean(S.iv, S.niv) * 1e3, ivp99 * 1e3, ivmax * 1e3, S.late, 100.0 * S.late / (S.n ? S.n : 1), S.gaps);
  double dm = -1, dp = -1;
  if (S.ndec) { dm = mean(S.dec, S.ndec); dp = pct(S.dec, S.ndec, 0.99); printf("decode (%s): mean %.1f ms, p99 %.1f ms per frame\n", fmt == UVC_FRAME_FORMAT_MJPEG ? "libjpeg like camerad" : "YUYV->RGB", dm * 1e3, dp * 1e3); }
  else printf("decode: not measured (%s)\n", fmt == UVC_FRAME_FORMAT_H264 ? "no H.264 decoder in this tool: delivery numbers only" : "libjpeg missing at build");
  double am = -1, ap = -1;
  if (S.nage) { am = mean(S.age, S.nage); ap = pct(S.age, S.nage, 0.99); printf("frame age at delivery (now - camera capture-time estimate): mean %.1f ms, p99 %.1f ms\n", am * 1e3, ap * 1e3); }
  else printf("frame age: the camera gave no usable capture timestamp\n");
  printf("CPU of this process: %.1f%% of one core (USB handling + decode)\n", cpu);
  row->ok = 1; row->fps = fps; row->mbit = mbit; row->reserved_mbit = reserved; row->ivp99 = ivp99 * 1e3; row->ivmax = ivmax * 1e3;
  row->late_pct = 100.0 * S.late / (S.n ? S.n : 1); row->gaps = S.gaps; row->dec_mean = dm * 1e3; row->dec_p99 = dp * 1e3; row->age_mean = am * 1e3; row->age_p99 = ap * 1e3; row->cpu = cpu;
  row->size_kb = S.bytes / 1024.0 / (S.n ? S.n : 1);
  free(S.rgb);
  return 0;
}

static void list(uvc_device_handle_t *devh) {
  for (const uvc_format_desc_t *f = uvc_get_format_descs(devh); f; f = f->next) {
    const char *k = f->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG ? "MJPEG" : f->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED ? "uncompressed" :
                    f->bDescriptorSubtype == UVC_VS_FORMAT_FRAME_BASED ? "frame-based" : "other";
    printf("format %u: %s  fourcc '%.4s'\n", f->bFormatIndex, k, (const char *)f->fourccFormat);
    for (const uvc_frame_desc_t *fr = f->frame_descs; fr; fr = fr->next) {
      printf("   %4ux%-4u ", fr->wWidth, fr->wHeight);
      if (fr->intervals) { for (const uint32_t *i = fr->intervals; *i; i++) printf(" %.0f", 1e7 / *i); printf(" fps"); }
      else printf(" continuous %.0f..%.0f fps", 1e7 / (fr->dwMaxFrameInterval ? fr->dwMaxFrameInterval : 1), 1e7 / (fr->dwMinFrameInterval ? fr->dwMinFrameInterval : 1));
      if (fr->wWidth == W && fr->wHeight == H) printf("   <-- 1280x720");
      printf("\n");
    }
  }
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: fmtbench list | run <mjpeg|yuyv|h264> [s] | all [s]\n"); return 2; }
  uvc_context_t *ctx; uvc_device_t *dev; uvc_device_handle_t *devh;
  if (uvc_init(&ctx, NULL) < 0) return 1;
  if (uvc_find_device(ctx, &dev, 0x046d, 0x082d, NULL) < 0) { fprintf(stderr, "no C920 found\n"); return 1; }
  if (uvc_open(dev, &devh) < 0) { fprintf(stderr, "cannot open the camera - stop openpilot first (tmux kill-session -t comma)\n"); return 1; }
  if (!strcmp(argv[1], "list")) list(devh);
  else if (!strcmp(argv[1], "run") && argc >= 3) { Row r; run(devh, argv[2], argc >= 4 ? atof(argv[3]) : 20, &r); }
  else if (!strcmp(argv[1], "all")) {
    double sec = argc >= 3 ? atof(argv[2]) : 20; Row rows[3]; const char *n[3] = {"mjpeg", "yuyv", "h264"};
    for (int i = 0; i < 3; i++) { memset(&rows[i], 0, sizeof(Row)); run(devh, n[i], sec, &rows[i]); sleep(3); }
    printf("\n%-6s %7s %8s %9s %8s %7s %6s %9s %9s %8s %7s\n", "format", "fps", "Mbit/s", "frame KB", "iv p99ms", "late%", "drops", "decode ms", "age p99ms", "CPU %", "ok");
    for (int i = 0; i < 3; i++) printf("%-6s %7.2f %8.1f %9.0f %8.1f %7.1f %6.0f %9.1f %9.1f %8.1f %7s\n", rows[i].name, rows[i].fps, rows[i].mbit, rows[i].size_kb, rows[i].ivp99, rows[i].late_pct,
                                      rows[i].gaps, rows[i].dec_mean, rows[i].age_p99, rows[i].cpu, rows[i].ok ? "yes" : "n/a");
  }
  uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx);
  return 0;
}
