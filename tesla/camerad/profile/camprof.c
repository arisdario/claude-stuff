/* camprof - PROFILING ONLY: where does camerad spend CPU per C920 frame, step by step, on the EON. Changes nothing anywhere.
 *
 *   ./camprof [frame.jpg] [iterations]       default: /dev/c920_live.jpg (a real frame written by camerad), 200 iterations
 *
 * It replays the CPU part of camerad's per-frame path on one real frame, timing every stage separately (same libjpeg settings, same pack function, same sizes):
 *   decode_rgb      libjpeg to RGB, IFAST, no fancy upsampling      = what camerad does today
 *   decode_raw      libjpeg to planar YCbCr only (no colour conversion, no upsampling)  -> the difference to decode_rgb is the cost of libjpeg's colour conversion
 *   hdr_only        jpeg_create + read_header + destroy             = per-frame setup/allocation cost inside libjpeg
 *   flip180         in-place 180 degree turn of the RGB frame (only when /data/camera_flip=1, which is YOUR case)
 *   luma_sample     the sparse brightness sample for the profile engine
 *   pack_bgr        RGB -> BGR with crop+pad into the 1164x874 vision buffer (ext_c920_pack_bgr, copied verbatim)
 *   publish_live    writing the raw JPEG to a file for the web page (every 3rd-4th frame, ~10/s)
 *   memcpy_vb       a plain memcpy of the same 1164x874x3 buffer (the floor for "touch every byte once")
 *   memcpy_yuv      memcpy of the 1.5 MB YUV frame that modeld copies on the CPU for every frame (stock behaviour)
 * and two HYPOTHETICAL stages, only to estimate what other designs would cost on the CPU:
 *   cpu_rgb2i420    the GPU kernel rgb_to_yuv.cl done on the CPU (same integer math) - shows what moving it off the GPU would cost
 *   cpu_direct_i420 planar YCbCr from the JPEG -> limited-range I420 on the 1164x874 canvas (crop, pad, range scaling, 2x2 chroma on the canvas grid)
 * Each stage: mean and p95 in ms, and the CPU share at 20 fps (mean*20/10 %% of one core). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <setjmp.h>
#include <stdint.h>
#include <jpeglib.h>

#define SRC_W 1280
#define SRC_H 720
#define OUT_W 1164
#define OUT_H 874
#define CROP_X 58
#define PAD_Y 77
#define MAXN 2000

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec * 1e-6; }
struct jerr { struct jpeg_error_mgr pub; jmp_buf jb; };
static void jexit(j_common_ptr c) { longjmp(((struct jerr *)c->err)->jb, 1); }

static uint8_t *file_data; static size_t file_len;
static uint8_t *rgb, *vb, *yuvA, *yuvB, *scratch, *planes[3]; static int pw[3], ph[3], pstride[3];
static int hsamp, vsamp;

static int dec_rgb(void) {
  struct jpeg_decompress_struct ci; struct jerr je; ci.err = jpeg_std_error(&je.pub); je.pub.error_exit = jexit;
  if (setjmp(je.jb)) { jpeg_destroy_decompress(&ci); return 0; }
  jpeg_create_decompress(&ci); jpeg_mem_src(&ci, file_data, file_len); jpeg_read_header(&ci, TRUE);
  ci.out_color_space = JCS_RGB; ci.dct_method = JDCT_IFAST; ci.do_fancy_upsampling = FALSE;
  jpeg_start_decompress(&ci);
  while (ci.output_scanline < ci.output_height) { JSAMPROW row = rgb + (size_t)ci.output_scanline * SRC_W * 3; jpeg_read_scanlines(&ci, &row, 1); }
  jpeg_finish_decompress(&ci); jpeg_destroy_decompress(&ci); return 1;
}
static int dec_raw(void) {
  struct jpeg_decompress_struct ci; struct jerr je; ci.err = jpeg_std_error(&je.pub); je.pub.error_exit = jexit;
  if (setjmp(je.jb)) { jpeg_destroy_decompress(&ci); return 0; }
  jpeg_create_decompress(&ci); jpeg_mem_src(&ci, file_data, file_len); jpeg_read_header(&ci, TRUE);
  ci.out_color_space = JCS_YCbCr; ci.raw_data_out = TRUE; ci.dct_method = JDCT_IFAST; ci.do_fancy_upsampling = FALSE;
  jpeg_start_decompress(&ci);
  int mv = ci.max_v_samp_factor * DCTSIZE;
  JSAMPROW rows[3][32];
  for (int line = 0; line < (int)ci.output_height;) {
    for (int c = 0; c < 3; c++) { int n = ci.comp_info[c].v_samp_factor * DCTSIZE; for (int i = 0; i < n; i++) { int r = line * ci.comp_info[c].v_samp_factor / ci.max_v_samp_factor + i; rows[c][i] = planes[c] + (size_t)(r < ph[c] ? r : ph[c] - 1) * pstride[c]; } }
    JSAMPARRAY arr[3] = {rows[0], rows[1], rows[2]};
    line += jpeg_read_raw_data(&ci, arr, mv);
  }
  jpeg_finish_decompress(&ci); jpeg_destroy_decompress(&ci); return 1;
}
static void hdr_only(void) {
  struct jpeg_decompress_struct ci; struct jerr je; ci.err = jpeg_std_error(&je.pub); je.pub.error_exit = jexit;
  if (setjmp(je.jb)) { jpeg_destroy_decompress(&ci); return; }
  jpeg_create_decompress(&ci); jpeg_mem_src(&ci, file_data, file_len); jpeg_read_header(&ci, TRUE); jpeg_destroy_decompress(&ci);
}
static void flip180(void) { uint8_t *a = rgb, *z = rgb + (size_t)SRC_W * SRC_H * 3 - 3; for (; a < z; a += 3, z -= 3) for (int k = 0; k < 3; k++) { uint8_t t = a[k]; a[k] = z[k]; z[k] = t; } }
static volatile float sink;
static void luma_sample(void) { long sum = 0, n = 0; for (int y = SRC_H * 45 / 100; y < SRC_H * 90 / 100; y += 12) for (int x = 0; x < SRC_W; x += 16) { const uint8_t *p = rgb + ((size_t)y * SRC_W + x) * 3; sum += (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8; n++; } sink = (float)sum / n; }
static void pack_bgr(const uint8_t *r, int src_w, int src_h, uint8_t *dst, int dst_stride, int dst_w, int dst_h, int x0, int y0) {   /* verbatim from camera_ext_c920.cc */
  for (int y = 0; y < src_h && y + y0 < dst_h; y++) { const uint8_t *s = r + (size_t)y * src_w * 3 + (size_t)x0 * 3; uint8_t *d = dst + (size_t)(y + y0) * dst_stride;
    for (int x = 0; x < dst_w; x++) { d[3 * x] = s[3 * x + 2]; d[3 * x + 1] = s[3 * x + 1]; d[3 * x + 2] = s[3 * x]; } }
}
static void publish_live(const char *dir) { char p[300], q[300]; snprintf(p, sizeof p, "%s/camprof_live.tmp", dir); snprintf(q, sizeof q, "%s/camprof_live.jpg", dir);
  FILE *f = fopen(p, "wb"); if (f) { fwrite(file_data, 1, file_len, f); fclose(f); rename(p, q); } }
#define RGB_TO_Y(r, g, b) ((((b * 13 + g * 65 + r * 33) + 64) >> 7) + 16)
#define RGB_TO_U(r, g, b) ((b * 56 - g * 37 - r * 19 + 0x8080) >> 8)
#define RGB_TO_V(r, g, b) ((r * 56 - g * 47 - b * 9 + 0x8080) >> 8)
static void cpu_rgb2i420(void) {   /* the arithmetic of rgb_to_yuv.cl: Y per pixel, U/V from the average of each 2x2 block of the 1164x874 BGR buffer */
  const int stride = OUT_W * 3; uint8_t *Y = yuvA, *U = yuvA + OUT_W * OUT_H, *V = U + (OUT_W / 2) * (OUT_H / 2);
  for (int y = 0; y < OUT_H; y += 2) for (int x = 0; x < OUT_W; x += 2) {
    const uint8_t *p0 = vb + y * stride + x * 3, *p1 = p0 + stride; int ab = 0, ag = 0, ar = 0;
    for (int k = 0; k < 2; k++) { const uint8_t *a = (k ? p1 : p0); for (int j = 0; j < 2; j++) { Y[(y + k) * OUT_W + x + j] = RGB_TO_Y(a[j * 3 + 2], a[j * 3 + 1], a[j * 3]); ab += a[j * 3]; ag += a[j * 3 + 1]; ar += a[j * 3 + 2]; } }
    ab = (ab + 1) >> 1; ag = (ag + 1) >> 1; ar = (ar + 1) >> 1;      /* the GPU code halves the 4-sample sum (sic) */
    U[(y / 2) * (OUT_W / 2) + x / 2] = RGB_TO_U(ar, ag, ab); V[(y / 2) * (OUT_W / 2) + x / 2] = RGB_TO_V(ar, ag, ab); }
}
static uint8_t lutY[256], lutC[256];
static void cpu_direct_i420(void) {
  /* planar YCbCr (full range, JFIF) -> limited-range I420 on the 1164x874 canvas: crop CROP_X, pad PAD_Y rows, chroma averaged on the CANVAS 2x2 grid (the picture starts on an odd canvas row) */
  uint8_t *Y = yuvB, *U = yuvB + OUT_W * OUT_H, *V = U + (OUT_W / 2) * (OUT_H / 2);
  memset(Y, 16, OUT_W * OUT_H); memset(U, 128, (OUT_W / 2) * (OUT_H / 2) * 2);
  for (int y = 0; y < SRC_H && y + PAD_Y < OUT_H; y++) { const uint8_t *s = planes[0] + (size_t)y * pstride[0] + CROP_X; uint8_t *d = Y + (size_t)(y + PAD_Y) * OUT_W; for (int x = 0; x < OUT_W; x++) d[x] = lutY[s[x]]; }
  for (int cy = 0; cy < OUT_H / 2; cy++) { int r0 = 2 * cy - PAD_Y, r1 = r0 + 1; if (r1 < 0 || r0 >= SRC_H) continue;
    int c0 = r0 < 0 ? 0 : r0, c1 = r1 >= SRC_H ? SRC_H - 1 : r1; int q0 = c0 * ph[1] / SRC_H, q1 = c1 * ph[1] / SRC_H;
    for (int cx = 0; cx < OUT_W / 2; cx++) { int sx = (CROP_X / 2) + cx; if (sx >= pw[1]) break;
      U[cy * (OUT_W / 2) + cx] = lutC[(planes[1][q0 * pstride[1] + sx] + planes[1][q1 * pstride[1] + sx] + 1) >> 1];
      V[cy * (OUT_W / 2) + cx] = lutC[(planes[2][q0 * pstride[2] + sx] + planes[2][q1 * pstride[2] + sx] + 1) >> 1]; } }
}

static int cmpd(const void *a, const void *b) { double x = *(double *)a, y = *(double *)b; return x < y ? -1 : x > y; }
#define BENCH(name, expr) do { double v[MAXN]; for (int w = 0; w < 5; w++) { expr; } for (int i = 0; i < n; i++) { double t0 = now(); expr; v[i] = now() - t0; } \
  double s = 0; for (int i = 0; i < n; i++) s += v[i]; qsort(v, n, sizeof(double), cmpd); printf("%-16s mean %7.2f ms   p95 %7.2f ms   = %5.1f%% of a core at 20 fps\n", name, s / n, v[(int)(0.95 * (n - 1))], s / n * 2.0); } while (0)

int main(int argc, char **argv) {
  const char *path = argc > 1 ? argv[1] : "/dev/c920_live.jpg"; int n = argc > 2 ? atoi(argv[2]) : 200; if (n > MAXN) n = MAXN;
  FILE *f = fopen(path, "rb"); if (!f) { fprintf(stderr, "cannot open %s\n", path); return 1; }
  fseek(f, 0, SEEK_END); file_len = ftell(f); fseek(f, 0, SEEK_SET); file_data = malloc(file_len); if (fread(file_data, 1, file_len, f) != file_len) return 1; fclose(f);
  struct jpeg_decompress_struct ci; struct jerr je; ci.err = jpeg_std_error(&je.pub); jpeg_create_decompress(&ci); jpeg_mem_src(&ci, file_data, file_len); jpeg_read_header(&ci, TRUE);
  if (ci.image_width != SRC_W || ci.image_height != SRC_H) { fprintf(stderr, "frame is %ux%u, expected 1280x720\n", ci.image_width, ci.image_height); return 1; }
  hsamp = ci.comp_info[0].h_samp_factor / ci.comp_info[1].h_samp_factor; vsamp = ci.comp_info[0].v_samp_factor / ci.comp_info[1].v_samp_factor;
  printf("frame %s: %zu bytes, %ux%u, %d components, chroma subsampling %dx%d (%s)\n", path, file_len, ci.image_width, ci.image_height, ci.num_components, hsamp, vsamp,
         hsamp == 2 && vsamp == 1 ? "4:2:2" : hsamp == 2 && vsamp == 2 ? "4:2:0" : "other");
  jpeg_destroy_decompress(&ci);
  rgb = malloc((size_t)SRC_W * SRC_H * 3); vb = calloc((size_t)OUT_W * OUT_H * 3, 1); yuvA = malloc((size_t)OUT_W * OUT_H * 3 / 2); yuvB = malloc((size_t)OUT_W * OUT_H * 3 / 2); scratch = malloc((size_t)OUT_W * OUT_H * 3);
  pw[0] = SRC_W; ph[0] = SRC_H; pw[1] = pw[2] = SRC_W / hsamp; ph[1] = ph[2] = SRC_H / vsamp;
  for (int c = 0; c < 3; c++) { pstride[c] = pw[c]; planes[c] = malloc((size_t)pstride[c] * (ph[c] + 32)); }
  for (int i = 0; i < 256; i++) { lutY[i] = 16 + (i * 219 + 127) / 255; lutC[i] = 128 + (((i - 128) * 224 + (i >= 128 ? 127 : -127)) / 255); }
  dec_rgb(); pack_bgr(rgb, SRC_W, SRC_H, vb, OUT_W * 3, OUT_W, OUT_H, CROP_X, PAD_Y);
  printf("\n-- what camerad does on the CPU for every frame --\n");
  BENCH("decode_rgb", dec_rgb());
  BENCH("flip180", flip180());
  BENCH("luma_sample", luma_sample());
  BENCH("pack_bgr", pack_bgr(rgb, SRC_W, SRC_H, vb, OUT_W * 3, OUT_W, OUT_H, CROP_X, PAD_Y));
  BENCH("publish_live", publish_live(argc > 3 ? argv[3] : "/tmp"));
  printf("   (publish_live runs ~10 times per second, not 20: halve its share)\n");
  printf("\n-- breakdown / floors --\n");
  BENCH("hdr_only", hdr_only());
  BENCH("decode_raw", dec_raw());
  BENCH("memcpy_vb", memcpy(scratch, vb, (size_t)OUT_W * OUT_H * 3));
  BENCH("memcpy_yuv", memcpy(yuvB, yuvA, (size_t)OUT_W * OUT_H * 3 / 2));   /* yuvA/yuvB are 1.5 MB each */
  printf("   (memcpy_yuv is what modeld does with every frame, in stock openpilot)\n");
  printf("\n-- hypothetical, estimates only --\n");
  BENCH("cpu_rgb2i420", cpu_rgb2i420());
  dec_raw();
  BENCH("cpu_direct_i420", cpu_direct_i420());
  /* how far is the direct path from the GPU path? compare the two I420 results (both made from the same frame) */
  dec_rgb(); pack_bgr(rgb, SRC_W, SRC_H, vb, OUT_W * 3, OUT_W, OUT_H, CROP_X, PAD_Y); cpu_rgb2i420(); dec_raw(); cpu_direct_i420();
  long dy = 0, du = 0, nmax = 0; int my = 0, mu = 0; size_t ny = (size_t)OUT_W * OUT_H, nc = (size_t)(OUT_W / 2) * (OUT_H / 2);
  for (size_t i = 0; i < ny; i++) { int d = abs((int)yuvA[i] - (int)yuvB[i]); dy += d; if (d > my) my = d; }
  for (size_t i = 0; i < nc * 2; i++) { int d = abs((int)yuvA[ny + i] - (int)yuvB[ny + i]); du += d; if (d > mu) mu = d; }
  printf("\ncompatibility of the direct YCbCr path with the stock path (same frame): Y mean |diff| %.2f max %d | U,V mean |diff| %.2f max %d  (levels, 0..255)\n", (double)dy / ny, my, (double)du / (2 * nc), mu); (void)nmax;
  printf("\nnote: with the picture starting on canvas row %d (odd), the 2x2 chroma blocks on the top/bottom canvas edges mix picture and padding rows in the stock path too.\n", PAD_Y);
  return 0;
}
