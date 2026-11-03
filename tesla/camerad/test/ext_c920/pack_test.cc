// Off-EON check of ext_c920_pack_bgr: decodes a 1280x720 JPEG (libjpeg), packs it exactly as camerad would (crop 58 px each side, centred on a black 1164x874 BGR canvas) and writes the raw canvas.
// build/run: see test_pack.py
#include <setjmp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jpeglib.h>
#include "camera_ext_c920.h"
volatile sig_atomic_t do_exit = 0;
int main(int argc, char **argv) {
  if (argc < 3) return 2;
  FILE *f = fopen(argv[1], "rb"); if (!f) return 3;
  fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
  unsigned char *jp = (unsigned char *)malloc(n); if (fread(jp, 1, n, f) != (size_t)n) return 4; fclose(f);
  struct jpeg_decompress_struct ci; struct jpeg_error_mgr er; ci.err = jpeg_std_error(&er);
  jpeg_create_decompress(&ci); jpeg_mem_src(&ci, jp, n); jpeg_read_header(&ci, TRUE); ci.out_color_space = JCS_RGB; jpeg_start_decompress(&ci);
  int w = ci.output_width, h = ci.output_height; unsigned char *rgb = (unsigned char *)malloc((size_t)w * h * 3);
  while (ci.output_scanline < ci.output_height) { JSAMPROW r = rgb + (size_t)ci.output_scanline * w * 3; jpeg_read_scanlines(&ci, &r, 1); }
  jpeg_finish_decompress(&ci);
  const int OW = 1164, OH = 874, stride = 3600;                       // a stride larger than width*3, like the aligned buffer on the EON
  unsigned char *dst = (unsigned char *)calloc((size_t)stride * OH, 1);
  ext_c920_pack_bgr(rgb, w, h, dst, stride, OW, OH, 58, 77);
  FILE *o = fopen(argv[2], "wb"); for (int y = 0; y < OH; y++) fwrite(dst + (size_t)y * stride, 1, OW * 3, o); fclose(o);
  printf("%dx%d -> canvas %dx%d written\n", w, h, OW, OH);
  return 0;
}
