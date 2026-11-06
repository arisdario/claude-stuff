// Off-EON test of camera_h264.h: feeds an Annex-B H.264 stream (access units separated by AUD NALs) to the decoder the way the camera's UVC frames arrive.
//   h264_test STREAM OUT.rgb MODE [N]    MODE: all | skipfirst (drop access unit 0) | corrupt (truncate access unit N to 40%)
// prints one line per access unit: "AU i out" (good picture, appended to OUT.rgb) or "AU i drop"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "camera_h264.h"

int main(int argc, char **argv) {
  if (argc < 4) { fprintf(stderr, "usage\n"); return 2; }
  FILE *f = fopen(argv[1], "rb"); if (!f) return 2;
  std::vector<uint8_t> all; uint8_t buf[65536]; size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) all.insert(all.end(), buf, buf + n);
  fclose(f);
  std::vector<size_t> starts;                       // access unit = from one AUD (nal type 9) to the next
  for (size_t i = 0; i + 4 < all.size(); i++)
    if (all[i] == 0 && all[i + 1] == 0 && all[i + 2] == 1 && (all[i + 3] & 31) == 9) { size_t s = (i > 0 && all[i - 1] == 0) ? i - 1 : i; starts.push_back(s); i += 3; }
  starts.push_back(all.size());
  const char *mode = argv[3]; int N = argc > 4 ? atoi(argv[4]) : -1;
  h264::Decoder d; char why[256] = "";
  if (!d.open(why, sizeof why)) { printf("OPEN FAILED: %s\n", why); return 3; }
  const int W = 1280, H = 720; std::vector<uint8_t> rgb((size_t)W * H * 3);
  FILE *o = fopen(argv[2], "wb");
  int outs = 0;
  for (size_t i = 0; i + 1 < starts.size(); i++) {
    if (!strcmp(mode, "skipfirst") && i == 0) { printf("AU %zu skipped\n", i); continue; }
    size_t len = starts[i + 1] - starts[i];
    if (!strcmp(mode, "corrupt") && (int)i == N) len = len * 4 / 10;
    bool ok = d.decode(&all[starts[i]], len, W, H, rgb.data());
    if (ok) { fwrite(rgb.data(), 1, rgb.size(), o); outs++; }
    printf("AU %zu %s\n", i, ok ? "out" : "drop");
  }
  fclose(o);
  printf("SUMMARY outs=%d errors=%ld dropped=%ld\n", outs, d.errors(), d.dropped());
  return 0;
}
