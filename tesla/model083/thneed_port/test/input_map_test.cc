// input_map_test.cc - thneed_match_inputs (thneed/input_map.h): pairs SNPE's input buffers with the runner's by size.
//   g++ -std=c++11 -I../thneed input_map_test.cc -o /tmp/imt && /tmp/imt
#include <cstdio>
#include "input_map.h"

static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { printf("FAIL: %s\n", m); fails++; } else printf("ok:   %s\n", m); } while (0)

int main() {
  const size_t ours[4] = {1572864, 2048, 32, 8};          // image, memory state, desire, traffic side (bytes)
  int perm[4];
  { const size_t t[4] = {2048, 8, 32, 1572864};            // 0.8.3's order: recurrent, traffic, desire, image
    CHECK(thneed_match_inputs(t, 4, ours, 4, perm) && perm[0] == 1 && perm[1] == 3 && perm[2] == 2 && perm[3] == 0, "0.8.3 order"); }
  { const size_t t[4] = {1572864, 32, 2048, 8};
    CHECK(thneed_match_inputs(t, 4, ours, 4, perm) && perm[0] == 0 && perm[1] == 2 && perm[2] == 1 && perm[3] == 3, "another order"); }
  { const size_t t[4] = {2048, 16, 32, 1572864};           // SNPE padded the 2-float input to 4 floats
    CHECK(thneed_match_inputs(t, 4, ours, 4, perm) && perm[1] == 3, "padding accepted"); }
  { const size_t t[3] = {2048, 32, 1572864};
    CHECK(!thneed_match_inputs(t, 3, ours, 4, perm), "an input missing: refused"); }
  { const size_t t[4] = {2048, 8, 32, 1000000};            // image buffer smaller than the image
    CHECK(!thneed_match_inputs(t, 4, ours, 4, perm), "a buffer smaller than our data: refused"); }
  { const size_t t[4] = {2048, 8, 9000, 1572864};          // far too big for the desire
    CHECK(!thneed_match_inputs(t, 4, ours, 4, perm), "a buffer far larger than our data: refused"); }
  { const size_t t[4] = {2048, 32, 32, 1572864};
    CHECK(!thneed_match_inputs(t, 4, ours, 4, perm), "two buffers of the same size (ambiguous): refused"); }
  { const size_t o2[4] = {1572864, 32, 32, 8}; const size_t t[4] = {32, 8, 32, 1572864};
    CHECK(!thneed_match_inputs(t, 4, o2, 4, perm), "two of OUR inputs the same size: refused"); }
  printf("%s\n", fails ? "FAILED" : "all passed");
  return fails ? 1 : 0;
}
