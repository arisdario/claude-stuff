// thneed_bench - the parked check BEFORE modeld uses thneed. Runs the installed supercombo.dlc through the same runner
// modeld uses (runners/snpemodel.cc): with thneed, or plain SNPE with THNEED_OFF=1. Prints the self-test result (from
// the runner), the time and the CPU per run, and a checksum of the final output: the same inputs go through both
// modes, so equal checksums mean the replay gives exactly SNPE's result over every run, not only the first one.
//   cd /data/openpilot/selfdrive/modeld
//   export LD_LIBRARY_PATH=/data/pythonpath/phonelibs/snpe/aarch64
//   THNEED_OFF=1 ./thneed/thneed_bench ../../models/supercombo.dlc 6472 50
//   ./thneed/thneed_bench ../../models/supercombo.dlc 6472 50
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <sys/resource.h>
#include <vector>
#include "../runners/snpemodel.h"

static double now_s() { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static double cpu_s() {
  struct rusage r; getrusage(RUSAGE_SELF, &r);
  return r.ru_utime.tv_sec + r.ru_utime.tv_usec * 1e-6 + r.ru_stime.tv_sec + r.ru_stime.tv_usec * 1e-6;
}

int main(int argc, char **argv) {
  const char *path = argc > 1 ? argv[1] : "../../models/supercombo.dlc";
  const int out_n = argc > 2 ? atoi(argv[2]) : 6472;     // 0.8.12 model: 6472 floats (memory state included)
  const int runs = argc > 3 ? atoi(argv[3]) : 50;
  const int TEMPORAL = 512, DESIRE = 8, TRAFFIC = 2, FRAME = 6 * 128 * 256;
  if (out_n <= TEMPORAL || runs < 1) { fprintf(stderr, "usage: thneed_bench model.dlc output_floats runs\n"); return 2; }

  std::vector<float> output(out_n, 0.f), desire(DESIRE, 0.f), traffic(TRAFFIC, 0.f), input(2 * FRAME);
  traffic[0] = 1.f;                                       // right-hand traffic, as on the car
  uint32_t seed = 12345;
  for (auto &v : input) { seed = seed * 1103515245u + 12345u; v = (float)((seed >> 16) & 0xFF); }

  SNPEModel m(path, output.data(), out_n, USE_GPU_RUNTIME);
  m.addRecurrent(&output[out_n - TEMPORAL], TEMPORAL);   // as driving.cc: the memory state lives at the end of the output
  m.addDesire(desire.data(), DESIRE);
  m.addTrafficConvention(traffic.data(), TRAFFIC);

  double t = now_s();
  m.execute(input.data(), 2 * FRAME);                     // first run: thneed records + self-tests here
  printf("first run (incl. recording and self-test): %.0f ms\n", (now_s() - t) * 1000.);

  double t0 = now_s(), c0 = cpu_s();
  for (int i = 0; i < runs; i++) {
    for (int k = 0; k < 64; k++) input[(size_t)(i * 7919 + k * 104729) % input.size()] = (float)((i * 31 + k) & 0xFF);   // new picture
    m.execute(input.data(), 2 * FRAME);
  }
  const double wall = (now_s() - t0) / runs * 1000., cpu = (cpu_s() - c0) / runs * 1000.;

  uint64_t h = 1469598103934665603ULL;                     // FNV-1a over the final output
  const unsigned char *p = (const unsigned char *)output.data();
  for (size_t i = 0; i < output.size() * sizeof(float); i++) { h ^= p[i]; h *= 1099511628211ULL; }
  printf("%s: %.1f ms per run, CPU %.1f ms per run (user+sys, all threads), output checksum %016llx\n",
         getenv("THNEED_OFF") ? "plain SNPE" : "thneed build (see the self-test line above)", wall, cpu, (unsigned long long)h);
  return 0;
}
