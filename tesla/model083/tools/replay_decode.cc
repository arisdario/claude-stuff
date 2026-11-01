// replay_decode - batch version of decode.cc: reads N raw 0.8.3 outputs (each OUTPUT_SIZE + TEMPORAL_SIZE floats,
// as written by replay083.py) and writes, per frame, what the EON's adapter (model083_adapter.h) publishes:
//   path y[192], left lane y[192], right lane y[192]   (metres, left positive, at x = 0..191 m)
//   left prob, right prob, lead prob, lead dist (m from the camera), lead y (left +)
// = 581 float32 per frame.   g++ -std=c++11 -O2 replay_decode.cc -o replay_decode && ./replay_decode in.bin out.bin
#include <stdio.h>
#include <vector>
#include "../model083_adapter.h"
using namespace m083;

int main(int argc, char **argv) {
  if (argc < 3) { fprintf(stderr, "usage: replay_decode raw_outputs.bin decoded.bin\n"); return 1; }
  FILE *fi = fopen(argv[1], "rb"), *fo = fopen(argv[2], "wb");
  if (!fi || !fo) { fprintf(stderr, "cannot open files\n"); return 1; }
  const int n = OUTPUT_SIZE + TEMPORAL_SIZE;
  std::vector<float> o(n), rec;
  int frames = 0;
  while (fread(o.data(), sizeof(float), n, fi) == (size_t)n) {
    Line p, l, r; Lead ld;
    path_from_plan(o.data(), p); lane_line(o.data(), LANE_LEFT, l); lane_line(o.data(), LANE_RIGHT, r); lead(o.data(), 0, ld);
    rec.assign(p.points, p.points + PATH_N);
    rec.insert(rec.end(), l.points, l.points + PATH_N);
    rec.insert(rec.end(), r.points, r.points + PATH_N);
    float tail[5] = {l.prob, r.prob, ld.prob, ld.dist, ld.rel_y};
    rec.insert(rec.end(), tail, tail + 5);
    fwrite(rec.data(), sizeof(float), rec.size(), fo);
    frames++;
  }
  fclose(fi); fclose(fo);
  printf("decoded %d frames\n", frames);
  return 0;
}
