#pragma once
// thneed self-test comparison (17 Nov 2026). Acceptance is EXACT: every output float must have the same bits as the
// reference (no tolerance). The rest only describes a mismatch for the log. NaN / infinity are recognised from the
// bits, so this also holds in a build with fast-math.
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct ThneedCompare {
  size_t n_diff = 0;                // outputs whose bits differ (0 = identical: the only accepted result)
  size_t first = (size_t)-1;        // index of the first one
  float first_got = 0.f, first_want = 0.f;
  double max_abs = 0.;              // largest |got - want| among finite pairs
  double max_rel = 0.;              // largest |got - want| / |want| among finite pairs with want != 0
  size_t n_nonfinite = 0;           // differing pairs where either value is NaN or infinite
  size_t n_nan = 0;                 // ... of which either value is NaN
};

static inline bool thneed_bits_nan(uint32_t u) { return (u & 0x7f800000u) == 0x7f800000u && (u & 0x007fffffu) != 0; }
static inline bool thneed_bits_inf(uint32_t u) { return (u & 0x7fffffffu) == 0x7f800000u; }

inline ThneedCompare thneed_compare_outputs(const float *got, const float *want, size_t n) {
  ThneedCompare c;
  for (size_t i = 0; i < n; i++) {
    uint32_t a, b;
    memcpy(&a, &got[i], sizeof(a));
    memcpy(&b, &want[i], sizeof(b));
    if (a == b) continue;
    if (c.n_diff++ == 0) { c.first = i; c.first_got = got[i]; c.first_want = want[i]; }
    const bool nan = thneed_bits_nan(a) || thneed_bits_nan(b);
    if (nan) c.n_nan++;
    if (nan || thneed_bits_inf(a) || thneed_bits_inf(b)) { c.n_nonfinite++; continue; }
    const double d = fabs((double)got[i] - (double)want[i]);
    if (d > c.max_abs) c.max_abs = d;
    if (b != 0u && b != 0x80000000u) {
      const double r = d / fabs((double)want[i]);
      if (r > c.max_rel) c.max_rel = r;
    }
  }
  return c;
}
