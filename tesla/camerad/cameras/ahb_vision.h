#pragma once
// ahb_vision.h - SHADOW automatic-high-beam vision for the external C920 (nothing is switched by this code).
// Finds vehicle LIGHT SOURCES (headlights) near the horizon and counts street lamps above it, 5 times a second,
// and writes the result to /dev/c920_lights.json for the logger (tesla/tools/ahb_shadow.py decides what an automatic
// high beam WOULD do). Off unless /data/ahb_vision_on exists. Prototype + numbers: tesla/camerad/AHB_VISION.md.
#include <cstdint>

const int AHB_MAX_LIGHTS = 12;

struct AhbLight {
  int x, y;        // full-res pixel (upright frame)
  int area;        // full-res pixels of the saturated core
  float halo;      // share of the ring around the core that glows (>= 0.45 = a lamp, not a reflection)
};

struct AhbLights {
  int n;                         // light sources found in the band around the horizon (n may exceed AHB_MAX_LIGHTS)
  AhbLight l[AHB_MAX_LIGHTS];    // the first AHB_MAX_LIGHTS of them
  int lamps_above;               // saturated light sources clearly ABOVE the horizon: street lamps -> a lit road
  float band_luma;               // mean brightness (0-255) of the band: how dark the road ahead is
};

// Pure function (no I/O): rgb = w*h*3 bytes, RGB, upright (after camerad's flip). horizon = full-res row.
AhbLights detect_lights(const uint8_t *rgb, int w, int h, int horizon);

// Call once per captured frame after the flip. A cached flag check unless /data/ahb_vision_on exists; then it
// runs detect_lights() every AHB_EVERY_N_FRAMES frames and writes /dev/c920_lights.json atomically.
void ahb_vision_tick(const uint8_t *rgb, int w, int h);
