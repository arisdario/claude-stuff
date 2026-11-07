#pragma once
// sign_detect_io.h - the only piece that touches anything outside sign_detect.cc's pure function:
// gates on a flag file, rate-limits (CV on every frame would waste CPU the model/camerad need), and
// writes the result atomically. See SIGN_DETECT.md for the full data flow and the opt-in build gate.
#include <cstdint>

// Call once per captured frame (e.g. from on_frame() in camera_ext_c920.cc), after any flip so the
// orientation matches what the model sees. rgb must be w*h*3 bytes, RGB order (matches camera_ext_c920's
// own decode_jpeg output). Internally a near-zero-cost no-op unless /data/sign_detect_on exists; even
// then, only actually runs detect_signs() every SIGN_DETECT_EVERY_N_FRAMES frames.
void sign_detect_tick(const uint8_t *rgb, int w, int h);
