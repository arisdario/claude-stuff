#ifndef CAMERA_EXT_C920_H
#define CAMERA_EXT_C920_H
// External road camera for the EON: a Logitech C920 over USB (libuvc), used INSTEAD of the phone's rear camera. The phone's FRONT camera (driver monitoring) is untouched.
// Chosen with /data/camera_source: "external" -> C920 (only if it is plugged in AND streams at startup, otherwise the phone camera is used); anything else / no file -> the phone camera.
#include <stdint.h>
#include "camera_qcom.h"

#ifdef __cplusplus
extern "C" {
#endif

int ext_c920_wanted(void);                      // 1 if /data/camera_source (or env CAMERA_SOURCE) says "external"
int ext_c920_present(void);                     // 1 if a C920 (046d:082d) is on the USB bus
int ext_c920_selftest(int timeout_ms);          // opens it, streams, waits for a frame, closes: 1 = it works
void ext_c920_configure(CameraState *rear);     // gives the rear CameraState the geometry of a 1164x874 BGR picture
int ext_c920_start(CameraState *rear, VisionBuf *bufs);   // starts the capture thread (it dispatches frames into rear->camera_tb); 0 = ok
void ext_c920_stop(void);

// pure helper (also unit-tested off the EON): RGB source (src_w x src_h) -> BGR rows at dst_stride on a dst_w x dst_h canvas: the source is cropped at column x0 (dst_w columns) and placed from row y0
void ext_c920_pack_bgr(const uint8_t *rgb, int src_w, int src_h, uint8_t *dst, int dst_stride, int dst_w, int dst_h, int x0, int y0);

#ifdef __cplusplus
}
#endif
#endif
