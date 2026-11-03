#ifndef LIBUVC_CONFIG_H
#define LIBUVC_CONFIG_H
/* libuvc v0.0.7 (github.com/libuvc/libuvc, commit 68d07a0), configured for the EON test build:
   no libjpeg (MJPEG frames are saved as .jpg files as they come from the camera) */
#define LIBUVC_VERSION_MAJOR 0
#define LIBUVC_VERSION_MINOR 0
#define LIBUVC_VERSION_PATCH 7
#define LIBUVC_VERSION_STR "0.0.7"
#define LIBUVC_VERSION_INT ((0 << 16) | (0 << 8) | (7))
#define LIBUVC_VERSION_GTE(major, minor, patch) (LIBUVC_VERSION_INT >= (((major) << 16) | ((minor) << 8) | (patch)))
/* LIBUVC_HAS_JPEG not defined */
#endif
