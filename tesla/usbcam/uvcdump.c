/* uvcdump - READ-ONLY report of every UVC control the Logitech C920 answers, with current/min/max/step/default values.
 *
 *   ./uvcdump            all controls, named, decoded (human readable)
 *   ./uvcdump --json     the same as one JSON object (for the analysis scripts)
 *
 * Only GET_INFO / GET_LEN / GET_CUR / GET_MIN / GET_MAX / GET_RES / GET_DEF are sent. Nothing is written to the camera.
 * One program can open the camera at a time: stop openpilot (camerad) first, or let camerad write /data/camera_controls.json itself.
 * Standard control selectors from the USB Video Class 1.1 spec (camera terminal and processing unit); the Logitech
 * extension units are scanned too (selector 1..30) because Logitech keeps vendor features there (LED, "RightLight"-related, etc).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libuvc/libuvc.h"

typedef struct { int sel; const char *name; int len; int is_signed; const char *note; } ctl_t;

static const ctl_t CT[] = {   /* camera terminal */
  {1, "scanning_mode", 1, 0, ""}, {2, "auto_exposure_mode", 1, 0, "bitmask: 1 manual, 2 auto, 4 shutter-priority, 8 aperture-priority (C920 'auto' = 8)"},
  {3, "auto_exposure_priority", 1, 0, "1 = camera may DROP the frame rate in low light (Logitech RightLight); 0 = constant frame rate"},
  {4, "exposure_time_absolute", 4, 0, "units of 100 us"}, {5, "exposure_time_relative", 1, 1, ""}, {6, "focus_absolute", 2, 0, ""},
  {7, "focus_relative", 2, 1, ""}, {8, "focus_auto", 1, 0, ""}, {9, "iris_absolute", 2, 0, ""}, {0x0b, "zoom_absolute", 2, 0, ""},
  {0x0d, "pan_tilt_absolute", 8, 1, ""}, {0x11, "privacy", 1, 0, ""}};
static const ctl_t PU[] = {   /* processing unit */
  {1, "backlight_compensation", 2, 0, ""}, {2, "brightness", 2, 1, ""}, {3, "contrast", 2, 0, ""}, {4, "gain", 2, 0, "analog/digital sensor gain, camera units"},
  {5, "power_line_frequency", 1, 0, "0 off, 1 50 Hz, 2 60 Hz (50 Hz in Romania)"}, {6, "hue", 2, 1, ""}, {7, "saturation", 2, 0, ""},
  {8, "sharpness", 2, 0, ""}, {9, "gamma", 2, 0, ""}, {0x0a, "white_balance_temperature", 2, 0, "kelvin"}, {0x0b, "white_balance_temperature_auto", 1, 0, ""},
  {0x0c, "white_balance_component", 4, 0, ""}, {0x0d, "white_balance_component_auto", 1, 0, ""}, {0x0e, "digital_multiplier", 2, 0, ""},
  {0x10, "hue_auto", 1, 0, ""}, {0x13, "contrast_auto", 1, 0, ""}};

static long long dec(const uint8_t *b, int len, int sgn) {
  long long v = 0; for (int i = len - 1; i >= 0; i--) v = (v << 8) | b[i];
  if (sgn && len < 8 && (v >> (8 * len - 1)) & 1) v -= 1LL << (8 * len);
  return v;
}

static int first = 1, json = 0;

static void one(uvc_device_handle_t *devh, const char *unit_name, int unit, const ctl_t *c) {
  uint8_t info = 0; int ri = uvc_get_ctrl(devh, unit, c->sel, &info, 1, UVC_GET_INFO);
  if (ri < 0) return;                                   /* the camera does not implement this control */
  int l = uvc_get_ctrl_len(devh, unit, c->sel); if (l <= 0 || l > 8) l = c->len;
  static const struct { enum uvc_req_code c; const char *n; } R[] = {{UVC_GET_CUR, "cur"}, {UVC_GET_MIN, "min"}, {UVC_GET_MAX, "max"}, {UVC_GET_RES, "step"}, {UVC_GET_DEF, "default"}};
  long long v[5]; int ok[5];
  for (int k = 0; k < 5; k++) { uint8_t b[8] = {0}; ok[k] = uvc_get_ctrl(devh, unit, c->sel, b, l, R[k].c) >= 0; v[k] = ok[k] ? dec(b, l, c->is_signed) : 0; }
  if (json) {
    printf("%s\"%s.%s\":{\"unit\":%d,\"selector\":%d,\"len\":%d,\"can_get\":%d,\"can_set\":%d", first ? "" : ",", unit_name, c->name, unit, c->sel, l, info & 1, (info >> 1) & 1);
    for (int k = 0; k < 5; k++) if (ok[k]) printf(",\"%s\":%lld", R[k].n, v[k]);
    printf("}\n"); first = 0; return;
  }
  printf("%-34s unit %2d sel 0x%02x len %d %s%s |", c->name, unit, c->sel, l, (info & 1) ? "GET" : "", (info & 2) ? "+SET" : " (read only)");
  for (int k = 0; k < 5; k++) { if (ok[k]) printf(" %s=%lld", R[k].n, v[k]); else printf(" %s=n/a", R[k].n); }
  printf("%s%s\n", c->note[0] ? "   # " : "", c->note);
}

static void guid_str(const uint8_t *g, char *out) {
  sprintf(out, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7], g[8], g[9], g[10], g[11], g[12], g[13], g[14], g[15]);
}

int main(int argc, char **argv) {
  json = argc > 1 && !strcmp(argv[1], "--json");
  uvc_context_t *ctx; uvc_device_t *dev; uvc_device_handle_t *devh;
  if (uvc_init(&ctx, NULL) < 0) { fprintf(stderr, "uvc_init failed\n"); return 1; }
  if (uvc_find_device(ctx, &dev, 0x046d, 0x082d, NULL) < 0) { fprintf(stderr, "no C920 (046d:082d) found\n"); return 1; }
  if (uvc_open(dev, &devh) < 0) { fprintf(stderr, "cannot open the camera - is openpilot/camerad running? stop it first (tmux kill-session -t comma)\n"); return 1; }
  const uvc_input_terminal_t *ct = uvc_get_camera_terminal(devh);
  const uvc_processing_unit_t *pu = uvc_get_processing_units(devh);
  if (json) printf("{\n");
  else printf("C920 UVC controls (read-only). camera terminal id %d, processing unit id %d\n\n", ct ? ct->bTerminalID : -1, pu ? pu->bUnitID : -1);
  if (ct) { if (!json) printf("== camera terminal\n"); for (unsigned i = 0; i < sizeof(CT) / sizeof(CT[0]); i++) one(devh, "ct", ct->bTerminalID, &CT[i]); }
  if (pu) { if (!json) printf("\n== processing unit\n"); for (unsigned i = 0; i < sizeof(PU) / sizeof(PU[0]); i++) one(devh, "pu", pu->bUnitID, &PU[i]); }
  for (const uvc_extension_unit_t *u = uvc_get_extension_units(devh); u; u = u->next) {
    char gs[40]; guid_str(u->guidExtensionCode, gs);
    if (!json) printf("\n== extension unit %d guid %s\n", u->bUnitID, gs);
    for (int s = 1; s <= 30; s++) {
      ctl_t c = {s, "xu", 1, 0, ""}; char nm[24]; sprintf(nm, "xu%d_sel%d", u->bUnitID, s); c.name = nm;
      uint8_t info = 0; if (uvc_get_ctrl(devh, u->bUnitID, s, &info, 1, UVC_GET_INFO) < 0) continue;
      int l = uvc_get_ctrl_len(devh, u->bUnitID, s); if (l <= 0 || l > 8) continue; c.len = l;
      one(devh, "xu", u->bUnitID, &c);
    }
  }
  if (json) printf("}\n");
  uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx);
  return 0;
}
