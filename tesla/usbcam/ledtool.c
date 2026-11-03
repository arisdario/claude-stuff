/* ledtool - read / set the Logitech "LED1 Mode" through its UVC extension unit, using libuvc.
 *
 * Definition taken from Logitech's own uvcdynctrl file logitech.xml (revision 0.32), control "peripheral_led_control_v3":
 *   extension unit GUID  FFE52D21-8030-4e2c-82D9-F587D00540BD  (UVC_GUID_LOGITECH_PERIPHERAL)
 *                        = 212de5ff-3080-2c4e-82d9-f587d00540bd in USB byte order (what this EON's C920 reports as unit 11)
 *   selector 0x09 (XU_PERIPHERALCONTROL_LED), size 5 bytes, requests SET_CUR GET_CUR GET_MIN GET_MAX GET_RES GET_DEF
 *   "LED1 Mode"      = bits 8..9  (low 2 bits of byte 1): 0 Off, 1 On, 2 Blinking, 3 Auto
 *   "LED1 Frequency" = bits 24..31 (byte 3)
 * Selector 0x0A on that unit is PERIPHERAL_STATUS (2 bytes, read only) - NOT the LED.
 * NOTE: the file only says the control exists for Logitech devices in general. Nothing in it promises that a C920 implements it,
 * and an earlier read-only scan of this camera's unit 11 got no answer to GET_INFO/GET_LEN on selectors 1..30. If the camera does not
 * answer, this tool refuses and touches nothing.
 *
 *   ./ledtool scan     READ ONLY: every extension unit, selectors 1..30: GET_INFO, length, current value
 *   ./ledtool probe    READ ONLY: the LED control (unit with the GUID above, selector 9): INFO, LEN, CUR, MIN, MAX, RES, DEF
 *   ./ledtool set <0-3>  write the LED mode (0 = off). Only if ALL hold: unit found by GUID; GET_INFO says the control can be read and SET;
 *                        GET_LEN is exactly 5; GET_CUR (5 bytes) can be read. Then it changes ONLY the 2 mode bits, writes the same 5 bytes
 *                        back with SET_CUR, reads again and reports VERIFIED only if the mode bits are the new value and the other 38 bits are unchanged.
 *
 * The camera forgets the setting when it loses power. Stop openpilot (camerad) first: only one program can open the camera.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libuvc/libuvc.h"

#define LED_GUID "212de5ff-3080-2c4e-82d9-f587d00540bd"
#define LED_SELECTOR 9
#define LED_LEN 5
#define MODE_BYTE 1
#define MODE_MASK 0x03

static void guid_str(const uint8_t *g, char *out) {
  sprintf(out, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7], g[8], g[9], g[10], g[11], g[12], g[13], g[14], g[15]);
}

static void show(uvc_device_handle_t *devh, int unit, int sel, int all_requests) {
  uint8_t info = 0; int ri = uvc_get_ctrl(devh, unit, sel, &info, 1, UVC_GET_INFO);   /* bit0 = can GET, bit1 = can SET */
  int l = uvc_get_ctrl_len(devh, unit, sel);
  if (ri < 0 && l <= 0 && !all_requests) return;
  printf("  selector %3d  info %s0x%02x%s  len %d", sel, ri < 0 ? "(err " : "", ri < 0 ? (unsigned)(-ri) : info, ri < 0 ? ")" : (info & 2 ? " (can SET)" : ""), l);
  if (l > 0 && l <= 16) {
    static const struct { enum uvc_req_code c; const char *n; } R[] = {{UVC_GET_CUR, "cur"}, {UVC_GET_MIN, "min"}, {UVC_GET_MAX, "max"}, {UVC_GET_RES, "res"}, {UVC_GET_DEF, "def"}};
    for (int k = 0; k < (all_requests ? 5 : 1); k++) {
      uint8_t buf[16] = {0}; int r = uvc_get_ctrl(devh, unit, sel, buf, l, R[k].c);
      printf("  %s ", R[k].n); if (r < 0) printf("(err %d)", r); else for (int i = 0; i < l; i++) printf("%02x", buf[i]);
    }
  }
  printf("\n");
}

int main(int argc, char **argv) {
  const char *mode = argc >= 2 ? argv[1] : "scan";
  int do_set = !strcmp(mode, "set"), value = -1;
  if (do_set) {
    if (argc < 3) { fprintf(stderr, "usage: ledtool set <0..3>   (0 off, 1 on, 2 blink, 3 auto)\n"); return 2; }
    value = atoi(argv[2]);
    if (value < 0 || value > 3) { fprintf(stderr, "value must be 0..3\n"); return 2; }
  } else if (strcmp(mode, "scan") && strcmp(mode, "probe")) { fprintf(stderr, "usage: ledtool [scan|probe|set <0..3>]\n"); return 2; }
  uvc_context_t *ctx; uvc_device_t *dev; uvc_device_handle_t *devh; int rc = 0;
  if (uvc_init(&ctx, NULL) < 0) { fprintf(stderr, "uvc_init failed\n"); return 1; }
  if (uvc_find_device(ctx, &dev, 0x046d, 0x082d, NULL) < 0) { fprintf(stderr, "no C920 (046d:082d) found\n"); return 1; }
  if (uvc_open(dev, &devh) < 0) { fprintf(stderr, "cannot open the camera - is openpilot/camerad running? stop it first\n"); return 1; }
  int unit = -1; char gs[40];
  for (const uvc_extension_unit_t *u = uvc_get_extension_units(devh); u; u = u->next) {
    guid_str(u->guidExtensionCode, gs); int c = !strcmp(gs, LED_GUID);
    printf("extension unit id %u  guid %s%s\n", u->bUnitID, gs, c ? "   <- Logitech peripheral unit (LED control lives here)" : "");
    if (c && unit < 0) unit = u->bUnitID;
  }
  if (!strcmp(mode, "scan")) {
    for (const uvc_extension_unit_t *u = uvc_get_extension_units(devh); u; u = u->next) { printf("--- unit %u\n", u->bUnitID); for (int s = 1; s <= 30; s++) show(devh, u->bUnitID, s, 0); }
  } else if (unit < 0) {
    printf("no extension unit with GUID %s: not touching anything.\n", LED_GUID); rc = 3;
  } else if (!strcmp(mode, "probe")) {
    printf("--- unit %d, selector %d (LED control, 5 bytes expected):\n", unit, LED_SELECTOR);
    show(devh, unit, LED_SELECTOR, 1);
  } else {
    uint8_t info = 0; int ri = uvc_get_ctrl(devh, unit, LED_SELECTOR, &info, 1, UVC_GET_INFO), len = uvc_get_ctrl_len(devh, unit, LED_SELECTOR);
    uint8_t cur[LED_LEN] = {0}; int rcur = uvc_get_ctrl(devh, unit, LED_SELECTOR, cur, LED_LEN, UVC_GET_CUR);
    printf("unit %d selector %d: GET_INFO %s0x%02x, GET_LEN %d, GET_CUR %s", unit, LED_SELECTOR, ri < 0 ? "(error) " : "", ri < 0 ? 0 : info, len, rcur < 0 ? "(unreadable)" : "");
    if (rcur >= 0) for (int i = 0; i < LED_LEN; i++) printf("%02x", cur[i]);
    printf("\n");
    if (ri < 0 || (info & 3) != 3) { printf("REFUSED: the camera does not say this control exists and can be both read and SET. Nothing was written.\n"); rc = 4; }
    else if (len != LED_LEN)       { printf("REFUSED: GET_LEN is %d, expected %d. Nothing was written.\n", len, LED_LEN); rc = 4; }
    else if (rcur < 0)             { printf("REFUSED: the current value cannot be read, so the read-back check is impossible. Nothing was written.\n"); rc = 4; }
    else {
      printf("current LED mode = %d (0 off, 1 on, 2 blink, 3 auto); frequency byte = %d\n", cur[MODE_BYTE] & MODE_MASK, cur[3]);
      uint8_t nw[LED_LEN]; memcpy(nw, cur, LED_LEN); nw[MODE_BYTE] = (uint8_t)((cur[MODE_BYTE] & ~MODE_MASK) | value);
      int w = uvc_set_ctrl(devh, unit, LED_SELECTOR, nw, LED_LEN);
      uint8_t back[LED_LEN] = {0}; int rb = uvc_get_ctrl(devh, unit, LED_SELECTOR, back, LED_LEN, UVC_GET_CUR);
      int ok = (w >= 0 && rb >= 0 && (back[MODE_BYTE] & MODE_MASK) == value);
      for (int i = 0; i < LED_LEN; i++) if (i != MODE_BYTE && back[i] != cur[i]) ok = 0;
      if (ok && (back[MODE_BYTE] & ~MODE_MASK) != (cur[MODE_BYTE] & ~MODE_MASK)) ok = 0;
      printf("SET_CUR result %d; read-back %02x%02x%02x%02x%02x (result %d) -> LED mode %d -> %s\n", w, back[0], back[1], back[2], back[3], back[4], rb, back[MODE_BYTE] & MODE_MASK, ok ? "VERIFIED" : "NOT VERIFIED");
      rc = ok ? 0 : 5;
    }
  }
  uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx);
  return rc;
}
