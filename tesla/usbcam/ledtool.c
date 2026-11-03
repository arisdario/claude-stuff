/* ledtool - look at / set the status LED of a Logitech C920 through its Logitech extension unit, using libuvc.
 *
 *   ./ledtool                list the camera's extension units and read the "LED1 Mode" control (READ ONLY)
 *   ./ledtool scan           READ ONLY: dump every extension-unit control's length and current value (to find the LED one)
 *   ./ledtool set 0          write LED1 Mode = 0 (off).  1 = on, 2 = blink, 3 = auto (what the camera does by default)
 *
 * What it does and does not touch: it only ever reads/writes ONE control (selector 10 of the Logitech extension unit,
 * the same one uvcdynctrl calls "LED1 Mode"), and it refuses to write unless the camera reports that control is 1 byte long.
 * The numbers (GUID 82066163-7050-ab49-b8cc-b3855e8d221e, selector 10, 1 byte, 0=off 1=on 2=blink 3=auto) come from
 * uvcdynctrl's logitech.xml as I remember it - NOT verified on a C920 here. Run the read-only mode first and look at the output.
 * The camera forgets the setting when it loses power. Stop openpilot (camerad) first: only one program can open the camera.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "libuvc/libuvc.h"

#define LED_SELECTOR 10
static const uint8_t GUID[16] = {0x82,0x06,0x61,0x63,0x70,0x50,0xab,0x49,0xb8,0xcc,0xb3,0x85,0x5e,0x8d,0x22,0x1e};   /* as written in the xml */

static int is_logitech_xu(const uint8_t *g) {                  /* the USB descriptor stores the first three groups little-endian */
  uint8_t le[16]; memcpy(le, GUID, 16);
  le[0] = GUID[3]; le[1] = GUID[2]; le[2] = GUID[1]; le[3] = GUID[0]; le[4] = GUID[5]; le[5] = GUID[4]; le[6] = GUID[7]; le[7] = GUID[6];
  return !memcmp(g, GUID, 16) || !memcmp(g, le, 16);
}

int main(int argc, char **argv) {
  int want_set = (argc >= 3 && !strcmp(argv[1], "set")), value = want_set ? atoi(argv[2]) : -1;
  if (want_set && (value < 0 || value > 3)) { fprintf(stderr, "value must be 0..3\n"); return 2; }
  uvc_context_t *ctx; uvc_device_t *dev; uvc_device_handle_t *devh;
  if (uvc_init(&ctx, NULL) < 0) { fprintf(stderr, "uvc_init failed\n"); return 1; }
  if (uvc_find_device(ctx, &dev, 0x046d, 0x082d, NULL) < 0) { fprintf(stderr, "no C920 (046d:082d) found\n"); return 1; }
  if (uvc_open(dev, &devh) < 0) { fprintf(stderr, "cannot open the camera - is openpilot/camerad running? stop it first\n"); return 1; }
  const uvc_extension_unit_t *u = uvc_get_extension_units(devh);
  int found_unit = -1;
  for (; u; u = u->next) {
    printf("extension unit id %u  guid ", u->bUnitID);
    for (int i = 0; i < 16; i++) printf("%02x%s", u->guidExtensionCode[i], (i == 3 || i == 5 || i == 7 || i == 9) ? "-" : "");
    int lg = is_logitech_xu(u->guidExtensionCode); printf("%s\n", lg ? "   <- Logitech extension unit" : "");
    if (lg) found_unit = u->bUnitID;
  }
  if (argc >= 2 && !strcmp(argv[1], "scan")) {      /* READ ONLY: for every extension unit and selector 1..30 print the control length and its current value */
    for (u = uvc_get_extension_units(devh); u; u = u->next) {
      printf("--- unit %u\n", u->bUnitID);
      for (int sel = 1; sel <= 30; sel++) {
        int l = uvc_get_ctrl_len(devh, u->bUnitID, sel);
        if (l <= 0 || l > 16) continue;
        uint8_t buf[16] = {0}; int r = uvc_get_ctrl(devh, u->bUnitID, sel, buf, l, UVC_GET_CUR);
        printf("  selector %2d  len %2d  cur ", sel, l);
        if (r < 0) printf("(read error %d)", r); else for (int i = 0; i < l; i++) printf("%02x ", buf[i]);
        printf("\n");
      }
    }
    goto out;
  }
  if (found_unit < 0) { printf("no Logitech extension unit with the expected GUID: not touching anything.\n"); goto out; }
  int len = uvc_get_ctrl_len(devh, found_unit, LED_SELECTOR);
  printf("selector %d length reported by the camera: %d\n", LED_SELECTOR, len);
  if (len != 1) { printf("not a 1-byte control, so this is not the LED control I expect: not touching anything.\n"); goto out; }
  uint8_t cur = 255; int r = uvc_get_ctrl(devh, found_unit, LED_SELECTOR, &cur, 1, UVC_GET_CUR);
  printf("LED1 Mode now: %d (read result %d)   0=off 1=on 2=blink 3=auto\n", cur, r);
  if (want_set) {
    uint8_t v = (uint8_t)value; r = uvc_set_ctrl(devh, found_unit, LED_SELECTOR, &v, 1);
    printf("set LED1 Mode = %d -> result %d\n", value, r);
    cur = 255; uvc_get_ctrl(devh, found_unit, LED_SELECTOR, &cur, 1, UVC_GET_CUR); printf("read back: %d\n", cur);
  }
out:
  uvc_close(devh); uvc_unref_device(dev); uvc_exit(ctx);
  return 0;
}
