/* gps_standalone_test.cc - talks to the EON phone's own internal GPS chip directly through
 * Android's GPS HAL (hardware/gps.h) - the exact same call this fork's selfdrive/sensord/gpsd.cc
 * already makes (hw_get_module/open/get_gps_interface/init/start, same GpsCallbacks shape), just
 * with every bit of cereal/capnp/pub-sub stripped out: no openpilot process, no other app, nothing
 * in between you and the chip. It prints raw NMEA sentences, a parsed one-line summary (fix quality,
 * satellites used, lat/lon from $GGA; satellites in view per constellation from $GSV), and Android's
 * own location_callback output (lat/lon/alt/speed/bearing/accuracy/timestamp) whenever it fires.
 *
 * IMPORTANT - the GPS HAL device is a SINGLETON: only one process can hold it open. openpilot's own
 * gpsd almost certainly already has it open as part of normal boot. Stop that first:
 *   ps | grep gpsd            # find its pid
 *   kill <pid>                # or: pkill -f _gpsd / whatever manager.py calls it
 * Run this, watch it, Ctrl+C to stop (cleanly calls stop()/cleanup() on the HAL), then let the
 * normal process restart (reboot, or however your manager.py respawns killed children).
 *
 * BUILD (on the EON itself, same include paths selfdrive/sensord/SConscript already uses for
 * gpsd.cc - substitute <OP_ROOT> for your actual checkout root, e.g. /data/openpilot or /data/cs_new):
 *   g++ -std=c++11 -o gps_standalone_test gps_standalone_test.cc \
 *     -I<OP_ROOT>/phonelibs/android_hardware_libhardware/include \
 *     -I<OP_ROOT>/phonelibs/android_system_core/include \
 *     -lhardware -lcutils -llog -lpthread
 *   ./gps_standalone_test
 *
 * NOTE: this file has NOT been compiled or run anywhere in this session - there is no Android
 * toolchain/libhardware headers available in the sandbox this was written in, unlike every other
 * native change made this project (sign_detect.cc etc.), which WAS built and run before shipping.
 * The HAL-opening code is copied verbatim from gpsd.cc's own proven-working gps_init(), so that part
 * should be solid; the NMEA parsing is new and untested. Please run it on-device and paste back
 * whatever the compiler or the program itself prints - especially any compile error - so this can be
 * fixed against real feedback instead of more guessing.
 */
#include <assert.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <pthread.h>

#include <hardware/gps.h>

namespace {

volatile sig_atomic_t do_exit = 0;
void set_do_exit(int sig) { do_exit = 1; }

const GpsInterface *gGpsInterface = NULL;
const AGpsInterface *gAGpsInterface = NULL;

pthread_t create_thread_callback(const char *name, void (*start)(void *), void *arg) {
  fprintf(stdout, "[hal] creating thread: %s\n", name);
  pthread_t thread;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_create(&thread, &attr, (void *(*)(void *))start, arg);
  return thread;
}

/* --- tiny, deliberately-forgiving NMEA field splitter: good enough for a diagnostic printout,
 * not a general NMEA library. Ignores checksum, doesn't choke on a short/odd line. --- */
int nmea_fields(const char *nmea, char out[32][16]) {
  int n = 0;
  const char *p = nmea;
  while (*p && *p != '*' && n < 32) {
    const char *comma = strchr(p, ',');
    size_t len = comma ? (size_t)(comma - p) : strlen(p);
    if (len >= sizeof(out[0])) len = sizeof(out[0]) - 1;
    memcpy(out[n], p, len);
    out[n][len] = '\0';
    n++;
    if (!comma) break;
    p = comma + 1;
  }
  return n;
}

void handle_gga(char f[32][16], int n) {
  // $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,fixQuality,numSatsUsed,hdop,alt,M,...
  if (n < 8) return;
  int fix_quality = atoi(f[6]);
  int sats_used = atoi(f[7]);
  fprintf(stdout, "[gga] utc=%s fixQuality=%d satsUsed=%d lat=%s%s lon=%s%s\n",
          f[1], fix_quality, sats_used, f[2], f[3], f[4], f[5]);
}

void handle_gsv(const char *talker, char f[32][16], int n) {
  // $??GSV,numMsgs,msgNum,satsInView,...
  if (n < 4) return;
  fprintf(stdout, "[gsv] %s satsInView=%s (msg %s/%s)\n", talker, f[3], f[2], f[1]);
}

void nmea_callback(GpsUtcTime timestamp, const char *nmea, int length) {
  (void)length;
  fprintf(stdout, "[nmea] ts=%lld %s", (long long)timestamp, nmea);
  if (nmea[strlen(nmea) - 1] != '\n') fprintf(stdout, "\n");

  char fields[32][16];
  int n = nmea_fields(nmea, fields);
  if (n < 1) return;
  const char *sentence = fields[0];
  size_t slen = strlen(sentence);
  if (slen >= 6 && strcmp(sentence + slen - 3, "GGA") == 0) {
    handle_gga(fields, n);
  } else if (slen >= 6 && strcmp(sentence + slen - 3, "GSV") == 0) {
    handle_gsv(sentence, fields, n);
  }
}

void location_callback(GpsLocation *location) {
  time_t secs = (time_t)(location->timestamp / 1000);
  struct tm tm_utc;
  gmtime_r(&secs, &tm_utc);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  fprintf(stdout,
          "[location] %s lat=%.6f lon=%.6f alt=%.1fm speed=%.1fm/s bearing=%.1fdeg accuracy=%.1fm flags=0x%x\n",
          buf, location->latitude, location->longitude, location->altitude, location->speed,
          location->bearing, location->accuracy, location->flags);
}

GpsCallbacks gps_callbacks = {
    sizeof(GpsCallbacks),
    location_callback,
    NULL,
    NULL,  // sv_status_cb left NULL, same as gpsd.cc - satellite counts come from parsing $GSV/$GGA
           // above instead, since that reuses only the already-proven nmea_callback mechanism
    nmea_callback,
    NULL,
    NULL,
    NULL,
    create_thread_callback,
};

void agps_status_cb(AGpsStatus *status) {
  switch (status->status) {
    case GPS_REQUEST_AGPS_DATA_CONN:
      fprintf(stdout, "[agps] data_conn_open requested (no network here - ignoring, GPS-only test)\n");
      break;
    case GPS_RELEASE_AGPS_DATA_CONN:
      fprintf(stdout, "[agps] data_conn_closed\n");
      break;
  }
}

AGpsCallbacks agps_callbacks = {
    agps_status_cb,
    create_thread_callback,
};

void gps_init() {
  fprintf(stdout, "[hal] opening GPS HAL...\n");
  hw_module_t *module = NULL;
  hw_get_module(GPS_HARDWARE_MODULE_ID, (hw_module_t const **)&module);
  if (!module) {
    fprintf(stderr, "[hal] hw_get_module failed - no GPS HAL on this device?\n");
    exit(1);
  }

  static hw_device_t *device = NULL;
  module->methods->open(module, GPS_HARDWARE_MODULE_ID, &device);
  if (!device) {
    fprintf(stderr, "[hal] module->open failed - is gpsd/sensord already holding the HAL open? "
                     "stop that process first (ps | grep gpsd)\n");
    exit(1);
  }

  gps_device_t *gps_device = (gps_device_t *)device;
  gGpsInterface = gps_device->get_gps_interface(gps_device);
  assert(gGpsInterface);

  gAGpsInterface = (const AGpsInterface *)gGpsInterface->get_extension(AGPS_INTERFACE);

  gGpsInterface->init(&gps_callbacks);
  if (gAGpsInterface) {
    gAGpsInterface->init(&agps_callbacks);
  }

  gGpsInterface->start();
  gGpsInterface->set_position_mode(GPS_POSITION_MODE_MS_BASED, GPS_POSITION_RECURRENCE_PERIODIC, 100, 0, 0);
  fprintf(stdout, "[hal] started - waiting for NMEA/location callbacks (ctrl+C to stop)...\n");
}

void gps_destroy() {
  fprintf(stdout, "[hal] stopping...\n");
  gGpsInterface->stop();
  gGpsInterface->cleanup();
}

}  // namespace

int main() {
  setpriority(PRIO_PROCESS, 0, -13);
  signal(SIGINT, (sighandler_t)set_do_exit);
  signal(SIGTERM, (sighandler_t)set_do_exit);

  gps_init();
  while (!do_exit) pause();
  gps_destroy();

  return 0;
}
