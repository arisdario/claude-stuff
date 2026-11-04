// g++ -std=c++11 -I../camerad/cameras test_camera_profile.cc -o /tmp/tcp && /tmp/tcp
#include <assert.h>
#include <map>
#include "camera_profile.h"
using namespace camprof;

struct Mock : Iface {
  int val[N_CTL] = {8, 250, 255, 128, 128, 128, 128, 0, 1, 3500, 2};
  int fail_set = -1, wrong_readback = -1, fail_get = -1;
  int sets = 0;
  bool get(int c, int *v) override { if (c == fail_get) return false; *v = val[c]; return true; }
  bool set(int c, int v) override { sets++; if (c == fail_set) return false; val[c] = (c == wrong_readback) ? v + 1 : v; return true; }
};

int main() {
  // parsing
  std::string w;
  Conf cf = parse_conf("# c\nday.power_line=1\nday.brightness = 140\nnight.ae_mode=1\nnight.exposure=300\nnight.gain=120\nnight.wb_auto=0\nnight.wb_kelvin=3800\nauto.night_luma=30\nbogus=1\nday.nope=3\n", &w);
  assert(cf.day.v[POWER_LINE] == 1 && cf.day.v[BRIGHTNESS] == 140 && cf.night.v[EXPOSURE] == 300 && cf.a.night_luma == 30);
  assert(cf.day.v[GAIN] == -1 && w.find("bogus") != std::string::npos && w.find("day.nope") != std::string::npos);
  // apply: only touched controls, clamped, exposure/gain skipped in auto exposure, kelvin skipped in auto WB
  { Mock m; std::string e, ch; Profile p; p.v[POWER_LINE] = 1; p.v[EXPOSURE] = 300; p.v[GAIN] = 10; p.v[WB_KELVIN] = 3000; p.v[BRIGHTNESS] = 999;
    assert(apply(m, p, &e, &ch)); assert(m.val[POWER_LINE] == 1 && m.val[BRIGHTNESS] == 255 && m.val[EXPOSURE] == 250 && m.val[GAIN] == 255 && m.val[WB_KELVIN] == 3500); }
  // manual exposure + manual WB are applied, exposure clamped to 500
  { Mock m; std::string e, ch; Profile p; p.v[AE_MODE] = 1; p.v[EXPOSURE] = 2000; p.v[GAIN] = 100; p.v[WB_AUTO] = 0; p.v[WB_KELVIN] = 3800;
    assert(apply(m, p, &e, &ch)); assert(m.val[AE_MODE] == 1 && m.val[EXPOSURE] == 500 && m.val[GAIN] == 100 && m.val[WB_AUTO] == 0 && m.val[WB_KELVIN] == 3800); }
  // failure -> everything changed so far is restored
  { Mock m; m.wrong_readback = SATURATION; Mock ref = m; std::string e, ch; Profile p; p.v[POWER_LINE] = 1; p.v[BRIGHTNESS] = 100; p.v[SATURATION] = 150;
    assert(!apply(m, p, &e, &ch)); assert(e.find("saturation") != std::string::npos);
    for (int i = 0; i < N_CTL; i++) assert(m.val[i] == ref.val[i] || i == SATURATION); }
  { Mock m; m.fail_set = BRIGHTNESS; Mock ref = m; std::string e, ch; Profile p; p.v[POWER_LINE] = 1; p.v[BRIGHTNESS] = 100;
    assert(!apply(m, p, &e, &ch)); assert(m.val[POWER_LINE] == ref.val[POWER_LINE]); }
  { Mock m; m.fail_get = CONTRAST; std::string e, ch; Profile p; p.v[POWER_LINE] = 1; p.v[CONTRAST] = 100;
    assert(!apply(m, p, &e, &ch)); assert(m.val[POWER_LINE] == 2); }
  // controller: OFF does nothing at all
  { Mock m; Controller c; Measure me; me.frames_ok = true; me.stream_age_s = 100; Conf cf2; cf2.day.v[POWER_LINE] = 1;
    for (int t = 0; t < 50; t++) c.tick(t, M_OFF, false, cf2, me, m); assert(m.sets == 0); }
  // controller: manual DAY applies, waits for a stable stream, then OFF restores the original values
  { Mock m; Controller c; Measure me; me.frames_ok = false; me.stream_age_s = 1; Conf cf2; cf2.day.v[POWER_LINE] = 1; cf2.day.v[BRIGHTNESS] = 140;
    c.tick(10, M_DAY, true, cf2, me, m); assert(m.sets == 0);                 // stream not stable yet -> nothing touched
    me.frames_ok = true; me.stream_age_s = 20; c.tick(11, M_DAY, false, cf2, me, m);
    assert(m.val[POWER_LINE] == 1 && m.val[BRIGHTNESS] == 140 && c.active == 0);
    c.tick(30, M_OFF, false, cf2, me, m); assert(m.val[POWER_LINE] == 2 && m.val[BRIGHTNESS] == 128 && c.active == -1);
    assert(!c.log.empty()); }
  // controller: failing apply is logged, restored, and does not become 'active'
  { Mock m; m.wrong_readback = BRIGHTNESS; Controller c; Measure me; me.frames_ok = true; me.stream_age_s = 20; Conf cf2; cf2.day.v[POWER_LINE] = 1; cf2.day.v[BRIGHTNESS] = 140;
    c.tick(10, M_DAY, true, cf2, me, m); assert(c.active == -1 && m.val[POWER_LINE] == 2 && c.last_error.find("brightness") != std::string::npos); }
  // AUTO: hysteresis. Dusk flicker must not switch; sustained dark switches once; bright for long enough switches back; dwell respected.
  { Mock m; Controller c; Measure me; me.frames_ok = true; me.stream_age_s = 100; Conf cf2; cf2.day.v[POWER_LINE] = 1; cf2.night.v[POWER_LINE] = 1; cf2.night.v[SATURATION] = 150;
    int switches = 0, last = -1;
    auto step = [&](double t, float luma, int exp, int gain) { me.luma = luma; me.exposure = exp; me.gain = gain; c.tick(t, M_AUTO, false, cf2, me, m); if (c.active != last) { switches++; last = c.active; } };
    for (int t = 0; t < 60; t++) step(t, 120, 60, 20);                              // bright day
    int day_switches = switches;
    for (int t = 60; t < 400; t++) step(t, (t % 20 < 10) ? 25 : 70, (t % 20 < 10) ? 450 : 200, (t % 20 < 10) ? 200 : 80);   // twilight: dark 10 s / lighter 10 s, never 15 s continuously
    assert(switches == day_switches);                                                // no switch at all
    for (int t = 400; t < 440; t++) step(t, 20, 480, 230);                          // really dark for 40 s
    assert(c.active == 1 && m.val[SATURATION] == 150);
    int s1 = switches;
    for (int t = 440; t < 500; t++) step(t, 130, 80, 30);                           // sudden brightness (tunnel exit): confirm 45 s AND dwell 90 s
    assert(c.active == 1);                                                            // 60 s bright < 90 s dwell: still NIGHT
    for (int t = 500; t < 560; t++) step(t, 130, 80, 30);
    assert(c.active == 0 && switches == s1 + 1);                                       // after dwell + confirm: back to DAY, exactly one more switch
    (void)s1; }
  puts("camera_profile: all tests passed");
  return 0;
}
