// DAY / NIGHT image profiles for the C920 road camera: pure logic, no libuvc, no openpilot headers (unit-tested on a PC).
//
// Rules (from the project requirements):
//   * resolution, frame rate, crop and rotation are never touched; auto-exposure PRIORITY stays 0 (constant 20 fps) - it is not even a profile control
//   * mode "off" (the default, file missing) = camerad behaves exactly as before; nothing here runs
//   * a profile value of -1 means "do not touch this control"
//   * every change is a transaction: read the old value, set, read back; any mismatch or error restores everything changed so far
//   * leaving a profile mode (-> off) restores the values the camera had before the first profile was applied
//   * AUTO switching has hysteresis: confirm times, a minimum dwell, and different thresholds for going to NIGHT and back to DAY
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

namespace camprof {

enum Ctl { AE_MODE, EXPOSURE, GAIN, BRIGHTNESS, CONTRAST, SATURATION, SHARPNESS, BACKLIGHT, WB_AUTO, WB_KELVIN, POWER_LINE, N_CTL };
static const char *const NAMES[N_CTL] = {"ae_mode", "exposure", "gain", "brightness", "contrast", "saturation", "sharpness", "backlight", "wb_auto", "wb_kelvin", "power_line"};
// hard limits, read from the user's C920 (uvcdump): exposure 3..2047 but only <= 500 (50 ms) is usable at a constant 20 fps
static const int LO[N_CTL] = {1, 3, 0, 0, 0, 0, 0, 0, 0, 2000, 1};
static const int HI[N_CTL] = {8, 500, 255, 255, 255, 255, 255, 1, 1, 6500, 2};
// application order: mode first, exposure/gain only matter in manual exposure (ae_mode 1), kelvin only with manual white balance (wb_auto 0)
static const int ORDER[N_CTL] = {POWER_LINE, AE_MODE, EXPOSURE, GAIN, WB_AUTO, WB_KELVIN, BRIGHTNESS, CONTRAST, SATURATION, SHARPNESS, BACKLIGHT};

struct Profile {
  int v[N_CTL];
  Profile() { for (int i = 0; i < N_CTL; i++) v[i] = -1; }
};

struct AutoCfg {
  // evidence for NIGHT: the camera's own auto exposure is at its limits, or the picture is dark
  int night_exposure = 400;     // 100 us units (400 = 40 ms of the 50 ms frame)
  int night_gain = 150;
  float night_luma = 35;        // mean luma of the road region, 0..255
  // evidence for DAY: bright picture, or (auto exposure running) short exposure and low gain
  float day_luma = 95;
  int day_exposure = 150;
  int day_gain = 60;
  int night_confirm_s = 15;
  int day_confirm_s = 45;       // slower to leave NIGHT than to enter it: no flapping at twilight
  int min_dwell_s = 90;
};

struct Conf {
  Profile day, night;
  AutoCfg a;
};

// "day.brightness=128" / "night.gain=100" / "auto.night_luma=35" lines; '#' comments; unknown keys are ignored (reported in `warn`)
inline bool set_key(Conf &c, const std::string &key, double val) {
  size_t dot = key.find('.');
  if (dot == std::string::npos) return false;
  std::string sec = key.substr(0, dot), k = key.substr(dot + 1);
  if (sec == "day" || sec == "night") {
    Profile &p = sec == "day" ? c.day : c.night;
    for (int i = 0; i < N_CTL; i++) if (k == NAMES[i]) { p.v[i] = (int)val; return true; }
    return false;
  }
  if (sec == "auto") {
    AutoCfg &a = c.a;
    if (k == "night_exposure") a.night_exposure = (int)val; else if (k == "night_gain") a.night_gain = (int)val;
    else if (k == "night_luma") a.night_luma = (float)val; else if (k == "day_luma") a.day_luma = (float)val;
    else if (k == "day_exposure") a.day_exposure = (int)val; else if (k == "day_gain") a.day_gain = (int)val;
    else if (k == "night_confirm_s") a.night_confirm_s = (int)val; else if (k == "day_confirm_s") a.day_confirm_s = (int)val;
    else if (k == "min_dwell_s") a.min_dwell_s = (int)val; else return false;
    return true;
  }
  return false;
}

inline Conf parse_conf(const char *text, std::string *warn = NULL) {
  Conf c;
  const char *p = text;
  while (p && *p) {
    const char *e = strchr(p, '\n');
    std::string line = e ? std::string(p, e - p) : std::string(p);
    p = e ? e + 1 : NULL;
    size_t h = line.find('#');
    if (h != std::string::npos) line = line.substr(0, h);
    size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    std::string k = line.substr(0, eq), v = line.substr(eq + 1);
    while (!k.empty() && (k.back() == ' ' || k.back() == '\t')) k.pop_back();
    while (!k.empty() && (k[0] == ' ' || k[0] == '\t')) k.erase(0, 1);
    char *end = NULL;
    double d = strtod(v.c_str(), &end);
    if (end == v.c_str() || !set_key(c, k, d)) { if (warn) *warn += "ignored: " + line + "\n"; }
  }
  return c;
}

// the camera, as seen by the logic. Implemented with libuvc in camera_ext_c920.cc and with a mock in the tests.
struct Iface {
  virtual ~Iface() {}
  virtual bool get(int ctl, int *v) = 0;
  virtual bool set(int ctl, int v) = 0;
};

inline int clampv(int ctl, int v) { return v < LO[ctl] ? LO[ctl] : (v > HI[ctl] ? HI[ctl] : v); }

// Applies `p`. Returns true if everything requested was set and read back identically. On any failure every control changed so far is put back
// (best effort) and `err` says which one failed. `changed` receives "name old->new" for the log.
inline bool apply(Iface &c, const Profile &p, std::string *err, std::string *changed) {
  int old[N_CTL];
  bool did[N_CTL];
  for (int i = 0; i < N_CTL; i++) did[i] = false;
  int ae = p.v[AE_MODE];
  int wba = p.v[WB_AUTO];
  for (int oi = 0; oi < N_CTL; oi++) {
    int i = ORDER[oi];
    if (p.v[i] < 0) continue;
    if ((i == EXPOSURE || i == GAIN) && ae != 1) continue;            // the camera ignores them in auto exposure
    if (i == WB_KELVIN && wba != 0) continue;                          // ... and the colour temperature in auto white balance
    int want = clampv(i, p.v[i]), cur = 0;
    bool ok = c.get(i, &cur);
    if (ok) old[i] = cur;
    if (!ok) { if (err) *err = std::string("cannot read ") + NAMES[i]; goto fail; }
    if (cur == want) continue;
    if (!c.set(i, want)) { if (err) *err = std::string("cannot set ") + NAMES[i]; did[i] = true; goto fail; }
    did[i] = true;
    int back = 0;
    if (!c.get(i, &back) || back != want) { if (err) *err = std::string("read-back mismatch on ") + NAMES[i]; goto fail; }
    if (changed) { char b[64]; snprintf(b, sizeof b, "%s%s %d->%d", changed->empty() ? "" : ", ", NAMES[i], cur, want); *changed += b; }
  }
  return true;
fail:
  for (int oi = N_CTL - 1; oi >= 0; oi--) { int i = ORDER[oi]; if (did[i]) c.set(i, old[i]); }
  return false;
}

struct Measure {
  float luma = -1;       // mean luma of the road region, -1 = not available
  int exposure = -1;     // camera's current exposure (100 us), -1 = not readable
  int gain = -1;
  bool frames_ok = false;  // frames are flowing right now
  double stream_age_s = 0; // seconds since streaming started
};

enum Mode { M_OFF, M_DAY, M_NIGHT, M_AUTO };
inline Mode parse_mode(const char *s) {
  if (!s) return M_OFF;
  while (*s == ' ' || *s == '\n' || *s == '\t') s++;
  if (!strncmp(s, "day", 3)) return M_DAY;
  if (!strncmp(s, "night", 5)) return M_NIGHT;
  if (!strncmp(s, "auto", 4)) return M_AUTO;
  return M_OFF;
}
inline const char *mode_name(Mode m) { return m == M_DAY ? "day" : m == M_NIGHT ? "night" : m == M_AUTO ? "auto" : "off"; }

class Controller {
 public:
  Mode mode = M_OFF;
  int active = -1;                 // -1 none, 0 day, 1 night
  std::string last_reason, last_error;
  double t_applied = -1e9, t_switch = -1e9, night_ev = -1, day_ev = -1;
  bool have_orig = false;
  int orig[N_CTL];
  int fails = 0;
  int apply_count = 0;             // number of apply attempts so far (the web page waits for this to change after "Apply")
  bool apply_ok = true;            // result of the last attempt
  std::vector<std::string> log;     // lines for the caller to print / append to /data/camera_profile.log

  // "Restore defaults": the camera's own factory values (uvcdump defaults) with 50 Hz mains. Applied once, as a transaction; exposure/gain stay with the camera's auto exposure.
  static Profile factory_profile() {
    Profile p;
    p.v[AE_MODE] = 8; p.v[BRIGHTNESS] = 128; p.v[CONTRAST] = 128; p.v[SATURATION] = 128; p.v[SHARPNESS] = 128;
    p.v[BACKLIGHT] = 0; p.v[WB_AUTO] = 1; p.v[POWER_LINE] = 1;
    return p;
  }
  bool factory(double t, Iface &cam) {
    std::string err, ch;
    bool ok = apply(cam, factory_profile(), &err, &ch);
    apply_count++; apply_ok = ok;
    if (ok) last_error.clear(); else last_error = err;
    note(t, ok ? "restore defaults: factory values applied [" + ch + "]" : "restore defaults FAILED (" + err + "), previous settings restored");
    have_orig = false; active = -1;
    return ok;
  }

  // the camera was re-opened (camerad restart of the stream): forget what we applied so the profile is applied again once it streams
  void reopened() { active = -1; have_orig = false; night_ev = day_ev = -1; }

  void note(double t, const std::string &s) { char b[48]; snprintf(b, sizeof b, "[%.0f] ", t); log.push_back(b + s); last_reason = s; }

  // call about once a second. `conf_changed`: the profile file or the mode file changed since the last tick.
  void tick(double t, Mode want, bool conf_changed, const Conf &cf, const Measure &m, Iface &cam) {
    if (want != mode) { note(t, std::string("mode ") + mode_name(mode) + " -> " + mode_name(want)); mode = want; conf_changed = true; night_ev = day_ev = -1; }
    if (mode == M_OFF) {
      if (active >= 0 || have_orig) restore(t, cam);
      return;
    }
    if (!m.frames_ok || m.stream_age_s < 5.0) return;                 // safe to touch the camera only while it streams normally
    int target = active;
    std::string why;
    if (mode == M_DAY) { target = 0; why = "manual DAY"; }
    else if (mode == M_NIGHT) { target = 1; why = "manual NIGHT"; }
    else target = decide(t, cf.a, m, &why);
    bool switching = target != active;
    if (switching && mode == M_AUTO && active >= 0 && t - t_switch < cf.a.min_dwell_s) return;   // dwell: never switch twice within min_dwell_s
    if (!switching && !conf_changed) return;
    if (t - t_applied < 5.0 && !switching) return;                      // slider drags are coalesced
    if (fails >= 5 && t - t_applied < 60.0) return;                     // a profile that keeps failing is retried only once a minute
    if (!have_orig) snapshot(cam);
    std::string err, ch;
    bool ok = apply(cam, target == 0 ? cf.day : cf.night, &err, &ch);
    t_applied = t;
    apply_count++; apply_ok = ok;
    if (!ok) { fails++; last_error = err; note(t, std::string("profile ") + (target ? "NIGHT" : "DAY") + " FAILED (" + err + "), previous settings restored"); return; }
    fails = 0; last_error.clear();
    if (switching) { t_switch = t; note(t, std::string("profile -> ") + (target ? "NIGHT" : "DAY") + " because " + why + (ch.empty() ? "" : " [" + ch + "]")); }
    else if (!ch.empty()) note(t, std::string("profile ") + (target ? "NIGHT" : "DAY") + " edited [" + ch + "]");
    active = target;
    night_ev = day_ev = -1;
  }

 private:
  void snapshot(Iface &cam) {
    for (int i = 0; i < N_CTL; i++) { int v; orig[i] = cam.get(i, &v) ? v : -1; }
    have_orig = true;
  }
  void restore(double t, Iface &cam) {
    if (have_orig) {
      Profile p;
      for (int i = 0; i < N_CTL; i++) p.v[i] = orig[i];
      p.v[WB_AUTO] = orig[WB_AUTO]; p.v[AE_MODE] = orig[AE_MODE];
      std::string err, ch;
      bool ok = apply(cam, p, &err, &ch);
      note(t, ok ? "profiles off: camera settings restored [" + ch + "]" : "profiles off: restore FAILED (" + err + ")");
    }
    have_orig = false; active = -1;
  }
  // AUTO: returns 0 (day) / 1 (night); `why` explains. Evidence must hold continuously for the confirm time.
  int decide(double t, const AutoCfg &a, const Measure &m, std::string *why) {
    if (active < 0) active_init(m, a);
    bool night_ev_now = (m.luma >= 0 && m.luma < a.night_luma) || (m.exposure >= a.night_exposure && m.gain >= a.night_gain);
    bool day_ev_now = (m.luma >= a.day_luma) || (m.exposure >= 0 && m.gain >= 0 && m.exposure <= a.day_exposure && m.gain <= a.day_gain && m.luma >= a.night_luma);
    int cur = active < 0 ? 0 : active;
    char b[160];
    if (cur == 0) {
      if (night_ev_now) { if (night_ev < 0) night_ev = t; if (t - night_ev >= a.night_confirm_s) { snprintf(b, sizeof b, "dark for %ds (luma %.0f, exposure %d, gain %d)", a.night_confirm_s, m.luma, m.exposure, m.gain); *why = b; return 1; } }
      else night_ev = -1;
    } else {
      if (day_ev_now) { if (day_ev < 0) day_ev = t; if (t - day_ev >= a.day_confirm_s) { snprintf(b, sizeof b, "bright for %ds (luma %.0f, exposure %d, gain %d)", a.day_confirm_s, m.luma, m.exposure, m.gain); *why = b; return 0; } }
      else day_ev = -1;
    }
    *why = "no change";
    return cur;
  }
  void active_init(const Measure &, const AutoCfg &) {}
};

}  // namespace camprof
