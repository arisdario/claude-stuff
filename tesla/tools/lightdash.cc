// lightdash.cc - C++ twin of tesla/tools/lightdash.py: ONE small read-only web page showing what the panda tells the instrument cluster.
// Same page, same JSON, same byte-for-byte replay of the firmware's 0x557/0x559/0x659 -> 0x239/0x309 expansion (safety_tesla.h, v1.7.3).
// Reads openpilot messages (sendcan, model, carState, controlsState) and, optionally, camerad's own sign/light detector output
// (/data/sign_detect.json, if that opt-in feature is built and enabled - see camerad/cameras/SIGN_DETECT.md) - it never opens USB or the
// panda, sends nothing, and writes no file of its own. Single thread, poll(), no outside libraries beyond cereal's own (capnp, zmq/msgq).
//
//   build on the EON:  sh /data/claude-stuff/tesla/tools/build_lightdash.sh        (compiles against /data/openpilot, its generated cereal)
//   run:               nohup nice -n 19 /data/lightdash > /data/lightdash.log 2>&1 &       then open http://<EON-IP>:8090/
//   check without the car:  /data/lightdash --selftest
//   env: LIGHTDASH_PORT (8090), LIGHTDASH_BIND (0.0.0.0, or 127.0.0.1)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <string>
#include <cmath>
#include <vector>

#include <capnp/serialize.h>

#include "messaging.hpp"
#include "log.capnp.h"

static const double IC_LANE_SCALE = 2.0;
static const double PANDA_STALE_S = 2.0;      // firmware: no 0x659 for 2 s -> reset_DAS_data()
static const double X_IDXS[25] = {0., 0.1875, 0.75, 1.6875, 3., 4.6875, 6.75, 9.1875, 12., 15.1875, 18.75, 22.6875, 27., 31.6875, 36.75,
                                  42.1875, 48., 54.1875, 60.75, 67.6875, 75., 82.6875, 90.75, 99.1875, 108.};

static double now_s() { timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static double wall_s() { timespec ts; clock_gettime(CLOCK_REALTIME, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }

static void appendf(std::string &s, const char *fmt, ...) {
  char buf[512]; va_list ap; va_start(ap, fmt); int n = vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
  if (n > 0) s.append(buf, n < (int)sizeof buf ? n : (int)sizeof buf - 1);
}

static const char *op_text(int op) {
  switch (op) { case 0: return "disabled"; case 1: return "unavailable"; case 2: return "available"; case 3: return "active";
    case 4: return "restricted"; case 5: return "active (nav)"; case 8: return "aborting"; case 9: return "aborted"; default: return "?"; }
}

// ---------------------------------------------------------------------------------------------- the firmware's state and expansion
struct Mirror {
  int lWidth, curv[4], laneRange, lLine, rLine;
  bool have557;
  uint32_t objLo[3], objHi[3];
  int op, cc, alca, hands, acc, lng;
  double t659; int emu; int laneIdx;

  Mirror() : t659(0), emu(-1), laneIdx(0) { reset(); }
  void reset() {                                                    // defaults from safety_tesla.h lines 154-186
    lWidth = 29; curv[0] = 0x64; curv[1] = curv[2] = curv[3] = 0x7F; laneRange = 160; lLine = rLine = 0; have557 = false;
    for (int i = 0; i < 3; i++) { objLo[i] = 0xFFFFFF00u | i; objHi[i] = 0x03FFFF83u; }
    op = 1; cc = 0; alca = 5; hands = 0; acc = 0; lng = 0;
  }
  bool stale(double now) const { return t659 == 0 || now - t659 > PANDA_STALE_S; }

  void feed(uint32_t addr, const uint8_t *d, size_t n, double now) {       // mirrors tesla_tx_hook for the frames openpilot sends
    if (addr == 0x554) {                                                  // switches: only 3 bytes on the wire (the others are 8)
      if (n >= 1) emu = (d[0] >> 5) & 1;
      return;
    }
    if (n < 8) return;
    uint32_t lo, hi; memcpy(&lo, d, 4); memcpy(&hi, d + 4, 4);
    if (addr == 0x557) {
      int rq = lo >> 16 & 3, lq = lo >> 18 & 3;
      rLine = rq > 1; lLine = lq > 1;
      lWidth = lo >> 20 & 0xF;
      curv[0] = lo >> 24 & 0xFF; curv[1] = hi & 0xFF; curv[2] = hi >> 8 & 0xFF; curv[3] = hi >> 16 & 0xFF;
      laneRange = (hi >> 24 & 0x3F) * 4; have557 = true;
    } else if (addr == 0x559) {
      int lane = lo & 3;
      if (lane <= 2) {
        if ((lo >> 8 & 0xFF) == 0) { objLo[lane] = 0xFFFFFF00u | lane; objHi[lane] = 0x03FFFF83u; }
        else { objLo[lane] = lo; objHi[lane] = hi; }
      }
    } else if (addr == 0x659) {
      if (t659 != 0 && now - t659 > PANDA_STALE_S) reset();
      t659 = now;
      op = d[0] & 0x0F; lng = d[0] >> 7 & 1; cc = d[3] >> 6 & 3; alca = d[3] & 0x1F; hands = d[2] & 7; acc = d[1];
    }
  }

  void lanesFrame(uint8_t out[8]) const {                          // exactly safety_tesla.h:705-711, 32-bit int arithmetic
    const uint32_t fuse = 2;
    uint32_t mlb = ((uint32_t)lWidth << 4) + ((uint32_t)rLine << 1) + (uint32_t)lLine + ((uint32_t)laneRange << 8) +
                   ((uint32_t)curv[0] << 16) + ((uint32_t)curv[1] << 24);
    uint32_t mhb = 0x00F00000u + ((uint32_t)laneIdx << 28) + (uint32_t)curv[2] + ((uint32_t)curv[3] << 8) +
                   (((((uint32_t)rLine * fuse) << 2) + ((uint32_t)lLine * fuse)) << 16);
    memcpy(out, &mlb, 4); memcpy(out + 4, &mhb, 4);
  }
};

// read-only from the car's own 'can' stream (never sent): 0x30A byte 0 = lamp state (1 Hz + on change), 0x283 byte 7 = outside temperature (~1 Hz)
struct CarState {
  int hb = 0, fl = 0; double t30a = 0; double out = 0; bool out_ok = false; double t283 = 0;
  void feed(uint32_t addr, const uint8_t *d, size_t n, double now) {
    if (addr == 0x30A && n >= 1) { hb = (d[0] >> 3) & 1; fl = (d[0] >> 2) & 1; t30a = now; }
    else if (addr == 0x283 && n >= 8) { out_ok = d[7] < 0xFE; out = d[7] * 0.5 - 40; t283 = now; }   // 0xFE init / 0xFF SNA = no value
  }
};
static const double CAR_HOLD_S = 3.0;                                  // 0x30A comes every 1 s: hold the last value across one lost frame

// BODY-BEGIN read-only, the car's own body/gateway frames on bus 0 (opendbc tesla_can.dbc), never sent:
// 0x318 GTW_carState doors/frunk/trunk, 0x283 BODY_R1 lamps/light sensor/wipers/park brake/reverse/inside temp,
// 0x238 UI_driverAssistRoadSign record 3 = map speed limit (m/s), 0x2F8 UI_gpsVehicleSpeed bit 46 = limit units (1 = km/h)
static const double BODY_HOLD_S = 3.0;     // older than this = unknown, never "closed"/"off"
static const double LIMIT_HOLD_S = 10.0;   // 0x238 cycles through several record types
struct Body {
  int fl = 0, fr = 0, rl = 0, rr = 0, trunk = 0, frunk = 0; double t318 = 0;
  int lo = 0, hi = 0, night = 0, tunnel = 0, sens_bad = 0, wiper = 0, pbrake = 0, rev = 0, inside = 0xFF; double t283 = 0;
  double lim_ms = 0, tlim = 0; bool kph = true;
  void feed(uint32_t addr, const uint8_t *d, size_t n, double now) {
    if (addr == 0x318 && n >= 8) {
      fl = d[1] >> 4 & 3; fr = d[1] >> 6 & 3; rl = d[2] >> 6 & 3; rr = d[3] >> 5 & 3; trunk = d[5] >> 6 & 3; frunk = d[6] >> 2 & 3; t318 = now;
    } else if (addr == 0x283 && n >= 8) {
      lo = d[4] & 1; hi = d[4] >> 1 & 1; night = d[2] >> 6 & 1; tunnel = d[2] >> 4 & 1; sens_bad = (d[2] >> 3 & 1) | (d[2] >> 5 & 1);
      wiper = d[3] >> 5 & 1; pbrake = d[3] >> 4 & 1; rev = d[4] >> 6 & 3; inside = d[5]; t283 = now;
    } else if (addr == 0x238 && n >= 2 && d[0] == 3) {
      lim_ms = d[1] * 0.25; tlim = now;
    } else if (addr == 0x2F8 && n >= 6) {
      kph = (d[5] >> 6 & 1) == 1;
    }
  }
};
static const char *tri(bool live, int v) { return !live ? "null" : v == 1 ? "true" : v == 0 ? "false" : "null"; }   // 0 closed 1 open 2 init 3 SNA
static void json_body(std::string &s, const Body &b, double now) {
  bool a = b.t318 != 0 && now - b.t318 <= BODY_HOLD_S, c = b.t283 != 0 && now - b.t283 <= BODY_HOLD_S;
  appendf(s, "\"body\":{\"doors\":[%s,%s,%s,%s],\"frunk\":%s,\"trunk\":%s,", tri(a, b.fl), tri(a, b.fr), tri(a, b.rl), tri(a, b.rr), tri(a, b.frunk), tri(a, b.trunk));
  if (c) appendf(s, "\"lo\":%s,\"hi\":%s,\"wiper\":%s,\"pbrake\":%s,\"rev\":%s,", b.lo ? "true" : "false", b.hi ? "true" : "false",
                 b.wiper ? "true" : "false", b.pbrake ? "true" : "false", b.rev == 1 ? "true" : "false");
  else s += "\"lo\":null,\"hi\":null,\"wiper\":null,\"pbrake\":null,\"rev\":null,";
  if (!c || b.sens_bad) s += "\"light\":null,"; else appendf(s, "\"light\":\"%s\",", b.tunnel ? "tunnel" : b.night ? "night" : "day");
  if (c && b.inside != 0xFF) appendf(s, "\"inside\":%.1f,", b.inside * 0.25); else s += "\"inside\":null,";
  if (b.tlim != 0 && now - b.tlim <= LIMIT_HOLD_S && b.lim_ms > 0) {
    double v = b.lim_ms * (b.kph ? 3.6 : 2.236936);
    appendf(s, "\"limit\":%d,", (int)(std::floor(v / 5. + 0.5) * 5));          // map limits are multiples of 5 (0.25 m/s steps are noisy)
  } else s += "\"limit\":null,";
  appendf(s, "\"unit\":\"%s\"}", b.kph ? "km/h" : "mph");
}
static void json_str(std::string &s, const char *t) {                         // alert texts: escape for JSON, cap the length
  s += '"';
  for (int i = 0; t && t[i] && i < 80; i++) {
    unsigned char ch = t[i];
    if (ch == '"' || ch == '\\') { s += '\\'; s += (char)ch; } else if (ch < 0x20) s += ' '; else s += (char)ch;
  }
  s += '"';
}
// BODY-END

// OPTIONAL: camerad's own stop-sign/traffic-light detector output (see camerad/cameras/SIGN_DETECT.md).
// Read-only, read fresh every tick (the file is tiny) - never opens the camera or any process, just
// one open()/stat() on a file that may not exist. No JSON library: this repo controls both the writer
// (sign_detect_io.cc) and this reader, so a few strstr()/atof() calls on the known fixed format is the
// simplest thing that's still exactly as robust as a parser here, consistent with how this file already
// avoids pulling in dependencies beyond cereal's own.
struct SignState {
  bool stop = false; double stop_dist = -1, stop_conf = 0;
  bool light = false; int light_color = -1; double light_conf = 0;
  bool fresh = false;   // false if the file is missing or its mtime is older than SIGN_HOLD_S
};
static const double SIGN_HOLD_S = 2.0;                                 // the detector writes at ~2.5 Hz when enabled
static const char *SIGN_DETECT_FILE = "/data/sign_detect.json";

static double jfield(const char *s, const char *key) {
  const char *p = strstr(s, key);
  return p ? atof(p + strlen(key)) : 0.0;
}
static bool jbool(const char *s, const char *key) {
  const char *p = strstr(s, key);
  return p && !strncmp(p + strlen(key), "true", 4);
}

static SignState read_sign_state() {
  SignState s;
  struct stat st;
  if (stat(SIGN_DETECT_FILE, &st) != 0) return s;                     // not built/enabled - common case, cheap
  if (time(nullptr) - st.st_mtime > (time_t)SIGN_HOLD_S) return s;    // stale - camerad stopped writing
  FILE *f = fopen(SIGN_DETECT_FILE, "r");
  if (!f) return s;
  char buf[512] = {0};
  size_t n = fread(buf, 1, sizeof(buf) - 1, f);
  fclose(f);
  if (n == 0) return s;
  s.fresh = true;
  s.stop = jbool(buf, "\"stop_sign\":");
  s.stop_dist = jfield(buf, "\"stop_sign_dist_m\":");
  s.stop_conf = jfield(buf, "\"stop_sign_confidence\":");
  s.light = jbool(buf, "\"light_present\":");
  s.light_color = (int)jfield(buf, "\"light_color\":");
  s.light_conf = jfield(buf, "\"light_confidence\":");
  return s;
}

static void json_lanes(std::string &s, const uint8_t *b) {          // DAS_lanes 0x239, DBC scaling
  appendf(s, "\"lanes\":{\"width\":%.4f,\"left\":%d,\"right\":%d,\"range\":%d,\"c\":[%.3f,%.5f,%.7f,%.9f],\"use\":[%d,%d],\"counter\":%d}",
          2 + 0.3125 * (b[0] >> 4), b[0] & 1, b[0] >> 1 & 1, b[1], b[2] * 0.035 - 3.5, b[3] * 0.0016 - 0.2, b[4] * 2e-5 - 0.0025,
          b[5] * 2.4e-7 - 3e-5, b[6] & 3, b[6] >> 2 & 3, b[7] >> 4);
}

static void json_objects(std::string &s, const Mirror &m) {         // DAS_object 0x309 -> vehicles present, distances restored to real metres
  s += "\"obj\":[";
  bool first = true;
  const char *slot[3] = {"lead", "left", "right"};
  for (int lane = 0; lane < 3; lane++) {
    uint64_t v = (uint64_t)m.objLo[lane] | ((uint64_t)m.objHi[lane] << 32);
    auto bits = [&](int start, int n) { return (int)((v >> start) & ((1ull << n) - 1)); };
    const int sp[2][6] = {{3, 7, 8, 16, 20, 27}, {34, 38, 39, 47, 51, 58}};
    for (int k = 0; k < 2; k++) {
      int dxr = bits(sp[k][2], 8);
      if (dxr == 0xFF || dxr == 0) continue;
      int vxr = bits(sp[k][3], 4);
      if (!first) s += ",";
      first = false;
      appendf(s, "{\"n\":%d,\"type\":%d,\"relevant\":%d,\"dx\":%.1f,\"dy\":%.2f,", k + 1, bits(sp[k][0], 3), bits(sp[k][1], 1), dxr * 0.5 * IC_LANE_SCALE,
              bits(sp[k][4], 7) * 0.35 - 22.05);
      if (vxr == 15) s += "\"vx\":null,"; else appendf(s, "\"vx\":%.1f,", (vxr * 4 - 30) * IC_LANE_SCALE);
      appendf(s, "\"id\":%d,\"slot\":\"%s\"}", bits(sp[k][5], k == 0 ? 7 : 6), slot[lane]);
    }
  }
  s += "]";
}

struct Extra {
  double v = 0, ang = 0; bool en = false;
  double a = 0; bool bl = false, br = false, gas = false, brk = false, act = false; std::string al1, al2; int ast = 0;
  bool have_model = false; std::vector<float> p, l, r, e_l, e_r; float pl = 0, pr = 0; bool have_edges = false;
};

static std::string snapshot(const Mirror &m, const Extra &x, double now, const CarState &cs = CarState(), const Body &bd = Body()) {
  std::string s; s.reserve(1400);
  bool stale = m.stale(now);
  appendf(s, "{\"present\":%s,\"emu\":%s,", stale ? "false" : "true", m.emu < 0 ? "null" : (m.emu ? "true" : "false"));
  if (m.t659 != 0) appendf(s, "\"age\":%.1f,", now - m.t659); else s += "\"age\":null,";
  appendf(s, "\"default\":%s,\"status\":{\"op\":%d,\"long\":%d,\"cc\":%d,\"alca\":%d,\"hands\":%d,\"acc\":%d,\"text\":\"%s\"},",
          (!m.have557 || stale) ? "true" : "false", m.op, m.lng, m.cc, m.alca, m.hands, m.acc, op_text(m.op));
  uint8_t lf[8]; m.lanesFrame(lf); json_lanes(s, lf); s += ",";
  json_objects(s, m);
  appendf(s, ",\"v\":%.3f,\"ang\":%.3f,\"en\":%s,", x.v, x.ang, x.en ? "true" : "false");
  appendf(s, "\"a\":%.2f,\"bl\":%s,\"br\":%s,\"gas\":%s,\"brk\":%s,\"act\":%s,\"ast\":%d,\"al1\":", x.a, x.bl ? "true" : "false", x.br ? "true" : "false",
          x.gas ? "true" : "false", x.brk ? "true" : "false", x.act ? "true" : "false", x.ast);
  json_str(s, x.al1.c_str()); s += ",\"al2\":"; json_str(s, x.al2.c_str()); s += ",";
  json_body(s, bd, now); s += ",";
  s += "\"car\":{";
  if (cs.t30a != 0 && now - cs.t30a <= CAR_HOLD_S) appendf(s, "\"hb\":%d,\"fl\":%d,", cs.hb, cs.fl); else s += "\"hb\":null,\"fl\":null,";
  if (cs.out_ok && cs.t283 != 0 && now - cs.t283 <= 5.0) appendf(s, "\"out\":%.1f},", cs.out); else s += "\"out\":null},";
  SignState sg = read_sign_state();
  s += "\"sign\":{";
  if (sg.fresh && sg.stop) appendf(s, "\"stop\":true,\"stop_dist\":%.1f,", sg.stop_dist); else s += "\"stop\":false,\"stop_dist\":null,";
  if (sg.fresh && sg.light) appendf(s, "\"light\":true,\"light_color\":%d},", sg.light_color); else s += "\"light\":false,\"light_color\":null},";
  if (x.have_model) {
    s += "\"model\":{";
    auto arr = [&](const char *k, const std::vector<float> &a) {
      if (a.empty()) { appendf(s, "\"%s\":null,", k); return; }
      appendf(s, "\"%s\":[", k);
      for (size_t i = 0; i < a.size(); i++) appendf(s, "%s%.2f", i ? "," : "", a[i]);
      s += "],";
    };
    arr("p", x.p); arr("l", x.l); arr("r", x.r);
    appendf(s, "\"pl\":%.2f,\"pr\":%.2f", x.pl, x.pr);
    if (x.have_edges) {
      s += ",\"e\":{\"x\":[";
      for (int i = 0; i < 25; i++) appendf(s, "%s%.4g", i ? "," : "", X_IDXS[i]);
      s += "],\"l\":[";
      for (size_t i = 0; i < x.e_l.size(); i++) appendf(s, "%s%.2f", i ? "," : "", x.e_l[i]);
      s += "],\"r\":[";
      for (size_t i = 0; i < x.e_r.size(); i++) appendf(s, "%s%.2f", i ? "," : "", x.e_r[i]);
      s += "]}";
    }
    s += "},";
  } else {
    s += "\"model\":null,";
  }
  appendf(s, "\"t\":%.2f}", wall_s());
  return s;
}

// ---------------------------------------------------------------------------------------------- web
static const char PAGE[] = R"LIGHTDASHPAGE(<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>cluster view</title><style>html,body{margin:0;height:100%;background:#0b0d10;color:#e8edf2;font:14px system-ui,Arial,sans-serif;overflow:hidden}
canvas{display:block;width:100vw;height:100vh}</style></head><body><canvas id=c></canvas><script>
var cv=document.getElementById('c'),g=cv.getContext('2d'),S=null,tdv=false,mode=0,W=0,H=0,SC=2;
function rs(){var d=Math.min(window.devicePixelRatio||1,1.5);W=innerWidth;H=innerHeight;cv.width=W*d;cv.height=H*d;g.setTransform(d,0,0,d,0,0);draw()}
addEventListener('resize',rs);cv.addEventListener('click',function(){mode=(mode+1)%3;tdv=(mode==1);draw()});   // drive -> top-down -> diagnostic
function P(x,y){ // real metres (x ahead, y left) -> screen
 if(tdv){var s=H*0.8/110;return [W/2-y*s*4,H*0.92-x*s]}
 var K=H*0.9,d=x+5;return [W/2-y*K/d,H*0.38+2.5*K/d]}
function line(pts,col,w,dash){if(pts.length<2)return;g.beginPath();g.setLineDash(dash||[]);g.strokeStyle=col;g.lineWidth=w;pts.forEach(function(p,i){i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke();g.setLineDash([])}
function poly(c,x){return c[0]+c[1]*x+c[2]*x*x+c[3]*x*x*x}
function rr(x,y,w,h,r){g.beginPath();g.moveTo(x+r,y);g.lineTo(x+w-r,y);g.quadraticCurveTo(x+w,y,x+w,y+r);g.lineTo(x+w,y+h-r);g.quadraticCurveTo(x+w,y+h,x+w-r,y+h);g.lineTo(x+r,y+h);g.quadraticCurveTo(x,y+h,x,y+h-r);g.lineTo(x,y+r);g.quadraticCurveTo(x,y,x+r,y);g.closePath()}
function carBack(cx,y0,w,rel){ // a car seen from behind; y0 = where its tyres touch the road, w = 1.8 m of real width
 var h=w*0.8,dark='#0e1217',gr;
 if(w<14){g.fillStyle='#aab3bf';g.fillRect(cx-w/2,y0-h,w,h);return}
 g.save();g.translate(cx,y0);g.scale(1,0.12);g.fillStyle='rgba(0,0,0,.55)';g.beginPath();g.arc(0,0,w*0.58,0,6.2832);g.fill();g.restore();
 if(rel){g.save();g.translate(cx,y0);g.scale(1,0.18);gr=g.createRadialGradient(0,0,0,0,0,w*0.8);gr.addColorStop(0,'rgba(255,60,60,.38)');gr.addColorStop(1,'rgba(255,60,60,0)');g.fillStyle=gr;g.beginPath();g.arc(0,0,w*0.8,0,6.2832);g.fill();g.restore()}
 g.fillStyle=dark;rr(cx-w*0.46,y0-h*0.30,w*0.17,h*0.30,w*0.03);g.fill();rr(cx+w*0.29,y0-h*0.30,w*0.17,h*0.30,w*0.03);g.fill();
 gr=g.createLinearGradient(0,y0-h,0,y0-h*0.15);gr.addColorStop(0,'#eef1f5');gr.addColorStop(0.55,'#bcc3cd');gr.addColorStop(1,'#7c8592');g.fillStyle=gr;
 rr(cx-w/2,y0-h*0.58,w,h*0.40,w*0.08);g.fill();
 g.beginPath();g.moveTo(cx-w*0.42,y0-h*0.56);g.quadraticCurveTo(cx-w*0.36,y0-h*0.99,cx-w*0.28,y0-h);g.lineTo(cx+w*0.28,y0-h);g.quadraticCurveTo(cx+w*0.36,y0-h*0.99,cx+w*0.42,y0-h*0.56);g.closePath();g.fill();
 gr=g.createLinearGradient(0,y0-h*0.94,0,y0-h*0.60);gr.addColorStop(0,'#2b3747');gr.addColorStop(1,'#0f151d');g.fillStyle=gr;
 g.beginPath();g.moveTo(cx-w*0.36,y0-h*0.62);g.quadraticCurveTo(cx-w*0.31,y0-h*0.92,cx-w*0.25,y0-h*0.94);g.lineTo(cx+w*0.25,y0-h*0.94);g.quadraticCurveTo(cx+w*0.31,y0-h*0.92,cx+w*0.36,y0-h*0.62);g.closePath();g.fill();
 g.fillStyle=rel?'#ff4d52':'#9e4a4e';if(rel){g.shadowColor='rgba(255,70,70,.95)';g.shadowBlur=w*0.12}
 rr(cx-w*0.47,y0-h*0.51,w*0.22,h*0.09,w*0.02);g.fill();rr(cx+w*0.25,y0-h*0.51,w*0.22,h*0.09,w*0.02);g.fill();g.shadowBlur=0;
 g.fillStyle='#d5dbe2';g.fillRect(cx-w*0.10,y0-h*0.33,w*0.20,h*0.07);
 g.fillStyle='rgba(0,0,0,.30)';rr(cx-w/2,y0-h*0.23,w,h*0.05,w*0.02);g.fill()}
function carTop(cx,yr,w,l,rel){ // seen from above, rear edge at screen y=yr, nose pointing up (4.5 m long, 1.8 m wide)
 var dark='#0e1217',gl='#18212c',gr;
 g.fillStyle=dark;rr(cx-w*0.53,yr-l*0.84,w*0.10,l*0.18,w*0.03);g.fill();rr(cx+w*0.43,yr-l*0.84,w*0.10,l*0.18,w*0.03);g.fill();rr(cx-w*0.53,yr-l*0.20,w*0.10,l*0.18,w*0.03);g.fill();rr(cx+w*0.43,yr-l*0.20,w*0.10,l*0.18,w*0.03);g.fill();
 gr=g.createLinearGradient(cx-w/2,0,cx+w/2,0);gr.addColorStop(0,'#aeb6c1');gr.addColorStop(0.5,'#e6e9ee');gr.addColorStop(1,'#aeb6c1');g.fillStyle=gr;rr(cx-w/2,yr-l,w,l,w*0.30);g.fill();
 g.fillStyle=gl;rr(cx-w*0.38,yr-l*0.66,w*0.76,l*0.22,w*0.10);g.fill();rr(cx-w*0.36,yr-l*0.30,w*0.72,l*0.13,w*0.08);g.fill();
 g.fillStyle=rel?'#ff4d52':'#9e4a4e';if(rel){g.shadowColor='rgba(255,70,70,.95)';g.shadowBlur=w*0.5}
 rr(cx-w*0.46,yr-l*0.06,w*0.22,l*0.04,w*0.02);g.fill();rr(cx+w*0.24,yr-l*0.06,w*0.22,l*0.04,w*0.02);g.fill();g.shadowBlur=0}
function drawDiag(){g.fillStyle='#0b0d10';g.fillRect(0,0,W,H);if(!S||!S.lanes){g.fillStyle='#9aa4af';g.fillText('waiting for data...',20,30);return}
 var L=S.lanes,R=Math.min(110,L.range*SC),pw=[],i;
 // grid
 g.strokeStyle='#1b2128';g.lineWidth=1;for(i=10;i<=100;i+=10){var a=P(i,-6),b=P(i,6);g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.stroke()}
 var M=SMD; // the model's own, faint (smoothed)
 if(M){var xs=[];for(i=0;i<25;i++)xs.push(i*4);
  if(M.l)line(M.l.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.r)line(M.r.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.p){var cl=M.p.map(function(y,k){return P(xs[k],y-.9)}),cr=M.p.map(function(y,k){return P(xs[k],y+.9)}).reverse();g.fillStyle='rgba(60,200,120,.12)';g.beginPath();cl.concat(cr).forEach(function(p,k){k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.fill()}
  if(M.e){line(M.e.l.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2);line(M.e.r.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2)}}
 // what the cluster is told: virtual lane = centre curve +- width/2 (cluster x = real/2)
 var cc=[],lf=[],rt=[];for(i=0;i<=R;i+=2){var xc=i/SC,yc=-poly(L.c,xc);cc.push(P(i,yc));lf.push(P(i,yc+L.width/2));rt.push(P(i,yc-L.width/2))}
 line(cc,'rgba(255,255,255,.18)',1,[6,8]);
 line(lf,L.left?'#ffd23f':'#555e68',L.left?5:2,L.left?null:[10,10]);line(rt,L.right?'#ffd23f':'#555e68',L.right?5:2,L.right?null:[10,10]);
 // lead / side vehicles
 SO.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w,h;g.globalAlpha=o.al;
  if(tdv){h=Math.max(26,4.5*H*0.8/110*1.6);w=Math.max(12,h/2.2);carTop(p[0],p[1],w,h,o.relevant)}   // an icon with car proportions: the top view stretches the sideways scale 4x, so true scale would look squashed
  else{w=Math.max(12,1.8*H*0.9/(o.dx+5));h=w*0.8;carBack(p[0],p[1],w,o.relevant)}
  g.fillStyle='#e8edf2';g.textAlign='center';g.fillText(o.slot+' '+Math.round(o.dx)+' m',p[0],p[1]-h-6);g.textAlign='left';g.globalAlpha=1});
 // header
 var st=S.status,y0=22;g.font='15px system-ui';g.fillStyle='#e8edf2';
 g.fillText('openpilot '+(S.en?'ENGAGED':'off')+'   speed '+Math.round((S.v||0)*3.6)+' km/h   wheel '+(S.ang||0).toFixed(1)+' deg',16,y0);
 g.fillStyle=S.present?'#7bd88f':'#e5383b';g.fillText('panda data: '+(S.present?'live':('STALE ('+S.age+' s) - cluster is shown defaults')),16,y0+20);
 g.fillStyle='#e8edf2';g.fillText('cluster status: '+st.text+'   emulation: '+(S.emu===null?'?':S.emu?'on':'OFF (panda sends nothing)'),16,y0+40);
 g.fillText('lanes: width '+L.width.toFixed(2)+' m  left '+(L.left?'drawn':'not drawn')+'  right '+(L.right?'drawn':'not drawn')+'  range '+L.range+' (x'+SC+')'+(S.default?'   [DEFAULT frame - no lane data from openpilot]':''),16,y0+60);
 var B=S.body||{};g.fillText('accel '+(S.a||0).toFixed(2)+' m/s2   blinker '+(S.bl?'L ':'')+(S.br?'R':'')+'   doors '+JSON.stringify(B.doors||null)+'   limit '+(B.limit||'-')+' '+(B.unit||'')+'   light '+(B.light||'?')+'   wipers '+(B.wiper?'on':'off')+(S.al1?'   alert: '+S.al1:''),16,y0+80);
 g.fillStyle='#9aa4af';g.font='12px system-ui';g.fillText('yellow = lane lines sent to the cluster   cyan = model lanes   green = model path   red = road edges   tap = next view (drive / top / diagnostic)',16,H-14)}
function grad(y0,y1,st){var gr=g.createLinearGradient(0,y0,0,y1);for(var i=0;i<st.length;i++)gr.addColorStop(st[i][0],st[i][1]);return gr}
function ribbon(a,b,fill){if(a.length<2)return;g.beginPath();a.forEach(function(p,i){i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});for(var i=b.length-1;i>=0;i--)g.lineTo(b[i][0],b[i][1]);g.closePath();g.fillStyle=fill;g.fill()}
function chip(x,y,t,col,bg){g.font='600 12px system-ui';var w=g.measureText(t).width+20;g.fillStyle=bg;rr(x,y,w,24,12);g.fill();g.fillStyle=col;g.textAlign='left';g.fillText(t,x+10,y+16);return w}
function pill(cx,cy,t){g.font='600 12px system-ui';var w=g.measureText(t).width+14;g.fillStyle='rgba(0,0,0,.45)';rr(cx-w/2,cy-11,w,20,10);g.fill();g.fillStyle='#dfe5ec';g.textAlign='center';g.fillText(t,cx,cy+4);g.textAlign='left'}
function cruiseIcon(cx,cy,r,col){g.strokeStyle=col;g.lineWidth=3;g.beginPath();g.arc(cx,cy,r,Math.PI*0.80,Math.PI*2.20);g.stroke();g.beginPath();g.moveTo(cx,cy+r*0.55);g.lineTo(cx+r*0.45,cy-r*0.45);g.stroke();g.fillStyle=col;g.beginPath();g.arc(cx,cy+r*0.55,3,0,6.2832);g.fill()}
function wheelIcon(cx,cy,r,col){g.strokeStyle=col;g.lineWidth=3.2;g.beginPath();g.arc(cx,cy,r,0,6.2832);g.stroke();g.beginPath();g.moveTo(cx-r,cy);g.lineTo(cx-r*0.35,cy);g.moveTo(cx+r,cy);g.lineTo(cx+r*0.35,cy);g.moveTo(cx,cy+r*0.35);g.lineTo(cx,cy+r);g.stroke();g.beginPath();g.arc(cx,cy,r*0.22,0,6.2832);g.fillStyle=col;g.fill()}
function beamIcon(cx,cy,r,col){g.strokeStyle=col;g.fillStyle=col;g.lineWidth=2.6;g.beginPath();g.moveTo(cx+r*0.15,cy-r*0.8);g.quadraticCurveTo(cx-r*0.95,cy-r*0.8,cx-r*0.95,cy);g.quadraticCurveTo(cx-r*0.95,cy+r*0.8,cx+r*0.15,cy+r*0.8);g.closePath();g.fill();for(var k=-1;k<=1;k++){g.beginPath();g.moveTo(cx+r*0.45,cy+k*r*0.62);g.lineTo(cx+r*1.1,cy+k*r*0.62);g.stroke()}}
function drawDrive(){
 g.fillStyle=grad(0,H,[[0,'#06090d'],[0.45,'#0b131b'],[1,'#10202c']]);g.fillRect(0,0,W,H);
 var eng=!!(S&&S.en),BLUE='rgba(64,156,255,0.95)';
 g.save();g.translate(W/2,H*0.72);g.scale(0.55,1);var rg=g.createRadialGradient(0,0,0,0,0,H*0.62);rg.addColorStop(0,eng?'rgba(40,120,210,.16)':'rgba(110,135,175,.10)');rg.addColorStop(1,'rgba(40,120,210,0)');g.fillStyle=rg;g.beginPath();g.arc(0,0,H*0.62,0,6.2832);g.fill();g.restore();
 if(!S||!S.lanes){g.fillStyle='#7d8792';g.font='16px system-ui';g.textAlign='center';g.fillText('waiting for data...',W/2,H/2);g.textAlign='left';return}
 var M=SMD,i,xs=[];for(i=0;i<25;i++)xs.push(i*4);
 var yb=P(0,0)[1],yh=P(110,0)[1];
 if(M&&M.l&&M.r){
  var lw=Math.max(2.6,Math.min(4.4,(M.l[2]-M.r[2])||3.4));
  var mk=function(arr,off){var q=arr.map(function(y,k){return P(xs[k],y+off)});q.unshift(P(-1.5,arr[0]+off));q.unshift(P(-3.5,arr[0]+off));return q};   // two extra points behind the car so the road runs to the bottom of the screen
  var lp=mk(M.l,0),rp=mk(M.r,0),ll=mk(M.l,lw),rl=mk(M.r,-lw);
  ribbon(lp,rp,grad(yb,yh,[[0,eng?'rgba(60,140,230,.22)':'rgba(150,165,190,.16)'],[1,'rgba(60,140,230,0)']]));
  var sd=grad(yb,yh,[[0,'rgba(150,165,190,.07)'],[1,'rgba(150,165,190,0)']]);ribbon(ll,lp,sd);ribbon(rp,rl,sd);
  var path=function(a){g.beginPath();a.forEach(function(p,k){k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke()};
  g.save();g.lineCap='round';
  g.strokeStyle=grad(yb,yh,[[0,'rgba(190,200,215,.28)'],[1,'rgba(190,200,215,0)']]);g.lineWidth=2;[ll,rl].forEach(path);
  if(eng){g.shadowColor='rgba(64,156,255,.85)';g.shadowBlur=14}
  g.strokeStyle=grad(yb,yh,[[0,eng?BLUE:'rgba(235,240,248,.80)'],[0.75,eng?'rgba(64,156,255,.45)':'rgba(235,240,248,.30)'],[1,'rgba(64,156,255,0)']]);g.lineWidth=5;[lp,rp].forEach(path);
  g.restore();
  if(M.e){g.save();g.strokeStyle=grad(yb,yh,[[0,'rgba(170,180,195,.30)'],[1,'rgba(170,180,195,0)']]);g.lineWidth=2;[M.e.l,M.e.r].forEach(function(a){g.beginPath();a.forEach(function(y,k){var p=P(M.e.x[k],y);k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke()});g.restore()}
 }
 SO.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w=Math.max(12,1.8*H*0.9/(o.dx+5));g.globalAlpha=o.al;carBack(p[0],p[1],w,o.relevant);pill(p[0],p[1]-w*0.8-14,Math.round(o.dx)+' m');g.globalAlpha=1});
 var ew=Math.min(H*0.27,W*0.2),ey=H*0.92;
 if(eng){g.save();g.translate(W/2,ey);g.scale(1,0.14);var eg=g.createRadialGradient(0,0,0,0,0,ew*0.95);eg.addColorStop(0,'rgba(64,156,255,.55)');eg.addColorStop(1,'rgba(64,156,255,0)');g.fillStyle=eg;g.beginPath();g.arc(0,0,ew*0.95,0,6.2832);g.fill();g.restore()}
 carBack(W/2,ey,ew,false);
 // the middle of the instrument panel: icons either side of the speed
 var pw=Math.min(W*0.5,380),py=14,ph=96,px=W/2-pw/2;
 g.fillStyle='rgba(8,14,20,.72)';rr(px,py,pw,ph,18);g.fill();g.strokeStyle='rgba(255,255,255,.06)';g.lineWidth=1;g.stroke();
 var ic=eng?'rgba(64,156,255,1)':'rgba(120,130,142,.9)';
 cruiseIcon(px+pw*0.14,py+ph*0.46,15,ic);wheelIcon(px+pw*0.86,py+ph*0.46,16,ic);
 var cr=S.car||{},bdy=S.body||{};if(cr.hb||cr.fl||bdy.lo)beamIcon(px+pw*0.14,py+ph*0.46+34,9,cr.hb?'rgba(64,156,255,1)':bdy.lo&&!cr.fl?'rgba(110,220,130,1)':'rgba(235,240,248,.85)');   // blue = high, green = low, white = flash
 if(cr.out!==null&&cr.out!==undefined){g.fillStyle='#aab4c2';g.font='500 14px system-ui';g.textAlign='center';g.fillText(cr.out.toFixed(1)+' \u00b0C',px+pw*0.86,py+ph*0.46+38);g.textAlign='left'}
 var sg=S.sign||{};if(sg.stop||sg.light){g.font='600 12px system-ui';var parts=[];
  if(sg.stop)parts.push({t:'STOP '+(sg.stop_dist!=null?Math.round(sg.stop_dist)+'m':''),bg:'rgba(180,30,30,.85)',col:'#fff'});
  if(sg.light){var lc=['rgba(220,40,40,.9)','rgba(230,180,20,.9)','rgba(40,180,70,.9)'][sg.light_color]||'rgba(140,140,140,.9)';parts.push({t:'●',bg:lc,col:'#fff'})}
  var tw=0;for(var k=0;k<parts.length;k++)tw+=g.measureText(parts[k].t).width+20+(k?6:0);
  var cx0=W/2-tw/2,cy0=py+ph+8;
  for(var k=0;k<parts.length;k++){var w=chip(cx0,cy0,parts[k].t,parts[k].col,parts[k].bg);cx0+=w+6}
 }
 extras(px,py,pw,ph);
 g.fillStyle='#f3f6fa';g.font='300 58px system-ui';g.textAlign='center';g.fillText(Math.round((S.v||0)*3.6),W/2,py+62);g.fillStyle='#8590a0';g.font='500 13px system-ui';g.fillText('km/h',W/2,py+84);
 g.fillStyle=S.present?'rgba(130,150,170,.55)':'rgba(255,120,120,.9)';g.font='11px system-ui';
 g.fillText((S.present?'panda live':'PANDA STALE')+'  ·  cluster '+S.status.text+'  ·  emulation '+(S.emu===null?'?':S.emu?'on':'off')+(S.default?'  ·  no lane data':'')+'  ·  tap: next view',W/2,H-12);g.textAlign='left'}
var SM={},SMM={},ST=0,SO=[],SMD=null,SU=0,SP=0,GN=0;
function ease(dt,tau){return 1-Math.exp(-dt/tau)}
function cl(v,m){return Math.max(-m,Math.min(m,v))}
function smooth(){ // cluster-style cars: glide (predicted between the 10 Hz updates), hold through short gaps, coast + fade out when lost, fade / slide in when new
 var now=Date.now()/1000,dt=ST?Math.min(0.25,now-ST):0.1;ST=now;SO=[];if(!S)return;var id,q,k=ease(dt,0.5);
 if(SU!==SP){SP=SU;var seen={};   // a new data frame arrived
  var c0=(S.lanes&&S.lanes.c)?S.lanes.c[0]:0,cur={};(S.obj||[]).forEach(function(o){cur[o.slot+':'+o.id]=1});
  (S.obj||[]).forEach(function(o){id=o.slot+':'+o.id;q=SM[id];
   var oy=c0-o.dy;   // the cluster frame is RIGHT-positive and centred on the lane (car code: dy = curv0 - yRel): back to real metres, left-positive, like the lanes drawn here
   if(q&&(Math.abs(q.tx-o.dx)>20||Math.abs(q.ty-oy)>3)){SM[id+'#'+(GN++)]=q;q.lost=q.lost||now;q=null}   // a jump = another car: the old one coasts out, the new one starts
   if(!q){var j,c,hit=null;for(j in SM){c=SM[j];if(cur[j]||seen[j])continue;   // a track this frame no longer has: the same car moved to another slot (right lane -> lead on a cut-in) or came back under a new id
     if((o.id>0&&o.id<61&&c.rid===o.id)||(Math.abs(c.tx-o.dx)<6&&Math.abs(c.ty-oy)<2.5)){hit=j;break}}
    if(hit!==null){q=SM[hit];delete SM[hit];q.lost=0}}   // keep it: it glides from where it was instead of a new car fading in
   if(!q){var near=o.dx<40,side=oy>=0?1:-1;q={dx:o.dx,dy:near?oy+side*1.6:oy,tx:o.dx,ty:oy,vx:0,vy:0,tu:now,al:0,fin:near?0.6:0.25}}   // close (cut-in): slow fade, slides in from its own side; far: quick fade, it is small anyway
   else{var du=Math.max(0.05,now-q.tu);q.vx=cl(q.vx+((o.dx-q.tx)/du-q.vx)*0.5,30);q.vy=cl(q.vy+((oy-q.ty)/du-q.vy)*0.5,3);q.tx=o.dx;q.ty=oy;q.tu=now}
   q.lost=0;q.slot=o.slot;q.rel=o.relevant;q.rid=o.id;SM[id]=q;seen[id]=1});
  for(id in SM)if(!seen[id]&&!SM[id].lost)SM[id].lost=now}
 for(id in SM){q=SM[id];
  if(q.lost){q.tx+=q.vx*dt;q.ty+=q.vy*dt;q.tu=now;if(now-q.lost>0.5)q.al-=dt/0.7;if(q.al<=0||q.tx<-3){delete SM[id];continue}}   // hold 0.5 s, then fade over 0.7 s while it keeps moving
  else q.al=Math.min(1,q.al+dt/q.fin);
  var pt=Math.min(0.3,now-q.tu)+0.45;q.dx+=(q.tx+q.vx*pt-q.dx)*k;q.dy+=(q.ty+q.vy*pt-q.dy)*k;   // aim ahead by the smoothing lag: smooth AND where the car really is
  SO.push({dx:q.dx,dy:q.dy,slot:q.slot,relevant:q.rel,al:Math.max(0,Math.min(1,q.al))})}
 var M=S.model;if(!M){SMD=null;return}
 ['l','r','p'].forEach(function(n){var a=M[n],b=SMM[n];if(!a){SMM[n]=null;return}if(!b||b.length!=a.length){SMM[n]=a.slice();return}for(var i=0;i<a.length;i++)b[i]+=(a[i]-b[i])*ease(dt,0.2)});
 SMD={l:SMM.l,r:SMM.r,p:SMM.p,e:M.e,pl:M.pl,pr:M.pr}}
function limitSign(cx,cy,r,v){g.fillStyle='#fff';g.beginPath();g.arc(cx,cy,r,0,6.2832);g.fill();g.strokeStyle='#d0202a';g.lineWidth=r*0.18;g.beginPath();g.arc(cx,cy,r*0.86,0,6.2832);g.stroke();g.fillStyle='#111';g.font='700 '+Math.round(r*0.85)+'px system-ui';g.textAlign='center';g.fillText(v,cx,cy+r*0.3);g.textAlign='left'}
function arrow(cx,cy,s,dir,on){g.fillStyle=on?'#33d17a':'rgba(120,130,142,.22)';g.beginPath();g.moveTo(cx+dir*s,cy);g.lineTo(cx,cy-s*0.8);g.lineTo(cx,cy-s*0.35);g.lineTo(cx-dir*s*0.9,cy-s*0.35);g.lineTo(cx-dir*s*0.9,cy+s*0.35);g.lineTo(cx,cy+s*0.35);g.lineTo(cx,cy+s*0.8);g.closePath();g.fill()}
function extras(px,py,pw,ph){var B=S.body||{},blink=(Date.now()%800)<450,i;
 arrow(px-24,py+ph*0.46,14,-1,S.bl&&blink);arrow(px+pw+24,py+ph*0.46,14,1,S.br&&blink);
 if(B.limit){if(px>=100)limitSign(px-72,py+ph*0.46,24,B.limit);else if(px>=56)limitSign(Math.max(23,px-62),py+ph*0.46,20,B.limit);else limitSign(W-26,py+ph+16,20,B.limit)}
 var a=Math.max(-3,Math.min(3,S.a||0)),bw=pw*0.36,bx=px+pw/2,by=py+ph-9;   // acceleration: right/green = speeding up, left/red = slowing, full = 3 m/s2
 g.fillStyle='rgba(255,255,255,.08)';g.fillRect(bx-bw/2,by,bw,4);g.fillStyle=a>=0?'#33d17a':'#ff5a5f';g.fillRect(a>=0?bx:bx+a/3*bw/2,by,Math.abs(a)/3*bw/2,4);
 var ch=[],D=B.doors||[],nm=['FL','FR','RL','RR'],warn='rgba(205,120,20,.92)';
 ch.push(S.en?{t:'openpilot ON',bg:'rgba(40,110,210,.92)'}:{t:'openpilot off',bg:'rgba(70,78,90,.85)'});
 for(i=0;i<4;i++)if(D[i])ch.push({t:'door '+nm[i]+' open',bg:warn});
 if(B.frunk)ch.push({t:'frunk open',bg:warn});if(B.trunk)ch.push({t:'trunk open',bg:warn});
 if(B.pbrake)ch.push({t:'park brake',bg:'rgba(180,40,40,.92)'});if(B.rev)ch.push({t:'R',bg:'rgba(90,90,90,.92)'});
 if(S.brk)ch.push({t:'brake',bg:'rgba(180,40,40,.92)'});if(B.wiper)ch.push({t:'wipers',bg:'rgba(60,120,170,.92)'});
 if(B.light&&B.light!='day')ch.push({t:B.light,bg:'rgba(60,70,110,.92)'});
 if(B.inside!=null)ch.push({t:'in '+B.inside.toFixed(1)+' \u00b0C',bg:'rgba(50,58,70,.92)'});
 g.font='600 12px system-ui';var rows=[[]],rw=[0],maxw=W-16;   // chips wrap onto more rows on a narrow screen
 ch.forEach(function(c){var w=g.measureText(c.t).width+20,r=rows.length-1;if(rows[r].length&&rw[r]+6+w>maxw){rows.push([]);rw.push(0);r++}rw[r]+=(rows[r].length?6:0)+w;rows[r].push(c)});
 rows.forEach(function(row,k){var cx0=W/2-rw[k]/2,cy0=py+ph+40+k*30;row.forEach(function(c){cx0+=chip(cx0,cy0,c.t,'#fff',c.bg)+6})});
 if(S.al1){var col=S.ast==2?'rgba(200,30,40,.93)':S.ast==1?'rgba(210,140,20,.93)':'rgba(30,40,55,.93)',bw2=Math.min(W*0.8,560),bh=S.al2?62:42,bx2=W/2-bw2/2,by2=H-bh-34;
  g.fillStyle=col;rr(bx2,by2,bw2,bh,12);g.fill();g.fillStyle='#fff';g.textAlign='center';g.font='700 18px system-ui';g.fillText(S.al1,W/2,by2+26);
  if(S.al2){g.font='500 14px system-ui';g.fillText(S.al2,W/2,by2+48)}g.textAlign='left'}}
function draw(){smooth();if(mode==0)drawDrive();else drawDiag()}
function conn(){var es=new EventSource('/stream');es.onmessage=function(e){S=JSON.parse(e.data);SU=Date.now()/1000};es.onerror=function(){es.close();setTimeout(conn,1500)}}
var LF=0;function loop(t){if(t-LF>=33){LF=t;draw()}requestAnimationFrame(loop)}   // ~30 fps: data arrives at 10 Hz, the smoothing fills the frames in between
rs();conn();requestAnimationFrame(loop);</script></body></html>)LIGHTDASHPAGE";

struct Client { int fd; bool sse; };

struct Stats {                                         // LIGHTDASH_STATS=1: timings printed to stderr every 5 s (off by default)
  bool on = false; double last_print = 0, last_tick = 0;
  double tick_gap_max = 0, accept_ms_max = 0, drain_ms_max = 0, drain_ms_sum = 0, tick_ms_max = 0; long drain_msgs_max = 0, drain_msgs_sum = 0, ticks = 0, dropped = 0, conns = 0;
};
static Stats g_st;
static volatile sig_atomic_t g_quit = 0;
static void on_sig(int) { g_quit = 1; }

static bool send_all(int fd, const char *p, size_t n) {
  while (n) {
    ssize_t k = send(fd, p, n, MSG_NOSIGNAL);
    if (k < 0) { if (errno == EINTR) continue; return false; }       // EAGAIN on a stuck client: drop it, never block
    p += k; n -= k;
  }
  return true;
}

static std::string http_reply(int code, const char *ctype, const std::string &body) {
  char h[256];
  int n = snprintf(h, sizeof h, "HTTP/1.1 %d OK\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n", code, ctype, body.size());
  return std::string(h, n) + body;
}
struct Conn { int fd; double t0; std::string in, out; size_t off; bool stream, unused; };

static int selftest() {
  Mirror m; uint8_t b[8]; m.lanesFrame(b);
  char hex[32]; for (int i = 0; i < 7; i++) sprintf(hex + 2 * i, "%02x", b[i]);
  bool ok = strcmp(hex, "d0a1647f7f7ff0") == 0;
  printf("default lane frame: %s (the real 21:31 dump: d0a1647f7f7ff0) -> %s\n", hex, ok ? "OK" : "MISMATCH");
  // a 0x557 as the car packs it: lines seen both sides (q3), width byte 0xB, C0 byte 0x6E, C1..C3 mid, range 28 (30 m / 4 -> 7)
  uint8_t g557[8] = {0x50, 0x80, 0xBF, 0x6E, 0x7F, 0x80, 0x7F, 7};          // lead 40 m, dy .3, width/quality byte 0xBF, C0 0x6E
  const uint8_t f659[8] = {5, 90, 0, 0xC8, 0, 0, 0, 0};
  m.feed(0x557, g557, 8, 1.0); m.feed(0x659, f659, 8, 1.0);
  m.lanesFrame(b);
  bool ok2 = (b[0] & 3) == 3 && b[1] == 28 && b[2] == 0x6E;
  printf("lane frame after a 0x557 with both lines seen: left=%d right=%d range=%d C0byte=0x%02x -> %s\n", b[0] & 1, b[0] >> 1 & 1, b[1], b[2], ok2 ? "OK" : "MISMATCH");
  const uint8_t g554[3] = {0x20, 0, 0};                                      // as the car packs it: 3 bytes
  m.feed(0x554, g554, 3, 1.0);
  bool ok4 = m.emu == 1;
  printf("0x554 with 3 bytes sets the emulation flag: emu=%d -> %s\n", m.emu, ok4 ? "OK" : "MISMATCH");
  std::string s = snapshot(m, Extra(), 1.5); bool ok3 = s.find("\"present\":true") != std::string::npos && s.find("\"op\":5") != std::string::npos;
  printf("snapshot json: %zu bytes -> %s\n", s.size(), ok3 ? "OK" : "MISMATCH");
  CarState cs; const uint8_t b30[8] = {0x88, 0x67, 0x76, 0, 0, 0, 0, 0}, b283[8] = {0, 0, 0, 0, 0, 0, 0, 0x5F};
  cs.feed(0x30A, b30, 8, 1.0); cs.feed(0x283, b283, 8, 1.0);
  std::string sc = snapshot(m, Extra(), 1.5, cs); std::string sl = snapshot(m, Extra(), 7.0, cs);
  bool ok5 = sc.find("\"hb\":1,\"fl\":0,\"out\":7.5") != std::string::npos && sl.find("\"hb\":null,\"fl\":null,\"out\":null") != std::string::npos;
  printf("car state: 0x30A 0x88 -> high beam, 0x283 byte 7 0x5F -> 7.5 C, both dropped when stale -> %s\n", ok5 ? "OK" : "MISMATCH");
  return (ok && ok2 && ok3 && ok4 && ok5) ? 0 : 1;
}

static void fill_model(Extra &x, cereal::ModelData::Reader mm) {
  auto pts = [](cereal::ModelData::PathData::Reader pd, std::vector<float> &out) {
    out.clear(); auto p = pd.getPoints();
    if (p.size() >= 100) for (size_t i = 0; i < 100; i += 4) out.push_back(p[i]);
  };
  pts(mm.getPath(), x.p); pts(mm.getLeftLane(), x.l); pts(mm.getRightLane(), x.r);
  x.pl = mm.getLeftLane().getProb(); x.pr = mm.getRightLane().getProb();
  auto fp = mm.getFreePath(); x.have_edges = false;
  if (fp.size() >= 69 && fp[0] == 1.0f) {               // road edges first; the 0.8.12 adapter appends model FCW after them
    x.e_l.clear(); x.e_r.clear();
    for (int i = 0; i < 25; i++) { x.e_l.push_back(fp[1 + i]); x.e_r.push_back(fp[34 + i]); }
    x.have_edges = true;
  }
  x.have_model = true;
}

static bool read_event(Message *msg, kj::Array<capnp::word> &buf, capnp::FlatArrayMessageReader *&rd) {
  size_t words = msg->getSize() / sizeof(capnp::word) + 1;
  buf = kj::heapArray<capnp::word>(words);                 // aligned copy, as cereal does (works with old and new capnp)
  memcpy(buf.begin(), msg->getData(), msg->getSize());
  try { rd = new capnp::FlatArrayMessageReader(buf.asPtr()); return true; } catch (...) { rd = nullptr; return false; }
}

int main(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "--selftest")) return selftest();
  signal(SIGINT, on_sig); signal(SIGTERM, on_sig); signal(SIGPIPE, SIG_IGN);
  g_st.on = getenv("LIGHTDASH_STATS") != nullptr;
  const char *pe = getenv("LIGHTDASH_PORT"), *be = getenv("LIGHTDASH_BIND");
  int port = pe ? atoi(pe) : 8090; const char *bind_addr = be ? be : "0.0.0.0";
  int ls = socket(AF_INET, SOCK_STREAM, 0); int one = 1; setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a = {}; a.sin_family = AF_INET; a.sin_port = htons(port); inet_pton(AF_INET, bind_addr, &a.sin_addr);
  if (bind(ls, (sockaddr *)&a, sizeof a) < 0 || listen(ls, 8) < 0) { perror("lightdash: bind/listen"); return 1; }
  fcntl(ls, F_SETFL, O_NONBLOCK);
  Context *ctx = Context::create();
  SubSocket *sc = SubSocket::create(ctx, "sendcan");
  SubSocket *scan = SubSocket::create(ctx, "can");
  SubSocket *sm = SubSocket::create(ctx, "model", "127.0.0.1", true);
  SubSocket *scs = SubSocket::create(ctx, "carState", "127.0.0.1", true);
  SubSocket *sct = SubSocket::create(ctx, "controlsState", "127.0.0.1", true);
  if (!sc || !scan || !sm || !scs || !sct) { fprintf(stderr, "lightdash: cannot subscribe to openpilot messages (is openpilot running?)\n"); return 1; }
  printf("lightdash (C++) on http://%s:%d/  (sendcan + can + model; never opens the panda, never sends)\n", bind_addr, port); fflush(stdout);

  Mirror mirror; CarState car; Body body; Extra extra; std::vector<Client> clients; std::vector<Conn> pend; std::string last = "{}"; double want_until = 0, next_tick = 0;
  kj::Array<capnp::word> buf;
  while (!g_quit) {
    double now = now_s();
    bool active = !clients.empty() || now < want_until;
    std::vector<pollfd> pf; pf.push_back({ls, POLLIN, 0});
    for (auto &c : pend) pf.push_back({c.fd, (short)(c.out.empty() ? POLLIN : POLLOUT), 0});
    for (auto &c : clients) pf.push_back({c.fd, POLLIN, 0});            // POLLIN/HUP only: notice a closed browser tab at once
    int timeout = active ? (int)((next_tick > now ? next_tick - now : 0) * 1000) + 1 : 300;
    if (!pend.empty() && timeout > 50) timeout = 50;
    poll(pf.data(), pf.size(), timeout);
    now = now_s();
    // new connections: never wait for a request, each one is a small state machine serviced by poll
    for (;;) {
      int c = accept(ls, nullptr, nullptr); if (c < 0) break;
      g_st.conns++; fcntl(c, F_SETFL, O_NONBLOCK);
      if (pend.size() >= 16) { close(c); g_st.dropped++; continue; }    // flood guard
      pend.push_back(Conn{c, now, std::string(), std::string(), 0, false, false});
    }
    // clients: a readable SSE socket is a close (or junk): drop it
    for (size_t i = 0; i < clients.size();) {
      char tmp[64]; ssize_t n = recv(clients[i].fd, tmp, sizeof tmp, 0);
      if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) { close(clients[i].fd); clients.erase(clients.begin() + i); g_st.dropped++; } else i++;
    }
    for (size_t i = 0; i < pend.size();) {
      Conn &c = pend[i]; bool done = false, to_stream = false;
      if (c.out.empty()) {
        char tmp[1024]; ssize_t n = recv(c.fd, tmp, sizeof tmp, 0);
        if (n > 0) c.in.append(tmp, n);
        else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) done = true;
        if (!done && c.in.find('\n') != std::string::npos) {         // request line complete
          const char *req = c.in.c_str();
          if (!strncmp(req, "GET /stream", 11)) {
            c.out = "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nConnection: keep-alive\r\n\r\nretry: 1500\n\n"; c.stream = true;
          } else if (!strncmp(req, "GET /s ", 7) || !strncmp(req, "GET /s?", 7)) { want_until = now + 3; c.out = http_reply(200, "application/json", last); }
          else if (!strncmp(req, "GET /health", 11)) c.out = http_reply(200, "text/plain", "ok");
          else if (!strncmp(req, "GET / ", 6) || !strncmp(req, "GET /index.html", 15) || !strncmp(req, "GET /?", 6)) c.out = http_reply(200, "text/html; charset=utf-8", PAGE);
          else c.out = http_reply(404, "text/plain", "not found");
        } else if (c.in.size() > 2048) done = true;
      }
      if (!done && !c.out.empty()) {                                  // write what the socket takes now, the rest on POLLOUT
        while (c.off < c.out.size()) {
          ssize_t k = send(c.fd, c.out.data() + c.off, c.out.size() - c.off, MSG_NOSIGNAL);
          if (k > 0) c.off += k; else if (k < 0 && errno == EINTR) continue; else break;
        }
        if (c.off >= c.out.size()) { if (c.stream) to_stream = true; else done = true; }
      }
      if (now - c.t0 > 3) { done = true; g_st.dropped++; }            // slow-loris / idle / stuck reader: 3 s max
      if (to_stream) { clients.push_back({c.fd, true}); pend.erase(pend.begin() + i); }
      else if (done) { double ms = (now_s() - c.t0) * 1000; if (ms > g_st.accept_ms_max) g_st.accept_ms_max = ms; close(c.fd); pend.erase(pend.begin() + i); }
      else i++;
    }
    now = now_s();
    if (!(!clients.empty() || now < want_until) || now < next_tick) continue;
    next_tick = now + 0.1;                                              // 10 Hz
    double t_tick0 = now_s(); if (g_st.last_tick > 0 && t_tick0 - g_st.last_tick > g_st.tick_gap_max) g_st.tick_gap_max = t_tick0 - g_st.last_tick; g_st.last_tick = t_tick0;
    long nd = 0;
    // drain sendcan (every frame matters), conflated latest for the rest
    while (nd < 4096) {                                                 // bounded: a flood cannot starve the tick
      Message *msg = sc->receive(true); if (!msg) break;
      nd++;
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) {
        auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::SENDCAN) for (auto c : ev.getSendcan()) {
          uint32_t ad = c.getAddress();
          if (ad == 0x557 || ad == 0x559 || ad == 0x659 || ad == 0x554) { auto d = c.getDat(); mirror.feed(ad, d.begin(), d.size(), now); }
        }
        delete rd;
      }
      delete msg;
    }
    for (long nc = 0; nc < 4096; nc++) {                                // the car's own frames: only 0x30A / 0x283 on bus 0 are looked at
      Message *msg = scan->receive(true); if (!msg) break;
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) {
        auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::CAN) for (auto c : ev.getCan()) {
          uint32_t ad = c.getAddress();
          if ((ad == 0x30A || ad == 0x283) && c.getSrc() == 0) { auto d = c.getDat(); car.feed(ad, d.begin(), d.size(), now); }
          if ((ad == 0x318 || ad == 0x283 || ad == 0x238 || ad == 0x2F8) && c.getSrc() == 0) { auto d = c.getDat(); body.feed(ad, d.begin(), d.size(), now); }
        }
        delete rd;
      }
      delete msg;
    }
    { double ms = (now_s() - t_tick0) * 1000; g_st.drain_ms_sum += ms; if (ms > g_st.drain_ms_max) g_st.drain_ms_max = ms; g_st.drain_msgs_sum += nd; if (nd > g_st.drain_msgs_max) g_st.drain_msgs_max = nd; g_st.ticks++; }
    if (Message *msg = scs->receive(true)) {
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) { auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::CAR_STATE) { auto cs = ev.getCarState(); extra.v = cs.getVEgo(); extra.ang = cs.getSteeringAngle();
          extra.a = cs.getAEgo(); extra.bl = cs.getLeftBlinker(); extra.br = cs.getRightBlinker(); extra.gas = cs.getGasPressed(); extra.brk = cs.getBrakePressed(); }
        delete rd; }
      delete msg;
    }
    if (Message *msg = sct->receive(true)) {
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) { auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::CONTROLS_STATE) { auto ct = ev.getControlsState(); extra.en = ct.getEnabled(); extra.act = ct.getActive();
          extra.al1 = std::string(ct.getAlertText1().cStr()); extra.al2 = std::string(ct.getAlertText2().cStr()); extra.ast = (int)ct.getAlertStatus(); }
        delete rd; }
      delete msg;
    }
    if (Message *msg = sm->receive(true)) {
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) { auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::MODEL) { try { fill_model(extra, ev.getModel()); } catch (...) { extra.have_model = false; } }
        delete rd; }
      delete msg;
    }
    last = snapshot(mirror, extra, now, car, body);
    mirror.laneIdx = (mirror.laneIdx + 1) % 16;
    std::string frame = "data: " + last + "\n\n";
    for (size_t i = 0; i < clients.size();) {
      if (!send_all(clients[i].fd, frame.data(), frame.size())) { close(clients[i].fd); clients.erase(clients.begin() + i); g_st.dropped++; } else i++;
    }
    { double ms = (now_s() - t_tick0) * 1000; if (ms > g_st.tick_ms_max) g_st.tick_ms_max = ms; }
    if (g_st.on && now - g_st.last_print >= 5) {
      fprintf(stderr, "stats: ticks %ld  tick gap max %.0f ms  tick work max %.1f ms  sendcan drain: %.2f ms avg, %.1f ms max, %.1f msgs/tick avg, %ld max  connection open max %.0f ms  conns %ld  dropped clients %ld\n",
              g_st.ticks, g_st.tick_gap_max * 1000, g_st.tick_ms_max, g_st.ticks ? g_st.drain_ms_sum / g_st.ticks : 0., g_st.drain_ms_max, g_st.ticks ? (double)g_st.drain_msgs_sum / g_st.ticks : 0., g_st.drain_msgs_max, g_st.accept_ms_max, g_st.conns, g_st.dropped);
      g_st.last_print = now; g_st.tick_gap_max = g_st.accept_ms_max = g_st.drain_ms_max = g_st.tick_ms_max = 0; g_st.drain_msgs_max = 0; g_st.ticks = 0; g_st.drain_ms_sum = 0; g_st.drain_msgs_sum = 0;
    }
  }
  for (auto &c : clients) close(c.fd);
  return 0;
}
