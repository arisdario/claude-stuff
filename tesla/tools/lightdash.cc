// lightdash.cc - C++ twin of tesla/tools/lightdash.py: ONE small read-only web page showing what the panda tells the instrument cluster.
// Same page, same JSON, same byte-for-byte replay of the firmware's 0x557/0x559/0x659 -> 0x239/0x309 expansion (safety_tesla.h, v1.7.3).
// Reads ONLY openpilot messages (sendcan, model, carState, controlsState) - it never opens USB or the panda, sends nothing, writes no file,
// and does no work while no browser is connected. Single thread, poll(), no outside libraries beyond cereal's own (capnp, zmq/msgq).
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
#include <time.h>
#include <unistd.h>

#include <string>
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
  bool have_model = false; std::vector<float> p, l, r, e_l, e_r; float pl = 0, pr = 0; bool have_edges = false;
};

static std::string snapshot(const Mirror &m, const Extra &x, double now) {
  std::string s; s.reserve(1400);
  bool stale = m.stale(now);
  appendf(s, "{\"present\":%s,\"emu\":%s,", stale ? "false" : "true", m.emu < 0 ? "null" : (m.emu ? "true" : "false"));
  if (m.t659 != 0) appendf(s, "\"age\":%.1f,", now - m.t659); else s += "\"age\":null,";
  appendf(s, "\"default\":%s,\"status\":{\"op\":%d,\"long\":%d,\"cc\":%d,\"alca\":%d,\"hands\":%d,\"acc\":%d,\"text\":\"%s\"},",
          (!m.have557 || stale) ? "true" : "false", m.op, m.lng, m.cc, m.alca, m.hands, m.acc, op_text(m.op));
  uint8_t lf[8]; m.lanesFrame(lf); json_lanes(s, lf); s += ",";
  json_objects(s, m);
  appendf(s, ",\"v\":%.3f,\"ang\":%.3f,\"en\":%s,", x.v, x.ang, x.en ? "true" : "false");
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
var cv=document.getElementById('c'),g=cv.getContext('2d'),S=null,tdv=false,W=0,H=0,SC=2;
function rs(){var d=Math.min(window.devicePixelRatio||1,1.5);W=innerWidth;H=innerHeight;cv.width=W*d;cv.height=H*d;g.setTransform(d,0,0,d,0,0);draw()}
addEventListener('resize',rs);cv.addEventListener('click',function(){tdv=!tdv;draw()});
function P(x,y){ // real metres (x ahead, y left) -> screen
 if(tdv){var s=H*0.8/110;return [W/2-y*s*4,H*0.92-x*s]}
 var K=H*0.9,d=x+5;return [W/2-y*K/d,H*0.38+2.5*K/d]}
function line(pts,col,w,dash){if(pts.length<2)return;g.beginPath();g.setLineDash(dash||[]);g.strokeStyle=col;g.lineWidth=w;pts.forEach(function(p,i){i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke();g.setLineDash([])}
function poly(c,x){return c[0]+c[1]*x+c[2]*x*x+c[3]*x*x*x}
function rr(x,y,w,h,r){g.beginPath();g.moveTo(x+r,y);g.lineTo(x+w-r,y);g.quadraticCurveTo(x+w,y,x+w,y+r);g.lineTo(x+w,y+h-r);g.quadraticCurveTo(x+w,y+h,x+w-r,y+h);g.lineTo(x+r,y+h);g.quadraticCurveTo(x,y+h,x,y+h-r);g.lineTo(x,y+r);g.quadraticCurveTo(x,y,x+r,y);g.closePath()}
function carBack(cx,y0,w,rel){ // a car seen from behind; y0 = where its tyres touch the road, w = 1.8 m of real width
 var h=w*0.8,body=rel?'#b8484b':'#7d8792',dark='#12161b',glass='#1b2733';
 if(w<14){g.fillStyle=body;g.fillRect(cx-w/2,y0-h,w,h);return}
 g.save();g.translate(cx,y0);g.scale(1,0.12);g.fillStyle='rgba(0,0,0,.45)';g.beginPath();g.arc(0,0,w*0.56,0,6.2832);g.fill();g.restore();
 g.fillStyle=dark;rr(cx-w*0.46,y0-h*0.30,w*0.17,h*0.30,w*0.03);g.fill();rr(cx+w*0.29,y0-h*0.30,w*0.17,h*0.30,w*0.03);g.fill();
 g.fillStyle=body;rr(cx-w/2,y0-h*0.58,w,h*0.40,w*0.07);g.fill();
 g.beginPath();g.moveTo(cx-w*0.41,y0-h*0.56);g.lineTo(cx-w*0.30,y0-h);g.lineTo(cx+w*0.30,y0-h);g.lineTo(cx+w*0.41,y0-h*0.56);g.closePath();g.fill();
 g.fillStyle=glass;g.beginPath();g.moveTo(cx-w*0.35,y0-h*0.61);g.lineTo(cx-w*0.26,y0-h*0.93);g.lineTo(cx+w*0.26,y0-h*0.93);g.lineTo(cx+w*0.35,y0-h*0.61);g.closePath();g.fill();
 g.fillStyle=rel?'#ff5a5f':'#9a4a4d';rr(cx-w*0.47,y0-h*0.50,w*0.21,h*0.10,w*0.02);g.fill();rr(cx+w*0.26,y0-h*0.50,w*0.21,h*0.10,w*0.02);g.fill();
 g.fillStyle='#cfd6dd';g.fillRect(cx-w*0.10,y0-h*0.33,w*0.20,h*0.07);
 g.fillStyle='rgba(0,0,0,.35)';rr(cx-w/2,y0-h*0.23,w,h*0.05,w*0.02);g.fill()}
function carTop(cx,yr,w,l,rel){ // seen from above, rear edge at screen y=yr, nose pointing up (4.5 m long, 1.8 m wide)
 var body=rel?'#b8484b':'#7d8792',dark='#12161b',glass='#1b2733';
 g.fillStyle=dark;rr(cx-w*0.53,yr-l*0.84,w*0.10,l*0.18,w*0.03);g.fill();rr(cx+w*0.43,yr-l*0.84,w*0.10,l*0.18,w*0.03);g.fill();rr(cx-w*0.53,yr-l*0.20,w*0.10,l*0.18,w*0.03);g.fill();rr(cx+w*0.43,yr-l*0.20,w*0.10,l*0.18,w*0.03);g.fill();
 g.fillStyle=body;rr(cx-w/2,yr-l,w,l,w*0.30);g.fill();
 g.fillStyle=glass;rr(cx-w*0.38,yr-l*0.66,w*0.76,l*0.22,w*0.10);g.fill();rr(cx-w*0.36,yr-l*0.30,w*0.72,l*0.13,w*0.08);g.fill();
 g.fillStyle=rel?'#ff5a5f':'#9a4a4d';rr(cx-w*0.46,yr-l*0.06,w*0.22,l*0.04,w*0.02);g.fill();rr(cx+w*0.24,yr-l*0.06,w*0.22,l*0.04,w*0.02);g.fill()}
function draw(){g.fillStyle='#0b0d10';g.fillRect(0,0,W,H);if(!S||!S.lanes){g.fillStyle='#9aa4af';g.fillText('waiting for data...',20,30);return}
 var L=S.lanes,R=Math.min(110,L.range*SC),pw=[],i;
 // grid
 g.strokeStyle='#1b2128';g.lineWidth=1;for(i=10;i<=100;i+=10){var a=P(i,-6),b=P(i,6);g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.stroke()}
 var M=S.model; // the model's own, faint
 if(M){var xs=[];for(i=0;i<25;i++)xs.push(i*4);
  if(M.l)line(M.l.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.r)line(M.r.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.p){var cl=M.p.map(function(y,k){return P(xs[k],y-.9)}),cr=M.p.map(function(y,k){return P(xs[k],y+.9)}).reverse();g.fillStyle='rgba(60,200,120,.12)';g.beginPath();cl.concat(cr).forEach(function(p,k){k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.fill()}
  if(M.e){line(M.e.l.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2);line(M.e.r.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2)}}
 // what the cluster is told: virtual lane = centre curve +- width/2 (cluster x = real/2)
 var cc=[],lf=[],rt=[];for(i=0;i<=R;i+=2){var xc=i/SC,yc=poly(L.c,xc);cc.push(P(i,yc));lf.push(P(i,yc+L.width/2));rt.push(P(i,yc-L.width/2))}
 line(cc,'rgba(255,255,255,.18)',1,[6,8]);
 line(lf,L.left?'#ffd23f':'#555e68',L.left?5:2,L.left?null:[10,10]);line(rt,L.right?'#ffd23f':'#555e68',L.right?5:2,L.right?null:[10,10]);
 // lead / side vehicles
 S.obj.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w,h;
  if(tdv){h=Math.max(26,4.5*H*0.8/110*1.6);w=Math.max(12,h/2.2);carTop(p[0],p[1],w,h,o.relevant)}   // an icon with car proportions: the top view stretches the sideways scale 4x, so true scale would look squashed
  else{w=Math.max(12,1.8*H*0.9/(o.dx+5));h=w*0.8;carBack(p[0],p[1],w,o.relevant)}
  g.fillStyle='#e8edf2';g.textAlign='center';g.fillText(o.slot+' '+Math.round(o.dx)+' m',p[0],p[1]-h-6);g.textAlign='left'});
 // header
 var st=S.status,y0=22;g.font='15px system-ui';g.fillStyle='#e8edf2';
 g.fillText('openpilot '+(S.en?'ENGAGED':'off')+'   speed '+Math.round((S.v||0)*3.6)+' km/h   wheel '+(S.ang||0).toFixed(1)+' deg',16,y0);
 g.fillStyle=S.present?'#7bd88f':'#e5383b';g.fillText('panda data: '+(S.present?'live':('STALE ('+S.age+' s) - cluster is shown defaults')),16,y0+20);
 g.fillStyle='#e8edf2';g.fillText('cluster status: '+st.text+'   emulation: '+(S.emu===null?'?':S.emu?'on':'OFF (panda sends nothing)'),16,y0+40);
 g.fillText('lanes: width '+L.width.toFixed(2)+' m  left '+(L.left?'drawn':'not drawn')+'  right '+(L.right?'drawn':'not drawn')+'  range '+L.range+' (x'+SC+')'+(S.default?'   [DEFAULT frame - no lane data from openpilot]':''),16,y0+60);
 g.fillStyle='#9aa4af';g.font='12px system-ui';g.fillText('yellow = lane lines sent to the cluster   cyan = model lanes   green = model path   red = road edges   tap = switch view',16,H-14)}
function conn(){var es=new EventSource('/stream');es.onmessage=function(e){S=JSON.parse(e.data);draw()};es.onerror=function(){es.close();setTimeout(conn,1500)}}
rs();conn();</script></body></html>)LIGHTDASHPAGE";

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
  return (ok && ok2 && ok3 && ok4) ? 0 : 1;
}

static void fill_model(Extra &x, cereal::ModelData::Reader mm) {
  auto pts = [](cereal::ModelData::PathData::Reader pd, std::vector<float> &out) {
    out.clear(); auto p = pd.getPoints();
    if (p.size() >= 100) for (size_t i = 0; i < 100; i += 4) out.push_back(p[i]);
  };
  pts(mm.getPath(), x.p); pts(mm.getLeftLane(), x.l); pts(mm.getRightLane(), x.r);
  x.pl = mm.getLeftLane().getProb(); x.pr = mm.getRightLane().getProb();
  auto fp = mm.getFreePath(); x.have_edges = false;
  if (fp.size() == 69 && fp[0] == 1.0f) {
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
  SubSocket *sm = SubSocket::create(ctx, "model", "127.0.0.1", true);
  SubSocket *scs = SubSocket::create(ctx, "carState", "127.0.0.1", true);
  SubSocket *sct = SubSocket::create(ctx, "controlsState", "127.0.0.1", true);
  if (!sc || !sm || !scs || !sct) { fprintf(stderr, "lightdash: cannot subscribe to openpilot messages (is openpilot running?)\n"); return 1; }
  printf("lightdash (C++) on http://%s:%d/  (sendcan + model; never opens the panda)\n", bind_addr, port); fflush(stdout);

  Mirror mirror; Extra extra; std::vector<Client> clients; std::vector<Conn> pend; std::string last = "{}"; double want_until = 0, next_tick = 0;
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
    { double ms = (now_s() - t_tick0) * 1000; g_st.drain_ms_sum += ms; if (ms > g_st.drain_ms_max) g_st.drain_ms_max = ms; g_st.drain_msgs_sum += nd; if (nd > g_st.drain_msgs_max) g_st.drain_msgs_max = nd; g_st.ticks++; }
    if (Message *msg = scs->receive(true)) {
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) { auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::CAR_STATE) { extra.v = ev.getCarState().getVEgo(); extra.ang = ev.getCarState().getSteeringAngle(); }
        delete rd; }
      delete msg;
    }
    if (Message *msg = sct->receive(true)) {
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) { auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::CONTROLS_STATE) { extra.en = ev.getControlsState().getEnabled(); }
        delete rd; }
      delete msg;
    }
    if (Message *msg = sm->receive(true)) {
      capnp::FlatArrayMessageReader *rd; if (read_event(msg, buf, rd)) { auto ev = rd->getRoot<cereal::Event>();
        if (ev.which() == cereal::Event::MODEL) { try { fill_model(extra, ev.getModel()); } catch (...) { extra.have_model = false; } }
        delete rd; }
      delete msg;
    }
    last = snapshot(mirror, extra, now);
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
