// uvcview - live view of a UVC camera (C920) in a web browser, to AIM the camera: MJPEG stream + guide lines + a focus slider.
// Standalone like uvctest: does not touch openpilot, camerad or the panda. No password: use it on your own network, only while aiming, then Ctrl+C.
//   ./uvcview 046d:082d [WIDTH HEIGHT FPS] [PORT]      then open  http://<EON ip>:8090/  on a laptop or phone on the same network
//   UVC_FOCUS=100 ./uvcview ...   start with a focus value (0 = infinity .. 250 = closest); the slider changes it live
//   UVC_FAKE=picture.jpg ./uvcview 0:0 1280 720 10 8090   test the page without a camera
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "libuvc/libuvc.h"

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t newframe = PTHREAD_COND_INITIALIZER;
static unsigned char *jpg = NULL;
static size_t jpg_len = 0, jpg_cap = 0;
static unsigned long seq = 0;
static uvc_device_handle_t *devh = NULL;
static int cam_w = 1280, cam_h = 720, focus_now = -1;
static volatile int running = 1;

static void publish(const unsigned char *d, size_t n) {
  pthread_mutex_lock(&lock);
  if (n > jpg_cap) { jpg = realloc(jpg, n); jpg_cap = n; }
  memcpy(jpg, d, n); jpg_len = n; seq++;
  pthread_cond_broadcast(&newframe);
  pthread_mutex_unlock(&lock);
}
static void on_frame(uvc_frame_t *f, void *u) { (void)u; if (f->frame_format == UVC_FRAME_FORMAT_MJPEG) publish(f->data, f->data_bytes); }

static const char *PAGE =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'><title>C920 aim</title>"
"<style>body{margin:0;background:#111;color:#ddd;font:15px system-ui}#w{position:relative;display:inline-block;line-height:0}img{max-width:100vw;height:auto}"
".l{position:absolute;background:rgba(255,60,60,.9)}.b{position:absolute;border:2px dashed rgba(60,255,120,.9);box-sizing:border-box}#c{padding:10px;line-height:1.5}</style>"
"<div id=w><img id=i src=/stream>"
"<div class=l style='left:50%;top:0;width:1px;height:100%'></div><div class=l style='left:0;top:50%;width:100%;height:1px'></div>"
"<div class=b style='left:4.53%;width:90.94%;top:0;height:100%'></div></div>"
"<div id=c>Red lines = centre. Model view = <b>green box</b> (the EON model uses the middle 1164 of 1280 px; its horizon is the red HORIZONTAL line, so put the real horizon / the road's vanishing point on it).<br>"
"Focus <input id=f type=range min=0 max=250 value=0 oninput='s(this.value)' style='width:50%'> <b id=v>0</b> (0 = infinity: use this on the road; a screen 35 cm away needs about 100-150)<br>"
"<a href=/snap.jpg style='color:#8cf'>snapshot</a></div>"
"<script>function s(v){document.getElementById('v').textContent=v;fetch('/focus?v='+v)}fetch('/state').then(r=>r.json()).then(j=>{if(j.focus>=0){f.value=j.focus;v.textContent=j.focus}})</script>";

static int send_all(int fd, const void *b, size_t n) {
  const char *p = b;
  while (n) { ssize_t k = send(fd, p, n, MSG_NOSIGNAL); if (k <= 0) return -1; p += k; n -= (size_t)k; }
  return 0;
}

static void *client(void *arg) {
  int fd = (int)(long)arg;
  char req[1024]; ssize_t n = recv(fd, req, sizeof(req) - 1, 0);
  if (n <= 0) { close(fd); return NULL; }
  req[n] = 0;
  char hdr[256];
  if (!strncmp(req, "GET /stream", 11)) {
    { const char *sh = "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-cache\r\n\r\n"; send_all(fd, sh, strlen(sh)); }
    unsigned long last = 0;
    while (running) {
      pthread_mutex_lock(&lock);
      while (running && seq == last) pthread_cond_wait(&newframe, &lock);
      size_t len = jpg_len; unsigned char *copy = len ? malloc(len) : NULL;
      if (copy) memcpy(copy, jpg, len);
      last = seq;
      pthread_mutex_unlock(&lock);
      if (!copy) continue;
      int h = snprintf(hdr, sizeof(hdr), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", len);
      int bad = send_all(fd, hdr, (size_t)h) || send_all(fd, copy, len) || send_all(fd, "\r\n", 2);
      free(copy);
      if (bad) break;
    }
  } else if (!strncmp(req, "GET /snap.jpg", 13)) {
    pthread_mutex_lock(&lock);
    size_t len = jpg_len; unsigned char *copy = len ? malloc(len) : NULL;
    if (copy) memcpy(copy, jpg, len);
    pthread_mutex_unlock(&lock);
    int h = snprintf(hdr, sizeof(hdr), "HTTP/1.0 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", len);
    send_all(fd, hdr, (size_t)h); if (copy) send_all(fd, copy, len);
    free(copy);
  } else if (!strncmp(req, "GET /focus?v=", 13)) {
    int v = atoi(req + 13); if (v < 0) v = 0; if (v > 250) v = 250;
    int r = devh ? (int)uvc_set_focus_abs(devh, (uint16_t)v) : 0;
    if (r == 0) focus_now = v;
    int h = snprintf(hdr, sizeof(hdr), "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nContent-Length: 20\r\n\r\n");
    send_all(fd, hdr, (size_t)h); char body[32]; snprintf(body, sizeof(body), "{\"ok\":%d,\"f\":%4d}   ", r == 0, v); send_all(fd, body, 20);
  } else if (!strncmp(req, "GET /state", 10)) {
    char body[64]; int bl = snprintf(body, sizeof(body), "{\"focus\":%d,\"w\":%d,\"h\":%d}", focus_now, cam_w, cam_h);
    int h = snprintf(hdr, sizeof(hdr), "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n", bl);
    send_all(fd, hdr, (size_t)h); send_all(fd, body, (size_t)bl);
  } else if (!strncmp(req, "GET / ", 6) || !strncmp(req, "GET /index", 10)) {
    int h = snprintf(hdr, sizeof(hdr), "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: %zu\r\n\r\n", strlen(PAGE));
    send_all(fd, hdr, (size_t)h); send_all(fd, PAGE, strlen(PAGE));
  } else {
    { const char *nf = "HTTP/1.0 404 Not Found\r\nContent-Length: 0\r\n\r\n"; send_all(fd, nf, strlen(nf)); }
  }
  close(fd);
  return NULL;
}

static void *fake_source(void *arg) {
  FILE *f = fopen((const char *)arg, "rb"); if (!f) { perror("UVC_FAKE"); exit(2); }
  unsigned char *b = malloc(8 << 20); size_t n = fread(b, 1, 8 << 20, f); fclose(f);
  while (running) { publish(b, n); usleep(100000); }
  return NULL;
}

static void stop(int s) { (void)s; running = 0; pthread_cond_broadcast(&newframe); }

int main(int argc, char **argv) {
  int vid = 0, pid = 0, fps = 30, port = 8090, a = 1;
  if (argc > a && strchr(argv[a], ':')) { sscanf(argv[a], "%x:%x", &vid, &pid); a++; }
  if (argc > a + 2) { cam_w = atoi(argv[a]); cam_h = atoi(argv[a + 1]); fps = atoi(argv[a + 2]); a += 3; }
  if (argc > a) port = atoi(argv[a]);
  signal(SIGINT, stop); signal(SIGTERM, stop); signal(SIGPIPE, SIG_IGN);
  const char *fake = getenv("UVC_FAKE");
  uvc_context_t *ctx = NULL; uvc_device_t *dev = NULL; uvc_stream_ctrl_t ctrl; uvc_error_t r;
  if (fake) { pthread_t t; pthread_create(&t, NULL, fake_source, (void *)fake); }
  else {
    if ((r = uvc_init(&ctx, NULL)) < 0) { uvc_perror(r, "uvc_init"); return 1; }
    if ((r = uvc_find_device(ctx, &dev, vid, pid, NULL)) < 0) { uvc_perror(r, "camera not found (check the USB list)"); return 2; }
    if ((r = uvc_open(dev, &devh)) < 0) { uvc_perror(r, "uvc_open - run as root; is another program using the camera?"); return 3; }
    uvc_set_ae_priority(devh, 0); uvc_set_focus_auto(devh, 0);
    const char *want = getenv("UVC_FOCUS"); uint16_t cur = 0;
    if (want && *want) uvc_set_focus_abs(devh, (uint16_t)atoi(want));
    if (uvc_get_focus_abs(devh, &cur, UVC_GET_CUR) == UVC_SUCCESS) focus_now = cur;
    if ((r = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, cam_w, cam_h, fps)) < 0) { uvc_perror(r, "no MJPEG mode of that size"); return 4; }
    if ((r = uvc_start_streaming(devh, &ctrl, on_frame, NULL, 0)) < 0) { uvc_perror(r, "uvc_start_streaming"); return 5; }
  }
  int s = socket(AF_INET, SOCK_STREAM, 0), one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in sa; memset(&sa, 0, sizeof(sa)); sa.sin_family = AF_INET; sa.sin_addr.s_addr = htonl(INADDR_ANY); sa.sin_port = htons((uint16_t)port);
  if (bind(s, (struct sockaddr *)&sa, sizeof(sa)) < 0 || listen(s, 4) < 0) { perror("bind/listen"); return 6; }
  printf("uvcview: %dx%d @ %d fps. Open  http://<EON ip>:%d/  on a laptop or phone on the same network (no password: only while aiming). Ctrl+C to stop.\n", cam_w, cam_h, fps, port);
  fflush(stdout);
  while (running) {
    fd_set fs; FD_ZERO(&fs); FD_SET(s, &fs); struct timeval tv = {1, 0};
    if (select(s + 1, &fs, NULL, NULL, &tv) <= 0) continue;
    int c = accept(s, NULL, NULL); if (c < 0) continue;
    pthread_t t; pthread_attr_t at; pthread_attr_init(&at); pthread_attr_setdetachstate(&at, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&t, &at, client, (void *)(long)c) != 0) close(c);
  }
  if (devh) { uvc_stop_streaming(devh); uvc_close(devh); }
  if (dev) uvc_unref_device(dev);
  if (ctx) uvc_exit(ctx);
  printf("stopped\n");
  return 0;
}
