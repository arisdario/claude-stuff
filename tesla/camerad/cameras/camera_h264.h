#ifndef CAMERA_H264_H
#define CAMERA_H264_H
// Optional H.264 input for the external C920 road camera (the default stays MJPEG; /data/camera_format = "h264" selects this).
//  - scan_nals / yuv420_to_rgb: pure helpers, unit-tested off the EON (tests: test/ext_c920/test_h264.py).
//  - Decoder: libavcodec loaded at RUN time with dlopen, only when H.264 is selected. camerad is NOT linked against libavcodec, so a missing or
//    odd ffmpeg on the phone can never stop the MJPEG path from starting. Any failure here is reported and camerad falls back to MJPEG.
//    The library major version is checked against the headers it was compiled with; a mismatch is refused.
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

namespace h264 {

// Annex-B scan of one access unit: bit 0 = contains an IDR slice (nal type 5), bit 1 = contains an SPS (type 7)
inline int scan_nals(const uint8_t *d, size_t n) {
  int r = 0;
  for (size_t i = 0; i + 3 < n; i++) {
    if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1) {
      int t = d[i + 3] & 31;
      if (t == 5) r |= 1;
      if (t == 7) r |= 2;
      i += 2;
    }
  }
  return r;
}

static inline uint8_t clip8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v); }

// planar YUV 4:2:0 -> RGB24, BT.601, chroma replicated (the same convention as the MJPEG path: libjpeg YCbCr->RGB with fancy upsampling off).
// full_range = JPEG-style 0..255 (the C920's H.264 is yuvj420p); otherwise video-range 16..235.
struct YuvTables {                                 // the same fixed-point math as the direct form, with the per-chroma-sample products tabulated
  int crr[256], cbb[256], crg[256], cbg[256];
  YuvTables() {
    for (int i = 0; i < 256; i++) {
      int c = i - 128;
      crr[i] = (91881 * c + 32768) >> 16; cbb[i] = (116130 * c + 32768) >> 16; crg[i] = 46802 * c; cbg[i] = 22554 * c + 32768;
    }
  }
};

inline void yuv420_to_rgb(const uint8_t *Y, int ys, const uint8_t *U, int us, const uint8_t *V, int vs, int w, int h, bool full_range, uint8_t *rgb) {
  if (full_range && (w & 1) == 0) {
    static const YuvTables T;
    for (int y = 0; y < h; y++) {
      const uint8_t *yr = Y + (size_t)y * ys, *ur = U + (size_t)(y >> 1) * us, *vr = V + (size_t)(y >> 1) * vs;
      uint8_t *o = rgb + (size_t)y * w * 3;
      for (int x = 0; x < w; x += 2, o += 6) {
        const int u = ur[x >> 1], v = vr[x >> 1];
        const int dr = T.crr[v], db = T.cbb[u], dg = (T.cbg[u] + T.crg[v]) >> 16;
        int l = yr[x];
        o[0] = clip8(l + dr); o[1] = clip8(l - dg); o[2] = clip8(l + db);
        l = yr[x + 1];
        o[3] = clip8(l + dr); o[4] = clip8(l - dg); o[5] = clip8(l + db);
      }
    }
    return;
  }
  for (int y = 0; y < h; y++) {
    const uint8_t *yr = Y + (size_t)y * ys, *ur = U + (size_t)(y >> 1) * us, *vr = V + (size_t)(y >> 1) * vs;
    uint8_t *o = rgb + (size_t)y * w * 3;
    for (int x = 0; x < w; x++, o += 3) {
      int cb = ur[x >> 1] - 128, cr = vr[x >> 1] - 128, l = yr[x];
      if (full_range) {
        o[0] = clip8(l + ((91881 * cr + 32768) >> 16));
        o[1] = clip8(l - ((22554 * cb + 46802 * cr + 32768) >> 16));
        o[2] = clip8(l + ((116130 * cb + 32768) >> 16));
      } else {
        int yy = ((l - 16) * 76309 + 32768) >> 16;                     // 255/219 in 16.16
        o[0] = clip8(yy + ((104597 * cr + 32768) >> 16));              // 1.596
        o[1] = clip8(yy - ((25675 * cb + 53279 * cr + 32768) >> 16));  // 0.391, 0.813
        o[2] = clip8(yy + ((132201 * cb + 32768) >> 16));              // 2.018
      }
    }
  }
}

}  // namespace h264

// Only when the build asks for it (-DEXT_WANT_AVCODEC, set by SConscript after a trial compile of this header succeeded on the phone). Without that
// flag nothing below is compiled and camera_ext_c920.cc is the MJPEG-only camerad it always was.
#if defined(EXT_WANT_AVCODEC) && defined(__has_include)
#if __has_include(<libavcodec/avcodec.h>) && __has_include(<libavutil/frame.h>)
#define EXT_HAVE_AVCODEC 1
#endif
#endif

#ifdef EXT_HAVE_AVCODEC
#include <dlfcn.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
}

namespace h264 {

class Decoder {
 public:
  Decoder() {}
  ~Decoder() { close(); }

  // dir: where libavcodec/libavutil live (env EXT_AVCODEC_DIR, else the phone's /system/comma/usr/lib, else the default library path)
  bool open(char *why, size_t whylen) {
    close();
    const char *e = getenv("EXT_AVCODEC_DIR");
    const char *dirs[3] = {e && *e ? e : "/system/comma/usr/lib", e && *e ? NULL : "", NULL};   // EXT_AVCODEC_DIR set: only there (tests); else the phone's dir, then the default library path
    char maj_c[16], maj_u[16];
    snprintf(maj_c, sizeof maj_c, "%d", LIBAVCODEC_VERSION_MAJOR);
    snprintf(maj_u, sizeof maj_u, "%d", LIBAVUTIL_VERSION_MAJOR);
    for (int i = 0; dirs[i] && !(hc_ && hu_); i++) {
      const char *sl = *dirs[i] ? "/" : "";
      char p[512];
      for (int k = 0; k < 2 && !hc_; k++) {
        if (k == 0) snprintf(p, sizeof p, "%s%slibavcodec.so", dirs[i], sl); else snprintf(p, sizeof p, "%s%slibavcodec.so.%s", dirs[i], sl, maj_c);
        hc_ = dlopen(p, RTLD_NOW | RTLD_LOCAL);
      }
      for (int k = 0; k < 2 && hc_ && !hu_; k++) {
        if (k == 0) snprintf(p, sizeof p, "%s%slibavutil.so", dirs[i], sl); else snprintf(p, sizeof p, "%s%slibavutil.so.%s", dirs[i], sl, maj_u);
        hu_ = dlopen(p, RTLD_NOW | RTLD_LOCAL);
      }
      if (hc_ && !hu_) { dlclose(hc_); hc_ = NULL; }                  // both from the same directory or neither
    }
    if (!hc_ || !hu_) { snprintf(why, whylen, "libavcodec/libavutil not found (set EXT_AVCODEC_DIR or install them)"); close(); return false; }
    typedef unsigned (*verfn)(void);
    verfn vc = (verfn)dlsym(hc_, "avcodec_version"), vu = (verfn)dlsym(hu_, "avutil_version");
    if (!vc || !vu || (vc() >> 16) != (unsigned)LIBAVCODEC_VERSION_MAJOR || (vu() >> 16) != (unsigned)LIBAVUTIL_VERSION_MAJOR) {
      snprintf(why, whylen, "libavcodec/libavutil major version differs from the headers camerad was built with (lib %u/%u, headers %d/%d): refused", vc ? vc() >> 16 : 0, vu ? vu() >> 16 : 0, LIBAVCODEC_VERSION_MAJOR, LIBAVUTIL_VERSION_MAJOR);
      close(); return false;
    }
#define SYM(h, f) f_##f = (decltype(&f))dlsym(h, #f); if (!f_##f) { snprintf(why, whylen, "symbol %s missing", #f); close(); return false; }
    SYM(hc_, avcodec_find_decoder) SYM(hc_, avcodec_alloc_context3) SYM(hc_, avcodec_open2) SYM(hc_, avcodec_send_packet) SYM(hc_, avcodec_receive_frame)
    SYM(hc_, avcodec_flush_buffers) SYM(hc_, avcodec_free_context) SYM(hc_, av_packet_alloc) SYM(hc_, av_packet_free)
    SYM(hu_, av_frame_alloc) SYM(hu_, av_frame_free)
#undef SYM
    const AVCodec *codec = f_avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) { snprintf(why, whylen, "this libavcodec has no H.264 decoder"); close(); return false; }
    codec_ = codec;
    if (!make_ctx()) { snprintf(why, whylen, "avcodec_open2 failed"); close(); return false; }
    pkt_ = f_av_packet_alloc(); fr_ = f_av_frame_alloc();
    if (!pkt_ || !fr_) { snprintf(why, whylen, "out of memory"); close(); return false; }
    need_idr_ = true; errors_ = 0; dropped_ = 0; rx_ = 0; decoded_ = 0; stats_reset();
    return true;
  }

  void close() {
    if (fr_ && f_av_frame_free) f_av_frame_free(&fr_);
    if (pkt_ && f_av_packet_free) f_av_packet_free(&pkt_);
    if (ctx_ && f_avcodec_free_context) f_avcodec_free_context(&ctx_);
    fr_ = NULL; pkt_ = NULL; ctx_ = NULL;
    if (hu_) { dlclose(hu_); hu_ = NULL; }
    if (hc_) { dlclose(hc_); hc_ = NULL; }
  }

  bool is_open() const { return ctx_ != NULL; }
  long errors() const { return errors_; }
  long dropped() const { return dropped_; }
  long rx() const { return rx_; }                 // access units handed to decode() since open()
  long decoded() const { return decoded_; }       // pictures that came out good
  double dec_ms() const { return decoded_stat_ ? dec_ns_ / 1e6 / decoded_stat_ : 0; }     // libavcodec time per good picture, since the last stats_reset()
  double conv_ms() const { return decoded_stat_ ? conv_ns_ / 1e6 / decoded_stat_ : 0; }   // YUV -> RGB time per good picture
  void stats_reset() { dec_ns_ = conv_ns_ = 0; decoded_stat_ = 0; }

  // One UVC frame (= one access unit, Annex B). Returns true when `rgb` (w x h x 3) holds a good picture.
  // Safety: nothing is output until an IDR frame has been seen, and after ANY decode error or a frame the decoder itself flags as damaged
  // the decoder is flushed and frames are dropped until the next IDR, so a broken reference chain never reaches the model.
  bool decode(const uint8_t *data, size_t len, int w, int h, uint8_t *rgb) {
    if (!ctx_ || len < 8) return false;
    rx_++;
    int nals = scan_nals(data, len);
    if (need_idr_ && !(nals & 1)) { dropped_++; return false; }
    const int64_t t0 = mono_ns();
    pkt_->data = (uint8_t *)data; pkt_->size = (int)len;
    int r = f_avcodec_send_packet(ctx_, pkt_);
    if (r < 0) return fail();
    bool got = false;
    for (;;) {
      r = f_avcodec_receive_frame(ctx_, fr_);
      if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) break;
      if (r < 0) return fail();
      bool bad = false;
#ifdef AV_FRAME_FLAG_CORRUPT
      if (fr_->flags & AV_FRAME_FLAG_CORRUPT) bad = true;
#endif
      if (fr_->decode_error_flags) bad = true;
      if (bad) return fail();
      if (fr_->width != w || fr_->height != h || (fr_->format != AV_PIX_FMT_YUV420P && fr_->format != AV_PIX_FMT_YUVJ420P)) return fail();
      bool full = fr_->format == AV_PIX_FMT_YUVJ420P || fr_->color_range == AVCOL_RANGE_JPEG;
      const int64_t t1 = mono_ns();
      yuv420_to_rgb(fr_->data[0], fr_->linesize[0], fr_->data[1], fr_->linesize[1], fr_->data[2], fr_->linesize[2], w, h, full, rgb);
      const int64_t t2 = mono_ns();
      dec_ns_ += t1 - t0; conv_ns_ += t2 - t1; decoded_stat_++;
      got = true;
    }
    if (got) { need_idr_ = false; decoded_++; }
    return got;
  }

 private:
  bool make_ctx() {
    if (ctx_) f_avcodec_free_context(&ctx_);
    ctx_ = f_avcodec_alloc_context3(codec_);
    if (!ctx_) return false;
    ctx_->thread_count = 1;                       // as measured with `ffmpeg -threads 1`; also no frame-threading delay
    ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    if (f_avcodec_open2(ctx_, codec_, NULL) < 0) { f_avcodec_free_context(&ctx_); return false; }
    return true;
  }
  // any error: a fresh decoder (not just a flush - a flush after damage left the next pictures slightly off in the test) and wait for the next IDR
  bool fail() { errors_++; need_idr_ = true; if (ctx_ && !make_ctx()) { /* ctx_ is NULL now: decode() returns false from here on */ } return false; }
  void *hc_ = NULL, *hu_ = NULL;
  const AVCodec *codec_ = NULL;
  AVCodecContext *ctx_ = NULL;
  AVPacket *pkt_ = NULL;
  AVFrame *fr_ = NULL;
  static int64_t mono_ns() { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec; }
  bool need_idr_ = true;
  long errors_ = 0, dropped_ = 0, rx_ = 0, decoded_ = 0, decoded_stat_ = 0;
  int64_t dec_ns_ = 0, conv_ns_ = 0;
  decltype(&avcodec_find_decoder) f_avcodec_find_decoder = NULL;
  decltype(&avcodec_alloc_context3) f_avcodec_alloc_context3 = NULL;
  decltype(&avcodec_open2) f_avcodec_open2 = NULL;
  decltype(&avcodec_send_packet) f_avcodec_send_packet = NULL;
  decltype(&avcodec_receive_frame) f_avcodec_receive_frame = NULL;
  decltype(&avcodec_flush_buffers) f_avcodec_flush_buffers = NULL;
  decltype(&avcodec_free_context) f_avcodec_free_context = NULL;
  decltype(&av_packet_alloc) f_av_packet_alloc = NULL;
  decltype(&av_packet_free) f_av_packet_free = NULL;
  decltype(&av_frame_alloc) f_av_frame_alloc = NULL;
  decltype(&av_frame_free) f_av_frame_free = NULL;
};

}  // namespace h264
#endif  // EXT_HAVE_AVCODEC
#endif
