# C920 frame path: profiling-only investigation (no code changes)

Goal: lower **frame age and delivery jitter** (not file size). Nothing here changes the codec, frame rate, geometry, model input or steering.

## 1. The path of one frame (read from `camera_ext_c920.cc`, openpilot 0.7.5.a `camerad/main.cc`, `transforms/rgb_to_yuv.cl`, `modeld/modeld.cc`)

| # | Stage | Where | Data moved | Notes |
|---|---|---|---|---|
| 1 | USB payloads -> libuvc frame buffer | libuvc thread (CPU) | ~130-200 KB copy | 1 copy; plus a hold-buffer swap, no second copy |
| 2 | **JPEG decode to RGB** (`decode_jpeg`) | CPU | write 2.76 MB (1280x720x3) | libjpeg, IFAST, no fancy upsampling. `jpeg_create/destroy_decompress` allocates and frees libjpeg's internal tables every frame |
| 3 | 180 deg flip, in place (only if `/data/camera_flip=1`, i.e. your camera) | CPU | read+write 2.76 MB | an extra full pass over the frame |
| 4 | luma sample for the profile engine | CPU | ~3.6 K pixels, once per second | negligible |
| 5 | **`ext_c920_pack_bgr`**: RGB->BGR, crop 58 px/side, pad 77 rows | CPU | read 2.5 MB, write 3.05 MB (1164x874x3) | per-pixel byte shuffle |
| 6 | `visionbuf_sync(TO_DEVICE)` | CPU cache flush | 3.05 MB | cost is the ION cache maintenance, not measurable off-device |
| 7 | `publish_live` (web page) | CPU | fopen/fwrite 130-200 KB/rename, ~10/s | only while radarweb or the logger needs it; cheap to switch off |
| 8 | GPU copy of the BGR buffer for the UI (`rgb_bufs`) | GPU | 3.05 MB | not CPU |
| 9 | `rgb_to_yuv.cl`: BGR -> limited-range BT.601 I420, 2x2 chroma average | **GPU** | 3.05 MB -> 1.53 MB | the model's input is made here |
| 10 | `visionbuf_sync(FROM_DEVICE)` | CPU cache invalidate | 1.53 MB | |
| 11 | `modeld`: `memcpy(yuv_ion.addr, buf->addr, 1.53 MB)` | CPU, in modeld | 1.53 MB, every frame | stock behaviour (so ~31 MB/s of copying) |

Allocations per frame: only libjpeg's internal ones (stage 2). All big buffers (`g_rgb`, vision buffers) are allocated once.

## 2. Is direct YCbCr output compatible with the model input?
The model gets **limited-range BT.601 I420** made by `rgb_to_yuv.cl` from the BGR buffer: `Y = 16 + (13B+65G+33R)/128`, chroma from the average of each 2x2 block of the *canvas* (the picture starts on odd canvas row 77, so the top and bottom chroma rows mix picture and padding in the stock path as well).
libjpeg can give planar full-range JFIF YCbCr (the C920's JPEG is normally 4:2:2). Converting that to the same I420 needs: range scaling (Y x 219/255 + 16, C x 224/255 + 128), 4:2:2 -> 4:2:0 on the canvas grid, crop/pad placement and the 180 degree turn.
`camprof` implements that conversion and compares it with the stock result on the same frame: **Y mean difference ~0.5 level (max 3), U/V mean ~0.2 (max 6)** on test frames (to be repeated on a real C920 frame). So the content is compatible within a level or two but not bit-identical; any change would have to be validated on model outputs.
**The catch:** the BGR buffer is not only for the model. The UI draws the road camera from it (stage 8) and the GPU kernel (stage 9) reads it. Producing I420 directly on the CPU would therefore *add* work (the UI still needs BGR) unless the UI's camera view and the GPU kernel are also reworked - and moving stage 9 to the CPU would cost more CPU than it saves (`cpu_rgb2i420` below).

## 3. Measure (on the EON, car parked, openpilot running; the test takes a couple of seconds)
```sh
cd /data/claude-stuff/tesla/camerad/profile && sh build.sh
./camprof                       # uses /dev/c920_live.jpg (a real frame); prints one line per stage
./camprof /dev/c920_live.jpg 300
```
Read it like this (CPU share = ms per frame x 20 fps):
* `decode_rgb` is today's decode. `decode_raw` is the same without libjpeg's colour conversion/upsampling: **the difference is the most any YCbCr-direct design could save inside libjpeg**.
* `hdr_only` is libjpeg's per-frame setup (allocation) cost.
* `flip180`, `pack_bgr` are the two extra full-frame passes camerad adds (the first only because the camera is mounted upside down).
* `memcpy_vb` is the floor for touching every byte of the vision buffer once: if `pack_bgr` is close to it, there is nothing left to gain by tuning it.
* `memcpy_yuv` is modeld's copy; it is not camerad's, but it is part of the frame's latency budget.
* `cpu_rgb2i420` shows why moving stage 9 off the GPU is not attractive; `cpu_direct_i420` is the cost of the direct-YCbCr conversion.

## 4. Estimate before proposing anything
Possible savings, to be filled from the EON numbers: (a) `decode_rgb - decode_raw`, only if BGR for the UI/GPU path could still be produced cheaply; (b) fuse flip + pack_bgr into one pass (removes `flip180`, one full read+write of 2.76 MB); (c) skip `publish_live` when nothing consumes it; (d) reuse the libjpeg decompress object (removes `hdr_only`). (b)-(d) change no pixel and no timing of the output; (a) changes the pixels slightly and the architecture a lot.
None of them is applied. Frame **age** is dominated by exposure (up to 50 ms at night), not by these CPU stages.
