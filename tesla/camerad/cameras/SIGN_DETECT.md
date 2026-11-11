# Stop-sign / traffic-light detection (`sign_detect.h/.cc`)

Classical CV, not a second neural net - the Adreno GPU on this EON is already fully claimed by the
driving model's SNPE inference, so a second real-time net alongside it isn't realistic. This runs on
CPU: HSV color thresholding + blob shape analysis, in **plain C++ with no OpenCV** (the EON's NEOS has
no OpenCV, so the earlier OpenCV version could never be built there). Same pipeline and thresholds.

**Frames are never modified.** The capture thread only does one `memcpy` of the RGB frame (every 8th
frame, and never waits: `trylock`, skip if the worker is busy). Detection runs on a separate
low-priority thread on that copy, at half resolution (640x360), ~12 ms/frame on a desktop CPU.

**Status: validated against synthetic generated test images only (`camerad/test/ext_c920/sign_detect_test.cc`,
18 checks, all passing), NOT yet tuned against real road footage.** The synthetic tests prove the shape
math, color ranges, distance formula and false-positive filters (red truck/wall, taillight bar, a red
blob near the bottom of frame) are logically sound. They cannot prove real-world accuracy - JPEG
compression, real lighting, partial occlusion, weather, and non-idealized sign/light shapes will all
move the thresholds. **Before trusting this for anything beyond a display readout, capture real frames
and re-tune the HSV ranges in `sign_detect.cc` against them** (see "Tuning" below).

OFF by default everywhere: compiled in only if the build's trial-compile of the detector succeeds (see
camerad/SConscript's aarch64 branch - mirrors the existing `EXT_WANT_AVCODEC`/H.264 opt-in pattern
exactly: "any doubt = camerad is built exactly as before"), and even when compiled in, runs only if
`/data/sign_detect_on` exists. With neither, this is zero-cost: no extra code path, no extra CPU.

## What it does

- **Stop sign:** HSV red mask (two hue ranges - red wraps 0/180) -> morphological close/open ->
  contours -> keep ones that are convex (solidity > 0.85), simplify to 6-10 vertices (an octagon
  under perspective/approximation noise), roughly square bounding box (aspect 0.7-1.43). The largest
  qualifying one wins. Distance = `(real sign width m * 907px) / detected pixel width` - a plain
  pinhole estimate using the native C920 frame's own focal length (camera_ext_c920.cc's own comment:
  "~907 px (78 deg diagonal)", matching the EON's ~910px so the native frame needs no rescaling for
  this math).
- **Traffic light:** same per-color HSV masks (red/yellow/green), restricted to the **upper two-thirds
  of the frame** (lights are mounted up high - this is the single biggest practical defense against
  brake lights, which this classical approach otherwise cannot reliably distinguish from a lit red
  signal), kept only if the contour is small and highly circular (>0.65). Whichever color's best
  candidate is most circular is reported.

## Distance constant - READ THIS

`STOP_SIGN_WIDTH_M` in `sign_detect.h` is a generic placeholder (0.75 m), not a verified standard for
any specific country/road class. A wrong value only skews the reported distance (never causes a false
positive/negative), but get it right for your region before relying on the number.

## Data flow (read-only, never touches CAN or control)

`camera_ext_c920.cc`'s `on_frame()` -> (opt-in, rate-limited to a few Hz, not every frame) ->
`sign_detect_tick()` in `sign_detect_io.cc` -> `detect_signs()` -> atomically written to
`/data/sign_detect.json` (write-tmp-then-rename, same pattern as this file's existing `publish_live()`)
-> read by `tesla/sign_detect.py` (pure function, same shape as `steer_limits.py`) -> `carcontroller.py`'s
`checkWhichSign()` sets the *existing, previously-unfed* `stopSign_distance`/`stopLight_distance`/
`stopLight_color` fields -> the cluster's own sign display (already wired, via `create_fake_DAS_sign_msg`).
Also read directly by `lightdash.cc`/`.py` for an on-screen indicator.

## Tuning against real footage

1. `touch /data/sign_detect_on` and build with the OpenCV trial passing (see SConscript).
2. Save some real frames (there's already a live-frame path in this file, `publish_live()` ->
   `/dev/c920_live.jpg`, or capture from a drive).
3. Re-run `sign_detect_test.cc`-style checks against real frames (swap in real JPEGs for the synthetic
   `cv::Mat`s) and adjust the `Scalar(...)` HSV bounds in `red_mask`/`yellow_mask`/`green_mask` and the
   solidity/circularity/area thresholds until false positives/negatives on your real footage are
   acceptable. Don't deploy on vibes - re-run the test suite after any threshold change.
