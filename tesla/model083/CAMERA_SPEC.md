# Camera geometry the driving model expects

These are the intrinsics baked into the openpilot pipeline on this EON (`common/transformations/camera.py`
eon_intrinsics, mirrored in `tools/replay083.py`). Any replacement camera must match this geometry, or the model's
path / lanes / lead come out wrong — the model is trained for one specific field of view.

| Parameter            | Value                               |
|----------------------|-------------------------------------|
| Full frame           | 1164 x 874 px (4:3)                 |
| Focal length         | 910 px  (focal/width = 0.782)       |
| Horizontal FOV       | **65.2 deg**  (the critical spec)   |
| Vertical FOV         | 51.3 deg                            |
| Diagonal FOV         | 77.3 deg                            |
| Principal point      | (582, 437) = exact image centre     |
| Camera height (model)| 1.22 m above the road               |

## Requirements for a replacement camera

1. **HFOV ~ 65 deg (60-70).** NOT a wide-angle dashcam lens (120-150 deg) and not a telephoto. This is make-or-break.
2. **~4:3 aspect**, or a wider sensor cropped to 4:3.
3. **Resolution >= 1164 x 874** so frames are only ever downscaled to the model frame, never upscaled.
4. **Lens focal length ~ 0.78 x sensor width (mm):**
   - 1/3"  (4.80 mm) -> ~3.75 mm
   - 1/2.7" (5.37 mm) -> ~4.20 mm
   - 1/2.5" (5.76 mm) -> ~4.50 mm
   - 1/2.3" (6.17 mm) -> ~4.82 mm
5. **Fixed focus at infinity — not autofocus** (autofocus drift caused the blurry bench images on the OnePlus 3T).
6. **Global shutter, or fast rolling shutter** (rolling shutter smears at speed).
7. **Good low-light / HDR sensor** — the actual night benefit; bigger pixels and HDR readout.
8. **Mount ~1.2 m high**, horizon near image centre. Calibration corrects small pitch/yaw via the vanishing point
   (`/data/params/d/CalibrationParams`), but large offsets waste usable FOV.
9. **Fixed or well-behaved exposure** (not one that hunts).

## Caveats (beyond geometry)

- If the FOV is matched, bringing a new camera online is: pipe its frames into `camerad`/`modeld` in place of the
  built-in feed, then recalibrate (just the vanishing point). If the FOV differs, you must also update the intrinsic
  matrix K (fx, cx, cy) so the warp to the medmodel frame is correct; large FOV differences still degrade the model
  because the road appears at a different scale than training.
- On the EON specifically, USB is the hard part: the kernel has **no uvcvideo driver**, so a UVC camera must be driven
  in userspace over libusb (the `tesla/usbcam/` experiment, paused) before any of the above applies.
- Night weakness is partly a camera-*count* problem (the comma three adds wide + IR cameras). A single better low-light
  camera helps sensitivity but does not add the wide/IR advantage.
