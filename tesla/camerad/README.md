# camerad as running on the owner's EON (mirror, 3 Nov 2026)

A copy of `/data/openpilot/selfdrive/camerad/` SOURCE from the EON (openpilot 0.7.5.a). It was compared with the stock 0.7.5 tree and is **identical** (main.cc, SConscript, camera_qcom.c/.h, camera_common.h,
camera_webcam.cc/.h, rgb_to_yuv.c: 0 differing lines). Left out on purpose: the compiled `camerad`, the `.o` files, `snapshot/libvisionipc.so`, and the `.075` backup copy of camera_qcom.c.
Purpose: build the external-camera (C920) option against exactly what runs on the EON. Do not edit these files for a deployment; changes go in new files or in `tesla/usbcam/`.

Note (3 Nov): the EON's `selfdrive/car/tesla` is BEHIND this repo (e.g. `mapcurved.py` 210 lines there vs 479 here; `PCC_module.py`, `carcontroller.py` and two tests also differ). Deploy from the repo, do not copy back from the EON.
