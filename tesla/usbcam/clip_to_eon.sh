#!/bin/sh
# Turns a C920 clip recorded with   UVC_FOCUS=0 UVC_SAVE=clip.mjpg ./uvctest 046d:082d 1280 720 30 25
# into a video with the geometry the EON's driving model expects (1164x874, focal length 910 px), for tools/replay083.py on a PC:
#   sh clip_to_eon.sh clip.mjpg fcamera_c920.hevc
# 1280x720 on the C920 has a focal length of ~907 px (from the 78 deg diagonal spec - MEASURE it), almost exactly the EON's 910 px, so NO scaling is done:
# the width is cropped from 1280 to 1164 (58 px each side) and the 720 rows are centred on a black 874-row canvas. The model therefore sees a narrower VERTICAL view (43 deg instead of 51 deg).
set -e
[ -n "$2" ] || { echo "usage: sh clip_to_eon.sh clip.mjpg out.hevc"; exit 1; }
ffmpeg -y -f mjpeg -framerate 30 -i "$1" -vf "crop=1164:720:58:0,pad=1164:874:0:77:black" -c:v libx265 -pix_fmt yuv420p -an "$2"
echo "wrote $2 (1164x874). Run:  python3 ../model083/tools/replay083.py $2 --vp 582 437 --model supercombo.onnx --out replay_c920.mp4"
