#!/bin/sh
# Builds uvctest and uvcview on the EON (or any Linux with libusb-1.0 installed). Run from anywhere:  sh tesla/usbcam/build.sh
set -e
cd "$(dirname "$0")"
CC=${CC:-$(command -v clang || command -v gcc || command -v cc)}
[ -n "$CC" ] || { echo "no C compiler found"; exit 1; }
INC=""; LIB=""
for p in /data/data/com.termux/files/usr /usr/local /usr; do      # the EON's toolchain lives in termux
  if [ -f "$p/include/libusb-1.0/libusb.h" ]; then
    INC="-I$p/include/libusb-1.0"
    [ -d "$p/lib" ] && LIB="-L$p/lib -Wl,-rpath,$p/lib"
    break
  fi
done
[ -n "$INC" ] || { echo "libusb-1.0 headers not found (boardd builds against them - is this the EON?)"; exit 1; }
SRC="uvctest.c libuvc/src/ctrl.c libuvc/src/ctrl-gen.c libuvc/src/device.c libuvc/src/diag.c libuvc/src/frame.c libuvc/src/init.c libuvc/src/misc.c libuvc/src/stream.c"
FLAGS="-O2 -Ilibuvc/include $INC"
echo "building uvctest with $CC"
# Android's libc has pthread built in; plain Linux needs -lpthread
$CC $FLAGS $SRC $LIB -lusb-1.0 -lpthread -o uvctest 2>build.log || $CC $FLAGS $SRC $LIB -lusb-1.0 -o uvctest 2>>build.log || { cat build.log; exit 1; }
# uvcview: live view in a browser, for aiming the camera
SRCV=$(echo "$SRC" | sed 's#uvctest.c#uvcview.c#')
$CC $FLAGS $SRCV $LIB -lusb-1.0 -lpthread -o uvcview 2>build_view.log || $CC $FLAGS $SRCV $LIB -lusb-1.0 -o uvcview 2>>build_view.log || { cat build_view.log; exit 1; }
SRCL=$(echo "$SRC" | sed 's#uvctest.c#ledtool.c#')
$CC $FLAGS $SRCL $LIB -lusb-1.0 -lpthread -o ledtool 2>build_led.log || $CC $FLAGS $SRCL $LIB -lusb-1.0 -o ledtool 2>>build_led.log || { cat build_led.log; exit 1; }
SRCD=$(echo "$SRC" | sed 's#uvctest.c#uvcdump.c#')
$CC $FLAGS $SRCD $LIB -lusb-1.0 -lpthread -o uvcdump 2>build_dump.log || $CC $FLAGS $SRCD $LIB -lusb-1.0 -o uvcdump 2>>build_dump.log || { cat build_dump.log; exit 1; }
SRCF=$(echo "$SRC" | sed 's#uvctest.c#fmtbench.c#')
# fmtbench: with libjpeg for the MJPEG decode timing; without it only that one number is missing
$CC $FLAGS $SRCF $LIB -lusb-1.0 -lpthread -ljpeg -o fmtbench 2>build_fmt.log || $CC $FLAGS $SRCF $LIB -lusb-1.0 -ljpeg -o fmtbench 2>>build_fmt.log || $CC $FLAGS $SRCF $LIB -lusb-1.0 -lpthread -o fmtbench 2>>build_fmt.log || $CC $FLAGS $SRCF $LIB -lusb-1.0 -o fmtbench 2>>build_fmt.log || { cat build_fmt.log; exit 1; }
echo "built: $(pwd)/uvctest, $(pwd)/uvcview, $(pwd)/ledtool and $(pwd)/uvcdump"
