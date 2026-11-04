#!/bin/sh
# builds camprof on the EON (or any Linux with libjpeg headers). Does not touch openpilot.
cd "$(dirname "$0")" || exit 1
CC=${CC:-$(command -v clang || command -v gcc || command -v cc)}
for p in /data/data/com.termux/files/usr /usr/local /usr; do
  [ -f "$p/include/jpeglib.h" ] && { INC="-I$p/include"; LIB="-L$p/lib -Wl,-rpath,$p/lib"; break; }
done
$CC -O2 $INC camprof.c $LIB -ljpeg -o camprof && echo "built $(pwd)/camprof"
