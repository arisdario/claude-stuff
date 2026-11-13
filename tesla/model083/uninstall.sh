#!/bin/sh
# Puts back exactly what install.sh replaced (the *.075 backups) and rebuilds modeld. Run on the EON, parked.
#   sh /data/claude-stuff/tesla/model083/uninstall.sh
set -e
OP=${OP:-/data/openpilot}
MD=$OP/selfdrive/modeld/models
CAR=$OP/selfdrive/car/tesla
back() { if [ -f "$1.075" ]; then cp "$1.075" "$1"; echo "restored $1"; else echo "no backup for $1 (left as is)"; fi; }

back "$OP/models/supercombo.dlc"
back "$MD/driving.h"
back "$MD/driving.cc"
rm -f "$MD/model083_adapter.h" "$MD/model0812_adapter.h"
# PCC: the backup is what was on the car before install.sh. Keep the new version instead by running with KEEP_PCC=1
# (the curve fix only matters with the 0.8.3 model, so going back is safe either way).
if [ "${KEEP_PCC:-0}" != 1 ]; then back "$CAR/PCC_module.py"; back "$CAR/pcc_curves.py"; fi
cd "$OP" && scons -j4 selfdrive/modeld/_modeld
echo "back on the 0.7.5 model - reboot"
