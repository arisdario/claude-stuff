#!/bin/bash
# Builds libmpc1.so (openpilot v0.7.5 longitudinal MPC, its own generated ACADO code) for the simulator, from an
# openpilot v0.7.5 checkout:   ./build_libmpc.sh /path/to/openpilot-v0.7.5
set -e
OP=$(cd "$1" && pwd); M=$OP/selfdrive/controls/lib/longitudinal_mpc; Q=$OP/phonelibs/qpoases
HERE=$(cd "$(dirname "$0")" && pwd); B=$(mktemp -d); cd "$B"
for f in $Q/SRC/Bounds.cpp $Q/SRC/Constraints.cpp $Q/SRC/CyclingManager.cpp $Q/SRC/Indexlist.cpp $Q/SRC/MessageHandling.cpp \
         $Q/SRC/QProblem.cpp $Q/SRC/QProblemB.cpp $Q/SRC/SubjectTo.cpp $Q/SRC/Utils.cpp $Q/SRC/EXTRAS/SolutionAnalysis.cpp \
         $M/lib_mpc_export/acado_qpoases_interface.cpp; do
  g++ -c -O2 -fPIC -w -I$Q/INCLUDE -I$Q/INCLUDE/EXTRAS -I$Q/SRC -I$Q -I$M/lib_mpc_export $f -o $(basename $f .cpp).o
done
for f in $M/longitudinal_mpc.c $M/lib_mpc_export/acado_solver.c $M/lib_mpc_export/acado_integrator.c $M/lib_mpc_export/acado_auxiliary_functions.c; do
  gcc -c -O2 -fPIC -w -I$Q/INCLUDE -I$M/lib_mpc_export $f -o $(basename $f .c).o
done
g++ -shared -o "$HERE/libmpc1.so" *.o -lm && ls -la "$HERE/libmpc1.so"
