#!/bin/bash
# usage: run_logged.sh OUTDIR NAME -- command...   Runs detached; records pid/log/start/end/seconds/exit code in OUTDIR/NAME.*
out=$1; name=$2; shift 3; mkdir -p "$out"
nohup setsid bash -c '
out=$1; name=$2; shift 2
echo $$ > "$out/$name.pid"; date -u +%FT%TZ > "$out/$name.start"; s=$(date +%s)
"$@" > "$out/$name.log" 2>&1; rc=$?
echo $rc > "$out/$name.exit"; date -u +%FT%TZ > "$out/$name.end"; echo $(( $(date +%s)-s )) > "$out/$name.seconds"
' _ "$out" "$name" "$@" > /dev/null 2>&1 &
