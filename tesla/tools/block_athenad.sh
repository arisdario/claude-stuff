#!/bin/sh
# block_athenad.sh - stop openpilot from starting athenad (the comma-cloud client) by adding manage_athenad to the launch script's /data/no_<name> block list.
#   sh block_athenad.sh           dry run: shows what it would change
#   sh block_athenad.sh --yes     backs up the launch script ONCE, adds the name, creates /data/no_manage_athenad. Takes effect at the next start of openpilot (reboot)
#   sh block_athenad.sh --undo    restores the backup and removes the marker file
# It edits ONE word list in /data/openpilot/launch_chffrplus.sh and nothing else. If that line is not exactly as expected it stops and changes nothing.
LAUNCH=${LAUNCH:-/data/openpilot/launch_chffrplus.sh}; BAK=${BAK:-/data/launch_chffrplus.sh.bak_before_block}; MARK=${MARK:-/data/no_manage_athenad}
OLD='for p in loggerd uploader deleter mapcurved; do'
NEW='for p in loggerd uploader deleter mapcurved manage_athenad; do'
[ -f "$LAUNCH" ] || { echo "STOP: $LAUNCH not found"; exit 1; }
case "$1" in
  --undo)
    [ -f "$BAK" ] && cp "$BAK" "$LAUNCH" && echo "launch script restored from $BAK" || echo "no backup at $BAK: launch script left as it is"
    rm -f "$MARK"; echo "marker removed. Reboot to start athenad again."; exit 0;;
  --yes) ;;
  "") if grep -qF "$NEW" "$LAUNCH"; then echo "already patched (manage_athenad is in the list)."
      elif grep -qF "$OLD" "$LAUNCH"; then echo "would: back up $LAUNCH to $BAK (if no backup yet), change the list to:"; echo "  $NEW"; echo "      and create $MARK"
      else echo "the block loop is not in the expected form: it would change nothing."; fi
      echo "dry run only. Add --yes to apply."; exit 0;;
  *) echo "usage: sh $0 [--yes|--undo]"; exit 2;;
esac
if grep -qF "$NEW" "$LAUNCH"; then
  echo "already patched."
else
  grep -qF "$OLD" "$LAUNCH" || { echo "STOP: the block loop is not in the expected form; nothing changed"; exit 1; }
  [ -f "$BAK" ] || cp "$LAUNCH" "$BAK"
  sed -i "s/$OLD/$NEW/" "$LAUNCH"
  grep -qF "$NEW" "$LAUNCH" || { cp "$BAK" "$LAUNCH"; echo "edit failed; original restored"; exit 1; }
fi
touch "$MARK"
echo "launch script now has:"; grep -n -F "$NEW" "$LAUNCH"
echo "marker: $(ls $MARK)    backup: $BAK"
echo "Reboot to apply. After boot: ps | grep athena | grep -v grep   (no output = blocked).  Undo: sh $0 --undo"
