#!/system/bin/sh
# lightdash_tunnel.sh - publish lightdash (read-only web page, port 8090) through a Cloudflare Tunnel, so it can be
# opened from a phone anywhere. Only lightdash is published: never the launcher (8087, has buttons) or radarweb.
#
#   NAMED tunnel (recommended: fixed address + Cloudflare Access login):
#     put the tunnel token (Cloudflare dashboard > Zero Trust > Networks > Tunnels > your tunnel) in /data/cloudflared_token
#     and in that tunnel set the public hostname -> service http://localhost:8090
#   QUICK tunnel (no account, random address that CHANGES every start, anyone with it can view): used when no token file.
#     The address is written to /data/lightdash_url.txt.
#
# Off switch: touch /data/lightdash_tunnel_off.     Log: /data/lightdash_tunnel.log
# Auto-start: add this line to /data/openpilot/launch_chffrplus.sh next to the autoclock line:
#   (sh /data/claude-stuff/tesla/tools/lightdash_tunnel.sh &)
# Needs (one time, see the chat): /system/etc/resolv.conf with nameservers, and a CA bundle (below).
set -u
BIN=${CLOUDFLARED:-/data/cloudflared}
TOKEN_FILE=/data/cloudflared_token
OFF=/data/lightdash_tunnel_off
LOG=/data/lightdash_tunnel.log
URL_FILE=/data/lightdash_url.txt

[ -f "$OFF" ] && exit 0
[ -x "$BIN" ] || { echo "$(date) no $BIN" >> "$LOG"; exit 1; }
pgrep -f "cloudflared tunnel" > /dev/null 2>&1 && exit 0          # already running

for f in /data/cacert.pem /data/data/com.termux/files/usr/etc/tls/cert.pem; do
  [ -s "$f" ] && { export SSL_CERT_FILE="$f"; break; }
done
[ -n "${SSL_CERT_FILE:-}" ] || export SSL_CERT_DIR=/system/etc/security/cacerts

while [ ! -f "$OFF" ]; do
  echo "$(date) starting tunnel" >> "$LOG"
  if [ -s "$TOKEN_FILE" ]; then
    "$BIN" tunnel --no-autoupdate run --token "$(cat "$TOKEN_FILE")" >> "$LOG" 2>&1
  else
    rm -f "$URL_FILE"
    "$BIN" tunnel --no-autoupdate --url http://localhost:8090 2>&1 | while IFS= read -r line; do
      echo "$line" >> "$LOG"
      case "$line" in *https://*trycloudflare.com*)
        echo "$line" | grep -o 'https://[a-z0-9-]*\.trycloudflare\.com' > "$URL_FILE";;
      esac
    done
  fi
  echo "$(date) tunnel exited, retry in 30 s" >> "$LOG"
  sleep 30                                    # no network yet / dropped: try again
done
