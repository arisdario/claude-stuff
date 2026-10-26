#!/usr/bin/env python3
# canmarks.py - reads a canraw.py recording and shows which CAN bytes change right after each mark (e.g. "screen 2"),
# while staying quiet the rest of the time. Use it to find the messages behind a button press or a cluster screen.
# Read-only, needs nothing but Python 3 (3.7 OK).
#
#   python3 canmarks.py /data/canraw/can_20261026_101500.bin.gz            # per mark: bytes that changed
#   python3 canmarks.py FILE --window 3 --quiet 0.3                         # look 3 s after each mark; background <= 0.3 changes/s
#   python3 canmarks.py FILE --id 0x3c2                                     # every change of one address, with marks in between
#
# A byte is listed for a mark when its value right before the mark differs from a value within --window seconds after
# it, and it changes at most --quiet times per second outside the mark windows (counters and checksums are filtered out).
import argparse, gzip, json, struct, sys


def read(path):
  f = gzip.open(path, 'rb'); hdr = json.loads(f.readline().decode())
  data = f.read(); i = 0; frames = []; marks = []
  while i < len(data):
    typ = data[i]
    if typ == 1:
      if i + 9 > len(data): break
      _, t, bus, addr, n = struct.unpack_from('<BIBHB', data, i); i += 9
      frames.append((t / 1000., bus, addr, data[i:i + n])); i += n
    elif typ == 2:
      i += 15
    elif typ == 4:
      if i + 7 > len(data): break
      _, t, n = struct.unpack_from('<BIH', data, i); i += 7
      marks.append((t / 1000., data[i:i + n].decode('utf-8', 'replace'))); i += n
    else:
      break                                   # truncated tail (power loss)
  return hdr, frames, marks


def bus_name(b):
  return ('EON>%d' % (b - 128)) if b >= 128 else str(b)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('file')
  ap.add_argument('--window', type=float, default=2.0, help='seconds after each mark to look for changes')
  ap.add_argument('--quiet', type=float, default=0.2, help='max background changes per second for a byte to count')
  ap.add_argument('--id', type=lambda s: int(s, 0), default=None, help='print every change of this address')
  a = ap.parse_args()
  hdr, frames, marks = read(a.file)
  dur = frames[-1][0] if frames else 0.
  print('%s  label=%r  %.0f s  %d frames  %d marks' % (a.file, hdr.get('label'), dur, len(frames), len(marks)))

  if a.id is not None:
    ev = sorted([(t, 0, '%-6s 0x%03x  %s' % (bus_name(b), ad, d.hex(' ') if sys.version_info >= (3, 8) else d.hex()))
                 for t, b, ad, d in frames if ad == a.id] + [(t, 1, '------ MARK: ' + m) for t, m in marks])
    last = {}
    for t, _, s in ev:
      if s.startswith('------') or last.get(s[:12]) != s:
        print('%8.2f  %s' % (t, s)); last[s[:12]] = s
    return

  inwin = lambda t: any(tm - 0.2 <= t <= tm + a.window for tm, _ in marks)
  bg_time = max(1., dur - len(marks) * (a.window + 0.2))
  prev = {}; changes = {}                                          # (bus, addr, byte) -> [(t, old, new)]
  for t, b, ad, d in frames:
    p = prev.get((b, ad)); prev[(b, ad)] = d
    if p is None: continue
    for k in range(max(len(p), len(d))):
      o = p[k] if k < len(p) else None; n = d[k] if k < len(d) else None
      if o != n: changes.setdefault((b, ad, k), []).append((t, o, n))
  quiet = {key for key, ch in changes.items() if sum(1 for c in ch if not inwin(c[0])) / bg_time <= a.quiet}
  hits = {}
  for tm, label in marks:
    print('\n%7.2f s  MARK: %s' % (tm, label))
    found = False
    for key in sorted(quiet):
      ch = [c for c in changes[key] if tm < c[0] <= tm + a.window]
      if not ch: continue
      found = True; hits[key] = hits.get(key, 0) + 1
      vals = ' -> '.join(['%02x' % ch[0][1] if ch[0][1] is not None else '--'] + ['%02x' % c[2] if c[2] is not None else '--' for c in ch[:6]])
      print('   bus %-6s 0x%03x byte %d: %s   (+%.2f s)' % (bus_name(key[0]), key[1], key[2], vals, ch[0][0] - tm))
    if not found: print('   (no quiet byte changed)')
  if marks:
    print('\nchanged at most marks (address, byte: marks hit / %d):' % len(marks))
    for key, n in sorted(hits.items(), key=lambda kv: -kv[1])[:20]:
      print('   bus %-6s 0x%03x byte %d: %d' % (bus_name(key[0]), key[1], key[2], n))


if __name__ == '__main__':
  main()
