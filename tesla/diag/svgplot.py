"""Dependency-free stacked time-series plots as SVG (works on the EON, opens in any browser). stdlib only."""
import html
import math

COLORS = ('#2563eb', '#dc2626', '#16a34a', '#d97706', '#7c3aed', '#0891b2', '#be185d', '#525252')


def _nice(lo, hi):
  if hi - lo < 1e-12:
    hi = lo + 1.0
  return lo, hi


def figure(path, title, panels, vlines=(), xlabel='time (s, boot clock, relative)', width=1100, ph=150, shade=()):
  """panels: [{'ylabel':str,'series':[(label, xs, ys)]}]; vlines: [(x,label)]; shade: [(x0,x1)] highlighted spans."""
  ml, mr, mt, mb = 70, 20, 34, 40
  H = mt + len(panels) * (ph + 24) + mb
  xs_all = [x for p in panels for s in p['series'] for x in s[1]]
  if not xs_all:
    open(path, 'w').write('<svg xmlns="http://www.w3.org/2000/svg" width="400" height="40"><text x="10" y="25">no data: %s</text></svg>' % html.escape(title))
    return
  x0, x1 = _nice(min(xs_all), max(xs_all))
  W = width - ml - mr
  fx = lambda x: ml + (x - x0) / (x1 - x0) * W
  o = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="sans-serif" font-size="11">' % (width, H),
       '<rect width="100%%" height="100%%" fill="#fff"/><text x="%d" y="20" font-size="14" font-weight="600">%s</text>' % (ml, html.escape(title))]
  for i, p in enumerate(panels):
    top = mt + i * (ph + 24)
    ys_all = [y for s in p['series'] for y in s[2] if y is not None and not math.isnan(y)]
    lo, hi = _nice(min(ys_all) if ys_all else 0, max(ys_all) if ys_all else 1)
    pad = 0.06 * (hi - lo)
    lo, hi = lo - pad, hi + pad
    fy = lambda y, lo=lo, hi=hi, top=top: top + ph - (y - lo) / (hi - lo) * ph
    o.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#fafafa" stroke="#ccc"/>' % (ml, top, W, ph))
    for a, b in shade:
      o.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="#fde68a" opacity="0.35"/>' % (fx(max(a, x0)), top, max(0, fx(min(b, x1)) - fx(max(a, x0))), ph))
    for k in range(5):
      yv = lo + (hi - lo) * k / 4
      o.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#e5e5e5"/><text x="%d" y="%.1f" text-anchor="end" fill="#555">%.3g</text>'
               % (ml, ml + W, fy(yv), fy(yv), ml - 5, fy(yv) + 4, yv))
    o.append('<text x="14" y="%d" transform="rotate(-90 14 %d)" text-anchor="middle" fill="#333">%s</text>' % (top + ph / 2, top + ph / 2, html.escape(p['ylabel'])))
    for j, (lab, xs, ys) in enumerate(p['series']):
      c = COLORS[j % len(COLORS)]
      pts = ' '.join('%.1f,%.1f' % (fx(x), fy(y)) for x, y in zip(xs, ys) if y is not None and not math.isnan(y))
      if pts:
        o.append('<polyline fill="none" stroke="%s" stroke-width="1.3" points="%s"/>' % (c, pts))
      o.append('<text x="%d" y="%d" fill="%s">&#9632; %s</text>' % (ml + 6 + j * 190, top - 6 + 0, c, html.escape(lab)))
    for vx, vl in vlines:
      if x0 <= vx <= x1:
        o.append('<line x1="%.1f" x2="%.1f" y1="%d" y2="%d" stroke="#ef4444" stroke-dasharray="4 3"/>' % (fx(vx), fx(vx), top, top + ph))
        if i == 0:
          o.append('<text x="%.1f" y="%d" fill="#ef4444" font-size="10">%s</text>' % (fx(vx) + 3, top + 11, html.escape(vl)))
  base = mt + len(panels) * (ph + 24) - 8
  for k in range(7):
    xv = x0 + (x1 - x0) * k / 6
    o.append('<text x="%.1f" y="%d" text-anchor="middle" fill="#555">%.1f</text>' % (fx(xv), base + 14, xv))
  o.append('<text x="%d" y="%d" text-anchor="middle" fill="#333">%s</text></svg>' % (ml + W / 2, base + 30, html.escape(xlabel)))
  with open(path, 'w') as f:
    f.write('\n'.join(o))
