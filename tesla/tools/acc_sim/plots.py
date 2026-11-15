import os, harness as h, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, math, re
BLUE, ORANGE, GRAY, INK, MUTED = '#2a78d6', '#eb6834', '#8a8984', '#0b0b0b', '#52514e'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': '#c9c8c2', 'axes.labelcolor': MUTED, 'xtick.color': MUTED, 'ytick.color': MUTED,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'grid.color': '#ecebe7', 'grid.linewidth': 0.6})
NEW, OLD = h.REPO + '/ACC_module.py', h.H + '/ACC_before.py'
os.makedirs('plots', exist_ok=True)
for name, sc in h.scenarios().items():
  fig, ax = plt.subplots(4, 3, figsize=(13, 9.5), sharex=True)
  for c, v0 in enumerate((60, 90, 120)):
    band = 'town' if v0 < 70 else 'hwy'; U = h.USER[band]
    for path, col, lab, tag in ((OLD, ORANGE, 'car now (d15539e)', 'before'), (NEW, BLUE, 'acc-redesign (plan + servo + safety)', 'new')):
      o = h.run(path, sc, v0, seed=1, tag=tag); t = o['t']
      jerk = [0.] + [(y - x) / 0.05 for x, y in zip(o['a'], o['a'][1:])]
      jf = []; s = 0.
      for j in jerk: s += (j - s) * 0.25; jf.append(s)                   # 0.2 s smoothing for reading only
      ax[0][c].plot(t, [x * 3.6 for x in o['v']], color=col, lw=1.6, label=lab)
      ax[1][c].plot(t, o['a'], color=col, lw=1.4)
      ax[2][c].plot(t, jf, color=col, lw=1.2)
      ax[3][c].plot(t, [d if d == d and d > -50 else float('nan') for d in o['d']], color=col, lw=1.4)
      for tc in o['cancel'][:1]:
        ax[1][c].annotate('cancel', (tc, -1.5), color=col, fontsize=7)
    ax[0][c].plot(t, [x * 3.6 for x in o['lv']], color=GRAY, lw=1.2, ls='--', label='car ahead (true)')
    for y in (U['up90'], -0.81 if band == 'town' else -0.76): ax[1][c].axhline(y, color=GRAY, lw=0.8, ls=':')
    for y in (U['jerk90'], -U['jerk90']): ax[2][c].axhline(y, color=GRAY, lw=0.8, ls=':')
    wg = [h.gap_s(x) * x for x in o['v']]; ax[3][c].plot(t, wg, color=GRAY, lw=0.9, ls=':', label='your gap x speed')
    ax[0][c].set_title('%d km/h' % v0, color=INK, fontsize=10)
    ax[1][c].set_ylim(-2, 1.4); ax[2][c].set_ylim(-1.2, 1.2); ax[3][c].set_ylim(0, None)
  ax[0][0].set_ylabel('speed km/h'); ax[1][0].set_ylabel('accel m/s2\n(dotted: your p90)'); ax[2][0].set_ylabel('jerk m/s3\n(dotted: your p90)'); ax[3][0].set_ylabel('distance m\n(dotted: your gap)')
  for c in range(3): ax[3][c].set_xlabel('time s')
  ax[0][0].legend(loc='lower left', fontsize=7, frameon=False); ax[3][0].legend(loc='lower left', fontsize=7, frameon=False)
  fig.suptitle('%s  -  simulation (not car-verified), seed 1' % name, color=INK, fontsize=12, x=0.01, ha='left')
  fig.tight_layout(); fn = 'plots/' + re.sub(r'[^a-z0-9]+', '_', name.lower()).strip('_') + '.png'; fig.savefig(fn, dpi=110); plt.close(fig); print(fn, flush=True)
