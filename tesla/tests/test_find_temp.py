import json, os, random, sys, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import find_temp as F


def make_log(path, outside, inside, seed, rows=80, hide=True, inside_to=None):
  """a fake maplog: random chatter on many addresses, a counter, a clock - and, hidden, 0x2e5 byte 3 = (outside+40)*2 and 0x2e5 byte 5 = inside*10/ 5 (16-bit would be harder)"""
  rnd = random.Random(seed)
  with open(path, 'w') as f:
    for i in range(rows):
      raw = {}
      for a in range(0x100, 0x140):
        raw['0x%x' % a] = bytes(rnd.randrange(256) for _ in range(8)).hex()
      raw['0x318'] = bytes([0x9a, 0x0b, i % 60, 0x12, 4, (i // 60) % 60, 0x40, 1]).hex()          # a clock
      raw['0x1f0'] = bytes([i & 0xFF, 0, 0, 0, 0, 0, 0, 0]).hex()                                  # a counter
      if hide:
        t_out = outside + 0.05 * (i % 3); t_in = inside if inside_to is None else inside + (inside_to - inside) * i / float(rows - 1)
        raw['0x2e5'] = bytes([1, 2, 3, int(round((t_out + 40) * 2)), 7, int(round(t_in * 5)), 9, 9]).hex()   # outside: x0.5 -40 ; inside: x0.2
      f.write(json.dumps({'t': i, 'raw': raw}) + '\n')


class T(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp(prefix='ft_')

  def logs(self, specs):
    out = []
    for n, (o, i) in enumerate(specs):
      p = os.path.join(self.d, 'l%d.jsonl' % n); make_log(p, o, i, seed=n)
      per, _ = F.load(p); out.append((F.scan(per), {'outside': o, 'inside': i}))
    return out

  def test_finds_the_hidden_temperature_with_three_logs(self):
    logs = self.logs([(5.0, 19.0), (11.0, 23.0), (14.5, 27.5)])
    rows, n, distinct = F.search(logs, 'outside')
    self.assertEqual((n, distinct), (3, 3))
    top = rows[0]
    self.assertEqual(top['addr'], 0x2e5); self.assertEqual(top['key'][2], 24)                   # byte 3 = bit 24
    self.assertEqual((top['scale'], top['offset']), (0.5, -40.0))
    rows, _, _ = F.search(logs, 'inside')
    self.assertEqual(rows[0]['addr'], 0x2e5); self.assertEqual(rows[0]['key'][2], 40); self.assertEqual((rows[0]['scale'], rows[0]['offset']), (0.2, 0.0))

  def test_the_clock_and_the_counter_are_not_temperatures(self):
    logs = self.logs([(5.0, 19.0), (11.0, 23.0), (14.5, 27.5)])
    for target in ('outside', 'inside'):
      rows, _, _ = F.search(logs, target)
      self.assertNotIn(0x318, [r['addr'] for r in rows]); self.assertNotIn(0x1f0, [r['addr'] for r in rows])

  def test_random_chatter_does_not_survive_three_logs(self):
    logs = self.logs([(5.0, 19.0), (11.0, 23.0), (14.5, 27.5)])
    rows, _, _ = F.search(logs, 'outside')
    self.assertEqual(set(r['addr'] for r in rows), {0x2e5})                                    # nothing else fits all three

  def test_a_single_log_is_flagged_weak(self):
    logs = self.logs([(5.0, 19.0)])
    rows, n, distinct = F.search(logs, 'outside')
    self.assertEqual((n, distinct), (1, 1)); self.assertIn(0x2e5, [r['addr'] for r in rows])  # the real one is there, among others

  def test_a_field_that_never_moves_is_rejected_when_the_temperature_did(self):
    p1, p2 = os.path.join(self.d, 'a.jsonl'), os.path.join(self.d, 'b.jsonl')
    for p in (p1, p2):
      with open(p, 'w') as f:
        for i in range(30):
          f.write(json.dumps({'raw': {'0x100': bytes([0, 0, 0, 90, 0, 0, 0, 0]).hex()}}) + '\n')   # same raw in both logs
    logs = [(F.scan(F.load(p1)[0]), {'outside': 5.0}), (F.scan(F.load(p2)[0]), {'outside': 15.0})]
    self.assertEqual(F.search(logs, 'outside')[0], [])

  def test_spec_parsing(self):
    self.assertEqual(F.parse_spec('/data/x.jsonl:outside=5,inside=19.5'), ('/data/x.jsonl', {'outside': 5.0, 'inside': 19.5}))
    with self.assertRaises(ValueError): F.parse_spec('x.jsonl')
    with self.assertRaises(ValueError): F.parse_spec('x.jsonl:foo=1')

  def test_big_endian_word(self):
    p = os.path.join(self.d, 'be.jsonl')
    with open(p, 'w') as f:
      for i in range(30):
        f.write(json.dumps({'raw': {'0x200': bytes([0, 0, 0, 0, 0x01, 0xF4, 0, 0]).hex()}}) + '\n')     # 0x01F4 = 500 big-endian at bytes 4-5 -> x0.1 = 50.0
    sc = F.scan(F.load(p)[0])
    self.assertIn((0x200, ('M', 16, 4, False)), sc); self.assertEqual(sc[(0x200, ('M', 16, 4, False))][0], 500)


class OneLogMoving(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp(prefix='ft2_')

  def test_inside_that_moves_inside_one_log_is_found_with_start_greater_than_end(self):
    p = os.path.join(self.d, 'move.jsonl'); make_log(p, 9.0, 23.5, seed=5, rows=100, inside_to=27.0)
    per, _ = F.load(p)
    # the moving field is not steady over the whole log, but steady in the first and last fifth
    entries = [(F.scan(per, 'first'), {'inside': 23.5}), (F.scan(per, 'last'), {'inside': 27.0})]
    rows, n, distinct = F.search(entries, 'inside', tol=0.75)
    self.assertEqual((n, distinct), (2, 2))
    top = rows[0]
    self.assertEqual(top['addr'], 0x2e5); self.assertEqual(top['key'][2], 40); self.assertEqual((top['scale'], top['offset']), (0.2, 0.0))

  def test_cli_accepts_start_greater_than_end(self):
    p = os.path.join(self.d, 'cli.jsonl'); make_log(p, 9.0, 23.5, seed=6, rows=100, inside_to=27.0)
    self.assertEqual(F.parse_spec(p + ':outside=9,inside=23.5>27')[1], {'outside': 9.0, 'inside': (23.5, 27.0)})
    self.assertEqual(F.parse_spec(p + ':outside=9,inside=23.5..27')[1], {'outside': 9.0, 'inside': (23.5, 27.0)})   # the form to type: > is a shell redirect
    self.assertEqual(F.parse_spec('x.jsonl:inside=23..26')[1], {'inside': (23.0, 26.0)})
    import io, contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
      rc = F.main([p + ':inside=23.5..27'])
    out = buf.getvalue()
    self.assertEqual(rc, 0); self.assertIn('0x2e5', out); self.assertIn('2 reading point(s), 2 distinct', out)


class Live(unittest.TestCase):
  class C:
    def __init__(self, src, address, dat): self.src, self.address, self.dat = src, address, dat

  class Ev:
    def __init__(self, can): self.can = can

  def test_collect_live_keeps_one_bus_and_arrival_order(self):
    C, Ev = self.C, self.Ev
    clock = [0.0]
    batches = [[Ev([C(0, 0x2e5, bytes([0, 0, 0, 90, 0, 0, 0, 0])), C(1, 0x2e5, bytes(8)), C(0, 0x100, bytes([1] * 8))])],
               [Ev([C(0, 0x2e5, bytes([0, 0, 0, 92, 0, 0, 0, 0]))])]]
    def drain():
      return batches.pop(0) if batches else []
    def sleep(_): clock[0] += 1.0
    per, frames = F.collect_live(drain, 5, bus=0, now=lambda: clock[0], sleep=sleep)
    self.assertEqual(frames, 3)                                           # the bus-1 frame is ignored
    self.assertEqual([(v >> 24) & 0xFF for v in per[0x2e5]], [90, 92])    # arrival order kept (first/last fifth depend on it)
    self.assertNotIn(0x2e5 | 0x10000, per)

  def test_live_search_finds_the_hidden_field_with_start_and_end(self):
    C, Ev = self.C, self.Ev
    rnd = random.Random(3)
    per = {}
    for i in range(100):                                                  # a fake live capture: noise, a clock, and 0x2e5 byte 5 = inside*5 rising 21 -> 27
      for a in range(0x100, 0x120):
        per.setdefault(a, []).append(int.from_bytes(bytes(rnd.randrange(256) for _ in range(8)), 'little'))
      per.setdefault(0x318, []).append(int.from_bytes(bytes([0x9a, 0x0b, i % 60, 0x12, 4, 1, 0x40, 1]), 'little'))
      per.setdefault(0x2e5, []).append(int.from_bytes(bytes([1, 2, 3, 4, 5, int(round((21 + 6 * i / 99.0) * 5)), 7, 8]), 'little'))
    import io, contextlib
    answers = iter(['27'])
    old_input = __builtins__['input'] if isinstance(__builtins__, dict) else __builtins__.input
    try:
      if isinstance(__builtins__, dict): __builtins__['input'] = lambda prompt='': next(answers)
      else: __builtins__.input = lambda prompt='': next(answers)
      buf = io.StringIO()
      with contextlib.redirect_stdout(buf):
        rc = F.main(['--live', '10', 'inside=21', '--ask-end'], capture=lambda secs, bus: (per, 3200))
    finally:
      if isinstance(__builtins__, dict): __builtins__['input'] = old_input
      else: __builtins__.input = old_input
    out = buf.getvalue()
    self.assertEqual(rc, 0); self.assertIn('0x2e5', out); self.assertIn('2 reading point(s), 2 distinct', out); self.assertIn("inside': (21.0, 27.0)", out)

  def test_read_number_ignores_stray_keys(self):
    self.assertEqual(F.read_number('?', reader=lambda p: '\x1b[B22'), 22.0)          # a down-arrow typed during the capture
    self.assertEqual(F.read_number('?', reader=lambda p: ' 23,5 '), 23.5)
    self.assertIsNone(F.read_number('?', reader=lambda p: ''))
    self.assertIsNone(F.read_number('?', reader=lambda p: '\x1b[A'))

  def test_parse_readings(self):
    self.assertEqual(F.parse_readings('outside=9,inside=23.5'), {'outside': 9.0, 'inside': 23.5})
    self.assertEqual(F.parse_readings('inside=23.5..26'), {'inside': (23.5, 26.0)})
    with self.assertRaises(ValueError): F.parse_readings('speed=3')


if __name__ == '__main__':
  unittest.main()
