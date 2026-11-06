"""Logic of tools/build_lightdash.sh with a fake compiler: which libraries it picks, and the -no-pie retry (the EON failure: a static non-PIC libzmq.a in a PIE link)."""
import os, subprocess, tempfile, unittest
TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools')
FAKE = r'''#!/bin/sh
# fake c++ compiler: logs the command; a link (has lightdash.o) with a static zmq and without -no-pie FAILS like the real linker did
echo "$@" >> "$FAKELOG"
out=""; prev=""; for a in "$@"; do [ "$prev" = "-o" ] && out="$a"; prev="$a"; done
case "$*" in *lightdash.o*)
  case "$*" in *libzmq.a*) case "$*" in *-no-pie*) ;; *) echo "relocation R_AARCH64_ADR_PREL_PG_HI21 ... recompile with -fPIC" >&2; exit 1;; esac;; esac
  printf '#!/bin/sh\necho selftest-ok\n' > "$out"; chmod +x "$out"; exit 0;;
esac
[ -n "$out" ] && : > "$out"; exit 0
'''


def links(log):
  """the link command lines: they take lightdash.o as an input (the compile line has -c and produces it)"""
  return [l for l in log if 'lightdash.o' in l and ' -c ' not in l]


def build(with_system_libs, with_static):
  d = tempfile.mkdtemp(prefix='bld_')
  op, sysd = os.path.join(d, 'op'), os.path.join(d, 'sys')
  for p in ('cereal/gen/cpp', 'cereal/messaging', 'phonelibs/zmq/aarch64-linux/lib', 'phonelibs/zmq/aarch64-linux/include', 'phonelibs/capnp-cpp/aarch64/lib', 'phonelibs/capnp-cpp/include'):
    os.makedirs(os.path.join(op, p))
  for f in ('cereal/gen/cpp/log.capnp.h', 'cereal/services.h', 'cereal/libcereal.a') + tuple('cereal/messaging/%s.cc' % n for n in ('messaging', 'impl_zmq', 'impl_msgq', 'msgq')):
    open(os.path.join(op, f), 'w').close()
  os.makedirs(sysd)
  if with_system_libs:
    for n in ('capnp', 'kj', 'zmq'):
      open(os.path.join(sysd, 'lib%s.so' % n), 'w').close()
  if with_static:
    for n, sub in (('zmq', 'zmq/aarch64-linux/lib'), ('capnp', 'capnp-cpp/aarch64/lib'), ('kj', 'capnp-cpp/aarch64/lib')):
      open(os.path.join(op, 'phonelibs', sub, 'lib%s.a' % n), 'w').close()
  cxx = os.path.join(d, 'fakecxx'); open(cxx, 'w').write(FAKE); os.chmod(cxx, 0o755)
  log = os.path.join(d, 'log'); out = os.path.join(d, 'lightdash')
  r = subprocess.run(['sh', os.path.join(TOOLS, 'build_lightdash.sh')], env=dict(os.environ, OP=op, SYS=sysd, OUT=out, CXX=cxx, FAKELOG=log), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
  return r, open(log).read().splitlines()


class T(unittest.TestCase):
  def test_prefers_the_shared_system_libraries(self):
    r, log = build(True, True)
    self.assertEqual(r.returncode, 0, r.stdout)
    link = links(log)
    self.assertEqual(len(link), 1)                                          # no retry needed
    self.assertIn('-lcapnp', link[0]); self.assertIn('-lkj', link[0]); self.assertIn('-lzmq', link[0]); self.assertNotIn('.a', ' '.join(a for a in link[0].split() if 'libzmq' in a))
    self.assertIn('-Wl,-rpath', link[0]); self.assertIn('selftest-ok', r.stdout)

  def test_static_libs_retry_with_no_pie(self):
    r, log = build(False, True)
    self.assertEqual(r.returncode, 0, r.stdout)
    link = links(log)
    self.assertEqual(len(link), 2); self.assertNotIn('-no-pie', link[0]); self.assertIn('-no-pie', link[1])
    self.assertIn('retrying with -no-pie', r.stdout)

  def test_each_library_is_chosen_on_its_own(self):
    r, log = build(False, False)                                            # nothing found: system default names for all three
    link = links(log)[0]
    self.assertEqual(r.returncode, 0, r.stdout)
    for n in ('-lcapnp', '-lkj', '-lzmq'):
      self.assertIn(n, link)

  def test_unknown_warning_option_is_silenced(self):
    r, log = build(True, False)
    self.assertTrue(any('-Wno-unknown-warning-option' in l for l in log))


if __name__ == '__main__':
  unittest.main()
