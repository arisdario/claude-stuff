#!/usr/bin/env python3
"""THNEED port (tesla/model083/thneed_port): the rules that keep modeld safe.
 1. the input matcher pairs SNPE's buffers with the runner's by size and refuses anything ambiguous (C++ test).
 2. the build file can never break or change a normal build: aarch64 only, try/except, trial compile; _modeld only;
    _dmonitoringmodeld keeps the plain objects.
 3. every thneed line in the runner is behind USE_THNEED, and the plain SNPE path is still there at the end of execute().
 4. thneed.cc has no assert left (0.8.3's crash points became a fallback).
 5. the kernel-driver header is the real file 0.8.3 pointed to, not a Git LFS pointer.
 6. the installer refuses unknown files and a running modeld, and keeps a backup / restore."""
import hashlib
import os
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'model083', 'thneed_port')
read = lambda p: open(os.path.join(ROOT, p)).read()


class T(unittest.TestCase):
  @unittest.skipUnless(shutil.which('g++'), 'needs g++')
  def test_input_matcher(self):
    d = tempfile.mkdtemp()
    try:
      exe = os.path.join(d, 'imt')
      subprocess.check_call(['g++', '-std=c++11', '-I' + os.path.join(ROOT, 'thneed'),
                             os.path.join(ROOT, 'test', 'input_map_test.cc'), '-o', exe])
      r = subprocess.run([exe], stdout=subprocess.PIPE, universal_newlines=True)
      self.assertEqual(r.returncode, 0, r.stdout)
    finally:
      shutil.rmtree(d)

  def test_build_file(self):
    s = read('SConscript')
    i = s.index('# THNEED')
    block = s[i:s.index("lenv.Program('_dmonitoringmodeld'")]
    self.assertIn('if arch == "aarch64":', block)
    self.assertIn('try:', block); self.assertIn('except Exception', block)
    self.assertIn('TryCompile', block)
    self.assertGreater(block.index('modeld_objs = [o for o'), block.index('if trial_ok:'))
    self.assertIn("modeld_objs = common\n", block)                       # default: exactly the old objects
    dm = s[s.index("lenv.Program('_dmonitoringmodeld'"):s.index("lenv.Program('_modeld'")]
    self.assertIn(']+common, LIBS=libs)', dm)                            # driver monitoring: plain SNPE objects
    self.assertIn(']+modeld_objs, LIBS=libs)', s[s.index("lenv.Program('_modeld'"):])

  def test_class_layout_does_not_depend_on_the_macro(self):
    # 16 Nov 2026: members under #ifdef USE_THNEED -> driving.cc (built without it) allocated 168 bytes, the constructor
    # (built with it) wrote 248 -> SIGBUS at modeld start. No preprocessor line may sit inside the class.
    h = read('runners/snpemodel.h')
    body = h[h.index('class SNPEModel'):h.index('};', h.index('class SNPEModel'))]
    self.assertFalse([l for l in body.split('\n') if l.strip().startswith('#')])
    self.assertIn('class Thneed;', h); self.assertNotIn('thneed/thneed.h', h)

  def test_runner_thneed_only_behind_the_macro(self):
    s = read('runners/snpemodel.cc')        # the header declares the thneed fields always (same layout), the CODE is behind the macro
    depth, inside = 0, []
    for line in s.split('\n'):
      if line.startswith('#ifdef USE_THNEED'):
        depth += 1
      elif line.startswith('#endif') and depth:
        depth -= 1
      elif re.search(r'\bthneed\b|Thneed|thneed_', line) and not line.strip().startswith('//'):
        inside.append((depth, line))
    self.assertTrue(inside)
    self.assertTrue(all(dep > 0 for dep, _ in inside), [l for dep, l in inside if dep == 0])
    cc = read('runners/snpemodel.cc')
    ex = cc[cc.index('void SNPEModel::execute('):]
    self.assertTrue(ex.rstrip().endswith('''  assert(inputBuffer->setBufferAddress(net_input_buf));
  if (!snpe->execute(inputMap, outputMap)) {
    PrintErrorStringAndExit();
  }
}'''))                                                                     # the original plain path, unchanged

  def test_no_asserts_in_thneed(self):
    self.assertNotIn('assert(', read('thneed/thneed.cc'))

  def test_kgsl_header_is_real(self):
    data = open(os.path.join(ROOT, 'thneed', 'include', 'msm_kgsl.h'), 'rb').read()
    self.assertEqual(hashlib.sha256(data).hexdigest(), '089b21b377325c0b0f04b96b6ed4a8e5975b1c050191598cd64dc0a3a3565a71')

  def test_installer(self):
    s = read('install_thneed.sh')
    subprocess.check_call(['sh', '-n', os.path.join(ROOT, 'install_thneed.sh')])
    self.assertEqual(len(re.findall(r'\b[0-9a-f]{32}\b', s.split('WANT=')[1].split('\n')[0])), 3)
    self.assertIn('pgrep -f _modeld', s); self.assertIn('--restore', s); self.assertIn('THNEED_OFF=1', s)

  def test_installer_keeps_only_a_working_build(self):
    """--yes with fake scons / pgrep / bench: the new _modeld stays only when the bench says it works"""
    car = os.environ.get('THNEED_CAR_TREE') or os.path.join(ROOT, 'test', 'car075')   # the car's original files (fixture)
    if not car:
      self.skipTest('set THNEED_CAR_TREE to the car files the port was made from')
    BENCH = {
      'works': 'thneed selftest passed (9 GPU commands replayed)\nX: 30.0 ms per run, CPU 8.0 ms per run (user+sys, all threads), output checksum 00000000deadbeef',
      'crash': 'Segmentation fault',
      'unusable': 'thneed: recording unusable: x - plain SNPE\nX: 40.0 ms per run, CPU 30.0 ms per run (user+sys, all threads), output checksum 00000000deadbeef',
      'differs': 'thneed selftest passed (9 GPU commands replayed)\nX: 30.0 ms per run, CPU 8.0 ms per run (user+sys, all threads), output checksum 1111111111111111',
    }
    BENCH['modeld_died'] = BENCH['works']      # the bench passes, but the REAL modeld dies at start (16 Nov 2026)
    for scenario, kept in (('works', True), ('crash', False), ('unusable', False), ('differs', False), ('modeld_died', False)):
      d = tempfile.mkdtemp()
      try:
        M = os.path.join(d, 'op', 'selfdrive', 'modeld')
        shutil.copytree(car, M)
        os.makedirs(os.path.join(M, 'models'), exist_ok=True)
        open(os.path.join(M, 'models', 'model0812_adapter.h'), 'w').close()
        open(os.path.join(M, '_modeld'), 'w').write('OLD')
        open(os.path.join(M, 'modeld'), 'w').write('#!/bin/sh\necho "loaded model with size: 1"\n' +
                                                   ('exit 1\n' if scenario == 'modeld_died' else 'sleep 30\n'))
        os.chmod(os.path.join(M, 'modeld'), 0o755)
        bindir = os.path.join(d, 'bin'); os.makedirs(bindir)
        plain = 'X: 40.0 ms per run, CPU 30.0 ms per run (user+sys, all threads), output checksum 00000000deadbeef'
        bench = '#!/bin/sh\nif [ -n "$THNEED_OFF" ]; then echo "%s"; else printf "%s\\n"; fi\n' % (plain, BENCH[scenario])
        open(os.path.join(bindir, 'scons'), 'w').write(
          '#!/bin/sh\ncase "$*" in *thneed_bench*) mkdir -p %s/thneed; printf \'%s\' > %s/thneed/thneed_bench; chmod +x %s/thneed/thneed_bench;;\n'
          '*) echo "modeld: thneed is built in (dlsym offset -724; off switch: touch /data/thneed_off)"; echo NEW > %s/_modeld;; esac\n'
          % (M, bench.replace("'", "'\\''"), M, M, M))
        open(os.path.join(bindir, 'pgrep'), 'w').write('#!/bin/sh\nexit 1\n')
        for f in ('scons', 'pgrep'): os.chmod(os.path.join(bindir, f), 0o755)
        env = dict(os.environ, OP=os.path.join(d, 'op'), BK=os.path.join(d, 'bk'), PATH=bindir + ':' + os.environ['PATH'], THNEED_GATE_S='1')
        r = subprocess.run(['sh', os.path.join(ROOT, 'install_thneed.sh'), '--yes'], env=env, cwd=d,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        modeld = open(os.path.join(M, '_modeld')).read().strip()
        self.assertEqual(modeld, 'NEW' if kept else 'OLD', scenario + '\n' + r.stdout)
        self.assertIn('KEPT' if kept else 'RESTORED', r.stdout, scenario)
        if not kept:   # the sources are back too
          self.assertEqual(open(os.path.join(M, 'runners', 'snpemodel.cc')).read(), open(os.path.join(car, 'runners', 'snpemodel.cc')).read())
      finally:
        shutil.rmtree(d)


if __name__ == '__main__':
  unittest.main()
