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

  def test_runner_thneed_only_behind_the_macro(self):
    s = read('runners/snpemodel.cc') + read('runners/snpemodel.h')
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


if __name__ == '__main__':
  unittest.main()
