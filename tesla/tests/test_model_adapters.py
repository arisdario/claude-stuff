#!/usr/bin/env python3
"""Builds and runs the C++ unit tests of both model adapters (model083/test_adapter.cc, model083/test_adapter0812.cc).
test_adapter0812.cc writes its fake output through comma's own v0.8.12 structs, so it also checks every offset."""
import os
import shutil
import subprocess
import tempfile
import unittest

M = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'model083')


@unittest.skipUnless(shutil.which('g++'), 'no g++')
class T(unittest.TestCase):
  def run_cc(self, src, std):
    exe = os.path.join(tempfile.mkdtemp(), 't')
    subprocess.check_call(['g++', '-std=' + std, '-O2', '-w', '-o', exe, os.path.join(M, src)], cwd=M)
    r = subprocess.run([exe], cwd=M, stdout=subprocess.PIPE, universal_newlines=True)
    self.assertEqual(r.returncode, 0, r.stdout)
    self.assertIn('all passed', r.stdout)

  def test_adapter_083(self):
    self.run_cc('test_adapter.cc', 'c++11')

  def test_adapter_0812(self):
    self.run_cc('test_adapter0812.cc', 'c++17')

  def test_driving_cc_picks_adapter_and_checks_model_size(self):
    s = open(os.path.join(M, 'driving.cc')).read()
    self.assertIn('#if __has_include("model0812_adapter.h")', s)
    self.assertIn('MODEL_ADAPTER::MODEL_DLC_BYTES', s)
    self.assertNotIn('m083::', s)                                      # everything goes through the selected adapter

  def test_install_scripts_switch_both(self):
    i083 = open(os.path.join(M, 'install.sh')).read(); i0812 = open(os.path.join(M, 'install0812.sh')).read()
    self.assertIn('rm -f "$MD/model0812_adapter.h"', i083)             # back to 0.8.3 removes the 0.8.12 adapter
    self.assertIn('model0812_adapter.h', i0812)
    self.assertIn('209e9544e456dbc2a7d60490da65154e129bc84830909d8d931f97b3df93949b', i0812)   # the 0.8.12 dlc

  def test_model_file_switched_only_after_a_successful_build(self):
    for name, tag in (('install.sh', '083'), ('install0812.sh', '0812')):
      s = open(os.path.join(M, name)).read()
      build = s.index('scons -j4 selfdrive/modeld/_modeld')
      swap = s.index('cp "$OP/models/supercombo.dlc.%s" "$OP/models/supercombo.dlc"' % tag)
      self.assertLess(build, swap, name)                                 # an old modeld never runs a new model file
      import re
      writes = re.findall(r'^\s*cp [^\n]*"\$OP/models/supercombo\.dlc"\s*$', s, re.M)
      self.assertEqual(len(writes), 1, (name, writes))                   # exactly one place writes the live model file
      self.assertIn('putting the previous sources back', s)

  def test_0812_lead_speed_is_made_relative(self):
    s = open(os.path.join(M, 'model0812_adapter.h')).read()
    self.assertIn('R.rel_vel = m[2] - v_ego;', s)                       # 0.8.12 lead v is absolute; 0.7.5 wants relative
    self.assertIn('const float *m = d, *sd = d + LEAD_MHP_VALS;', s)    # leadFuture = point 0, like comma's leadTwo


if __name__ == '__main__':
  unittest.main()
