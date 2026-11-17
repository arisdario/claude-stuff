# thneed for Tinkla 0.7.5.a's modeld (port of openpilot 0.8.3's thneed)

**Status: built and tested on a PC only (compile with gcc and clang against the car's own headers, the real build file
under SCons, unit tests). Not yet run on the EON.** The parked bench below is the first real test.

## What it does

openpilot 0.7.6 introduced it ("Huge CPU savings in modeld by using thneed!"); this is the 0.8.3 version, the
generation the 0.8.12 model belongs to. During one model run it records the GPU commands SNPE sends to the kernel's GPU
driver (KGSL ioctls), then every frame it replays only those commands - SNPE's per-frame CPU work is skipped. Same
model, same GPU kernels: the result must be identical, and the self-test checks exactly that.

Like 0.8.3 it also gives the model's GPU work top priority and runs the GPU at full clock while the model runs.

## Changes from 0.8.3 (comma.ai, MIT licence)

| 0.8.3 | here |
|---|---|
| asserts in code that runs on every SNPE call, also outside a recording | a `failed` flag: the runner keeps plain SNPE, modeld never stops |
| self-test failure: `assert(false)` (modeld stops) | plain SNPE from then on, the frame's output is the recorded run's |
| inputs copied in a fixed order {state, traffic side, desire, image} | matched **by size** (`input_map.h`), anything ambiguous is refused |
| all recorded kernels run again at every `clFinish` | only the ones queued since the previous `clFinish` (same with one call per run) |
| `.thneed` files compiled at build time, loaded at start | not ported: recorded at every modeld start (one extra model run) |
| kernel optimizer, disassembler, serialization | not ported |
| linked into modeld and driver monitoring | `_modeld` only |
| 512 kB of GPU memory for the recorded commands | 2 MB, and running out ends the recording instead of crashing |

## Safety

- Built in only after a trial compile on the EON; otherwise the installer puts the previous `_modeld` back, unchanged.
- First frame: SNPE run, recorded run, then SNPE's own result for a **second, changed input**; thneed then replays both
  inputs and each must equal SNPE's result **bit for bit** (no tolerance), or plain SNPE for good. The second input
  catches a replay that writes new frames into the wrong buffer (the model would see the recorded picture forever) -
  the recorded input alone cannot.
- `touch /data/thneed_off` switches it off without uninstalling, `rm` to switch it on again. Off really is plain SNPE:
  no GPU priority change, no kernel-argument capture, no recording (17 Nov 2026: before, the priority change stayed).
- `sh install_thneed.sh --restore` puts the previous sources and `_modeld` binary back. A failed build restores by itself.

## Fail-safe and ownership (17 Nov 2026 hardening)

- Any problem while recording, mapping, testing or replaying = thneed off for good, that frame and every later one from
  plain SNPE (a failed replay never touches the output; plain SNPE computes that frame). Nothing in the thneed path
  ends modeld; the one exit left is SNPE's own, on a failing normal run, exactly as before thneed.
- Checked OpenCL calls: kernel name / argument count / argument names (for SNPE's input and output kernels), context
  and device, buffer sizes, retain, map, kernel launches and clFinish while recording; the output read and the GPU
  wait (2 s bound) while replaying.
- Ownership: thneed frees what it created (its GPU command memory: munmap + IOCTL_KGSL_GPUOBJ_FREE, also after a
  half-failed allocation; its copies of the commands; an output staging buffer) and returns exactly the OpenCL
  references it took (one on SNPE's queue, one on each input / output buffer it keeps). SNPE's input buffers stay
  mapped while thneed replays and are unmapped (after waiting for the GPU) when thneed goes - at a fallback or when the
  model is deleted. If the GPU never confirms it finished, the command memory is deliberately kept, not freed.
- The model input size is checked (`buf_size` counts floats, as every caller passes it); a wrong one = thneed off.
- One recording per process. Kernel arguments are captured from the process start (SNPE may set one long before the
  recorded run) until that recording ends, then the capture stops; a destroyed kernel's arguments are dropped
  (clReleaseKernel), so a new kernel at a reused address starts clean. The kernel counter is per recording.
- Threads: modeld calls the model from its main loop only; the recording's shared state is behind a mutex that is never
  held across a driver call (no deadlock with a driver thread), and the replay takes no lock.
- Not changed: `RunModel` (runmodel.h) has no virtual destructor, so modeld's `delete s->m` (driving.cc, at exit) does
  not reach `~SNPEModel`. Making it virtual would rebuild driver monitoring too; at that point modeld is exiting anyway.

Host tests (`tesla/tests/test_thneed_port.py`, `test/host/`): the real `thneed.cc` and `snpemodel.cc` against a fake
SNPE / OpenCL / KGSL that executes recorded command buffers, under AddressSanitizer and UBSan: 23 scenarios (every
failure above, plus normal running), each checking every frame's output bit for bit against an independent reference
and that everything was given back. They do NOT prove the Qualcomm driver behaves like the fake: the EON bench does.

## Install (parked)

```
tmux kill-session -t comma
sh /data/claude-stuff/tesla/model083/thneed_port/install_thneed.sh --yes
reboot
```
The installer refuses unless the car's `SConscript`, `runners/snpemodel.cc` and `.h` are the exact files the port was
made from (or an earlier version of this port), and openpilot is stopped. It builds `_modeld` and `thneed/thneed_bench`, then runs the bench 50 times with
plain SNPE and 50 times with thneed (same inputs) and prints: time and CPU per run of each, the self-test result, and
whether the final outputs are identical. `RESULT: thneed works ...` = safe to reboot into it.

After the reboot, modeld's log shows `thneed selftest passed` (or the reason it fell back):
`tmux capture-pane -p -t comma -S -3000 | grep -i thneed | tail -5`
