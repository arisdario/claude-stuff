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

- Built in only after a trial compile on the EON; otherwise `_modeld` is built exactly as before.
- First frame: SNPE run, recorded run, replay - the replay must equal the recorded output **bit for bit**, or plain SNPE.
- `touch /data/thneed_off` switches it off without uninstalling (plain SNPE, as before), `rm` to switch it on again.
- `sh install_thneed.sh --restore` puts the previous sources and `_modeld` binary back.

## Install (parked)

```
tmux kill-session -t comma
sh /data/claude-stuff/tesla/model083/thneed_port/install_thneed.sh --yes
reboot
```
The installer refuses unless the car's `SConscript`, `runners/snpemodel.cc` and `.h` are the exact files the port was
made from, and openpilot is stopped. It builds `_modeld` and `thneed/thneed_bench`, then runs the bench 50 times with
plain SNPE and 50 times with thneed (same inputs) and prints: time and CPU per run of each, the self-test result, and
whether the final outputs are identical. `RESULT: thneed works ...` = safe to reboot into it.

After the reboot, modeld's log shows `thneed selftest passed` (or the reason it fell back):
`tmux capture-pane -p -t comma -S -3000 | grep -i thneed | tail -5`
