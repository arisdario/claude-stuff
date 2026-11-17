// HOST TESTS ONLY: the real runners/snpemodel.cc and thneed/thneed.cc against fake_gpu.cc.
//   thneed_host_test SCENARIO WORKDIR       exit 0 + "RESULT ok", or "FAIL ..." lines and exit 1
// Every scenario drives the model like driving.cc (memory state at the end of the output, inputs that change every
// frame) and checks EVERY frame's output bit for bit against an independent reference (fake_net() in a plain loop):
// whatever thneed does or fails at, the model's results must be exactly SNPE's. Then it checks the mode, and that
// everything thneed took (GPU memory, maps, OpenCL references) was given back.
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "fake_gpu.h"
#include "runners/snpemodel.h"
#include "thneed/input_map.h"
#include "thneed/selftest.h"

extern "C" int ioctl(int, unsigned long, void *);   // thneed's interceptor

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL line %d: %s -- ", __LINE__, #c); printf(__VA_ARGS__); printf("\n"); } } while (0)

static std::string g_model;

struct Car {                                     // one model the way driving.cc owns it
  float output[N_OUT] = {0};
  float ref[N_OUT] = {0};                        // the reference: the same network, no SNPE, no thneed
  float desire[N_DES] = {0}, traffic[N_TC] = {1.f, 0.f};
  std::vector<float> img = std::vector<float>(N_IMG);
  SNPEModel *m;
  int frame = 0, bad_frames = 0;
  Car() {
    m = new SNPEModel(g_model.c_str(), output, N_OUT, USE_GPU_RUNTIME);
    m->addRecurrent(&output[N_OUT - N_REC], N_REC);
    m->addDesire(desire, N_DES);
    m->addTrafficConvention(traffic, N_TC);
  }
  void step(int buf_size = N_IMG) {
    for (int i = 0; i < N_IMG; i++) img[i] = (float)((frame * 31 + i * 7) % 256);
    for (int i = 0; i < N_DES; i++) desire[i] = (frame % 3 == 1 && i == frame % N_DES) ? 1.f : 0.f;
    m->execute(img.data(), buf_size);
    float next[N_OUT];
    fake_net(img.data(), &ref[N_OUT - N_REC], desire, traffic, next);
    memcpy(ref, next, sizeof(ref));
    if (memcmp(output, ref, sizeof(ref)) != 0) {
      const ThneedCompare c = thneed_compare_outputs(output, ref, N_OUT);
      bad_frames++;
      printf("FAIL frame %d: output differs from the reference (%zu values, first at %zu)\n", frame, c.n_diff, c.first);
      fails++;
    }
    frame++;
  }
};

static void expect_all_given_back(const char *when) {
  CHECK(S.maps_open == 0, "%s: %d input maps still open", when, S.maps_open);
  CHECK(fake_refs_extra() == 0, "%s: %d OpenCL references not given back", when, fake_refs_extra());
  CHECK(thneed_recording() == NULL, "%s: a recording is still registered", when);
}

static bool priority_set(unsigned int flags) { return (flags & KGSL_CONTEXT_PRIORITY_MASK) == (1u << KGSL_CONTEXT_PRIORITY_SHIFT); }

// a drive that must end on plain SNPE (thneed refused at the first frame), exact outputs, nothing left behind
static void expect_fallback_at_start(int thneed_freed) {
  Car c;
  for (int i = 0; i < 6; i++) { c.step(); CHECK(c.m->thneed_mode() == -1, "frame %d mode %d", i, c.m->thneed_mode()); }
  expect_all_given_back("after the fallback");
  CHECK(fake_thneed_frees() == thneed_freed, "thneed command memory freed %d times, want %d", fake_thneed_frees(), thneed_freed);
  delete c.m;
  CHECK(fake_thneed_frees() == thneed_freed, "after delete: freed %d times, want %d (no double free)", fake_thneed_frees(), thneed_freed);
}

static void unit_tests() {
  // compare: exact bits, with diagnostics
  float a[6] = {1.f, -0.f, NAN, INFINITY, 2.f, 3.f}, b[6] = {1.f, 0.f, NAN, INFINITY, 2.5f, 3.f};
  ThneedCompare c = thneed_compare_outputs(a, a, 6);
  CHECK(c.n_diff == 0, "identical arrays: %zu differ", c.n_diff);
  c = thneed_compare_outputs(a, b, 6);
  CHECK(c.n_diff == 2 && c.first == 1, "n_diff %zu first %zu (-0 vs +0 is a difference, the same NaN bits are not)", c.n_diff, c.first);
  CHECK(fabs(c.max_abs - 0.5) < 1e-12 && fabs(c.max_rel - 0.2) < 1e-12, "max_abs %g max_rel %g", c.max_abs, c.max_rel);
  float d[2] = {NAN, 1.f}, e[2] = {1.f, INFINITY};
  c = thneed_compare_outputs(d, e, 2);
  CHECK(c.n_diff == 2 && c.n_nonfinite == 2 && c.n_nan == 1 && c.max_abs == 0., "nonfinite %zu nan %zu max_abs %g", c.n_nonfinite, c.n_nan, c.max_abs);
  float f1 = 1.f, f2 = nextafterf(1.f, 2.f);
  c = thneed_compare_outputs(&f1, &f2, 1);
  CHECK(c.n_diff == 1, "one ulp must count as a difference");

  // the camera slot
  const int p_ok[4] = {2, 0, 3, 1}, p_bad[4] = {1, 2, 3, 1};
  CHECK(thneed_image_slot(p_ok, 4) == 1, "slot of input 0");
  CHECK(thneed_image_slot(p_bad, 4) == -1, "no slot: -1");

  // GPUMalloc: whole, half-built (allocated, mmap refused), not allocated; release() is idempotent
  {
    S = FakeStats();
    GPUMalloc g(0x200000, fake_fd());
    CHECK(g.ok(), "allocation");
    CHECK(g.alloc(100) != NULL && g.alloc(0x200000) == NULL, "sub-allocation and its limit");
    CHECK(g.alloc((size_t)-1) == NULL, "a huge size is refused, not wrapped around");
    CHECK(g.release() && fake_thneed_frees() == 1, "first release frees once");
    CHECK(g.release() && fake_thneed_frees() == 1, "second release does nothing");
  }
  CHECK(fake_thneed_frees() == 1, "the destructor after release() frees nothing more");
  {
    S = FakeStats();
    int wo = open("/proc/self/exe", 0);           // a read-only fd: mmap(PROT_WRITE, MAP_SHARED) is refused
    GPUMalloc g(0x200000, wo);
    CHECK(!g.ok(), "the mapping must fail");
    CHECK(fake_thneed_frees() == 1, "half-built: the GPU object is freed at once (%d)", fake_thneed_frees());
    g.release();
    CHECK(fake_thneed_frees() == 1, "and only once");
    close(wo);
  }
  {
    S = FakeStats();
    K.fail_alloc = 1;
    GPUMalloc g(0x200000, fake_fd());
    K.fail_alloc = 0;
    CHECK(!g.ok() && S.free_ids.empty(), "nothing allocated, nothing freed");
  }

  // Thneed: one recording at a time; release() gives back its memory once and unregisters it
  {
    S = FakeStats();
    struct kgsl_gpuobj_alloc al;
    memset(&al, 0, sizeof(al));
    al.size = 4096;
    ioctl(fake_fd(), IOCTL_KGSL_GPUOBJ_ALLOC, &al);   // through thneed's interceptor: it learns the fd
    Thneed *t = new Thneed();
    CHECK(!t->failed && thneed_recording() == t, "recording started");
    Thneed *t2 = new Thneed();
    CHECK(t2->failed && thneed_recording() == t, "a second recording is refused (%s)", t2->why_failed.c_str());
    delete t2;
    CHECK(thneed_recording() == t, "deleting the refused one leaves the first registered");
    CHECK(t->release() && thneed_recording() == NULL, "release unregisters");
    const int freed = fake_thneed_frees();
    CHECK(freed == 2, "both command memories freed once (%d)", freed);
    CHECK(t->release() && fake_thneed_frees() == freed, "second release: nothing more");
    delete t;
    CHECK(fake_thneed_frees() == freed, "delete after release: nothing more");
  }

  // stale kernel arguments: a destroyed kernel's arguments are forgotten, a still-referenced one keeps them
  {
    cl_mem mem = fake_new_mem(64);
    cl_kernel k = fake_new_kernel("zero_pad_image_float", {"input", "output"});
    thneed_clSetKernelArg(k, 0, sizeof(cl_mem), &mem);
    CHECK(thneed_test_captured_args(k) == 1, "captured");
    clRetainKernel(k);
    thneed_clReleaseKernel(k);
    CHECK(thneed_test_captured_args(k) == 1, "still referenced: kept");
    thneed_clReleaseKernel(k);
    CHECK(thneed_test_captured_args(k) == 0, "destroyed: forgotten");
    cl_kernel k2 = fake_new_kernel("zero_pad_image_float", {"input", "output"});
    CHECK(k2 == k && thneed_test_captured_args(k2) == 0, "a new kernel at the same address starts with nothing");
  }

  // the kernels a recording queues are counted per recording (0.8.3: one counter for the whole process, so a second
  // recording ran none of its kernels): two recordings in a row, each must run the kernel it queued
  {
    cl_mem bufs[5];
    for (int i = 0; i < 5; i++) bufs[i] = fake_new_mem(N_OUT * 4 > N_IMG * 4 ? N_OUT * 4 : N_IMG * 4);
    cl_kernel net = fake_new_kernel("fake_network", {"img", "rec", "des", "tc", "out"});
    for (int i = 0; i < 5; i++) clSetKernelArg(net, i, sizeof(cl_mem), &bufs[i]);
    cl_command_queue q = fake_new_queue();
    const size_t gws[1] = {1}, lws[1] = {1};
    for (int r = 0; r < 2; r++) {
      Thneed *t = new Thneed();
      CHECK(!t->failed, "recording %d started (%s)", r, t->why_failed.c_str());
      const int before = S.snpe_cmds;
      CHECK(thneed_clEnqueueNDRangeKernel(q, net, 1, NULL, gws, lws, 0, NULL, NULL) == CL_SUCCESS, "queued");
      CHECK(S.snpe_cmds == before, "recording %d: queued, not run yet", r);
      CHECK(thneed_clFinish(q) == CL_SUCCESS, "finish");
      CHECK(S.snpe_cmds == before + 1, "recording %d: the kernel it queued ran at clFinish (%d)", r, S.snpe_cmds - before);
      t->stop_recording();
      delete t;
    }
  }
}

int main(int argc, char **argv) {
  if (argc < 3) return 2;
  const std::string scn = argv[1];
  fake_setup(argv[2]);
  g_model = std::string(argv[2]) + "/model.dlc";
  FILE *mf = fopen(g_model.c_str(), "wb");
  fputs("fake model", mf);
  fclose(mf);

  if (scn == "unit") {
    unit_tests();
  } else if (scn == "normal" || scn == "refuse_priority" || scn == "args_once" || scn == "stale_kernel") {
    K.refuse_priority = scn == "refuse_priority";
    K.args_once = scn == "args_once";
    K.stale_kernel = scn == "stale_kernel";
    Car c;
    for (int i = 0; i < 12; i++) { c.step(); CHECK(c.m->thneed_mode() == 1, "frame %d mode %d", i, c.m->thneed_mode()); }
    CHECK(S.snpe_runs == 3, "SNPE ran %d times (the normal run, the recorded run, the second self-test input)", S.snpe_runs);
    CHECK(S.replay_cmds == 2 + 11, "replayed commands %d (2 self-test + 11 frames)", S.replay_cmds);
    CHECK(!S.ctx_flags.empty() && priority_set(S.ctx_flags[0]), "GPU priority asked for");
    if (K.refuse_priority) CHECK(S.ctx_flags.size() == 2 && S.ctx_flags[1] == 0x4200ad2, "refused: retried with SNPE's own flags");
    if (K.stale_kernel) CHECK(S.stale_args_after_release == 0, "the destroyed kernel's arguments are forgotten (%d)", S.stale_args_after_release);
    CHECK(S.maps_open == 4, "4 inputs mapped while replaying (%d)", S.maps_open);
    delete c.m;
    expect_all_given_back("after delete");
    CHECK(fake_thneed_frees() == 1, "command memory freed %d times", fake_thneed_frees());
  } else if (scn == "off") {                     // run with THNEED_OFF=1
    Car c;
    for (int i = 0; i < 6; i++) { c.step(); CHECK(c.m->thneed_mode() == -1, "frame %d mode %d", i, c.m->thneed_mode()); }
    CHECK(S.snpe_runs == 6 && S.replay_cmds == 0 && fake_thneed_allocs() == 0, "runs %d replays %d allocs %d", S.snpe_runs, S.replay_cmds, fake_thneed_allocs());
    CHECK(S.ctx_flags.size() == 1 && S.ctx_flags[0] == 0x4200ad2, "SNPE's GPU context untouched (0x%x)", S.ctx_flags.empty() ? 0 : S.ctx_flags[0]);
    CHECK(thneed_test_captured_args(fake_last_network_kernel()) == 0, "no kernel arguments captured");
    delete c.m;
    expect_all_given_back("after delete");
  } else if (scn == "two_models") {              // one recording per process: the second model stays on SNPE, exactly
    Car a;
    for (int i = 0; i < 3; i++) a.step();
    CHECK(a.m->thneed_mode() == 1, "first model replays");
    Car b;
    for (int i = 0; i < 3; i++) { b.step(); a.step(); }
    CHECK(b.m->thneed_mode() == -1 && a.m->thneed_mode() == 1, "modes a %d b %d", a.m->thneed_mode(), b.m->thneed_mode());
    delete b.m;
    delete a.m;
    expect_all_given_back("after delete");
    CHECK(fake_thneed_frees() == 2, "both command memories freed (%d)", fake_thneed_frees());
  } else if (scn == "fail_kernel_name") { K.fail_kernel_name = 1; expect_fallback_at_start(1);
  } else if (scn == "fail_arg_info") { K.fail_arg_info = 1; expect_fallback_at_start(1);
  } else if (scn == "fail_context") { K.fail_context = 1; expect_fallback_at_start(1);
  } else if (scn == "fail_mem_info") { K.fail_mem_info = 1; expect_fallback_at_start(1);
  } else if (scn == "fail_map") { K.fail_map = 1; expect_fallback_at_start(1);
  } else if (scn == "fail_exec") { K.fail_exec_at = 2; expect_fallback_at_start(1);       // the recorded run's network launch
  } else if (scn == "fail_snpe_run") { K.fail_snpe_run = 2; expect_fallback_at_start(1);  // SNPE fails while being recorded
  } else if (scn == "mismatch") { K.perturb_replay = 1; expect_fallback_at_start(1);
  } else if (scn == "decoy") { K.decoy_input = 1; expect_fallback_at_start(1);
  } else if (scn == "layout_mismatch") { K.layout_mismatch = 1; expect_fallback_at_start(1);
  } else if (scn == "mmap_fail") { K.mmap_fail = 1; expect_fallback_at_start(1);          // half-built GPUMalloc
  } else if (scn == "alloc_fail") { K.fail_alloc = 1; expect_fallback_at_start(0);
  } else if (scn == "replay_fail" || scn == "wait_timeout" || scn == "read_fail") {
    if (scn == "replay_fail") K.fail_replay_cmd_at = 2 + 3;   // replayed commands: 2 in the self-test, then one per frame
    if (scn == "wait_timeout") K.fail_wait_from = 2 + 3;
    if (scn == "read_fail") K.fail_read_at = 2 + 3;
    Car c;
    for (int i = 0; i < 3; i++) { c.step(); CHECK(c.m->thneed_mode() == 1, "frame %d replays", i); }
    c.step();                                    // the replay fails: this frame comes from plain SNPE
    CHECK(c.m->thneed_mode() == -1, "permanent fallback");
    const int runs = S.snpe_runs;
    for (int i = 0; i < 4; i++) c.step();
    CHECK(S.snpe_runs == runs + 4, "plain SNPE every frame from then on (%d)", S.snpe_runs - runs);
    CHECK(c.m->thneed_mode() == -1, "and it stays off");
    expect_all_given_back("right after the fallback");
    if (scn == "wait_timeout") CHECK(fake_thneed_frees() == 0, "the GPU never confirmed: command memory deliberately kept");
    else CHECK(fake_thneed_frees() == 1, "command memory freed at the fallback (%d)", fake_thneed_frees());
    delete c.m;
  } else if (scn == "bad_input") {
    Car c;
    c.step();
    c.step();
    CHECK(c.m->thneed_mode() == 1, "replaying");
    c.step(N_IMG - 1);                           // a wrong input size: thneed must not read it with its own size
    CHECK(c.m->thneed_mode() == -1, "off after a wrong input size");
    c.step();
    expect_all_given_back("after the fallback");
    delete c.m;
  } else {
    printf("unknown scenario %s\n", scn.c_str());
    return 2;
  }
  if (fails == 0) printf("RESULT ok\n");
  return fails == 0 ? 0 : 1;
}
