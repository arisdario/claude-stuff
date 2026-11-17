#pragma once
// HOST TESTS ONLY. A fake of what thneed sits between on the EON:
//  - SNPE (fake_snpe.hpp): a small network whose GPU work goes through thneed's OpenCL interceptors
//    (thneed_clSetKernelArg / thneed_clEnqueueNDRangeKernel / thneed_clFinish / thneed_clReleaseKernel), as SNPE's
//    calls do through dlsym on the car;
//  - OpenCL (the cl* functions thneed calls): buffers, kernels, a queue, reference counts, maps;
//  - the KGSL device (thneed_test_kgsl): GPU memory IDs, draw contexts, waits - and GPU commands that EXECUTE the
//    command buffer they point to, so a replay of thneed's copy really recomputes the network from the buffers.
// Every failure thneed must survive can be switched on (FakeKnobs).
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>
#include "thneed/thneed.h"
#include "fake_snpe.hpp"

enum { N_IMG = 768, N_DES = 8, N_TC = 2, N_REC = 64, N_OUT = 264 };   // floats; the memory state is the last N_REC outputs

// the network (one definition: the fake GPU and the tests' reference both call it)
void fake_net(const float *img, const float *rec, const float *des, const float *tc, float *out);

struct FakeKnobs {
  int refuse_priority = 0;     // the kernel refuses KGSL_CONTEXT_PRIORITY: thneed must retry with SNPE's flags
  int fail_alloc = 0;          // thneed's command memory cannot be allocated
  int mmap_fail = 0;           // the KGSL fd is write-only: thneed's mmap fails after the allocation worked
  int fail_kernel_name = 0;    // clGetKernelInfo(CL_KERNEL_FUNCTION_NAME) fails
  int fail_arg_info = 0;       // clGetKernelArgInfo fails (a program built without argument info)
  int fail_context = 0;        // clGetContextInfo fails
  int fail_mem_info = 0;       // clGetMemObjectInfo fails
  int fail_map = 0;            // clEnqueueMapBuffer fails
  int fail_exec_at = 0;        // the n-th launch of the network kernel fails
  int fail_snpe_run = 0;       // the n-th SNPE run returns false
  int perturb_replay = 0;      // GPU commands from thneed's copy compute output[0] one ulp off
  int decoy_input = 0;         // the image kernel's "input" argument is a buffer the network never reads
  int layout_mismatch = 0;     // SNPE pads desire and traffic convention to the same size: the matcher must refuse
  int args_once = 0;           // SNPE sets kernel arguments once at build, never again
  int stale_kernel = 0;        // a released kernel's address is reused by SNPE's image kernel
  int fail_replay_cmd_at = 0;  // the n-th GPU command from thneed's copy fails
  int fail_wait_from = 0;      // GPU waits fail from the n-th one on
  int fail_read_at = 0;        // the n-th output read fails
};

struct FakeStats {
  int snpe_runs = 0, snpe_cmds = 0, replay_cmds = 0, waits = 0, reads = 0, maps_open = 0, unmaps = 0;
  std::vector<unsigned int> ctx_flags;            // flags of every draw context created
  std::vector<unsigned int> alloc_ids, free_ids;
  std::vector<uint64_t> alloc_sizes;
  int stale_args_after_release = -1;
};

extern FakeKnobs K;
extern FakeStats S;

void fake_setup(const char *dir);   // the GPU memory file and the KGSL hook
int fake_fd();                      // the KGSL "device" (read-write)
int fake_refs_extra();              // OpenCL references held above SNPE's own (0 = all given back)
int fake_thneed_allocs();           // command-memory allocations (thneed's: 0x200000 bytes)
int fake_thneed_frees();            // ... of them freed
cl_kernel fake_new_kernel(const char *name, std::vector<std::string> args);
cl_mem fake_new_mem(size_t bytes);
cl_command_queue fake_new_queue();
cl_kernel fake_last_network_kernel();
