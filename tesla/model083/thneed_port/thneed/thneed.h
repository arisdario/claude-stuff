#pragma once
// thneed - port of openpilot 0.8.3's selfdrive/modeld/thneed (comma.ai, MIT) to Tinkla 0.7.5.a's modeld.
// "thneed is an SNPE accelerator ... It runs on the local device, and caches a single model run. Then it replays it,
// but fast." It records the GPU commands SNPE sends to the kernel driver (KGSL ioctls) during one model run, then
// replays only those commands every frame, skipping SNPE's per-frame CPU work.
//
// Changes from 0.8.3 (THNEED.md has the list): no loading/saving of .thneed files (runtime recording only);
// every assert that 0.8.3 hit on SNPE calls made outside a recording became a `failed` flag the runner checks (it
// then keeps plain SNPE instead of crashing modeld); execute() returns whether the replay worked; inputs are copied
// with the caller's sizes (matched by size in snpemodel.cc, not by an assumed order).

#ifndef __user
  #define __user __attribute__(())
#endif

#include <stdlib.h>
#include <stdint.h>
#include "include/msm_kgsl.h"
#include <vector>
#include <memory>
#include <string>
#include <CL/cl.h>

#define THNEED_RECORD 1
#define THNEED_DEBUG 2
#define THNEED_VERBOSE_DEBUG 4

class Thneed;

class GPUMalloc {
  public:
    GPUMalloc(int size, int fd);
    void *alloc(int size);     // NULL when the area is used up (0.8.3: assert)
    bool ok() const { return base != 0; }
  private:
    uint64_t base = 0;
    int remaining = 0;
};

class CLQueuedKernel {
  public:
    CLQueuedKernel(Thneed *lthneed,
                   cl_kernel _kernel,
                   cl_uint _work_dim,
                   const size_t *_global_work_size,
                   const size_t *_local_work_size);
    cl_int exec();
    void debug_print(bool verbose);
    cl_program program;
    std::string name;
    cl_uint num_args;
    std::vector<std::string> arg_names;
    std::vector<std::string> arg_types;
    std::vector<std::string> args;
    std::vector<int> args_size;
    cl_kernel kernel = NULL;

    cl_uint work_dim;
    size_t global_work_size[3] = {0};
    size_t local_work_size[3] = {0};
    bool has_local = true;
  private:
    Thneed *thneed;
};

class CachedIoctl {
  public:
    virtual ~CachedIoctl() {}
    virtual int exec() { return 0; }
};

class CachedSync: public CachedIoctl {
  public:
    CachedSync(Thneed *lthneed, std::string ldata) { thneed = lthneed; data = ldata; }
    int exec();
  private:
    Thneed *thneed;
    std::string data;
};

class CachedCommand: public CachedIoctl {
  public:
    CachedCommand(Thneed *lthneed, struct kgsl_gpu_command *cmd);
    int exec();
  private:
    struct kgsl_gpu_command cache;
    std::unique_ptr<kgsl_command_object[]> cmds;
    std::unique_ptr<kgsl_command_object[]> objs;
    Thneed *thneed;
    std::vector<std::shared_ptr<CLQueuedKernel> > kq;
};

class Thneed {
  public:
    Thneed();
    void stop();
    // finputs / fbytes in the order of `inputs` below; returns false if any replayed command failed
    bool execute(float **finputs, const size_t *fbytes, float *foutput, bool slow=false);
    void wait();

    std::vector<void *> inputs;
    std::vector<size_t> input_sizes;
    cl_mem output = NULL;

    cl_context context = NULL;
    cl_command_queue command_queue = NULL;
    cl_device_id device_id = NULL;
    int context_id = 0;

    int record;
    int timestamp;
    std::unique_ptr<GPUMalloc> ram;
    std::vector<std::unique_ptr<CachedIoctl> > cmds;
    int fd;
    bool failed = false;          // something during recording was not as 0.8.3 expects: do not use this recording
    std::string why_failed;

    void find_inputs_outputs();
    void copy_inputs(float **finputs, const size_t *fbytes);
    void copy_output(float *foutput);
    cl_int clexec();
    std::vector<std::shared_ptr<CLQueuedKernel> > kq;      // all CL kernels
    std::vector<std::shared_ptr<CLQueuedKernel> > ckq;     // pending CL kernels
    void fail(const char *why);
};

extern Thneed *g_thneed;
extern int g_fd;
