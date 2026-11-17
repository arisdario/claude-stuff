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
//
// OWNERSHIP (17 Nov 2026 hardening):
//  - created here, freed here: the command memory (GPUMalloc: one KGSL GPU object + its mmap), the host copies of the
//    recorded commands (CachedCommand / CachedSync), the output staging buffer.
//  - SNPE's, NOT ours: the command queue, the input / output cl_mem, the kernels, the KGSL file descriptor and the
//    KGSL draw context. Thneed takes ONE OpenCL reference (clRetain*) on the queue and on every buffer it keeps, and
//    gives back exactly those references in release(); it never releases anything it did not retain. Kernels are not
//    retained: they are only used while recording, inside SNPE's own run.
//  - SNPE's input buffers stay MAPPED (clEnqueueMapBuffer) for thneed's whole life: the replayed GPU commands read
//    them through their GPU address, and thneed writes each frame's inputs into them through the mapping (as 0.8.3;
//    Adreno buffers are shared host memory). release() waits for thneed's last GPU command, unmaps them and finishes
//    the queue, so SNPE gets its buffers back unmapped when it runs again.
//
// THREADS: modeld calls the model from its main loop only (modeld.cc), so SNPE's OpenCL calls come from one thread.
// What the OpenCL driver does inside is not known, so everything a recording shares between calls (the recording
// itself, the kernel-argument capture) is behind a mutex - only while recording. The replay touches none of it: its
// ioctls pass the interceptor with one atomic load.

#ifndef __user
  #define __user __attribute__(())
#endif

#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include "include/msm_kgsl.h"
#include <vector>
#include <memory>
#include <string>
#ifndef CL_TARGET_OPENCL_VERSION
#define CL_TARGET_OPENCL_VERSION 120
#endif
#include <CL/cl.h>

#define THNEED_RECORD 1
#define THNEED_DEBUG 2
#define THNEED_VERBOSE_DEBUG 4

class Thneed;

// switched off with /data/thneed_off or THNEED_OFF (read once per process). Off = SNPE runs exactly as before thneed:
// no GPU priority change, no kernel-argument capture, no recording.
bool thneed_enabled();
// the end of the one recording a process makes (any outcome): the kernel-argument capture stops for good
void thneed_capture_end();

// One KGSL GPU object, mapped into this process; replayed commands are copied into it. Owns the object ID and the
// mapping: release() frees what exists (munmap, then IOCTL_KGSL_GPUOBJ_FREE), also after a half-failed constructor,
// and does nothing the second time.
class GPUMalloc {
  public:
    GPUMalloc(int size, int fd);
    ~GPUMalloc() { release(); }
    GPUMalloc(const GPUMalloc &) = delete;
    GPUMalloc &operator=(const GPUMalloc &) = delete;
    void *alloc(size_t size);  // NULL when the area is used up (0.8.3: assert)
    bool ok() const { return map_addr != NULL; }
    bool release();            // true if everything that existed was given back
    void abandon();            // forget it WITHOUT freeing: the GPU may still read it (see Thneed::release)
  private:
    int fd = -1;
    bool allocated = false;    // IOCTL_KGSL_GPUOBJ_ALLOC succeeded: `id` is ours to free
    unsigned int id = 0;
    void *map_addr = NULL;     // mmap succeeded (never MAP_FAILED)
    size_t map_size = 0;
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
    bool ok = true;            // every OpenCL query about the kernel worked
    std::string why;
    cl_program program = NULL;
    std::string name;
    cl_uint num_args = 0;
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

// One of SNPE's input buffers, mapped for writing: the buffer (one reference held by thneed), the mapping, its size.
struct ThneedMappedBuffer {
  cl_mem mem = NULL;
  void *ptr = NULL;
  size_t size = 0;
};

class Thneed {
  public:
    Thneed();                  // starts recording (when it could get its command memory; else `failed`)
    ~Thneed();                 // release()
    Thneed(const Thneed &) = delete;
    Thneed &operator=(const Thneed &) = delete;

    void stop_recording();     // no more commands are recorded (the kernel-argument capture ends too)
    void map_io();             // finds and maps SNPE's input buffers and the output (`failed` on any problem)
    void stop() { stop_recording(); map_io(); }
    // finputs / fbytes in the order of input_sizes(); foutput gets the output only if everything worked (it is not
    // touched otherwise). false = this replay did not work.
    bool execute(float **finputs, const size_t *fbytes, float *foutput, size_t foutput_bytes, bool slow=false);
    bool release();            // idempotent; true if all GPU memory could be freed (see the .cc)
    int wait(unsigned int timeout_ms);

    std::vector<size_t> input_sizes() const;
    size_t num_inputs() const { return inputs.size(); }
    bool has_output() const { return output != NULL; }

    std::vector<ThneedMappedBuffer> inputs;
    cl_mem output = NULL;      // retained by thneed when `output_retained`
    size_t output_size = 0;
    bool output_retained = false;
    std::vector<unsigned char> output_staging;

    cl_context context = NULL;
    cl_command_queue command_queue = NULL;   // retained by thneed when `queue_retained`
    bool queue_retained = false;
    cl_device_id device_id = NULL;
    int context_id = 0;

    int record;
    int timestamp;
    bool submitted = false;    // a replayed command reached the GPU: release() waits for `last_submitted` first
    int last_submitted = 0;
    bool released = false;
    std::unique_ptr<GPUMalloc> ram;
    std::vector<std::unique_ptr<CachedIoctl> > cmds;
    int fd;
    bool failed = false;          // something during recording was not as 0.8.3 expects: do not use this recording
    std::string why_failed;

    void copy_inputs(float **finputs, const size_t *fbytes);
    cl_int clexec();
    void abandon_recording(const char *why);   // run what is queued, stop recording, mark failed
    std::vector<std::shared_ptr<CLQueuedKernel> > kq;      // all CL kernels
    std::vector<std::shared_ptr<CLQueuedKernel> > ckq;     // pending CL kernels
    size_t executed = 0;       // kernels of kq already run (0.8.3: a file-level static, shared by every recording)
    pthread_mutex_t rec_lock;  // recursive: the driver may submit a command (ioctl) from inside clexec
    void fail(const char *why);
};

// the Thneed currently recording (NULL when none); atomic
Thneed *thneed_recording();

#ifdef THNEED_TEST
// host tests only: KGSL ioctls go to this function instead of the kernel driver; argument-capture introspection
extern int (*thneed_test_kgsl)(int fd, unsigned long request, void *argp);
size_t thneed_test_captured_args(cl_kernel k);
cl_int thneed_clSetKernelArg(cl_kernel kernel, cl_uint arg_index, size_t arg_size, const void *arg_value);
cl_int thneed_clEnqueueNDRangeKernel(cl_command_queue command_queue, cl_kernel kernel, cl_uint work_dim,
                                     const size_t *global_work_offset, const size_t *global_work_size,
                                     const size_t *local_work_size, cl_uint num_events_in_wait_list,
                                     const cl_event *event_wait_list, cl_event *event);
cl_int thneed_clFinish(cl_command_queue command_queue);
cl_int thneed_clReleaseKernel(cl_kernel kernel);
#endif
