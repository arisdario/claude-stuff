// thneed - port of openpilot 0.8.3's selfdrive/modeld/thneed/thneed.cc (comma.ai, MIT) to Tinkla 0.7.5.a.
// See thneed.h and THNEED.md for what changed. Rule of the port: anything unexpected while RECORDING marks the
// recording failed (the runner then keeps plain SNPE); nothing here may stop modeld.
#include <sys/mman.h>
#include <dlfcn.h>
#include <limits.h>
#include <map>
#include <new>
#include <string>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include "thneed.h"            // (no <unistd.h>: on Android it can declare ioctl(), which this file defines)

using std::string;
using std::map;
using std::pair;
using std::make_pair;
using std::unique_ptr;

#define THNEED_WAIT_MS 2000           // one replay takes ~23 ms; a GPU that has not finished after 2 s = replay failed
#define THNEED_RELEASE_WAIT_MS 2000
#define THNEED_MAX_IOCTL_ERRORS 20    // GPU ioctl failures printed per process (then silence: no log flood at 20 Hz)

// *********** process-wide state ***********
// g_thneed: the Thneed that is recording (NULL otherwise). g_fd: the KGSL file descriptor SNPE's driver uses (seen on
// its first GPUOBJ_ALLOC; SNPE's, never closed here). Both are read by the interceptors from any thread: atomics.
static Thneed *g_thneed = NULL;
static int g_fd = -1;
static int g_enabled = -1;                   // thneed_enabled(): -1 not read yet, 0 off, 1 on
static int g_capture = 1;                    // kernel-argument capture: until the process's one recording has ended
static int g_ioctl_errors = 0;
// the arguments SNPE sets on its kernels (thneed must know them for the kernels it records). Captured from the start
// of the process, because SNPE may set an argument once, long before the recorded run. An entry is dropped when its
// kernel is destroyed (clReleaseKernel to 0), so a new kernel at a reused address never inherits old arguments.
static pthread_mutex_t g_args_lock = PTHREAD_MUTEX_INITIALIZER;
static map<pair<cl_kernel, int>, string> g_args;
static map<pair<cl_kernel, int>, int> g_args_size;

Thneed *thneed_recording() { return __atomic_load_n(&g_thneed, __ATOMIC_ACQUIRE); }
static bool claim_recording(Thneed *t) {
  Thneed *none = NULL;
  return __atomic_compare_exchange_n(&g_thneed, &none, t, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
static void end_recording(Thneed *t) {
  Thneed *expected = t;
  __atomic_compare_exchange_n(&g_thneed, &expected, (Thneed *)NULL, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
static bool is_recording(Thneed *t) { return __atomic_load_n(&t->record, __ATOMIC_ACQUIRE) & THNEED_RECORD; }

bool thneed_enabled() {
  int e = __atomic_load_n(&g_enabled, __ATOMIC_ACQUIRE);
  if (e < 0) {
    FILE *f = fopen("/data/thneed_off", "r");
    e = (f != NULL || getenv("THNEED_OFF") != NULL) ? 0 : 1;
    if (f != NULL) fclose(f);
    __atomic_store_n(&g_enabled, e, __ATOMIC_RELEASE);
  }
  return e == 1;
}

static bool capture_on() { return __atomic_load_n(&g_capture, __ATOMIC_ACQUIRE) && thneed_enabled(); }

void thneed_capture_end() {
  __atomic_store_n(&g_capture, 0, __ATOMIC_RELEASE);
  pthread_mutex_lock(&g_args_lock);          // nothing reads them after the recording: give the memory back
  g_args.clear();
  g_args_size.clear();
  pthread_mutex_unlock(&g_args_lock);
}

#ifdef THNEED_TEST
size_t thneed_test_captured_args(cl_kernel k) {
  pthread_mutex_lock(&g_args_lock);
  size_t n = 0;
  for (auto it = g_args.lower_bound(make_pair(k, 0)); it != g_args.end() && it->first.first == k; ++it) n++;
  pthread_mutex_unlock(&g_args_lock);
  return n;
}
#endif

// *********** ioctl interceptor (as 0.8.3) ***********

#ifdef THNEED_TEST
int (*thneed_test_kgsl)(int fd, unsigned long request, void *argp) = NULL;
#endif

extern "C" {

static int (*my_ioctl)(int filedes, unsigned long request, void *argp) = NULL;

static int real_ioctl(int filedes, unsigned long request, void *argp) {
#ifdef THNEED_TEST
  if (thneed_test_kgsl != NULL && ((request >> 8) & 0xff) == KGSL_IOC_TYPE) return thneed_test_kgsl(filedes, request, argp);
#endif
  int (*f)(int, unsigned long, void *) = __atomic_load_n(&my_ioctl, __ATOMIC_ACQUIRE);
  if (f == NULL) {
    f = reinterpret_cast<int (*)(int, unsigned long, void *)>(dlsym(RTLD_NEXT, "ioctl"));
    __atomic_store_n(&my_ioctl, f, __ATOMIC_RELEASE);
  }
  return f(filedes, request, argp);
}

#undef ioctl
int ioctl(int filedes, unsigned long request, void *argp) {
  request &= 0xFFFFFFFF;  // needed on QCOM2

  // save the fd (SNPE's driver allocates GPU memory before any model run)
  if (request == IOCTL_KGSL_GPUOBJ_ALLOC) __atomic_store_n(&g_fd, filedes, __ATOMIC_RELEASE);

  // 0.8.3: the model's GPU work gets top priority. PORT: only while thneed is enabled (thneed_off = SNPE exactly as
  // before), and if this kernel refuses the priority flag the context is created again with SNPE's own flags.
  if (request == IOCTL_KGSL_DRAWCTXT_CREATE && thneed_enabled()) {
    struct kgsl_drawctxt_create *create = (struct kgsl_drawctxt_create *)argp;
    const unsigned int orig_flags = create->flags;
    create->flags &= ~KGSL_CONTEXT_PRIORITY_MASK;
    create->flags |= 1 << KGSL_CONTEXT_PRIORITY_SHIFT;   // priority from 1-15, 1 is max priority
    int r = real_ioctl(filedes, request, argp);
    printf("IOCTL_KGSL_DRAWCTXT_CREATE: context with flags 0x%x: %s\n", create->flags, r == 0 ? "ok" : "refused, retrying with SNPE's flags");
    if (r == 0) return 0;
    create->flags = orig_flags;
    return real_ioctl(filedes, request, argp);
  }

  Thneed *thneed = thneed_recording();       // the replay never gets past this line: nothing is recording then
  if (thneed != NULL && is_recording(thneed)) {
    pthread_mutex_lock(&thneed->rec_lock);   // never held across a driver call (see clexec): cannot deadlock
    try {
      if (request == IOCTL_KGSL_GPU_COMMAND) {
        struct kgsl_gpu_command *cmd = (struct kgsl_gpu_command *)argp;
        thneed->timestamp = cmd->timestamp;
        thneed->context_id = cmd->context_id;
        thneed->cmds.push_back(unique_ptr<CachedCommand>(new CachedCommand(thneed, cmd)));
      } else if (request == IOCTL_KGSL_GPUOBJ_SYNC) {
        struct kgsl_gpuobj_sync *cmd = (struct kgsl_gpuobj_sync *)argp;
        if (cmd->count > 0 && cmd->objs == 0) {
          thneed->fail("GPU sync without objects");
        } else {
          struct kgsl_gpuobj_sync_obj *objs = (struct kgsl_gpuobj_sync_obj *)(cmd->objs);
          thneed->cmds.push_back(unique_ptr<CachedSync>(new CachedSync(thneed,
                                 string((char *)objs, sizeof(struct kgsl_gpuobj_sync_obj)*cmd->count))));
        }
      }
    } catch (...) {
      thneed->fail("out of memory while recording");
    }
    pthread_mutex_unlock(&thneed->rec_lock);
  }

  int ret = real_ioctl(filedes, request, argp);
  // PORT: only the GPU driver's failures are reported (0.8.3 printed every failed ioctl in the process, e.g. errno 25
  // from devices that are not the GPU), and at most THNEED_MAX_IOCTL_ERRORS of them
  if (ret != 0 && ((request >> 8) & 0xff) == KGSL_IOC_TYPE) {
    const int n = __atomic_add_fetch(&g_ioctl_errors, 1, __ATOMIC_RELAXED);
    if (n <= THNEED_MAX_IOCTL_ERRORS) {
      fprintf(stderr, "thneed: GPU ioctl 0x%lx returned %d, errno %d%s\n", request, ret, errno,
              n == THNEED_MAX_IOCTL_ERRORS ? " (further GPU ioctl errors are not printed)" : "");
    }
  }
  return ret;
}

}

// *********** GPUMalloc ***********

GPUMalloc::GPUMalloc(int size, int lfd) {
  fd = lfd;
  if (fd < 0 || size <= 0) return;
  struct kgsl_gpuobj_alloc alloc;
  memset(&alloc, 0, sizeof(alloc));
  alloc.size = size;
  alloc.flags = 0x10000a00;
  if (ioctl(fd, IOCTL_KGSL_GPUOBJ_ALLOC, &alloc) != 0) return;
  allocated = true;                            // from here on, `id` is ours to free (release() does)
  id = alloc.id;
  void *addr = mmap64(NULL, alloc.mmapsize, 0x3, 0x1, fd, (off64_t)alloc.id * 0x1000);
  if (addr == MAP_FAILED) {
    fprintf(stderr, "thneed: could not map the command memory (errno %d)\n", errno);
    release();                                 // half-built: the GPU object is freed at once
    return;
  }
  map_addr = addr;
  map_size = alloc.mmapsize;
  base = (uint64_t)addr;
  remaining = size;
}

void *GPUMalloc::alloc(size_t size) {
  if (map_addr == NULL || size > (size_t)remaining) return NULL;
  size = (size + 0xff) & ~(size_t)0xFF;
  if (size > (size_t)remaining) return NULL;
  void *ret = (void*)base;
  remaining -= (int)size;
  base += size;
  return ret;
}

// munmap, then free the ID. Valid in either order for KGSL (the mapping holds its own reference to the object, which
// is destroyed when the last one goes); every step only for what exists, each only once.
bool GPUMalloc::release() {
  bool ok = true;
  if (map_addr != NULL) {
    if (munmap(map_addr, map_size) != 0) ok = false;
    map_addr = NULL;
    map_size = 0;
  }
  if (allocated) {
    struct kgsl_gpuobj_free f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    if (ioctl(fd, IOCTL_KGSL_GPUOBJ_FREE, &f) != 0) ok = false;
    allocated = false;
  }
  base = 0;
  remaining = 0;
  return ok;
}

void GPUMalloc::abandon() {
  map_addr = NULL;
  map_size = 0;
  allocated = false;
  base = 0;
  remaining = 0;
}

// *********** CachedSync, at the ioctl layer ***********

int CachedSync::exec() {
  struct kgsl_gpuobj_sync cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.objs = (uint64_t)data.data();
  cmd.obj_len = data.length();
  cmd.count = data.length() / sizeof(struct kgsl_gpuobj_sync_obj);
  return ioctl(thneed->fd, IOCTL_KGSL_GPUOBJ_SYNC, &cmd);
}

// *********** CachedCommand, at the ioctl layer ***********

// (runs inside the ioctl interceptor with rec_lock held: memory work only, no driver call)
CachedCommand::CachedCommand(Thneed *lthneed, struct kgsl_gpu_command *cmd) {
  thneed = lthneed;
  memcpy(&cache, cmd, sizeof(cache));
  cache.cmdlist = 0;                           // set below to OUR copies, never left pointing at SNPE's lists
  cache.objlist = 0;
  if (cmd->numsyncs != 0) { thneed->fail("GPU command with syncs"); return; }

  if (cmd->numcmds > 0) {
    if (cmd->cmdlist == 0 || cmd->cmdsize != sizeof(struct kgsl_command_object)) { thneed->fail("unexpected GPU command list"); return; }
    cmds = std::unique_ptr<kgsl_command_object[]>(new kgsl_command_object[cmd->numcmds]);
    memcpy(cmds.get(), (void *)cmd->cmdlist, sizeof(struct kgsl_command_object)*cmd->numcmds);
    cache.cmdlist = (uint64_t)cmds.get();
    for (unsigned int i = 0; i < cmd->numcmds; i++) {
      if (cmds[i].gpuaddr == 0 && cmds[i].size > 0) { thneed->fail("GPU command without memory"); return; }
      void *nn = thneed->ram->alloc(cmds[i].size);
      if (nn == NULL) { thneed->fail("command memory used up"); return; }
      memcpy(nn, (void*)cmds[i].gpuaddr, cmds[i].size);
      cmds[i].gpuaddr = (uint64_t)nn;
    }
  }

  if (cmd->numobjs > 0) {
    if (cmd->objlist == 0 || cmd->objsize != sizeof(struct kgsl_command_object)) { thneed->fail("unexpected GPU object list"); return; }
    objs = std::unique_ptr<kgsl_command_object[]>(new kgsl_command_object[cmd->numobjs]);
    memcpy(objs.get(), (void *)cmd->objlist, sizeof(struct kgsl_command_object)*cmd->numobjs);
    cache.objlist = (uint64_t)objs.get();
    for (unsigned int i = 0; i < cmd->numobjs; i++) {
      void *nn = thneed->ram->alloc(objs[i].size);
      if (nn == NULL) { thneed->fail("command memory used up"); return; }
      memset(nn, 0, objs[i].size);
      objs[i].gpuaddr = (uint64_t)nn;
    }
  }

  kq = thneed->ckq;
  thneed->ckq.clear();
}

int CachedCommand::exec() {
  cache.timestamp = ++thneed->timestamp;       // the draw context uses user-generated timestamps (flag 0x80)
  int ret = ioctl(thneed->fd, IOCTL_KGSL_GPU_COMMAND, &cache);
  if (thneed->record & THNEED_DEBUG) printf("CachedCommand::exec got %d\n", ret);
  if (ret == 0) {
    thneed->submitted = true;
    thneed->last_submitted = thneed->timestamp;
  }
  return ret;
}

// *********** Thneed ***********

Thneed::Thneed() {
  pthread_mutexattr_t a;
  pthread_mutexattr_init(&a);
  pthread_mutex_init(&rec_lock, &a);
  pthread_mutexattr_destroy(&a);
  fd = __atomic_load_n(&g_fd, __ATOMIC_ACQUIRE);
  record = 0;
  timestamp = -1;
  if (fd == -1) { fail("no GPU device seen yet"); return; }
  ram = std::unique_ptr<GPUMalloc>(new (std::nothrow) GPUMalloc(0x200000, fd));   // 0.8.3: 0x80000 - more room, it is cheap
  if (!ram || !ram->ok()) { fail("could not allocate command memory"); return; }
  if (!claim_recording(this)) { fail("another recording is running"); return; }
  __atomic_store_n(&record, THNEED_RECORD, __ATOMIC_RELEASE);
}

Thneed::~Thneed() {
  release();
  pthread_mutex_destroy(&rec_lock);
}

void Thneed::fail(const char *why) {
  if (!__atomic_exchange_n(&failed, true, __ATOMIC_ACQ_REL)) {
    why_failed = why;
    fprintf(stderr, "thneed: recording unusable: %s\n", why);
  }
}

void Thneed::stop_recording() {
  pthread_mutex_lock(&rec_lock);
  __atomic_and_fetch(&record, ~THNEED_RECORD, __ATOMIC_ACQ_REL);
  pthread_mutex_unlock(&rec_lock);
  end_recording(this);
  thneed_capture_end();
}

void Thneed::abandon_recording(const char *why) {
  fail(why);
  clexec();                                    // what is queued runs now, so SNPE's run still computes the right thing
  __atomic_and_fetch(&record, ~THNEED_RECORD, __ATOMIC_ACQ_REL);
  end_recording(this);
}

std::vector<size_t> Thneed::input_sizes() const {
  std::vector<size_t> s;
  for (auto &in : inputs) s.push_back(in.size);
  return s;
}

// SNPE's input buffers (the "input" of its zero_pad_image_float kernels) and output (the "output" of
// image2d_to_buffer_float), as 0.8.3. Each one kept gets one reference from thneed; inputs are mapped for writing.
void Thneed::map_io() {
  if (failed || released) return;
  try {
    if (cmds.empty()) { fail("no GPU commands were recorded"); return; }
    if (command_queue == NULL) { fail("no OpenCL queue seen while recording"); return; }
    if (clRetainCommandQueue(command_queue) != CL_SUCCESS) { fail("could not retain SNPE's command queue"); return; }
    queue_retained = true;
    cl_mem out = NULL;
    for (auto &k : kq) {
      const bool is_in = k->name == "zero_pad_image_float", is_out = k->name == "image2d_to_buffer_float";
      if (!is_in && !is_out) continue;
      if (k->arg_names.size() != k->num_args) { fail("cannot read the argument names of SNPE's input/output kernel"); return; }
      for (unsigned int i = 0; i < k->num_args; i++) {
        const bool want_in = is_in && k->arg_names[i] == "input", want_out = is_out && k->arg_names[i] == "output";
        if (!want_in && !want_out) continue;
        if (k->args[i].size() != sizeof(cl_mem)) { fail(want_in ? "input argument has an unexpected size" : "output argument has an unexpected size"); return; }
        cl_mem aa = NULL;
        memcpy(&aa, k->args[i].data(), sizeof(cl_mem));
        if (aa == NULL) { fail("an input/output argument is not set"); return; }
        if (want_out) { out = aa; continue; }
        size_t sz = 0;
        if (clGetMemObjectInfo(aa, CL_MEM_SIZE, sizeof(sz), &sz, NULL) != CL_SUCCESS || sz == 0) { fail("could not read the size of an input"); return; }
        if (clRetainMemObject(aa) != CL_SUCCESS) { fail("could not retain an input"); return; }
        cl_int err = CL_SUCCESS;
        void *p = clEnqueueMapBuffer(command_queue, aa, CL_TRUE, CL_MAP_WRITE, 0, sz, 0, NULL, NULL, &err);
        if (err != CL_SUCCESS || p == NULL) { clReleaseMemObject(aa); fail("could not map an input"); return; }
        ThneedMappedBuffer b;
        b.mem = aa; b.ptr = p; b.size = sz;
        inputs.push_back(b);
      }
    }
    if (out != NULL) {
      size_t sz = 0;
      if (clGetMemObjectInfo(out, CL_MEM_SIZE, sizeof(sz), &sz, NULL) != CL_SUCCESS || sz == 0) { fail("could not read the size of the output"); return; }
      if (clRetainMemObject(out) != CL_SUCCESS) { fail("could not retain the output"); return; }
      output = out;
      output_retained = true;
      output_size = sz;
      output_staging.resize(sz);
    }
  } catch (...) {
    fail("out of memory while mapping");
    return;
  }
  fprintf(stderr, "Thneed::stop: recorded %zu commands, %zu inputs, output %s\n", cmds.size(), inputs.size(), output ? "found" : "NOT found");
}

void Thneed::copy_inputs(float **finputs, const size_t *fbytes) {
  for (size_t idx = 0; idx < inputs.size(); ++idx) {
    size_t n = fbytes[idx] < inputs[idx].size ? fbytes[idx] : inputs[idx].size;
    memcpy(inputs[idx].ptr, finputs[idx], n);
    if (n < inputs[idx].size) memset((char *)inputs[idx].ptr + n, 0, inputs[idx].size - n);
  }
}

int Thneed::wait(unsigned int timeout_ms) {
  struct kgsl_device_waittimestamp_ctxtid wait;
  memset(&wait, 0, sizeof(wait));
  wait.context_id = context_id;
  wait.timestamp = last_submitted;
  wait.timeout = timeout_ms;
  return ioctl(fd, IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID, &wait);
}

bool Thneed::execute(float **finputs, const size_t *fbytes, float *foutput, size_t foutput_bytes, bool slow) {
  if (failed || released || output == NULL || !ram || finputs == NULL || fbytes == NULL || foutput == NULL) return false;
  if (output_size > foutput_bytes || output_staging.size() != output_size) return false;   // never past the caller's buffer
  for (size_t i = 0; i < inputs.size(); i++) {
    if (finputs[i] == NULL || inputs[i].ptr == NULL) return false;
  }
  copy_inputs(finputs, fbytes);

  // ****** set power constraint (as 0.8.3: GPU at full clock while the model runs; a refusal is not an error)
  struct kgsl_device_constraint_pwrlevel pwrlevel;
  pwrlevel.level = KGSL_CONSTRAINT_PWR_MAX;
  struct kgsl_device_constraint constraint;
  constraint.type = KGSL_CONSTRAINT_PWRLEVEL;
  constraint.context_id = context_id;
  constraint.data = (void*)&pwrlevel;
  constraint.size = sizeof(pwrlevel);
  struct kgsl_device_getproperty prop;
  prop.type = KGSL_PROP_PWR_CONSTRAINT;
  prop.value = (void*)&constraint;
  prop.sizebytes = sizeof(constraint);
  ioctl(fd, IOCTL_KGSL_SETPROPERTY, &prop);

  // ****** run commands; the output is read (into thneed's staging buffer) only when the GPU confirmed it finished
  bool ok = !cmds.empty();
  size_t i = 0;
  for (auto &it : cmds) {
    ++i;
    if (it->exec() != 0) { ok = false; break; }
    if ((i == cmds.size()) || slow) {
      if (wait(THNEED_WAIT_MS) != 0) { ok = false; break; }
    }
  }
  if (ok) ok = clEnqueueReadBuffer(command_queue, output, CL_TRUE, 0, output_size, output_staging.data(), 0, NULL, NULL) == CL_SUCCESS;
  if (ok) memcpy(foutput, output_staging.data(), output_size);    // the caller's output changes only on success

  // ****** unset power constraint
  constraint.type = KGSL_CONSTRAINT_NONE;
  constraint.data = NULL;
  constraint.size = 0;
  ioctl(fd, IOCTL_KGSL_SETPROPERTY, &prop);
  return ok;
}

// Gives back everything thneed holds, once: waits for its last replayed GPU command, unmaps SNPE's input buffers,
// finishes the queue, returns the OpenCL references it took, frees the command memory. If the GPU does not confirm it
// finished, the command memory is deliberately NOT freed (the GPU may still read it). true = all given back.
bool Thneed::release() {
  if (released) return true;
  released = true;
  __atomic_and_fetch(&record, ~THNEED_RECORD, __ATOMIC_ACQ_REL);
  end_recording(this);
  const bool held = submitted || !inputs.empty() || output_retained || queue_retained || ram;
  bool gpu_done = true;
  if (submitted) gpu_done = wait(THNEED_RELEASE_WAIT_MS) == 0;

  size_t unmapped = 0, kept = 0;
  std::vector<bool> can_release(inputs.size(), true);
  for (size_t i = 0; i < inputs.size(); i++) {
    if (inputs[i].ptr == NULL) continue;
    if (queue_retained && clEnqueueUnmapMemObject(command_queue, inputs[i].mem, inputs[i].ptr, 0, NULL, NULL) == CL_SUCCESS) {
      unmapped++;
    } else {
      can_release[i] = false;                  // still mapped: thneed keeps its reference (releasing it would be undefined)
      kept++;
    }
    inputs[i].ptr = NULL;
  }
  if (queue_retained) clFinish(command_queue);
  for (size_t i = 0; i < inputs.size(); i++) {
    if (inputs[i].mem != NULL && can_release[i]) clReleaseMemObject(inputs[i].mem);
    inputs[i].mem = NULL;
  }
  inputs.clear();
  if (output != NULL && output_retained) clReleaseMemObject(output);
  output = NULL;
  output_retained = false;
  output_size = 0;
  std::vector<unsigned char>().swap(output_staging);
  if (queue_retained) clReleaseCommandQueue(command_queue);
  queue_retained = false;
  command_queue = NULL;

  kq.clear();
  ckq.clear();
  cmds.clear();
  bool freed = true;
  if (ram) {
    if (gpu_done) {
      freed = ram->release();
    } else {
      ram->abandon();
      freed = false;
    }
    ram.reset();
  }
  if (held) {
    fprintf(stderr, "thneed: released (%zu inputs unmapped%s, command memory %s)\n", unmapped,
            kept ? ", some could not be unmapped and stay referenced" : "",
            !gpu_done ? "KEPT: the GPU did not confirm it finished" : (freed ? "freed" : "could not be freed"));
  }
  return gpu_done && freed && kept == 0;
}

// While recording, kernels are queued and run here, at SNPE's clFinish. rec_lock only guards the lists: it is never
// held while the driver runs (a driver thread submitting a command would wait for it otherwise).
cl_int Thneed::clexec() {
  std::vector<std::shared_ptr<CLQueuedKernel> > run;
  pthread_mutex_lock(&rec_lock);
  try {
    for (size_t j = executed; j < kq.size(); j++) {
      run.push_back(kq[j]);
      if (is_recording(this)) ckq.push_back(kq[j]);
    }
  } catch (...) {
    fail("out of memory while recording");
  }
  executed = kq.size();
  pthread_mutex_unlock(&rec_lock);
  for (auto &k : run) {
    cl_int ret = k->exec();
    if (ret != CL_SUCCESS) {
      char m[96];
      snprintf(m, sizeof(m), "a recorded kernel failed to run (OpenCL error %d)", ret);
      fail(m);
    }
  }
  if (command_queue == NULL) return CL_SUCCESS;
  cl_int ret = clFinish(command_queue);
  if (ret != CL_SUCCESS) {
    char m[96];
    snprintf(m, sizeof(m), "clFinish failed while recording (OpenCL error %d)", ret);
    fail(m);
  }
  return ret;
}

// *********** OpenCL interceptor (SNPE looks these up with dlsym) ***********

cl_int thneed_clSetKernelArg(cl_kernel kernel, cl_uint arg_index, size_t arg_size, const void *arg_value) {
  cl_int ret = clSetKernelArg(kernel, arg_index, arg_size, arg_value);
  if (ret == CL_SUCCESS && capture_on()) {     // after the recording (or with thneed off): a plain pass-through
    pthread_mutex_lock(&g_args_lock);
    try {
      g_args_size[make_pair(kernel, (int)arg_index)] = arg_size;
      g_args[make_pair(kernel, (int)arg_index)] = arg_value != NULL ? string((const char*)arg_value, arg_size) : string("");
    } catch (...) {
      // out of memory: this argument stays unknown, so a recording that needs it fails (and falls back) by itself
    }
    pthread_mutex_unlock(&g_args_lock);
  }
  return ret;
}

cl_int thneed_clReleaseKernel(cl_kernel kernel) {
  if (capture_on()) {
    cl_uint refs = 0;
    if (clGetKernelInfo(kernel, CL_KERNEL_REFERENCE_COUNT, sizeof(refs), &refs, NULL) == CL_SUCCESS && refs <= 1) {
      pthread_mutex_lock(&g_args_lock);        // this release destroys the kernel: forget its arguments
      g_args.erase(g_args.lower_bound(make_pair(kernel, 0)), g_args.upper_bound(make_pair(kernel, INT_MAX)));
      g_args_size.erase(g_args_size.lower_bound(make_pair(kernel, 0)), g_args_size.upper_bound(make_pair(kernel, INT_MAX)));
      pthread_mutex_unlock(&g_args_lock);
    }
  }
  return clReleaseKernel(kernel);
}

cl_int thneed_clEnqueueNDRangeKernel(cl_command_queue command_queue,
  cl_kernel kernel,
  cl_uint work_dim,
  const size_t *global_work_offset,
  const size_t *global_work_size,
  const size_t *local_work_size,
  cl_uint num_events_in_wait_list,
  const cl_event *event_wait_list,
  cl_event *event) {

  Thneed *thneed = thneed_recording();
  if (thneed != NULL && is_recording(thneed)) {
    // 0.8.3 asserted these ("SNPE doesn't use these"): here such a kernel ends the recording, and the kernels queued
    // so far run first, so this run of SNPE still computes the right thing
    const char *why = NULL;
    if (num_events_in_wait_list != 0 || global_work_offset != NULL || event_wait_list != NULL || work_dim < 1 || work_dim > 3 ||
        global_work_size == NULL) {
      why = "SNPE used an OpenCL feature thneed does not record";
    } else if (thneed->context == NULL) {
      cl_context ctx = NULL;
      size_t dsz = 0;
      if (clGetKernelInfo(kernel, CL_KERNEL_CONTEXT, sizeof(ctx), &ctx, NULL) != CL_SUCCESS || ctx == NULL ||
          clGetContextInfo(ctx, CL_CONTEXT_DEVICES, 0, NULL, &dsz) != CL_SUCCESS || dsz < sizeof(cl_device_id)) {
        why = "could not read SNPE's OpenCL context";
      } else {
        std::vector<cl_device_id> devs(dsz / sizeof(cl_device_id));
        if (clGetContextInfo(ctx, CL_CONTEXT_DEVICES, devs.size() * sizeof(cl_device_id), devs.data(), NULL) != CL_SUCCESS) {
          why = "could not read SNPE's OpenCL device";
        } else {
          thneed->command_queue = command_queue;
          thneed->context = ctx;
          thneed->device_id = devs[0];
        }
      }
    }
    std::shared_ptr<CLQueuedKernel> k;
    if (why == NULL) {
      try {
        k = std::make_shared<CLQueuedKernel>(thneed, kernel, work_dim, global_work_size, local_work_size);
        if (!k->ok) why = k->why.c_str();
      } catch (...) {
        why = "out of memory while recording";
      }
    }
    if (why == NULL) {
      pthread_mutex_lock(&thneed->rec_lock);
      try {
        thneed->kq.push_back(k);
      } catch (...) {
        why = "out of memory while recording";
      }
      pthread_mutex_unlock(&thneed->rec_lock);
    }
    if (why != NULL) {
      thneed->abandon_recording(why);
      return clEnqueueNDRangeKernel(command_queue, kernel, work_dim, global_work_offset, global_work_size, local_work_size,
                                    num_events_in_wait_list, event_wait_list, event);
    }
    // while recording, the kernel is queued here and run at clFinish (as 0.8.3: SNPE gets no event for it)
    if (event != NULL) *event = NULL;
    return CL_SUCCESS;
  }
  return clEnqueueNDRangeKernel(command_queue, kernel, work_dim, global_work_offset, global_work_size, local_work_size,
                                num_events_in_wait_list, event_wait_list, event);
}

cl_int thneed_clFinish(cl_command_queue command_queue) {
  Thneed *thneed = thneed_recording();
  if (thneed != NULL && is_recording(thneed) && thneed->command_queue != NULL) {
    return thneed->clexec();
  }
  return clFinish(command_queue);
}

#ifndef THNEED_TEST
void *dlsym(void *handle, const char *symbol) {
#if defined(QCOM) || defined(QCOM2)
  void *(*my_dlsym)(void *handle, const char *symbol) = (void *(*)(void *handle, const char *symbol))((uintptr_t)dlopen + DLSYM_OFFSET);
#else
  #error "Unsupported platform for thneed"
#endif
  if (memcmp("REAL_", symbol, 5) == 0) {
    return my_dlsym(handle, symbol+5);
  } else if (strcmp("clFinish", symbol) == 0) {
    return (void*)thneed_clFinish;
  } else if (strcmp("clEnqueueNDRangeKernel", symbol) == 0) {
    return (void*)thneed_clEnqueueNDRangeKernel;
  } else if (strcmp("clSetKernelArg", symbol) == 0) {
    return (void*)thneed_clSetKernelArg;
  } else if (strcmp("clReleaseKernel", symbol) == 0) {
    return (void*)thneed_clReleaseKernel;
  } else {
    return my_dlsym(handle, symbol);
  }
}
#endif

// *********** CLQueuedKernel ***********

// The kernel's name and argument count are required (they say which kernel this is); its argument names are only
// needed for SNPE's input/output kernels (map_io checks those).
CLQueuedKernel::CLQueuedKernel(Thneed *lthneed,
                               cl_kernel _kernel,
                               cl_uint _work_dim,
                               const size_t *_global_work_size,
                               const size_t *_local_work_size) {
  thneed = lthneed;
  kernel = _kernel;
  work_dim = _work_dim;
  has_local = _local_work_size != NULL;                    // PORT: 0.8.3 assumed SNPE always passes one
  for (unsigned int i = 0; i < work_dim && i < 3; i++) {
    global_work_size[i] = _global_work_size[i];
    local_work_size[i] = has_local ? _local_work_size[i] : 0;
  }

  char _name[0x100] = {0};
  if (clGetKernelInfo(kernel, CL_KERNEL_FUNCTION_NAME, sizeof(_name) - 1, _name, NULL) != CL_SUCCESS) {
    ok = false; why = "could not read a kernel's name"; return;
  }
  name = string(_name);
  if (clGetKernelInfo(kernel, CL_KERNEL_NUM_ARGS, sizeof(num_args), &num_args, NULL) != CL_SUCCESS || num_args > 1024) {
    ok = false; why = "could not read a kernel's argument count"; return;
  }

  bool names_ok = true;
  for (unsigned int i = 0; i < num_args && names_ok; i++) {
    char arg_name[0x100] = {0}, arg_type[0x100] = {0};
    if (clGetKernelArgInfo(kernel, i, CL_KERNEL_ARG_NAME, sizeof(arg_name) - 1, arg_name, NULL) != CL_SUCCESS ||
        clGetKernelArgInfo(kernel, i, CL_KERNEL_ARG_TYPE_NAME, sizeof(arg_type) - 1, arg_type, NULL) != CL_SUCCESS) {
      names_ok = false;                       // arg_names stays shorter than num_args: map_io sees it if it matters
      break;
    }
    arg_names.push_back(string(arg_name));
    arg_types.push_back(string(arg_type));
  }
  pthread_mutex_lock(&g_args_lock);
  for (unsigned int i = 0; i < num_args; i++) {
    auto a = g_args.find(make_pair(kernel, (int)i));
    auto s = g_args_size.find(make_pair(kernel, (int)i));
    args.push_back(a != g_args.end() ? a->second : string());
    args_size.push_back(s != g_args_size.end() ? s->second : 0);
  }
  pthread_mutex_unlock(&g_args_lock);
  if (clGetKernelInfo(kernel, CL_KERNEL_PROGRAM, sizeof(program), &program, NULL) != CL_SUCCESS) program = NULL;   // debug only
}

cl_int CLQueuedKernel::exec() {
  if (thneed->record & THNEED_DEBUG) debug_print(thneed->record & THNEED_VERBOSE_DEBUG);
  return clEnqueueNDRangeKernel(thneed->command_queue,
    kernel, work_dim, NULL, global_work_size, has_local ? local_work_size : NULL, 0, NULL, NULL);
}

void CLQueuedKernel::debug_print(bool verbose) {
  printf("%p %56s -- ", (void *)kernel, name.c_str());
  for (unsigned int i = 0; i < work_dim; i++) printf("%4zu ", global_work_size[i]);
  printf(" -- ");
  for (unsigned int i = 0; i < work_dim; i++) printf("%4zu ", local_work_size[i]);
  printf("\n");
  if (verbose) {
    for (unsigned int i = 0; i < num_args && i < arg_names.size(); i++) printf("  %s %s (%d bytes)\n", arg_types[i].c_str(), arg_names[i].c_str(), args_size[i]);
  }
}
