// thneed - port of openpilot 0.8.3's selfdrive/modeld/thneed/thneed.cc (comma.ai, MIT) to Tinkla 0.7.5.a.
// See thneed.h and THNEED.md for what changed. Rule of the port: anything unexpected while RECORDING marks the
// recording failed (the runner then keeps plain SNPE); nothing here may stop modeld.
#include <cassert>
#include <sys/mman.h>
#include <dlfcn.h>
#include <map>
#include <string>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include "common/timing.h"
#include "thneed.h"

using std::string;
using std::map;
using std::pair;
using std::make_pair;
using std::unique_ptr;

Thneed *g_thneed = NULL;
int g_fd = -1;
map<pair<cl_kernel, int>, string> g_args;
map<pair<cl_kernel, int>, int> g_args_size;

// *********** ioctl interceptor (as 0.8.3) ***********

extern "C" {

int (*my_ioctl)(int filedes, unsigned long request, void *argp) = NULL;
#undef ioctl
int ioctl(int filedes, unsigned long request, void *argp) {
  request &= 0xFFFFFFFF;  // needed on QCOM2
  if (my_ioctl == NULL) my_ioctl = reinterpret_cast<decltype(my_ioctl)>(dlsym(RTLD_NEXT, "ioctl"));
  Thneed *thneed = g_thneed;

  // save the fd
  if (request == IOCTL_KGSL_GPUOBJ_ALLOC) g_fd = filedes;

  // note that this runs always, even without a thneed object (0.8.3: the model's GPU work gets top priority).
  // PORT: comma's NEOS 14 thneed (0.7.7-0.7.9) did not do this; if this kernel refuses the priority flag, the context
  // is created again with SNPE's own flags, so SNPE can always start.
  if (request == IOCTL_KGSL_DRAWCTXT_CREATE) {
    struct kgsl_drawctxt_create *create = (struct kgsl_drawctxt_create *)argp;
    const unsigned int orig_flags = create->flags;
    create->flags &= ~KGSL_CONTEXT_PRIORITY_MASK;
    create->flags |= 1 << KGSL_CONTEXT_PRIORITY_SHIFT;   // priority from 1-15, 1 is max priority
    int r = my_ioctl(filedes, request, argp);
    printf("IOCTL_KGSL_DRAWCTXT_CREATE: context with flags 0x%x: %s\n", create->flags, r == 0 ? "ok" : "refused, retrying with SNPE's flags");
    if (r == 0) return 0;
    create->flags = orig_flags;
    return my_ioctl(filedes, request, argp);
  }

  if (thneed != NULL && (thneed->record & THNEED_RECORD)) {
    if (request == IOCTL_KGSL_GPU_COMMAND) {
      struct kgsl_gpu_command *cmd = (struct kgsl_gpu_command *)argp;
      thneed->timestamp = cmd->timestamp;
      thneed->context_id = cmd->context_id;
      thneed->cmds.push_back(unique_ptr<CachedCommand>(new CachedCommand(thneed, cmd)));
    } else if (request == IOCTL_KGSL_GPUOBJ_SYNC) {
      struct kgsl_gpuobj_sync *cmd = (struct kgsl_gpuobj_sync *)argp;
      struct kgsl_gpuobj_sync_obj *objs = (struct kgsl_gpuobj_sync_obj *)(cmd->objs);
      thneed->cmds.push_back(unique_ptr<CachedSync>(new
            CachedSync(thneed, string((char *)objs, sizeof(struct kgsl_gpuobj_sync_obj)*cmd->count))));
    }
  }

  int ret = my_ioctl(filedes, request, argp);
  // PORT: only the GPU driver's failures are reported (0.8.3 printed every failed ioctl in the process, e.g. errno 25
  // from devices that are not the GPU - 44 lines of noise per start on this EON)
  if (ret != 0 && ((request >> 8) & 0xff) == KGSL_IOC_TYPE) fprintf(stderr, "thneed: GPU ioctl 0x%lx returned %d, errno %d\n", request, ret, errno);
  return ret;
}

}

// *********** GPUMalloc ***********

GPUMalloc::GPUMalloc(int size, int fd) {
  struct kgsl_gpuobj_alloc alloc;
  memset(&alloc, 0, sizeof(alloc));
  alloc.size = size;
  alloc.flags = 0x10000a00;
  if (ioctl(fd, IOCTL_KGSL_GPUOBJ_ALLOC, &alloc) != 0) return;
  void *addr = mmap64(NULL, alloc.mmapsize, 0x3, 0x1, fd, alloc.id*0x1000);
  if (addr == MAP_FAILED) return;
  base = (uint64_t)addr;
  remaining = size;
}

void *GPUMalloc::alloc(int size) {
  size = (size+0xff) & (~0xFF);
  if (base == 0 || size > remaining) return NULL;
  void *ret = (void*)base;
  remaining -= size;
  base += size;
  return ret;
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

CachedCommand::CachedCommand(Thneed *lthneed, struct kgsl_gpu_command *cmd) {
  thneed = lthneed;
  memcpy(&cache, cmd, sizeof(cache));
  if (cmd->numsyncs != 0) { thneed->fail("GPU command with syncs"); return; }

  if (cmd->numcmds > 0) {
    cmds = std::unique_ptr<kgsl_command_object[]>(new kgsl_command_object[cmd->numcmds]);
    memcpy(cmds.get(), (void *)cmd->cmdlist, sizeof(struct kgsl_command_object)*cmd->numcmds);
    cache.cmdlist = (uint64_t)cmds.get();
    for (unsigned int i = 0; i < cmd->numcmds; i++) {
      void *nn = thneed->ram->alloc(cmds[i].size);
      if (nn == NULL) { thneed->fail("command memory used up"); return; }
      memcpy(nn, (void*)cmds[i].gpuaddr, cmds[i].size);
      cmds[i].gpuaddr = (uint64_t)nn;
    }
  }

  if (cmd->numobjs > 0) {
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
  cache.timestamp = ++thneed->timestamp;
  int ret = ioctl(thneed->fd, IOCTL_KGSL_GPU_COMMAND, &cache);
  if (thneed->record & THNEED_DEBUG) printf("CachedCommand::exec got %d\n", ret);
  return ret;
}

// *********** Thneed ***********

Thneed::Thneed() {
  fd = g_fd;
  record = THNEED_RECORD;
  timestamp = -1;
  if (fd == -1) { fail("no GPU device seen yet"); return; }
  ram = std::unique_ptr<GPUMalloc>(new GPUMalloc(0x200000, fd));      // 0.8.3: 0x80000 - more room, it is cheap
  if (!ram->ok()) { fail("could not allocate command memory"); return; }
  g_thneed = this;
}

void Thneed::fail(const char *why) {
  if (!failed) printf("thneed: recording unusable: %s\n", why);
  if (!failed) why_failed = why;
  failed = true;
}

void Thneed::stop() {
  find_inputs_outputs();
  printf("Thneed::stop: recorded %zu commands, %zu inputs, output %s\n", cmds.size(), inputs.size(), output ? "found" : "NOT found");
  record = 0;
}

void Thneed::find_inputs_outputs() {
  cl_int err;
  if (inputs.size() > 0 || failed) return;

  // save the global inputs/outputs (SNPE's own kernel names, as 0.8.3)
  for (auto &k : kq) {
    for (unsigned int i = 0; i < k->num_args; i++) {
      if (k->name == "zero_pad_image_float" && k->arg_names[i] == "input") {
        if (k->args[i].size() != sizeof(cl_mem)) { fail("input argument has an unexpected size"); return; }
        cl_mem aa = *(cl_mem*)(k->args[i].data());
        size_t sz = 0;
        clGetMemObjectInfo(aa, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
        void *ret = clEnqueueMapBuffer(command_queue, aa, CL_TRUE, CL_MAP_WRITE, 0, sz, 0, NULL, NULL, &err);
        if (err != CL_SUCCESS || ret == NULL) { fail("could not map an input"); return; }
        input_sizes.push_back(sz);
        inputs.push_back(ret);
      }
      if (k->name == "image2d_to_buffer_float" && k->arg_names[i] == "output") {
        if (k->args[i].size() != sizeof(cl_mem)) { fail("output argument has an unexpected size"); return; }
        output = *(cl_mem*)(k->args[i].data());
      }
    }
  }
}

void Thneed::copy_inputs(float **finputs, const size_t *fbytes) {
  for (size_t idx = 0; idx < inputs.size(); ++idx) {
    size_t n = fbytes[idx] < input_sizes[idx] ? fbytes[idx] : input_sizes[idx];
    memcpy(inputs[idx], finputs[idx], n);
    if (n < input_sizes[idx]) memset((char *)inputs[idx] + n, 0, input_sizes[idx] - n);
  }
}

void Thneed::copy_output(float *foutput) {
  if (output != NULL) {
    size_t sz;
    clGetMemObjectInfo(output, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
    clEnqueueReadBuffer(command_queue, output, CL_TRUE, 0, sz, foutput, 0, NULL, NULL);
  }
}

void Thneed::wait() {
  struct kgsl_device_waittimestamp_ctxtid wait;
  wait.context_id = context_id;
  wait.timestamp = timestamp;
  wait.timeout = -1;
  ioctl(fd, IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID, &wait);
}

bool Thneed::execute(float **finputs, const size_t *fbytes, float *foutput, bool slow) {
  if (failed || output == NULL) return false;
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

  // ****** run commands
  bool ok = true;
  size_t i = 0;
  for (auto &it : cmds) {
    ++i;
    if (it->exec() != 0) { ok = false; break; }
    if ((i == cmds.size()) || slow) wait();
  }

  if (ok) copy_output(foutput);

  // ****** unset power constraint
  constraint.type = KGSL_CONSTRAINT_NONE;
  constraint.data = NULL;
  constraint.size = 0;
  ioctl(fd, IOCTL_KGSL_SETPROPERTY, &prop);
  return ok;
}

// PORT: 0.8.3 ran ALL recorded kernels at every clFinish; here only the ones queued since the previous clFinish (the
// same thing when SNPE calls clFinish once per run, correct if this older SNPE calls it more often)
static size_t g_executed = 0;

cl_int Thneed::clexec() {
  for (size_t j = g_executed; j < kq.size(); j++) {
    if (record & THNEED_RECORD) ckq.push_back(kq[j]);
    cl_int ret = kq[j]->exec();
    if (ret != CL_SUCCESS) fail("a recorded kernel failed to run");
  }
  g_executed = kq.size();
  return clFinish(command_queue);
}

// *********** OpenCL interceptor (SNPE looks these up with dlsym) ***********

cl_int thneed_clSetKernelArg(cl_kernel kernel, cl_uint arg_index, size_t arg_size, const void *arg_value) {
  g_args_size[make_pair(kernel, (int)arg_index)] = arg_size;
  if (arg_value != NULL) {
    g_args[make_pair(kernel, (int)arg_index)] = string((char*)arg_value, arg_size);
  } else {
    g_args[make_pair(kernel, (int)arg_index)] = string("");
  }
  return clSetKernelArg(kernel, arg_index, arg_size, arg_value);
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

  Thneed *thneed = g_thneed;
  if (thneed != NULL && (thneed->record & THNEED_RECORD)) {
    // 0.8.3 asserted these ("SNPE doesn't use these"): here such a kernel ends the recording, and the kernels queued
    // so far run first, so this run of SNPE still computes the right thing
    if (num_events_in_wait_list != 0 || global_work_offset != NULL || event_wait_list != NULL || work_dim > 3) {
      thneed->fail("SNPE used an OpenCL feature thneed does not record");
      thneed->clexec();
      thneed->record = 0;
      return clEnqueueNDRangeKernel(command_queue, kernel, work_dim, global_work_offset, global_work_size, local_work_size,
                                    num_events_in_wait_list, event_wait_list, event);
    }
    if (thneed->context == NULL) {
      thneed->command_queue = command_queue;
      clGetKernelInfo(kernel, CL_KERNEL_CONTEXT, sizeof(thneed->context), &thneed->context, NULL);
      clGetContextInfo(thneed->context, CL_CONTEXT_DEVICES, sizeof(thneed->device_id), &thneed->device_id, NULL);
    }
    // while recording, the kernel is queued here and run at clFinish
    thneed->kq.push_back(unique_ptr<CLQueuedKernel>(new CLQueuedKernel(thneed, kernel, work_dim, global_work_size, local_work_size)));
    if (event != NULL) *event = NULL;
    return CL_SUCCESS;
  }
  return clEnqueueNDRangeKernel(command_queue, kernel, work_dim, global_work_offset, global_work_size, local_work_size,
                                num_events_in_wait_list, event_wait_list, event);
}

cl_int thneed_clFinish(cl_command_queue command_queue) {
  Thneed *thneed = g_thneed;
  if (thneed != NULL && (thneed->record & THNEED_RECORD)) {
    return thneed->clexec();
  }
  return clFinish(command_queue);
}

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
  } else {
    return my_dlsym(handle, symbol);
  }
}

// *********** CLQueuedKernel ***********

CLQueuedKernel::CLQueuedKernel(Thneed *lthneed,
                               cl_kernel _kernel,
                               cl_uint _work_dim,
                               const size_t *_global_work_size,
                               const size_t *_local_work_size) {
  thneed = lthneed;
  kernel = _kernel;
  work_dim = _work_dim;
  has_local = _local_work_size != NULL;                    // PORT: 0.8.3 assumed SNPE always passes one
  for (unsigned int i = 0; i < work_dim; i++) {
    global_work_size[i] = _global_work_size[i];
    local_work_size[i] = has_local ? _local_work_size[i] : 0;
  }

  char _name[0x100] = {0};
  clGetKernelInfo(kernel, CL_KERNEL_FUNCTION_NAME, sizeof(_name), _name, NULL);
  name = string(_name);
  num_args = 0;
  clGetKernelInfo(kernel, CL_KERNEL_NUM_ARGS, sizeof(num_args), &num_args, NULL);

  for (unsigned int i = 0; i < num_args; i++) {
    char arg_name[0x100] = {0};
    clGetKernelArgInfo(kernel, i, CL_KERNEL_ARG_NAME, sizeof(arg_name), arg_name, NULL);
    arg_names.push_back(string(arg_name));
    clGetKernelArgInfo(kernel, i, CL_KERNEL_ARG_TYPE_NAME, sizeof(arg_name), arg_name, NULL);
    arg_types.push_back(string(arg_name));
    args.push_back(g_args[make_pair(kernel, (int)i)]);
    args_size.push_back(g_args_size[make_pair(kernel, (int)i)]);
  }
  clGetKernelInfo(kernel, CL_KERNEL_PROGRAM, sizeof(program), &program, NULL);
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
    for (unsigned int i = 0; i < num_args; i++) printf("  %s %s (%d bytes)\n", arg_types[i].c_str(), arg_names[i].c_str(), args_size[i]);
  }
}
