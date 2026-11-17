// HOST TESTS ONLY - see fake_gpu.h.
#include "fake_gpu.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <set>

extern "C" int ioctl(int, unsigned long, void *);   // thneed's interceptor (thneed.cc)

FakeKnobs K;
FakeStats S;

static int g_fd_rw = -1, g_fd_wo = -1, g_driver_fd = -1;
static unsigned int g_next_id = 0;
static unsigned int g_ctx = 0, g_snpe_ts = 0;
static int g_launches = 0;
static cl_kernel g_last_net = NULL;

#define FAKE_MAGIC 0x7e57c0deu
struct FakeCmd { uint32_t magic; float *in[4]; float *out; };   // what a "GPU command buffer" holds
static std::set<const FakeCmd *> g_snpe_cmdbufs;                // the driver's own (anything else = thneed's copy)

struct FakeMem { size_t size = 0; std::vector<unsigned char> data; int refs = 1; int maps = 0; };
struct FakeKernel { std::string name; std::vector<std::string> argn; std::vector<cl_mem> argv; int refs = 1; };
struct FakeQueue { int refs = 1; };
static std::vector<FakeMem *> g_mems;
static std::vector<FakeKernel *> g_kernels, g_free_kernels;
static std::vector<FakeQueue *> g_queues;
static FakeMem *M(cl_mem m) { return (FakeMem *)m; }
static FakeKernel *KN(cl_kernel k) { return (FakeKernel *)k; }

void fake_net(const float *img, const float *rec, const float *des, const float *tc, float *out) {
  for (int j = 0; j < N_OUT; j++) {
    out[j] = img[(j * 7) % N_IMG] * 0.25f + rec[j % N_REC] * 0.5f + des[j % N_DES] + tc[j % N_TC] * 2.f + (float)j * 0.001f;
  }
}

cl_mem fake_new_mem(size_t bytes) {
  FakeMem *m = new FakeMem;
  m->size = bytes;
  m->data.assign(bytes, 0);
  g_mems.push_back(m);
  return (cl_mem)m;
}

cl_kernel fake_new_kernel(const char *name, std::vector<std::string> args) {
  FakeKernel *k;
  if (!g_free_kernels.empty()) { k = g_free_kernels.back(); g_free_kernels.pop_back(); *k = FakeKernel(); }   // same address again
  else { k = new FakeKernel; g_kernels.push_back(k); }
  k->name = name;
  k->argn = args;
  k->argv.assign(args.size(), (cl_mem)NULL);
  return (cl_kernel)k;
}

cl_command_queue fake_new_queue() {
  FakeQueue *q = new FakeQueue;
  g_queues.push_back(q);
  return (cl_command_queue)q;
}

cl_kernel fake_last_network_kernel() { return g_last_net; }
int fake_fd() { return g_fd_rw; }

int fake_refs_extra() {
  int n = 0;
  for (auto m : g_mems) n += m->refs - 1;
  for (auto q : g_queues) n += q->refs - 1;
  return n;
}

int fake_thneed_allocs() {
  int n = 0;
  for (size_t i = 0; i < S.alloc_sizes.size(); i++) n += S.alloc_sizes[i] == 0x200000;
  return n;
}

int fake_thneed_frees() {
  int n = 0;
  for (size_t i = 0; i < S.alloc_sizes.size(); i++) {
    if (S.alloc_sizes[i] != 0x200000) continue;
    for (auto f : S.free_ids) n += f == S.alloc_ids[i];
  }
  return n;
}

// *********** the KGSL device ***********
static int fake_kgsl(int fd, unsigned long req, void *argp) {
  (void)fd;
  if (req == IOCTL_KGSL_GPUOBJ_ALLOC) {
    struct kgsl_gpuobj_alloc *a = (struct kgsl_gpuobj_alloc *)argp;
    if (K.fail_alloc && a->size == 0x200000) { errno = ENOMEM; return -1; }
    a->id = (++g_next_id) * 1024;                          // mmap offset id*4096: 4 MB apart in the memory file
    a->mmapsize = (a->size + 4095) & ~4095ull;
    S.alloc_ids.push_back(a->id);
    S.alloc_sizes.push_back(a->size);
    return 0;
  }
  if (req == IOCTL_KGSL_GPUOBJ_FREE) {
    S.free_ids.push_back(((struct kgsl_gpuobj_free *)argp)->id);
    return 0;
  }
  if (req == IOCTL_KGSL_DRAWCTXT_CREATE) {
    struct kgsl_drawctxt_create *c = (struct kgsl_drawctxt_create *)argp;
    S.ctx_flags.push_back(c->flags);
    if (K.refuse_priority && (c->flags & KGSL_CONTEXT_PRIORITY_MASK)) { errno = EINVAL; return -1; }
    c->drawctxt_id = g_ctx = 3;
    return 0;
  }
  if (req == IOCTL_KGSL_GPU_COMMAND) {
    struct kgsl_gpu_command *cmd = (struct kgsl_gpu_command *)argp;
    if (cmd->numcmds != 1 || cmd->cmdlist == 0) { errno = EINVAL; return -1; }
    const struct kgsl_command_object *obj = (const struct kgsl_command_object *)cmd->cmdlist;
    const FakeCmd *c = (const FakeCmd *)obj->gpuaddr;
    if (c == NULL || obj->size != sizeof(FakeCmd) || c->magic != FAKE_MAGIC) { errno = EINVAL; return -1; }
    const bool replay = g_snpe_cmdbufs.count(c) == 0;
    if (replay) {
      S.replay_cmds++;
      if (K.fail_replay_cmd_at && S.replay_cmds == K.fail_replay_cmd_at) { errno = EIO; return -1; }
    } else {
      S.snpe_cmds++;
    }
    fake_net(c->in[0], c->in[1], c->in[2], c->in[3], c->out);
    if (replay && K.perturb_replay) c->out[0] = nextafterf(c->out[0], INFINITY);
    return 0;
  }
  if (req == IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID) {
    S.waits++;
    if (K.fail_wait_from && S.waits >= K.fail_wait_from) { errno = ETIMEDOUT; return -1; }
    return 0;
  }
  return 0;   // SETPROPERTY, GPUOBJ_SYNC, ...
}

void fake_setup(const char *dir) {
  std::string p = std::string(dir) + "/gpu.mem";
  g_fd_rw = open(p.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0600);
  if (g_fd_rw < 0 || ftruncate(g_fd_rw, 64 << 20) != 0) { perror("fake_setup"); exit(2); }
  g_fd_wo = open(p.c_str(), O_WRONLY);
  thneed_test_kgsl = fake_kgsl;
}

// *********** OpenCL ***********
extern "C" {
cl_int clSetKernelArg(cl_kernel k, cl_uint i, size_t sz, const void *v) {
  if (k == NULL || i >= KN(k)->argn.size()) return CL_INVALID_ARG_INDEX;
  if (sz == sizeof(cl_mem) && v != NULL) KN(k)->argv[i] = *(const cl_mem *)v;
  return CL_SUCCESS;
}

cl_int clGetKernelInfo(cl_kernel k, cl_kernel_info p, size_t sz, void *v, size_t *ret) {
  FakeKernel *fk = KN(k);
  if (p == CL_KERNEL_FUNCTION_NAME) {
    if (K.fail_kernel_name) return CL_INVALID_KERNEL;
    if (sz <= fk->name.size()) return CL_INVALID_VALUE;
    memcpy(v, fk->name.c_str(), fk->name.size() + 1);
    return CL_SUCCESS;
  }
  if (p == CL_KERNEL_NUM_ARGS) { *(cl_uint *)v = (cl_uint)fk->argn.size(); return CL_SUCCESS; }
  if (p == CL_KERNEL_REFERENCE_COUNT) { *(cl_uint *)v = (cl_uint)fk->refs; return CL_SUCCESS; }
  if (p == CL_KERNEL_CONTEXT) { *(cl_context *)v = (cl_context)0xC0; return CL_SUCCESS; }
  if (p == CL_KERNEL_PROGRAM) { *(cl_program *)v = (cl_program)0x9; return CL_SUCCESS; }
  (void)ret;
  return CL_INVALID_VALUE;
}

cl_int clGetKernelArgInfo(cl_kernel k, cl_uint i, cl_kernel_arg_info p, size_t sz, void *v, size_t *ret) {
  (void)ret;
  if (K.fail_arg_info) return CL_KERNEL_ARG_INFO_NOT_AVAILABLE;
  FakeKernel *fk = KN(k);
  if (i >= fk->argn.size()) return CL_INVALID_ARG_INDEX;
  std::string s = p == CL_KERNEL_ARG_NAME ? fk->argn[i] : std::string("float*");
  if (sz <= s.size()) return CL_INVALID_VALUE;
  memcpy(v, s.c_str(), s.size() + 1);
  return CL_SUCCESS;
}

cl_int clGetContextInfo(cl_context, cl_context_info p, size_t sz, void *v, size_t *ret) {
  if (K.fail_context) return CL_INVALID_CONTEXT;
  if (p != CL_CONTEXT_DEVICES) return CL_INVALID_VALUE;
  if (ret) *ret = sizeof(cl_device_id);
  if (v != NULL) {
    if (sz < sizeof(cl_device_id)) return CL_INVALID_VALUE;
    *(cl_device_id *)v = (cl_device_id)0xD0;
  }
  return CL_SUCCESS;
}

cl_int clGetMemObjectInfo(cl_mem m, cl_mem_info p, size_t sz, void *v, size_t *ret) {
  (void)ret;
  if (K.fail_mem_info) return CL_INVALID_MEM_OBJECT;
  if (p != CL_MEM_SIZE || sz < sizeof(size_t)) return CL_INVALID_VALUE;
  *(size_t *)v = M(m)->size;
  return CL_SUCCESS;
}

void *clEnqueueMapBuffer(cl_command_queue, cl_mem m, cl_bool, cl_map_flags, size_t off, size_t sz, cl_uint, const cl_event *, cl_event *, cl_int *err) {
  if (K.fail_map || off + sz > M(m)->size) { if (err) *err = CL_MAP_FAILURE; return NULL; }
  M(m)->maps++;
  S.maps_open++;
  if (err) *err = CL_SUCCESS;
  return M(m)->data.data() + off;
}

cl_int clEnqueueUnmapMemObject(cl_command_queue, cl_mem m, void *p, cl_uint, const cl_event *, cl_event *) {
  if (M(m)->maps <= 0 || p < (void *)M(m)->data.data() || p >= (void *)(M(m)->data.data() + M(m)->size)) return CL_INVALID_VALUE;
  M(m)->maps--;
  S.maps_open--;
  S.unmaps++;
  return CL_SUCCESS;
}

cl_int clEnqueueReadBuffer(cl_command_queue, cl_mem m, cl_bool, size_t off, size_t sz, void *p, cl_uint, const cl_event *, cl_event *) {
  S.reads++;
  if (K.fail_read_at && S.reads == K.fail_read_at) return CL_OUT_OF_RESOURCES;
  if (off + sz > M(m)->size) return CL_INVALID_VALUE;
  memcpy(p, M(m)->data.data() + off, sz);
  return CL_SUCCESS;
}

cl_int clRetainMemObject(cl_mem m) { M(m)->refs++; return CL_SUCCESS; }
cl_int clReleaseMemObject(cl_mem m) { if (M(m)->refs <= 0) return CL_INVALID_MEM_OBJECT; M(m)->refs--; return CL_SUCCESS; }
cl_int clRetainCommandQueue(cl_command_queue q) { ((FakeQueue *)q)->refs++; return CL_SUCCESS; }
cl_int clReleaseCommandQueue(cl_command_queue q) { if (((FakeQueue *)q)->refs <= 0) return CL_INVALID_COMMAND_QUEUE; ((FakeQueue *)q)->refs--; return CL_SUCCESS; }
cl_int clRetainKernel(cl_kernel k) { KN(k)->refs++; return CL_SUCCESS; }
cl_int clReleaseKernel(cl_kernel k) {
  if (KN(k)->refs <= 0) return CL_INVALID_KERNEL;
  if (--KN(k)->refs == 0) g_free_kernels.push_back(KN(k));
  return CL_SUCCESS;
}
cl_int clFinish(cl_command_queue) { return CL_SUCCESS; }

// a kernel launch: only the network does GPU work, and it does it the way the driver does: a GPU command (an ioctl,
// which thneed records) pointing at a command buffer that says which buffers to read and write
cl_int clEnqueueNDRangeKernel(cl_command_queue, cl_kernel k, cl_uint, const size_t *, const size_t *, const size_t *,
                              cl_uint, const cl_event *, cl_event *) {
  FakeKernel *fk = KN(k);
  if (fk->name != "fake_network") return CL_SUCCESS;
  g_launches++;
  if (K.fail_exec_at && g_launches == K.fail_exec_at) return CL_OUT_OF_RESOURCES;
  FakeCmd *c = new FakeCmd;
  c->magic = FAKE_MAGIC;
  for (int i = 0; i < 4; i++) c->in[i] = (float *)M(fk->argv[i])->data.data();
  c->out = (float *)M(fk->argv[4])->data.data();
  g_snpe_cmdbufs.insert(c);
  struct kgsl_command_object obj;
  memset(&obj, 0, sizeof(obj));
  obj.gpuaddr = (uint64_t)c;
  obj.size = sizeof(FakeCmd);
  struct kgsl_gpu_command cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.cmdlist = (uint64_t)&obj;
  cmd.cmdsize = sizeof(obj);
  cmd.numcmds = 1;
  cmd.context_id = g_ctx;
  cmd.timestamp = ++g_snpe_ts;
  return ioctl(g_driver_fd, IOCTL_KGSL_GPU_COMMAND, &cmd) == 0 ? CL_SUCCESS : CL_OUT_OF_RESOURCES;
}
}  // extern "C"

// *********** SNPE ***********
namespace zdl {
namespace DlSystem {
const char *getLastErrorString() { return "fake SNPE error"; }
bool IUserBuffer::setBufferAddress(void *p) { addr = p; return true; }
}  // namespace DlSystem

namespace SNPE {
// SNPE's own buffers for the 4 inputs, in the tensor order img, state, desire, traffic convention
struct FakeNet {
  cl_command_queue q;
  cl_mem in[4], decoy, out;
  cl_kernel pad[4], net, tobuf;
  int order[4] = {1, 3, 2, 0};   // the order SNPE runs its input kernels: state, traffic, desire, image
};
static const char *TENSOR[4] = {"input_imgs", "initial_state", "desire", "traffic_convention"};

static void set_args(FakeNet *n) {
  for (int i = 0; i < 4; i++) {
    cl_mem src = (i == 0 && K.decoy_input) ? n->decoy : n->in[i];
    thneed_clSetKernelArg(n->pad[i], 0, sizeof(cl_mem), &src);
    thneed_clSetKernelArg(n->pad[i], 1, sizeof(cl_mem), &n->in[i]);
    thneed_clSetKernelArg(n->net, i, sizeof(cl_mem), &n->in[i]);
  }
  thneed_clSetKernelArg(n->net, 4, sizeof(cl_mem), &n->out);
  thneed_clSetKernelArg(n->tobuf, 0, sizeof(cl_mem), &n->out);
  thneed_clSetKernelArg(n->tobuf, 1, sizeof(cl_mem), &n->out);
}

std::unique_ptr<SNPE> SNPEBuilder::build() {
  g_driver_fd = K.mmap_fail ? g_fd_wo : g_fd_rw;
  struct kgsl_gpuobj_alloc a;                      // the driver's own GPU memory: thneed learns the KGSL fd here
  memset(&a, 0, sizeof(a));
  a.size = 4096;
  ioctl(g_driver_fd, IOCTL_KGSL_GPUOBJ_ALLOC, &a);
  struct kgsl_drawctxt_create c;
  memset(&c, 0, sizeof(c));
  c.flags = 0x4200ad2;                             // SNPE's flags on the car (user-generated timestamps: 0x80)
  ioctl(g_driver_fd, IOCTL_KGSL_DRAWCTXT_CREATE, &c);

  FakeNet *n = new FakeNet;
  FakeQueue *q = new FakeQueue;
  g_queues.push_back(q);
  n->q = (cl_command_queue)q;
  const size_t sizes[4] = {N_IMG * 4, N_REC * 4, (size_t)(K.layout_mismatch ? 64 : N_DES * 4), (size_t)(K.layout_mismatch ? 64 : N_TC * 4)};
  for (int i = 0; i < 4; i++) n->in[i] = fake_new_mem(sizes[i]);
  n->decoy = fake_new_mem(N_IMG * 4);
  n->out = fake_new_mem(N_OUT * 4);
  if (K.stale_kernel) {                            // a kernel SNPE made and destroyed; its address comes back below
    cl_kernel t = fake_new_kernel("zero_pad_image_float", {"input", "output"});
    cl_mem junk = fake_new_mem(N_IMG * 4);
    thneed_clSetKernelArg(t, 0, sizeof(cl_mem), &junk);
    thneed_clReleaseKernel(t);
    S.stale_args_after_release = (int)thneed_test_captured_args(t);
  }
  for (int i = 0; i < 4; i++) n->pad[i] = fake_new_kernel("zero_pad_image_float", {"input", "output"});
  n->net = g_last_net = fake_new_kernel("fake_network", {"img", "rec", "des", "tc", "out"});
  n->tobuf = fake_new_kernel("image2d_to_buffer_float", {"input", "output"});
  if (K.args_once) set_args(n);
  return std::unique_ptr<SNPE>(new SNPE(n));
}

SNPE::~SNPE() { delete net; }

DlSystem::Optional<DlSystem::StringList> SNPE::getInputTensorNames() const {
  return DlSystem::StringList({"input_imgs", "desire", "traffic_convention", "initial_state"});   // snpemodel.cc: 0 image, 1 desire, 2 traffic, 3 state
}
DlSystem::Optional<DlSystem::StringList> SNPE::getOutputTensorNames() const { return DlSystem::StringList({"outputs"}); }
DlSystem::Optional<DlSystem::TensorShape> SNPE::getInputDimensions(const char *name) const {
  if (!strcmp(name, "input_imgs")) return DlSystem::TensorShape({1, (size_t)N_IMG});
  return DlSystem::Optional<DlSystem::TensorShape>();
}

bool SNPE::execute(DlSystem::UserBufferMap &inm, DlSystem::UserBufferMap &outm) {
  S.snpe_runs++;
  if (K.fail_snpe_run && S.snpe_runs == K.fail_snpe_run) return false;
  for (int i = 0; i < 4; i++) {                    // SNPE copies the user buffers into its own GPU buffers
    DlSystem::IUserBuffer *b = inm.get(TENSOR[i]);
    if (b == NULL || b->addr == NULL) return false;
    memcpy(M(net->in[i])->data.data(), b->addr, b->bytes < M(net->in[i])->size ? b->bytes : M(net->in[i])->size);
  }
  if (!K.args_once) set_args(net);
  const size_t gws[1] = {1}, lws[1] = {1};
  for (int j = 0; j < 4; j++) {
    if (thneed_clEnqueueNDRangeKernel(net->q, net->pad[net->order[j]], 1, NULL, gws, lws, 0, NULL, NULL) != CL_SUCCESS) return false;
  }
  if (thneed_clEnqueueNDRangeKernel(net->q, net->net, 1, NULL, gws, lws, 0, NULL, NULL) != CL_SUCCESS) return false;
  if (thneed_clEnqueueNDRangeKernel(net->q, net->tobuf, 1, NULL, gws, lws, 0, NULL, NULL) != CL_SUCCESS) return false;
  if (thneed_clFinish(net->q) != CL_SUCCESS) return false;
  DlSystem::IUserBuffer *o = outm.get("outputs");
  if (o == NULL || o->addr == NULL) return false;
  memcpy(o->addr, M(net->out)->data.data(), o->bytes < M(net->out)->size ? o->bytes : M(net->out)->size);
  return true;
}
}  // namespace SNPE
}  // namespace zdl
