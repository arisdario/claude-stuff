#pragma once
// HOST TESTS ONLY: the part of Qualcomm's SNPE API that runners/snpemodel.cc uses, backed by fake_gpu.cc (a fake
// network that runs its "GPU" work through thneed's OpenCL interceptors and a fake KGSL device, as SNPE does on the EON).
#include <stddef.h>
#include <stdint.h>
#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace zdl {
namespace DlSystem {
enum class Runtime_t { CPU, GPU, DSP };
enum class PerformanceProfile_t { HIGH_PERFORMANCE };

class StringList {
 public:
  StringList() {}
  StringList(std::initializer_list<const char *> l) { for (auto s : l) v.push_back(s); }
  size_t size() const { return v.size(); }
  const char *at(size_t i) const { return v.at(i).c_str(); }
  std::vector<std::string> v;
};

template <class T> class Optional {
 public:
  Optional() : ok(false) {}
  Optional(const T &t) : ok(true), val(t) {}
  explicit operator bool() const { return ok; }
  const T &operator*() const { return val; }
  bool ok;
  T val;
};

class TensorShape {
 public:
  TensorShape() {}
  TensorShape(std::vector<size_t> d) : dims(d) {}
  size_t rank() const { return dims.size(); }
  size_t operator[](size_t i) const { return dims[i]; }
  std::vector<size_t> dims;
};

class UserBufferEncoding { public: virtual ~UserBufferEncoding() {} };
class UserBufferEncodingFloat : public UserBufferEncoding {};

class IUserBuffer {
 public:
  IUserBuffer(void *p, size_t n) : addr(p), bytes(n) {}
  virtual ~IUserBuffer() {}
  bool setBufferAddress(void *p);           // fake_gpu.cc (can be made to refuse)
  void *addr;
  size_t bytes;
};

class UserBufferMap {
 public:
  void add(const char *name, IUserBuffer *b) { m[name] = b; }
  IUserBuffer *get(const std::string &n) const { auto it = m.find(n); return it == m.end() ? NULL : it->second; }
  std::map<std::string, IUserBuffer *> m;
};

class IUserBufferFactory {
 public:
  std::unique_ptr<IUserBuffer> createUserBuffer(void *p, size_t bytes, const std::vector<size_t> &, UserBufferEncoding *) {
    return std::unique_ptr<IUserBuffer>(new IUserBuffer(p, bytes));
  }
};

const char *getLastErrorString();
}  // namespace DlSystem

namespace DlContainer {
class IDlContainer {
 public:
  static std::unique_ptr<IDlContainer> open(const uint8_t *, size_t) { return std::unique_ptr<IDlContainer>(new IDlContainer()); }
};
}  // namespace DlContainer

namespace SNPE {
struct FakeNet;
class SNPE {
 public:
  explicit SNPE(FakeNet *n) : net(n) {}
  ~SNPE();
  DlSystem::Optional<DlSystem::StringList> getInputTensorNames() const;
  DlSystem::Optional<DlSystem::StringList> getOutputTensorNames() const;
  DlSystem::Optional<DlSystem::TensorShape> getInputDimensions(const char *name) const;
  bool execute(DlSystem::UserBufferMap &in, DlSystem::UserBufferMap &out);
  FakeNet *net;
};

class SNPEBuilder {
 public:
  explicit SNPEBuilder(DlContainer::IDlContainer *) {}
  SNPEBuilder &setOutputLayers(DlSystem::StringList) { return *this; }
  SNPEBuilder &setRuntimeProcessor(DlSystem::Runtime_t) { return *this; }
  SNPEBuilder &setUseUserSuppliedBuffers(bool) { return *this; }
  SNPEBuilder &setPerformanceProfile(DlSystem::PerformanceProfile_t) { return *this; }
  std::unique_ptr<SNPE> build();
};

class SNPEFactory {
 public:
  static bool isRuntimeAvailable(DlSystem::Runtime_t) { return true; }
  static DlSystem::IUserBufferFactory &getUserBufferFactory() { static DlSystem::IUserBufferFactory f; return f; }
};
}  // namespace SNPE
}  // namespace zdl
