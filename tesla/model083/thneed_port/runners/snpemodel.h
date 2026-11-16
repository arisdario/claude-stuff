#ifndef SNPEMODEL_H
#define SNPEMODEL_H

#include <SNPE/SNPE.hpp>
#include <SNPE/SNPEBuilder.hpp>
#include <SNPE/SNPEFactory.hpp>
#include <DlContainer/IDlContainer.hpp>
#include <DlSystem/DlError.hpp>
#include <DlSystem/ITensor.hpp>
#include <DlSystem/ITensorFactory.hpp>
#include <DlSystem/IUserBuffer.hpp>
#include <DlSystem/IUserBufferFactory.hpp>

#include "runmodel.h"

// THNEED (tesla/model083/thneed_port, THNEED.md). The class below has THE SAME LAYOUT with or without USE_THNEED:
// driving.cc creates the object, snpemodel.cc builds it, and they are not compiled with the same flags (16 Nov 2026:
// a layout that depended on USE_THNEED made driving.cc allocate a smaller object than the constructor wrote -> SIGBUS).
// So: no #ifdef inside the class, and Thneed is only declared here (its header is included by snpemodel.cc).
class Thneed;

#define USE_CPU_RUNTIME 0
#define USE_GPU_RUNTIME 1
#define USE_DSP_RUNTIME 2

class SNPEModel : public RunModel {
public:
  SNPEModel(const char *path, float *output, size_t output_size, int runtime);
  ~SNPEModel() {
    if (model_data) free(model_data);
  }
  void addRecurrent(float *state, int state_size);
  void addTrafficConvention(float *state, int state_size);
  void addDesire(float *state, int state_size);
  void execute(float *net_input_buf, int buf_size);
private:
  // THNEED: 0 = not tried yet, 1 = running on the replay, -1 = plain SNPE (switched off, or the self-test failed)
  int thneed_state = 0;
  Thneed *thneed = NULL;
  float *thneed_in[4] = {NULL, NULL, NULL, NULL};
  size_t thneed_in_bytes[4] = {0, 0, 0, 0};
  int thneed_image = -1;                 // which of thneed's inputs is the camera image (its pointer is set every frame)
  int thneed_start(float *net_input_buf); // records + self-tests on the first frame (defined only with USE_THNEED)
  zdl::DlSystem::Runtime_t runtime_used = zdl::DlSystem::Runtime_t::CPU;
  float *output_buf = NULL;              // THNEED needs these (0.7.5 did not keep them)
  size_t output_floats = 0;
  size_t input_floats = 0;
  float *recurrent = NULL; size_t recurrent_size = 0;
  float *trafficConvention = NULL; size_t trafficConvention_size = 0;
  float *desire = NULL; size_t desire_size = 0;
  uint8_t *model_data = NULL;

  // snpe model stuff
  std::unique_ptr<zdl::SNPE::SNPE> snpe;

  // snpe input stuff
  zdl::DlSystem::UserBufferMap inputMap;
  std::unique_ptr<zdl::DlSystem::IUserBuffer> inputBuffer;

  // snpe output stuff
  zdl::DlSystem::UserBufferMap outputMap;
  std::unique_ptr<zdl::DlSystem::IUserBuffer> outputBuffer;
  float *output;

  // recurrent and desire
  std::unique_ptr<zdl::DlSystem::IUserBuffer> addExtra(float *state, int state_size, int idx);
  std::unique_ptr<zdl::DlSystem::IUserBuffer> recurrentBuffer;
  std::unique_ptr<zdl::DlSystem::IUserBuffer> trafficConventionBuffer;
  std::unique_ptr<zdl::DlSystem::IUserBuffer> desireBuffer;
};

#endif

