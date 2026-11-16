#include <cassert>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "common/util.h"
#include "snpemodel.h"
#ifdef USE_THNEED
#include "../thneed/input_map.h"
#endif

void PrintErrorStringAndExit() {
  const char* const errStr = zdl::DlSystem::getLastErrorString();
  std::cerr << zdl::DlSystem::getLastErrorString() << std::endl;
  std::exit(EXIT_FAILURE);
}

SNPEModel::SNPEModel(const char *path, float *output, size_t output_size, int runtime) {
  output_buf = output;
  output_floats = output_size;
#ifdef QCOM
  zdl::DlSystem::Runtime_t Runtime;
  if (runtime==USE_GPU_RUNTIME) {
    Runtime = zdl::DlSystem::Runtime_t::GPU;
  } else if (runtime==USE_DSP_RUNTIME) {
    Runtime = zdl::DlSystem::Runtime_t::DSP;
  } else {
    Runtime = zdl::DlSystem::Runtime_t::CPU;
  }
  assert(zdl::SNPE::SNPEFactory::isRuntimeAvailable(Runtime));
  runtime_used = Runtime;
#endif
  size_t model_size;
  model_data = (uint8_t *)read_file(path, &model_size);
  assert(model_data);

  // load model
  std::unique_ptr<zdl::DlContainer::IDlContainer> container = zdl::DlContainer::IDlContainer::open(model_data, model_size);
  if (!container) { PrintErrorStringAndExit(); }
  printf("loaded model with size: %u\n", model_size);

  // create model runner
  zdl::SNPE::SNPEBuilder snpeBuilder(container.get());
  while (!snpe) {
#ifdef QCOM
    snpe = snpeBuilder.setOutputLayers({})
                      .setRuntimeProcessor(Runtime)
                      .setUseUserSuppliedBuffers(true)
                      .setPerformanceProfile(zdl::DlSystem::PerformanceProfile_t::HIGH_PERFORMANCE)
                      .build();
#else
    snpe = snpeBuilder.setOutputLayers({})
                      .setUseUserSuppliedBuffers(true)
                      .setPerformanceProfile(zdl::DlSystem::PerformanceProfile_t::HIGH_PERFORMANCE)
                      .build();
#endif
    if (!snpe) std::cerr << zdl::DlSystem::getLastErrorString() << std::endl;
  }

  // get input and output names
  const auto &strListi_opt = snpe->getInputTensorNames();
  if (!strListi_opt) throw std::runtime_error("Error obtaining Input tensor names");
  const auto &strListi = *strListi_opt;
  //assert(strListi.size() == 1);
  const char *input_tensor_name = strListi.at(0);

  const auto &strListo_opt = snpe->getOutputTensorNames();
  if (!strListo_opt) throw std::runtime_error("Error obtaining Output tensor names");
  const auto &strListo = *strListo_opt;
  assert(strListo.size() == 1);
  const char *output_tensor_name = strListo.at(0);

  printf("model: %s -> %s\n", input_tensor_name, output_tensor_name);

  zdl::DlSystem::UserBufferEncodingFloat userBufferEncodingFloat;
  zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();

  // create input buffer
  {
    const auto &inputDims_opt = snpe->getInputDimensions(input_tensor_name);
    const zdl::DlSystem::TensorShape& bufferShape = *inputDims_opt;
    std::vector<size_t> strides(bufferShape.rank());
    strides[strides.size() - 1] = sizeof(float);
    size_t product = 1;
    for (size_t i = 0; i < bufferShape.rank(); i++) product *= bufferShape[i];
    size_t stride = strides[strides.size() - 1];
    for (size_t i = bufferShape.rank() - 1; i > 0; i--) {
      stride *= bufferShape[i];
      strides[i-1] = stride;
    }
    printf("input product is %u\n", product);
    input_floats = product;
    inputBuffer = ubFactory.createUserBuffer(NULL, product*sizeof(float), strides, &userBufferEncodingFloat);

    inputMap.add(input_tensor_name, inputBuffer.get());
  }

  // create output buffer
  {
    std::vector<size_t> outputStrides = {output_size * sizeof(float), sizeof(float)};
    outputBuffer = ubFactory.createUserBuffer(output, output_size * sizeof(float), outputStrides, &userBufferEncodingFloat);
    outputMap.add(output_tensor_name, outputBuffer.get());
  }
}

void SNPEModel::addRecurrent(float *state, int state_size) {
  recurrent = state; recurrent_size = state_size;
  recurrentBuffer = this->addExtra(state, state_size, 3);
}

void SNPEModel::addTrafficConvention(float *state, int state_size) {
  trafficConvention = state; trafficConvention_size = state_size;
  trafficConventionBuffer = this->addExtra(state, state_size, 2);
}

void SNPEModel::addDesire(float *state, int state_size) {
  desire = state; desire_size = state_size;
  desireBuffer = this->addExtra(state, state_size, 1);
}

std::unique_ptr<zdl::DlSystem::IUserBuffer> SNPEModel::addExtra(float *state, int state_size, int idx) {
  // get input and output names
  const auto &strListi_opt = snpe->getInputTensorNames();
  if (!strListi_opt) throw std::runtime_error("Error obtaining Input tensor names");
  const auto &strListi = *strListi_opt;
  const char *input_tensor_name = strListi.at(idx);
  printf("adding index %d: %s\n", idx, input_tensor_name);

  zdl::DlSystem::UserBufferEncodingFloat userBufferEncodingFloat;
  zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
  std::vector<size_t> retStrides = {state_size * sizeof(float), sizeof(float)};
  auto ret = ubFactory.createUserBuffer(state, state_size * sizeof(float), retStrides, &userBufferEncodingFloat);
  inputMap.add(input_tensor_name, ret.get());
  return ret;
}

#ifdef USE_THNEED
// THNEED, first frame (as 0.8.3's SNPEModel::execute): one normal SNPE run, then a RECORDED run with the memory state
// zeroed, then the replay with the same inputs must give the recorded run's output BIT FOR BIT. Any problem: plain SNPE
// from then on (0.8.3 stopped modeld). This frame's output is valid either way.
int SNPEModel::thneed_start(float *net_input_buf) {
  if (access("/data/thneed_off", F_OK) == 0 || getenv("THNEED_OFF") != NULL) {
    printf("thneed: switched off (/data/thneed_off or THNEED_OFF) - plain SNPE\n");
    return -1;
  }
  if (!recurrent || !trafficConvention || !desire || !output_buf || input_floats == 0) {
    printf("thneed: this model does not have the 4 inputs thneed expects - plain SNPE\n");
    return -1;
  }
  assert(inputBuffer->setBufferAddress(net_input_buf));
  if (!snpe->execute(inputMap, outputMap)) PrintErrorStringAndExit();

  memset(recurrent, 0, recurrent_size*sizeof(float));
  thneed = new Thneed();
  if (!snpe->execute(inputMap, outputMap)) PrintErrorStringAndExit();
  thneed->stop();
  g_thneed = NULL;                         // from here on the interceptors only pass calls through

  float *golden = (float *)malloc(output_floats*sizeof(float));
  memcpy(golden, output_buf, output_floats*sizeof(float));
  int result = -1;
  const size_t ours[4] = {input_floats*sizeof(float), recurrent_size*sizeof(float), desire_size*sizeof(float),
                          trafficConvention_size*sizeof(float)};
  float *ourp[4] = {net_input_buf, recurrent, desire, trafficConvention};
  int perm[4];
  if (thneed->failed) {
    printf("thneed: recording unusable (%s) - plain SNPE\n", thneed->why_failed.c_str());
  } else if (thneed->inputs.size() != 4 || thneed->output == NULL ||
             !thneed_match_inputs(thneed->input_sizes.data(), 4, ours, 4, perm)) {
    printf("thneed: found %zu inputs (want 4), output %s - not this SNPE's layout - plain SNPE\n",
           thneed->inputs.size(), thneed->output ? "found" : "NOT found");
    for (size_t i = 0; i < thneed->input_sizes.size(); i++) printf("  thneed input %zu: %zu bytes\n", i, thneed->input_sizes[i]);
  } else {
    for (int i = 0; i < 4; i++) {
      thneed_in[i] = ourp[perm[i]]; thneed_in_bytes[i] = ours[perm[i]];
      if (perm[i] == 0) thneed_image = i;
    }
    memset(output_buf, 0, output_floats*sizeof(float));      // also zeroes the memory state (it lives in the output)
    memset(recurrent, 0, recurrent_size*sizeof(float));
    bool ran = thneed->execute(thneed_in, thneed_in_bytes, output_buf);
    if (ran && memcmp(output_buf, golden, output_floats*sizeof(float)) == 0) {
      printf("thneed selftest passed (%zu GPU commands replayed)\n", thneed->cmds.size());
      result = 1;
    } else {
      size_t bad = 0; float worst = 0.f;
      for (size_t i = 0; i < output_floats; i++) {
        if (output_buf[i] != golden[i]) { bad++; float d = fabsf(output_buf[i] - golden[i]); if (d > worst) worst = d; }
      }
      printf("thneed selftest FAILED (replay %s, %zu of %zu outputs differ, largest difference %g) - plain SNPE\n",
             ran ? "ran" : "did not run", bad, output_floats, worst);
    }
  }
  if (result != 1) memcpy(output_buf, golden, output_floats*sizeof(float));   // this frame: the recorded run's output
  free(golden);
  return result;
}
#endif

void SNPEModel::execute(float *net_input_buf, int buf_size) {
#ifdef USE_THNEED
  if (thneed_state >= 0 && runtime_used == zdl::DlSystem::Runtime_t::GPU) {
    if (thneed_state == 0) {
      thneed_state = thneed_start(net_input_buf);
      if (thneed_state == 1 || thneed != NULL) return;          // this frame was computed while recording / testing
    } else {
      thneed_in[thneed_image] = net_input_buf;
      if (thneed->execute(thneed_in, thneed_in_bytes, output_buf)) return;
      printf("thneed: replay failed - plain SNPE from now on\n");
      thneed_state = -1;
    }
  }
#endif
  assert(inputBuffer->setBufferAddress(net_input_buf));
  if (!snpe->execute(inputMap, outputMap)) {
    PrintErrorStringAndExit();
  }
}

