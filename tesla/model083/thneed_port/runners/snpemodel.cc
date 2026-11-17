#include <cassert>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <algorithm>
#include "common/util.h"
#include "snpemodel.h"
#ifdef USE_THNEED
#include <new>
#include "../thneed/thneed.h"
#include "../thneed/input_map.h"
#include "../thneed/selftest.h"
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

SNPEModel::~SNPEModel() {
#ifdef USE_THNEED
  thneed_drop();                         // before SNPE goes: thneed unmaps SNPE's buffers and returns its references
#endif
  if (model_data) free(model_data);
  model_data = NULL;
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
// THNEED (tesla/model083/thneed_port, THNEED.md). Fail-safe: any problem while recording, mapping, testing or
// replaying switches thneed off for good and this model continues on plain SNPE - nothing here ends modeld. (The one
// exit left is SNPE's own: a failing NORMAL run is handled exactly as plain execute() handles it.)

void SNPEModel::thneed_drop() {
  if (thneed != NULL) {
    delete thneed;                       // waits for its GPU work, unmaps SNPE's buffers, returns its references and memory
    thneed = NULL;
  }
  thneed_image = -1;
  for (int i = 0; i < 4; i++) { thneed_in[i] = NULL; thneed_in_bytes[i] = 0; }
}

void SNPEModel::thneed_give_up(const char *why) {
  if (why != NULL) fprintf(stderr, "thneed: %s - plain SNPE from now on\n", why);
  thneed_state = -1;
  thneed_drop();
  thneed_capture_end();
}

static void thneed_report_mismatch(const char *which, bool ran, const ThneedCompare &c, size_t n) {
  if (!ran) {
    fprintf(stderr, "thneed selftest FAILED on the %s input: the replay did not run - plain SNPE\n", which);
    return;
  }
  fprintf(stderr, "thneed selftest FAILED on the %s input: %zu of %zu outputs differ, first at index %zu (got %.9g, want %.9g), "
          "largest absolute difference %g, largest relative difference %g, %zu non-finite mismatches (%zu with NaN) - plain SNPE\n",
          which, c.n_diff, n, c.first, c.first_got, c.first_want, c.max_abs, c.max_rel, c.n_nonfinite, c.n_nan);
}

// First frame (as 0.8.3's SNPEModel::execute, plus a second check): 1. a normal SNPE run; 2. a RECORDED run of the same
// input with the memory state zeroed; 3. SNPE's own result for a DIFFERENT input (a changed picture, the state run 2
// produced), before thneed maps anything; 4. thneed maps SNPE's buffers and replays BOTH inputs: each must give
// SNPE's result BIT FOR BIT. Check 2 of 4 (the second input) catches a replay that does not follow new inputs (it
// would repeat the recorded picture forever) - the first check alone cannot see that.
// Returns true when this frame's output is in output_buf (the recorded run's), false when execute() must compute it.
bool SNPEModel::thneed_start(float *net_input_buf) {
  thneed_state = -1;                     // until every check below has passed
  if (!thneed_enabled()) {
    fprintf(stderr, "thneed: switched off (/data/thneed_off or THNEED_OFF) - plain SNPE\n");
    thneed_capture_end();
    return false;
  }
  if (!recurrent || !trafficConvention || !desire || !output_buf || input_floats == 0 || output_floats == 0 || recurrent_size == 0) {
    fprintf(stderr, "thneed: this model does not have the 4 inputs thneed expects - plain SNPE\n");
    thneed_capture_end();
    return false;
  }
  if (!inputBuffer->setBufferAddress(net_input_buf)) {
    thneed_give_up("SNPE refused the input buffer");
    return false;
  }
  // 1. the normal run: exactly what plain execute() would do now (and an SNPE error is handled the same way)
  if (!snpe->execute(inputMap, outputMap)) PrintErrorStringAndExit();

  // 2. the recorded run
  thneed = new (std::nothrow) Thneed();
  if (thneed == NULL || thneed->failed) {
    thneed_give_up(thneed == NULL ? "out of memory for the recording" : "could not start a recording");
    return true;                         // this frame: run 1's output (nothing was changed after it)
  }
  memset(recurrent, 0, recurrent_size*sizeof(float));
  const bool ran = snpe->execute(inputMap, outputMap);
  thneed->stop_recording();
  if (!ran || thneed->failed) {
    // run 2's output cannot be trusted: plain SNPE computes this frame again, from the same zeroed state
    thneed_give_up(ran ? "the recording failed" : "SNPE failed while being recorded");
    memset(recurrent, 0, recurrent_size*sizeof(float));
    return false;
  }

  const size_t out_bytes = output_floats*sizeof(float), rec_bytes = recurrent_size*sizeof(float), in_bytes = input_floats*sizeof(float);
  float *golden = (float *)malloc(out_bytes), *golden_b = (float *)malloc(out_bytes);
  float *state_b = (float *)malloc(rec_bytes), *input_b = (float *)malloc(in_bytes);
  bool pass = false;
  int perm[4];
  if (golden == NULL || golden_b == NULL || state_b == NULL || input_b == NULL) {
    fprintf(stderr, "thneed: out of memory for the self-test - plain SNPE\n");
  } else {
    memcpy(golden, output_buf, out_bytes);
    // 3. SNPE's result for a different input, while nothing is mapped and nothing is recorded
    memcpy(input_b, net_input_buf, in_bytes);
    for (size_t i = 0; i < input_floats; i += 61) input_b[i] = 255.f - input_b[i];
    memcpy(state_b, recurrent, rec_bytes);
    const bool b_ran = inputBuffer->setBufferAddress(input_b) && snpe->execute(inputMap, outputMap);
    if (b_ran) memcpy(golden_b, output_buf, out_bytes);
    if (!inputBuffer->setBufferAddress(net_input_buf)) fprintf(stderr, "thneed: SNPE refused the input buffer back (plain SNPE sets it again)\n");

    // 4. map, match, replay both inputs
    if (b_ran) thneed->map_io();
    const std::vector<size_t> theirs = thneed->input_sizes();
    const size_t ours[4] = {in_bytes, rec_bytes, desire_size*sizeof(float), trafficConvention_size*sizeof(float)};
    float *ourp[4] = {net_input_buf, recurrent, desire, trafficConvention};
    int slot = -1;
    if (!b_ran) {
      fprintf(stderr, "thneed: SNPE could not run the self-test input - plain SNPE\n");
    } else if (thneed->failed) {
      fprintf(stderr, "thneed: recording unusable (%s) - plain SNPE\n", thneed->why_failed.c_str());
    } else if (theirs.size() != 4 || !thneed->has_output() || !thneed_match_inputs(theirs.data(), 4, ours, 4, perm)) {
      fprintf(stderr, "thneed: found %zu inputs (want 4), output %s - not this SNPE's layout - plain SNPE\n",
              theirs.size(), thneed->has_output() ? "found" : "NOT found");
      for (size_t i = 0; i < theirs.size(); i++) fprintf(stderr, "  thneed input %zu: %zu bytes\n", i, theirs[i]);
    } else if ((slot = thneed_image_slot(perm, 4)) < 0) {
      fprintf(stderr, "thneed: the camera image has no input slot - plain SNPE\n");
    } else {
      for (int i = 0; i < 4; i++) { thneed_in[i] = ourp[perm[i]]; thneed_in_bytes[i] = ours[perm[i]]; }
      thneed_image = slot;
      // replay A: the recorded input, zeroed state (the output buffer is cleared first: nothing can pass by being left over)
      memset(output_buf, 0, out_bytes);
      memset(recurrent, 0, rec_bytes);
      const bool ran_a = thneed->execute(thneed_in, thneed_in_bytes, output_buf, out_bytes);
      const ThneedCompare ca = thneed_compare_outputs(output_buf, golden, output_floats);
      if (!ran_a || ca.n_diff != 0) {
        thneed_report_mismatch("recorded", ran_a, ca, output_floats);
      } else {
        // replay B: the changed picture and run 2's state, against SNPE's own result for them
        memset(output_buf, 0, out_bytes);
        memcpy(recurrent, state_b, rec_bytes);
        thneed_in[slot] = input_b;
        const bool ran_b = thneed->execute(thneed_in, thneed_in_bytes, output_buf, out_bytes);
        thneed_in[slot] = net_input_buf;
        const ThneedCompare cb = thneed_compare_outputs(output_buf, golden_b, output_floats);
        if (!ran_b || cb.n_diff != 0) {
          thneed_report_mismatch("second (changed)", ran_b, cb, output_floats);
        } else {
          fprintf(stderr, "thneed selftest passed (%zu GPU commands replayed, 2 inputs identical to SNPE bit for bit)\n", thneed->cmds.size());
          pass = true;
        }
      }
    }
    // this frame's output: run 2's, with run 2's state (as before the self-test)
    memset(recurrent, 0, rec_bytes);
    memcpy(output_buf, golden, out_bytes);
  }
  free(golden); free(golden_b); free(state_b); free(input_b);
  if (pass) {
    thneed_state = 1;
  } else {
    thneed_give_up(NULL);                // (the reason is printed above) - SNPE gets its buffers back unmapped
  }
  return true;
}
#endif

void SNPEModel::execute(float *net_input_buf, int buf_size) {
#ifdef USE_THNEED
  if (thneed_state >= 0 && runtime_used == zdl::DlSystem::Runtime_t::GPU) {
    if (net_input_buf == NULL || buf_size <= 0 || (size_t)buf_size != input_floats) {
      // buf_size counts floats (every caller); thneed copies input_floats of them, so it never runs on anything else
      fprintf(stderr, "thneed: model input %s (%d floats given, the model takes %zu)\n", net_input_buf ? "has the wrong size" : "is missing",
              buf_size, input_floats);
      thneed_give_up("unexpected model input");
    } else if (thneed_state == 0) {
      if (thneed_start(net_input_buf)) return;          // this frame was computed while recording / testing
    } else if (thneed == NULL || thneed_image < 0 || thneed_image > 3) {
      thneed_give_up("replay state lost");
    } else {
      thneed_in[thneed_image] = net_input_buf;
      if (thneed->execute(thneed_in, thneed_in_bytes, output_buf, output_floats*sizeof(float))) return;
      thneed_give_up("replay failed");                  // the output was not touched: plain SNPE computes this frame below
    }
  }
#endif
  assert(inputBuffer->setBufferAddress(net_input_buf));
  if (!snpe->execute(inputMap, outputMap)) {
    PrintErrorStringAndExit();
  }
}
