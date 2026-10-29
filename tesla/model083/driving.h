#ifndef MODEL_H
#define MODEL_H

// Drop-in for openpilot 0.7.5 selfdrive/modeld/models/driving.h that runs the 0.8.3 driving model (supercombo.dlc).
// Same functions and constants modeld.cc uses; only ModelDataRaw now just carries the raw output, which
// model083_adapter.h turns into the 0.7.5 "model" message. See README.md in this folder.

// gate this here
#define TEMPORAL
#define DESIRE
#define TRAFFIC_CONVENTION

#ifdef QCOM
#include <eigen3/Eigen/Dense>     // modeld.cc relies on this include for its calibration math
#else
#include <Eigen/Dense>
#endif

#include "common/mat.h"
#include "common/util.h"

#include "commonmodel.h"
#include "runners/run.h"

#include "cereal/gen/cpp/log.capnp.h"
#include <czmq.h>
#include <capnp/serialize.h>
#include "messaging.hpp"

#define MODEL_WIDTH 512
#define MODEL_HEIGHT 256
#define MODEL_FRAME_SIZE MODEL_WIDTH * MODEL_HEIGHT * 3 / 2
#define MODEL_NAME "supercombo_dlc"

#define MODEL_PATH_DISTANCE 192
#define POLYFIT_DEGREE 4
#define DESIRE_LEN 8
#define TRAFFIC_CONVENTION_LEN 2

struct ModelDataRaw {
  const float *raw;               // the 0.8.3 model output, laid out as in model083_adapter.h
};

typedef struct ModelState {
  ModelFrame frame;
  float *output;
  float *input_frames;
  RunModel *m;
#ifdef DESIRE
  float *prev_desire;
  float *pulse_desire;
#endif
#ifdef TRAFFIC_CONVENTION
  float *traffic_convention;
#endif
} ModelState;

void model_init(ModelState* s, cl_device_id device_id,
                cl_context context, int temporal);
ModelDataRaw model_eval_frame(ModelState* s, cl_command_queue q,
                           cl_mem yuv_cl, int width, int height,
                           mat3 transform, void* sock, float *desire_in);
void model_free(ModelState* s);
void poly_fit(float *in_pts, float *in_stds, float *out);

void model_publish(PubSocket* sock, uint32_t frame_id,
                   const ModelDataRaw data, uint64_t timestamp_eof);
void posenet_publish(PubSocket* sock, uint32_t frame_id,
                   const ModelDataRaw data, uint64_t timestamp_eof);
#endif
