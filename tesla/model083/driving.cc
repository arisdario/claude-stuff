// Drop-in for openpilot 0.7.5 selfdrive/modeld/models/driving.cc that runs the 0.8.3 driving model.
// Model loading, frame preparation and the SNPE runner are 0.7.5's (the 0.8.3 model takes the same inputs: two
// 512x256 YUV frames, desire, traffic convention, recurrent state; same calibration warp and EON intrinsics).
// The outputs are converted by model083_adapter.h into the 0.7.5 "model" and "cameraOdometry" messages.
// Unlike stock 0.7.5, path/lane points and stds are always published (stock only did that with DEBUG set), so
// PCC_module's camera curve limit gets the path points it reads.
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include "common/timing.h"
#include "common/params.h"
#include "driving.h"
#include "model083_adapter.h"

void model_init(ModelState* s, cl_device_id device_id, cl_context context, int temporal) {
  frame_init(&s->frame, MODEL_WIDTH, MODEL_HEIGHT, device_id, context);
  s->input_frames = (float*)calloc(MODEL_FRAME_SIZE * 2, sizeof(float));

  const int output_size = m083::OUTPUT_SIZE + m083::TEMPORAL_SIZE;
  s->output = (float*)calloc(output_size, sizeof(float));

  s->m = new DefaultRunModel("../../models/supercombo.dlc", s->output, output_size, USE_GPU_RUNTIME);

#ifdef TEMPORAL
  assert(temporal);
  s->m->addRecurrent(&s->output[m083::OUTPUT_SIZE], m083::TEMPORAL_SIZE);
#endif

#ifdef DESIRE
  s->prev_desire = (float*)calloc(DESIRE_LEN, sizeof(float));
  s->pulse_desire = (float*)calloc(DESIRE_LEN, sizeof(float));
  s->m->addDesire(s->pulse_desire, DESIRE_LEN);
#endif

#ifdef TRAFFIC_CONVENTION
  s->traffic_convention = (float*)calloc(TRAFFIC_CONVENTION_LEN, sizeof(float));
  s->m->addTrafficConvention(s->traffic_convention, TRAFFIC_CONVENTION_LEN);

  char *string;
  const int result = read_db_value(NULL, "IsRHD", &string, NULL);
  if (result == 0) {
    bool is_rhd = string[0] == '1';
    free(string);
    s->traffic_convention[is_rhd ? 1 : 0] = 1.0;
  } else {
    s->traffic_convention[0] = 1.0;
  }
#endif
}

ModelDataRaw model_eval_frame(ModelState* s, cl_command_queue q,
                           cl_mem yuv_cl, int width, int height,
                           mat3 transform, void* sock,
                           float *desire_in) {
#ifdef DESIRE
  if (desire_in != NULL) {
    // 0.7.5's modeld sets vec_desire[desire - 1]; the 0.8.3 model is indexed by desire itself
    float d83[DESIRE_LEN];
    m083::desire_to_083(desire_in, d83);
    for (int i = 1; i < DESIRE_LEN; i++) {
      // Model decides when action is completed, so desire input is just a pulse triggered on rising edge
      s->pulse_desire[i] = (d83[i] - s->prev_desire[i] > .99) ? d83[i] : 0.0;
      s->prev_desire[i] = d83[i];
    }
  }
#endif

  float *new_frame_buf = frame_prepare(&s->frame, q, yuv_cl, width, height, transform);
  memmove(&s->input_frames[0], &s->input_frames[MODEL_FRAME_SIZE], sizeof(float)*MODEL_FRAME_SIZE);
  memmove(&s->input_frames[MODEL_FRAME_SIZE], new_frame_buf, sizeof(float)*MODEL_FRAME_SIZE);
  s->m->execute(s->input_frames, MODEL_FRAME_SIZE*2);

  clEnqueueUnmapMemObject(q, s->frame.net_input, (void*)new_frame_buf, 0, NULL, NULL);

  ModelDataRaw net_outputs;
  net_outputs.raw = s->output;
  return net_outputs;
}

void model_free(ModelState* s) {
  free(s->output);
  free(s->input_frames);
  frame_free(&s->frame);
  delete s->m;
}

void poly_fit(float *in_pts, float *in_stds, float *out) {
  m083::poly_fit(in_pts, in_stds, out);
}

static void fill_line(cereal::ModelData::PathData::Builder path, const m083::Line &L) {
  path.setPoints(kj::ArrayPtr<const float>(L.points, m083::PATH_N));
  path.setStds(kj::ArrayPtr<const float>(L.stds, m083::PATH_N));
  path.setPoly(kj::ArrayPtr<const float>(L.poly, 4));
  path.setProb(L.prob);
  path.setStd(L.std);
}

static void fill_lead(cereal::ModelData::LeadData::Builder lead, const m083::Lead &R) {
  lead.setProb(R.prob);
  lead.setDist(R.dist);
  lead.setStd(R.std);
  lead.setRelY(R.rel_y);
  lead.setRelYStd(R.rel_y_std);
  lead.setRelVel(R.rel_vel);
  lead.setRelVelStd(R.rel_vel_std);
  lead.setRelA(R.rel_a);
  lead.setRelAStd(R.rel_a_std);
}

void model_publish(PubSocket *sock, uint32_t frame_id,
                   const ModelDataRaw net_outputs, uint64_t timestamp_eof) {
  const float *o = net_outputs.raw;
  m083::Line p, l, r; m083::Lead ld, lf; m083::Longi g; m083::Meta mt;
  m083::path_from_plan(o, p);
  m083::lane_line(o, m083::LANE_LEFT, l);
  m083::lane_line(o, m083::LANE_RIGHT, r);
  m083::lead(o, 0, ld);
  m083::lead(o, 1, lf);
  m083::longi(o, g);
  m083::meta(o, mt);

  capnp::MallocMessageBuilder msg;
  cereal::Event::Builder event = msg.initRoot<cereal::Event>();
  event.setLogMonoTime(nanos_since_boot());

  auto framed = event.initModel();
  framed.setFrameId(frame_id);
  framed.setTimestampEof(timestamp_eof);

  fill_line(framed.initPath(), p);
  fill_line(framed.initLeftLane(), l);
  fill_line(framed.initRightLane(), r);

  auto longi = framed.initLongitudinal();
  longi.setDistances(kj::ArrayPtr<const float>(g.dist, 10));
  longi.setSpeeds(kj::ArrayPtr<const float>(g.speed, 10));
  longi.setAccelerations(kj::ArrayPtr<const float>(g.accel, 10));

  fill_lead(framed.initLead(), ld);
  fill_lead(framed.initLeadFuture(), lf);

  auto meta = framed.initMeta();
  meta.setDesireState(kj::ArrayPtr<const float>(mt.desire_state, DESIRE_LEN));
  meta.setEngagedProb(mt.engaged);
  meta.setGasDisengageProb(mt.gas);
  meta.setBrakeDisengageProb(mt.brake);
  meta.setSteerOverrideProb(mt.steer);
  meta.setDesirePrediction(kj::ArrayPtr<const float>(mt.desire_pred, m083::DESIRE_PRED_N));

  auto words = capnp::messageToFlatArray(msg);
  auto bytes = words.asBytes();
  sock->send((char*)bytes.begin(), bytes.size());
}

void posenet_publish(PubSocket *sock, uint32_t frame_id,
                   const ModelDataRaw net_outputs, uint64_t timestamp_eof) {
  m083::Pose ps;
  m083::pose(net_outputs.raw, ps);

  capnp::MallocMessageBuilder msg;
  cereal::Event::Builder event = msg.initRoot<cereal::Event>();
  event.setLogMonoTime(nanos_since_boot());

  auto posenetd = event.initCameraOdometry();
  posenetd.setTrans(kj::ArrayPtr<const float>(ps.trans, 3));
  posenetd.setRot(kj::ArrayPtr<const float>(ps.rot, 3));
  posenetd.setTransStd(kj::ArrayPtr<const float>(ps.trans_std, 3));
  posenetd.setRotStd(kj::ArrayPtr<const float>(ps.rot_std, 3));
  posenetd.setTimestampEof(timestamp_eof);
  posenetd.setFrameId(frame_id);

  auto words = capnp::messageToFlatArray(msg);
  auto bytes = words.asBytes();
  sock->send((char*)bytes.begin(), bytes.size());
}
