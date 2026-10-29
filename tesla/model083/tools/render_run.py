# PC only (needs numpy, onnxruntime, pillow): python3 render_run.py [half_lane_width_m] [shift_m: + = car LEFT of centre] [path/to/supercombo.onnx]
# then: g++ -std=c++11 -O2 -o decode decode.cc && ./decode   (prints what the adapter publishes for that frame)
# Render a straight road (lane lines at +-half_w m) in the 0.8.3 model's camera frame, pack it like loadyuv.cl,
# run the real 0.8.3 supercombo.onnx for N frames (feeding the recurrent state back), dump the last raw output.
import numpy as np, onnxruntime as ort, sys
half_w = float(sys.argv[1]) if len(sys.argv) > 1 else 1.8
shift = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0          # + = car LEFT of the lane centre (m)
W, H, FL, CX, CY, CAM_H = 512, 256, 910.0, 256.0, 47.6, 1.22
v, u = np.mgrid[0:H, 0:W].astype(np.float64)
Y = np.full((H, W), 165.0)                                         # sky / distance
ground = v > CY + 0.5
x = FL * CAM_H / np.maximum(v - CY, 1e-6)                          # forward distance of each ground pixel
yl = (CX - u) * x / FL + shift                                     # lateral position, left positive, lane frame
Y[ground] = 85 + 10 * np.random.RandomState(0).rand(*Y.shape)[ground]   # asphalt with a little texture
lw = 0.15
for c in (half_w, -half_w):
  line = ground & (np.abs(yl - c) < lw / 2) & (x < 150)
  Y[line] = 215
Y = Y.astype(np.uint8)
U = np.full((H // 2, W // 2), 128, np.uint8); V = U.copy()
def pack(Y, U, V):
  return np.stack([Y[0::2, 0::2], Y[1::2, 0::2], Y[0::2, 1::2], Y[1::2, 1::2], U, V]).astype(np.float32)
f = pack(Y, U, V)
sess = ort.InferenceSession(sys.argv[3] if len(sys.argv) > 3 else 'supercombo.onnx', providers=['CPUExecutionProvider'])
state = np.zeros((1, 512), np.float32); desire = np.zeros((1, 8), np.float32); tc = np.array([[1, 0]], np.float32)
imgs = np.concatenate([f, f])[None]
for i in range(20):
  out = sess.run(None, {'input_imgs': imgs, 'desire': desire, 'traffic_convention': tc, 'initial_state': state})[0]
  state = out[:, -512:].copy()
out[0].astype(np.float32).tofile('raw_out.bin')
print('ran 20 frames, raw output', out.shape, 'finite:', bool(np.isfinite(out).all()))
from PIL import Image; Image.fromarray(Y).save('road_model_frame.png')
