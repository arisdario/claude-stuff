import json, os, sys, tempfile
from types import SimpleNamespace as N
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import pose_watch as pw


def test_extract_model_and_odometry():
    ev = N(model=N(path=N(poly=[0, 0, 0.01, 0.3], std=1.0),
                   leftLane=N(poly=[0, 0, 0, 1.6], prob=0.7),
                   rightLane=N(poly=[0, 0, 0, -1.6], prob=0.2),
                   frameId=5, frameDropPerc=0.0, timestampEof=1, modelExecutionTime=0.04))
    d = pw.extract('model', ev)
    assert d['path'][3] == 0.3 and d['fid'] == 5 and d['lane_l_p'] == 0.7
    d = pw.extract('cameraOdometry', N(cameraOdometry=N(trans=[1, 0, 0], rot=[0, 0.01, 0.02],
                                                      transStd=[1, 1, 1], rotStd=[1, 1, 1], frameId=5)))
    assert d['rot'][2] == 0.02


def test_missing_fields_do_not_crash():
    assert pw.extract('controlsState', N(controlsState=N()))['en'] is None
    assert pw.extract('carState', N(carState=N(vEgo=3.0)))['v'] == 3.0
    assert pw.extract('nope', N()) is None


def test_summary_runs():
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'x.jsonl')
    with open(p, 'w') as f:
        for i in range(200):
            f.write(json.dumps({'s': 'model', 't': i * .05, 'lt': i * .05, 'fid': i, 'drop': 0,
                                'path': [0, 0, 0, 0.1 * (i % 7 == 0)]}) + '\n')
            f.write(json.dumps({'s': 'controlsState', 't': i * .05, 'des': 1, 'act': 0.5, 'en': True}) + '\n')
    pw.summarize(p)


def test_read_only():
    src = open(pw.__file__).read()
    for bad in ('pub_sock', 'PubMaster', 'put_nonblocking', 'Params(', 'import carcontroller', 'import controlsd'):
        assert bad not in src, bad
