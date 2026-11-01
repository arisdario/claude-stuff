#!/usr/bin/env python3
"""
radarassist.py - AI assistant for radarweb (/assist): explains the car's data (live state, interventions, trips,
read-only diagnostics, CAN signals) and answers short spoken questions while driving.

ADVISORY ONLY. The assistant can only READ what radarweb already shows. Its tools return data; none of them sends
anything on CAN, starts a UDS read, changes a setting or touches openpilot. It cannot drive, brake or steer.

Off until at least one API key file exists on the EON (never put a key in the repo):
  /data/gemini_api_key       Gemini   (aistudio.google.com -> Get API key)     free tier
  /data/groq_api_key         Groq     (console.groq.com -> API Keys)           free tier, very fast
  /data/anthropic_api_key    Claude   (console.anthropic.com -> API keys)      paid per use
  /data/openai_api_key       ChatGPT  (platform.openai.com -> API keys)        paid per use
  /data/xai_api_key          Grok     (console.x.ai -> API keys)               paid per use
Any one is enough. "Auto" asks them in the order below; when one is rate-limited, overloaded or unreachable the
next one answers. App subscriptions (Claude Pro, ChatGPT Plus, SuperGrok...) do not include API use.
Optional settings: /data/ai_assist.json, e.g.
  {"order": ["groq", "gemini", "claude"], "models": {"gemini": "gemini-2.5-flash"}, "reasoning_effort": {"gemini": "low"}}
A model of "auto" (the default for Gemini, Groq and Grok) is picked from the provider's own model list once a day.
GPS positions are removed from everything sent, unless "include_gps": true is set there.

Plain urllib on purpose: the official SDKs need Python >= 3.10 and compiled packages that do not build on the EON.
"""
import json
import os
import re
import ssl
import time
import urllib.error
import urllib.request

# provider -> how to reach it. 'oai' = OpenAI-style chat completions (OpenAI, and the compatible endpoints of the others)
PROVIDERS = {
    'gemini': {'label': 'Gemini', 'kind': 'oai', 'base': 'https://generativelanguage.googleapis.com/v1beta/openai',
               'model': 'auto', 'fallback_model': 'gemini-2.5-flash', 'max_field': 'max_tokens'},
    'groq':   {'label': 'Groq', 'kind': 'oai', 'base': 'https://api.groq.com/openai/v1',
               'model': 'auto', 'fallback_model': 'llama-3.3-70b-versatile', 'max_field': 'max_tokens'},
    'claude': {'label': 'Claude', 'kind': 'claude', 'model': 'claude-opus-5-5'},
    'openai': {'label': 'ChatGPT', 'kind': 'oai', 'base': 'https://api.openai.com/v1',
               'model': 'gpt-5', 'max_field': 'max_completion_tokens'},
    'grok':   {'label': 'Grok', 'kind': 'oai', 'base': 'https://api.x.ai/v1',
               'model': 'auto', 'fallback_model': 'grok-4', 'max_field': 'max_tokens'},
}
ORDER = ['gemini', 'groq', 'claude', 'openai', 'grok']      # free ones first
KEY_FILES = {'gemini': '/data/gemini_api_key', 'groq': '/data/groq_api_key', 'claude': '/data/anthropic_api_key',
             'openai': '/data/openai_api_key', 'grok': '/data/xai_api_key'}
CONFIG_FILE = '/data/ai_assist.json'
DEFAULTS = {'order': ORDER, 'models': {}, 'reasoning_effort': {}, 'include_gps': False, 'timeout': 90}
MODEL_TTL = 24 * 3600
_AUTO = {}                  # provider -> (time, model) picked from its model list

CLAUDE_URL = 'https://api.anthropic.com/v1/messages'
CLAUDE_VERSION = '2026-10-21'
CLAUDE_BETA = 'server-side-fallback-2026-07-01'     # for "fallbacks": "default" (a declined request is re-run server-side)
OPENAI_URL = 'https://api.openai.com/v1/chat/completions'

MAX_TOOL_ROUNDS = 6
MAX_TOOL_CHARS = 24000
MAX_HISTORY = 20            # text turns kept from the page
RETRY_STATUS = (408, 409, 429, 500, 502, 503, 504, 529)
USER_AGENT = 'radarweb-assistant/1.0'   # Python's default "Python-urllib/x" is blocked (403) by some API front ends (Cloudflare)

GPS_KEYS = {'gps', 'lat', 'lon', 'lng', 'latitude', 'longitude', 'hdg', 'heading', 'MCU_latitude', 'MCU_longitude',
            'MCU_gpsAccuracy', 'MCU_elevation', 'gps_acc'}

SYSTEM_BASE = """You are the assistant inside radarweb, a page that runs on the owner's EON (a phone running openpilot \
0.7.5 Tinkla) in their Tesla Model S. The car is a pre-Autopilot Model S with a retrofitted Bosch radar; a black panda \
connects the EON to the car's CAN buses. The EON runs a 0.8.3 driving model through an adapter, and the PCC module \
(pedal cruise control) handles speed. openpilot steers by angle and can only slow the car with regenerative braking.

Units: speeds in the data are m/s unless a field says km/h; distances are metres; for radar objects d is metres ahead, \
y is metres to the LEFT (positive left), vr is relative speed (negative = closing). 'op' means openpilot engaged, \
'acc' means cruise is active, 'alc' means auto lane change. Events (interventions) are recorded moments: the driver \
took over, braked, steered against openpilot, or a close pass.

Facts about this car's CAN data that you cannot see from the names:
- Every DAS_* message (DAS_status, DAS_bodyControls 0x3E9, DAS_control...) and IDs 0x659/0x65A are produced by the black panda / EON pretending to be the Autopilot computer. They show what openpilot ASKS the car to do, never what the car actually does. DAS_headlightRequest means "Autopilot computer asks for headlights on", not high beam.
- GTW_carState BC_headLightLStatus/RStatus = headlights on (any beam), not high beam.
- There is no reliable high-beam state on the bus: BODY_R1 HiBm_On and LoBm_On_Rq stay 0 on this car. Whether the lights obey a DAS high/low beam decision can only be seen by looking at the lights.
- STW_ACTN_RQ HiBmLvr_Stat is the stalk: 0 idle, 1 high beam pushed, 2 flash pulled.
- MCU_chassisControl MCU_ahlbEnable is always 0 (no Autopilot menu on this pre-AP car).
If unsure whether a signal is a command or a measurement, say so.

What you can do: read data through your tools (live state, events, trips, diagnostics that were already read, CAN \
signals). Use them instead of guessing, and say when data is missing or old. You cannot control the car, change \
settings, start diagnostic reads, or send anything to the car.

Rules:
- Safety first. Never tell the driver to look at the screen, type, or take hands off the wheel while moving. If \
something looks unsafe (faults, warnings, odd radar), say so plainly and suggest pulling over or disengaging.
- Diagnostics are read-only by the owner's choice: never suggest security access, unlocking modules, writing \
data, clearing fault codes, routines, resets or firmware changes.
- Be honest about uncertainty. Explain what the numbers mean in plain words."""

MODE_TEXT = {
    'explain': """Mode: EXPLAIN (the car is parked or a passenger is reading). Answer clearly and completely but \
without padding. Short paragraphs or short lists; show the numbers you used.""",
    'drive': """Mode: DRIVING CO-PILOT. Your answer is spoken aloud to the driver. Answer in one or two short \
sentences, plain speech, no lists, no markdown, no symbols, round numbers. If the answer needs more, give the key \
point and say to ask again when parked.""",
}

TOOLS = [
    {'name': 'get_live_state',
     'description': "Current snapshot of the car as radarweb sees it right now: speed, gear, openpilot/cruise state, "
                    "alert text, lead car, nearest radar objects, lane probabilities, planner target, battery, power, "
                    "speed limit, driver inputs. Use for any 'now' question.",
     'input_schema': {'type': 'object', 'properties': {}, 'required': []}},
    {'name': 'list_events',
     'description': "Recorded interventions and close passes, newest first, without their sample data: time, kind, "
                    "reason, speed and summary fields. Use to answer 'what happened', 'why did it disengage'.",
     'input_schema': {'type': 'object', 'properties': {
         'limit': {'type': 'integer', 'description': 'how many events (1-50, default 15)'}}, 'required': []}},
    {'name': 'get_event',
     'description': "One recorded event in detail, including its time series around the moment (downsampled). "
                    "Pass the 'file' value from list_events.",
     'input_schema': {'type': 'object', 'properties': {
         'file': {'type': 'string', 'description': "event file name, like ev_20261101_093011_takeover.json"}},
         'required': ['file']}},
    {'name': 'list_trips',
     'description': "Drive logs on the EON, newest first (file names contain the start date and time).",
     'input_schema': {'type': 'object', 'properties': {}, 'required': []}},
    {'name': 'get_trip_summary',
     'description': "Statistics for one drive log: duration, distance, speeds, time above the speed limit, share of "
                    "time with cruise / openpilot, stops, radar objects seen by kind. No positions.",
     'input_schema': {'type': 'object', 'properties': {
         'name': {'type': 'string', 'description': "log name from list_trips, like map_20261101_093011.jsonl"}},
         'required': ['name']}},
    {'name': 'get_diagnostics',
     'description': "Results of the read-only module diagnostics the owner already ran on the CAN data page: module "
                    "identification and stored fault codes (DTCs) with their decoded meaning. Does not start a read.",
     'input_schema': {'type': 'object', 'properties': {}, 'required': []}},
    {'name': 'find_can_signals',
     'description': "Search the decoded CAN signals currently on the bus by name (case-insensitive text match on the "
                    "message or signal name, e.g. 'BMS', 'tire', 'DI_torque', 'temp'). Returns current values.",
     'input_schema': {'type': 'object', 'properties': {
         'query': {'type': 'string', 'description': 'part of a message or signal name'}}, 'required': ['query']}},
]


# ---------------------------------------------------------------- settings and keys

def config():
    c = dict(DEFAULTS)
    try:
        with open(CONFIG_FILE) as f:
            u = json.load(f)
        if isinstance(u, dict):
            c.update(dict((k, v) for k, v in u.items() if k in DEFAULTS))
    except Exception:
        pass
    if not isinstance(c['order'], list): c['order'] = ORDER
    for k in ('models', 'reasoning_effort'):
        if not isinstance(c[k], dict): c[k] = {}
    c['order'] = [p for p in c['order'] if p in PROVIDERS] + [p for p in ORDER if p not in c['order']]
    return c


def model_setting(p, cfg):
    return cfg['models'].get(p) or PROVIDERS[p]['model']


def read_key(provider):
    try:
        with open(KEY_FILES[provider]) as f:
            k = f.read().strip()
        return k or None
    except Exception:
        return None


def providers(cfg=None):
    """usable providers, preferred first"""
    cfg = cfg or config()
    return [p for p in cfg['order'] if read_key(p)]


def status():
    cfg = config(); have = providers(cfg)
    return {'enabled': bool(have), 'providers': have,
            'labels': dict((p, PROVIDERS[p]['label']) for p in PROVIDERS),
            'models': dict((p, (_AUTO[p][1] if model_setting(p, cfg) == 'auto' and p in _AUTO else model_setting(p, cfg))) for p in PROVIDERS),
            'include_gps': bool(cfg['include_gps']), 'key_files': KEY_FILES}


# ---------------------------------------------------------------- data shaping (what the model gets to see)

def scrub(x, gps=False, depth=0):
    """copy of x without GPS fields, long lists cut, deep nesting flattened to a note"""
    if depth > 6:
        return '...'
    if isinstance(x, dict):
        return dict((k, scrub(v, gps, depth + 1)) for k, v in x.items() if gps or k not in GPS_KEYS)
    if isinstance(x, (list, tuple)):
        y = [scrub(v, gps, depth + 1) for v in x[:40]]
        if len(x) > 40:
            y.append('... %d more' % (len(x) - 40))
        return y
    if isinstance(x, float):
        return round(x, 3)
    return x


def clip(s, n=MAX_TOOL_CHARS):
    return s if len(s) <= n else s[:n] + '\n... (cut, %d characters more)' % (len(s) - n)


def shape_state(st, gps=False):
    st = dict(st or {})
    objs = [o for o in (st.pop('o', None) or []) if isinstance(o, dict)]
    objs.sort(key=lambda o: abs(o.get('d') or 999))
    st['radar_objects'] = {'count': len(objs), 'nearest': objs[:8]}
    m = st.get('model')
    if isinstance(m, dict):       # polynomial coefficients mean nothing to a reader: keep the lane probabilities
        st['model'] = {'left_lane_prob': m.get('pl'), 'right_lane_prob': m.get('pr'), 'path_seen': m.get('p') is not None}
    st['age_s'] = round(time.time() - st['t'], 1) if isinstance(st.get('t'), (int, float)) and st.get('t') else None
    return scrub(st, gps)


def shape_event(ev, gps=False, max_samples=60):
    if not isinstance(ev, dict):
        return ev
    ev = dict(ev)
    s = ev.get('samples')
    if isinstance(s, list) and len(s) > max_samples:
        step = (len(s) + max_samples - 1) // max_samples
        ev['samples'] = s[::step]
        ev['samples_note'] = 'every %d. sample of %d' % (step, len(s))
    return scrub(ev, gps)


def trip_summary(t):
    """statistics from a radartrip result, without positions"""
    if not isinstance(t, dict) or t.get('error') or t.get('pending'):
        return t
    cols = t.get('cols', {}).get('points', [])
    ix = dict((c, i) for i, c in enumerate(cols))
    pts = t.get('points') or []
    dist = dur = over = acc_s = op_s = 0.0; vmax = 0.0; prev = None
    for p in pts:
        try:
            tt, kmh = p[ix['t']], p[ix['kmh']] or 0
            if prev is not None:
                dt = min(max(tt - prev[0], 0), 5.0)          # gaps (no fix) do not count as driving
                dur += dt; dist += kmh / 3.6 * dt
                lim = p[ix['limit']] if 'limit' in ix else None
                if lim and kmh > lim + 2: over += dt
                if 'acc' in ix and p[ix['acc']]: acc_s += dt
                if 'op' in ix and p[ix['op']]: op_s += dt
            vmax = max(vmax, kmh); prev = (tt, kmh)
        except Exception:
            continue
    kinds = {}
    oc = t.get('cols', {}).get('objects', [])
    if 'kind' in oc:
        k = oc.index('kind')
        for o in t.get('objects') or []:
            try: kinds[o[k]] = kinds.get(o[k], 0) + 1
            except Exception: pass
    stops = t.get('stops') or []
    sc = t.get('cols', {}).get('stops', [])
    stop_s = [s[sc.index('seconds')] for s in stops if 'seconds' in sc]
    pct = (lambda a: round(100.0 * a / dur) if dur else None)
    return {'file': t.get('file'), 'gps_points': len(pts), 'driving_minutes': round(dur / 60, 1),
            'distance_km': round(dist / 1000, 2), 'max_kmh': round(vmax), 'avg_kmh': round(dist / dur * 3.6) if dur else None,
            'pct_time_over_limit': pct(over), 'pct_time_cruise': pct(acc_s), 'pct_time_openpilot': pct(op_s),
            'stops': len(stops), 'stop_seconds': stop_s[:30], 'radar_objects_marked_by_kind': kinds}


def find_signals(msgs, query, limit=40):
    q = (query or '').strip().lower(); out = []
    if not q:
        return {'error': 'empty query'}
    for key, m in sorted((msgs or {}).items()):
        name = str(m.get('name', key)); hit_msg = q in name.lower()
        for sig, val in (m.get('sig') or {}).items():
            if hit_msg or q in sig.lower():
                out.append({'msg': name, 'id': '0x%X' % m.get('addr', 0), 'signal': sig, 'value': val})
                if len(out) >= limit:
                    return {'matches': out, 'note': 'first %d matches - search more precisely for others' % limit}
    return {'matches': out}


class DataTools(object):
    """the tools, over functions radarweb hands in (so this file needs nothing from radarweb to be tested)"""
    def __init__(self, state=None, events=None, event=None, trips=None, trip=None, diagnostics=None, can=None):
        self.f = {'state': state, 'events': events, 'event': event, 'trips': trips, 'trip': trip,
                  'diagnostics': diagnostics, 'can': can}

    def _need(self, k):
        if self.f.get(k) is None:
            raise RuntimeError('not available on this EON')
        return self.f[k]

    def run(self, name, args, gps=False):
        """tool result as text; errors come back as text too, so the model can say what failed"""
        args = args if isinstance(args, dict) else {}
        try:
            if name == 'get_live_state':
                r = shape_state(self._need('state')(), gps)
            elif name == 'list_events':
                n = max(1, min(int(args.get('limit') or 15), 50))
                r = [scrub(e, gps) for e in (self._need('events')() or [])[:n]]
                if not r: r = {'events': [], 'note': 'no events recorded'}
            elif name == 'get_event':
                f = str(args.get('file') or '')
                if not (f.startswith('ev_') and f.endswith('.json')) or '/' in f or '..' in f:
                    return 'error: bad event file name'
                r = shape_event(self._need('event')(f), gps) or {'error': 'no such event'}
            elif name == 'list_trips':
                r = self._need('trips')()
                if isinstance(r, dict) and isinstance(r.get('files'), list): r['files'] = r['files'][:40]
            elif name == 'get_trip_summary':
                r = self._need('trip')(str(args.get('name') or ''))
                if isinstance(r, dict) and r.get('pending'):
                    r = {'pending': True, 'note': 'the EON is processing this log; ask again in about 30 seconds'}
                else:
                    r = trip_summary(r)
            elif name == 'get_diagnostics':
                r = scrub(self._need('diagnostics')(), gps)
            elif name == 'find_can_signals':
                r = scrub(find_signals(self._need('can')(), args.get('query')), gps)
            else:
                return 'error: unknown tool %s' % name
            return clip(json.dumps(r, default=str, separators=(',', ':')))
        except Exception as e:
            return 'error: %s' % e


# ---------------------------------------------------------------- HTTP

class ApiError(Exception):
    def __init__(self, provider, status, msg):
        Exception.__init__(self, '%s: %s %s' % (provider, status, msg))
        self.provider, self.status, self.msg = provider, status, msg

    @property
    def retry_other(self):        # any failure of one service (busy, down, bad key, blocked region): try the next one
        return True


def _http_error(provider, e):
    try:
        raw = e.read().decode('utf8', 'replace')
    except Exception:
        raw = ''
    try:
        d = json.loads(raw); err = d.get('error')
        m = (err.get('message') if isinstance(err, dict) else err) or str(d)
    except Exception:
        import re as _re
        t = _re.sub(r'<[^>]+>', ' ', raw); t = ' '.join(t.split())[:200]       # e.g. a Cloudflare block page
        m = '%s%s' % (e.reason, (' - ' + t) if t else '')
    return ApiError(provider, e.code, m)


def _ssl_ctx():
    try:
        import certifi              # openpilot ships it (requests depends on it); the phone's own CA store is old
        return ssl.create_default_context(cafile=certifi.where())
    except Exception:
        return ssl.create_default_context()


def post_json(url, headers, body, provider, timeout=90, opener=None):
    req = urllib.request.Request(url, data=json.dumps(body).encode('utf8'), method='POST',
                                 headers=dict(headers, **{'content-type': 'application/json', 'user-agent': USER_AGENT}))
    try:
        if opener is not None:
            raw = opener(req, timeout)
        else:
            with urllib.request.urlopen(req, timeout=timeout, context=_ssl_ctx()) as r:
                raw = r.read()
        return json.loads(raw.decode('utf8'))
    except urllib.error.HTTPError as e:
        raise _http_error(provider, e)
    except urllib.error.URLError as e:
        raise ApiError(provider, None, 'cannot reach the API (%s) - is the EON online?' % e.reason)
    except (OSError, ValueError) as e:
        raise ApiError(provider, None, str(e))


def get_json(url, headers, provider, timeout=20, opener=None):
    req = urllib.request.Request(url, headers=dict(headers, **{'user-agent': USER_AGENT}))
    try:
        if opener is not None:
            raw = opener(req, timeout)
        else:
            with urllib.request.urlopen(req, timeout=timeout, context=_ssl_ctx()) as r:
                raw = r.read()
        return json.loads(raw.decode('utf8'))
    except urllib.error.HTTPError as e:
        raise _http_error(provider, e)
    except (urllib.error.URLError, OSError, ValueError) as e:
        raise ApiError(provider, None, str(e))


# ---------------------------------------------------------------- Claude (Messages API)

def claude_body(cfg, mode, history):
    drive = mode == 'drive'
    return {
        'model': model_setting('claude', cfg),
        'max_tokens': 2000 if drive else 8000,
        'output_config': {'effort': 'low' if drive else 'medium'},   # thinking is always on; effort sets its depth
        'fallbacks': 'default',
        'system': [{'type': 'text', 'text': SYSTEM_BASE},
                   {'type': 'text', 'text': MODE_TEXT[mode]}],
        'tools': TOOLS,
        'tool_choice': {'type': 'auto'},
        'cache_control': {'type': 'ephemeral'},        # caches tools + system + earlier turns automatically
        'messages': [{'role': m['role'], 'content': m['text']} for m in history],
    }


def ask_claude(p, key, cfg, mode, history, tools, opener=None):
    headers = {'x-api-key': key, 'anthropic-version': CLAUDE_VERSION, 'anthropic-beta': CLAUDE_BETA}
    body = claude_body(cfg, mode, history); used = []
    for _ in range(MAX_TOOL_ROUNDS + 1):
        r = post_json(CLAUDE_URL, headers, body, 'claude', cfg['timeout'], opener)
        sr = r.get('stop_reason'); content = r.get('content') or []
        if sr == 'refusal':
            return {'text': "I can't help with that one.", 'stop': sr, 'tools': used, 'model': r.get('model')}
        calls = [b for b in content if b.get('type') == 'tool_use']
        if sr == 'tool_use' and calls and len(used) < MAX_TOOL_ROUNDS * 4:
            body['messages'].append({'role': 'assistant', 'content': content})   # unchanged, thinking blocks included
            res = []
            for b in calls:
                used.append(b.get('name'))
                res.append({'type': 'tool_result', 'tool_use_id': b.get('id'),
                            'content': tools.run(b.get('name'), b.get('input'), cfg['include_gps'])})
            body['messages'].append({'role': 'user', 'content': res})
            continue
        text = '\n'.join(b.get('text', '') for b in content if b.get('type') == 'text').strip()
        if sr == 'max_tokens':
            text = (text + '\n(answer cut short)').strip()
        return {'text': text or '(no answer)', 'stop': sr, 'tools': used, 'model': r.get('model')}
    return {'text': 'Stopped: too many data lookups for one question.', 'stop': 'tool_rounds', 'tools': used}


# ---------------------------------------------------------------- OpenAI-style chat completions (ChatGPT, Gemini, Groq, Grok)

def _ver(name):
    return tuple(int(x) for x in re.findall(r'\d+', name)[:3])


def pick_model(p, ids):
    """a sensible chat model with tool use from a provider's model list (None if nothing fits)"""
    ids = [i[7:] if i.startswith('models/') else i for i in ids if isinstance(i, str)]
    bad = re.compile(r'image|imagine|vision|video|tts|audio|live|embed|whisper|guard|transcribe|search|aqa|robotics|computer|native')
    ids = [i for i in ids if not bad.search(i)]
    if p == 'gemini':
        plain = [i for i in ids if re.match(r'^gemini-\d+(\.\d+)?-flash$', i)]                 # stable Flash: free tier, fast
        if plain: return max(plain, key=_ver)
        flash = [i for i in ids if i.startswith('gemini-') and 'flash' in i and 'lite' not in i]
        return max(flash, key=lambda i: (_ver(i), 'preview' not in i and 'exp' not in i)) if flash else None
    if p == 'groq':
        for want in ('openai/gpt-oss-120b', 'llama-3.3-70b-versatile', 'openai/gpt-oss-20b'):
            if want in ids: return want
        big = [i for i in ids if 'gpt-oss' in i or '70b' in i]
        return big[0] if big else None
    if p == 'grok':
        g = [i for i in ids if i.startswith('grok-') and 'code' not in i and 'mini' not in i]
        if not g: return None
        top = max(_ver(i)[:2] for i in g)                          # newest generation, then the fast variant
        g = [i for i in g if _ver(i)[:2] == top]
        return sorted(g, key=lambda i: ('fast' not in i, 'non-reasoning' not in i, len(i)))[0]
    return None


def resolve_model(p, key, cfg, opener=None):
    m = model_setting(p, cfg)
    if m != 'auto':
        return m
    hit = _AUTO.get(p)
    if hit and time.time() - hit[0] < MODEL_TTL:
        return hit[1]
    try:
        r = get_json(PROVIDERS[p]['base'] + '/models', {'authorization': 'Bearer ' + key}, p, 20, opener)
        m = pick_model(p, [d.get('id') for d in (r.get('data') or []) if isinstance(d, dict)])
    except ApiError as e:
        if e.status in (401, 403):
            raise
        m = None
    m = m or PROVIDERS[p]['fallback_model']
    _AUTO[p] = (time.time(), m)
    return m


def openai_tools():
    return [{'type': 'function', 'function': {'name': t['name'], 'description': t['description'],
                                              'parameters': t['input_schema']}} for t in TOOLS]


def openai_body(p, model, cfg, mode, history):
    drive = mode == 'drive'
    b = {'model': model,
         PROVIDERS[p]['max_field']: 2000 if drive else 8000,
         'messages': [{'role': 'system', 'content': SYSTEM_BASE + '\n\n' + MODE_TEXT[mode]}] +
                     [{'role': m['role'], 'content': m['text']} for m in history],
         'tools': openai_tools(), 'tool_choice': 'auto'}
    eff = cfg['reasoning_effort'].get(p)
    if eff:                                       # only for reasoning models; others reject the field
        b['reasoning_effort'] = eff
    return b


def _echo(msg):
    """the assistant turn to send back: only the fields every compatible API accepts (Gemini's extra_content
    carries its thought signatures and must come back)"""
    m = {'role': 'assistant', 'content': msg.get('content')}
    for k in ('tool_calls', 'extra_content'):
        if msg.get(k): m[k] = msg[k]
    return m


def ask_oai(p, key, cfg, mode, history, tools, opener=None):
    headers = {'authorization': 'Bearer ' + key}
    model = resolve_model(p, key, cfg, opener)
    body = openai_body(p, model, cfg, mode, history); used = []
    url = PROVIDERS[p]['base'] + '/chat/completions'
    for _ in range(MAX_TOOL_ROUNDS + 1):
        try:
            r = post_json(url, headers, body, p, cfg['timeout'], opener)
        except ApiError as e:
            if e.status == 404 and model_setting(p, cfg) == 'auto':
                _AUTO.pop(p, None)                # picked model gone: pick again next time
            raise
        ch = (r.get('choices') or [{}])[0]; msg = ch.get('message') or {}; fr = ch.get('finish_reason')
        calls = msg.get('tool_calls') or []
        if calls and len(used) < MAX_TOOL_ROUNDS * 4:
            body['messages'].append(_echo(msg))
            for i, c in enumerate(calls):
                fn = c.get('function') or {}; used.append(fn.get('name'))
                try: args = json.loads(fn.get('arguments') or '{}')
                except ValueError: args = None
                out = tools.run(fn.get('name'), args, cfg['include_gps']) if isinstance(args, dict) else 'error: arguments were not valid JSON'
                body['messages'].append({'role': 'tool', 'tool_call_id': c.get('id') or 'call_%d' % i, 'content': out})
            continue
        text = (msg.get('content') or msg.get('refusal') or '').strip()
        if fr == 'length':
            text = (text + '\n(answer cut short)').strip()
        return {'text': text or '(no answer)', 'stop': fr, 'tools': used, 'model': r.get('model') or model}
    return {'text': 'Stopped: too many data lookups for one question.', 'stop': 'tool_rounds', 'tools': used}


def ask_provider(p, key, cfg, mode, history, tools, opener=None):
    f = ask_claude if PROVIDERS[p]['kind'] == 'claude' else ask_oai
    return f(p, key, cfg, mode, history, tools, opener)


def clean_history(history):
    """[{role, text}] from the page: alternating, starting and ending with the user, trimmed"""
    out = []
    for m in (history or [])[-MAX_HISTORY:]:
        if not isinstance(m, dict) or m.get('role') not in ('user', 'assistant'):
            continue
        t = str(m.get('text') or '').strip()[:4000]
        if not t:
            continue
        if out and out[-1]['role'] == m['role']:
            out[-1]['text'] += '\n' + t
        else:
            out.append({'role': m['role'], 'text': t})
    while out and out[0]['role'] != 'user':
        out.pop(0)
    if not out or out[-1]['role'] != 'user':
        return None
    return out


def ask(history, mode='explain', tools=None, provider=None, opener=None):
    """answer the last user turn. provider None = preferred one, then the other if the first is busy or down"""
    if mode not in MODE_TEXT:
        mode = 'explain'
    hist = clean_history(history)
    if hist is None:
        return {'ok': False, 'error': 'nothing to answer'}
    cfg = config(); order = providers(cfg)
    if not order:
        return {'ok': False, 'error': 'off: no API key. Put one in %s on the EON.' % ' or '.join(KEY_FILES[p] for p in ORDER)}
    if provider in order:
        order.remove(provider); order.insert(0, provider)
    tools = tools or DataTools(); errors = []; fails = []; t0 = time.time()
    for i, p in enumerate(order):
        try:
            r = ask_provider(p, read_key(p), cfg, mode, hist, tools, opener)
            r.update({'ok': True, 'provider': p, 'seconds': round(time.time() - t0, 1)})
            if errors:
                r['note'] = 'answered by %s because %s' % (PROVIDERS[p]['label'], errors[-1])
            return r
        except ApiError as e:
            errors.append('%s was unavailable (%s)' % (PROVIDERS[p]['label'], e.status or 'network'))
            fails.append('%s: %s %s' % (PROVIDERS[p]['label'], e.status or '', e.msg))
            if not e.retry_other or i == len(order) - 1:
                return {'ok': False, 'error': ' | '.join(fails), 'provider': p}
    return {'ok': False, 'error': ' | '.join(fails)}
