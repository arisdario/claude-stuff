"""radarassist: request shapes, tool loops, provider fall-over, GPS removal. No network: a fake opener answers."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import radarassist as ra  # noqa: E402


class Fake(object):
    """answers requests from a list of (url_part, status, body); records what was sent"""
    def __init__(self, replies):
        self.replies = list(replies); self.sent = []

    def __call__(self, req, timeout):
        body = json.loads(req.data.decode()) if req.data else None
        self.sent.append((req.full_url, dict(req.header_items()), body))
        part, status, reply = self.replies.pop(0)
        assert part in req.full_url, (part, req.full_url)
        if status != 200:
            import io
            import urllib.error
            raise urllib.error.HTTPError(req.full_url, status, 'err', {}, io.BytesIO(json.dumps({'error': {'message': 'busy'}}).encode()))
        return json.dumps(reply).encode()


STATE = {'v': 20.0, 'gear': 'drive', 'op': True, 'gps': {'lat': 1.5, 'lon': 2.5},
         'o': [{'s': 1, 'd': 40.0, 'y': 0.1}, {'s': 2, 'd': 8.0, 'y': -3.0}],
         'model': {'p': [0, 0, 0, 0], 'l': None, 'r': None, 'pl': 0.9, 'pr': 0.8}}


def tools():
    return ra.DataTools(state=lambda: dict(STATE), events=lambda: [{'file': 'ev_1.json', 'kind': 'takeover', 'gps': [1, 2, 3]}],
                        event=lambda f: {'file': f, 'samples': [{'v': i, 'gps': [1, 2, 3]} for i in range(300)]},
                        trips=lambda: {'files': [{'name': 'map_20261101_090000.jsonl'}]},
                        trip=lambda n: {'file': n, 'points': [[0, 1, 2, 36, 30, 1, 0, 5], [2, 1, 2, 36, 30, 1, 1, 5]],
                                        'objects': [[0, 1, 2, 'car']], 'stops': [],
                                        'cols': {'points': ['t', 'lat', 'lon', 'kmh', 'limit', 'acc', 'op', 'gps_acc'],
                                                 'objects': ['t', 'lat', 'lon', 'kind'], 'stops': ['t', 'lat', 'lon', 'seconds']}},
                        diagnostics=lambda: {'available': True, 'modules': []},
                        can=lambda: {'1:BMS': {'name': 'BMS_status', 'addr': 0x102, 'sig': {'BMS_state': 2}}})


class Base(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self._kf, self._cf = dict(ra.KEY_FILES), ra.CONFIG_FILE
        ra.KEY_FILES['claude'] = os.path.join(self.d, 'ak'); ra.KEY_FILES['openai'] = os.path.join(self.d, 'ok')
        ra.CONFIG_FILE = os.path.join(self.d, 'cfg.json')
        for p in ra.PROVIDERS: ra.KEY_FILES[p] = os.path.join(self.d, p + '_key')
        ra._AUTO.clear()

    def tearDown(self):
        ra.KEY_FILES.update(self._kf); ra.CONFIG_FILE = self._cf

    def key(self, p, v='k-' + 'x' * 8):
        with open(ra.KEY_FILES[p], 'w') as f: f.write(v + '\n')


class TestOff(Base):
    def test_no_key_is_off(self):
        self.assertFalse(ra.status()['enabled'])
        r = ra.ask([{'role': 'user', 'text': 'hi'}], opener=Fake([]))
        self.assertFalse(r['ok']); self.assertIn('no API key', r['error'])

    def test_bad_history(self):
        self.key('claude')
        self.assertFalse(ra.ask([{'role': 'assistant', 'text': 'x'}], opener=Fake([]))['ok'])


class TestClaude(Base):
    def test_tool_loop_and_request_shape(self):
        self.key('claude')
        f = Fake([('anthropic', 200, {'model': 'claude-opus-5-5', 'stop_reason': 'tool_use', 'content': [
                      {'type': 'thinking', 'thinking': '', 'signature': 's'},
                      {'type': 'tool_use', 'id': 't1', 'name': 'get_live_state', 'input': {}}]}),
                  ('anthropic', 200, {'model': 'claude-opus-5-5', 'stop_reason': 'end_turn', 'content': [
                      {'type': 'text', 'text': 'You are at 72 km/h.'}]})])
        r = ra.ask([{'role': 'user', 'text': 'how fast?'}], 'drive', tools(), opener=f)
        self.assertTrue(r['ok']); self.assertEqual(r['provider'], 'claude'); self.assertEqual(r['tools'], ['get_live_state'])
        url, hd, b = f.sent[0]
        hd = dict((k.lower(), v) for k, v in hd.items())
        self.assertEqual(hd['anthropic-version'], '2026-10-21'); self.assertEqual(hd['anthropic-beta'], 'server-side-fallback-2026-07-01')
        self.assertEqual(b['model'], 'claude-opus-5-5'); self.assertEqual(b['fallbacks'], 'default')
        self.assertEqual(b['output_config'], {'effort': 'low'}); self.assertNotIn('thinking', b)
        self.assertEqual(b['tool_choice'], {'type': 'auto'})
        b2 = f.sent[1][2]
        self.assertEqual(b2['messages'][1]['content'][0]['type'], 'thinking')     # echoed unchanged
        res = b2['messages'][2]['content'][0]
        self.assertEqual(res['tool_use_id'], 't1')
        self.assertNotIn('lat', res['content']); self.assertIn('"nearest"', res['content'])

    def test_refusal(self):
        self.key('claude')
        f = Fake([('anthropic', 200, {'stop_reason': 'refusal', 'content': []})])
        r = ra.ask([{'role': 'user', 'text': 'x'}], tools=tools(), opener=f)
        self.assertTrue(r['ok']); self.assertEqual(r['stop'], 'refusal')


class TestFallover(Base):
    def test_rate_limited_claude_goes_to_openai(self):
        self.key('claude'); self.key('openai')
        f = Fake([('anthropic', 429, None),
                  ('openai', 200, {'model': 'gpt-5', 'choices': [{'finish_reason': 'tool_calls', 'message': {
                      'role': 'assistant', 'content': None, 'tool_calls': [{'id': 'c1', 'type': 'function', 'function': {
                          'name': 'find_can_signals', 'arguments': '{"query": "bms"}'}}]}}]}),
                  ('openai', 200, {'model': 'gpt-5', 'choices': [{'finish_reason': 'stop', 'message': {'role': 'assistant', 'content': 'BMS ok'}}]})])
        r = ra.ask([{'role': 'user', 'text': 'battery?'}], tools=tools(), opener=f)
        self.assertTrue(r['ok']); self.assertEqual(r['provider'], 'openai'); self.assertIn('Claude was unavailable', r['note'])
        hd = dict((k.lower(), v) for k, v in f.sent[1][1].items())
        self.assertTrue(hd['authorization'].startswith('Bearer '))
        b = f.sent[2][2]
        self.assertEqual(b['messages'][0]['role'], 'system'); self.assertEqual(b['messages'][-1]['role'], 'tool')
        self.assertIn('BMS_state', b['messages'][-1]['content'])

    def test_auth_error_does_not_fall_over(self):
        self.key('claude'); self.key('openai')
        r = ra.ask([{'role': 'user', 'text': 'x'}], tools=tools(), opener=Fake([('anthropic', 401, None)]))
        self.assertFalse(r['ok']); self.assertEqual(r['provider'], 'claude')

    def test_chosen_provider_first(self):
        self.key('claude'); self.key('openai')
        f = Fake([('openai', 200, {'choices': [{'finish_reason': 'stop', 'message': {'content': 'hi'}}]})])
        self.assertEqual(ra.ask([{'role': 'user', 'text': 'x'}], provider='openai', tools=tools(), opener=f)['provider'], 'openai')


OAI_STOP = {'choices': [{'finish_reason': 'stop', 'message': {'role': 'assistant', 'content': 'ok'}}]}


class TestFreeProviders(Base):
    def test_gemini_first_auto_model_and_echo(self):
        self.key('gemini'); self.key('claude')
        f = Fake([('generativelanguage.googleapis.com/v1beta/openai/models', 200, {'data': [
                      {'id': 'models/gemini-2.5-flash'}, {'id': 'models/gemini-3.6-flash'}, {'id': 'models/gemini-3.6-flash-lite'},
                      {'id': 'models/gemini-3.8-flash-image'}, {'id': 'models/gemini-2.5-pro'}]}),
                  ('generativelanguage.googleapis.com/v1beta/openai/chat/completions', 200, {'choices': [{'finish_reason': 'tool_calls', 'message': {
                      'role': 'assistant', 'content': None, 'reasoning': 'x', 'extra_content': {'google': {'thought_signature': 'sig'}},
                      'tool_calls': [{'id': '', 'type': 'function', 'function': {'name': 'list_events', 'arguments': '{}'}}]}}]}),
                  ('chat/completions', 200, OAI_STOP)])
        r = ra.ask([{'role': 'user', 'text': 'events?'}], tools=tools(), opener=f)
        self.assertEqual(r['provider'], 'gemini')
        b = f.sent[1][2]
        self.assertEqual(b['model'], 'gemini-3.6-flash'); self.assertIn('max_tokens', b)
        echo = f.sent[2][2]['messages'][-2]
        self.assertNotIn('reasoning', echo); self.assertEqual(echo['extra_content']['google']['thought_signature'], 'sig')
        self.assertEqual(f.sent[2][2]['messages'][-1]['tool_call_id'], 'call_0')
        # model list is cached: the next question asks no /models
        f2 = Fake([('chat/completions', 200, OAI_STOP)])
        ra.ask([{'role': 'user', 'text': 'x'}], tools=tools(), opener=f2)
        self.assertEqual(len(f2.sent), 1)

    def test_groq_rate_limited_goes_on(self):
        self.key('groq'); self.key('grok')
        f = Fake([('api.groq.com/openai/v1/models', 200, {'data': [{'id': 'whisper-large-v3'}, {'id': 'llama-3.3-70b-versatile'}, {'id': 'openai/gpt-oss-120b'}]}),
                  ('api.groq.com/openai/v1/chat/completions', 429, None),
                  ('api.x.ai/v1/models', 200, {'data': [{'id': 'grok-4-1-fast-reasoning'}, {'id': 'grok-4.7'}, {'id': 'grok-4.7-fast'}, {'id': 'grok-imagine-image'}]}),
                  ('api.x.ai/v1/chat/completions', 200, OAI_STOP)])
        r = ra.ask([{'role': 'user', 'text': 'x'}], tools=tools(), opener=f)
        self.assertTrue(r['ok']); self.assertEqual(r['provider'], 'grok'); self.assertIn('Groq was unavailable', r['note'])
        self.assertEqual(f.sent[1][2]['model'], 'openai/gpt-oss-120b'); self.assertEqual(f.sent[3][2]['model'], 'grok-4.7-fast')

    def test_models_list_fails_uses_fallback(self):
        self.key('groq')
        f = Fake([('/models', 500, None), ('chat/completions', 200, OAI_STOP)])
        ra.ask([{'role': 'user', 'text': 'x'}], tools=tools(), opener=f)
        self.assertEqual(f.sent[1][2]['model'], ra.PROVIDERS['groq']['fallback_model'])

    def test_config_order_and_model(self):
        self.key('gemini'); self.key('groq')
        with open(ra.CONFIG_FILE, 'w') as fh: json.dump({'order': ['groq'], 'models': {'groq': 'my-model'}, 'reasoning_effort': {'groq': 'low'}}, fh)
        self.assertEqual(ra.providers(), ['groq', 'gemini'])
        f = Fake([('api.groq.com/openai/v1/chat/completions', 200, OAI_STOP)])
        ra.ask([{'role': 'user', 'text': 'x'}], tools=tools(), opener=f)
        self.assertEqual(f.sent[0][2]['model'], 'my-model'); self.assertEqual(f.sent[0][2]['reasoning_effort'], 'low')

    def test_pick_model_nothing(self):
        self.assertIsNone(ra.pick_model('gemini', ['models/text-embedding-004']))


class TestData(unittest.TestCase):
    def test_event_downsampled_no_gps(self):
        r = json.loads(tools().run('get_event', {'file': 'ev_1.json'}))
        self.assertLessEqual(len(r['samples']), 60); self.assertNotIn('gps', r['samples'][0])

    def test_event_name_checked(self):
        self.assertTrue(tools().run('get_event', {'file': '../etc/passwd'}).startswith('error'))

    def test_gps_kept_when_asked(self):
        self.assertIn('"gps"', tools().run('get_live_state', {}, gps=True))

    def test_trip_summary(self):
        r = json.loads(tools().run('get_trip_summary', {'name': 'x'}))
        self.assertEqual(r['distance_km'], 0.02); self.assertEqual(r['pct_time_over_limit'], 100)
        self.assertEqual(r['pct_time_openpilot'], 100); self.assertEqual(r['radar_objects_marked_by_kind'], {'car': 1})
        self.assertNotIn('lat', json.dumps(r))

    def test_unknown_tool(self):
        self.assertTrue(tools().run('send_can', {}).startswith('error'))


if __name__ == '__main__':
    unittest.main()
