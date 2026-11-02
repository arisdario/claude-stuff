"""OSM tag rules for car routing: which ways a car may use, in which direction, and how fast. Pure functions over a tag dict
(so they are unit-testable and the validator can re-apply them to the raw tags stored in the graph).

NOT modelled (the data used does not carry them): turn-restriction relations, time-conditional access, vehicle weight/height
limits, toll/ferry preferences. A route can therefore contain a turn that a "no_left_turn" relation forbids; the validation
report states this limitation."""
import re
import unicodedata

ROUTABLE = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link', 'secondary', 'secondary_link',
            'tertiary', 'tertiary_link', 'unclassified', 'residential', 'living_street', 'service'}
DEFAULT_KPH = {'motorway': 110, 'motorway_link': 60, 'trunk': 90, 'trunk_link': 50, 'primary': 70, 'primary_link': 45,
               'secondary': 60, 'secondary_link': 40, 'tertiary': 50, 'tertiary_link': 35, 'unclassified': 40,
               'residential': 30, 'living_street': 10, 'service': 20}
MAX_KPH = 130.0
ACCESS_KEYS = ('motorcar', 'motor_vehicle', 'vehicle', 'access')       # most specific first
DENY = {'no', 'private', 'customers', 'delivery', 'agricultural', 'forestry', 'military', 'emergency', 'permit', 'discouraged'}
DESTINATION = {'destination'}                                           # allowed, but not for through traffic: penalised
BAD_SERVICE = {'parking_aisle', 'driveway', 'drive-through', 'emergency_access'}
ROUNDABOUT = {'roundabout', 'circular'}
SPEED_WORDS = {'ro:urban': 50, 'ro:rural': 90, 'ro:trunk': 100, 'ro:motorway': 130, 'walk': 7, 'none': None}


def _s(tags, key):
    v = tags.get(key)
    if v is None:
        return ''
    s = str(v).strip().lower()
    return '' if s in ('nan', 'none') else s


def access_ok(tags):
    """(allowed, cost_factor, reason). The most specific present access tag decides; 'destination' is allowed but penalised."""
    hw = _s(tags, 'highway')
    if hw not in ROUTABLE:
        return False, 1.0, 'class:%s' % (hw or 'none')
    if hw == 'service' and _s(tags, 'service') in BAD_SERVICE:
        return False, 1.0, 'service:%s' % _s(tags, 'service')
    for key in ACCESS_KEYS:
        v = _s(tags, key)
        if not v:
            continue
        if v in DENY:
            return False, 1.0, '%s=%s' % (key, v)
        if v in DESTINATION:
            return True, 1.6, '%s=destination' % key
        return True, 1.0, 'ok'
    return True, 1.0, 'ok'


def oneway_dir(tags):
    """'both' | 'fwd' (along way order) | 'rev' (against way order) | 'none' (unusable: reversible lanes)."""
    for key in ('oneway:motorcar', 'oneway:motor_vehicle', 'oneway'):
        v = _s(tags, key)
        if not v:
            continue
        if v in ('yes', '1', 'true'):
            return 'fwd'
        if v in ('-1', 'reverse'):
            return 'rev'
        if v in ('no', '0', 'false'):
            return 'both'
        if v in ('reversible', 'alternating'):
            return 'none'
    if _s(tags, 'junction') in ROUNDABOUT:
        return 'fwd'
    if _s(tags, 'highway') in ('motorway', 'motorway_link'):
        return 'fwd'
    return 'both'


def maxspeed_kph(tags):
    v = _s(tags, 'maxspeed')
    if not v:
        return None
    if v in SPEED_WORDS:
        return SPEED_WORDS[v]
    m = re.match(r'^(\d+(?:\.\d+)?)\s*(mph)?$', v)
    if not m:
        return None
    k = float(m.group(1)) * (1.609344 if m.group(2) else 1.0)
    return k if 3.0 <= k <= 200.0 else None


def speed_kph(tags):
    ms = maxspeed_kph(tags)
    v = ms if ms else DEFAULT_KPH.get(_s(tags, 'highway'), 30)
    return min(float(v), MAX_KPH)


def is_roundabout(tags):
    return _s(tags, 'junction') in ROUNDABOUT


def norm_name(s):
    """case/diacritic-insensitive key: 'Timișu de Sus' == 'Timisu de sus' == 'Timişu de Sus' (comma-below vs cedilla)"""
    if s is None:
        return ''
    t = unicodedata.normalize('NFKD', str(s)).encode('ascii', 'ignore').decode('ascii')
    return re.sub(r'\s+', ' ', t.casefold()).strip()
