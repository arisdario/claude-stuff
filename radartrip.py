#!/usr/bin/env python3
# radartrip.py - drive logs on a map. Reads radarweb's 1-per-second CAN log (/data/maplog/map_*.jsonl) and shows the
# route (coloured by speed, or by speed vs the map speed limit), stops, and what the radar saw placed at its real
# position (GPS + heading + radar distance/lateral offset): vehicles, oncoming traffic, people/bikes crossing in front,
# the radar's own pedestrian/two-wheel calls, and static obstacles. Map: OpenStreetMap via Leaflet (free, no key).
#   laptop:  python3 radartrip.py map_20261025_224420.jsonl            -> map_20261025_224420.html (open in a browser)
#   EON:     radarweb's /trips page runs  radartrip.py --json IN OUT  in the background and shows the same viewer
# Pure python (3.6+), no numpy. Logs written by the newer radarweb also carry its live 10 Hz results per second ('o':
# class, crossing, model guess); older logs only have the raw radar frames, decoded here at 1 per second (coarser).
# The output contains your GPS positions: keep the HTML/JSON files private.
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radarml

CL = {0: 'unknown', 1: 'four_wheel', 2: 'two_wheel', 3: 'pedestrian', 4: 'construction'}
SAMPLE_S = 3.0          # one map marker per radar track per this many seconds (keeps the map readable)


def _num(v):
    try: return float(str(v).split()[0])
    except (TypeError, ValueError, IndexError): return None

def _place(lat, lon, hdg, d, y):
    """lat/lon of a point d m ahead and y m to the left (radar convention) of a car at lat/lon heading hdg (deg from north)"""
    h = math.radians(hdg); east = d * math.sin(h) - y * math.cos(h); north = d * math.cos(h) + y * math.sin(h)
    return round(lat + north / 111320.0, 6), round(lon + east / (111320.0 * math.cos(math.radians(lat))), 6)

def _radar_objects(raw, v):
    out = []
    for k in range(32):
        A = raw.get(hex(0x310 + 3 * k)); B = raw.get(hex(0x311 + 3 * k))
        if not A or not B: continue
        a = radarml._dec(bytes.fromhex(A), radarml.SIG_A); b = radarml._dec(bytes.fromhex(B), radarml.SIG_B)
        if not (a['Valid'] or a['Tracked']) or a['LongDist'] <= 0 or a['Index'] != b['Index2']: continue
        out.append({'s': k, 'd': a['LongDist'], 'y': a['LatDist'], 'gs': a['LongSpeed'] + v, 'ls': b['LatSpeed'],
                    'rc': CL.get(b['Class'], 'unknown'), 'pc': b['ProbClass']})
    return out


def trip(path):
    pts, objs, stops = [], [], []
    last_mark = {}; prev = {}; stop0 = None; n = 0; t_first = None
    for ln in open(path):
        try: r = json.loads(ln)
        except ValueError: continue
        n += 1; t = r.get('t', 0); v = r.get('v') or 0.0; msgs = r.get('msgs', {})
        loc = msgs.get('MCU_locationStatus', {}); gps = msgs.get('UI_gpsVehicleSpeed', {})
        lat, lon = _num(loc.get('MCU_latitude')), _num(loc.get('MCU_longitude'))
        acc = _num(loc.get('MCU_gpsAccuracy')); hdg = _num(gps.get('UI_gpsVehicleHeading'))
        lim = _num(gps.get('UI_mppSpeedLimit'))
        steer = _num(msgs.get('STW_ANGL_STAT', {}).get('StW_Angl')) or 0.0
        if lat is None or lon is None or abs(lat) < 0.1 or abs(lat) > 90 or abs(lon) > 180: continue   # no fix yet
        if t_first is None: t_first = t
        kmh = round(v * 3.6, 1)
        pts.append([round(t, 1), round(lat, 6), round(lon, 6), kmh, lim if lim else None, 1 if r.get('acc') else 0,
                    1 if r.get('alc') else 0, round(acc, 1) if acc is not None else None])
        # stops
        if v < 0.5:
            if stop0 is None: stop0 = (t, lat, lon)
        else:
            if stop0 and t - stop0[0] >= 5: stops.append([round(stop0[0], 1), stop0[1], stop0[2], round(t - stop0[0])])
            stop0 = None
        if hdg is None: continue
        # objects: the newer radarweb logs its own live (10 Hz) results; otherwise decode the raw frames (1 Hz)
        live = r.get('o')
        if live is not None:
            cur = [{'s': o[0], 'd': o[1], 'y': o[2], 'gs': o[3], 'vy': o[4], 'c': o[5], 'rc': o[6], 'ml': o[7]} for o in live]
            for x in r.get('xs', []):      # crossing detections inside this second, so none is lost between log lines
                cur.append({'s': x[0], 'd': x[1], 'y': x[2], 'gs': x[3], 'vy': x[4], 'c': x[5], 'rc': 'unknown', 'ml': None, 'xs': True})
        else:
            cur = _radar_objects(r.get('raw', {}), v)
            for o in cur:                  # sideways speed from 1-s position change, own rotation removed (see radarml CROSSING)
                p = prev.get(o['s'])
                o['vy_raw'] = (o['y'] - p['y']) / (t - p['t']) if p and 0.5 < t - p['t'] < 1.6 and abs(o['d'] - p['d']) < 6 else None
                if o['vy_raw'] is not None and 0.3 < abs(o['ls']) < 15 and abs(o['vy_raw']) > 0.3 and (o['ls'] > 0) != (o['vy_raw'] > 0): o['vy_raw'] = None
            yaw = radarml.scene_yaw(cur)
            for o in cur:
                o['vy'] = radarml.ground_vy(o.pop('vy_raw'), yaw, o['d'], v, steer)
                xc = radarml.cross_class(o['vy'], o['gs'], o['d'])
                o['c'] = xc or (o['rc'] if o['rc'] != 'unknown' and o['pc'] >= 50 else 'unknown'); o['xs'] = bool(xc); o['ml'] = None
            prev = dict((o['s'], {'t': t, 'd': o['d'], 'y': o['y']}) for o in cur)
        for o in cur:
            gs = o['gs']
            if o.get('xs') or o.get('cross'): kind = {'pedestrian': 'ped_cross', 'two_wheel': 'bike_cross'}.get(o['c'], 'car_cross')
            elif gs < -3: kind = 'oncoming'
            elif gs > 3: kind = 'vehicle'
            elif o['c'] in ('pedestrian', 'two_wheel'): kind = 'radar_' + o['c']
            else: kind = 'static'
            key = (kind, o['s'])
            if t - last_mark.get(key, -1e9) < (SAMPLE_S if kind != 'static' else 30.0): continue
            last_mark[key] = t
            la, lo = _place(lat, lon, hdg, o['d'], o['y'])
            objs.append([round(t, 1), la, lo, kind, round(o['d'], 1), round(o['y'], 1), round(gs * 3.6), None if o.get('vy') is None else round(o['vy'] * 3.6, 1),
                         o.get('rc'), (o['ml'] or {}).get('c') if isinstance(o.get('ml'), dict) else o.get('ml')])
    if stop0 and pts and pts[-1][0] - stop0[0] >= 5: stops.append([round(stop0[0], 1), stop0[1], stop0[2], round(pts[-1][0] - stop0[0])])
    return {'file': os.path.basename(path), 'rows': n, 'points': pts, 'objects': objs, 'stops': stops,
            'cols': {'points': ['t', 'lat', 'lon', 'kmh', 'limit', 'acc', 'op', 'gps_acc'],
                     'objects': ['t', 'lat', 'lon', 'kind', 'd', 'y', 'ground_kmh', 'side_kmh', 'radar_class', 'model'],
                     'stops': ['t', 'lat', 'lon', 'seconds']}}


VIEWER = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Radar trips</title>
<link rel=stylesheet href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<style>
:root{--bg:#101215;--fg:#e6e8ea;--mut:#8b9199;--card:#1b1e22;--line:#2a2e34}
@media(prefers-color-scheme:light){:root{--bg:#f3f4f6;--fg:#1c1f23;--mut:#5b626b;--card:#fff;--line:#e1e4e8}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
body{display:flex;flex-direction:column}
#top{display:flex;gap:12px;align-items:center;padding:8px 16px;flex-wrap:wrap}#top a{color:inherit}
select,button{font:inherit;padding:4px 8px;border-radius:6px;border:1px solid var(--line);background:var(--card);color:var(--fg)}
#map{flex:1;min-height:300px}
#chart{height:110px;border-top:1px solid var(--line);background:var(--card);position:relative}
#chart canvas{width:100%;height:100%;display:block;cursor:crosshair}
#info{padding:4px 16px;color:var(--mut);font-size:12px;min-height:18px}
.leg{display:inline-flex;align-items:center;gap:4px;margin-right:10px;font-size:12px}.dot{width:10px;height:10px;border-radius:50%;display:inline-block}
.leaflet-popup-content{font:13px system-ui,sans-serif}
</style></head><body>
<div id=top><span id=nav></span><b>Trips</b><select id=sel></select><select id=colour><option value=speed>colour: speed</option><option value=limit>colour: vs speed limit</option><option value=op>colour: openpilot / cruise</option></select><span id=status class=m></span></div>
<div id=map></div>
<div id=info></div>
<div id=chart><canvas id=cv></canvas></div>
<script>
var KIND={vehicle:['Vehicle (same direction)','#e5484d',5],oncoming:['Oncoming vehicle','#ff8a00',5],ped_cross:['Pedestrian crossing','#1a73e8',7],
 bike_cross:['Cyclist crossing','#8e44ad',7],car_cross:['Vehicle crossing','#b71c1c',6],radar_pedestrian:['Radar: pedestrian','#4fc3f7',6],
 radar_two_wheel:['Radar: two-wheeler','#ce93d8',6],static:['Static obstacle','#8b9199',3]};
var map=L.map('map',{preferCanvas:true}),tiles=L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
map.setView([0,0],2);
var route=L.layerGroup().addTo(map),stopsL=L.layerGroup().addTo(map),layers={},ctl=null,T=null,cursor=null;
Object.keys(KIND).forEach(function(k){layers[k]=L.layerGroup();if(k!='static')layers[k].addTo(map)});
function esc(x){return String(x==null?'-':x).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function hm(t){var d=new Date(t*1000);return d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit'})}
function spdCol(k){var s=Math.min(1,k/120);return 'hsl('+Math.round(130-130*s)+',75%,48%)'}
function col(p,mode){if(mode=='limit'){if(!p[4])return '#8b9199';var r=p[3]/p[4];return r<=1.0?'#2e9d4f':r<=1.1?'#d4a017':'#d62828'}
 if(mode=='op')return p[6]?'#1a73e8':p[5]?'#00a3a3':'#8b9199';return spdCol(p[3])}
function draw(){
 route.clearLayers();stopsL.clearLayers();Object.keys(layers).forEach(function(k){layers[k].clearLayers()});
 if(!T||!T.points.length){$('status').textContent=T?'no GPS fix in this log':'';return}
 var P=T.points,mode=$('colour').value,seg=[],c0=null;
 for(var i=1;i<P.length;i++){var a=P[i-1],b=P[i];if(b[0]-a[0]>10)continue;var c=col(b,mode);
  L.polyline([[a[1],a[2]],[b[1],b[2]]],{color:c,weight:5,opacity:.9}).bindTooltip(hm(b[0])+' · '+b[3]+' km/h'+(b[4]?' · limit '+b[4]:'')+(b[6]?' · openpilot':b[5]?' · cruise':'')).addTo(route)}
 L.circleMarker([P[0][1],P[0][2]],{radius:7,color:'#fff',weight:2,fillColor:'#2e9d4f',fillOpacity:1}).bindPopup('Start '+hm(P[0][0])).addTo(route);
 var e=P[P.length-1];L.circleMarker([e[1],e[2]],{radius:7,color:'#fff',weight:2,fillColor:'#d62828',fillOpacity:1}).bindPopup('End '+hm(e[0])).addTo(route);
 T.stops.forEach(function(s){L.circleMarker([s[1],s[2]],{radius:6,color:'#000',weight:1,fillColor:'#ffd400',fillOpacity:.9}).bindPopup('Stopped '+s[3]+' s at '+hm(s[0])).addTo(stopsL)});
 var cnt={};T.objects.forEach(function(o){var k=KIND[o[3]];if(!k)return;cnt[o[3]]=(cnt[o[3]]||0)+1;
  L.circleMarker([o[1],o[2]],{radius:k[2],color:k[1],weight:1,fillColor:k[1],fillOpacity:o[3]=='static'?.5:.85}).bindPopup('<b>'+k[0]+'</b><br>'+hm(o[0])+'<br>'+o[4]+' m ahead, '+Math.abs(o[5])+' m '+(o[5]>=0?'left':'right')+
   '<br>ground speed '+o[6]+' km/h'+(o[7]!=null?' · sideways '+Math.abs(o[7])+' km/h':'')+'<br>radar class: '+esc(o[8])+(o[9]?'<br>model: '+esc(o[9]):'')).addTo(layers[o[3]])});
 if(ctl)map.removeControl(ctl);var ov={'Route':route,'Stops':stopsL};
 Object.keys(KIND).forEach(function(k){ov['<span class=dot style="background:'+KIND[k][1]+'"></span> '+KIND[k][0]+' ('+(cnt[k]||0)+')']=layers[k]});
 ctl=L.control.layers(null,ov,{collapsed:innerWidth<700}).addTo(map);
 map.fitBounds(L.latLngBounds(P.map(function(p){return [p[1],p[2]]})).pad(0.05));
 var dist=0;for(var i=1;i<P.length;i++)if(P[i][0]-P[i-1][0]<10)dist+=map.distance([P[i-1][1],P[i-1][2]],[P[i][1],P[i][2]]);
 var mx=Math.max.apply(null,P.map(function(p){return p[3]}));
 $('status').textContent=hm(P[0][0])+' - '+hm(e[0])+' · '+(dist/1000).toFixed(1)+' km · max '+Math.round(mx)+' km/h · '+T.stops.length+' stops';
 chart()}
function chart(){var cv=$('cv'),r=cv.getBoundingClientRect(),dp=devicePixelRatio||1;cv.width=r.width*dp;cv.height=r.height*dp;var g=cv.getContext('2d');g.scale(dp,dp);
 var W=r.width,H=r.height,P=T&&T.points;g.clearRect(0,0,W,H);if(!P||P.length<2)return;
 var t0=P[0][0],t1=P[P.length-1][0],mx=Math.max(30,Math.max.apply(null,P.map(function(p){return Math.max(p[3],p[4]||0)})))*1.1,fg=getComputedStyle(document.body).color;
 function X(t){return 36+(W-44)*(t-t0)/(t1-t0)}function Y(v){return H-16-(H-26)*v/mx}
 g.fillStyle=fg;g.globalAlpha=.6;g.font='11px system-ui';[0,50,100,150].forEach(function(v){if(v<mx){g.fillText(v,4,Y(v)+4);g.globalAlpha=.12;g.fillRect(36,Y(v),W-44,1);g.globalAlpha=.6}});g.globalAlpha=1;
 g.strokeStyle='#d62828';g.setLineDash([4,3]);g.beginPath();var on=false;P.forEach(function(p){if(p[4]){on?g.lineTo(X(p[0]),Y(p[4])):g.moveTo(X(p[0]),Y(p[4]));on=true}else on=false});g.stroke();g.setLineDash([]);
 g.strokeStyle='#1a73e8';g.lineWidth=1.5;g.beginPath();P.forEach(function(p,i){i?g.lineTo(X(p[0]),Y(p[3])):g.moveTo(X(p[0]),Y(p[3]))});g.stroke();
 T.objects.forEach(function(o){if(o[3]=='ped_cross'||o[3]=='bike_cross'||o[3]=='radar_pedestrian'||o[3]=='radar_two_wheel'){g.fillStyle=KIND[o[3]][1];g.fillRect(X(o[0])-1,H-12,3,8)}});
 cv.onmousemove=cv.onclick=function(ev){var x=ev.offsetX,t=t0+(x-36)/(W-44)*(t1-t0),best=P[0];P.forEach(function(p){if(Math.abs(p[0]-t)<Math.abs(best[0]-t))best=p});
  $('info').textContent=hm(best[0])+' · '+best[3]+' km/h'+(best[4]?' · limit '+best[4]:'')+(best[7]!=null?' · GPS ±'+best[7]+' m':'');
  if(!cursor)cursor=L.circleMarker([best[1],best[2]],{radius:8,color:'#fff',weight:3,fillColor:'#000',fillOpacity:.6}).addTo(map);else cursor.setLatLng([best[1],best[2]]);
  if(ev.type=='click')map.panTo([best[1],best[2]])}}
var $=function(i){return document.getElementById(i)};
$('colour').onchange=draw;onresize=function(){chart()};
if(window.TRIP){T=window.TRIP;$('sel').style.display='none';draw()}
else{$('nav').innerHTML='<a href="/">road view</a> <a href="/ml">what it learned</a>';
 fetch('/api/trips').then(function(r){return r.json()}).then(function(j){var s=$('sel');
  s.innerHTML=j.files.map(function(f){return '<option value="'+esc(f.name)+'">'+esc(f.name.replace('map_','').replace('.jsonl',''))+' · '+(f.size/1e6).toFixed(1)+' MB'+(f.ready?'':' (not processed yet)')+'</option>'}).join('');
  s.onchange=function(){load(s.value)};var q=(location.search.match(/f=([\w.]+)/)||[])[1];if(q)s.value=q;if(s.value)load(s.value)})}
function load(f){$('status').textContent='processing '+f+'… (runs in the background, can take a minute on the EON)';
 fetch('/api/trip?f='+encodeURIComponent(f)).then(function(r){return r.json()}).then(function(j){if(j.pending){setTimeout(function(){load(f)},3000);return}
  if(j.error){$('status').textContent=j.error;return}T=j;draw()}).catch(function(){$('status').textContent='failed, retrying…';setTimeout(function(){load(f)},5000)})}
</script></body></html>"""


def html(t):
    return VIEWER.replace('<script>\nvar KIND', '<script>window.TRIP=' + json.dumps(t, separators=(',', ':')) + ';\n</script>\n<script>\nvar KIND', 1)


if __name__ == '__main__':
    a = sys.argv[1:]
    if len(a) == 3 and a[0] == '--json':                      # used by radarweb: write the trip JSON (atomically)
        t = trip(a[1]); json.dump(t, open(a[2] + '.tmp', 'w'), separators=(',', ':')); os.replace(a[2] + '.tmp', a[2])
    elif a and a[0] not in ('-h', '--help'):
        for fn in a:
            t = trip(fn); out = os.path.splitext(fn)[0] + '.html'
            open(out, 'w').write(html(t))
            k = {}
            for o in t['objects']: k[o[3]] = k.get(o[3], 0) + 1
            print('%s: %d GPS points, %d stops, objects %s -> %s' % (fn, len(t['points']), len(t['stops']), k, out))
    else:
        print('usage: python3 radartrip.py map_YYYYMMDD_HHMMSS.jsonl [...]   (writes .html next to each log)')
