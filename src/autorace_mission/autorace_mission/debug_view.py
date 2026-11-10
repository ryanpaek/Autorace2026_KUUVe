"""맵 전체 디버깅 화면 (웹). 리모에서 돌리고 노트북 브라우저로 본다: http://<리모IP>:8080

추가 설치 없음 (파이썬 표준 http.server). 보여 주는 것:
  - 트랙 도면(경로 중심선, 차로 근사, 흰 선, 구간 점, 주차칸)과 그 위의 차 위치(도면 위치 s + 좌우 d + 방향)
  - 라이다 점(차 위치 기준으로 도면에 겹침): 콘·정차 리모·벽이 도면과 맞는지
  - 미션 상태/단계/이유(note), 예비 경로(FALLBACK) 기록, 속도 명령, 차선 상태, YOLO 확인 명령,
    주차·M7 회피·갈림길·라바콘 상태, 차선 노드 디버그 영상(/lane/debug/compressed)
차 위치는 mission_node의 /route/preview (s, d_mm, psi). 차선을 오래 놓치면 pose_ok=false -> 경로 방향으로 그림.
제어에는 아무것도 보내지 않는다 (구독만).
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from autorace_mission.route import RouteMap

JSON_TOPICS = {
    'mission': '/mission/state', 'preview': '/route/preview', 'lane': '/lane/status',
    'commands': '/perception/commands', 'detections': '/perception/detections',
    'parking': '/parking/state', 'obstacle': '/obstacle/state', 'fork': '/fork/state',
}
LANE_HALF = 250.0


def track_geometry(route: RouteMap) -> dict:
    """화면에 그릴 도면: 중심선·양쪽 선(차로 폭 500 근사)·흰 선·구간 점·주차칸. 단위 mm, 도면 좌표."""
    center, left, right = [], [], []
    s = 0.0
    while s <= route.length:
        x, y = route.xy(s)
        h = route.heading(s)
        center.append((round(x), round(y)))
        left.append((round(x - LANE_HALF * math.sin(h)), round(y + LANE_HALF * math.cos(h))))
        right.append((round(x + LANE_HALF * math.sin(h)), round(y - LANE_HALF * math.cos(h))))
        s += 50.0
    lines = []
    for lm in route.landmarks:
        x, y = route.xy(lm['s'])
        h = route.heading(lm['s'])
        nx, ny = -math.sin(h), math.cos(h)
        lines.append({'name': lm['name'], 's': round(lm['s']),
                      'a': (round(x + LANE_HALF * nx), round(y + LANE_HALF * ny)),
                      'b': (round(x - LANE_HALF * nx), round(y - LANE_HALF * ny))})
    lm_names = {lm['name'] for lm in route.landmarks}
    pts = [{'name': k, 's': round(v), 'xy': tuple(round(c) for c in route.xy(v))}
           for k, v in route.points.items() if k not in lm_names]
    cells = []
    if 'm4_parking_from' in route.points:
        s0 = route.points['m4_parking_from']
        x0, y0 = route.xy(s0)
        h = route.heading(s0)
        fx, fy, rx, ry = math.cos(h), math.sin(h), math.sin(h), -math.cos(h)   # 진행 방향, 오른쪽
        for row in range(2):
            for col in range(3):
                a = col * 520.0
                d = LANE_HALF + 20.0 + row * 620.0
                quad = [(a, d), (a + 500, d), (a + 500, d + 600), (a, d + 600)]
                cells.append({'n': row * 3 + col + 1,
                              'poly': [(round(x0 + u * fx + v * rx), round(y0 + u * fy + v * ry)) for u, v in quad]})
    return {'center': center, 'left': left, 'right': right, 'lines': lines, 'points': pts, 'cells': cells,
            'length': round(route.length)}


class DebugView(Node):
    def __init__(self):
        super().__init__('debug_view')
        self.declare_parameter('port', 8080)
        self.declare_parameter('route_file', '')
        self.declare_parameter('front_axle_from_lidar_mm', -47.0)   # parking_node와 같은 값
        self.declare_parameter('laser_yaw', 0.0)
        self.lock = threading.Lock()
        self.data = {k: None for k in JSON_TOPICS}
        self.stamp = {}
        self.scan = None
        self.cmd = None
        self.jpeg = None
        self.events = []
        self._last_state = None
        self.route = None
        path = self.get_parameter('route_file').value
        if path and os.path.exists(path):
            self.route = RouteMap.from_yaml(path)
        self.track = track_geometry(self.route) if self.route else None
        for key, topic in JSON_TOPICS.items():
            self.create_subscription(String, topic, lambda m, k=key: self._on_json(k, m), 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(String, '/avoid/state', self._on_avoid, 10)     # 한 줄 글자 (JSON 아님)
        self.create_subscription(CompressedImage, '/lane/debug/compressed', self._on_img, 2)
        port = int(self.get_parameter('port').value)
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, body, ctype):
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self._send(PAGE.encode(), 'text/html; charset=utf-8')
                elif self.path == '/track':
                    self._send(json.dumps(node.track).encode(), 'application/json')
                elif self.path.startswith('/state'):
                    self._send(json.dumps(node.snapshot()).encode(), 'application/json')
                elif self.path.startswith('/lane.jpg') and node.jpeg:
                    self._send(node.jpeg, 'image/jpeg')
                else:
                    self.send_response(404)
                    self.end_headers()

        self.server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'debug view: http://<this LIMO IP>:{port}  (route map: {path or "none"})')

    def _on_json(self, key, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.data[key] = d
            self.stamp[key] = time.time()
            if key == 'mission' and isinstance(d, dict):
                st = (d.get('state'), d.get('sub'))
                if st != self._last_state:
                    s = (d.get('route') or {}).get('s_mm')
                    self.events.append({'t': time.strftime('%H:%M:%S'), 'state': f'{st[0]}/{st[1]}',
                                        's': s, 'note': d.get('note', '')})
                    self.events = self.events[-40:]
                    self._last_state = st

    def _on_scan(self, m):
        lx = -float(self.get_parameter('front_axle_from_lidar_mm').value)
        yaw = float(self.get_parameter('laser_yaw').value)
        pts = []
        for i in range(0, len(m.ranges), 2):
            r = m.ranges[i]
            if not (math.isfinite(r) and m.range_min <= r <= min(m.range_max, 3.0)):
                continue
            a = m.angle_min + m.angle_increment * i + yaw
            pts.append((round(r * 1000.0 * math.cos(a) + lx), round(r * 1000.0 * math.sin(a))))  # 앞바퀴 기준 앞+ 왼+
        with self.lock:
            self.scan = pts
            self.stamp['scan'] = time.time()

    def _on_avoid(self, m):
        with self.lock:
            self.data['avoid'] = m.data
            self.stamp['avoid'] = time.time()

    def _on_cmd(self, m):
        with self.lock:
            self.cmd = (round(m.linear.x, 3), round(m.angular.z, 3))

    def _on_img(self, m):
        with self.lock:
            self.jpeg = bytes(m.data)

    def snapshot(self):
        with self.lock:
            now = time.time()
            age = {k: round(now - v, 1) for k, v in self.stamp.items()}
            pose = None
            pv = self.data.get('preview')
            if self.route is not None and isinstance(pv, dict) and pv.get('s') is not None:
                s = float(pv['s'])
                x, y = self.route.xy(s)
                h = self.route.heading(s)
                d = float(pv.get('d_mm', 0.0)) if pv.get('pose_ok') else 0.0
                psi = float(pv['psi']) if pv.get('pose_ok') and pv.get('psi') is not None else h
                pose = {'x': x - d * math.sin(h), 'y': y + d * math.cos(h), 'psi': psi, 's': s,
                        'ok': bool(pv.get('pose_ok'))}
            return {'data': self.data, 'age': age, 'scan': self.scan, 'cmd': self.cmd, 'pose': pose,
                    'events': self.events, 'has_img': self.jpeg is not None}


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoRace 디버그</title><style>
:root{--bg:#111418;--panel:#1b2026;--fg:#e8eaed;--mute:#9aa0a6;--ok:#34a853;--warn:#fbbc04;--bad:#ea4335;--acc:#8ab4f8}
body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.4 system-ui,"Malgun Gothic",sans-serif;display:flex;height:100vh}
#map{flex:1;min-width:0;position:relative}canvas{width:100%;height:100%;display:block}
#side{width:380px;overflow:auto;background:var(--panel);padding:10px;box-sizing:border-box}
h3{margin:10px 0 4px;font-size:13px;color:var(--acc)}.big{font-size:20px;font-weight:700}
.kv{display:grid;grid-template-columns:110px 1fr;gap:1px 6px}.kv div:nth-child(odd){color:var(--mute)}
.ev{font-size:12px;border-bottom:1px solid #2a2f36;padding:2px 0}.fb{color:var(--warn)}
#tools{position:absolute;top:8px;left:8px;display:flex;gap:6px}button{background:#2a2f36;color:var(--fg);border:1px solid #3c4043;border-radius:4px;padding:4px 8px;cursor:pointer}
img{width:100%;border-radius:4px;margin-top:4px}.stale{color:var(--bad)}
</style></head><body><div id="map"><canvas id="c"></canvas><div id="tools">
<button id="follow">차 따라가기: 켬</button><button id="fit">전체 보기</button><span id="hint" style="color:var(--mute);align-self:center">휠: 확대 · 끌기: 이동</span></div></div>
<div id="side"><div class="big" id="st">연결 중…</div><div id="note" style="color:var(--mute)"></div>
<h3>차</h3><div class="kv" id="car"></div>
<h3>인식</h3><div class="kv" id="perc"></div>
<h3>미션 노드</h3><div class="kv" id="nodes"></div>
<h3>차선 영상</h3><img id="lane" style="display:none"><div id="noimg" style="color:var(--mute)">(차선 디버그 영상 없음)</div>
<h3>상태 기록</h3><div id="ev"></div></div>
<script>
const cv=document.getElementById('c'),cx=cv.getContext('2d');let T=null,S=null,view={x:5000,y:3000,k:0.08},follow=true,drag=null;
function rs(){cv.width=cv.clientWidth*devicePixelRatio;cv.height=cv.clientHeight*devicePixelRatio}addEventListener('resize',rs);rs();
const W2S=(x,y)=>[cv.width/2+(x-view.x)*view.k*devicePixelRatio,cv.height/2-(y-view.y)*view.k*devicePixelRatio];
function fit(){view={x:5000,y:3000,k:Math.min(cv.clientWidth/10800,cv.clientHeight/6800)}}
cv.onwheel=e=>{e.preventDefault();view.k*=e.deltaY<0?1.15:1/1.15};
cv.onmousedown=e=>{drag=[e.clientX,e.clientY];follow=false;upd()};onmouseup=()=>drag=null;
onmousemove=e=>{if(!drag)return;view.x-=(e.clientX-drag[0])/view.k;view.y+=(e.clientY-drag[1])/view.k;drag=[e.clientX,e.clientY]};
document.getElementById('follow').onclick=()=>{follow=!follow;upd()};document.getElementById('fit').onclick=()=>{follow=false;fit();upd()};
function upd(){document.getElementById('follow').textContent='차 따라가기: '+(follow?'켬':'끔')}
function poly(p,close){cx.beginPath();p.forEach((q,i)=>{const[a,b]=W2S(q[0],q[1]);i?cx.lineTo(a,b):cx.moveTo(a,b)});if(close)cx.closePath()}
function draw(){cx.clearRect(0,0,cv.width,cv.height);const L=devicePixelRatio;
 if(T){cx.lineWidth=2*L;cx.strokeStyle='#fbbc04';poly(T.left);cx.stroke();cx.strokeStyle='#dadce0';poly(T.right);cx.stroke();
  cx.setLineDash([6*L,6*L]);cx.strokeStyle='#5f6368';cx.lineWidth=1*L;poly(T.center);cx.stroke();cx.setLineDash([]);
  T.cells.forEach(c=>{cx.strokeStyle='#80868b';cx.lineWidth=1*L;poly(c.poly,true);cx.stroke();const m=c.poly.reduce((a,p)=>[a[0]+p[0]/4,a[1]+p[1]/4],[0,0]);const[a,b]=W2S(m[0],m[1]);cx.fillStyle='#80868b';cx.fillText(c.n,a,b)});
  cx.font=(11*L)+'px sans-serif';T.lines.forEach(l=>{cx.strokeStyle='#ffffff';cx.lineWidth=3*L;poly([l.a,l.b]);cx.stroke();const[a,b]=W2S(l.a[0],l.a[1]);cx.fillStyle='#bdc1c6';cx.fillText(l.name,a+4,b-4)});
  T.points.forEach(p=>{const[a,b]=W2S(p.xy[0],p.xy[1]);cx.fillStyle='#8ab4f8';cx.beginPath();cx.arc(a,b,3*L,0,7);cx.fill();cx.fillText(p.name,a+4,b+12)})}
 const P=S&&S.pose;if(P){if(follow){view.x=P.x;view.y=P.y;if(view.k<0.2)view.k=0.25}
  const c=Math.cos(P.psi),s=Math.sin(P.psi);const fx=P.x,fy=P.y;   // 앞바퀴
  if(S.scan){cx.fillStyle='#f28b82';S.scan.forEach(q=>{const[a,b]=W2S(fx+q[0]*c-q[1]*s,fy+q[0]*s+q[1]*c);cx.fillRect(a-1.5*L,b-1.5*L,3*L,3*L)})}
  const body=[[-266,-110],[56,-110],[56,110],[-266,110]].map(q=>[fx+q[0]*c-q[1]*s,fy+q[0]*s+q[1]*c]);
  cx.fillStyle=P.ok?'rgba(52,168,83,.75)':'rgba(251,188,4,.75)';poly(body,true);cx.fill();
  cx.strokeStyle='#fff';cx.lineWidth=2*L;poly([[fx,fy],[fx+300*c,fy+300*s]]);cx.stroke()}
 requestAnimationFrame(draw)}
const kv=(id,o)=>document.getElementById(id).innerHTML=Object.entries(o).map(([k,v])=>`<div>${k}</div><div>${v??'-'}</div>`).join('');
const j=v=>v==null?'-':(typeof v=='object'?JSON.stringify(v):v);
async function poll(){try{const r=await fetch('/state');S=await r.json();const d=S.data,m=d.mission||{},rt=m.route||{},ln=d.lane||{},a=S.age;
 document.getElementById('st').textContent=(m.state||'-')+(m.sub?' / '+m.sub:'')+(m.estop?'  [ESTOP]':'');
 document.getElementById('note').textContent=(m.source?'제어: '+m.source+' · ':'')+(m.note||'');
 const old=k=>a[k]==null?'<span class=stale>없음</span>':(a[k]>1.5?`<span class=stale>${a[k]}초 전</span>`:'');
 kv('car',{'도면 위치 s':rt.s_mm!=null?rt.s_mm+' mm '+old('mission'):'-','다음 선':rt.next?`${rt.next} (${rt.next_in_mm} mm)`:'-','마지막 맞춤':j(rt.last_fix),
  '자세':S.pose?(S.pose.ok?'차선 기준':'<span class=fb>경로 방향(추정)</span>'):'-','주행 거리':m.dist_mm,'명령 v / w':S.cmd?S.cmd.join(' / '):'-',
  '차선':`${ln.status||'-'} ${old('lane')}`,'차선 오프셋':ln.offset_mm,'흰 선(mm)':j(ln.stoplines_mm),'라이다':S.scan?S.scan.length+'점 '+old('scan'):'<span class=stale>없음</span>'});
 const cm=d.commands||{};kv('perc',{'YOLO 확인':j(cm.confirmed_classes)+' '+old('commands'),'명령':(cm.commands||[]).map(c=>c.command+(c.route?'('+c.route+')':'')).join(', ')||'-',
  '이번 프레임':j(cm.observed_classes),'갈림길':m.fork_choice||'-','주차 표지':m.parking_sign?'봄':'-','주차 무장':m.parking_armed?'예':'-'});
 const pk=d.parking||{},ob=d.obstacle||{},fk=d.fork||{};kv('nodes',{'주차':`${pk.phase||'-'} ${pk.row||''} ${pk.stop_how||''} ${pk.result||''}`,
  'M7':j({active:ob.m7_active,done:ob.m7_done,trigger:ob.m7_trigger,exit:ob.m7_exit}),'라바콘':j({ahead:ob.cone_any_ahead_mm,done:ob.cones_done}),'회피 노드':(typeof d.avoid=='string'?d.avoid:'-')+' '+old('avoid'),'갈림길 노드':j(fk.decision||fk.direction||fk)});
 document.getElementById('ev').innerHTML=S.events.slice().reverse().map(e=>`<div class="ev ${/FALLBACK/.test(e.note)?'fb':''}">${e.t} <b>${e.state}</b> s=${e.s??'-'}<br>${e.note||''}</div>`).join('');
 if(S.has_img){const im=document.getElementById('lane');im.style.display='block';document.getElementById('noimg').style.display='none';im.src='/lane.jpg?'+Date.now()}}catch(e){document.getElementById('st').textContent='연결 끊김'}setTimeout(poll,150)}
fetch('/track').then(r=>r.json()).then(t=>{T=t;fit()});upd();poll();draw();
</script></body></html>"""


def main(args=None):
    rclpy.init(args=args)
    node = DebugView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
