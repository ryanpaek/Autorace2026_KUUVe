"""맵 전체 디버깅 화면 (웹). 리모에서 돌리고 노트북 브라우저로 본다: http://<리모IP>:8080

추가 설치 없음 (파이썬 표준 http.server). 보여 주는 것:
  - 트랙 도면(경로 중심선, 차로 근사, 흰 선, 구간 점, 주차칸)과 그 위의 차 위치(도면 위치 s + 좌우 d + 방향)
  - 라이다 점(차 위치 기준으로 도면에 겹침): 콘·정차 리모·벽이 도면과 맞는지
  - 미션 상태/단계/이유(note), 예비 경로(FALLBACK) 기록, 속도 명령, 차선 상태, YOLO 확인 명령,
    주차·M7 회피·갈림길·라바콘 상태, 차선 노드 디버그 영상(/lane/debug/compressed)
차 위치는 mission_node의 /route/preview (s, d_mm, psi). 차선을 오래 놓치면 pose_ok=false -> 경로 방향으로 그림.
위치 확인 (11/11): 추정 위치 궤적(초록 전진·빨강 후진)과 흰 선 보정 지점(오차 mm)을 도면 위에 남긴다.
  manual_drive:=true로 띄우고 teleop으로 몰면서 궤적이 실제 달린 길과 맞는지 본다. '궤적 지우기'로 초기화.
센서 융합·판단 설명 (11/11):
  - 실제 카메라 영상 위에 YOLO 박스, 라이다 점(바닥 닿는 곳), 카메라가 본 차로 가운데, 도면이 예상하는 차로 가운데,
    흰 가로선, 자율주행이 낼 진행 궤적을 겹쳐 그린다. 라이다 -> 영상은 차선 노드 BEV 호모그래피를 거꾸로 쓴다
    (cone_core.GroundProjector와 같은 방식, 바닥 평면 가정이라 높이 있는 물체는 바닥 닿는 곳에 찍힘)
  - YOLO 박스 방향에 있는 라이다 점으로 그 물체까지 거리를 붙인다 (카메라 + 라이다 융합)
  - 위에서 본 라이다 그림 (차 기준): 차로 폭 안 가장 가까운 점, YOLO 방향, 진행 궤적
  - '지금 판단': 상태·제어 주체·바뀐 이유·다음 단계로 넘어가는 조건과 지금 값
  그림은 전부 노트북 브라우저가 그린다. 리모는 받은 JPEG를 그대로 넘기기만 해서 CPU를 거의 안 쓴다.
미션 조작 (11/11): 미션 고르기·건너뛰기·출발·비상 정지 버튼 (debug_control.py). 그 밖의 제어 명령(/cmd_vel 등)은 내지 않는다.
  끄기: debug_control:=false (대회 때). 그래도 비상 정지는 받는다. 비밀번호: control_password
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
from std_msgs.msg import Bool, Empty, String

from autorace_mission.debug_control import CHOOSABLE, control_request
from autorace_mission.explain import explain
from autorace_mission.route import RouteMap

JSON_TOPICS = {
    'mission': '/mission/state', 'preview': '/route/preview', 'lane': '/lane/status',
    'commands': '/perception/commands', 'detections': '/perception/detections',
    'parking': '/parking/state', 'obstacle': '/obstacle/state', 'fork': '/fork/state',
}
LANE_HALF = 250.0


def track_geometry(route: RouteMap) -> dict:
    """화면에 그릴 도면: 중심선·양쪽 선(차로 폭 500 근사)·흰 선·구간 점·주차칸. 단위 mm, 도면 좌표."""
    center, left, right = [], [[]], [[]]
    sp = route.three['spacing'] if route.three else 0.0
    s = 0.0
    while s <= route.length:
        x, y = route.xy(s)
        h = route.heading(s)
        center.append((round(x), round(y)))
        half = LANE_HALF + sp * route.widen(s)       # 3차선이면 바깥 선이 차로 하나만큼 더 바깥 (taper 포함)
        left[-1].append((round(x - half * math.sin(h)), round(y + half * math.cos(h))))
        right[-1].append((round(x + half * math.sin(h)), round(y - half * math.cos(h))))
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
    # 3차선 점선 (다 벌어진 곳 ~ 섬 앞 끝), 섬, 갈림길 A/B 차로 중심 (11/11, 도면 2026Track.png)
    lanes3, island, forks = [], [], []
    t = route.three
    if t:
        def off_pt(s, off):
            x, y = route.xy(s)
            h = route.heading(s)
            return (round(x - off * math.sin(h)), round(y + off * math.cos(h)))
        for off in (sp / 2.0, -sp / 2.0):
            line, s = [], t['in'][1]
            while s <= t['island'][0]:
                line.append(off_pt(s, off))
                s += 50.0
            lanes3.append({'kind': 'dash', 'pts': line})
        island = [(round(x), round(y)) for x, y in t['island_xy']]
        for name, sign in (('A', 1.0), ('B', -1.0)):     # 진행 방향 왼쪽(+) = A
            line, s = [], t['island'][0] - 1500.0
            while s <= t['out'][1]:
                line.append(off_pt(s, sign * sp * route.widen(s)))
                s += 50.0
            mid = (t['island'][0] + t['island'][1]) / 2.0
            forks.append({'name': name, 'pts': line, 'label': off_pt(mid, sign * sp)})
    return {'center': center, 'left': left, 'right': right, 'lines': lines, 'points': pts, 'cells': cells,
            'lanes3': lanes3, 'island': island, 'forks': forks, 'length': round(route.length)}


class DebugView(Node):
    def __init__(self):
        super().__init__('debug_view')
        self.declare_parameter('port', 8080)
        self.declare_parameter('route_file', '')
        self.declare_parameter('front_axle_from_lidar_mm', -47.0)   # parking_node와 같은 값
        self.declare_parameter('laser_yaw', 0.0)
        # 센서 융합 화면 (11/11)
        self.declare_parameter('camera_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('camera_max_hz', 8.0)       # 브라우저로 넘길 최대 프레임 (와이파이 부담)
        self.declare_parameter('bev_config', '')            # 차선 노드 bev_limo.yaml (라이다 -> 영상 투영)
        # 미션 조작 버튼 (11/11). false면 비상 정지만. 비밀번호를 주면 버튼 누를 때 물어봄 (같은 와이파이 다른 사람 막기)
        self.declare_parameter('control_enabled', True)
        self.declare_parameter('control_password', '')
        self.ctl_pub = {'/mission/start': self.create_publisher(Empty, '/mission/start', 10),
                        '/mission/set_state': self.create_publisher(String, '/mission/set_state', 10),
                        '/mission/skip': self.create_publisher(String, '/mission/skip', 10),
                        '/mission/estop': self.create_publisher(Bool, '/mission/estop', 10)}
        self.declare_parameter('front_axle_mm', 200.0)      # 뒷차축(차선 노드 기준점) -> 앞차축
        self.cam = None
        self._cam_t = 0.0
        self._state_since = time.time()
        self.calib = None
        bev_path = self.get_parameter('bev_config').value
        if bev_path and os.path.exists(bev_path):
            try:
                import yaml
                with open(bev_path, encoding='utf-8') as f:
                    b = yaml.safe_load(f)['bev']
                self.calib = {k: b.get(k) for k in ('image_width', 'image_height', 'bev_h', 'mm_per_px_x',
                                                     'mm_per_px_y', 'vehicle_col', 'ref_to_bev_bottom_mm',
                                                     'homography', 'calibrated')}
            except Exception as e:      # 화면용이라 실패해도 나머지는 동작
                self.get_logger().warning(f'bev_config read failed ({bev_path}): {e}')
        self.lock = threading.Lock()
        self.data = {k: None for k in JSON_TOPICS}
        self.stamp = {}
        self.scan = None
        self.cmd = None
        self.jpeg = None
        self.events = []
        self._last_state = None
        self.trail = []          # [x, y, dir] 추정 위치 궤적 (mm, 도면)
        self.fixes = []          # 흰 선 보정 지점 {x, y, name, err}
        self._last_fixes = None
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
        self.create_subscription(CompressedImage, self.get_parameter('camera_topic').value,
                                 self._on_cam, qos_profile_sensor_data)
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

            def do_POST(self):
                if self.path != '/control':
                    self.send_response(404)
                    self.end_headers()
                    return
                try:
                    n = int(self.headers.get('Content-Length') or 0)
                    body = json.loads(self.rfile.read(min(n, 4096)) or b'{}')
                except ValueError:
                    body = None
                ok, text = node.control(body)
                self.send_response(200 if ok else 403)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'ok': ok, 'msg': text}, ensure_ascii=False).encode())

            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    self._send(PAGE.encode(), 'text/html; charset=utf-8')
                elif self.path == '/track':
                    self._send(json.dumps(node.track).encode(), 'application/json')
                elif self.path.startswith('/state'):
                    self._send(json.dumps(node.snapshot()).encode(), 'application/json')
                elif self.path.startswith('/trail'):
                    with node.lock:
                        body = json.dumps({'trail': node.trail, 'fixes': node.fixes})
                    self._send(body.encode(), 'application/json')
                elif self.path.startswith('/reset_trail'):
                    with node.lock:
                        node.trail, node.fixes = [], []
                    self._send(b'{}', 'application/json')
                elif self.path.startswith('/calib'):
                    lx = -float(node.get_parameter('front_axle_from_lidar_mm').value)
                    body = {'bev': node.calib, 'front_axle_mm': float(node.get_parameter('front_axle_mm').value),
                            'lidar_fwd_of_front_axle_mm': lx}
                    self._send(json.dumps(body).encode(), 'application/json')
                elif self.path.startswith('/cam.jpg') and node.cam:
                    with node.lock:
                        img = node.cam
                    self._send(img, 'image/jpeg')
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
            if key == 'preview' and isinstance(d, dict):
                pose = self._pose(d)
                if pose is not None:
                    last = self.trail[-1] if self.trail else None
                    if last is None or math.hypot(pose['x'] - last[0], pose['y'] - last[1]) > 30.0:
                        self.trail.append([round(pose['x']), round(pose['y']), int(d.get('dir', 1))])
                        self.trail = self.trail[-4000:]
            if key == 'mission' and isinstance(d, dict):
                rt = d.get('route') or {}
                nf = rt.get('fixes')
                if nf is not None and self._last_fixes is not None and nf > self._last_fixes:
                    pose = self._pose(self.data.get('preview'))
                    lf = rt.get('last_fix') or {}
                    if pose is not None and (not self.fixes or self.fixes[-1]['name'] != lf.get('name')):
                        self.fixes.append({'x': round(pose['x']), 'y': round(pose['y']),
                                           'name': lf.get('name'), 'err': lf.get('err_mm')})
                        self.fixes = self.fixes[-200:]
                if nf is not None:
                    self._last_fixes = nf
                st = (d.get('state'), d.get('sub'))
                if st != self._last_state:
                    self._state_since = time.time()
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

    def _on_cam(self, m):
        now = time.time()
        hz = float(self.get_parameter('camera_max_hz').value)
        if hz > 0 and now - self._cam_t < 1.0 / hz:
            return
        if 'jpeg' not in (m.format or '').lower() and 'jpg' not in (m.format or '').lower():
            return                       # png 등은 브라우저 부담이 커서 안 넘김
        with self.lock:
            self.cam = bytes(m.data)
            self._cam_t = now
            self.stamp['camera'] = now

    def _on_img(self, m):
        with self.lock:
            self.jpeg = bytes(m.data)

    def control(self, body):
        with self.lock:
            cur = (self.data.get('mission') or {}).get('state')
        ok, text, pubs = control_request(body, bool(self.get_parameter('control_enabled').value),
                                         str(self.get_parameter('control_password').value), cur)
        for topic, val in pubs:
            if val is None:
                msg = Empty()
            elif isinstance(val, bool):
                msg = Bool(data=val)
            else:
                msg = String(data=val)
            self.ctl_pub[topic].publish(msg)
        log = self.get_logger().warning if ok else self.get_logger().info
        log(f'[debug 조작] {text}' + ('' if ok else ' (거절)'))
        with self.lock:
            self.events.append({'t': time.strftime('%H:%M:%S'), 'state': '조작' if ok else '조작 거절',
                                's': None, 'note': text})
            self.events = self.events[-40:]
        return ok, text

    def _pose(self, pv):
        if self.route is None or not isinstance(pv, dict) or pv.get('s') is None:
            return None
        s = float(pv['s'])
        x, y = self.route.xy(s)
        h = self.route.heading(s)
        # 11/11: 차선으로 최근에 못 맞췄어도 오도메트리로 적분한 d·방향을 그린다 (예전엔 가운데에 그림)
        d = float(pv.get('d_mm') or 0.0)
        psi = float(pv['psi']) if pv.get('psi') is not None else h
        return {'x': x - d * math.sin(h), 'y': y + d * math.cos(h), 'psi': psi, 's': s,
                'ok': bool(pv.get('pose_ok')), 'd': round(d), 'dpsi': pv.get('dpsi'),
                'dir': pv.get('dir', 1), 'rev_src': pv.get('rev_src', ''),
                'has_d': pv.get('d_mm') is not None, 'lane_fix': pv.get('lane_fix', ''),
                'pts': pv.get('pts')}

    def snapshot(self):
        with self.lock:
            now = time.time()
            age = {k: round(now - v, 1) for k, v in self.stamp.items()}
            pose = self._pose(self.data.get('preview'))
            why = None
            m = self.data.get('mission')
            if isinstance(m, dict):
                try:
                    why = explain(m, self.data.get('lane'), self.data.get('commands'),
                                  self.data.get('obstacle'), self.data.get('parking'),
                                  self.route.points if self.route else None,
                                  sub_age=now - self._state_since, cmd=self.cmd,
                                  stale={k: age.get(k) for k in ('lane', 'commands', 'scan', 'camera')
                                         if k in age})
                except Exception as e:   # 설명이 틀려도 화면은 살아 있게
                    why = {'state': '', 'title': f'설명 오류: {e}', 'control': '', 'next': [],
                           'because': [], 'warn': []}
            ctl = {'enabled': bool(self.get_parameter('control_enabled').value),
                   'password': bool(self.get_parameter('control_password').value), 'states': CHOOSABLE}
            return {'why': why, 'control': ctl, 'data': self.data, 'age': age, 'scan': self.scan, 'cmd': self.cmd, 'pose': pose,
                    'events': self.events, 'has_img': self.jpeg is not None, 'has_cam': self.cam is not None}


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoRace 디버그</title><style>
:root{--bg:#111418;--panel:#1b2026;--fg:#e8eaed;--mute:#9aa0a6;--ok:#34a853;--warn:#fbbc04;--bad:#ea4335;--acc:#8ab4f8}
body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.4 system-ui,"Malgun Gothic",sans-serif;display:flex;height:100vh}
#left{flex:1;min-width:0;display:flex;flex-direction:column}
#sensors{height:46%;display:flex;gap:4px;border-bottom:1px solid #2a2f36}#sensors.off{display:none}
#camw{flex:1.55;position:relative;min-width:0}#lidw{flex:1;position:relative;min-width:0}
#map{flex:1;min-height:0;position:relative}canvas{width:100%;height:100%;display:block}
.cap{position:absolute;left:6px;bottom:4px;color:var(--mute);font-size:11px;pointer-events:none}
#side{width:400px;overflow:auto;background:var(--panel);padding:10px;box-sizing:border-box}
h3{margin:10px 0 4px;font-size:13px;color:var(--acc)}.big{font-size:20px;font-weight:700}
.kv{display:grid;grid-template-columns:110px 1fr;gap:1px 6px}.kv div:nth-child(odd){color:var(--mute)}
.ev{font-size:12px;border-bottom:1px solid #2a2f36;padding:2px 0}.fb{color:var(--warn)}
#tools{position:absolute;top:8px;left:8px;display:flex;gap:6px;flex-wrap:wrap}button{background:#2a2f36;color:var(--fg);border:1px solid #3c4043;border-radius:4px;padding:4px 8px;cursor:pointer}
img{width:100%;border-radius:4px;margin-top:4px}.stale{color:var(--bad)}
#why{background:#11161c;border:1px solid #2a2f36;border-radius:6px;padding:8px;margin-bottom:6px}
#why .t{font-size:15px;font-weight:700;margin:2px 0}#why .c{color:var(--mute);font-size:12px}
#why ul{margin:4px 0 0 0;padding-left:16px}#why li{margin:1px 0}.wr{color:var(--bad)}
.cond{display:grid;grid-template-columns:16px 1fr auto;gap:4px;align-items:center;margin:3px 0;font-size:12px}
.bar{grid-column:2/4;height:4px;background:#2a2f36;border-radius:2px;overflow:hidden}.bar i{display:block;height:100%;background:var(--acc)}
.met{color:var(--ok)}
#ctl{background:#11161c;border:1px solid #2a2f36;border-radius:6px;padding:8px;margin-bottom:6px}
#ctl select{background:#2a2f36;color:var(--fg);border:1px solid #3c4043;border-radius:4px;padding:4px;width:100%;margin:4px 0}
#ctl .row{display:flex;gap:4px;flex-wrap:wrap;margin-top:4px}#ctl .row button{flex:1}
#estop{background:#b3261e;border-color:#ea4335;font-weight:700;font-size:15px;padding:8px}#estopoff{flex:0.6}
#ctlmsg{font-size:12px;color:var(--mute);margin-top:4px;min-height:16px}
#modal{position:fixed;inset:0;background:rgba(0,0,0,.55);display:none;align-items:center;justify-content:center;z-index:9}
#modal .box{background:var(--panel);border:1px solid #3c4043;border-radius:8px;padding:16px;max-width:380px}
#modal .box b{font-size:15px}#modal .row{display:flex;gap:8px;margin-top:12px;justify-content:flex-end}
</style></head><body><div id="left">
<div id="sensors"><div id="camw"><canvas id="cam"></canvas><div class="cap" id="camcap">카메라 + 라이다 + YOLO + 선 (영상 없음)</div></div>
<div id="lidw"><canvas id="lid"></canvas><div class="cap">라이다 위에서 (차 기준, 앞 = 위, 원 0.5 m)</div></div></div>
<div id="map"><canvas id="c"></canvas><div id="tools">
<button id="follow">차 따라가기: 켬</button><button id="fit">전체 보기</button><button id="clr">궤적 지우기</button>
<button id="sens">센서 화면: 켬</button><button id="snd">🔔 소리: 끔</button>
<span id="hint" style="color:var(--mute);align-self:center">휠: 확대 · 끌기: 이동</span></div></div></div>
<div id="side"><div id="why"><div class="big" id="st">연결 중…</div></div>
<div id="ctl"><div style="display:flex;justify-content:space-between"><b>미션 조작</b><span id="ctlstate" style="color:var(--mute);font-size:12px"></span></div>
<select id="ctlsel"></select>
<div class="row"><button id="b_set" title="차를 그 미션 시작 위치(실행 가이드 3번 표)에 놓았을 때. 도면 위치도 그 시작점으로">시작 위치에서 시작</button>
<button id="b_skip" title="지금 자리에서 그 미션으로. 도면 위치는 그대로">지금 자리에서 이 미션</button></div>
<div class="row"><button id="b_next">다음 미션 ▶</button><button id="b_start">출발</button></div>
<div class="row"><button id="estop">■ 비상 정지 (Space)</button><button id="estopoff">해제</button></div>
<div id="ctlmsg"></div></div>
<h3>차</h3><div class="kv" id="car"></div>
<h3>인식</h3><div class="kv" id="perc"></div>
<h3>미션 노드</h3><div class="kv" id="nodes"></div>
<h3>차선 영상 (BEV)</h3><img id="lane" style="display:none"><div id="noimg" style="color:var(--mute)">(차선 디버그 영상 없음)</div>
<h3>상태 기록</h3><div id="ev"></div>
<div style="color:var(--mute);font-size:11px;margin-top:8px">카메라 위 그림 — 점: 라이다(바닥 닿는 곳, 빨강 가까움) · 초록 실선: 명령 궤적 · 노랑: 카메라 차로 가운데 · 하늘 점선: 도면 예상 차로 가운데 · 분홍: 흰 가로선 · 상자: YOLO(실선 = 확정)</div></div>
<div id="modal"><div class="box"><b id="mt"></b><div id="mb" style="margin-top:6px;color:var(--mute)"></div>
<input id="mpw" type="password" placeholder="비밀번호" style="display:none;margin-top:8px;width:100%;background:#2a2f36;color:var(--fg);border:1px solid #3c4043;border-radius:4px;padding:4px">
<div class="row"><button id="mno">취소</button><button id="myes" style="background:#1a73e8;border-color:#1a73e8">보내기</button></div></div></div>
<script>
const $=id=>document.getElementById(id),DPR=devicePixelRatio;
const cv=$('c'),cx=cv.getContext('2d'),cc=$('cam'),ccx=cc.getContext('2d'),lc=$('lid'),lcx=lc.getContext('2d');
let T=null,S=null,C=null,HI=null,TR={trail:[],fixes:[]},view={x:5000,y:3000,k:0.08},follow=true,drag=null,camImg=null,camBusy=false,camT=0;
function rs(){[cv,cc,lc].forEach(c=>{c.width=c.clientWidth*DPR;c.height=c.clientHeight*DPR})}addEventListener('resize',rs);rs();
const W2S=(x,y)=>[cv.width/2+(x-view.x)*view.k*DPR,cv.height/2-(y-view.y)*view.k*DPR];
function fit(){view={x:5000,y:3000,k:Math.min(cv.clientWidth/10800,cv.clientHeight/6800)}}
cv.onwheel=e=>{e.preventDefault();view.k*=e.deltaY<0?1.15:1/1.15};
cv.onmousedown=e=>{drag=[e.clientX,e.clientY];follow=false;upd()};onmouseup=()=>drag=null;
onmousemove=e=>{if(!drag)return;view.x-=(e.clientX-drag[0])/view.k;view.y+=(e.clientY-drag[1])/view.k;drag=[e.clientX,e.clientY]};
$('follow').onclick=()=>{follow=!follow;upd()};$('fit').onclick=()=>{follow=false;fit();upd()};
$('clr').onclick=()=>fetch('/reset_trail').then(()=>TR={trail:[],fixes:[]});
$('sens').onclick=()=>{$('sensors').classList.toggle('off');$('sens').textContent='센서 화면: '+($('sensors').classList.contains('off')?'끔':'켬');setTimeout(rs,0)};
function upd(){$('follow').textContent='차 따라가기: '+(follow?'켬':'끔')}
async function pollTrail(){try{TR=await (await fetch('/trail')).json()}catch(e){}setTimeout(pollTrail,1000)}

// ---- 주차 완료 소리 (원본 합성음: 두 음 '띠링'). 브라우저는 클릭 뒤에만 소리를 낼 수 있다 ----
let AC=null;
function chime(){if(!AC)return;const t=AC.currentTime;
 [[1046.5,0],[1568.0,0.13]].forEach(([f,d])=>{[1,2].forEach(h=>{const o=AC.createOscillator(),g=AC.createGain();o.type='sine';o.frequency.value=f*h;
  const a=h==1?0.28:0.06;g.gain.setValueAtTime(0,t+d);g.gain.linearRampToValueAtTime(a,t+d+0.012);g.gain.exponentialRampToValueAtTime(0.0008,t+d+0.9);
  o.connect(g).connect(AC.destination);o.start(t+d);o.stop(t+d+0.95)})})}
$('snd').onclick=()=>{if(!AC){AC=new (window.AudioContext||window.webkitAudioContext)();$('snd').textContent='🔔 소리: 켬';chime()}else{chime()}};
let prevState=null;
function soundOnState(m){const st=m.state;if(prevState&&st!==prevState&&st==='FINISHED'&&prevState==='M4_PARKING')chime();prevState=st}

// ---- 바닥(차 기준 mm: xf 앞 = 뒷바퀴 축 기준, yl 왼쪽 +) <-> 카메라 영상 px (차선 노드 BEV 호모그래피) ----
function inv3(h){const[a,b,c,d,e,f,g,k,i]=h,A=e*i-f*k,B=-(d*i-f*g),Cc=d*k-e*g,det=a*A+b*B+c*Cc;
 return[A,-(b*i-c*k),b*f-c*e,B,a*i-c*g,-(a*f-c*d),Cc,-(a*k-b*g),a*e-b*d].map(v=>v/det)}
function g2i(xf,yl){const B=C.bev;if(xf<350)return null;const col=B.vehicle_col+(-yl)/B.mm_per_px_x,row=(B.bev_h-1)-(xf-B.ref_to_bev_bottom_mm)/B.mm_per_px_y;
 const q0=HI[0]*col+HI[1]*row+HI[2],q1=HI[3]*col+HI[4]*row+HI[5],q2=HI[6]*col+HI[7]*row+HI[8];if(Math.abs(q2)<1e-12)return null;return[q0/q2,q1/q2]}
function i2g(u,v){const B=C.bev,h=B.homography,q0=h[0]*u+h[1]*v+h[2],q1=h[3]*u+h[4]*v+h[5],q2=h[6]*u+h[7]*v+h[8];if(Math.abs(q2)<1e-12)return null;
 const col=q0/q2,row=q1/q2;return[B.ref_to_bev_bottom_mm+((B.bev_h-1)-row)*B.mm_per_px_y,-(col-B.vehicle_col)*B.mm_per_px_x]}
fetch('/calib').then(r=>r.json()).then(c=>{if(c&&c.bev&&c.bev.homography){C=c;HI=inv3(c.bev.homography.map(Number))}}).catch(()=>{});
const FA=()=>C?C.front_axle_mm:200;
// 라이다 점: 서버는 앞바퀴 기준 (앞 +, 왼쪽 +) -> 뒷바퀴 축 기준 xf
const lidarPts=()=>(S&&S.scan&&S.age&&S.age.scan<1.5)?S.scan.map(q=>[q[0]+FA(),q[1]]):[];
const distCol=d=>d<500?'#ea4335':d<1000?'#fb8c00':d<1800?'#fbbc04':'#4fc3f7';
function cmdArc(){if(!S||!S.cmd)return[];const[v,w]=S.cmd;if(Math.abs(v)<0.01)return[];const k=w/v/1000,dir=Math.sign(v),out=[];
 for(let s=0;s<=1600;s+=50){const ss=dir*s;out.push(Math.abs(k)<1e-6?[FA()+ss,0]:[FA()+Math.sin(k*ss)/k,(1-Math.cos(k*ss))/k])}return out}
const ALIAS={traffic_red:'red',traffic_yellow:'yellow',traffic_green:'green','sign_left(A)':'fork_a','sign_right(B)':'fork_b',sign_pedestrian:'pedestrian',sign_bump:'bump',sign_slow:'slow',sign_parking:'parking'};
const canon=c=>ALIAS[c]||c;
function detsNow(){const d=S&&S.data.detections;if(!d||!S.age||S.age.detections>1.0)return[];return d.detections||[]}

function drawCam(){const W=cc.width,H=cc.height;ccx.fillStyle='#000';ccx.fillRect(0,0,W,H);if(!camImg||!camImg.naturalWidth)return;
 const nw=camImg.naturalWidth,nh=camImg.naturalHeight,sc=Math.min(W/nw,H/nh),ox=(W-nw*sc)/2,oy=(H-nh*sc)/2;ccx.drawImage(camImg,ox,oy,nw*sc,nh*sc);
 const L=DPR,P=(u,v)=>[ox+u*sc,oy+v*sc];
 const G=(xf,yl)=>{if(!HI)return null;const q=g2i(xf,yl);if(!q)return null;const B=C.bev,u=q[0]*nw/B.image_width,v=q[1]*nh/B.image_height;
  if(u<-nw||u>2*nw||v<-nh||v>2*nh)return null;return P(u,v)};
 const line=(pts,col,w,dash)=>{const p=pts.map(q=>G(q[0],q[1])).filter(Boolean);if(p.length<2)return;ccx.strokeStyle=col;ccx.lineWidth=w*L;ccx.setLineDash(dash||[]);
  ccx.beginPath();p.forEach((q,i)=>i?ccx.lineTo(q[0],q[1]):ccx.moveTo(q[0],q[1]));ccx.stroke();ccx.setLineDash([])};
 ccx.font=(12*L)+'px sans-serif';
 if(HI&&S){const ln=S.data.lane||{};
  // 흰 가로선 (차선 노드 거리: 앞바퀴 기준)
  (ln.stoplines_mm||[]).forEach(d=>{const xw=(ln.crosswalk_lines_mm||[]).some(c=>Math.abs(c-d)<1);line([[d+FA(),260],[d+FA(),-260]],xw?'#ff4081':'#f48fb1',3);
   const q=G(d+FA(),-260);if(q){ccx.fillStyle='#f48fb1';ccx.fillText((xw?'정지선 ':'흰 선 ')+Math.round(d)+'mm',q[0]+4,q[1])}});
  // 카메라 차로 가운데
  if(ln.offset_mm!=null&&ln.status&&ln.status!=='NONE'){const hd=ln.heading_rad||0,o=ln.offset_mm;line([550,800,1050,1300].map(x=>[x,o+Math.tan(hd)*(x-550)]),'#fdd663',2)}
  // 도면 예상 차로 가운데 (pts: 오른쪽 +, 뒷바퀴 축 기준 앞 mm)
  const pp=S.pose&&S.pose.pts;if(pp)line(pp.map(q=>[q[1],-q[0]]),'#4fc3f7',2,[6*L,5*L]);
  // 명령 궤적
  const arc=cmdArc();if(arc.length)line(arc,'#34a853',3);
  // 라이다 점 (바닥 닿는 곳)
  const lp=lidarPts();lp.forEach(q=>{if(q[0]<350||q[0]>3500||Math.abs(q[1])>1500)return;const p=G(q[0],q[1]);if(!p)return;
   ccx.fillStyle=distCol(q[0]-FA());ccx.fillRect(p[0]-2*L,p[1]-7*L,4*L,8*L)});
  // YOLO + 라이다 거리
  const conf=new Set((S.data.commands||{}).confirmed_classes||[]),vote=((S.data.commands||{}).vote||{}).counts||{};
  detsNow().forEach(d=>{const[x,y,w,h]=d.bbox||[];if(w==null)return;const a=P(x,y),b=P(x+w,y+h),B=C.bev;
   let best=null;lp.forEach(q=>{if(q[0]<350)return;const g=g2i(q[0],q[1]);if(!g)return;const u=g[0]*nw/B.image_width;if(u>=x&&u<=x+w){const r=Math.hypot(q[0]-FA(),q[1]);if(best==null||r<best)best=r}});
   const cl=canon(d.cls),ok=conf.has(cl);ccx.strokeStyle=ok?'#34a853':'#fbbc04';ccx.lineWidth=2*L;ccx.setLineDash(ok?[]:[5*L,4*L]);
   ccx.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);ccx.setLineDash([]);
   const txt=`${d.cls} ${(d.conf*100|0)}%`+(vote[cl]!=null?` 표 ${vote[cl]}`:'')+(best!=null?` · 라이다 ${(best/1000).toFixed(2)}m`:'');
   ccx.fillStyle='rgba(0,0,0,.65)';ccx.fillRect(a[0],a[1]-16*L,ccx.measureText(txt).width+8*L,16*L);ccx.fillStyle=ok?'#81c995':'#fdd663';ccx.fillText(txt,a[0]+4*L,a[1]-4*L)});
 }
 const w=S&&S.why;if(w){const t=`${w.state} — ${w.title}`;ccx.font='bold '+(14*L)+'px sans-serif';ccx.fillStyle='rgba(0,0,0,.6)';ccx.fillRect(0,0,ccx.measureText(t).width+14*L,22*L);ccx.fillStyle='#fff';ccx.fillText(t,7*L,16*L)}
}

function drawLid(){const W=lc.width,H=lc.height,L=DPR;lcx.fillStyle='#0b0e11';lcx.fillRect(0,0,W,H);
 const k=Math.min(W/3400,H/3900),ox=W/2,oy=H-900*k;   // 앞바퀴가 원점, 앞 3 m · 뒤 0.9 m
 const T2=(f,l)=>[ox-l*k,oy-f*k];   // f: 앞바퀴 기준 앞 mm, l: 왼쪽 mm
 lcx.strokeStyle='#2a2f36';lcx.lineWidth=1*L;for(let r=500;r<=3000;r+=500){lcx.beginPath();lcx.arc(ox,oy,r*k,0,7);lcx.stroke()}
 lcx.setLineDash([4*L,4*L]);lcx.strokeStyle='#3c4043';[-250,250].forEach(l=>{const a=T2(-900,l),b=T2(3000,l);lcx.beginPath();lcx.moveTo(...a);lcx.lineTo(...b);lcx.stroke()});lcx.setLineDash([]);
 const body=[[-266,-110],[56,-110],[56,110],[-266,110]].map(q=>T2(q[0],q[1]));lcx.fillStyle='rgba(138,180,248,.5)';lcx.beginPath();body.forEach((p,i)=>i?lcx.lineTo(...p):lcx.moveTo(...p));lcx.fill();
 if(!S)return;const arc=cmdArc();if(arc.length){lcx.strokeStyle='#34a853';lcx.lineWidth=3*L;lcx.beginPath();arc.forEach((q,i)=>{const p=T2(q[0]-FA(),q[1]);i?lcx.lineTo(...p):lcx.moveTo(...p)});lcx.stroke()}
 // YOLO 방향 (영상 열 -> 바닥 위 두 점을 잇는 선)
 if(HI&&camImg&&camImg.naturalWidth){const B=C.bev;detsNow().forEach(d=>{const[x,y,w,h]=d.bbox||[];if(w==null)return;const u=(x+w/2)*B.image_width/camImg.naturalWidth;
  const g1=i2g(u,B.image_height*0.98),g2=i2g(u,B.image_height*0.75);if(!g1||!g2)return;const dx=g2[0]-g1[0],dy=g2[1]-g1[1],n=Math.hypot(dx,dy)||1;
  const a=T2(g1[0]-FA(),g1[1]),b=T2(g1[0]-FA()+dx/n*3500,g1[1]+dy/n*3500);lcx.strokeStyle='rgba(253,214,99,.7)';lcx.setLineDash([6*L,4*L]);lcx.lineWidth=2*L;
  lcx.beginPath();lcx.moveTo(...a);lcx.lineTo(...b);lcx.stroke();lcx.setLineDash([]);lcx.fillStyle='#fdd663';lcx.font=(11*L)+'px sans-serif';lcx.fillText(d.cls,b[0]+3,b[1])})}
 let near=null;(S.scan&&S.age.scan<1.5?S.scan:[]).forEach(q=>{const p=T2(q[0],q[1]);lcx.fillStyle=distCol(Math.hypot(q[0],q[1]));lcx.fillRect(p[0]-1.5*L,p[1]-1.5*L,3*L,3*L);
  if(q[0]>0&&Math.abs(q[1])<250&&(near==null||q[0]<near[0]))near=q});
 lcx.font=(12*L)+'px sans-serif';if(near){const p=T2(near[0],near[1]);lcx.strokeStyle='#fff';lcx.lineWidth=2*L;lcx.beginPath();lcx.arc(p[0],p[1],6*L,0,7);lcx.stroke();
  lcx.fillStyle='#fff';lcx.fillText(`차로 안 가장 가까운 점 ${Math.round(near[0])}mm`,8*L,18*L)}else{lcx.fillStyle='#9aa0a6';lcx.fillText('차로 안 앞쪽 라이다 점 없음',8*L,18*L)}
 const ob=S.data.obstacle||{};if(ob.cone_ahead_mm!=null){lcx.fillStyle='#fb8c00';lcx.fillText(`콘(장애물 노드) ${Math.round(ob.cone_ahead_mm)}mm`,8*L,36*L)}
}

function poly(p,close){cx.beginPath();p.forEach((q,i)=>{const[a,b]=W2S(q[0],q[1]);i?cx.lineTo(a,b):cx.moveTo(a,b)});if(close)cx.closePath()}
function draw(){cx.clearRect(0,0,cv.width,cv.height);const L=DPR;const fc=S&&S.data.mission?S.data.mission.fork_choice:null;
 if(T){cx.lineWidth=2*L;T.left.forEach(g=>{cx.strokeStyle='#fbbc04';poly(g);cx.stroke()});T.right.forEach(g=>{cx.strokeStyle='#dadce0';poly(g);cx.stroke()});
  cx.setLineDash([6*L,6*L]);cx.strokeStyle='#5f6368';cx.lineWidth=1*L;poly(T.center);cx.stroke();cx.setLineDash([]);
  (T.lanes3||[]).forEach(l=>{cx.strokeStyle='#dadce0';cx.lineWidth=2*L;cx.setLineDash([8*L,8*L]);poly(l.pts);cx.stroke();cx.setLineDash([])});
  if(T.island&&T.island.length){cx.fillStyle='rgba(218,220,224,.15)';cx.strokeStyle='#dadce0';cx.lineWidth=2*L;poly(T.island,true);cx.fill();cx.stroke()}
  cx.font='bold '+(13*L)+'px sans-serif';(T.forks||[]).forEach(f=>{const on=fc===f.name;cx.strokeStyle=on?'#8ab4f8':'rgba(138,180,248,.35)';cx.lineWidth=(on?3:1.5)*L;
   cx.setLineDash([4*L,6*L]);poly(f.pts);cx.stroke();cx.setLineDash([]);const[a,b]=W2S(f.label[0],f.label[1]);cx.fillStyle=on?'#8ab4f8':'#9aa0a6';cx.fillText(f.name,a-4*L,b+4*L)});
  T.cells.forEach(c=>{cx.strokeStyle='#80868b';cx.lineWidth=1*L;poly(c.poly,true);cx.stroke();const m=c.poly.reduce((a,p)=>[a[0]+p[0]/4,a[1]+p[1]/4],[0,0]);const[a,b]=W2S(m[0],m[1]);cx.fillStyle='#80868b';cx.fillText(c.n,a,b)});
  cx.font=(11*L)+'px sans-serif';T.lines.forEach(l=>{cx.strokeStyle='#ffffff';cx.lineWidth=3*L;poly([l.a,l.b]);cx.stroke();const[a,b]=W2S(l.a[0],l.a[1]);cx.fillStyle='#bdc1c6';cx.fillText(l.name,a+4,b-4)});
  T.points.forEach(p=>{const[a,b]=W2S(p.xy[0],p.xy[1]);cx.fillStyle='#8ab4f8';cx.beginPath();cx.arc(a,b,3*L,0,7);cx.fill();cx.fillText(p.name,a+4,b+12)})}
 cx.lineWidth=2*L;for(let i=1;i<TR.trail.length;i++){const a=TR.trail[i-1],b=TR.trail[i];if(Math.hypot(b[0]-a[0],b[1]-a[1])>400)continue;
  cx.strokeStyle=b[2]<0?'rgba(234,67,53,.9)':'rgba(52,168,83,.8)';poly([a,b]);cx.stroke()}
 TR.fixes.forEach(f=>{const[a,b]=W2S(f.x,f.y);const e=Math.abs(f.err||0);cx.fillStyle=e<100?'#34a853':e<250?'#fbbc04':'#ea4335';
  cx.beginPath();cx.arc(a,b,4*L,0,7);cx.fill();cx.fillStyle='#e8eaed';cx.fillText((f.err>0?'+':'')+f.err,a+6,b+4)});
 const P=S&&S.pose;if(P){if(follow){view.x=P.x;view.y=P.y;if(view.k<0.2)view.k=0.25}
  const c=Math.cos(P.psi),s=Math.sin(P.psi);const fx=P.x,fy=P.y;   // 앞바퀴
  if(S.scan){cx.fillStyle='#f28b82';S.scan.forEach(q=>{const[a,b]=W2S(fx+q[0]*c-q[1]*s,fy+q[0]*s+q[1]*c);cx.fillRect(a-1.5*L,b-1.5*L,3*L,3*L)})}
  const body=[[-266,-110],[56,-110],[56,110],[-266,110]].map(q=>[fx+q[0]*c-q[1]*s,fy+q[0]*s+q[1]*c]);
  cx.fillStyle=P.ok?'rgba(52,168,83,.75)':'rgba(251,188,4,.75)';poly(body,true);cx.fill();
  cx.strokeStyle='#fff';cx.lineWidth=2*L;poly([[fx,fy],[fx+300*c,fy+300*s]]);cx.stroke()}
 if(!$('sensors').classList.contains('off')){drawCam();drawLid()}
 requestAnimationFrame(draw)}
function camLoop(){if(!camBusy&&S&&S.has_cam&&!$('sensors').classList.contains('off')&&Date.now()-camT>110){camBusy=true;camT=Date.now();const im=new Image();
  im.onload=()=>{camImg=im;camBusy=false;$('camcap').textContent='카메라 + 라이다 + YOLO + 선'+(HI?'':' (bev_config 없음: 겹치기 끔)')};im.onerror=()=>camBusy=false;im.src='/cam.jpg?'+camT}
 setTimeout(camLoop,40)}
const kv=(id,o)=>$(id).innerHTML=Object.entries(o).map(([k,v])=>`<div>${k}</div><div>${v??'-'}</div>`).join('');
const j=v=>v==null?'-':(typeof v=='object'?JSON.stringify(v):v);
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
function renderWhy(w,m){if(!w){$('why').innerHTML='<div class="big">'+(m.state||'-')+'</div>';return}
 $('why').innerHTML=`<div class="big">${m.manual?'[수동] ':''}${esc(w.state)}${m.estop?' [ESTOP]':''}</div><div class="t">${esc(w.title)}</div><div class="c">${esc(w.control)}</div>`+
  (w.warn.length?`<ul>${w.warn.map(x=>`<li class="wr">${esc(x)}</li>`).join('')}</ul>`:'')+
  (w.because.length?`<h3>왜</h3><ul>${w.because.map(x=>`<li>${esc(x)}</li>`).join('')}</ul>`:'')+
  (w.next.length?`<h3>다음 단계로 넘어가는 조건 (하나라도)</h3>`+w.next.map(n=>`<div class="cond"><span class="${n.met?'met':''}">${n.met?'✔':'○'}</span><span>${esc(n.text)}</span><span style="color:var(--mute)">${esc(n.value)}</span>`+
   (n.progress!=null?`<div class="bar"><i style="width:${n.progress*100}%"></i></div>`:'')+'</div>').join(''):'')}
async function poll(){try{const r=await fetch('/state');S=await r.json();const d=S.data,m=d.mission||{},rt=m.route||{},ln=d.lane||{},a=S.age;
 renderWhy(S.why,m);soundOnState(m);renderCtl();
 const old=k=>a[k]==null?'<span class=stale>없음</span>':(a[k]>1.5?`<span class=stale>${a[k]}초 전</span>`:'');
 kv('car',{'도면 위치 s':rt.s_mm!=null?rt.s_mm+' mm '+old('mission'):'-','다음 선':rt.next?`${rt.next} (${rt.next_in_mm} mm)`:'-','마지막 맞춤':j(rt.last_fix),
  '자세':S.pose?(S.pose.ok?'차선으로 확인':'<span class=fb>오도메트리로 추정</span>'):'-','차선 맞춤':S.pose?esc(S.pose.lane_fix||'-'):'-',
  '진행':S.pose?(S.pose.dir<0?`<span class=stale>후진</span> (${S.pose.rev_src})`:'전진')+(m.reverse_mm?` · 후진 누적 ${m.reverse_mm} mm`:''):'-',
  '좌우 d / 방향 차':S.pose?`${S.pose.d} mm / ${S.pose.dpsi==null?'-':(S.pose.dpsi*57.3).toFixed(1)+'°'}`:'-',
  '길 잃음':rt.lost?'<span class=stale>예 (FSM에 위치 안 줌)</span>':'아니오','주행 거리':m.dist_mm,'명령 v / w':S.cmd?S.cmd.join(' / '):'-',
  '차선':`${ln.status||'-'} ${old('lane')}`,'선 종류':(ln.line_left?(ln.line_left.color[0]+(ln.line_left.dashed?'점':'실')):'-')+' | '+(ln.line_right?(ln.line_right.color[0]+(ln.line_right.dashed?'점':'실')):'-')+(ln.lane_id!=null?` → 카메라 차로 ${({1:'1차선',0:'2차선','-1':'3차선'})[ln.lane_id]}`:''),'차선 오프셋':ln.offset_mm,'흰 선(mm)':j(ln.stoplines_mm),'라이다':S.scan?S.scan.length+'점 '+old('scan'):'<span class=stale>없음</span>','카메라':S.has_cam?'받는 중 '+old('camera'):'<span class=stale>없음</span>'});
 const cm=d.commands||{};kv('perc',{'YOLO 확인':j(cm.confirmed_classes)+' '+old('commands'),'명령':(cm.commands||[]).map(c=>c.command+(c.route?'('+c.route+')':'')).join(', ')||'-',
  '이번 프레임':j(cm.observed_classes),'투표':cm.vote?`${cm.vote.need}/${cm.vote.window} · `+j(Object.fromEntries(Object.entries(cm.vote.counts||{}).filter(([k,v])=>v>0))):'-','갈림길':m.fork_choice||'-','주차 표지':m.parking_sign?'봄':'-','주차 무장':m.parking_armed?'예':'-'});
 const pk=d.parking||{},ob=d.obstacle||{},fk=d.fork||{};kv('nodes',{'주차':`${pk.phase||'-'} ${pk.row||''} ${pk.stop_how||''} ${pk.result||''}`,
  'M7':j({active:ob.m7_active,done:ob.m7_done,trigger:ob.m7_trigger,exit:ob.m7_exit}),'라바콘':j({ahead:ob.cone_any_ahead_mm,done:ob.cones_done}),'회피 노드':(typeof d.avoid=='string'?d.avoid:'-')+' '+old('avoid'),'갈림길 노드':j(fk.decision||fk.direction||fk)});
 $('ev').innerHTML=S.events.slice().reverse().map(e=>`<div class="ev ${/FALLBACK/.test(e.note)?'fb':''}">${e.t} <b>${e.state}</b> s=${e.s??'-'}<br>${esc(e.note||'')}</div>`).join('');
 if(S.has_img){const im=$('lane');im.style.display='block';$('noimg').style.display='none';im.src='/lane.jpg?'+Date.now()}}catch(e){$('why').innerHTML='<div class="big">연결 끊김</div>'}setTimeout(poll,150)}
// ---- 미션 조작 ----
let PW='',ctlFilled=false;
async function sendCtl(body){if(PW)body.password=PW;try{const r=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 const j=await r.json();$('ctlmsg').innerHTML=(j.ok?'✔ ':'<span class=stale>✖ ')+esc(j.msg)+(j.ok?'':'</span>');if(!j.ok&&/비밀번호/.test(j.msg))PW=''}catch(e){$('ctlmsg').innerHTML='<span class=stale>✖ 보내기 실패 (연결)</span>'}}
function ask(title,body,fn){const c=S&&S.control;$('mt').textContent=title;$('mb').textContent=body;const pw=c&&c.password&&!PW;$('mpw').style.display=pw?'block':'none';$('mpw').value='';
 $('modal').style.display='flex';$('myes').focus();$('myes').onclick=()=>{if(pw)PW=$('mpw').value;$('modal').style.display='none';fn()};$('mno').onclick=()=>$('modal').style.display='none'}
const selName=()=>$('ctlsel').value;
$('b_set').onclick=()=>ask(`${selName()} 시작 위치에서 시작`,'보내는 순간 차가 움직여요. 차를 그 미션 시작 위치에 놓았나요? 도면 위치도 그 시작점으로 다시 맞춰요.',()=>sendCtl({action:'set_state',state:selName()}));
$('b_skip').onclick=()=>ask(`지금 자리에서 ${selName()}`,'미션만 바꾸고 도면 위치는 그대로예요. 차가 바로 그 미션대로 움직여요.',()=>sendCtl({action:'skip',state:selName()}));
$('b_next').onclick=()=>{const m=(S&&S.data.mission)||{};ask('다음 미션으로 건너뛰기',`${m.state||'?'} 를 끝난 것으로 보고 다음 미션으로 넘겨요. 도면 위치는 그대로예요.`,()=>sendCtl({action:'next'}))};
$('b_start').onclick=()=>ask('출발','출발 체커에 반듯이 놓았나요? 보내는 순간 출발해요.',()=>sendCtl({action:'start'}));
$('estop').onclick=()=>sendCtl({action:'estop'});
$('estopoff').onclick=()=>ask('비상 정지 해제','해제하면 지금 미션대로 바로 다시 움직여요.',()=>sendCtl({action:'estop_off'}));
addEventListener('keydown',e=>{if(e.code==='Space'&&!/INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)){e.preventDefault();sendCtl({action:'estop'})}
 if(e.code==='Escape')$('modal').style.display='none'});
function renderCtl(){const c=S&&S.control,m=(S&&S.data.mission)||{};if(!c)return;
 if(!ctlFilled){$('ctlsel').innerHTML=c.states.map(s=>`<option>${s}</option>`).join('');ctlFilled=true}
 ['b_set','b_skip','b_next','b_start','estopoff','ctlsel'].forEach(id=>$(id).disabled=!c.enabled);
 $('ctlstate').innerHTML=(c.enabled?'':'<span class=stale>꺼짐 · 비상 정지만</span> ')+(m.estop?'<span class=stale>ESTOP 켜짐</span>':'')}
fetch('/track').then(r=>r.json()).then(t=>{T=t;fit()});upd();poll();pollTrail();camLoop();draw();
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
