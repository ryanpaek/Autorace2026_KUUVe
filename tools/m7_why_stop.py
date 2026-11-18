#!/usr/bin/env python3
"""M7에서 차가 왜 서 있나 실시간으로 보기 (전체 launch가 떠 있는 상태에서 다른 터미널로).

  source ~/autorace_ws/install/setup.bash
  python3 ~/autorace_ws/tools/m7_why_stop.py            # 0.5초마다 한 줄, 멈춰 있으면 원인 추정
  python3 ~/autorace_ws/tools/m7_why_stop.py --all      # 움직일 때도 계속 출력

원인을 아래 순서로 본다 (위가 먼저 걸리면 그게 원인):
  A. 비상 정지 / FSM이 M7이 아님 / FSM 'stop' 명령
  B. 센서 끊김: /odom, /scan, /imu가 0.5초 넘게 안 옴 -> 회피 노드가 0을 냄
  C. obstacle_node(m7_link): 아직 안 켬(phase armed 전), 회피 노드 소식 끊김으로 잡아 둠(m7_hold)
  D. 회피 노드(/avoid/state): phase STOP(멈춰 있기 기다림), stop(차체 앞 물체 안전 정지), why(옆 차로 막힘 등),
     EXEC 중 구간 사이 쉼, 섬 앞 가운데 차로
  E. 차선 노드: M7 켜기 전(코너·직선)은 차선 주행 -> 선 놓침(status NONE/stop)이면 섬
  F. mission_node가 고른 출처(source)와 /cmd_vel이 0인데 출처 쪽 명령은 0이 아님 -> 상한·정지 조건
그리고 최근 로그(/rosout)에서 [M7 재생]·m7·M7 줄을 같이 보여 준다.
"""
import argparse
import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

try:
    from rcl_interfaces.msg import Log
except ImportError:
    Log = None


def _j(msg):
    try:
        return json.loads(msg.data)
    except (ValueError, TypeError):
        return {}


class Why(Node):
    def __init__(self, show_all):
        super().__init__('m7_why_stop')
        self.all = show_all
        self.t = {}
        self.mission, self.avoid, self.obst, self.lane = {}, {}, {}, {}
        self.cmd = self.ocmd = self.lcmd = None
        self.active = None
        self.odom_v = None
        self.logs = deque(maxlen=6)
        self.stopped_since = None
        self.last_line = ''
        sub = self.create_subscription
        sub(String, '/mission/state', lambda m: self._set('mission', _j(m)), 10)
        sub(String, '/avoid/state', lambda m: self._set('avoid', _j(m)), 10)
        sub(String, '/obstacle/state', lambda m: self._set('obst', _j(m)), 10)
        sub(String, '/lane/status', lambda m: self._set('lane', _j(m)), 10)
        sub(Bool, '/avoid/active', self._on_active, 10)
        sub(Twist, '/cmd_vel', lambda m: self._tw('cmd', m), 10)
        sub(Twist, '/obstacle/cmd_vel', lambda m: self._tw('ocmd', m), 10)
        sub(Twist, '/lane/cmd_vel', lambda m: self._tw('lcmd', m), 10)
        sub(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        sub(LaserScan, '/scan', lambda m: self._stamp('scan'), qos_profile_sensor_data)
        sub(Imu, '/imu', lambda m: self._stamp('imu'), qos_profile_sensor_data)
        if Log is not None:
            sub(Log, '/rosout', self._on_log, 50)
        self.create_timer(0.5, self.tick)

    def _stamp(self, k):
        self.t[k] = time.monotonic()

    def _set(self, k, v):
        setattr(self, k, v)
        self._stamp(k)

    def _tw(self, k, m):
        setattr(self, k, (m.linear.x, m.angular.z))
        self._stamp(k)

    def _on_active(self, m):
        self.active = m.data
        self._stamp('active')

    def _on_odom(self, m):
        self.odom_v = m.twist.twist.linear.x
        p = m.pose.pose.position
        now = time.monotonic()
        last = getattr(self, '_lastp', None)
        if last is not None and now - last[0] > 0.05:
            self.speed = math.hypot(p.x - last[1], p.y - last[2]) / (now - last[0])
            self._lastp = (now, p.x, p.y)
        elif last is None:
            self._lastp = (now, p.x, p.y)
        self._stamp('odom')

    def _on_log(self, m):
        txt = m.msg
        if any(k in txt for k in ('M7', 'm7', 'M2 재생', '회피', '정지', '막힘')) or m.name in ('avoid_node',):
            self.logs.append(f'{m.name}: {txt[:150]}')
            self._new_log = True

    def age(self, k):
        return time.monotonic() - self.t[k] if k in self.t else math.inf

    def reasons(self):
        out = []
        ms, av, ob, ln = self.mission, self.avoid, self.obst, self.lane
        st = ms.get('state')
        # A
        if ms.get('estop'):
            out.append('A 비상 정지(estop) 켜짐 -> /mission/estop false')
        if self.age('mission') > 1.0:
            out.append('A /mission/state 안 옴 (mission_node 죽음?)')
        elif st != 'M7_OBSTACLE':
            out.append(f'A FSM 상태가 {st} (M7 아님) sub={ms.get("sub")}')
        if ms.get('stop'):
            out.append(f'A FSM 정지 명령: stop={ms.get("stop")} note={ms.get("note")}')
        # B
        for k, lim in (('odom', 0.5), ('scan', 0.6), ('imu', 0.5)):
            if self.age(k) > lim:
                out.append(f'B /{k} {self.age(k):.1f}초째 안 옴 -> 회피 노드가 0을 냄 (센서 드라이버 확인)')
        # C
        if st == 'M7_OBSTACLE':
            ph = ob.get('m7_phase')
            if ph in (None, 'idle'):
                out.append(f'C m7_link 아직 회피 노드를 안 켬: pos={ob.get("m7_pos_mm")} mm (시작선 기준) '
                           f'front={ob.get("m7_front_mm")} (그동안은 차선 주행 -> E 확인)')
            elif ph == 'armed' and not ob.get('m7_active'):
                out.append('C 켜기는 보냈는데 회피 노드가 시작을 안 함 -> avoid_node 로그 "/odom 또는 /scan 없음" 확인')
            elif ph == 'aborted':
                out.append(f'C m7_link 중단(aborted): off_by={ob.get("m7_off_by")}')
            if ob.get('m7_hold'):
                out.append(f'C m7_link가 잡아 둠(m7_hold): kind={ob.get("m7_hold_kind")} -> 회피 노드 /avoid/active가 끊겼었음')
            if ph == 'running' and self.age('active') > 1.0:
                out.append('C /avoid/active 1초 넘게 안 옴 -> 회피 노드(avoid_node) 죽었거나 막힘')
        # D
        if av and self.age('avoid') < 1.0 and av.get('state') == 'RUN':
            phase = av.get('phase')
            if av.get('stop'):
                out.append(f'D 안전 정지: {av.get("stop")} (진행 방향 차체 앞/뒤에 라이다 점)')
            if phase == 'STOP':
                out.append('D 회피 노드 STOP: 완전히 멈출 때까지 기다리는 중 (odom이 계속 조금씩 움직이면 못 빠져나옴)')
            if phase == 'BACKUP':
                out.append(f'D 후진 중 (남은 {av.get("backup")} m)')
            why = av.get('why') or ''
            if any(k in why for k in ('막힘', '옮길 곳', '닿음')):
                out.append(f'D 판단: {why}')
            gaps = av.get('gaps_mm') or {}
            if gaps:
                out.append('   차로별 앞 장애물(mm, 라이다): ' + ', '.join(
                    f'{({"1": "1차선", "0": "2차선", "-1": "3차선"}).get(k, k)} {v}' for k, v in sorted(gaps.items(), reverse=True)))
            out.append(f'   회피 노드: phase={phase} 차로={av.get("now_lane_no", av.get("now_lane"))} x={av.get("x")} '
                       f'y={av.get("y")} 방향={av.get("psi_deg")}° v={av.get("v")} 조향출처={av.get("steer_src")} '
                       f'm2={av.get("m2")}')
        elif st == 'M7_OBSTACLE' and ob.get('m7_active'):
            out.append('D /avoid/state 안 옴 (회피 노드 상태 모름)')
        # E
        if ln:
            if ln.get('status') in (None, 'NONE', 'stop') or ln.get('low_conf'):
                out.append(f'E 차선: status={ln.get("status")} low_conf={ln.get("low_conf")} '
                           f'(M7 켜기 전 차선 주행이면 이것 때문에 설 수 있음)')
        # F
        src = ms.get('source')
        if self.cmd is not None and abs(self.cmd[0]) < 0.01:
            want = {'obstacle': self.ocmd, 'lane': self.lcmd}.get(src)
            if want is not None and abs(want[0]) > 0.02:
                out.append(f'F mission_node 출처 {src}는 {want[0]:.2f} m/s인데 /cmd_vel 0 -> mission_node 정지 조건(상한·안전)')
            else:
                out.append(f'F /cmd_vel 0, 출처={src} ({src} 명령도 0) -> 위 원인 중 그 출처 쪽')
                if src == 'obstacle' and av:
                    # 11/17 밤: 출처가 회피(재생) 노드면 그 노드가 왜 0을 내는지 바로 (phase/why/stop)
                    out.append(f'   -> 재생 노드: phase={av.get("phase")} why={av.get("why") or "-"} '
                               f'stop={av.get("stop") or "-"} 후진={av.get("backup")} 차로 {av.get("lane_no")}'
                               f' 카메라차로 {av.get("cam_lane_no")} 조향={av.get("steer")}')
        return out

    def tick(self):
        v = getattr(self, 'speed', None)
        moving = v is not None and v > 0.02
        now = time.monotonic()
        if moving:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        stopped_for = 0.0 if self.stopped_since is None else now - self.stopped_since
        if moving and not self.all:
            return
        ms = self.mission
        head = (f'[{time.strftime("%H:%M:%S")}] {"움직임" if moving else f"멈춤 {stopped_for:.1f}s"} '
                f'| FSM {ms.get("state")}/{ms.get("sub")} 출처={ms.get("source")} '
                f'| cmd={None if self.cmd is None else round(self.cmd[0], 2)} '
                f'obst_cmd={None if self.ocmd is None else round(self.ocmd[0], 2)} '
                f'lane_cmd={None if self.lcmd is None else round(self.lcmd[0], 2)} '
                f'| odom v={None if self.odom_v is None else round(self.odom_v, 3)}')
        lines = [head] + ['  ' + r for r in self.reasons()]
        if getattr(self, '_new_log', False):
            lines += ['  로그: ' + s for s in list(self.logs)[-3:]]
            self._new_log = False
        txt = '\n'.join(lines)
        if txt != self.last_line or stopped_for < 1.0 or int(stopped_for * 2) % 4 == 0:
            print(txt, flush=True)
            self.last_line = txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    n = Why(a.all)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
