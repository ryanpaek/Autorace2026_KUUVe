"""path_recorder.py로 기록한 길을 그대로 따라 주행.

  python3 path_follower.py m7_left.csv 0.2 0.023
  (인자: CSV, 속도 m/s, 끝점 이만큼 앞에서 정지 명령 m)

켜는 순간의 차 위치·방향 = 기록 출발점. 기록할 때와 같은 자리, 같은 방향에 놓고 켠다.
처음 약 2초는 출력 연결 확인 + 자이로 편향 측정 → 차를 건드리지 말 것.

위치: odom이 잰 거리 + IMU 방향 (기록과 같은 계산, path_common.PoseFusion)
조향 (--ctrl track, 기본):
  곡률 = 기록 경로 곡률(조금 앞) - k_e x 옆 오차 - k_psi x 방향 오차
  → 위치와 방향을 둘 다 맞추고, 기록 곡률을 미리 넣어서 늦게 꺾지 않음
  --ctrl pp 는 예전 pure pursuit (위치 목표점만)
전진·후진: 기록에 후진이 있으면 방향이 바뀌는 곳마다 구간을 나눠 그대로 재생한다.
  구간 끝에서 멈추고(--pause초 쉼) 다음 구간 방향으로 출발. 후진 속도는 --rev_speed
멈추는 경우: 마지막 구간 끝점 도착 / 옆 오차 30 cm 또는 방향 오차 50° 넘음 / 센서 부호 이상 / odom·imu 끊김
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from path_common import MAX_CURV, PoseFusion, load_segments, read_rows, wrap, yaw_of

WHEELBASE = 0.222        # m, 브리지 유효 축거 (ackermann 타입으로 직접 낼 때)


def pick_pose(csv_path, pose):
    if pose != 'auto':
        return pose
    rows = read_rows(csv_path)
    if rows and 'xs' in rows[0] and any(r.get('xs') for r in rows):
        return 'fused'
    print('[알림] 예전 기록 파일(xs 열 없음) → --pose odom으로 재생. 새 path_recorder.py로 다시 기록 권장')
    return 'odom'


class Follower(Node):
    def __init__(self, a):
        super().__init__('path_follower')
        self.a = a
        self.pose_mode = pick_pose(a.csv, a.pose)
        self.segs, info = load_segments(a.csv, self.pose_mode)
        self.seg_i = 0
        self.path = self.segs[0] if self.segs else None
        self.pause_until = None
        self.idx = 0
        self.finished = False
        self.origin = None
        self.fu = PoseFusion()
        self.last_odom = None
        self.last_print = 0.0
        self.checked = False
        self.ok_to_move = False
        self.t_begin = self.now()
        self.n_cmd = 0
        self.n_sat = 0
        self.max_e = 0.0
        self.s = 0.0
        self.e = 0.0
        self.epsi = 0.0
        if a.type == 'twist':
            self.msg_cls, self.type_name = Twist, 'geometry_msgs/msg/Twist'
        else:
            try:
                from ackermann_msgs.msg import AckermannDrive
            except ImportError:
                print('[오류] ackermann_msgs를 불러올 수 없음: --type twist로 쓰거나 패키지 확인')
                self.finished = True
                return
            self.msg_cls, self.type_name = AckermannDrive, 'ackermann_msgs/msg/AckermannDrive'
        self.pub = self.create_publisher(self.msg_cls, a.topic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.1, self.watchdog)
        if not self.segs or sum(len(g) for g in self.segs) < 10:
            print('[오류] 경로 점이 너무 적음: CSV가 짧거나 비었음')
            self.finished = True
            return
        kmax = max(abs(k) for g in self.segs for k in g.k)
        last = self.segs[-1]
        print(f'경로: 구간 {len(self.segs)}개 (' + ' → '.join(f'{"전진" if g.dir > 0 else "후진"} {g.length:.2f}m' for g in self.segs)
              + f'), 끝 위치 x={last.x[-1]:.3f} y={last.y[-1]:+.3f} 방향 {math.degrees(last.h[-1] - self.segs[0].h[0]):+.1f}°, '
              f'최대 곡률 {kmax:.2f} (차 한계 {MAX_CURV:.2f}), 위치 기준 {self.pose_mode}')
        if kmax > 0.9 * MAX_CURV:
            print('[경고] 기록 경로가 차가 돌 수 있는 한계에 가까움 → 조향이 끝까지 붙음. 더 완만하게 다시 기록 권장')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return t if t > 0 else self.now()

    def send(self, v, curv):
        m = self.msg_cls()
        if self.a.type == 'twist':
            m.linear.x = v
            m.angular.z = v * curv
        else:
            m.speed = v
            m.steering_angle = math.atan(WHEELBASE * curv)
        self.pub.publish(m)

    def finish(self, why=''):
        try:
            self.send(0.0, 0.0)
        except Exception:
            pass
        if why:
            print(why)
        if self.n_cmd:
            print(f'  요약: 최대 옆 오차 {self.max_e * 100:.1f} cm, 조향 한계에 붙은 비율 {100 * self.n_sat / self.n_cmd:.0f}%, '
                  f'구간 {self.seg_i + 1}/{len(self.segs)} 진행 {self.s:.2f}/{self.path.length:.2f} m')
        self.finished = True

    def watchdog(self):
        if self.finished:
            return
        t = self.now()
        if not self.checked and t - self.t_begin > 2.0:
            self.checked = True
            if self.count_subscribers(self.a.topic) == 0:
                self.finish(f'[오류] {self.a.topic}을 받는 노드가 없음: 드라이버/브리지 확인')
                return
            try:
                types = {i.topic_type for i in self.get_subscriptions_info_by_topic(self.a.topic)}
                if self.type_name not in types:
                    self.finish(f'[오류] {self.a.topic} 타입 불일치: 받는 쪽은 {sorted(types)}, 보내는 쪽은 {self.type_name}')
                    return
            except Exception:
                pass
            try:
                pubs = self.count_publishers(self.a.topic)
                if pubs > 1:
                    print(f'[경고] {self.a.topic}에 나 말고 발행자가 {pubs - 1}개 더 있음 (teleop·mission_node 끄기)')
            except Exception:
                pass
            self.ok_to_move = True
        if self.last_odom is None and t - self.t_begin > 5.0:
            self.finish('[오류] 5초 동안 /odom 안 옴: 드라이버 확인')
        elif self.last_odom is not None and t - self.last_odom > 0.5:
            self.finish('[오류] /odom 끊김: 정지')
        elif self.pose_mode != 'odom' and self.fu.t_imu is None and t - self.t_begin > 3.0:
            self.finish('[오류] /imu 안 옴: --pose odom으로 하거나 IMU 확인')

    def on_imu(self, m):
        self.fu.on_imu(self.stamp(m), m.angular_velocity.z)

    def pose_now(self, m):
        p = m.pose.pose.position
        yaw = yaw_of(m.pose.pose.orientation)
        if self.pose_mode == 'odom':
            if self.origin is None:
                self.origin = (p.x, p.y, yaw)
            x0, y0, yaw0 = self.origin
            dx, dy = p.x - x0, p.y - y0
            return (math.cos(yaw0) * dx + math.sin(yaw0) * dy,
                    -math.sin(yaw0) * dx + math.cos(yaw0) * dy, wrap(yaw - yaw0))
        return self.fu.x, self.fu.y, self.fu.yaw

    def on_odom(self, m):
        if self.finished:
            return
        t = self.now()
        self.last_odom = t
        p = m.pose.pose.position
        self.fu.on_odom(self.stamp(m), p.x, p.y, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x)
        if not self.ok_to_move or (self.pose_mode != 'odom' and not self.fu.ready):
            self.send(0.0, 0.0)
            return
        if self.origin is None and self.pose_mode != 'odom':
            self.origin = True
            print(f'출발 (위치 기준 {self.pose_mode})')
        x, y, yaw = self.pose_now(m)
        if self.origin is not None and self.n_cmd == 0 and self.pose_mode == 'odom':
            print('출발 (위치 기준 odom)')
        bad = self.fu.sign_problem()
        if bad and self.pose_mode != 'odom':
            self.finish(f'[정지] {bad}')
            return
        if self.pause_until is not None:            # 구간 사이: 멈춰서 기다림
            self.send(0.0, 0.0)
            if t < self.pause_until or not self.fu.is_still(self.stamp(m)):
                return
            self.pause_until = None
            print(f'구간 {self.seg_i + 1}/{len(self.segs)} 출발 ({"전진" if self.path.dir > 0 else "후진"} {self.path.length:.2f} m)')
        P = self.path
        self.idx, s, e, hp = P.project(x, y, self.idx)
        epsi = wrap(yaw - hp)
        self.s, self.e, self.epsi = s, e, epsi
        self.max_e = max(self.max_e, abs(e))
        if s >= P.length - self.a.stop_early:
            msg = f'옆 오차 {e * 100:+.1f} cm, 방향 오차 {math.degrees(epsi):+.1f}°'
            if self.seg_i + 1 < len(self.segs):
                self.send(0.0, 0.0)
                print(f'구간 {self.seg_i + 1} 끝 ({msg}) → 멈춤')
                self.seg_i += 1
                self.path = self.segs[self.seg_i]
                self.idx = 0
                self.pause_until = t + self.a.pause
                return
            self.finish(f'종료: 끝점 도착 ({msg}, 추정 위치 기준 → 줄자로도 재세요)')
            return
        if abs(e) > self.a.max_err or abs(epsi) > math.radians(50):
            self.finish(f'[정지] 경로에서 너무 벗어남: 옆 {e * 100:+.0f} cm, 방향 {math.degrees(epsi):+.0f}° '
                        f'(출발 자세가 기록 때와 다르거나 센서 부호 문제)')
            return
        d = P.dir
        v = d * (self.a.speed if d > 0 else self.a.rev_speed)
        if self.a.ctrl == 'pp' and d > 0:
            L = self.a.lookahead
            tgt = next((i for i in range(self.idx, len(P)) if math.hypot(P.x[i] - x, P.y[i] - y) >= L), len(P) - 1)
            dx, dy = P.x[tgt] - x, P.y[tgt] - y
            l2 = max(dx * dx + dy * dy, 1e-4)
            curv = 2 * (-math.sin(yaw) * dx + math.cos(yaw) * dy) / l2
        else:
            # 곡률은 부호 있는 거리 기준 (후진이면 기록 곡률·방향 오차 항 부호가 바뀜. 옆 오차 항은 그대로)
            kff = d * P.k_at(s + abs(v) * self.a.preview)
            curv = kff - self.a.k_e * e - d * self.a.k_psi * math.sin(epsi)
        sat = abs(curv) >= MAX_CURV
        curv = max(-MAX_CURV, min(MAX_CURV, curv))
        self.n_cmd += 1
        self.n_sat += sat
        self.send(v, curv)
        if t - self.last_print >= 1.0:
            self.last_print = t
            print(f'  [{self.seg_i + 1}/{len(self.segs)} {"전진" if d > 0 else "후진"}] 진행 {s:.2f}/{P.length:.2f} m  옆 오차 {e * 100:+.1f} cm  방향 오차 {math.degrees(epsi):+.1f}°  '
                  f'곡률 {curv:+.2f}{" (한계)" if sat else ""}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('speed', type=float, nargs='?', default=0.2)
    ap.add_argument('stop_early', type=float, nargs='?', default=0.0)
    ap.add_argument('--pose', choices=['auto', 'fused', 'odom', 'imu'], default='auto',
                    help='auto: 새 기록이면 fused(odom 거리 + IMU 방향), 예전 기록이면 odom')
    ap.add_argument('--ctrl', choices=['track', 'pp'], default='track')
    ap.add_argument('--k_e', type=float, default=6.0, help='옆 오차 게인 (1/m^2)')
    ap.add_argument('--k_psi', type=float, default=3.4, help='방향 오차 게인 (1/m)')
    ap.add_argument('--preview', type=float, default=0.3, help='s, 기록 곡률을 이만큼 앞에서 미리 봄 (조향 지연 보정)')
    ap.add_argument('--rev_speed', type=float, default=None, help='m/s, 후진 구간 속도 (기본 = speed)')
    ap.add_argument('--pause', type=float, default=0.5, help='s, 전진↔후진 바뀔 때 멈춰 있는 시간')
    ap.add_argument('--max_err', type=float, default=0.30, help='m, 옆 오차가 이보다 크면 정지')
    ap.add_argument('--lookahead', type=float, default=0.3, help='m, --ctrl pp일 때만')
    ap.add_argument('--topic', default='/cmd_vel')
    ap.add_argument('--type', choices=['twist', 'ackermann'], default='twist')
    a = ap.parse_args()
    if a.rev_speed is None:
        a.rev_speed = a.speed
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = Follower(a)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print('중단(Ctrl+C): 정지 명령 전송')
        node.finish()
    finally:
        for _ in range(10):
            try:
                node.send(0.0, 0.0)
            except Exception:
                break
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
