#!/usr/bin/env python3
"""
fork_node — 미션 2 갈림길(A/B) 판단 노드

입력
  /perception/detections (std_msgs/String, JSON)  ← 팀 YOLO(detector_node). fork_a / fork_b 클래스
      input_mode: text 이면 대신 sign_topic(std_msgs/String, 'A'/'B' 등)을 받음
  /avoid/state  (std_msgs/String, JSON)  ← 회피 노드: 지금 차선(now_lane), 목표 차선(target), 섬까지 거리
  /mission/state (std_msgs/String, JSON) ← 팀 FSM (있으면): 지금 미션, FSM 이 이미 정한 fork_choice

출력
  /fork_direction (std_msgs/String) 'A'/'B'  → 회피 노드가 섬 앞 끝 전에 출구 차선으로 옮김 (0/1/2차선)
  /fork/state     (std_msgs/String, JSON)    → 결정, 지금 차선, 몇 차선 이동, 섬까지 거리, 상태 문장

결정 규칙
  - follow_fsm: true 이고 FSM 이 fork_choice 를 이미 정했으면 그걸 따름 (같은 YOLO 를 두 곳에서 따로 정해 엇갈리지 않게)
  - 아니면 최근 vote_window 프레임 중 vote_need 이상 (기본 8 중 5)
  - 한 번 정하면 유지 (allow_change: true 면 반대쪽이 다시 확정될 때 바꿈)
  - active_states 에 든 미션일 때만 투표 (FSM 이 없으면 항상)
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from autorace_fork.fork_core import ForkVote, parse_detections, parse_sign_text, plan


class ForkNode(Node):
    def __init__(self):
        super().__init__('fork_node')
        d = self.declare_parameter
        d('input_mode', 'detections')                 # detections | text
        d('det_topic', '/perception/detections')
        d('sign_topic', '/fork/sign')
        d('cls_a', 'fork_a')
        d('cls_b', 'fork_b')
        d('det_conf', 0.5)
        d('min_box_h', 0.0)
        d('vote_window', 8)
        d('vote_need', 5)
        d('follow_fsm', True)
        d('allow_change', False)
        d('active_states', ['M7_OBSTACLE', 'M2_FORK'])
        d('resend_max', 3)                            # 회피 노드가 방향을 잃었으면 다시 보낼 최대 횟수
        g = lambda n: self.get_parameter(n).value     # noqa: E731
        self.vote = ForkVote(int(g('vote_window')), int(g('vote_need')))
        self.decision = None
        self.source = None
        self.avoid = {}
        self.mission = None
        self.sent = None
        self.resends = 0
        self.t_resend = 0.0
        self.last_text = None
        if g('input_mode') == 'text':
            self.create_subscription(String, g('sign_topic'), lambda m: self.on_seen(parse_sign_text(m.data)), 10)
            src = g('sign_topic')
        else:
            self.create_subscription(String, g('det_topic'), self.on_det, 10)
            src = g('det_topic')
        self.create_subscription(String, '/avoid/state', self.on_avoid, 10)
        self.create_subscription(String, '/mission/state', self.on_mission, 10)
        self.pub_dir = self.create_publisher(String, '/fork_direction', 10)
        self.pub_state = self.create_publisher(String, '/fork/state', 10)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'fork_node 준비: 표지 입력 {src} ({g("input_mode")}), '
                               f'투표 {g("vote_need")}/{g("vote_window")}, FSM 따르기={g("follow_fsm")}')

    # ---------- 입력 ----------
    def active(self):
        if self.mission is None:
            return True                                # FSM 없이 단독 시험
        return self.mission.get('state') in list(self.get_parameter('active_states').value)

    def on_det(self, m):
        g = self.get_parameter
        self.on_seen(parse_detections(m.data, g('cls_a').value, g('cls_b').value,
                                      float(g('det_conf').value), float(g('min_box_h').value)))

    def on_seen(self, seen):
        if not self.active():
            return
        res = self.vote.push(seen)
        if res is None:
            return
        if self.decision is None or (res != self.decision and self.get_parameter('allow_change').value):
            self.decide(res, 'vote')

    def on_avoid(self, m):
        try:
            self.avoid = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def on_mission(self, m):
        try:
            self.mission = json.loads(m.data)
        except json.JSONDecodeError:
            return
        fc = self.mission.get('fork_choice')
        if self.get_parameter('follow_fsm').value and fc in ('A', 'B') and fc != self.decision:
            self.decide(fc, 'fsm')

    # ---------- 결정·전달 ----------
    def decide(self, d, source):
        old = self.decision
        self.decision, self.source = d, source
        na, nb = self.vote.counts()
        how = 'FSM 결정을 따름' if source == 'fsm' else f'투표 A {na} / B {nb}'
        self.get_logger().info(f'갈림길 {d} 결정 ({how})' + (f' — {old} 에서 바뀜' if old else ''))
        self.send()

    def send(self):
        self.pub_dir.publish(String(data=self.decision))
        self.sent = self.decision
        self.t_resend = time.monotonic()

    def tick(self):
        st = plan(self.decision, self.avoid)
        st['source'] = self.source
        st['votes'] = dict(zip(('A', 'B'), self.vote.counts()))
        st['active'] = self.active()
        self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))
        key = (st['decision'], st['lane'], st.get('status'))      # 숫자(남은 거리)만 바뀌면 다시 안 찍음
        if key != self.last_text and self.decision is not None:
            self.get_logger().info(st['text'])
            self.last_text = key
        # 회피 노드가 실행 중인데 출구 방향을 모르면(재시작 등) 다시 보냄 (횟수 제한)
        if self.decision and self.avoid.get('state') == 'RUN' and self.avoid.get('exit_pref') is None \
                and self.avoid.get('exit_plan') not in ('too_late', 'given_up') \
                and self.resends < int(self.get_parameter('resend_max').value) \
                and time.monotonic() - self.t_resend > 1.0:
            self.resends += 1
            self.t_resend = time.monotonic()
            self.send()


def main():
    rclpy.init()
    n = ForkNode()
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
