"""클래스 → 행동 요청 변환. 속도·거리·조향 제어와 ROS에 의존하지 않는다."""

import math
import uuid
from collections import deque


V3_CLASSES = ('red', 'yellow', 'green', 'fork_a', 'fork_b', 'pedestrian',
              'bump', 'slow', 'parking')
TRAINING_LABELS = ('traffic_red', 'traffic_yellow', 'traffic_green', 'sign_left(A)',
                   'sign_right(B)', 'sign_pedestrian', 'sign_bump', 'sign_slow',
                   'sign_parking')
_ALIASES = dict(zip(TRAINING_LABELS, V3_CLASSES))

# 기존 fsm.py의 구간 역할. 제어 전략은 그곳에 두고 여기서는 발행 대상만 제한한다.
MODE_CLASSES = {
    'M1_TRAFFIC': ('red', 'yellow', 'green', 'pedestrian'),
    'M2_FORK': ('fork_a', 'fork_b'),
    'M3_PEDESTRIAN': ('pedestrian',),
    'M4_PARKING': ('parking',),
    'M5_BUMP': ('slow', 'bump'),
    'M6_ZIGZAG': ('pedestrian',),    # 11/13 실차: 보행자 표지는 지그재그 끝 무렵에 보이고 M3 시작 땐 이미 시야 밖 -> M6에서 미리 받음
    'M7_OBSTACLE': ('fork_a', 'fork_b'),
    'M2_EXIT': (),                   # 11/14: 합류 끝 ~ 라바콘 시작선 (받는 표지 없음)
    'M8_CONES': (),
}


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def canonical_class(name):
    """ONNX의 팀 라벨과 .pt의 학습 라벨을 같은 클래스명으로 맞춘다."""
    return _ALIASES.get(name, name) if isinstance(name, str) else None


def validate_options(conf_threshold, yellow_action, vote_window=10, vote_need=7):
    if not _finite_number(conf_threshold) or not 0.0 <= conf_threshold <= 1.0:
        raise ValueError('conf_threshold must be a finite number in [0, 1]')
    if yellow_action not in ('none', 'pause'):
        raise ValueError('yellow_action must be none or pause')
    if (type(vote_window) is not int or type(vote_need) is not int
            or not 1 <= vote_need <= vote_window):
        raise ValueError('vote counts must be integers with 1 <= vote_need <= vote_window')


class ClassCommandRouter:
    """새 검출 프레임마다 명령 스냅샷을 만든다.

    기본값: 10프레임이 쌓인 뒤 최근 10프레임 중 7회 검출된 클래스만 명령으로 낸다.
    적색 정지는 녹색 확인 전까지 유지한다. 빈 검출이나 기본 무동작인 황색으로
    정지를 해제하지 않는다. 신호등 정지는 다른 행동보다 우선하는 수신 규약이다.
    반복 프레임은 같은 행동의 재시작 요청이 아니다. 실행 완료·기간은 수신 측 책임이다.
    """

    def __init__(self, conf_threshold=0.5, yellow_action='none', vote_window=10, vote_need=7):
        validate_options(conf_threshold, yellow_action, vote_window, vote_need)
        self.conf_threshold = conf_threshold
        self.yellow_action = yellow_action
        self.vote_window, self.vote_need = vote_window, vote_need
        self._history = deque(maxlen=vote_window)
        self._evidence = {}
        self.traffic_stop_active = False
        self._last_stamp = None
        self._stop = None
        self.mode = ''
        self.min_heights = dict.fromkeys(V3_CLASSES, 0.0)
        # 미션별 투표 기준 (11/10): 0이면 vote_need를 쓴다. 갈림길·신호등은 엄하게, 서행·주차는 느슨하게 둘 수 있다
        self.mode_need = dict.fromkeys(MODE_CLASSES, 0)
        self.early_modes = set()      # 창이 다 차기 전에도 need번이면 인정하는 미션 (11/12 밤: 갈림길)
        self._session = uuid.uuid4().hex
        self._request_counter = 0
        self._requests = {}

    def set_mode(self, mode):
        """구간 전환 시 이전 구간의 표·신호등 정지 요청을 폐기한다."""
        if not isinstance(mode, str):
            raise ValueError('mission state must be a string')
        if mode != self.mode:
            self.mode = mode
            self._history.clear()
            self._evidence.clear()
            self.traffic_stop_active = False
            self._stop = None
            self._requests.clear()

    def need_for(self, mode):
        """이 미션에서 최근 vote_window 프레임 중 몇 번 봐야 인정하나."""
        n = self.mode_need.get(mode, 0)
        return n if n > 0 else self.vote_need

    @property
    def last_stamp(self):
        return self._last_stamp

    def configure(self, conf_threshold, yellow_action, vote_window=None, vote_need=None,
                  min_heights=None, mode_need=None):
        window = self.vote_window if vote_window is None else vote_window
        need = self.vote_need if vote_need is None else vote_need
        heights = self.min_heights if min_heights is None else min_heights
        per_mode = self.mode_need if mode_need is None else mode_need
        validate_options(conf_threshold, yellow_action, window, need)
        if (set(per_mode) != set(MODE_CLASSES)
                or any(type(v) is not int or not 0 <= v <= window for v in per_mode.values())):
            raise ValueError('per-mission vote_need must be integers in [0, vote_window] (0 = vote_need)')
        if (set(heights) != set(V3_CLASSES)
                or any(not _finite_number(v) or v < 0 for v in heights.values())):
            raise ValueError('minimum bbox heights must be finite nonnegative pixels')
        if (conf_threshold != self.conf_threshold or window != self.vote_window
                or need != self.vote_need or heights != self.min_heights or per_mode != self.mode_need):
            # 예전 임계값에서 모은 표를 새 임계값으로 인정하지 않는다. 적색 정지는 유지.
            self._history = deque(maxlen=window)
            self._evidence.clear()
        self.conf_threshold, self.yellow_action = conf_threshold, yellow_action
        self.vote_window, self.vote_need = window, need
        self.min_heights = dict(heights)
        self.mode_need = dict(per_mode)

    def route(self, frame, *, slow_ground=False):
        """검출 → 요청. 중복/역행 첫 프레임은 버리고, 역행 시 투표를 새로 시작한다."""
        if not isinstance(frame, dict):
            raise ValueError('detection frame must be an object')
        stamp = frame.get('stamp')
        if not _finite_number(stamp) or stamp < 0:
            raise ValueError('stamp must be a finite nonnegative number')
        detections = frame.get('detections')
        if not isinstance(detections, list):
            raise ValueError('detections must be a list')
        if self._last_stamp is not None and stamp <= self._last_stamp:
            if stamp < self._last_stamp:
                self._history.clear()
                self._evidence.clear()
                self._last_stamp = stamp
                # 이미 확인한 정지는 유지. 새 시간축의 녹색도 다시 투표해야 한다.
            return None

        best = {}
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            cls = canonical_class(detection.get('cls'))
            conf = detection.get('conf')
            if (cls not in MODE_CLASSES.get(self.mode, ()) or not _finite_number(conf)
                    or not self.conf_threshold <= conf <= 1.0):
                continue
            if cls not in best or conf > best[cls]['confidence']:
                evidence = {'confidence': float(conf), 'source_stamp': stamp}
                bbox = detection.get('bbox')
                if (isinstance(bbox, (list, tuple)) and len(bbox) == 4
                        and all(_finite_number(v) and v >= 0 for v in bbox)):
                    evidence['bbox'] = list(bbox)  # 원본 영상의 [x, y, w, h] (px)
                if self.min_heights[cls] > 0 and (
                        'bbox' not in evidence or evidence['bbox'][3] < self.min_heights[cls]):
                    continue
                best[cls] = evidence

        self._last_stamp = stamp
        self._history.append(set(best))
        self._evidence.update(best)
        counts = {cls: sum(cls in sample for sample in self._history) for cls in V3_CLASSES}
        need = self.need_for(self.mode)
        # 11/12 밤: 갈림길 표지(M2·M7)는 창이 다 차기를(5Hz에서 2초) 기다리지 않고 need번 보이는 즉시 인정 (early_modes)
        ready = len(self._history) == self.vote_window or self.mode in self.early_modes
        confirmed = {cls for cls in V3_CLASSES if ready and counts[cls] >= need}
        stop_class = 'red' if 'red' in confirmed else (
            'yellow' if 'yellow' in confirmed and self.yellow_action == 'pause' else None)
        if stop_class is not None:
            if not self.traffic_stop_active:
                # 다음 녹색은 이전 출발 유지가 아니라 새 정지의 해제 요청이다.
                self._requests.pop(('green', 'RESUME_NORMAL', None), None)
                # 정지 시작 전에 본 녹색을 새 출발의 근거로 재사용하지 않는다.
                for sample in self._history:
                    sample.discard('green')
                counts['green'] = 0
                confirmed.discard('green')
            self.traffic_stop_active = True
            self._stop = {'class': stop_class, **self._evidence[stop_class]}
        elif 'green' in confirmed and 'green' in best:
            self.traffic_stop_active = False
            self._stop = None
            for cls in ('red', 'yellow'):
                self._requests.pop((cls, 'PAUSE_UNTIL_GREEN', None), None)

        commands = []

        def add(cls, command, **extra):
            commands.append({'class': cls, 'command': command,
                             **self._evidence[cls], 'held': cls not in best, **extra})

        if self.traffic_stop_active:
            commands.append({**self._stop, 'command': 'PAUSE_UNTIL_GREEN',
                             'stop_target': 'before_stop_line',
                             'held': self._stop['class'] not in best or stop_class is None})
        elif ('green' in confirmed and 'green' in best
              and not confirmed.intersection(('slow', 'bump'))):
            add('green', 'RESUME_NORMAL')

        for cls, command in (('slow', 'SLOW'), ('bump', 'CROSS_BUMP_SLOWLY'),
                             ('pedestrian', 'STOP_AT_PEDESTRIAN_LINE'),
                             ('parking', 'START_PARKING')):
            if cls in confirmed:
                if cls == 'slow' and (slow_ground is not True or cls not in best):
                    continue
                extra = {'traffic_policy': 'obey_if_present'} if cls == 'pedestrian' else {}
                add(cls, command, **extra)

        # 두 표지를 동시에 읽었다고 임의로 A/B를 고르지 않는다.
        fork_conflict = 'fork_a' in confirmed and 'fork_b' in confirmed
        if not fork_conflict:
            for cls, route in (('fork_a', 'A'), ('fork_b', 'B')):
                if cls in confirmed:
                    add(cls, 'SELECT_ROUTE', route=route)

        for command in commands:
            key = (command['class'], command['command'], command.get('route'))
            command['new_request'] = key not in self._requests
            if command['new_request']:
                self._request_counter += 1
                self._requests[key] = f'{self._session}:{self._request_counter}'
            command['request_id'] = self._requests[key]

        return {'schema_version': 2, 'stamp': stamp, 'mode': self.mode,
                # 부가 관측 신호. commands는 그대로 유지하여 제어 동작을 바꾸지 않는다.
                # 황색 기본 정책은 none: 정지/가속/적색 해제 요청이 아니다.
                'signals': ([{'class': 'yellow', 'signal': 'YELLOW_OBSERVED',
                              'action': self.yellow_action, **self._evidence['yellow']}]
                            if 'yellow' in confirmed and 'yellow' in best else []),
                'observed_classes': [cls for cls in V3_CLASSES if cls in best],
                'confirmed_classes': [cls for cls in V3_CLASSES if cls in confirmed],
                'vote': {'window': self.vote_window, 'need': need,
                         'frames': len(self._history), 'counts': counts},
                'traffic_stop_active': self.traffic_stop_active,
                'commands': commands,
                'conflicts': ['fork_direction'] if fork_conflict else []}
