"""/cmd_vel(Twist) -> /limo/ack_cmd(AckermannDrive) 변환 노드.

이 LIMO의 애커만 드라이버(limo_ackerman_node)는 /cmd_vel이 아니라 /limo/ack_cmd만 받는다.
우리 노드들은 전부 Twist(linear.x = v, angular.z = v * kappa)로 내므로 여기서 조향각으로 바꾼다.

  조향각 delta = atan(wheelbase * angular.z / linear.x)    (앞차축 중앙의 가상 바퀴, 왼쪽 +)

LIMO Pro: 축거 0.2 m, 안쪽 바퀴 최대 28도 -> 가상 바퀴 최대 약 0.41 rad, 최소 회전반경 약 0.46 m.
명령이 cmd_timeout_sec보다 오래 안 오면 속도 0으로 세운다 (mission_node가 죽었을 때 안전장치).
"""

import math
import time


def twist_to_ackermann(v: float, w: float, wheelbase: float = 0.2,
                       max_steer: float = 0.40, min_speed: float = 0.02):
    """(speed, steering_angle) 반환. 정지에 가까우면 조향 0."""
    if abs(v) < min_speed:
        return float(v), 0.0
    delta = math.atan(wheelbase * w / v)
    return float(v), float(max(-max_steer, min(max_steer, delta)))


def main(args=None):
    import rclpy
    from ackermann_msgs.msg import AckermannDrive
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    class AckermannBridge(Node):
        def __init__(self):
            super().__init__('ackermann_bridge')
            self.declare_parameter('wheelbase', 0.2)
            self.declare_parameter('max_steer', 0.40)         # rad, 가상 바퀴 기준
            self.declare_parameter('max_speed', 0.5)          # m/s 최종 안전 상한
            self.declare_parameter('cmd_timeout_sec', 0.3)
            self.declare_parameter('in_topic', '/cmd_vel')
            self.declare_parameter('out_topic', '/limo/ack_cmd')
            self.speed = 0.0
            self.steer = 0.0
            self.last = None
            self.pub = self.create_publisher(
                AckermannDrive, self.get_parameter('out_topic').value, 10)
            self.create_subscription(Twist, self.get_parameter('in_topic').value,
                                     self._on_cmd, 10)
            self.create_timer(0.05, self._tick)
            self.get_logger().info('ackermann_bridge: /cmd_vel -> /limo/ack_cmd')

        def _on_cmd(self, m: Twist):
            p = self.get_parameter
            v, d = twist_to_ackermann(m.linear.x, m.angular.z, p('wheelbase').value,
                                      p('max_steer').value)
            vmax = p('max_speed').value
            self.speed = max(-vmax, min(vmax, v))
            self.steer = d
            self.last = time.monotonic()
            self._tick()      # 타이머(20Hz)를 기다리지 않고 바로 보낸다 (지연 최대 50ms 줄임)

        def _tick(self):
            stale = (self.last is None or
                     time.monotonic() - self.last > self.get_parameter('cmd_timeout_sec').value)
            msg = AckermannDrive()
            msg.speed = 0.0 if stale else float(self.speed)
            msg.steering_angle = 0.0 if stale else float(self.steer)
            self.pub.publish(msg)

    rclpy.init(args=args)
    node = AckermannBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop = AckermannDrive()
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
