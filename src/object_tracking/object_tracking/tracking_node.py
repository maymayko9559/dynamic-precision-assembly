# ============================================================
# tracking_node.py
# ============================================================
# object_tracking 패키지의 유일한 ROS2 노드.
#
# [M3] 칼만 필터 통합 - 실시간 위치 추정
#   - /vision/detected_object 중 type=="target" 의 (x,y,z) 를 측정으로 사용
#   - 측정 콜백   : predict(dt) + update(z)
#   - 50Hz 타이머 : predict(dt) 후 현재 추정 위치를 PredictedTarget 로 발행
#                   (prediction_time = 0.0 ; 미래 예측은 M4)
#   - 측정이 max_coast_time 이상 끊기면 발행 중단
#
# dt 는 메시지에 타임스탬프가 없어 "노드 수신 시각" 차이로 계산.
# 기본 SingleThreadedExecutor 라 콜백/타이머가 직렬 실행 -> 락 불필요.
# ============================================================

import rclpy
from rclpy.node import Node

from assembly_interfaces.msg import DetectedObject, PredictedTarget

from .kalman_filter import ConstantVelocityKF


class TrackingNode(Node):

    def __init__(self):
        super().__init__("tracking_node")

        # ----- parameters -----
        self.declare_parameter("input_topic", "/vision/detected_object")
        self.declare_parameter("output_topic", "/tracking/predicted_target")
        self.declare_parameter("track_type", "target")
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("belt_speed", 50.0)
        self.declare_parameter("process_accel_std", 20.0)
        self.declare_parameter("meas_pos_std", [3.0, 3.0, 8.0])
        self.declare_parameter("gate_chi2", 7.815)
        self.declare_parameter("dt_min", 0.005)
        self.declare_parameter("dt_max", 0.15)
        self.declare_parameter("warmup_updates", 5)
        self.declare_parameter("max_coast_time", 1.0)

        gp = self.get_parameter
        input_topic = gp("input_topic").value
        output_topic = gp("output_topic").value
        self.track_type = gp("track_type").value
        self.max_coast_time = float(gp("max_coast_time").value)

        # ----- Kalman filter -----
        self.kf = ConstantVelocityKF(
            accel_std=gp("process_accel_std").value,
            meas_std=list(gp("meas_pos_std").value),
            belt_speed=gp("belt_speed").value,
            dt_min=gp("dt_min").value,
            dt_max=gp("dt_max").value,
            gate_chi2=gp("gate_chi2").value,
            warmup_updates=gp("warmup_updates").value,
        )

        # ----- time bookkeeping (수신 시각 기반) -----
        self.t_state = None         # 필터 상태가 유효한 시각 [s]
        self.t_last_meas = None     # 마지막으로 측정을 반영한 시각 [s]
        self.last_shape = "box"

        # ----- I/O -----
        self.sub = self.create_subscription(
            DetectedObject, input_topic, self.on_detection, 10)
        self.pub = self.create_publisher(
            PredictedTarget, output_topic, 10)

        hz = float(gp("update_rate_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.on_timer)

        self._rx = 0
        self.get_logger().info(
            f"tracking_node (M3) | sub={input_topic} type=='{self.track_type}' "
            f"-> pub={output_topic} @ {hz:.0f}Hz"
        )

    # --------------------------------------------------------

    def _now(self):
        # 현재 시각을 초 단위 float 으로
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------
    # 측정 콜백 : predict + update
    # --------------------------------------------------------

    def on_detection(self, msg: DetectedObject):

        if msg.type != self.track_type:
            return

        t = self._now()
        z = [msg.x, msg.y, msg.z]
        self.last_shape = msg.shape
        self._rx += 1

        # 첫 측정 -> 필터 켜기
        if not self.kf.initialized:
            self.kf.init_measurement(z)
            self.t_state = t
            self.t_last_meas = t
            self.get_logger().info(
                f"filter initialized at "
                f"({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) mm"
            )
            return

        # 지난 상태 시각 -> 이번 측정 시각까지 예측한 뒤 측정 반영
        self.kf.predict(t - self.t_state)
        accepted = self.kf.update(z)
        self.t_state = t

        if accepted:
            self.t_last_meas = t
        else:
            self.get_logger().warn(
                f"measurement rejected (NIS={self.kf.last_nis:.1f}) "
                f"z=({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})",
                throttle_duration_sec=1.0,
            )

    # --------------------------------------------------------
    # 50Hz 타이머 : predict + 현재 추정 발행
    # --------------------------------------------------------

    def on_timer(self):

        if not self.kf.initialized:
            return

        now = self._now()
        self.kf.predict(now - self.t_state)
        self.t_state = now

        # 측정이 너무 오래 끊겼으면(가림 등) 발행 중단
        coast = now - self.t_last_meas
        if coast > self.max_coast_time:
            self.get_logger().warn(
                f"target lost (no measurement for {coast:.2f}s) - not publishing",
                throttle_duration_sec=1.0,
            )
            return

        p = self.kf.position

        out = PredictedTarget()
        out.shape = self.last_shape
        out.x = float(p[0])
        out.y = float(p[1])
        out.z = float(p[2])
        out.prediction_time = 0.0        # M3: 현재 추정치. 미래 예측은 M4.
        self.pub.publish(out)

        v = self.kf.velocity
        self.get_logger().info(
            f"est pos=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm  "
            f"vel=({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}) mm/s  "
            f"|v|={self.kf.speed():.1f}  posStd={self.kf.pos_std().max():.2f}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
