# ============================================================
# tracking_node.py
# ============================================================
# object_tracking 패키지의 유일한 ROS2 노드.
#
# [M1 범위 - passthrough]
#   - /vision/detected_object (assembly_interfaces/DetectedObject) 구독
#   - msg.type == "target" 인 것만 통과
#   - (x, y, z) 를 그대로 assembly_interfaces/PredictedTarget 으로 재발행
#     (prediction_time = 0.0)
#
# [이후]
#   M2: ConstantVelocityKF (kalman_filter.py)
#   M3: 50Hz 타이머 predict + 측정 콜백 update  -> 실시간 위치 추정
#   M4: MotionPredictor (motion_predictor.py)   -> 미래 위치 예측
# ============================================================

import rclpy
from rclpy.node import Node

from assembly_interfaces.msg import DetectedObject, PredictedTarget


class TrackingNode(Node):

    def __init__(self):
        super().__init__("tracking_node")

        # ----------------------------------------------------
        # Parameters (M1 subset)
        # ----------------------------------------------------

        self.declare_parameter("input_topic", "/vision/detected_object")
        self.declare_parameter("output_topic", "/tracking/predicted_target")
        self.declare_parameter("track_type", "target")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.track_type = self.get_parameter("track_type").value

        # ----------------------------------------------------
        # I/O
        # ----------------------------------------------------
        #
        # vision_manager 는 DetectedObject 를 기본 QoS(depth 10, reliable)
        # 로 발행하므로 동일하게 맞춘다.

        self.sub = self.create_subscription(
            DetectedObject,
            input_topic,
            self.on_detection,
            10,
        )

        self.pub = self.create_publisher(
            PredictedTarget,
            output_topic,
            10,
        )

        self._rx_count = 0

        self.get_logger().info(
            "tracking_node (M1 passthrough) started | "
            f"sub={input_topic} (type=='{self.track_type}') "
            f"-> pub={output_topic}"
        )

    # --------------------------------------------------------
    # Detection callback
    # --------------------------------------------------------

    def on_detection(self, msg: DetectedObject):

        # target 이 아닌 검출(object 등)은 무시
        if msg.type != self.track_type:
            return

        self._rx_count += 1

        # M1: 측정값을 그대로 예측값으로 통과시킨다.
        out = PredictedTarget()
        out.shape = msg.shape
        out.x = float(msg.x)
        out.y = float(msg.y)
        out.z = float(msg.z)
        out.prediction_time = 0.0

        self.pub.publish(out)

        self.get_logger().info(
            f"[{self._rx_count}] target '{msg.shape}' "
            f"({out.x:.1f}, {out.y:.1f}, {out.z:.1f}) mm -> republished",
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
