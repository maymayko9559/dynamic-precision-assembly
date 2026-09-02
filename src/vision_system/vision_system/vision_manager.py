# ============================================================
# detection_node.py
# ============================================================
# [EN]
# Main ROS2 node for the Vision System.
# Receives images from the RealSense camera and performs
# OpenCV-based object detection.
#
# Main Responsibilities:
# - Subscribe to the camera Image topic
# - Convert ROS Image to OpenCV Image
# - Call image processing and shape detection functions
# - Publish detected object information
#
# [KR]
# Vision System의 메인 ROS2 Node.
# RealSense 카메라 영상을 받아 OpenCV 기반 도형 검출을 수행한다.
#
# 주요 역할:
# - 카메라 Image Topic Subscribe
# - ROS Image를 OpenCV Image로 변환
# - 이미지 전처리 및 도형 검출 함수 호출
# - 검출된 도형 정보를 ROS2 Topic으로 Publish
# ============================================================


import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .board_detector import BoardDetector
from .object_detector import ObjectDetector
from .target_detector import TargetDetector
from .coordinate_transform import CoordinateTransform


class VisionManager(Node):

    def __init__(self):

        super().__init__("vision_manager")

        # =====================================================
        # CvBridge
        # =====================================================

        self.bridge = CvBridge()

        # =====================================================
        # Vision Modules
        # =====================================================

        self.board_detector = BoardDetector()
        self.object_detector = ObjectDetector()
        self.target_detector = TargetDetector()
        self.coordinate_transform = CoordinateTransform()

        # =====================================================
        # Camera Subscriber
        # =====================================================
        self.declare_parameter(
            "image_topic",
            "/image_raw"
        )

        image_topic = self.get_parameter(
            "image_topic"
        ).value

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(
            "Vision Manager started."
        )

    # =========================================================
    # Camera Callback
    # =========================================================

    def image_callback(self, msg):
        """
        Main vision processing flow.

        Camera
            -> Board Detection
            -> ROI Separation
            -> Object / Target Detection
            -> Coordinate Transform
            -> Publish
        """

        # TODO: Implement later

        self.get_logger().info(
            "Camera frame received.",
            throttle_duration_sec=1.0 
        )


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = VisionManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()