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

'''
Terminal 1: 
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video4

  
Terminal 2:
cd ~/dynamic_assembly_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vision_system vision_manager \
  --ros-args -p image_topic:=/image_raw

When video is too dark,
Terminal 3:
v4l2-ctl -d /dev/video4 --set-ctrl=brightness=128
  
RealSense: 
ros2 run vision_system vision_manager \
  --ros-args -p image_topic:=/camera/color/image_raw

'''
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .board_detector import BoardDetector
from .object_detector import ObjectDetector
from .target_detector import TargetDetector
from .coordinate_transform import CoordinateTransform

import cv2

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
        self.declare_parameter("image_topic", "/image_raw")

        image_topic = self.get_parameter("image_topic").value

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info("Vision Manager started.")

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


        try:
            # ROS Image -> OpenCV Image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Show camera image
            cv2.imshow("Vision Manager - Camera", frame)

            # Required for OpenCV window update
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f"Failed to process camera image: {e}"
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
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()