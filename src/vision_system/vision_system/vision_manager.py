# ============================================================
# vision_manager.py
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
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .board_detector import BoardDetector
from .object_detector import ObjectDetector
from .target_detector import TargetDetector
from .coordinate_transform import CoordinateTransform
from assembly_interfaces.msg import DetectedObject





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


        # ====================================================
        # Detected Object Publisher
        # ====================================================

        self.detection_pub = self.create_publisher(
            DetectedObject,
            "/vision/detected_object",
            10
        )

        
        self.get_logger().info("Vision Manager started.")


    def publish_detection(self, detection, position):
        """
        Publish detected object/target information.

        detection:
            {
                "type": "object" or "target",
                "shape": "circle", "star", ...
            }

        position:
            Robot coordinate (x, y, z)
        """

        x, y, z = position

        msg = DetectedObject()

        msg.type = detection["type"]
        msg.shape = detection["shape"]

        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)

        self.detection_pub.publish(msg)


    def process_targets(self, raw_frame, board_corners):

        # =========================================================
        # Extract Board ROI
        # =========================================================

        board_roi = self.board_detector.extract_board_roi(raw_frame, board_corners)

        # =========================================================
        # Detect Targets
        # =========================================================

        targets = self.target_detector.detect(board_roi)

        # =========================================================
        # Draw Targets
        # =========================================================

        for target in targets:
            self.draw_detection(board_roi, target)

        return board_roi

    def process_objects(self, raw_frame, board_corners):

        # =========================================================
        # Create Pick ROI
        # =========================================================

        pick_roi = self.create_pick_roi(raw_frame, board_corners)

        # =========================================================
        # Detect Objects
        # =========================================================

        objects = self.object_detector.detect(pick_roi)

        # =========================================================
        # Draw Objects
        # =========================================================

        for obj in objects:
            self.draw_detection(pick_roi, obj)

        return pick_roi


    def create_pick_roi(self, raw_frame, board_corners):

        # =========================================================
        # Create Pick ROI
        # =========================================================

        pick_roi = raw_frame.copy()

        board_points = board_corners.astype(
            np.int32
        )

        cv2.fillPoly(
            pick_roi,
            [board_points],
            (0, 0, 0)
        )

        return pick_roi

    def draw_detection(self, image, detection):

        shape = detection["shape"]
        center = detection["center"]
        contour = detection["contour"]

        # Draw contour
        cv2.drawContours(
            image,
            [contour],
            -1,
            (0, 255, 0),
            2
        )

        # Draw center
        cv2.circle(
            image,
            center,
            5,
            (0, 0, 255),
            -1
        )

        # Draw shape name
        cv2.putText(
            image,
            shape,
            (
                center[0] + 10,
                center[1]
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2
        )

    # =========================================================
    # Camera Callback
    # =========================================================
    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            raw_frame = frame.copy()
            debug_frame = frame.copy()

            # =====================================================
            # Board Detection
            # =====================================================

            corners, ids = self.board_detector.detect(
                raw_frame
            )

            board_corners = self.board_detector.get_board_corners(
                corners,
                ids
            )

            debug_frame = self.board_detector.draw_board(
                debug_frame,
                corners,
                ids
            )

            # =====================================================
            # Board Found
            # =====================================================

            if board_corners is not None:

                debug_frame = self.board_detector.draw_board_boundary(
                    debug_frame,
                    board_corners
                )

                # Target detection
                board_roi = self.process_targets(
                    raw_frame,
                    board_corners
                )

                # Object detection
                pick_roi = self.process_objects(
                    raw_frame,
                    board_corners
                )

                cv2.imshow(
                    "Target Board ROI",
                    board_roi
                )

                cv2.imshow(
                    "Pick ROI",
                    pick_roi
                )

            else:

                self.get_logger().info(
                    "Waiting for ArUco IDs 0, 1, 2, 3...",
                    throttle_duration_sec=1.0
                )

            # =====================================================
            # Main Debug Window
            # =====================================================

            cv2.imshow(
                "Vision Manager - Camera",
                debug_frame
            )

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