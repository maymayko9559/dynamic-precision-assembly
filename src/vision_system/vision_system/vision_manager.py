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

RealSense:
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true
  
Terminal 2:
cd ~/dynamic_assembly_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vision_system vision_manager \
  --ros-args -p image_topic:=/image_raw

RealSense: 
ros2 run vision_system vision_manager --ros-args \
  -p color_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/depth/image_rect_raw


Terminal 3:
Webcam: when video is too dark,
v4l2-ctl -d /dev/video4 --set-ctrl=brightness=128

Realsense: 
ros2 param set /camera/camera align_depth.enable true


'''
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from .board_detector import BoardDetector
from .object_detector import ObjectDetector
from .target_detector import TargetDetector
from .coordinate_transform import CoordinateTransform
from assembly_interfaces.msg import DetectedObject

from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data




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
        # Camera Subscriber - for WebCam
        # =====================================================

        # self.declare_parameter("image_topic", "/image_raw")

        # image_topic = self.get_parameter("image_topic").value

        # self.image_sub = self.create_subscription(
        #     Image,
        #     image_topic,
        #     self.color_callback,
        #     10
        # )

        self.current_robot_pose = None

        self.robot_pose_sub = self.create_subscription(
            Float64MultiArray,
            '/robot/current_pose',
            self.robot_pose_callback,
            10
        )

        # ============================================================
        # Camera Topics
        # ============================================================

        self.declare_parameter(
            "color_topic",
            "/camera/color/image_raw"
        )

        self.declare_parameter(
            "depth_topic",
            "/camera/aligned_depth_to_color/image_raw"
        )

        color_topic = self.get_parameter(
            "color_topic"
        ).value

        depth_topic = self.get_parameter(
            "depth_topic"
        ).value


        # ============================================================
        # Camera Info Subscriber
        # ============================================================

        self.declare_parameter(
            "camera_info_topic",
            "/camera/aligned_depth_to_color/camera_info"
        )

        camera_info_topic = self.get_parameter(
            "camera_info_topic"
        ).value

        self.camera_intrinsics = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        # ============================================================
        # Color Image Subscriber
        # ============================================================

        self.color_sub = self.create_subscription(
            Image,
            color_topic,
            self.color_callback,
            qos_profile_sensor_data
        )

        # ============================================================
        # Depth Image Subscriber
        # ============================================================

        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.latest_depth_frame = None

        # ====================================================
        # Detected Object Publisher
        # ====================================================

        self.detection_pub = self.create_publisher(
            DetectedObject,
            "/vision/detected_object",
            10
        )

        
        self.get_logger().info("Vision Manager started.")


    def robot_pose_callback(self, msg):

        if len(msg.data) != 6:
            return

        self.current_robot_pose = list(msg.data)

    def camera_info_callback(self, msg):

        self.camera_intrinsics = {
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5]
        }

    def depth_callback(self, msg):

        try:

            self.latest_depth_frame = (
                self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="passthrough"
                )
            )

        except Exception as e:

            self.get_logger().error(
                f"Failed to convert depth image: {e}"
            )

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

        msg.angle = detection["angle"]

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

        return board_roi, targets

    # ============================================================
    # Process Objects
    # ============================================================

    def process_objects(self, raw_frame):

        if raw_frame is None:
            return None, []

        # Detect objects from the entire camera image.
        object_frame = raw_frame.copy()

        objects = self.object_detector.detect(
            object_frame
        )

        # Draw detected objects
        for obj in objects:
            self.draw_detection(
                object_frame,
                obj
            )

        return object_frame, objects

    # Future use
    def create_pick_roi(self, raw_frame, board_corners):

        # =========================================================
        # Create Pick ROI
        # =========================================================

        pick_roi = raw_frame.copy()

        board_points = board_corners.astype(np.int32)

        cv2.fillPoly(pick_roi, [board_points], (0, 0, 0))

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
        cv2.circle(image, center, 5, (0, 0, 255), -1)

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

    def is_camera_ready(self):

        if self.latest_depth_frame is None:
            self.get_logger().info(
                "Waiting for depth image...",
                throttle_duration_sec=1.0
            )
            return False

        if self.camera_intrinsics is None:
            self.get_logger().info(
                "Waiting for camera intrinsics...",
                throttle_duration_sec=1.0
            )
            return False

        return True
    
    def detect_board(self, raw_frame, debug_frame):

        corners, ids = self.board_detector.detect(
            raw_frame
        )

        board_corners = (
            self.board_detector.get_board_corners(
                corners,
                ids
            )
        )

        marker_centers = (
            self.board_detector.get_marker_centers(
                corners,
                ids
            )
        )

        debug_frame = (
            self.board_detector.draw_board(
                debug_frame,
                corners,
                ids
            )
        )

        if board_corners is not None:

            debug_frame = (
                self.board_detector.draw_board_boundary(
                    debug_frame,
                    board_corners
                )
            )

        return (
            board_corners,
            marker_centers,
            debug_frame
        )


    def process_target_positions(self, targets, board_corners, debug_frame):

        for target in targets:

            board_center = target["center"]

            camera_center = (
                self.board_detector.board_to_camera_pixel(
                    board_center,
                    board_corners
                )
            )

            u, v = camera_center

            cv2.circle(
                debug_frame,
                camera_center,
                8,
                (0, 0, 255),
                -1
            )

            # TEST
            self.get_logger().info(
                f'{target["shape"]}: '
                f'board={board_center}, '
                f'pixel=({u}, {v})'
            )

            camera_position = self.get_camera_position(u, v)



            if camera_position is None:
                self.get_logger().warning(
                    f'{target["shape"]}: camera position is None'
                )
                continue

            robot_pose = self.current_robot_pose

            if robot_pose is None:
                return

            T_base2gripper = (
                self.coordinate_transform.get_robot_pose_matrix(
                    *robot_pose
                )
            )


            self.get_logger().info(
                f"[TRANSFORM INPUT] "
                f"robot_pose={self.current_robot_pose}, "
                f"camera_xyz={camera_position}",
                throttle_duration_sec=0.5
            )

            robot_position = (
                self.coordinate_transform.camera_to_robot(
                    camera_position,
                    T_base2gripper
                )
            )

            self.get_logger().info(
                f'{target["shape"]}: '
                f'camera_xyz={camera_position}, '
                f'robot_xyz={robot_position}'
            )

            self.publish_detection(target, robot_position)

    def process_object_positions(self, objects):

        for obj in objects:

            u, v = obj["center"]

            camera_position = self.get_camera_position(u, v)

            if camera_position is None:
                continue

            if self.current_robot_pose is None:
                return

            T_base2gripper = (
                self.coordinate_transform.get_robot_pose_matrix(
                    *self.current_robot_pose
                )
            )

            robot_position = (
                self.coordinate_transform.camera_to_robot(
                    camera_position,
                    T_base2gripper
                )
            )

            self.get_logger().info(
                f'{obj["shape"]}: '
                f'camera_xyz={camera_position}, '
                f'robot_xyz={robot_position}'
            )

            self.publish_detection(
                obj,
                robot_position
            )

    def get_camera_position(self, u, v):

        h, w = self.latest_depth_frame.shape[:2]

        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warning(
                f"Pixel out of range: ({u}, {v})"
            )
            return None

        depth = self.latest_depth_frame[v, u]

        self.get_logger().info(
            f"pixel=({u}, {v}), depth={depth}"
        )

        if depth <= 0:
            self.get_logger().warning(
                f"Invalid depth: {depth}"
            )
            return None

        camera_position = self.coordinate_transform.pixel_to_camera(
            u,
            v,
            depth,
            self.camera_intrinsics
        )
            
        self.get_logger().info(
            f"Pixel -> Camera: "
            f"pixel=({u}, {v}), "
            f"depth={depth}, "
            f"camera_xyz={camera_position}"
        )

        return camera_position

    def show_windows(self, debug_frame, board_roi, pick_roi):

        cv2.imshow("Target Board ROI", board_roi)

        cv2.imshow("Pick ROI", pick_roi)

        self.show_debug(debug_frame)


    def show_debug(self, debug_frame):

        cv2.imshow(
            "Vision Manager - Camera",
            debug_frame
        )

        cv2.waitKey(1)

    def color_callback(self, msg):

        try:

            # ====================================================
            # ROS Image -> OpenCV
            # ====================================================

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            raw_frame = frame.copy()
            debug_frame = frame.copy()


            # ====================================================
            # Camera Ready Check
            # ====================================================

            if not self.is_camera_ready():
                return


            # ====================================================
            # 1. OBJECT DETECTION
            #
            # Object detection is independent from box detection.
            # Search the entire camera image.
            # ====================================================

            object_frame, objects = self.process_objects(
                raw_frame
            )

            self.process_object_positions(
                objects
            )


            # ====================================================
            # 2. BOX DETECTION
            #
            # ArUco IDs 0,1,2,3 define the box.
            # ====================================================

            board_corners, marker_centers, debug_frame = (
                self.detect_board(
                    raw_frame,
                    debug_frame
                )
            )


            if marker_centers is not None:

                for marker_id, center in marker_centers.items():

                    u = int(round(center[0]))
                    v = int(round(center[1]))

                    cv2.circle(
                        debug_frame,
                        (u, v),
                        6,
                        (0, 0, 255),
                        -1
                    )

                    cv2.putText(
                        debug_frame,
                        f"ID {marker_id}",
                        (u + 8, v),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2
                    )

            self.get_logger().info(
                f"[ARUCO CENTERS] {marker_centers}",
                throttle_duration_sec=1.0
            )

            # ====================================================
            # 3. BOX TARGET
            # ====================================================

            if board_corners is not None:

                self.process_box_target(
                    board_corners,
                    marker_centers,
                    debug_frame
                )

            else:

                self.get_logger().info(
                    "Waiting for Box ArUco IDs 0, 1, 2, 3...",
                    throttle_duration_sec=1.0
                )


            # ====================================================
            # 4. Debug Windows
            # ====================================================

            cv2.imshow(
                "Object Detection",
                object_frame
            )

            self.show_debug(
                debug_frame
            )


        except Exception as e:

            self.get_logger().error(
                f"Failed to process camera image: {e}"
            )

    # ============================================================
    # Process Box Center Target
    # ============================================================

    def process_box_target(
        self,
        board_corners,
        marker_centers,
        debug_frame
    ):
        if board_corners is None:
            return

        # ========================================================
        # 1. Calculate Box Center Pixel
        # ========================================================
        #
        # board_corners:
        # [
        #     [x0, y0],
        #     [x1, y1],
        #     [x2, y2],
        #     [x3, y3]
        # ]
        #
        # 네 개 corner의 평균 = box center
        # ========================================================

        corners = np.asarray(
            board_corners,
            dtype=np.float32
        ).reshape(-1, 2)

        if len(corners) != 4:
            self.get_logger().warning(
                f"[BOX TARGET] Invalid board corners: {len(corners)}"
            )
            return

        center = np.mean(
            corners,
            axis=0
        )

        u = int(round(center[0]))
        v = int(round(center[1]))


        # ========================================================
        # 2. Draw Box Target Center
        # ========================================================

        cv2.circle(
            debug_frame,
            (u, v),
            10,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            debug_frame,
            "BOX TARGET",
            (u + 15, v),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )



        # ========================================================
        # 3. Estimate Box Pose using ArUco
        # ========================================================

        pose_result = self.board_detector.estimate_box_pose(
            marker_centers,
            self.camera_intrinsics
        )

        if pose_result is None:
            self.get_logger().warning(
                "[BOX TARGET] solvePnP failed.",
                throttle_duration_sec=1.0
            )
            return


        camera_position, rvec, tvec = pose_result


        # ========================================================
        # 4. Camera -> Robot BASE Coordinate
        # ========================================================

        self.get_logger().info(
            f"[BOX PNP CAMERA] "
            f"camera_xyz={camera_position}",
            throttle_duration_sec=1.0
        )

        if self.current_robot_pose is None:
            self.get_logger().info(
                "[BOX TARGET] Waiting for robot pose...",
                throttle_duration_sec=1.0
            )
            return

        T_base2gripper = (
            self.coordinate_transform.get_robot_pose_matrix(
                *self.current_robot_pose
            )
        )

        robot_position = (
            self.coordinate_transform.camera_to_robot(
                camera_position,
                T_base2gripper
            )
        )


    
        self.get_logger().info(
            f"[BOX TARGET PNP] "
            f"pixel=({u}, {v}), "
            f"camera_xyz={camera_position}, "
            f"robot_xyz={robot_position}",
            throttle_duration_sec=1.0
)
        # ========================================================
        # DEBUG: Box Target Position
        # ========================================================

        # self.get_logger().info(
        #     f"[BOX TARGET] "
        #     f"pixel=({u}, {v}), "
        #     f"camera_xyz={camera_position}, "
        #     f"robot_xyz={robot_position}",
        #     throttle_duration_sec=1.0
        # )


        # ========================================================
        # 5. Create Target Detection
        # ========================================================
        #
        # shape는 지금 제어에는 사용하지 않지만
        # future use를 위해 field 자체는 유지.
        # ========================================================

        target = {
            "type": "target",
            "shape": "box",
            "center": (u, v),
            "angle": 0.0,
        }


        # ========================================================
        # 6. Publish
        # ========================================================

        self.publish_detection(
            target,
            robot_position
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

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()