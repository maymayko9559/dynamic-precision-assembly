# ============================================================
# assembly_controller.py
# ============================================================
# [EN]
# Main controller for managing the overall assembly sequence.
# Receives object/target information from the Vision System
# and controls the assembly process.
#
# Main Responsibilities:
# - Receive detected object/target information
# - Store detected objects and targets by shape
# - Match object with corresponding target
# - Calculate required rotation angle
# - Request current robot pose
# - Publish current robot pose
# - Approach the target
# - Align with the target
# - Perform insertion
# - Manage assembly completion
#
# [KR]
# 전체 조립 작업 순서를 관리하는 메인 ROS2 Node.
# Vision System에서 Object / Target 정보를 받아
# 전체 조립 작업을 관리한다.
#
# 주요 역할:
# - Vision 검출 결과 수신
# - Object / Target 정보를 도형별로 저장
# - 동일한 도형의 Object / Target 매칭
# - 필요한 회전 각도 계산
# - 현재 로봇 Pose 요청
# - 현재 로봇 Pose Publish
# - Target 접근
# - Target 위치 및 각도 정렬
# - 도형 삽입
# - 작업 완료 처리
# ============================================================


import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from assembly_interfaces.msg import DetectedObject

from .robot_init import RobotInit
from .motion_utils import MotionUtils


# ============================================================
# Robot Configuration
# ============================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL='m0609'


# ============================================================
# Assembly Controller
# ============================================================

class AssemblyController(Node):

    def __init__(self):

        super().__init__(
            "assembly_controller",
            namespace=ROBOT_ID
        )

        # ====================================================
        # Detected Objects
        # ====================================================
        #
        # Example:
        #
        # self.objects["triangle"] = {
        #     "x": 200.0,
        #     "y": 50.0,
        #     "z": 100.0,
        #     "angle": 20.0
        # }
        #
        # ====================================================

        self.objects = {}

        # ====================================================
        # Detected Targets
        # ====================================================
        #
        # Example:
        #
        # self.targets["triangle"] = {
        #     "x": 370.0,
        #     "y": 47.0,
        #     "z": 107.0,
        #     "angle": 65.0
        # }
        #
        # ====================================================

        self.targets = {}

        # ====================================================
        # Robot Initialization
        # ====================================================

        self.robot_init = RobotInit(self)

        self.mu = MotionUtils(
            self.robot_init
        )

        # ====================================================
        # Vision Detection Subscriber
        # ====================================================

        self.subscription = self.create_subscription(
            DetectedObject,
            "/vision/detected_object",
            self.listener_callback,
            10
        )

        self.get_logger().info(
            "DetectedObject 구독 시작"
        )

        # ====================================================
        # Robot Pose Publisher
        # ====================================================

        self.robot_pose_pub = self.create_publisher(
            Float64MultiArray,
            "/robot/current_pose",
            10
        )

        # ====================================================
        # Robot Pose Request State
        # ====================================================
        #
        # True:
        #     이전 GetCurrentPosx 요청의 응답을 기다리는 중
        #
        # False:
        #     새로운 요청 가능
        #
        # 응답이 오기 전에 새로운 Service Request가 계속
        # 쌓이는 것을 방지한다.
        # ====================================================

        self.pose_request_pending = False

        # ====================================================
        # Robot Pose Timer
        # ====================================================
        #
        # 0.1 sec = 10 Hz
        #
        # 0.1초마다 현재 TCP Pose를 요청한다.
        # ====================================================

        self.robot_pose_timer = self.create_timer(
            0.1,
            self.request_robot_pose
        )

        self.get_logger().info(
            "Assembly Controller started."
        )

    # ========================================================
    # Request Current Robot Pose
    # ========================================================

    def request_robot_pose(self):

        # 이전 요청의 응답을 기다리고 있으면
        # 새로운 요청을 보내지 않는다.
        if self.pose_request_pending:
            return

        # RobotInit에게 현재 Pose 요청
        requested = self.robot_init.request_current_pose(
            self.robot_pose_response
        )

        # Service 요청이 실제로 전송된 경우에만
        # pending 상태로 변경
        if requested:
            self.pose_request_pending = True

    # ========================================================
    # Current Robot Pose Response
    # ========================================================

    def robot_pose_response(self, future):

        # Service 응답을 받았으므로
        # 다음 요청을 받을 수 있도록 False
        self.pose_request_pending = False

        try:

            response = future.result()

            # =================================================
            # Check Service Result
            # =================================================

            if not response.success:

                self.get_logger().warning(
                    "Failed to get current robot pose."
                )

                return

            # =================================================
            # Check Pose Data
            # =================================================

            if len(response.task_pos_info) == 0:

                self.get_logger().warning(
                    "Current robot pose is empty."
                )

                return

            pose_data = response.task_pos_info[0].data

            # =================================================
            # Expected:
            #
            # [x, y, z, rx, ry, rz]
            #
            # task_pos_info may contain additional data.
            # Only the first 6 values are used.
            # =================================================

            if len(pose_data) < 6:

                self.get_logger().warning(
                    "Invalid current robot pose data."
                )

                return

            # =================================================
            # Convert Pose
            # =================================================

            robot_pose = [
                float(pose_data[0]),
                float(pose_data[1]),
                float(pose_data[2]),
                float(pose_data[3]),
                float(pose_data[4]),
                float(pose_data[5]),
            ]

            # =================================================
            # Publish Robot Pose
            # =================================================

            msg = Float64MultiArray()

            msg.data = robot_pose

            self.robot_pose_pub.publish(msg)

        except Exception as e:

            self.get_logger().error(
                f"Failed to get robot pose: {e}"
            )

    # ========================================================
    # Vision Detection Callback
    # ========================================================

    def listener_callback(
        self,
        msg: DetectedObject
    ):
        """
        Vision System에서 Object / Target 검출 결과가
        들어오면 호출된다.
        """

        # ====================================================
        # Object
        # ====================================================

        if msg.type == "object":

            self.objects[msg.shape] = {
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
                "angle": msg.angle,
            }

            self.get_logger().info(
                f"[OBJECT] "
                f"Shape: {msg.shape}, "
                f"X: {msg.x:.2f}, "
                f"Y: {msg.y:.2f}, "
                f"Z: {msg.z:.2f}, "
                f"Angle: {msg.angle:.2f}"
            )

        # ====================================================
        # Target
        # ====================================================

        elif msg.type == "target":

            self.targets[msg.shape] = {
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
                "angle": msg.angle,
            }

            self.get_logger().info(
                f"[TARGET] "
                f"Shape: {msg.shape}, "
                f"X: {msg.x:.2f}, "
                f"Y: {msg.y:.2f}, "
                f"Z: {msg.z:.2f}, "
                f"Angle: {msg.angle:.2f}"
            )

        else:

            self.get_logger().warning(
                f"Unknown detection type: {msg.type}"
            )

            return

        # ====================================================
        # Check Object / Target Match
        # ====================================================

        self.check_shape_match(msg.shape)

    # ========================================================
    # Check Shape Match
    # ========================================================

    def check_shape_match(self, shape):
        """
        Check whether both an object and its corresponding
        target have been detected.

        동일한 Shape의 Object와 Target이 모두 존재하는지
        확인한다.
        """

        if shape not in self.objects:
            return

        if shape not in self.targets:
            return

        obj = self.objects[shape]
        target = self.targets[shape]

        # ====================================================
        # Calculate Rotation Difference
        # ====================================================

        delta_angle = self.calculate_rotation_difference(
            shape,
            obj["angle"],
            target["angle"]
        )

        if delta_angle is None:

            self.get_logger().warning(
                f"Cannot calculate rotation for: {shape}"
            )

            return

        # ====================================================
        # Match Result
        # ====================================================

        self.get_logger().info(
            f"[MATCH] {shape} | "
            f"Object Angle: {obj['angle']:.2f} deg | "
            f"Target Angle: {target['angle']:.2f} deg | "
            f"Rotation: {delta_angle:.2f} deg"
        )

    # ========================================================
    # Calculate Rotation Difference
    # ========================================================

    def calculate_rotation_difference(
        self,
        shape,
        object_angle,
        target_angle
    ):
        """
        Calculate the minimum rotation required to align
        the object with its corresponding target.

        Object와 Target의 대칭성을 고려하여
        필요한 최소 회전 각도를 계산한다.
        """

        symmetry = {
            "circle": 360.0,
            "square": 90.0,
            "triangle": 120.0,
            "star": 72.0,
        }

        # Circle orientation does not matter.
        if shape == "circle":
            return 0.0

        # Unknown shape
        if shape not in symmetry:
            return None

        # Angle information is not available.
        if object_angle is None or target_angle is None:
            return None

        symmetry_angle = symmetry[shape]

        # ====================================================
        # Raw Rotation Difference
        # ====================================================

        delta = target_angle - object_angle

        # ====================================================
        # Apply Shape Symmetry
        # ====================================================

        delta = delta % symmetry_angle

        # ====================================================
        # Select Minimum Rotation
        # ====================================================

        if delta > symmetry_angle / 2.0:
            delta -= symmetry_angle

        return float(delta)

    # ========================================================
    # Get Matched Pair
    # ========================================================

    def get_matched_pair(self, shape):
        """
        Return object and target information for the
        requested shape.

        요청한 Shape에 해당하는 Object와 Target 정보를
        반환한다.
        """

        if shape not in self.objects:
            return None, None

        if shape not in self.targets:
            return None, None

        return (
            self.objects[shape],
            self.targets[shape]
        )

    # ========================================================
    # Test Run
    # ========================================================

    def test_run(
        self,
        shape
    ):
        """
        Test pick-up using the detected object position.

        현재는 Object 위치를 이용한 Pick-up 테스트용.
        """

        obj, target = self.get_matched_pair(shape)

        if obj is None:

            self.get_logger().warning(
                f"{shape} object is not detected."
            )

            return

        if target is None:

            self.get_logger().warning(
                f"{shape} target is not detected."
            )

            return

        # ====================================================
        # Rotation Difference
        # ====================================================

        delta_angle = self.calculate_rotation_difference(
            shape,
            obj["angle"],
            target["angle"]
        )

        self.get_logger().info(
            f"{shape} assembly information:"
        )

        self.get_logger().info(
            f"Object: "
            f"({obj['x']:.2f}, "
            f"{obj['y']:.2f}, "
            f"{obj['z']:.2f})"
        )

        self.get_logger().info(
            f"Target: "
            f"({target['x']:.2f}, "
            f"{target['y']:.2f}, "
            f"{target['z']:.2f})"
        )

        self.get_logger().info(
            f"Required Rotation: "
            f"{delta_angle:.2f} deg"
        )

        # ====================================================
        # Pick Up
        # ====================================================

        self.mu.pick_up(
            [
                obj["x"],
                obj["y"],
                obj["z"],
                100.08,
                179.98,
                100.9
            ]
        )


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = AssemblyController()
    import DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    try:
        node.robot_init.move_linear_ABS([363.80, -12.77, 396.74, 15.18, 179.83, 15.33], vel=20, acc=20)
        node.test_run(shape='Circle')
        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().warn('강제종료')

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":
    main()