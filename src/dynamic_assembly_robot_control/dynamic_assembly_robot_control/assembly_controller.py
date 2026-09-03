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
# - 현재 로봇 Pose 요청
# - 현재 로봇 Pose Publish
# - Target 접근
# - Target 위치 정렬
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
        # Latest Vision Detection
        # ====================================================

        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_z = 0.0

        self.type = ""
        self.shape = ""

        self.is_object_detected = False


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
        Vision System에서 검출 결과가 들어오면 호출된다.
        """

        self.latest_x = msg.x
        self.latest_y = msg.y
        self.latest_z = msg.z

        self.type = msg.type
        self.shape = msg.shape

        self.is_object_detected = True


        self.get_logger().info(
            f"Detected {self.type} - "
            f"Shape: {self.shape}, "
            f"X: {self.latest_x:.2f}, "
            f"Y: {self.latest_y:.2f}, "
            f"Z: {self.latest_z:.2f}"
        )


    # ========================================================
    # Test Run
    # ========================================================

    def test_run(
        self,
        is_object_detected,
        x,
        y,
        z
    ):

        if not is_object_detected:

            self.get_logger().warning(
                "Object is not detected."
            )

            return


        self.mu.pick_up(
            [
                x,
                y,
                z,
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

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":

    main()