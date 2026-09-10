# ============================================================
# assembly_controller.py
# ============================================================
# [EN]
#
# Main controller for the dynamic pick-and-place task.
#
# Current Task:
# - Detect an object
# - Pick the object
# - Detect the box target
# - Move above the center of the box
# - Drop the object into the box
#
# Future Extensions:
# - Target shape matching
# - Orientation alignment
# - Moving target tracking
# - Velocity estimation
# - Kalman Filter
#
#
# [KR]
#
# 동적 Pick-and-Place 작업을 관리하는 메인 ROS2 Node.
#
# 현재 작업:
# - Object 검출
# - Object Pick
# - Box Target 검출
# - Box 중심으로 이동
# - Box 안에 Object Drop
#
# 향후 확장:
# - Target Shape Matching
# - Orientation Alignment
# - Moving Target Tracking
# - Velocity Estimation
# - Kalman Filter
# ============================================================


import rclpy
import time
import math

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
#from sensor_msgs.msg import JointState

from assembly_interfaces.msg import DetectedObject

from .robot_init import RobotInit
from .motion_utils import MotionUtils
from .target_manager import TargetManager
from .motion_planner import MotionPlanner
from .voice_motion_handler import VoiceMotionHandler
from .db_manager import DbManager


# ============================================================
# Robot Configuration
# ============================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


# ============================================================
# Vision -> Robot Calibration Offsets
# ============================================================
#
# 현재 실험 환경에서 측정한 보정값.
#
# Shape별 offset이 아니라
# Object / Target 공통 calibration offset으로 사용한다.
# ============================================================

OBJECT_X_OFFSET = 7.0
OBJECT_Y_OFFSET = -5.0
OBJECT_Z_OFFSET = -40.0

# TARGET_X_OFFSET = -35.0   
# TARGET_Y_OFFSET = 0.0   
# TARGET_Z_OFFSET = 20.0

TARGET_X_OFFSET = 0.0   
TARGET_Y_OFFSET = 0.0   
TARGET_Z_OFFSET = 0.0


# ============================================================
# Robot Tool Orientation
# ============================================================
#
# 현재 바닥을 향하는 gripper orientation.
#
# 현재 task에서는 object / target angle을 사용하지 않고
# 고정된 수직 orientation을 유지한다.
# ============================================================

TOOL_RX = 100.08
TOOL_RY = 179.98
TOOL_RZ = 100.9


# ============================================================
# Motion Configuration
# ============================================================

PICK_APPROACH_HEIGHT = 80.0

BOX_APPROACH_HEIGHT = 100.0

# Box target Z보다 어느 정도 위에서 release할지.
#
# IMPORTANT:
# 실제 box 높이 / target Z 정의에 맞춰
# 실험 후 조정해야 한다.
BOX_DROP_HEIGHT = 20.0

BOX_RETREAT_HEIGHT = 100.0


# ============================================================
# LV3 ServoL Constant-Velocity Following Configuration
# ============================================================

LV3_TRACKING_DURATION = 1.0
LV3_MIN_MEASUREMENTS = 6

LV3_APPROACH_PREDICTION_TIME = 1.0

DROP_Z_SPEED = 60.0               # [mm/s]
SERVO_PERIOD = 0.02               # [sec] = 50 Hz
SERVO_LINEAR_VEL = 150.0          # [mm/s]
SERVO_ANGULAR_VEL = 40.0          # [deg/s]
SERVO_LINEAR_ACC = 300.0          # [mm/s^2]
SERVO_ANGULAR_ACC = 100.0         # [deg/s^2]


# ============================================================
# Robot Home Position
# ============================================================

HOME_JOINT = [
    0.0,    # J1
    0.0,    # J2
    90.0,   # J3
    0.0,    # J4
    90.0,   # J5
    0.0,    # J6
]

# ============================================================
# Camera View Poses
# ============================================================

# BOARD_TRACKING_POSE = [
#     -279.86,
#     -474.66,
#     362.31,
#     65.95,
#     -178.16,
#     157.31
# ]

BOARD_TRACKING_POSE = [
    263.76,
    -528.06,
    311.34,
    102.58,
    -177.54,
    102.93
]


OBJECT_VIEW_POSE = [
    363.80,
    -12.77,
    396.74,
    15.18,
    179.83,
    15.33
]

# ============================================================
# Assembly Controller
# ============================================================

class AssemblyController(Node):

    def __init__(self):

        super().__init__(
            "assembly_controller",
            namespace=ROBOT_ID
        )


        # ========================================================
        # 1. Detected Objects
        # ========================================================
        #
        # Shape별 최신 object 저장
        #
        # Example:
        #
        # self.objects["circle"] = {
        #     "type": "object",
        #     "shape": "circle",
        #     "x": ...,
        #     "y": ...,
        #     "z": ...,
        #     "angle": ...
        # }
        #
        # ========================================================

        self.objects = {}


        # ========================================================
        # 2. Detected Targets
        # ========================================================
        #
        # 현재 task에서는 shape matching을 사용하지 않는다.
        #
        # 하지만 future use를 위해
        # shape별 target 정보도 유지한다.
        #
        # ========================================================

        self.targets = {}


        # ========================================================
        # 3. Latest Box Target
        # ========================================================
        #
        # 현재 task:
        #
        # 가장 최근에 검출된 target을
        # Box Center로 사용한다.
        #
        # ArUco가 로봇에 의해 잠시 가려져도
        # 마지막 target을 바로 삭제하지 않는다.
        #
        # ========================================================

        self.latest_target = None

        self.latest_target_time = None

        # ========================================================
        # 3-1. Target Tracking Gate
        # ========================================================

        self.target_tracking_enabled = False


        # ========================================================
        # 4. Current Joint State
        # ========================================================
        #
        # 현재 Pick-and-Drop에서는 사용하지 않는다.
        #
        # Future:
        # orientation alignment / wrist rotation
        #
        # ========================================================

        self.current_joint_positions = None


        # ========================================================
        # 5. Robot Initialization
        # ========================================================

        self.robot_init = RobotInit(self)

        self.mu = MotionUtils(
            self.robot_init
        )


        # ========================================================
        # 6. Target Manager
        # ========================================================
        #
        # TargetManager:
        #
        # - latest target
        # - target history
        # - velocity estimation
        # - target freshness
        #
        # LV1:
        # velocity는 계산만 하고 robot control에는 사용하지 않음.
        #
        # LV3:
        # moving box prediction에 사용.
        #
        # ========================================================

        self.target_manager = TargetManager(
            node=self,
            history_size=20,
            stale_timeout=1.0,
            velocity_window=10,
            min_velocity_samples=4,
        )


        # ========================================================
        # 7. Motion Planner
        # ========================================================

        self.motion_planner = MotionPlanner(
            object_offset=[
                OBJECT_X_OFFSET,
                OBJECT_Y_OFFSET,
                OBJECT_Z_OFFSET,
            ],

            target_offset=[
                TARGET_X_OFFSET,
                TARGET_Y_OFFSET,
                TARGET_Z_OFFSET,
            ],

            tool_orientation=[
                TOOL_RX,
                TOOL_RY,
                TOOL_RZ,
            ],
        )


        # ========================================================
        # 8. Joint State Subscriber
        # ========================================================
        #
        # Future orientation-alignment용.
        #
        # ========================================================

        # self.joint_state_sub = self.create_subscription(
        #     JointState,
        #     "/dsr01/joint_states",
        #     self.joint_state_callback,
        #     10
        # )


        # ========================================================
        # 9. Vision Detection Subscriber
        # ========================================================

        self.subscription = self.create_subscription(
            DetectedObject,
            "/vision/detected_object",
            self.listener_callback,
            10
        )

        self.get_logger().info(
            "DetectedObject 구독 시작"
        )


        # ========================================================
        # 10. Robot Pose Publisher
        # ========================================================
        #
        # Vision에서 Camera -> Robot 좌표 변환에
        # robot TCP pose가 필요한 경우 사용.
        #
        # ========================================================

        self.robot_pose_pub = self.create_publisher(
            Float64MultiArray,
            "/robot/current_pose",
            10
        )


        # ========================================================
        # 11. Robot Pose Request State
        # ========================================================

        self.pose_request_pending = False


        # ========================================================
        # 12. Robot Pose Timer
        # ========================================================
        #
        # 0.1 sec = 10 Hz
        #
        # ========================================================

        self.robot_pose_timer = self.create_timer(
            0.1,
            self.request_robot_pose
        )


        # ========================================================
        # Initialization Complete
        # ========================================================

        self.get_logger().info(
            "Assembly Controller started."
        )
        # ========================================================
        # DB
        # ========================================================
        self.db = DbManager()

    # ========================================================
    # Move Robot to Home
    # ========================================================

    def move_to_home(self):

        self.get_logger().info(
            "[HOME] Moving robot to home position..."
        )

        self.robot_init.move_joint(
            HOME_JOINT,
            vel=70,
            acc=70
        )

        self.get_logger().info(
            "[HOME] Robot reached home position."
        )

    # ========================================================
    # Request Current Robot Pose
    # ========================================================

    def request_robot_pose(self):

        if self.pose_request_pending:
            return

        requested = self.robot_init.request_current_pose(
            self.robot_pose_response
        )

        if requested:
            self.pose_request_pending = True


    # ========================================================
    # Current Robot Pose Response
    # ========================================================

    def robot_pose_response(self, future):

        self.pose_request_pending = False

        try:

            response = future.result()

            if not response.success:

                self.get_logger().warning(
                    "Failed to get current robot pose."
                )

                return


            if len(response.task_pos_info) == 0:

                self.get_logger().warning(
                    "Current robot pose is empty."
                )

                return


            pose_data = response.task_pos_info[0].data


            # Expected:
            #
            # [x, y, z, rx, ry, rz, ...]
            #
            # Only first 6 values are used.

            if len(pose_data) < 6:

                self.get_logger().warning(
                    "Invalid current robot pose data."
                )

                return
            
            # self.get_logger().info(
            #     f"[ROBOT POSE] "
            #     f"x={pose_data[0]:.2f}, "
            #     f"y={pose_data[1]:.2f}, "
            #     f"z={pose_data[2]:.2f}, "
            #     f"rx={pose_data[3]:.2f}, "
            #     f"ry={pose_data[4]:.2f}, "
            #     f"rz={pose_data[5]:.2f}"
            # )

            robot_pose = [
                float(pose_data[0]),
                float(pose_data[1]),
                float(pose_data[2]),
                float(pose_data[3]),
                float(pose_data[4]),
                float(pose_data[5]),
            ]


            msg = Float64MultiArray()

            msg.data = robot_pose

            self.robot_pose_pub.publish(msg)


        except Exception as e:

            self.get_logger().error(
                f"Failed to get robot pose: {e}"
            )


    # ============================================================
    # DetectedObject Listener Callback
    # ============================================================

    def listener_callback(self, msg):

        # ========================================================
        # 1. Read Vision Message
        # ========================================================

        object_type = msg.type
        shape = msg.shape

        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)
        angle = float(msg.angle)


        # ========================================================
        # 2. Object Detection
        # ========================================================
        #
        # Object = robot이 집어야 하는 도형
        #
        # Shape별 최신 Object 위치를 저장한다.
        #
        # angle은 현재 Pick-and-Drop에서는 사용하지 않지만
        # future orientation alignment를 위해 유지한다.
        #
        # ========================================================

        if object_type == "object":

            object_info = {
                "type": "object",
                "shape": shape,
                "x": x,
                "y": y,
                "z": z,
                "angle": angle,
            }

            # Shape별 latest object 저장
            self.objects[shape] = object_info

            # self.get_logger().info(
            #     f"[OBJECT UPDATE] "
            #     f"shape={shape}, "
            #     f"xyz=({x:.1f}, {y:.1f}, {z:.1f}), "
            #     f"angle={angle:.1f}"
            # )

            return


        # ========================================================
        # 3. Target / Box Detection
        # ========================================================
        #
        # 현재 Task:
        #
        # target의 shape에 맞춰 삽입하지 않는다.
        #
        # Target은 Box의 위치를 나타내며,
        # 가장 최근 target을 box center로 사용한다.
        #
        # 그러나 shape 정보는 future use를 위해 유지한다.
        #
        # TargetManager:
        #
        #   latest target
        #   history
        #   velocity estimation
        #   timestamp
        #
        # 를 관리한다.
        #
        # ========================================================

        if object_type == "target":

            if not self.target_tracking_enabled:
                return

            # ----------------------------------------------------
            # TargetManager Update
            # ----------------------------------------------------

            target_info = self.target_manager.update_target(
                shape=shape,
                x=x,
                y=y,
                z=z,
                angle=angle,
            )


            # ----------------------------------------------------
            # Store target by shape
            #
            # Future:
            # shape-specific insertion을 다시 구현할 때 사용 가능
            # ----------------------------------------------------

            self.targets[shape] = target_info

            self.db.insert_data("target", shape , x, y, z, angle)
            self.get_logger().warn("타겟 데이터 삽입 완료")
                        
            # ----------------------------------------------------
            # Current Task:
            #
            # 가장 최근 target을 Box Center로 사용
            # ----------------------------------------------------

            self.latest_target = target_info

            # ---------------------------------
            # DB insert "object"
            # ---------------------------------


            #self.db.insert_data("target", shape, x, y, z, angle)

            self.latest_target_time = self.get_clock().now()


            # ----------------------------------------------------
            # Velocity
            #
            # 현재 LV1에서는 로봇 제어에 사용하지 않는다.
            # LV3 moving box에서 사용 예정.
            # ----------------------------------------------------

            vx, vy, vz = self.target_manager.get_velocity()


            # ----------------------------------------------------
            # Debug Log
            # ----------------------------------------------------

            # self.get_logger().info(
            #     f"[TARGET UPDATE] "
            #     f"shape={target_info['shape']}, "
            #     f"xyz=("
            #     f"{target_info['x']:.1f}, "
            #     f"{target_info['y']:.1f}, "
            #     f"{target_info['z']:.1f}), "
            #     f"angle={target_info['angle']:.1f}, "
            #     f"velocity=("
            #     f"{vx:.1f}, "
            #     f"{vy:.1f}, "
            #     f"{vz:.1f}) mm/s"
            # )

            return


        # ========================================================
        # 4. Unknown Detection Type
        # ========================================================

        # self.get_logger().warning(
        #     f"[VISION] Unknown detection type: "
        #     f"type={object_type}, "
        #     f"shape={shape}"
        # )

    # ========================================================
    # Get Object + Latest Box Target
    # ========================================================

    def get_pick_and_box_target(
        self,
        object_shape
    ):
        """
        Current Task용.

        선택한 object shape와
        가장 최근에 검출된 box target을 반환한다.

        Target shape는 현재 무시한다.
        """

        if object_shape not in self.objects:
            return None, None

        if self.latest_target is None:
            return None, None

        return (
            self.objects[object_shape],
            self.latest_target
        )


    # ========================================================
    # Get Latest Box Target
    # ========================================================

    def get_latest_target(self):
        """
        Return the last detected target.

        ArUco가 잠깐 가려져도 기존 target을
        바로 삭제하지 않는다.
        """

        return self.latest_target


    # ========================================================
    # Target Age
    # ========================================================

    def get_target_age(self):
        """
        Return how old the latest target measurement is
        in seconds.

        Future moving target tracking에서 사용 가능.
        """

        if self.latest_target_time is None:
            return None

        now = self.get_clock().now()

        age = (
            now - self.latest_target_time
        ).nanoseconds / 1e9

        return float(age)


    # ========================================================
    # Current Pick-and-Drop Task
    # ========================================================

    def run_pick_and_drop(
        self,
        shape
    ):

        # ====================================================
        # 1. Get detected object
        # ====================================================

        if shape not in self.objects:

            self.get_logger().warning(
                f"{shape} object is not detected."
            )

            return False


        obj = self.objects[shape]


        # ====================================================
        # 2. Freeze object position
        # ====================================================

        fixed_object = {
            "type": obj["type"],
            "shape": obj["shape"],
            "x": float(obj["x"]),
            "y": float(obj["y"]),
            "z": float(obj["z"]),
            "angle": float(obj["angle"]),
        }


        object_pose = [
            fixed_object["x"] + OBJECT_X_OFFSET,
            fixed_object["y"] + OBJECT_Y_OFFSET,
            fixed_object["z"] + OBJECT_Z_OFFSET,
            TOOL_RX,
            TOOL_RY,
            TOOL_RZ,
        ]


        # ====================================================
        # 3. Pick
        # ====================================================

        self.get_logger().info(
            f"[TASK] Pick object: {shape}"
        )

        self.mu.pick_up(
            object_pose,
            approach_height=PICK_APPROACH_HEIGHT,
            pick_offset=0.0,
            lift_height=100.0,
        )


        # ====================================================
        # 4. Move to Board Tracking Pose
        # ====================================================

        self.target_tracking_enabled = False

        self.get_logger().info(
            "[TASK] Moving to board tracking pose..."
        )

        self.robot_init.move_linear_ABS(
            BOARD_TRACKING_POSE,
            vel=70,
            acc=70
        )


        # ====================================================
        # 5. Stabilize robot pose before using ArUco target
        # ====================================================

        self.get_logger().info(
            "[TASK] Board tracking pose reached. "
            "Waiting for stabilization..."
        )

        settle_end = time.monotonic() + 0.8

        while rclpy.ok() and time.monotonic() < settle_end:
            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )


        # ====================================================
        # 6. Start NEW ArUco / Box Target Tracking
        # ====================================================

        self.latest_target = None
        self.latest_target_time = None
        self.targets.clear()
        self.target_manager.clear()

        self.target_tracking_enabled = True

        self.get_logger().info(
            "[TASK] ArUco box tracking ENABLED."
        )

        # ====================================================
        # 7. Collect Moving Box Measurements
        # ====================================================

        tracking_start = time.monotonic()

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

            elapsed = (
                time.monotonic()
                - tracking_start
            )

            enough_time = (
                elapsed
                >= LV3_TRACKING_DURATION
            )

            enough_samples = (
                self.target_manager.get_history_count()
                >= LV3_MIN_MEASUREMENTS
            )

            if (
                enough_time
                and enough_samples
            ):
                break


        if not self.target_manager.has_target():

            self.get_logger().warning(
                "[LV3] Box target was not detected."
            )

            return False


        # ====================================================
        # 8. Estimate Velocity
        # ====================================================

        vx, vy, vz = (
            self.target_manager.get_velocity()
        )

        # self.get_logger().info(
        #     f"[LV3 VELOCITY] "
        #     f"vx={vx:.2f}, "
        #     f"vy={vy:.2f}, "
        #     f"vz={vz:.2f} mm/s"
        # )


        # ====================================================
        # 9. Predict Initial Approach Target
        # ====================================================
        #
        # Box velocity is assumed constant.
        # We first predict where the box will be when the robot
        # reaches the initial approach area.
        # ====================================================

        approach_target = (
            self.target_manager.predict_from_now(
                future_time=LV3_APPROACH_PREDICTION_TIME
            )
        )

        if approach_target is None:

            self.get_logger().error(
                "[LV3] Cannot predict approach target."
            )

            return False


        approach_pose = [
            approach_target["x"] + TARGET_X_OFFSET,
            approach_target["y"] + TARGET_Y_OFFSET,
            approach_target["z"]
                + TARGET_Z_OFFSET
                + BOX_APPROACH_HEIGHT,
            TOOL_RX,
            TOOL_RY,
            TOOL_RZ,
        ]


        # self.get_logger().info(
        #     f"[LV3 APPROACH TARGET] "
        #     f"xyz=("
        #     f"{approach_pose[0]:.2f}, "
        #     f"{approach_pose[1]:.2f}, "
        #     f"{approach_pose[2]:.2f})"
        # )


        # ====================================================
        # 10. Freeze Vision Tracking During Robot Motion
        # ====================================================

        self.target_tracking_enabled = False


        # ====================================================
        # 11. Record Box State Before Robot Motion
        # ====================================================

        box_start = (
            self.target_manager.predict_from_now(
                future_time=0.0
            )
        )

        if box_start is None:

            self.get_logger().error(
                "[LV3] Cannot get current box state."
            )

            return False


        box_start_x = float(box_start["x"])
        box_start_y = float(box_start["y"])
        box_start_z = float(box_start["z"])


        # ====================================================
        # 12. Move To Predicted Approach Point
        # ====================================================

        approach_start_time = time.monotonic()

        self.robot_init.move_linear_ABS(
            approach_pose,
            vel=70,
            acc=70
        )

        approach_elapsed = (
            time.monotonic()
            - approach_start_time
        )


        self.get_logger().info(
            f"[LV3 APPROACH TIME] "
            f"{approach_elapsed:.3f} sec"
        )


        # ====================================================
        # 13. Predict Box Position At Actual Approach Arrival
        # ====================================================

        current_box_x = (
            box_start_x
            + vx * approach_elapsed
        )

        current_box_y = (
            box_start_y
            + vy * approach_elapsed
        )

        current_box_z = box_start_z


        # self.get_logger().info(
        #     f"[LV3 BOX AT APPROACH] "
        #     f"xyz=("
        #     f"{current_box_x:.2f}, "
        #     f"{current_box_y:.2f}, "
        #     f"{current_box_z:.2f})"
        # )


        # ====================================================
        # 14. Reposition Above Predicted Current Box Position
        # ====================================================

        follow_start_pose = [
            current_box_x + TARGET_X_OFFSET,
            current_box_y + TARGET_Y_OFFSET,
            current_box_z
                + TARGET_Z_OFFSET
                + BOX_APPROACH_HEIGHT,
            TOOL_RX,
            TOOL_RY,
            TOOL_RZ,
        ]

        correction_start = time.monotonic()

        self.robot_init.move_linear_ABS(
            follow_start_pose,
            vel=70,
            acc=70
        )

        correction_elapsed = (
            time.monotonic()
            - correction_start
        )


        # Box keeps moving during correction.
        current_box_x += (
            vx * correction_elapsed
        )

        current_box_y += (
            vy * correction_elapsed
        )


        self.get_logger().info(
            f"[LV3 CORRECTION TIME] "
            f"{correction_elapsed:.3f} sec"
        )

        # self.get_logger().info(
        #     f"[LV3 BOX BEFORE DESCENT] "
        #     f"xyz=("
        #     f"{current_box_x:.2f}, "
        #     f"{current_box_y:.2f}, "
        #     f"{current_box_z:.2f})"
        # )


        # ====================================================
        # 15. Calculate Descent Time
        # ====================================================

        approach_z = (
            current_box_z
            + TARGET_Z_OFFSET
            + BOX_APPROACH_HEIGHT
        )

        drop_z = (
            current_box_z
            + TARGET_Z_OFFSET
            + BOX_DROP_HEIGHT
        )

        vertical_distance = abs(
            approach_z
            - drop_z
        )

        if DROP_Z_SPEED <= 0.0:

            self.get_logger().error(
                "[LV3] DROP_Z_SPEED must be > 0."
            )

            return False

        descent_time = (
            vertical_distance
            / DROP_Z_SPEED
        )


        # ====================================================
        # 16. ServoL Constant-Velocity Following Drop
        # ====================================================
        #
        # Box velocity is assumed constant:
        #   x(t) = x0 + vx * t
        #   y(t) = y0 + vy * t
        #
        # During descent:
        #   robot vx = box vx
        #   robot vy = box vy
        #   robot vz = -DROP_Z_SPEED
        #
        # New Cartesian targets are streamed at 50 Hz.
        # ====================================================

        # self.get_logger().info(
        #     f"[LV3 SERVOL START] "
        #     f"vx={vx:.2f}, "
        #     f"vy={vy:.2f}, "
        #     f"z_speed={DROP_Z_SPEED:.2f}, "
        #     f"period={SERVO_PERIOD:.3f}s, "
        #     f"descent_time={descent_time:.3f}s"
        # )

        servo_start_time = time.monotonic()

        while rclpy.ok():

            elapsed = (
                time.monotonic()
                - servo_start_time
            )

            if elapsed >= descent_time:
                break

            servo_x = (
                current_box_x
                + vx * elapsed
                + TARGET_X_OFFSET
            )

            servo_y = (
                current_box_y
                + vy * elapsed
                + TARGET_Y_OFFSET
            )

            servo_z = (
                approach_z
                - DROP_Z_SPEED * elapsed
            )

            if servo_z < drop_z:
                servo_z = drop_z

            servo_pose = [
                servo_x,
                servo_y,
                servo_z,
                TOOL_RX,
                TOOL_RY,
                TOOL_RZ,
            ]

            self.robot_init.servo_linear_ABS(
                servo_pose,
                vel_linear=SERVO_LINEAR_VEL,
                vel_angular=SERVO_ANGULAR_VEL,
                acc_linear=SERVO_LINEAR_ACC,
                acc_angular=SERVO_ANGULAR_ACC,
            )

            time.sleep(
                SERVO_PERIOD
            )

        # ====================================================
        # 17. Final drop + keep following during release
        # ====================================================

        release_follow_time = 0.30

        release_start_time = time.monotonic()

        gripper_open_sent = False

        while rclpy.ok():

            release_elapsed = (
                time.monotonic()
                - release_start_time
            )

            if release_elapsed >= release_follow_time:
                break


            total_elapsed = (
                descent_time
                + release_elapsed
            )


            servo_x = (
                current_box_x
                + vx * total_elapsed
                + TARGET_X_OFFSET
            )

            servo_y = (
                current_box_y
                + vy * total_elapsed
                + TARGET_Y_OFFSET
            )

            servo_z = drop_z


            servo_pose = [
                servo_x,
                servo_y,
                servo_z,
                TOOL_RX,
                TOOL_RY,
                TOOL_RZ,
            ]


            self.robot_init.servo_linear_ABS(
                servo_pose,
                vel_linear=SERVO_LINEAR_VEL,
                vel_angular=SERVO_ANGULAR_VEL,
                acc_linear=SERVO_LINEAR_ACC,
                acc_angular=SERVO_ANGULAR_ACC,
            )


            if not gripper_open_sent:

                self.robot_init.open_gripper_nowait()

                gripper_open_sent = True


            time.sleep(
                SERVO_PERIOD
            )

        # ====================================================
        # 19. Retreat
        # ====================================================

        retreat_pose = [
            final_drop_pose[0],
            final_drop_pose[1],
            current_box_z
                + TARGET_Z_OFFSET
                + BOX_RETREAT_HEIGHT,
            TOOL_RX,
            TOOL_RY,
            TOOL_RZ,
        ]

        self.robot_init.move_linear_ABS(
            retreat_pose,
            vel=70,
            acc=70
        )

        self.get_logger().info(
            "[LV3 DROP] ServoL following drop completed."
        )

        return True

    # ========================================================
    # Joint State Callback
    # ========================================================
    #
    # Future orientation-alignment use.
    # ========================================================

    # def joint_state_callback(
    #     self,
    #     msg: JointState
    # ):

    #     if len(msg.position) < 6:
    #         return


    #     # ROS JointState position: rad
    #     #
    #     # Doosan posj: degree

    #     self.current_joint_positions = [
    #         float(p)
    #         * 180.0
    #         / 3.141592653589793

    #         for p in msg.position[:6]
    #     ]


    # ========================================================
    # FUTURE:
    # Shape Matching
    # ========================================================
    #
    # 현재 task에서는 사용하지 않는다.
    #
    # 나중에 shape-specific insertion을 다시 구현할 경우
    # 사용할 수 있도록 남겨둔다.
    # ========================================================

    def check_shape_match(
        self,
        shape
    ):

        if shape not in self.objects:
            return

        if shape not in self.targets:
            return


        obj = self.objects[shape]

        target = self.targets[shape]


        delta_angle = (
            self.calculate_rotation_difference(
                shape,
                obj["angle"],
                target["angle"]
            )
        )


        if delta_angle is None:

            self.get_logger().warning(
                f"Cannot calculate rotation for: "
                f"{shape}"
            )

            return


        # self.get_logger().info(
        #     f"[MATCH] {shape} | "
        #     f"Object Angle: "
        #     f"{obj['angle']:.2f} deg | "
        #     f"Target Angle: "
        #     f"{target['angle']:.2f} deg | "
        #     f"Rotation: "
        #     f"{delta_angle:.2f} deg"
        # )


    # ========================================================
    # FUTURE:
    # Rotation Difference
    # ========================================================

    def calculate_rotation_difference(
        self,
        shape,
        object_angle,
        target_angle
    ):

        symmetry = {
            "circle": 360.0,
            "square": 90.0,
            "triangle": 120.0,
            "star": 72.0,
        }


        if shape == "circle":
            return 0.0


        if shape not in symmetry:
            return None


        if (
            object_angle is None
            or target_angle is None
        ):
            return None


        symmetry_angle = symmetry[shape]

        delta = (
            target_angle
            - object_angle
        )

        delta = (
            delta
            % symmetry_angle
        )


        if delta > symmetry_angle / 2.0:

            delta -= symmetry_angle


        return float(delta)


    # ========================================================
    # FUTURE:
    # Get Shape-Matched Pair
    # ========================================================

    def get_matched_pair(
        self,
        shape
    ):

        if shape not in self.objects:
            return None, None

        if shape not in self.targets:
            return None, None


        return (
            self.objects[shape],
            self.targets[shape]
        )


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )


    node = AssemblyController()
    voice_handler = VoiceMotionHandler(node)

    # ========================================================
    # Doosan Robot Configuration
    # ========================================================

    import DR_init

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node


    try:

        node.robot_init.move_linear_ABS(
            OBJECT_VIEW_POSE, vel=70, acc=70
        )

        # ====================================================
        # Initial Gripper State
        # ====================================================

        node.robot_init.open_gripper()


        # ====================================================
        # Select Object Shape
        # ====================================================
        #
        # Target shape와는 관계 없음.
        #
        # Object pick selection 용도.
        # ====================================================

        voice_handler.request_shape()
        shape = voice_handler.keyword

        node.get_logger().info(
            f"{shape} object 검출 대기 중..."
        )
        if not shape:
            node.get_logger().warn(
                "음성 인식 실패: 도형을 선택하지 못했습니다.-작업중단"
            )
            shape = "circle"



        # ====================================================
        # Wait for Object ONLY
        # ====================================================

        node.target_tracking_enabled = False

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

            if shape in node.objects:
                break


        # ====================================================
        # Run Current Task
        # ====================================================

        node.run_pick_and_drop(
            shape
        )


        # ====================================================
        # Keep Node Alive
        # ====================================================

        rclpy.spin(node)


    except KeyboardInterrupt:

        node.get_logger().warning(
            "강제종료"
        )


    finally:
        if hasattr(node, 'db'):
            node.db.close()

        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":

    main()