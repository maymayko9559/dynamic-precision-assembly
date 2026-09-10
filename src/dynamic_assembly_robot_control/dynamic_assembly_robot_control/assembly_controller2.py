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
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
#from sensor_msgs.msg import JointState

from dsr_msgs2.srv import GetCurrentPosx

from assembly_interfaces.msg import DetectedObject
from assembly_interfaces.msg import PredictedTarget

from .robot_init import RobotInit
from .motion_utils import MotionUtils
from .motion_planner import MotionPlanner
from .voice_motion_handler import VoiceMotionHandler


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

# tracking 된 box Z 기준, 이만큼 "위"에서 그리퍼를 연다. 음수면 그 아래.
# 툴 장착 후 10cm 위로 조정: -110 -> -10.
# 부딪히면 올리고(값 ↑), 덜 들어가면 더 내린다(값 ↓).
BOX_DROP_HEIGHT = -10.0

BOX_RETREAT_HEIGHT = 100.0


# ============================================================
# LV3 Constant-Velocity Following Configuration
# ============================================================

# fix 를 잡는 시간. 짧으면 빨리 시작하지만 KF 속도(vx,vy)가 아직
# 수렴 전이라 "따라가면서 내려놓기" 가 안 된다. 1초는 줘야 속도가 선다.
LV3_TRACKING_DURATION = 1.0   # [s] 최소 수집 시간
LV3_MIN_MEASUREMENTS = 8      # coast 작은 메시지 이만큼 쌓이면 진행

# tracking_node.PredictedTarget.coast 가 이 값보다 크면
# "상자를 지금 안 보고 등속 예측만 하는 중" -> 신뢰하지 않는다.
# ArUco 검출이 좀 끊겨도 최근에 봤으면 통과하도록 살짝 완화.
LV3_MAX_COAST = 0.6          # [s]

# 이 안에 fix 를 못 잡으면: target 이 아예 없으면 중단,
# 있는데 조금 오래됐을 뿐이면 경고 후 진행 (follow 루프가 재획득함).
LV3_TRACKING_TIMEOUT = 6.0   # [s]
LV3_TIMEOUT_ACCEPT_COAST = 2.0   # [s] 타임아웃 시 이 이하 coast 면 그냥 진행

# 추정 XY 속도가 이 미만이면 "정지 상자" 로 보고 vx=vy=0 (따라가기 OFF).
# 벨트가 느리면(예: 20~30mm/s) 이 값이 크면 실제 이동을 죽여버리므로 낮게.
LV3_STATIC_SPEED = 5.0       # [mm/s]

# Approximate time used only for the initial intercept prediction.
LV3_APPROACH_PREDICTION_TIME = 2.0

# Descent (step 18): 한 번의 연속 movel 로 approach_z -> drop_z 까지 내려가며
# XY 는 [vx, vy, -DROP_Z_SPEED] 방향으로 이동 -> 내려가는 내내 상자를 따라감.
# acc 가 낮으면 가감속 구간에서 상자를 놓치므로 넉넉히.
DROP_Z_SPEED = 40.0       # [mm/s] 수직 하강 속도 (빠르게 하려면 ↑)
MOVING_DROP_ACC = 300.0   # [mm/s^2]


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

BOARD_TRACKING_POSE = [
    260.75,
    -527.91,
    309.92,
    88.63,
    -178.76,
    88.99
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
# Shared State  (SensorNode <-> AssemblyController)
# ============================================================
#
# SensorNode(백그라운드 executor 스레드)가 쓰고,
# AssemblyController(메인 스레드, 블로킹 movel)가 읽는다.
# 두 스레드가 동시에 건드리므로 lock 으로 보호한다.
# ============================================================

class TargetState:

    def __init__(self):
        self.lock = threading.Lock()

        # shape -> object dict  (pick 대상)
        self.objects = {}
        # shape -> target dict  (future shape-matching 용)
        self.targets = {}

        # 마지막 PredictedTarget 스냅샷
        self.latest_target = None
        self.latest_target_time = None      # rclpy.time.Time

        # tracking 재시작 이후 받은 target 메시지 수 (수렴 대기용)
        self.target_rx_count = 0
        # 그 중 coast <= LV3_MAX_COAST (상자를 실제로 보고 있는) 메시지 수
        self.fresh_rx_count = 0

        # run_pick_and_drop 이 로봇 이동 중 tracking 을 얼릴 때 사용
        self.tracking_enabled = False

    def reset_target(self):
        with self.lock:
            self.latest_target = None
            self.latest_target_time = None
            self.target_rx_count = 0
            self.fresh_rx_count = 0
            self.targets.clear()


# ============================================================
# Sensor Node
# ============================================================
#
# 목적: AssemblyController 가 블로킹 movel 로 멈춰 있는 동안에도
#       - /robot/current_pose 를 계속 발행 (vision_manager 가 eye-in-hand
#         변환에 사용 -> 이동 중 stale pose 문제 제거)
#       - /tracking/predicted_target, /vision/detected_object 콜백을 계속 처리
#       해야 한다.
#
# 그래서 이 노드만 MultiThreadedExecutor 로 백그라운드에서 spin 한다.
# DSR_ROBOT2 모션 함수(movel 등)는 g_node(AssemblyController)를 직접
# spin_until_future_complete 하므로, 그 노드는 executor 에 넣지 않는다.
# 이 노드는 DSR wrapper 를 쓰지 않고 raw async 서비스 클라이언트만 쓴다.
# ============================================================

class SensorNode(Node):

    def __init__(self, state: TargetState):

        super().__init__(
            "assembly_sensor_node",
            namespace=ROBOT_ID
        )

        self.state = state

        # ----- 구독 -----
        self.object_sub = self.create_subscription(
            DetectedObject,
            "/vision/detected_object",
            self.object_callback,
            10
        )

        self.target_sub = self.create_subscription(
            PredictedTarget,
            "/tracking/predicted_target",
            self.target_callback,
            10
        )

        # ----- 로봇 pose 발행 -----
        self.robot_pose_pub = self.create_publisher(
            Float64MultiArray,
            "/robot/current_pose",
            10
        )

        self.posx_client = self.create_client(
            GetCurrentPosx,
            "/dsr01/dsr_controller2/aux_control/get_current_posx"
        )

        self.pose_request_pending = False

        self.pose_timer = self.create_timer(
            0.1,                       # 10 Hz
            self.request_robot_pose
        )

        self.get_logger().info(
            "SensorNode: object/target 구독 + /robot/current_pose 발행 시작"
        )

    # --------------------------------------------------------
    # Object Callback  (vision_manager -> /vision/detected_object)
    # --------------------------------------------------------
    #
    # 이 토픽엔 type="target"(box)도 오지만 box 위치는 tracking_node 것을
    # 쓰므로 여기서는 무시한다.
    # --------------------------------------------------------

    def object_callback(self, msg: DetectedObject):

        if msg.type != "object":
            return

        info = {
            "type": "object",
            "shape": msg.shape,
            "x": float(msg.x),
            "y": float(msg.y),
            "z": float(msg.z),
            "angle": float(msg.angle),
        }

        with self.state.lock:
            self.state.objects[msg.shape] = info

        self.get_logger().info(
            f"[OBJECT UPDATE] shape={msg.shape}, "
            f"xyz=({info['x']:.1f}, {info['y']:.1f}, {info['z']:.1f}), "
            f"angle={info['angle']:.1f}",
            throttle_duration_sec=0.5,
        )

    # --------------------------------------------------------
    # Target Callback  (tracking_node -> /tracking/predicted_target)
    # --------------------------------------------------------
    #
    # tracking_node(칼만필터) 결과를 스냅샷으로 저장만 한다.
    # 별도 필터/속도추정 없음.
    # --------------------------------------------------------

    def target_callback(self, msg: PredictedTarget):

        if not self.state.tracking_enabled:
            return

        coast = float(msg.coast)
        valid = bool(msg.valid)

        info = {
            "shape": msg.shape,
            "x": float(msg.x),
            "y": float(msg.y),
            "z": float(msg.z),
            "angle": float(msg.angle),
            "vx": float(msg.vx),
            "vy": float(msg.vy),
            "vz": float(msg.vz),
            "prediction_time": float(msg.prediction_time),
            "coast": coast,
            "valid": valid,
        }

        with self.state.lock:
            self.state.targets[msg.shape] = info
            self.state.latest_target = info
            self.state.latest_target_time = self.get_clock().now()
            self.state.target_rx_count += 1
            if valid and coast <= LV3_MAX_COAST:
                self.state.fresh_rx_count += 1

        self.get_logger().info(
            f"[TARGET UPDATE] shape={info['shape']}, "
            f"xyz=({info['x']:.1f}, {info['y']:.1f}, {info['z']:.1f}), "
            f"vel=({info['vx']:.1f}, {info['vy']:.1f}, {info['vz']:.1f}) mm/s, "
            f"coast={coast:.2f}s",
            throttle_duration_sec=0.5,
        )

    # --------------------------------------------------------
    # Robot Pose  (raw async 서비스, DSR wrapper 미사용)
    # --------------------------------------------------------

    def request_robot_pose(self):

        if self.pose_request_pending:
            return

        if not self.posx_client.service_is_ready():
            self.get_logger().warning(
                "Waiting for get_current_posx service...",
                throttle_duration_sec=2.0
            )
            return

        req = GetCurrentPosx.Request()
        req.ref = 0        # DR_BASE

        future = self.posx_client.call_async(req)
        future.add_done_callback(self.robot_pose_response)
        self.pose_request_pending = True

    def robot_pose_response(self, future):

        self.pose_request_pending = False

        try:
            response = future.result()

            if not response.success:
                self.get_logger().warning(
                    "Failed to get current robot pose.",
                    throttle_duration_sec=2.0
                )
                return

            if len(response.task_pos_info) == 0:
                self.get_logger().warning(
                    "Current robot pose is empty.",
                    throttle_duration_sec=2.0
                )
                return

            pose_data = response.task_pos_info[0].data

            if len(pose_data) < 6:
                self.get_logger().warning("Invalid current robot pose data.")
                return

            msg = Float64MultiArray()
            msg.data = [float(pose_data[i]) for i in range(6)]
            self.robot_pose_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose: {e}")


# ============================================================
# Assembly Controller
# ============================================================

class AssemblyController2(Node):

    def __init__(self, state: TargetState):

        super().__init__(
            "assembly_controller2",
            namespace=ROBOT_ID
        )


        # ========================================================
        # 1~3. Shared state  (SensorNode 가 채운다)
        # ========================================================
        #
        # objects / targets / latest_target / latest_target_time /
        # target_rx_count / tracking_enabled 는 전부 TargetState 안에 있고
        # lock 으로 보호된다. (SensorNode 스레드가 write, 여기서 read)
        #
        #   state.objects[shape]       = {type,shape,x,y,z,angle}
        #   state.latest_target        = {shape,x,y,z,angle,vx,vy,vz,prediction_time}
        #
        # 위치/속도/미래예측은 아래 헬퍼(get_velocity / predict_target_from_now)가
        # 이 스냅샷 하나로 수행한다. (기존 TargetManager 역할 = tracking_node + 스냅샷)
        # ========================================================

        self.state = state

        # 기존 코드 호환용 alias (같은 dict 객체를 가리킴)
        self.objects = state.objects
        self.targets = state.targets

        # tracking_node 필터 리셋 채널 (publisher 는 spin 없이도 동작)
        self.tracking_reset_pub = self.create_publisher(
            Empty, "/tracking/reset", 10
        )


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
        # 6. Target estimation
        # ========================================================
        #
        # TargetManager 제거됨.
        # 위치/속도/미래예측은 tracking_node 의 칼만필터가 담당하고,
        # 이 노드는 그 결과(PredictedTarget)를 스냅샷으로 받아
        # get_velocity() / predict_target_from_now() 로만 사용한다.
        # ========================================================


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
        # 9. Detection 구독 / 로봇 pose 발행 -> SensorNode 로 이동
        # ========================================================
        #
        # object/target 구독과 /robot/current_pose 발행은 SensorNode 가
        # 백그라운드 MultiThreadedExecutor 에서 담당한다.
        # 이 노드(AssemblyController)는 블로킹 movel 동안 spin 되지 않으므로
        # 여기에 콜백/타이머를 두면 이동 중 멈춘다. (이전 LV3 실패 원인)
        # ========================================================


        # ========================================================
        # Initialization Complete
        # ========================================================

        self.get_logger().info(
            "Assembly Controller started."
        )

    # ========================================================
    # Move Robot to Home
    # ========================================================

    def move_to_home(self):

        self.get_logger().info(
            "[HOME] Moving robot to home position..."
        )

        self.robot_init.move_joint(
            HOME_JOINT,
            vel=30,
            acc=30
        )

        self.get_logger().info(
            "[HOME] Robot reached home position."
        )

    # ========================================================
    # Target estimation helpers  (기존 TargetManager 대체)
    # ========================================================
    #
    # 데이터는 SensorNode 가 TargetState 에 채운다. 여기서는 lock 을
    # 잡고 스냅샷을 복사한 뒤 계산만 한다. (SensorNode 스레드와 동시 접근)
    # ========================================================

    def get_velocity(self):
        """
        tracking_node 칼만필터가 추정한 box 속도 [mm/s].
        target 이 아직 없으면 (0, 0, 0).
        """
        with self.state.lock:
            t = self.state.latest_target
            if t is None:
                return (0.0, 0.0, 0.0)
            return (t["vx"], t["vy"], t["vz"])

    def predict_target_from_now(self, future_time=0.0, vel=None):
        """
        지금으로부터 future_time 초 뒤의 box 위치를 등속 외삽으로 예측.

        latest_target 의 (x,y,z) 는 (수신시각 + prediction_time) 시점 위치이므로
        지금(now)+future_time 까지의 실제 외삽 구간은:
            dt = age + future_time - prediction_time
        (age = 마지막 target 수신 이후 흐른 시간)

        vel: (vx,vy,vz) 를 넘기면 스냅샷 속도 대신 이 값으로 XY 외삽.
             (step 8 에서 정지 상자로 판정해 0 으로 만든 속도를 그대로 쓰기 위함)

        NOTE: Z 는 외삽하지 않는다. 상자는 컨베이어 위에서 수평 이동만 하므로
        벨트면 높이(z)는 사실상 일정하다. 노이즈 섞인 vz 를 곱하면
        drop_z 가 흔들려서 "너무 높게 놓거나 하강을 안 하는" 문제가 생긴다.
        -> z 는 항상 마지막으로 측정된 값을 그대로 쓴다.
        """
        with self.state.lock:
            t = self.state.latest_target
            t_time = self.state.latest_target_time
            if t is None or t_time is None:
                return None
            t = dict(t)   # 스냅샷 복사 후 lock 밖에서 계산

        if vel is None:
            vx, vy = t["vx"], t["vy"]
        else:
            vx, vy = vel[0], vel[1]

        age = (
            self.get_clock().now() - t_time
        ).nanoseconds / 1e9

        dt = age + float(future_time) - t["prediction_time"]

        return {
            "shape": t["shape"],
            "x": t["x"] + vx * dt,
            "y": t["y"] + vy * dt,
            "z": t["z"],                 # Z 는 외삽 안 함 (벨트면 높이는 일정)
            "angle": t["angle"],
            "prediction_time": float(future_time),
            "predicted": True,
        }

    def get_latest_target_snapshot(self):
        """lock 을 잡고 latest_target dict 사본을 반환 (없으면 None)."""
        with self.state.lock:
            t = self.state.latest_target
            return dict(t) if t is not None else None

    def wait_object_detected(self, shape):
        """해당 shape object 가 감지됐는지 (SensorNode 가 채움)."""
        with self.state.lock:
            return shape in self.state.objects

    def get_object_snapshot(self, shape):
        with self.state.lock:
            o = self.state.objects.get(shape)
            return dict(o) if o is not None else None

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

        obj = self.get_object_snapshot(object_shape)
        target = self.get_latest_target_snapshot()

        if obj is None or target is None:
            return None, None

        return (obj, target)


    # ========================================================
    # Get Latest Box Target
    # ========================================================

    def get_latest_target(self):
        """
        Return the last detected target snapshot (dict) or None.

        ArUco가 잠깐 가려져도 tracking_node 가 등속 예측으로
        계속 발행하므로 기존 target을 바로 삭제하지 않는다.
        """

        return self.get_latest_target_snapshot()


    # ========================================================
    # Target Age
    # ========================================================

    def get_target_age(self):
        """
        Return how old the latest target measurement is
        in seconds.
        """

        with self.state.lock:
            t_time = self.state.latest_target_time

        if t_time is None:
            return None

        age = (
            self.get_clock().now() - t_time
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
        #
        # SensorNode 는 백그라운드 executor 에서 계속 돌기 때문에
        # 이 블로킹 movel 동안에도 /robot/current_pose 발행과
        # target 콜백이 멈추지 않는다. 여기서는 그냥 time.sleep 으로 기다린다.
        # ====================================================

        self.state.tracking_enabled = False

        self.get_logger().info(
            "[TASK] Moving to board tracking pose..."
        )

        self.robot_init.move_linear_ABS(
            BOARD_TRACKING_POSE,
            vel=40,
            acc=50
        )


        # ====================================================
        # 5. Reset tracking + stabilize
        # ====================================================
        #
        # tracking_node 필터를 리셋해서 로봇 이동 중 생긴 transient 속도를
        # 버리고, 정지한 상태에서 다음 측정부터 속도 0 으로 다시 수렴시킨다.
        # ====================================================

        self.get_logger().info(
            "[TASK] Board tracking pose reached. Resetting tracker..."
        )

        self.state.reset_target()
        self.state.tracking_enabled = True
        # reset 메시지가 tracking_node 에 확실히 도달하도록 두 번 (publisher 는 spin 불필요)
        self.tracking_reset_pub.publish(Empty())
        time.sleep(0.1)
        self.tracking_reset_pub.publish(Empty())

        time.sleep(0.6)     # pose 릴레이 안정 + 필터 재초기화 대기


        # ====================================================
        # 6~7. Wait for first fresh box fix
        # ====================================================
        #
        # tracking_node 는 상자를 못 봐도 50Hz 로 등속 예측을 계속 발행한다.
        # 그래서 "메시지 수" 가 아니라 "coast 가 작은(=실제로 보고 있는)"
        # 메시지가 몇 개 쌓였는지로 판단한다.
        #
        # 타임아웃 시:
        #   - target 자체가 없음        -> 중단 (vision/보드 확인)
        #   - target 있고 coast 적당함   -> 경고 후 진행 (follow 루프가 재획득)
        # ====================================================

        self.get_logger().info(
            "[TASK] ArUco box tracking ENABLED. Waiting for first fresh fix..."
        )

        tracking_start = time.monotonic()

        while rclpy.ok():

            time.sleep(0.02)

            elapsed = time.monotonic() - tracking_start

            snap = self.get_latest_target_snapshot()
            with self.state.lock:
                fresh_rx = self.state.fresh_rx_count

            coast = snap.get("coast", 99.0) if snap is not None else 99.0
            valid = bool(snap.get("valid", False)) if snap is not None else False
            fresh_now = valid and coast <= LV3_MAX_COAST

            # 정상 획득
            if (
                elapsed >= LV3_TRACKING_DURATION
                and fresh_rx >= LV3_MIN_MEASUREMENTS
                and fresh_now
            ):
                self.get_logger().info(
                    f"[LV3] first fix in {elapsed:.2f}s "
                    f"(fresh_rx={fresh_rx}, coast={coast:.2f}s)"
                )
                break

            # 타임아웃
            if elapsed > LV3_TRACKING_TIMEOUT:
                if snap is not None and coast <= LV3_TIMEOUT_ACCEPT_COAST:
                    self.get_logger().warn(
                        f"[LV3] {elapsed:.1f}s 경과, 완전한 fresh fix 는 아니지만 "
                        f"target 있음 (coast={coast:.2f}s) -> 진행 "
                        f"(follow 루프가 재획득)"
                    )
                    break
                self.get_logger().error(
                    f"[LV3] {elapsed:.1f}s 안에 쓸만한 target 이 안 잡힘 "
                    f"(fresh_rx={fresh_rx}, coast={coast:.2f}s, valid={valid}). "
                    f"카메라가 ArUco 보드(ID 0~3)를 보는지 확인 필요. -> 작업 중단"
                )
                return False


        # ====================================================
        # 8. Estimate Velocity  (tracking_node 칼만필터 상태 속도)
        # ====================================================

        vx, vy, vz = self.get_velocity()

        # 벨트는 수평 이동만 하므로 정지 판정은 XY 속도로만 한다 (vz 노이즈 무시)
        speed = math.hypot(vx, vy)

        self.get_logger().info(
            f"[LV3 VELOCITY] "
            f"vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} mm/s  "
            f"|v_xy|={speed:.2f}"
        )

        # 정지 상자: 미세 노이즈가 ×예측시간 으로 증폭되지 않게 0 으로 고정
        if speed < LV3_STATIC_SPEED:
            self.get_logger().info(
                f"[LV3 VELOCITY] |v| < {LV3_STATIC_SPEED} mm/s "
                f"-> 정지 상자로 간주, 예측 외삽 OFF"
            )
            vx = vy = vz = 0.0


        # ====================================================
        # 9. Predict Initial Approach Target
        # ====================================================
        #
        # Box velocity is assumed constant.
        # We first predict where the box will be when the robot
        # reaches the initial approach area.
        # ====================================================

        approach_target = self.predict_target_from_now(
            future_time=LV3_APPROACH_PREDICTION_TIME,
            vel=(vx, vy, vz),        # step 8 에서 deadband 적용된 속도
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


        self.get_logger().info(
            f"[LV3 APPROACH TARGET] "
            f"xyz=("
            f"{approach_pose[0]:.2f}, "
            f"{approach_pose[1]:.2f}, "
            f"{approach_pose[2]:.2f})"
        )


        # ====================================================
        # 10. Freeze tracking during robot motion
        # ====================================================
        #
        # 여기서부터는 카메라를 안 쓰고, 잡아둔 위치 + 속도로
        # 등속 dead-reckoning 만 한다. (open-loop)
        # 내려가면서 카메라가 ArUco 를 놓쳐도 상관없다.
        # ====================================================

        self.state.tracking_enabled = False


        # ====================================================
        # 11. Record box state before robot motion
        # ====================================================

        box_start = self.predict_target_from_now(
            future_time=0.0,
            vel=(vx, vy, vz),
        )

        if box_start is None:
            self.get_logger().error("[LV3] Cannot get current box state.")
            return False

        box_start_x = float(box_start["x"])
        box_start_y = float(box_start["y"])
        box_start_z = float(box_start["z"])


        # ====================================================
        # 12. Move to predicted approach point
        # ====================================================

        approach_start_time = time.monotonic()

        self.robot_init.move_linear_ABS(
            approach_pose,
            vel=40,
            acc=50
        )

        approach_elapsed = time.monotonic() - approach_start_time

        self.get_logger().info(
            f"[LV3 APPROACH TIME] {approach_elapsed:.3f} sec"
        )


        # ====================================================
        # 13. Predict box position at actual approach arrival
        # ====================================================

        current_box_x = box_start_x + vx * approach_elapsed
        current_box_y = box_start_y + vy * approach_elapsed
        current_box_z = box_start_z

        self.get_logger().info(
            f"[LV3 BOX AT APPROACH] xyz=("
            f"{current_box_x:.2f}, {current_box_y:.2f}, {current_box_z:.2f})"
        )


        # ====================================================
        # 14. Reposition above predicted current box position
        # ====================================================

        follow_start_pose = [
            current_box_x + TARGET_X_OFFSET,
            current_box_y + TARGET_Y_OFFSET,
            current_box_z + TARGET_Z_OFFSET + BOX_APPROACH_HEIGHT,
            TOOL_RX,
            TOOL_RY,
            TOOL_RZ,
        ]

        correction_start = time.monotonic()

        self.robot_init.move_linear_ABS(
            follow_start_pose,
            vel=40,
            acc=50
        )

        correction_elapsed = time.monotonic() - correction_start

        # Box keeps moving during correction.
        current_box_x += vx * correction_elapsed
        current_box_y += vy * correction_elapsed

        self.get_logger().info(
            f"[LV3 BOX BEFORE DESCENT] xyz=("
            f"{current_box_x:.2f}, {current_box_y:.2f}, {current_box_z:.2f})"
        )


        # ====================================================
        # 15. Descent time
        # ====================================================

        approach_z = current_box_z + TARGET_Z_OFFSET + BOX_APPROACH_HEIGHT
        drop_z = current_box_z + TARGET_Z_OFFSET + BOX_DROP_HEIGHT

        vertical_distance = abs(approach_z - drop_z)

        if DROP_Z_SPEED <= 0.0:
            self.get_logger().error("[LV3] DROP_Z_SPEED must be > 0.")
            return False

        descent_time = vertical_distance / DROP_Z_SPEED


        # ====================================================
        # 16. Moving drop endpoint
        # ====================================================

        drop_pose = [
            current_box_x + vx * descent_time + TARGET_X_OFFSET,
            current_box_y + vy * descent_time + TARGET_Y_OFFSET,
            drop_z,
            TOOL_RX,
            TOOL_RY,
            TOOL_RZ,
        ]


        # ====================================================
        # 17. Cartesian path velocity  [vx, vy, -DROP_Z_SPEED]
        # ====================================================

        moving_drop_vel = math.sqrt(
            vx * vx + vy * vy + DROP_Z_SPEED * DROP_Z_SPEED
        )

        self.get_logger().info(
            f"[LV3 DROP] follow_v=({vx:.1f},{vy:.1f}) mm/s  "
            f"endpoint xyz=("
            f"{drop_pose[0]:.2f}, {drop_pose[1]:.2f}, {drop_pose[2]:.2f})  "
            f"path_vel={moving_drop_vel:.1f}  descent={vertical_distance:.1f}mm "
            f"/{descent_time:.2f}s"
        )
        if abs(vx) < 1.0 and abs(vy) < 1.0:
            self.get_logger().warn(
                "[LV3 DROP] vx,vy≈0 -> 수직 하강만 함(따라가기 없음). "
                "상자가 움직이는 중이면 [LV3 VELOCITY] 로그 확인: "
                "KF 속도가 0 이면 tracking 수집시간/검출 문제, "
                "LV3_STATIC_SPEED 로 죽은 거면 값 더 낮추기."
            )


        # ====================================================
        # 18. Follow box while descending  (single continuous move)
        # ====================================================

        self.robot_init.move_linear_ABS(
            drop_pose,
            vel=moving_drop_vel,
            acc=MOVING_DROP_ACC
        )


        # ====================================================
        # 19. Release
        # ====================================================

        self.get_logger().info("[LV3 DROP] Releasing object.")
        self.robot_init.open_gripper_nowait()
        


        # ====================================================
        # 20. Retreat
        # ====================================================

        self.robot_init.move_linear_ABS(
            [drop_pose[0], drop_pose[1],
             current_box_z + TARGET_Z_OFFSET + BOX_RETREAT_HEIGHT,
             TOOL_RX, TOOL_RY, TOOL_RZ],
            vel=40,
            acc=50
        )

        self.get_logger().info(
            "[LV3 DROP] Constant-velocity following drop completed."
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


        self.get_logger().info(
            f"[MATCH] {shape} | "
            f"Object Angle: "
            f"{obj['angle']:.2f} deg | "
            f"Target Angle: "
            f"{target['angle']:.2f} deg | "
            f"Rotation: "
            f"{delta_angle:.2f} deg"
        )


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

    # ========================================================
    # 공유 상태 + 2개 노드
    # ========================================================
    #
    #   SensorNode          : object/target 구독 + /robot/current_pose 발행.
    #                         백그라운드 MultiThreadedExecutor 스레드에서 spin.
    #   AssemblyController   : 태스크 로직 + 블로킹 DSR 모션.
    #                         메인 스레드에서 실행. executor 에 넣지 않는다
    #                         (DSR_ROBOT2 movel 이 이 노드를
    #                          spin_until_future_complete 로 직접 spin 하므로).
    # ========================================================

    state = TargetState()

    sensor_node = SensorNode(state)
    node = AssemblyController2(state)

    voice_handler = VoiceMotionHandler(node)

    # ========================================================
    # Doosan Robot Configuration
    # ========================================================

    import DR_init

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node


    # ========================================================
    # SensorNode 를 백그라운드에서 spin
    # ========================================================

    sensor_exec = MultiThreadedExecutor(num_threads=2)
    sensor_exec.add_node(sensor_node)

    spin_thread = threading.Thread(
        target=sensor_exec.spin,
        daemon=True,
    )
    spin_thread.start()


    try:

        node.robot_init.move_linear_ABS(
            OBJECT_VIEW_POSE, vel=40, acc=50
        )

        # ====================================================
        # Initial Gripper State
        # ====================================================

        node.robot_init.open_gripper()


        # ====================================================
        # Select Object Shape (음성)
        # ====================================================

        voice_handler.request_shape()
        shape = voice_handler.keyword

        if not shape:
            node.get_logger().warn(
                "음성 인식 실패: 도형을 선택하지 못했습니다. -> circle 로 진행"
            )
            shape = "circle"

        node.get_logger().info(
            f"{shape} object 검출 대기 중..."
        )


        # ====================================================
        # Wait for Object ONLY  (SensorNode 가 state.objects 채움)
        # ====================================================

        state.tracking_enabled = False

        while rclpy.ok() and not node.wait_object_detected(shape):
            time.sleep(0.05)


        # ====================================================
        # Run Current Task
        # ====================================================

        node.run_pick_and_drop(
            shape
        )


        # ====================================================
        # Keep Alive
        # ====================================================

        while rclpy.ok():
            time.sleep(0.5)


    except KeyboardInterrupt:

        node.get_logger().warning(
            "강제종료"
        )


    finally:

        sensor_exec.shutdown()
        spin_thread.join(timeout=2.0)

        sensor_node.destroy_node()
        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# Entry Point
# ============================================================

if __name__ == "__main__":

    main()