# ============================================================
# robot_init.py
# ============================================================
# [EN]
# Provides basic interfaces for controlling the Doosan robot
# and OnRobot gripper.
#
# Main Responsibilities:
# - Initialize Doosan robot configuration
# - Initialize gripper
# - Control gripper
# - Joint / Linear robot motion
# - Compliance control
# - Request current TCP pose
#
# [KR]
# Doosan Robot과 OnRobot Gripper의 기본 제어 기능을 제공한다.
#
# 주요 역할:
# - Doosan Robot 설정
# - Gripper 초기화
# - Gripper Open / Close
# - MoveJ / MoveL
# - Compliance Control
# - 현재 TCP Pose 요청
# ============================================================


import DR_init

from .onrobot import RG

from dsr_msgs2.srv import GetCurrentPosx


# ============================================================
# Robot Configuration
# ============================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"


# ============================================================
# Robot Init
# ============================================================

class RobotInit:

    def __init__(self, node):

        self.node = node


        # ====================================================
        # Doosan Robot Configuration
        # ====================================================

        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = node


        # ====================================================
        # Gripper Initialization
        # ====================================================

        self.gripper = RG(
            GRIPPER_NAME,
            TOOLCHARGER_IP,
            TOOLCHARGER_PORT
        )

        self.node.get_logger().info(
            "Robot & Gripper initialized successfully."
        )


        # ====================================================
        # Current Pose Service Client
        # ====================================================
        # Doosan Service:
        #
        # /dsr01/dsr_controller2/aux_control/get_current_posx
        #
        # Response:
        # [x, y, z, rx, ry, rz]
        # ====================================================

        self.current_posx_client = self.node.create_client(
            GetCurrentPosx,
            "/dsr01/dsr_controller2/aux_control/get_current_posx"
        )


    # ========================================================
    # Gripper Open
    # ========================================================

    def open_gripper(self):

        # Import DSR_ROBOT2 after ROS2 Node initialization
        from DSR_ROBOT2 import wait

        self.node.get_logger().info(
            "Gripper OPEN"
        )

        self.gripper.open_gripper()

        wait(1.0)


    # ========================================================
    # Gripper Close
    # ========================================================

    def close_gripper(self):

        from DSR_ROBOT2 import wait

        self.node.get_logger().info(
            "Gripper CLOSE"
        )

        self.gripper.close_gripper()

        wait(1.0)


    # ========================================================
    # Move Joint
    # ========================================================

    def move_joint(
        self,
        j_target,
        vel=30,
        acc=30
    ):

        from DSR_ROBOT2 import (
            movej,
            posj
        )

        target_posj = posj(j_target)

        self.node.get_logger().info(
            f"Moving Joint to: {target_posj}"
        )

        movej(
            target_posj,
            vel=vel,
            acc=acc
        )


    # ========================================================
    # Move Linear - Absolute
    # ========================================================

    def move_linear_ABS(
        self,
        pos_target,
        vel=20,
        acc=20
    ):

        from DSR_ROBOT2 import (
            movel,
            posx,
            DR_MV_MOD_ABS
        )

        target_pos = posx(pos_target)

        self.node.get_logger().info(
            f"절대좌표 이동: {target_pos}"
        )

        movel(
            target_pos,
            vel=vel,
            acc=acc,
            mod=DR_MV_MOD_ABS
        )


    # ========================================================
    # Move Linear - Relative
    # ========================================================

    def move_linear_REL(
        self,
        pos_offset,
        vel=20,
        acc=20
    ):

        from DSR_ROBOT2 import (
            movel,
            posx,
            DR_MV_MOD_REL
        )

        target_offset = posx(pos_offset)

        self.node.get_logger().info(
            f"상대좌표 이동: {target_offset}"
        )

        movel(
            target_offset,
            vel=vel,
            acc=acc,
            mod=DR_MV_MOD_REL
        )


    

    


    # ========================================================
    # Request Current Robot Pose
    # ========================================================

    def request_current_pose(
        self,
        callback
    ):

        # ====================================================
        # Check Service
        # ====================================================

        if not self.current_posx_client.service_is_ready():

            self.node.get_logger().warning(
                "Waiting for get_current_posx service...",
                throttle_duration_sec=1.0
            )

            return False


        # ====================================================
        # Create Request
        # ====================================================

        request = GetCurrentPosx.Request()

        # DR_BASE = 0
        request.ref = 0


        # ====================================================
        # Async Service Call
        # ====================================================

        future = self.current_posx_client.call_async(
            request
        )

        future.add_done_callback(
            callback
        )

        return True

    # ========================================================
    # Get External Force on TCP  (BASE frame: [fx, fy, fz, tx, ty, tz])
    # ========================================================

    def get_tcp_force(self):
        from DSR_ROBOT2 import get_tool_force, DR_BASE
        f = get_tool_force(DR_BASE)
        if not isinstance(f, (list, tuple)) or len(f) < 6:
            return None
        return [float(v) for v in f]

    def get_z_force(self):
        f = self.get_tcp_force()
        return None if f is None else f[2]

    # ========================================================
    # Compliance (부드러운 삽입 / 판 바닥 충격 완화)
    # ========================================================

    def enable_soft_z(self, stx=(2000, 2000, 500, 200, 200, 200)):
        from DSR_ROBOT2 import task_compliance_ctrl
        task_compliance_ctrl(list(stx))

    def disable_soft_z(self):
        from DSR_ROBOT2 import release_compliance_ctrl
        release_compliance_ctrl()