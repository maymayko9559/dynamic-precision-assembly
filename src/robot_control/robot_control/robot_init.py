import DR_init
from .onrobot import RG
from DSR_ROBOT2 import (
    get_current_posx,
    movej,
    movel,
    wait,
    DR_MV_MOD_REL
)

# 로봇 기본 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

class RobotController:
    def __init__(self, node):
        """
        ROS2 노드 객체를 받아와서 로봇과 그리퍼를 초기화합니다.
        """
        self.node = node
        
        # 두산 로봇 기본 환경 설정
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = node
        
        # 그리퍼 초기화
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        self.node.get_logger().info("RobotController & Gripper initialized successfully.")

    def open_gripper(self):
        """그리퍼 열기"""
        self.node.get_logger().info("Gripper OPEN")
        self.gripper.open_gripper()
        wait(1.0)

    def close_gripper(self):
        """그리퍼 닫기"""
        self.node.get_logger().info("Gripper CLOSE")
        self.gripper.close_gripper()
        wait(1.0)

    def move_joint(self, j_target, vel=30, acc=30):
        """관절 각도 이동 (Joint Move)"""
        self.node.get_logger().info(f"Moving Joint to: {j_target}")
        movej(j_target, vel=vel, acc=acc)

    def move_linear(self, pos_target, vel=20, acc=20, is_relative=False):
        """직선 이동 (Task Space Move)"""
        if is_relative:
            self.node.get_logger().info(f"Moving Relative Task pos: {pos_target}")
            movel(pos_target, vel=vel, acc=acc, mod=DR_MV_MOD_REL)
        else:
            self.node.get_logger().info(f"Moving Absolute Task pos: {pos_target}")
            movel(pos_target, vel=vel, acc=acc)