import DR_init
from .onrobot import RG

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

class RobotInit:
    def __init__(self, node):
        self.node = node
        
        # 두산 로봇 기본 환경 설정
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = node
        
        # 그리퍼 초기화
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        self.node.get_logger().info("RobotController & Gripper initialized successfully.")

    def open_gripper(self):
        # 👈 함수 안에서 임포트하여 노드가 생성된 이후에만 로드되도록 함
        from DSR_ROBOT2 import wait
        self.node.get_logger().info("Gripper OPEN")
        self.gripper.open_gripper()
        wait(1.0)

    def close_gripper(self):
        from DSR_ROBOT2 import wait
        self.node.get_logger().info("Gripper CLOSE")
        self.gripper.close_gripper()
        wait(1.0)

    def move_joint(self, j_target, vel=30, acc=30):
        from DSR_ROBOT2 import movej
        self.node.get_logger().info(f"Moving Joint to: {j_target}")
        movej(j_target, vel=vel, acc=acc)

    def move_linear(self, pos_target, vel=20, acc=20, is_relative=False):
        from DSR_ROBOT2 import movel, DR_MV_MOD_REL
        if is_relative:
            self.node.get_logger().info(f"Moving Relative Task pos: {pos_target}")
            movel(pos_target, vel=vel, acc=acc, mod=DR_MV_MOD_REL)
        else:
            self.node.get_logger().info(f"Moving Absolute Task pos: {pos_target}")
            movel(pos_target, vel=vel, acc=acc)