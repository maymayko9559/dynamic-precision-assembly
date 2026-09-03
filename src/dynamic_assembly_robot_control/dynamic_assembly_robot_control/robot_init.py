import DR_init
from .onrobot import RG

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

class RobotInit:
    def __init__(self,node):
        self.node=node
        
        
        # 그리퍼 초기화
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        self.node.get_logger().info("RobotController & Gripper initialized successfully.")

    def open_gripper(self):
        # 함수 안에서 임포트하여 노드가 생성된 이후에만 로드되도록 함
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
        from DSR_ROBOT2 import movej, posj
        target_posj = posj(j_target)
        self.node.get_logger().info(f"Moving Joint to: {target_posj}")
        movej(target_posj, vel=vel, acc=acc)

    # 절대좌표 movel_함수
    def move_linear_ABS(self, pos_target, vel=20, acc=20):
        from DSR_ROBOT2 import movel, posx, DR_MV_MOD_ABS
        self.pos_target = posx(pos_target)
        self.node.get_logger().info(f"절대좌표 이동: {self.pos_target}")
        movel(self.pos_target, vel=vel, acc=acc, mod=DR_MV_MOD_ABS)

    # 상대좌표 movel_함수
    def move_linear_REL(self,pos_offset, vel = 20 , acc = 20):
        from DSR_ROBOT2 import movel, posx, DR_MV_MOD_REL
        self.pos_offset = posx(pos_offset)
        self.node.get_logger().info(f"상대좌표 이동: {self.pos_offset}")
        movel(self.pos_offset, vel =vel,acc=acc,mod=DR_MV_MOD_REL)

    # compliance 시작
    def start_compliance(self):
        from DSR_ROBOT2 import(
            wait, task_compliance_ctrl,
            set_stiffnessx
        )
        
    def get_current_pose(self):
        """현재 로봇의 TCP pose를 가져옵니다. [x, y, z, rx, ry, rz]"""

        from DSR_ROBOT2 import get_current_posx

        try:
            robot_pose = get_current_posx()[0]
            return [ float(value) for value in robot_pose ]

        except Exception as e:
            self.node.get_logger().error(f"Failed to get current robot pose: {e}")
            return None