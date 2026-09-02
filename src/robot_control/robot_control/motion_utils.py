# ============================================================
# motion_utils.py
# ============================================================
# [EN]
# Provides reusable utility functions for robot motion.
#
# Main Responsibilities:
# - Provide common movej / movel functions
# - Move to Home / Approach positions
# - Calculate target positions and offsets
# - Reduce duplicated robot motion code
#
# [KR]
# 로봇 이동에 반복적으로 사용되는 공통 Motion 함수를
# 모아두는 Utility 파일.
#
# 주요 역할:
# - movej / movel 등의 공통 이동 기능
# - Home / Approach 위치 이동
# - Target 위치 및 Offset 계산
# - 중복되는 Robot Motion 코드 관리
# ============================================================
# ============================================================
# motion_utils.py
# ============================================================
# ============================================================
# motion_utils.py
# ============================================================
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
from .robot_init import RobotInit
class MotionUtils:
    def __init__(self, robot_init_instance: RobotInit):
        super().__init__()
        self.ri = robot_init_instance

    def pick_and_place_scenario(self, target_pose):
        from DSR_ROBOT2 import movej, posj
        """
        임의의 목표 좌표(target_pose)를 받아 접근 후 물체를 집어 올리는 시나리오
        """
        self.target_pose = target_pose
        self.ri.node.get_logger().info("Starting Pick and Place Motion Sequence...")

        # 1. 시작 전 그리퍼 확실히 열어두기
        self.ri.close_gripper()
        self.ri.open_gripper()

        # 2. 작업 준비 위치(Home 또는 Ready Pose)로 관절 이동 (예시 관절 각도)
        self.ri.node.get_logger().info("가기 전")
        self.ri.move_joint([0,0,50,0,90,0],vel=30, acc=30)
        # self.ri.move_joint([0,0,50,0,90,0], vel=30, acc=30)
        self.ri.node.get_logger().info("갔다")
        self.ri.move_joint(self.target_pose, vel=30, acc=30)

        

        self.ri.node.get_logger().info("Motion Sequence Finished Successfully!")
    def test_move_linear_ABS(self,target_pose):
        self.target_pose=target_pose
        self.ri.node.get_logger().info("ABS무브L시작!!")
        self.ri.move_linear_ABS(self.target_pose,vel=20,acc=20)
    def test_move_linear_REL(self,offset_pose):
        self.offset_pose=offset_pose
        self.ri.node.get_logger().info("REL무브L시작!!")
        self.ri.move_linear_REL(self.offset_pose,vel=20,acc=20)

    
    def run(self):
        # 예시 목표 좌표 ([200.0, 0.0, 150.0] 정도로 안전한 높이 테스트를 권장합니다!)
        self.pick_and_place_scenario([0,0,90,0,90,0])
        self.test_move_linear_ABS([367.37,6.30,215.33,100.08,179.98,100.9])
        self.test_move_linear_REL([0,-100,0,0,0,0])

# 임시로 이 파일 단독 실행을 테스트하기 위한 메인 함수
# motion_utils.py 의 main 함수 부분 수정

def main(args=None):
    import rclpy
    from rclpy.node import Node
    

    rclpy.init(args=args)
    
    # 1. ROS2 노드 먼저 생성
    node = Node('motion_utils_node',namespace=ROBOT_ID)
    
    # 2. DSR_ROBOT2가 인식할 수 있도록 DR_init 설정을 전역(Global)으로 확실히 세팅
    
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node  

    import DSR_ROBOT2
    # 3. 그 다음 RobotInit 및 MotionUtils 초기화
    robot_init = RobotInit(node)
    motion = MotionUtils(robot_init)
    
    node.get_logger().info("모션 유틸 단독 실행 시작")
    motion.run()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
