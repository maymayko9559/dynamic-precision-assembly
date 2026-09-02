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
import rclpy
from rclpy.node import Node
from .robot_init import RobotInit

class MotionUtils:
    def __init__(self, robot_init_instance: RobotInit):
        """
        RobotInit 객체를 주입받아 모션 시나리오를 구성합니다.
        """
        self.ri = robot_init_instance

    def pick_and_place_scenario(self, target_pose):
        """
        임의의 목표 좌표(target_pose)를 받아 접근 후 물체를 집어 올리는 시나리오
        """
        self.ri.node.get_logger().info("Starting Pick and Place Motion Sequence...")

        # 1. 시작 전 그리퍼 확실히 열어두기
        self.ri.close_gripper()
        self.ri.open_gripper()

        # 2. 작업 준비 위치(Home 또는 Ready Pose)로 관절 이동 (예시 관절 각도)
        self.ri.move_joint([0, 0, 50, 0, 90, 0], vel=30, acc=30)
        self.ri.node.get_logger().info("갔다")

        # 3. 목표 위치 바로 위(상공 100mm)로 Approach 이동 (안전거리 확보)
        approach_pose = target_pose
        approach_pose[2] += 100.0  # Z축 상공
        self.ri.move_linear(approach_pose, vel=30, acc=30)

        # 4. 실제 타겟 위치로 직선 하강 이동 (아까 말씀하신 Z=50 위치는 너무 낮으니 테스트 시 주의하세요!)
        self.ri.move_linear(target_pose, vel=15, acc=15)

        # 5. 그리퍼 닫기 (물체 집기)
        self.ri.close_gripper()

        # 6. 집은 상태로 위로 들어 올리기 (상대 좌표 Z +100mm)
        up_offset = [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]
        self.ri.move_linear(up_offset, vel=20, acc=20, is_relative=True)

        self.ri.node.get_logger().info("Motion Sequence Finished Successfully!")
    
    def run(self):
        # 예시 목표 좌표 ([200.0, 0.0, 150.0] 정도로 안전한 높이 테스트를 권장합니다!)
        self.pick_and_place_scenario([200.0, 0.0, 150.0,0.0,0.0,0.0])

# 임시로 이 파일 단독 실행을 테스트하기 위한 메인 함수
# motion_utils.py 의 main 함수 부분 수정

def main(args=None):
    rclpy.init(args=args)
    
    # 1. ROS2 노드 먼저 생성
    node = Node('motion_utils_node')
    
    # 2. DSR_ROBOT2가 인식할 수 있도록 DR_init 설정을 전역(Global)으로 확실히 세팅
    import DR_init
    DR_init.__dsr__id = "dsr01"
    DR_init.__dsr__model = "m0609"
    DR_init.__dsr__node = node  # 👈 이 부분이 DSR_ROBOT2 임포트보다 반드시 먼저 실행되어야 합니다!
    
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
