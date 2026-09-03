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

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
from .robot_init import RobotInit
class MotionUtils:
    def __init__(self, robot_init_instance: RobotInit):
        super().__init__()
        self.ri = robot_init_instance

    def place_object(self, target_pose):
        """
        임의의 목표 좌표(place_pose)를 받아 접근 후 물체를 내려놓는 시나리오
        """
        self.target_pose = target_pose
        self.ri.node.get_logger().info("Starting Place Motion Sequence...")

        # 1. 작업 준비 위치(Home 또는 Ready Pose)로 관절 이동 (예시 관절 각도)
        self.ri.move_joint([0, 0, 50, 0, 90, 0], vel=30, acc=30)

        up_target_pose = [
            self.target_pose[0],
            self.target_pose[1],
            self.target_pose[2]+200, # z축을 200mm 위로
            self.target_pose[3],
            self.target_pose[4],
            self.target_pose[5]
        ]

        # 2. 목표 위치의 위로 이동
        self.ri.move_linear_ABS(up_target_pose, vel=30, acc=30)

        # 3. 목표위치에 집어넣기
        self.ri.move_linear_REL([0, 0, -300, 0, 0, 0], vel=20, acc=20)

        # 3. 그리퍼 열기 (물체 놓기)
        self.ri.open_gripper()

        # 4. 작업 완료 후 Home 위치로 복귀
        self.ri.move_joint([0, 0, 50, 0, 90, 0], vel=30, acc=30)

        self.ri.node.get_logger().info("Place Motion Sequence Finished Successfully!")
        
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

    def async_pick_and_place_run(self, target_pose, offset_pose):
        import threading
        import time
        self.ri.node.get_logger().info("비동기 Pick & Place Test Start...")

        # 1. 시작 전 그리퍼 열기
        self.ri.open_gripper()
        
        # 2. 준비 자세 이동
        self.ri.move_joint([0, 0, 50, 0, 90, 0], vel=30, acc=30)

        # 3. [핵심] 절대 좌표 이동(movel)을 백그라운드 스레드로 실행
        motion_thread = threading.Thread(
            target=self.ri.move_linear_ABS,
            args=(target_pose,),
            kwargs={"vel": 20, "acc": 20}
        )
        motion_thread.start()

        # --------------------------------------------------------
        # 🚀 로봇이 이동하는 동안(동시에) 실행될 제어 영역
        # --------------------------------------------------------
        self.ri.node.get_logger().info("로봇 이동 중... 그리퍼 타이밍 대기")
        
        # 이동 시작 후 원하는 타이밍(예: 1.5초 뒤 도착 시점)에 그리퍼 작동
        time.sleep(1.0) 
        
        self.ri.node.get_logger().info("이동 도중 그리퍼 CLOSE 작동!")
        self.ri.close_gripper()
        self.ri.open_gripper()
        self.ri.close_gripper()
        # --------------------------------------------------------

        # 4. 모션 스레드가 완전히 끝날 때까지 대기
        motion_thread.join()

        # 5. 이후 상대 좌표 이동 수행
        self.ri.node.get_logger().info("상대 좌표 이동 수행")
        self.ri.move_linear_REL(offset_pose, vel=20, acc=20)

    
    def run(self):
        target = [367.37, 6.30, 215.33, 100.08, 179.98, 100.9]
        offset = [0, -100, 0, 0, 0, 0]
        
        self.async_pick_and_place_run(target, offset)
        
     # 절대좌표 홈위치 == posj([0,0,90,0,90,0])
     # self.test_move_linear_REL([0,-100,0,0,0,0])

# 임시로 이 파일 단독 실행을 테스트하기 위한 메인 함수
# motion_utils.py 의 main 함수 부분 수정

def main(args=None):
    import rclpy
    from rclpy.node import Node
    import DR_init
    
    

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
