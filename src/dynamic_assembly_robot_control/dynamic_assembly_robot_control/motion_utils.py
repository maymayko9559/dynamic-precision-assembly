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
    

    def pick_up(self, object_pose):
        from DSR_ROBOT2 import movej, posj,wait
        """
        임의의 목표 좌표(target_pose)를 받아 접근 후 물체를 집어 올리는 시나리오
        """
        self.object_pose = object_pose
        self.ri.node.get_logger().info("pick_up 동작 시퀀스 시작")

        # 1. 시작 전 그리퍼 확실히 열어두기
        self.ri.close_gripper()
        self.ri.open_gripper()

        # 2. 작업 준비 위치(Home 또는 Ready Pose)로 관절 이동 (예시 관절 각도)
        self.ri.node.get_logger().info("시작지점 가기 전")
        self.ri.move_joint([0,0,90,0,90,0],vel=30, acc=30)
        # self.ri.move_joint([0,0,50,0,90,0], vel=30, acc=30)
        self.ri.node.get_logger().info("시작지점 갔다")
        up_object=[
            self.object_pose[0],
            self.object_pose[1],
            self.object_pose[2]+100,
            self.object_pose[3],
            self.object_pose[4],
            self.object_pose[5]
        ]
        self.ri.node.get_logger().info('물체 위 위치로 이동중...')
        self.ri.move_linear_ABS(up_object, vel=30, acc=30)
        self.ri.node.get_logger().info('물체 위 위치로 이동완료!')
        wait(1.0)
        self.ri.node.get_logger().info('물체 잡으러 하강중...')
        self.ri.move_linear_REL([0.0,0.0,-95,0.0,0.0,0.0],vel=30, acc=30)
        self.ri.node.get_logger().info('물체 위치로 하강 완료!')
        self.ri.close_gripper()
        self.ri.node.get_logger().info('물체 잡기 완료!!')
        self.ri.move_linear_REL([0.0,0.0,100,0.0,0.0,0.0],vel=30, acc=30)
        self.ri.node.get_logger().info('물체 들고 안전하게 위로 올리기!')



    def test_z_retry(
        self,
        target_pose,
        insert_travel=40.0,   # 한 번에 내려볼 총 하강량(mm)
        step=20.0,             # 1스텝 하강량(mm)
        f_z_limit=8.0,        # z축 외력 임계값(N)
        seated_travel=50.0,   # 이만큼 내려갔는데 힘 안 걸리면 "성공"으로 간주
        retreat_z=50.0,       # 막혔을 때 떼는 높이(mm)
        max_retries=3,
        use_compliance=True,
    ):
        """
        현재 TCP 위치에서 그냥 Z축으로 하강.
        하강 중 z 외력이 f_z_limit 초과 + 아직 seated_travel 미만이면
        => retreat_z 만큼 떼고 같은 자리에서 다시 하강 (max_retries 회).
        pick_up 직후 물체를 든 상태로 호출해서 재시도 동작만 확인하는 용도.
        """
        from DSR_ROBOT2 import wait, get_tool_force, DR_BASE

        if use_compliance:
            from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl
        self.ri.move_linear_ABS(target_pose, vel=30, acc=30)

          

        for attempt in range(1, max_retries + 1):
            self.ri.node.get_logger().info(
                f"[TEST] 하강 시도 {attempt}/{max_retries}"
            )

            if use_compliance:
                task_compliance_ctrl([2000, 2000, 500, 200, 200, 200])
                wait(0.2)

            fz0 = self.ri.get_z_force() or 0.0
            self.ri.node.get_logger().info(f"[TEST] 힘 기준값 fz0 = {fz0:.2f} N")

            blocked = False
            travelled = 0.0

            while travelled < insert_travel:
                self.ri.move_linear_REL([0, 0, -step, 0, 0, 0], vel=30, acc=30)
                travelled += step

                fz = self.ri.get_z_force()  
                if fz is None:
                    continue
                ext = abs(fz - fz0)
                self.ri.node.get_logger().info(
                    f"[TEST] 하강 {travelled:.1f}mm | z 외력 {ext:.2f} N"
                )

                if ext > f_z_limit and travelled < seated_travel:
                    self.ri.node.get_logger().warn(
                        f"[TEST] z 외력 {ext:.2f}N 감지 → 떼고 재시도"
                    )
                    blocked = True
                    break

            if use_compliance:
                release_compliance_ctrl()
                wait(0.2)

            if not blocked:
                self.ri.node.get_logger().info(
                    f"[TEST] {travelled:.1f}mm 하강 완료 (힘 안 걸림) → 성공 처리, 종료"
                )
                #self.ri.move_linear_REL([0, 0, travelled, 0, 0, 0], vel=50, acc=30)
                self.ri.open_gripper()
                self.ri.move_linear_ABS([363.80, -12.77, 396.74, 15.18, 179.83, 15.33], vel=20, acc=20)
                return True

            # 막혔으면 떼고 같은 자리에서 다시
            self.ri.move_linear_REL([0, 0, retreat_z, 0, 0, 0], vel=40, acc=40)
            wait(0.5)

        self.ri.node.get_logger().error(f"[TEST] {max_retries}회 모두 막힘 - 종료")
        return False

    
    # def test_move_linear_ABS(self,target_pose):
    #     self.target_pose=target_pose
    #     self.ri.node.get_logger().info("ABS무브L시작!!")
    #     self.ri.move_linear_ABS(self.target_pose,vel=20,acc=20)
    # def test_move_linear_REL(self,offset_pose):
    #     self.offset_pose=offset_pose
    #     self.ri.node.get_logger().info("REL무브L시작!!")
    #     self.ri.move_linear_REL(self.offset_pose,vel=20,acc=20)


    # def async_pick_and_place_run(self, target_pose, offset_pose):
    #     import threading
    #     import time
    #     self.ri.node.get_logger().info("비동기 Pick & Place Test Start...")

    #     # 1. 시작 전 그리퍼 열기
    #     self.ri.open_gripper()
        
    #     # 2. 준비 자세 이동
    #     self.ri.move_joint([0, 0, 50, 0, 90, 0], vel=30, acc=30)

    #     # 3. [핵심] 절대 좌표 이동(movel)을 백그라운드 스레드로 실행
    #     motion_thread = threading.Thread(
    #         target=self.ri.move_linear_ABS,
    #         args=(target_pose,),
    #         kwargs={"vel": 20, "acc": 20}
    #     )
    #     motion_thread.start()

    #     # --------------------------------------------------------
    #     # 🚀 로봇이 이동하는 동안(동시에) 실행될 제어 영역
    #     # --------------------------------------------------------
    #     self.ri.node.get_logger().info("로봇 이동 중... 그리퍼 타이밍 대기")
        
    #     # 이동 시작 후 원하는 타이밍(예: 1.5초 뒤 도착 시점)에 그리퍼 작동
    #     time.sleep(1.0) 
        
    #     self.ri.node.get_logger().info("이동 도중 그리퍼 CLOSE 작동!")
    #     self.ri.close_gripper()
    #     self.ri.open_gripper()
    #     self.ri.close_gripper()
    #     # --------------------------------------------------------

    #     # 4. 모션 스레드가 완전히 끝날 때까지 대기
    #     motion_thread.join()

    #     # 5. 이후 상대 좌표 이동 수행
    #     self.ri.node.get_logger().info("상대 좌표 이동 수행")
    #     self.ri.move_linear_REL(offset_pose, vel=20, acc=20)

    
    def run(self):
        object = []#임시 좌표
        target = [367.37, 6.30, 215.33, 100.08, 179.98, 100.9]# 임시 좌표
    
        self.pick_up(object)
        self.test_z_retry()
        # self.place_object(target)
        # self.async_pick_and_place_run(target, offset)
        
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
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
