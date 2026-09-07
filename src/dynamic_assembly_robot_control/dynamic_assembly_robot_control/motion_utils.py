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
    
    def pick_up(
        self,
        object_pose,
        approach_height=80.0,
        pick_offset=0.0,
        lift_height=100.0,
    ):

        """
        object_pose:
            [x, y, z, rx, ry, rz]

        z는 Vision에서 얻은 object 기준 좌표.
        pick_offset은 실제 gripper TCP 보정 후 설정한다.
        """

        from DSR_ROBOT2 import wait

        x, y, z, rx, ry, rz = object_pose

        approach_pose = [
            x,
            y,
            z + approach_height,
            rx,
            ry,
            rz,
        ]

        pick_pose = [
            x,
            y,
            z + pick_offset,
            rx,
            ry,
            rz,
        ]

        lift_pose = [
            x,
            y,
            z + lift_height,
            rx,
            ry,
            rz,
        ]

        self.ri.node.get_logger().info(
            f"[PICK] approach={approach_pose}"
        )

        # 1. Gripper open
        self.ri.open_gripper()

        # 2. Object 위로 이동
        self.ri.move_linear_ABS(
            approach_pose,
            vel=20,
            acc=20
        )

        wait(0.5)

        # 3. 천천히 하강
        self.ri.move_linear_ABS(
            pick_pose,
            vel=10,
            acc=10
        )

        wait(0.3)

        # 4. Grip
        self.ri.close_gripper()

        wait(0.5)

        # 5. Lift
        self.ri.move_linear_ABS(
            lift_pose,
            vel=20,
            acc=20
        )

        self.ri.node.get_logger().info(
            "[PICK] Pick-up completed."
        )

    # ========================================================
    # Move Above Target
    # ========================================================

    def move_above_target(
        self,
        target_pose,
        approach_height=100.0,
        vel=20,
        acc=20,
    ):
        """
        Move to a safe position above the detected target.

        target_pose:
            [x, y, z, rx, ry, rz]

        approach_height:
            Target보다 위에서 정지할 높이 [mm]
        """

        x, y, z, rx, ry, rz = target_pose

        approach_pose = [
            x,
            y,
            z + approach_height,
            rx,
            ry,
            rz,
        ]

        self.ri.node.get_logger().info(
            f"[TARGET APPROACH] target={target_pose}"
        )

        self.ri.node.get_logger().info(
            f"[TARGET APPROACH] approach={approach_pose}"
        )

        self.ri.move_linear_ABS(
            approach_pose,
            vel=vel,
            acc=acc,
        )

        self.ri.node.get_logger().info(
            "[TARGET APPROACH] Reached target approach position."
        )

        return approach_pose

    def test_z_retry(
        self,
        target_pose,
        insert_travel=40.0,   # 한 번에 내려볼 총 하강량(mm)
        step=5.0,             # 1스텝 하강량(mm)
        f_z_limit=3.0,        # z축 외력 임계값(N)
        seated_travel=30.0,   # 이만큼 내려갔는데 힘 안 걸리면 "성공"으로 간주
        retreat_z=50.0,       # 막혔을 때 떼는 높이(mm)
        xy_correction=2.0,    # XY 보정 한 스텝 크기(mm)
        xy_offsets=None,      # 직접 (dx, dy) 후보 리스트를 주고 싶을 때
        settle_pose=None,     # 성공/종료 후 복귀할 안전 pose
        use_compliance=True,
    ):
        """
        target_pose(구멍 위치)에서 Z축으로 하강하며 삽입을 시도한다.

        하강 중 z 외력이 f_z_limit 를 초과하면(= 구멍에 안 들어가고 막힘)
        => retreat_z 만큼 위로 떼고, 기준 pose(target_pose)로 ABS 복귀한 뒤
        => XY 보정값을 하나씩 바꿔가며 다시 하강을 시도한다.

        보정 순서 (기본값, c = xy_correction):
            1) (0, 0)      보정 없음
            2) (+c, 0)     x+
            3) (-c, 0)     x-
            4) (0, +c)     y+
            5) (0, -c)     y-
            6) (+c, +c)    x+, y+
            7) (-c, -c)    x-, y-

        어느 한 보정값에서 힘이 안 걸리고 끝까지 내려가면
        => 삽입 성공 → 그리퍼 open, settle_pose 로 복귀, True 반환.
        모든 보정값이 실패하면 위로 뗀 뒤 False 반환.

        pick_up 직후 물체를 든 상태로 호출한다.
        """
        from DSR_ROBOT2 import wait

        if use_compliance:
            from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl

        # 모든 보정은 이 기준 pose + (dx, dy) 를 ABS 로 이동한다. (REL 누적 X)
        base = list(target_pose)

        if settle_pose is None:
            settle_pose = [363.80, -12.77, 396.74, 15.18, 179.83, 15.33]

        if xy_offsets is None:
            c = xy_correction
            xy_offsets = [
                (0.0, 0.0),     # 보정 없음
                (+c, 0.0),      # x+
                (-c, 0.0),      # x-
                (0.0, +c),      # y+
                (0.0, -c),      # y-
                (+c, +c),       # x+, y+
                (-c, -c), 
                (-c, +c),
                (+c, -c),
                (+2*c, 0.0),
                (-2*c, 0.0),
                (0.0, +2*c),
                (0.0, -2*c),      # x-, y-
            ]

        def descend_and_check():
            """현재 자리에서 Z 하강.
            막히면 (True, travelled), 끝까지 내려가면 (False, travelled) 반환."""
            if use_compliance:
                task_compliance_ctrl([500, 500, 500, 200, 200, 200])
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
                        f"[TEST] z 외력 {ext:.2f}N 감지 → 막힘"
                    )
                    blocked = True
                    break

            if use_compliance:
                release_compliance_ctrl()
                wait(0.2)
            return blocked, travelled

        total = len(xy_offsets)
        for idx, (dx, dy) in enumerate(xy_offsets, start=1):
            self.ri.node.get_logger().info(
                f"[TEST] 보정 시도 {idx}/{total} (dx={dx:+.1f}, dy={dy:+.1f})"
            )

            # 1) 기준 pose + XY 보정값으로 이동 (구멍 위)
            corrected = base[:]
            corrected[0] += dx
            corrected[1] += dy
            self.ri.move_linear_ABS(corrected, vel=30, acc=30)

            # 2) 하강 시도
            blocked, travelled = descend_and_check()

            # 3) 성공 판정 (힘 안 걸리고 끝까지 내려감)
            if not blocked:
                self.ri.node.get_logger().info(
                    f"[TEST] (dx={dx:+.1f}, dy={dy:+.1f}) 에서 {travelled:.1f}mm 삽입 성공 "
                    f"→ 그리퍼 open 후 종료"
                )
                self.ri.open_gripper()
                self.ri.move_linear_ABS(settle_pose, vel=20, acc=20)
                return True

            # 4) 막힘 → 위로 떼고 기준 pose 로 복귀 후 다음 보정값 시도
            self.ri.node.get_logger().warn(
                f"[TEST] (dx={dx:+.1f}, dy={dy:+.1f}) 막힘 "
                f"→ {retreat_z:.0f}mm 상승 후 기준 위치 복귀"
            )
            self.ri.move_linear_REL([0, 0, retreat_z, 0, 0, 0], vel=40, acc=40)
            wait(0.3)
            self.ri.move_linear_ABS(base, vel=30, acc=30)
            wait(0.3)

        self.ri.node.get_logger().error(
            f"[TEST] 보정값 {total}개 모두 삽입 실패 - 종료"
        )
        self.ri.move_linear_REL([0, 0, retreat_z, 0, 0, 0], vel=40, acc=40)
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
