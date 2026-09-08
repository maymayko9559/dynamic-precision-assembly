# ============================================================
# motion_utils.py
# ============================================================
# [EN]
#
# Reusable motion utilities for dynamic pick-and-place.
#
# Current Responsibilities:
# - Pick up detected objects
# - Move above box target
# - Drop object into box
# - Retreat to a safe height
#
# Future Responsibilities:
# - Moving target interception
# - Predictive placement
#
#
# [KR]
#
# 동적 Pick-and-Place에 반복적으로 사용되는
# Robot Motion Utility.
#
# 현재 주요 역할:
# - Object Pick
# - Box Target 상공 이동
# - Box 안에 Object Drop
# - Drop 후 안전 높이로 복귀
#
# 향후:
# - Moving Target Interception
# - Predictive Placement
# ============================================================


from .robot_init import RobotInit


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


class MotionUtils:

    def __init__(
        self,
        robot_init_instance: RobotInit
    ):

        self.ri = robot_init_instance


    # ========================================================
    # Pick Up Object
    # ========================================================

    def pick_up(
        self,
        object_pose,
        approach_height=80.0,
        pick_offset=0.0,
        lift_height=100.0,
    ):
        """
        Pick up an object.

        Parameters
        ----------
        object_pose:
            [x, y, z, rx, ry, rz]

        approach_height:
            Object 위에서 접근할 높이 [mm]

        pick_offset:
            실제 grasp height 조정값 [mm]

        lift_height:
            Pick 후 상승할 높이 [mm]
        """

        from DSR_ROBOT2 import wait


        x, y, z, rx, ry, rz = object_pose


        # ====================================================
        # Approach Pose
        # ====================================================

        approach_pose = [
            x,
            y,
            z + approach_height,
            rx,
            ry,
            rz,
        ]


        # ====================================================
        # Pick Pose
        # ====================================================

        pick_pose = [
            x,
            y,
            z + pick_offset,
            rx,
            ry,
            rz,
        ]


        # ====================================================
        # Lift Pose
        # ====================================================

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

        self.ri.node.get_logger().info(
            f"[PICK] pick={pick_pose}"
        )


        # ====================================================
        # 1. Open Gripper
        # ====================================================

        self.ri.open_gripper()


        # ====================================================
        # 2. Move Above Object
        # ====================================================

        self.ri.move_linear_ABS(
            approach_pose,
            vel=20,
            acc=20
        )

        wait(0.5)


        # ====================================================
        # 3. Descend to Object
        # ====================================================

        self.ri.move_linear_ABS(
            pick_pose,
            vel=10,
            acc=10
        )

        wait(0.3)


        # ====================================================
        # 4. Close Gripper
        # ====================================================

        self.ri.close_gripper()

        wait(0.5)


        # ====================================================
        # 5. Lift Object
        # ====================================================

        self.ri.move_linear_ABS(
            lift_pose,
            vel=20,
            acc=20
        )


        self.ri.node.get_logger().info(
            "[PICK] Pick-up completed."
        )


        return True


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
        Move to a safe position above the box target.

        target_pose:
            [x, y, z, rx, ry, rz]
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
            f"[TARGET APPROACH] "
            f"target={target_pose}"
        )


        self.ri.node.get_logger().info(
            f"[TARGET APPROACH] "
            f"approach={approach_pose}"
        )


        self.ri.move_linear_ABS(
            approach_pose,
            vel=vel,
            acc=acc
        )


        self.ri.node.get_logger().info(
            "[TARGET APPROACH] "
            "Reached target approach position."
        )


        return approach_pose


    # ========================================================
    # Drop Object Into Box
    # ========================================================

    def drop_object(
        self,
        target_pose,
        approach_height=100.0,
        drop_height=40.0,
        retreat_height=100.0,
        approach_vel=20,
        drop_vel=10,
        acc=20,
    ):
        """
        Drop the currently held object into the box.

        target_pose:
            [x, y, z, rx, ry, rz]

        approach_height:
            Target Z보다 위에서 접근할 높이 [mm]

        drop_height:
            Target Z보다 위에서 object를 놓을 높이 [mm]

        retreat_height:
            Object release 후 상승할 높이 [mm]
        """

        from DSR_ROBOT2 import wait

        x, y, z, rx, ry, rz = target_pose


        # ====================================================
        # Above Box
        # ====================================================

        approach_pose = [
            x,
            y,
            z + approach_height,
            rx,
            ry,
            rz,
        ]


        # ====================================================
        # Drop Position
        # ====================================================

        drop_pose = [
            x,
            y,
            z + drop_height,
            rx,
            ry,
            rz,
        ]


        # ====================================================
        # Retreat Position
        # ====================================================

        retreat_pose = [
            x,
            y,
            z + retreat_height,
            rx,
            ry,
            rz,
        ]


        # ====================================================
        # DEBUG: Planned Drop Motion
        # ====================================================

        self.ri.node.get_logger().info(
            "========================================"
        )

        self.ri.node.get_logger().info(
            f"[DROP PLAN] "
            f"target_pose={target_pose}"
        )

        self.ri.node.get_logger().info(
            f"[DROP PLAN] "
            f"approach_pose={approach_pose}"
        )

        self.ri.node.get_logger().info(
            f"[DROP PLAN] "
            f"drop_pose={drop_pose}"
        )

        self.ri.node.get_logger().info(
            f"[DROP PLAN] "
            f"retreat_pose={retreat_pose}"
        )


        # ====================================================
        # 1. Move Above Box
        # ====================================================

        self.ri.node.get_logger().info(
            f"[APPROACH COMMAND] "
            f"pose={approach_pose}, "
            f"vel={approach_vel}, "
            f"acc={acc}"
        )

        self.ri.move_linear_ABS(
            approach_pose,
            vel=approach_vel,
            acc=acc
        )

        wait(0.3)


        # ====================================================
        # 2. Move Down Into Safe Drop Height
        # ====================================================

        self.ri.node.get_logger().info(
            f"[DROP COMMAND] "
            f"pose={drop_pose}, "
            f"vel={drop_vel}, "
            f"acc={acc}"
        )

        self.ri.move_linear_ABS(
            drop_pose,
            vel=drop_vel,
            acc=acc
        )

        wait(0.3)


        # ====================================================
        # 3. Release Object
        # ====================================================

        self.ri.node.get_logger().info(
            f"[DROP RELEASE] "
            f"Opening gripper at pose={drop_pose}"
        )

        self.ri.open_gripper()

        wait(0.5)


        # ====================================================
        # 4. Retreat
        # ====================================================

        self.ri.node.get_logger().info(
            f"[RETREAT COMMAND] "
            f"pose={retreat_pose}, "
            f"vel={approach_vel}, "
            f"acc={acc}"
        )

        self.ri.move_linear_ABS(
            retreat_pose,
            vel=approach_vel,
            acc=acc
        )


        self.ri.node.get_logger().info(
            "[DROP] Object placement completed."
        )

        self.ri.node.get_logger().info(
            "========================================"
        )

        return True


    # ========================================================
    # Move to Safe Pose
    # ========================================================

    def move_to_safe_pose(
        self,
        safe_pose,
        vel=20,
        acc=20
    ):
        """
        Move robot to an explicitly provided safe Cartesian pose.
        """

        self.ri.node.get_logger().info(
            f"[SAFE] Moving to safe pose: "
            f"{safe_pose}"
        )


        self.ri.move_linear_ABS(
            safe_pose,
            vel=vel,
            acc=acc
        )


        return True


    # ========================================================
    # FUTURE:
    # Insertion with Force Retry
    # ========================================================
    #
    # Previous test_z_retry() is intentionally removed from
    # the current motion path.
    #
    # If shape insertion is used again later,
    # move that function into:
    #
    # insertion_controller.py
    #
    # rather than keeping it inside MotionUtils.
    # ========================================================




    # def test_z_retry(
    #     self,
    #     target_pose,
    #     insert_travel=40.0,   # 한 번에 내려볼 총 하강량(mm)
    #     step=5.0,             # 1스텝 하강량(mm)
    #     f_z_limit=3.0,        # z축 외력 임계값(N)
    #     seated_travel=30.0,   # 이만큼 내려갔는데 힘 안 걸리면 "성공"으로 간주
    #     retreat_z=50.0,       # 막혔을 때 떼는 높이(mm)
    #     xy_correction=2.0,    # XY 보정 한 스텝 크기(mm)
    #     xy_offsets=None,      # 직접 (dx, dy) 후보 리스트를 주고 싶을 때
    #     settle_pose=None,     # 성공/종료 후 복귀할 안전 pose
    #     use_compliance=True,
    # ):
    #     """
    #     target_pose(구멍 위치)에서 Z축으로 하강하며 삽입을 시도한다.

    #     하강 중 z 외력이 f_z_limit 를 초과하면(= 구멍에 안 들어가고 막힘)
    #     => retreat_z 만큼 위로 떼고, 기준 pose(target_pose)로 ABS 복귀한 뒤
    #     => XY 보정값을 하나씩 바꿔가며 다시 하강을 시도한다.

    #     보정 순서 (기본값, c = xy_correction):
    #         1) (0, 0)      보정 없음
    #         2) (+c, 0)     x+
    #         3) (-c, 0)     x-
    #         4) (0, +c)     y+
    #         5) (0, -c)     y-
    #         6) (+c, +c)    x+, y+
    #         7) (-c, -c)    x-, y-

    #     어느 한 보정값에서 힘이 안 걸리고 끝까지 내려가면
    #     => 삽입 성공 → 그리퍼 open, settle_pose 로 복귀, True 반환.
    #     모든 보정값이 실패하면 위로 뗀 뒤 False 반환.

    #     pick_up 직후 물체를 든 상태로 호출한다.
    #     """
    #     from DSR_ROBOT2 import wait

    #     if use_compliance:
    #         from DSR_ROBOT2 import task_compliance_ctrl, release_compliance_ctrl

    #     # 모든 보정은 이 기준 pose + (dx, dy) 를 ABS 로 이동한다. (REL 누적 X)
    #     base = list(target_pose)

    #     if settle_pose is None:
    #         settle_pose = [363.80, -12.77, 396.74, 15.18, 179.83, 15.33]

    #     if xy_offsets is None:
    #         c = xy_correction
    #         xy_offsets = [
    #             (0.0, 0.0),     # 보정 없음
    #             (+c, 0.0),      # x+
    #             (-c, 0.0),      # x-
    #             (0.0, +c),      # y+
    #             (0.0, -c),      # y-
    #             (+c, +c),       # x+, y+
    #             (-c, -c), 
    #             (-c, +c),
    #             (+c, -c),
    #             (+2*c, 0.0),
    #             (-2*c, 0.0),
    #             (0.0, +2*c),
    #             (0.0, -2*c),      # x-, y-
    #         ]

    #     def descend_and_check():
    #         """현재 자리에서 Z 하강.
    #         막히면 (True, travelled), 끝까지 내려가면 (False, travelled) 반환."""
    #         if use_compliance:
    #             task_compliance_ctrl([500, 500, 500, 200, 200, 200])
    #             wait(0.2)

    #         fz0 = self.ri.get_z_force() or 0.0
    #         self.ri.node.get_logger().info(f"[TEST] 힘 기준값 fz0 = {fz0:.2f} N")

    #         blocked = False
    #         travelled = 0.0
    #         while travelled < insert_travel:
    #             self.ri.move_linear_REL([0, 0, -step, 0, 0, 0], vel=30, acc=30)
    #             travelled += step

    #             fz = self.ri.get_z_force()
    #             if fz is None:
    #                 continue
    #             ext = abs(fz - fz0)
    #             self.ri.node.get_logger().info(
    #                 f"[TEST] 하강 {travelled:.1f}mm | z 외력 {ext:.2f} N"
    #             )

    #             if ext > f_z_limit and travelled < seated_travel:
    #                 self.ri.node.get_logger().warn(
    #                     f"[TEST] z 외력 {ext:.2f}N 감지 → 막힘"
    #                 )
    #                 blocked = True
    #                 break

    #         if use_compliance:
    #             release_compliance_ctrl()
    #             wait(0.2)
    #         return blocked, travelled

    #     total = len(xy_offsets)
    #     for idx, (dx, dy) in enumerate(xy_offsets, start=1):
    #         self.ri.node.get_logger().info(
    #             f"[TEST] 보정 시도 {idx}/{total} (dx={dx:+.1f}, dy={dy:+.1f})"
    #         )

    #         # 1) 기준 pose + XY 보정값으로 이동 (구멍 위)
    #         corrected = base[:]
    #         corrected[0] += dx
    #         corrected[1] += dy
    #         self.ri.move_linear_ABS(corrected, vel=30, acc=30)

    #         # 2) 하강 시도
    #         blocked, travelled = descend_and_check()

    #         # 3) 성공 판정 (힘 안 걸리고 끝까지 내려감)
    #         if not blocked:
    #             self.ri.node.get_logger().info(
    #                 f"[TEST] (dx={dx:+.1f}, dy={dy:+.1f}) 에서 {travelled:.1f}mm 삽입 성공 "
    #                 f"→ 그리퍼 open 후 종료"
    #             )
    #             self.ri.open_gripper()
    #             self.ri.move_linear_ABS(settle_pose, vel=20, acc=20)
    #             return True

    #         # 4) 막힘 → 위로 떼고 기준 pose 로 복귀 후 다음 보정값 시도
    #         self.ri.node.get_logger().warn(
    #             f"[TEST] (dx={dx:+.1f}, dy={dy:+.1f}) 막힘 "
    #             f"→ {retreat_z:.0f}mm 상승 후 기준 위치 복귀"
    #         )
    #         self.ri.move_linear_REL([0, 0, retreat_z, 0, 0, 0], vel=40, acc=40)
    #         wait(0.3)
    #         self.ri.move_linear_ABS(base, vel=30, acc=30)
    #         wait(0.3)

    #     self.ri.node.get_logger().error(
    #         f"[TEST] 보정값 {total}개 모두 삽입 실패 - 종료"
    #     )
    #     self.ri.move_linear_REL([0, 0, retreat_z, 0, 0, 0], vel=40, acc=40)
    #     return False
