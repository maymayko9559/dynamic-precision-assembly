# ============================================================
# motion_planner.py
# ============================================================
# [EN]
#
# Calculates robot target poses for dynamic pick-and-place.
#
# This class DOES NOT move the robot directly.
#
# Current Responsibilities:
# - Apply vision-to-robot calibration offsets
# - Generate object pick pose
# - Generate box drop pose
#
# Future Responsibilities:
# - Moving target prediction
# - Interception point calculation
# - Motion timing compensation
# - Kalman Filter prediction integration
#
#
# [KR]
#
# 동적 Pick-and-Place를 위한 Robot Target Pose를 계산한다.
#
# Robot을 직접 움직이지 않고
# "어디로 이동해야 하는가"만 계산한다.
#
# 현재 주요 역할:
# - Vision -> Robot Offset 적용
# - Object Pick Pose 생성
# - Box Drop Pose 생성
#
# 향후:
# - Moving Target Prediction
# - Interception Point 계산
# - Robot 이동시간 보정
# - Kalman Filter Prediction 연동
# ============================================================


class MotionPlanner:

    def __init__(
        self,
        object_offset=None,
        target_offset=None,
        tool_orientation=None,
    ):

        # ====================================================
        # Object Calibration Offset
        # ====================================================

        if object_offset is None:

            object_offset = [
                0.0,
                0.0,
                0.0,
            ]


        self.object_offset = [
            float(v)
            for v in object_offset
        ]


        # ====================================================
        # Target Calibration Offset
        # ====================================================

        if target_offset is None:

            target_offset = [
                0.0,
                0.0,
                0.0,
            ]


        self.target_offset = [
            float(v)
            for v in target_offset
        ]


        # ====================================================
        # Tool Orientation
        # ====================================================

        if tool_orientation is None:

            tool_orientation = [
                100.08,
                179.98,
                100.9,
            ]


        self.tool_orientation = [
            float(v)
            for v in tool_orientation
        ]


    # ========================================================
    # Build Object Pick Pose
    # ========================================================

    def build_object_pose(
        self,
        object_info
    ):
        """
        Vision Object XYZ에
        object calibration offset을 적용한다.

        Returns:
            [x, y, z, rx, ry, rz]
        """

        if object_info is None:
            return None


        x = (
            float(object_info["x"])
            + self.object_offset[0]
        )

        y = (
            float(object_info["y"])
            + self.object_offset[1]
        )

        z = (
            float(object_info["z"])
            + self.object_offset[2]
        )


        rx, ry, rz = (
            self.tool_orientation
        )


        return [
            x,
            y,
            z,
            rx,
            ry,
            rz,
        ]


    # ========================================================
    # Build Target Pose
    # ========================================================

    def build_target_pose(
        self,
        target_info
    ):
        """
        Vision Target XYZ에
        target calibration offset을 적용한다.

        Current:
            Static Box Center

        Future:
            Predicted Moving Box Center
        """

        if target_info is None:
            return None


        x = (
            float(target_info["x"])
            + self.target_offset[0]
        )

        y = (
            float(target_info["y"])
            + self.target_offset[1]
        )

        z = (
            float(target_info["z"])
            + self.target_offset[2]
        )


        rx, ry, rz = (
            self.tool_orientation
        )


        return [
            x,
            y,
            z,
            rx,
            ry,
            rz,
        ]


    # ========================================================
    # Predict Constant-Velocity Position
    # ========================================================

    def predict_position(
        self,
        target_info,
        velocity,
        prediction_time,
    ):
        """
        Constant velocity prediction.

        p(t + dt)
            =
        p(t) + v * dt

        Parameters
        ----------
        target_info:
            {
                "x": ...,
                "y": ...,
                "z": ...
            }

        velocity:
            (vx, vy, vz) [mm/s]

        prediction_time:
            dt [sec]

        Returns
        -------
        dict
        """

        if target_info is None:
            return None


        vx, vy, vz = velocity

        dt = float(
            prediction_time
        )


        result = dict(
            target_info
        )


        result["x"] = (
            float(target_info["x"])
            + float(vx) * dt
        )

        result["y"] = (
            float(target_info["y"])
            + float(vy) * dt
        )

        result["z"] = (
            float(target_info["z"])
            + float(vz) * dt
        )


        result["predicted"] = True

        result["prediction_time"] = dt


        return result


    # ========================================================
    # Estimate Robot Travel Time
    # ========================================================

    def estimate_travel_time(
        self,
        current_pose,
        target_pose,
        linear_speed,
    ):
        """
        Very simple Cartesian travel-time estimate.

        time ~= distance / speed

        현재는 단순 모델.
        이후 acceleration / robot trajectory time을
        포함하도록 개선 가능.

        Parameters
        ----------
        current_pose:
            [x, y, z, ...]

        target_pose:
            [x, y, z, ...]

        linear_speed:
            mm/sec
        """

        if (
            current_pose is None
            or target_pose is None
        ):
            return None


        speed = float(
            linear_speed
        )


        if speed <= 0.0:
            return None


        dx = (
            float(target_pose[0])
            - float(current_pose[0])
        )

        dy = (
            float(target_pose[1])
            - float(current_pose[1])
        )

        dz = (
            float(target_pose[2])
            - float(current_pose[2])
        )


        distance = (
            dx * dx
            + dy * dy
            + dz * dz
        ) ** 0.5


        travel_time = (
            distance
            / speed
        )


        return float(
            travel_time
        )


    # ========================================================
    # Plan Moving Target Pose
    # ========================================================

    def plan_moving_target_pose(
        self,
        target_info,
        velocity,
        prediction_time,
    ):
        """
        Future LV3 / LV4 helper.

        1. Moving target의 future XYZ 계산
        2. Target calibration offset 적용
        3. Robot pose 반환
        """

        predicted_target = (
            self.predict_position(
                target_info,
                velocity,
                prediction_time
            )
        )


        if predicted_target is None:
            return None


        return self.build_target_pose(
            predicted_target
        )