# ============================================================
# target_manager.py
# ============================================================
# [EN]
#
# Stores and manages target / box information received
# from the Vision System.
#
# Current Responsibilities:
# - Store the latest target position
# - Keep target shape information for future use
# - Keep recent target history
# - Check target freshness
#
# Future Responsibilities:
# - Estimate target velocity
# - Handle temporary ArUco occlusion
# - Predict moving target position
# - Kalman Filter integration
#
#
# [KR]
#
# Vision System에서 받은 Target / Box 정보를 관리한다.
#
# 현재 주요 역할:
# - 최신 Target 위치 저장
# - Target Shape 정보 유지
# - Target 위치 History 저장
# - Target 데이터가 최신인지 확인
#
# 향후:
# - Target 속도 추정
# - ArUco 일시적 가림 대응
# - 이동 Target 위치 예측
# - Kalman Filter 연동
# ============================================================


from collections import deque


class TargetManager:

    def __init__(
        self,
        node,
        history_size=20,
        stale_timeout=0.5,
    ):
        """
        Parameters
        ----------
        node:
            ROS2 Node instance.

        history_size:
            최근 target measurement를 몇 개 저장할지.

        stale_timeout:
            마지막 target 검출 이후 이 시간보다 오래 지나면
            target을 stale 상태로 판단한다. [sec]
        """

        self.node = node

        self.history_size = history_size

        self.stale_timeout = stale_timeout


        # ====================================================
        # Latest Target
        # ====================================================

        self.latest_target = None

        self.latest_target_time = None


        # ====================================================
        # Target History
        # ====================================================
        #
        # Each entry:
        #
        # {
        #     "time": float,
        #     "shape": str,
        #     "x": float,
        #     "y": float,
        #     "z": float,
        #     "angle": float
        # }
        #
        # ====================================================

        self.history = deque(
            maxlen=history_size
        )


        # ====================================================
        # Estimated Velocity
        # ====================================================
        #
        # 현재는 simple finite difference.
        #
        # LV4에서는 Kalman Filter 값으로 대체 가능.
        # ====================================================

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0


    # ========================================================
    # Update Target
    # ========================================================

    def update_target(
        self,
        shape,
        x,
        y,
        z,
        angle=0.0,
    ):
        """
        Update the latest detected target.

        Target shape는 현재 drop logic에서는 사용하지 않지만
        future use를 위해 계속 저장한다.
        """

        now = self.node.get_clock().now()

        time_sec = (
            now.nanoseconds
            / 1e9
        )


        target = {
            "shape": str(shape),
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "angle": float(angle),
            "time": float(time_sec),
        }


        # ====================================================
        # Add History
        # ====================================================

        self.history.append(
            target
        )


        # ====================================================
        # Latest Target
        # ====================================================

        self.latest_target = target

        self.latest_target_time = now


        # ====================================================
        # Estimate Velocity
        # ====================================================

        self._estimate_velocity()


        return target


    # ========================================================
    # Get Latest Target
    # ========================================================

    def get_latest_target(self):

        return self.latest_target


    # ========================================================
    # Has Target
    # ========================================================

    def has_target(self):

        return (
            self.latest_target
            is not None
        )


    # ========================================================
    # Get Target Age
    # ========================================================

    def get_target_age(self):
        """
        Last target measurement 이후 지난 시간 [sec].
        """

        if self.latest_target_time is None:
            return None


        now = self.node.get_clock().now()


        age = (
            now
            - self.latest_target_time
        ).nanoseconds / 1e9


        return float(age)


    # ========================================================
    # Is Target Fresh
    # ========================================================

    def is_target_fresh(self):
        """
        Target이 stale_timeout 이내에 관측되었는지 확인.
        """

        age = self.get_target_age()


        if age is None:
            return False


        return (
            age
            <= self.stale_timeout
        )


    # ========================================================
    # Estimate Velocity
    # ========================================================

    def _estimate_velocity(self):
        """
        최근 두 measurement를 이용해서
        simple finite-difference velocity를 계산한다.

        vx = dx / dt
        vy = dy / dt
        vz = dz / dt

        Unit:
            mm / sec
        """

        if len(self.history) < 2:

            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.velocity_z = 0.0

            return


        previous = self.history[-2]

        current = self.history[-1]


        dt = (
            current["time"]
            - previous["time"]
        )


        if dt <= 0.0:
            return


        self.velocity_x = (
            current["x"]
            - previous["x"]
        ) / dt


        self.velocity_y = (
            current["y"]
            - previous["y"]
        ) / dt


        self.velocity_z = (
            current["z"]
            - previous["z"]
        ) / dt


    # ========================================================
    # Get Velocity
    # ========================================================

    def get_velocity(self):
        """
        Returns:
            (vx, vy, vz) [mm/s]
        """

        return (
            float(self.velocity_x),
            float(self.velocity_y),
            float(self.velocity_z),
        )


    # ========================================================
    # Predict Target
    # ========================================================

    def predict_target(
        self,
        prediction_time
    ):
        """
        Constant Velocity Model.

        x_future = x + vx * dt
        y_future = y + vy * dt
        z_future = z + vz * dt

        Parameters
        ----------
        prediction_time:
            현재 latest target 이후 몇 초 뒤 위치를
            예측할 것인지. [sec]

        Returns
        -------
        dict or None
        """

        if self.latest_target is None:
            return None


        dt = float(
            prediction_time
        )


        predicted = {
            "shape":
                self.latest_target["shape"],

            "x":
                self.latest_target["x"]
                + self.velocity_x * dt,

            "y":
                self.latest_target["y"]
                + self.velocity_y * dt,

            "z":
                self.latest_target["z"]
                + self.velocity_z * dt,

            "angle":
                self.latest_target["angle"],

            "predicted":
                True,

            "prediction_time":
                dt,
        }


        return predicted


    # ========================================================
    # Predict From Current Time
    # ========================================================

    def predict_from_now(
        self,
        future_time=0.0
    ):
        """
        마지막 measurement 이후 이미 지난 시간까지 포함해서
        현재 또는 미래 위치를 예측한다.

        Example:

        marker가 마지막으로 보인 뒤 0.15 sec 지났고
        앞으로 0.20 sec 뒤 위치가 필요하다면

        total_dt = 0.15 + 0.20
        """

        if self.latest_target is None:
            return None


        age = self.get_target_age()


        if age is None:
            return None


        total_dt = (
            age
            + float(future_time)
        )


        return self.predict_target(
            total_dt
        )


    # ========================================================
    # Clear
    # ========================================================

    def clear(self):

        self.latest_target = None

        self.latest_target_time = None

        self.history.clear()

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0