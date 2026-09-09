# ============================================================
# target_manager.py
# ============================================================
from collections import deque
import numpy as np


class TargetManager:

    def __init__(
        self,
        node,
        history_size=20,
        stale_timeout=2.0,
        velocity_window=10,
        min_velocity_samples=4,
    ):
        self.node = node
        self.history_size = int(history_size)
        self.stale_timeout = float(stale_timeout)
        self.velocity_window = int(velocity_window)
        self.min_velocity_samples = int(min_velocity_samples)

        self.latest_target = None
        self.latest_target_time = None

        self.history = deque(
            maxlen=self.history_size
        )

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0


    def update_target(
        self,
        shape,
        x,
        y,
        z,
        angle=0.0,
    ):
        now = self.node.get_clock().now()
        time_sec = now.nanoseconds / 1e9

        target = {
            "shape": str(shape),
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "angle": float(angle),
            "time": float(time_sec),
        }

        self.history.append(target)
        self.latest_target = target
        self.latest_target_time = now

        self._estimate_velocity()

        return target


    def get_latest_target(self):
        return self.latest_target


    def has_target(self):
        return self.latest_target is not None


    def get_history_count(self):
        return len(self.history)


    def get_target_age(self):

        if self.latest_target_time is None:
            return None

        now = self.node.get_clock().now()

        age = (
            now
            - self.latest_target_time
        ).nanoseconds / 1e9

        return float(age)


    def is_target_fresh(self):

        age = self.get_target_age()

        if age is None:
            return False

        return age <= self.stale_timeout


    def _estimate_velocity(self):

        if len(self.history) < self.min_velocity_samples:
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            self.velocity_z = 0.0
            return

        samples = list(
            self.history
        )[-self.velocity_window:]

        times = np.array(
            [s["time"] for s in samples],
            dtype=np.float64
        )

        xs = np.array(
            [s["x"] for s in samples],
            dtype=np.float64
        )

        ys = np.array(
            [s["y"] for s in samples],
            dtype=np.float64
        )

        zs = np.array(
            [s["z"] for s in samples],
            dtype=np.float64
        )

        times = times - times[0]

        if times[-1] <= 0.0:
            return

        self.velocity_x = float(
            np.polyfit(times, xs, 1)[0]
        )

        self.velocity_y = float(
            np.polyfit(times, ys, 1)[0]
        )

        self.velocity_z = float(
            np.polyfit(times, zs, 1)[0]
        )


    def get_velocity(self):
        return (
            float(self.velocity_x),
            float(self.velocity_y),
            float(self.velocity_z),
        )


    def predict_target(
        self,
        prediction_time
    ):
        if self.latest_target is None:
            return None

        dt = float(prediction_time)

        return {
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


    def predict_from_now(
        self,
        future_time=0.0
    ):
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


    def clear(self):
        self.latest_target = None
        self.latest_target_time = None

        self.history.clear()

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.velocity_z = 0.0
