# ============================================================
# kalman_filter.py
# ============================================================
# 컨베이어 위 상자를 추적하는 등속(Constant Velocity) 칼만 필터.
# ROS 의존 없는 순수 numpy 클래스 -> pytest 로 단독 테스트 가능.
#
# 상태 x = [px, py, pz, vx, vy, vz]  (mm, mm/s), 로봇 BASE 좌표계
# 측정 z = [px, py, pz]              (mm)
#
# 한 사이클:
#   predict(dt) : dt 초 뒤 상태를 등속 물리로 예측
#   update(z)   : 새 측정으로 예측을 보정
# ============================================================

import numpy as np


class ConstantVelocityKF:

    def __init__(
        self,
        accel_std,
        meas_std,
        belt_speed=50.0,
        dt_min=0.005,
        dt_max=0.15,
        gate_chi2=7.815,
        warmup_updates=5,
    ):
        # accel_std : 가속도 잡음 표준편차 [mm/s^2] (스칼라 또는 길이 3)
        #             "상자가 등속에서 얼마나 벗어날 수 있나"
        # meas_std  : 측정 잡음 표준편차 [mm]       (스칼라 또는 길이 3)
        #             "카메라 위치값이 얼마나 떨리나"
        self.sa = np.broadcast_to(
            np.atleast_1d(np.asarray(accel_std, dtype=float)), (3,)
        ).astype(float)
        m = np.broadcast_to(
            np.atleast_1d(np.asarray(meas_std, dtype=float)), (3,)
        ).astype(float)

        # 측정 잡음 공분산 R = diag(분산)
        self.R = np.diag(m ** 2)

        # 측정 행렬 H : 상태 6개 중 위치 3개만 관측 -> [I3 | 0]
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0

        self.belt_speed = float(belt_speed)
        self.dt_min = float(dt_min)
        self.dt_max = float(dt_max)
        self.gate_chi2 = float(gate_chi2)
        self.warmup_updates = int(warmup_updates)

        # 첫 측정 전
        self.x = None            # 상태 (6,)
        self.P = None            # 상태 공분산 (6, 6) = 불확실도
        self.initialized = False
        self.n_updates = 0
        self.last_nis = None     # 마지막 update 의 NIS (튜닝 확인용)

    # --------------------------------------------------------
    # 이산화 : dt 가 정해졌을 때의 F, Q
    # --------------------------------------------------------

    def _F(self, dt):
        # 새 위치 = 위치 + 속도 * dt , 새 속도 = 속도
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        return F

    def _Q(self, dt):
        # "이 dt 동안 알 수 없는 가속도가 있었을 수 있다" 를 수식화.
        # 축마다 G = [dt^2/2, dt],  Q_axis = G G^T * sa^2
        Q = np.zeros((6, 6))
        q_pp = dt ** 4 / 4.0
        q_pv = dt ** 3 / 2.0
        q_vv = dt ** 2
        for i in range(3):
            s2 = self.sa[i] ** 2
            Q[i, i] = s2 * q_pp
            Q[i, i + 3] = s2 * q_pv
            Q[i + 3, i] = s2 * q_pv
            Q[i + 3, i + 3] = s2 * q_vv
        return Q

    # --------------------------------------------------------
    # 초기화 : 첫 측정으로 필터를 켠다
    # --------------------------------------------------------

    def init_measurement(self, z, t=None):
        # 위치 = 첫 측정값, 속도 = 0 에서 시작.
        # 속도를 모르니 속도 불확실도를 벨트 속도 규모로 크게 잡아
        # 이후 측정들로 필터가 스스로 속도를 찾게 한다 (약 1초).
        z = np.asarray(z, dtype=float)
        self.x = np.zeros(6)
        self.x[:3] = z
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = self.R.copy()
        self.P[3, 3] = self.P[4, 4] = self.P[5, 5] = self.belt_speed ** 2
        self.initialized = True
        self.n_updates = 0
        return True

    # --------------------------------------------------------
    # 예측
    # --------------------------------------------------------

    def predict(self, dt):
        if not self.initialized:
            return
        dt = float(np.clip(dt, self.dt_min, self.dt_max))
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)

    # --------------------------------------------------------
    # 보정 (측정 반영). 이상치면 버리고 False 반환.
    # --------------------------------------------------------

    def update(self, z):
        if not self.initialized:
            return False
        z = np.asarray(z, dtype=float)

        y = z - self.H @ self.x                    # 잔차 = 측정 - 예측
        S = self.H @ self.P @ self.H.T + self.R    # 잔차의 예상 크기
        S_inv = np.linalg.inv(S)

        nis = float(y @ S_inv @ y)                 # 이 측정이 얼마나 예상 밖인가
        self.last_nis = nis

        gating_on = self.n_updates >= self.warmup_updates
        if gating_on and self.gate_chi2 > 0.0 and nis > self.gate_chi2:
            return False                          # 이상치 -> 버림

        K = self.P @ self.H.T @ S_inv             # 칼만 이득 (측정을 얼마나 믿을지)

        self.x = self.x + K @ y
        I_KH = np.eye(6) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T   # Joseph form

        self.n_updates += 1
        return True

    # --------------------------------------------------------
    # 미래 위치 예측 (상태 불변)
    # --------------------------------------------------------

    def predict_future(self, horizon):
        h = max(float(horizon), 0.0)
        return self.x[:3] + self.x[3:] * h

    # --------------------------------------------------------
    # 조회용
    # --------------------------------------------------------

    @property
    def position(self):
        return self.x[:3].copy()

    @property
    def velocity(self):
        return self.x[3:].copy()

    def speed(self):
        return float(np.linalg.norm(self.x[3:]))

    def pos_std(self):
        return np.sqrt(np.clip(np.diag(self.P)[:3], 0.0, None))
