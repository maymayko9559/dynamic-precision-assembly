# ============================================================
# test_kalman_filter.py
# ============================================================
# ConstantVelocityKF 단위 테스트 (ROS 불필요).
#   PYTHONPATH=src/object_tracking python3 -m pytest \
#       src/object_tracking/test/test_kalman_filter.py -v
# ============================================================

import numpy as np

from object_tracking.kalman_filter import ConstantVelocityKF


DT = 1.0 / 30.0                 # 카메라 30 Hz
MEAS_STD = [3.0, 3.0, 8.0]      # mm
ACCEL_STD = 30.0               # mm/s^2  (테스트는 수렴을 빨리 보려고 조금 크게)


def make_kf():
    return ConstantVelocityKF(
        accel_std=ACCEL_STD,
        meas_std=MEAS_STD,
        belt_speed=50.0,
    )


def run_trajectory(kf, p0, vel, n_steps, dt=DT, noise_std=3.0, seed=0):
    """등속 궤적을 만들어 측정으로 먹인다. 참 위치 배열을 반환."""
    rng = np.random.default_rng(seed)
    p = np.asarray(p0, dtype=float)
    vel = np.asarray(vel, dtype=float)
    truth = []
    for _ in range(n_steps):
        p = p + vel * dt
        truth.append(p.copy())
        z = p + rng.normal(0.0, noise_std, size=3)
        if not kf.initialized:
            kf.init_measurement(z)
        else:
            kf.predict(dt)
            kf.update(z)
    return np.array(truth)


def test_converges_to_constant_velocity():
    kf = make_kf()
    vel = np.array([50.0, 0.0, 0.0])
    truth = run_trajectory(kf, [500, 0, 300], vel, n_steps=250)

    assert kf.initialized
    assert np.allclose(kf.velocity, vel, atol=8.0)
    assert np.allclose(kf.position, truth[-1], atol=6.0)


def test_stationary_speed_stays_small():
    kf = make_kf()
    run_trajectory(kf, [500, 0, 300], [0, 0, 0], n_steps=300)
    assert kf.speed() < 8.0


def test_variable_dt_is_stable():
    kf = make_kf()
    rng = np.random.default_rng(1)
    p = np.array([500.0, 0.0, 300.0])
    vel = np.array([0.0, 50.0, 0.0])
    for _ in range(300):
        dt = float(rng.uniform(0.01, 0.12))
        p = p + vel * dt
        z = p + rng.normal(0.0, 3.0, size=3)
        if not kf.initialized:
            kf.init_measurement(z)
        else:
            kf.predict(dt)
            kf.update(z)

    assert np.all(np.isfinite(kf.x))
    assert np.all(np.isfinite(kf.P))
    assert np.allclose(kf.velocity, vel, atol=12.0)


def test_coast_increases_uncertainty():
    kf = make_kf()
    run_trajectory(kf, [500, 0, 300], [50, 0, 0], n_steps=150)

    std_before = kf.pos_std().max()
    pos_before = kf.position.copy()

    for _ in range(30):
        kf.predict(DT)                       # 측정 없이 예측만 (가림 상황)

    assert kf.pos_std().max() > std_before   # 불확실도 증가
    moved = kf.position - pos_before
    assert moved[0] > 30.0                    # +x 로 계속 진행
    assert np.all(np.isfinite(kf.predict_future(0.5)))


def test_outlier_is_rejected():
    kf = make_kf()
    run_trajectory(kf, [500, 0, 300], [50, 0, 0], n_steps=150)

    pos_before = kf.position.copy()

    kf.predict(DT)
    bad = kf.position + np.array([200.0, 0.0, 0.0])     # 200 mm 튄 측정
    assert kf.update(bad) is False
    assert np.linalg.norm(kf.position - pos_before) < 15.0   # 상태 거의 안 흔들림

    kf.predict(DT)
    good = pos_before + np.array([50.0 * 2 * DT, 0.0, 0.0])
    assert kf.update(good) is True                       # 정상 측정은 계속 수용


def test_predict_future_extrapolates():
    kf = make_kf()
    run_trajectory(kf, [500, 0, 300], [50, 0, 0], n_steps=250)

    future = kf.predict_future(1.0)
    assert np.allclose(future, kf.position + kf.velocity * 1.0)
    assert future[0] - kf.position[0] > 40.0             # 약 50 mm 전진


def test_not_initialized_is_safe():
    kf = make_kf()
    kf.predict(DT)                            # 아무 일도 안 일어나야 함
    assert kf.update([1.0, 2.0, 3.0]) is False
    assert kf.initialized is False
