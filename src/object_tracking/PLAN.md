# object_tracking 구현 계획 (직접 작성하는 Kalman Filter)

## 1. 목표

컨베이어 벨트(등속, **50 mm/s**, 가속도 0) 위에서 움직이는 "도형 구멍 상자"를
칼만 필터로 **계속 위치 추정**하고, 로봇이 도달할 미래 시점의 상자 위치를
**계속 예측**하여 `PredictedTarget` 으로 발행한다.

- 환경: Doosan m0609, RealSense, ROS2(jazzy), Ubuntu, Python, `ament_python`
- 필터: 외부 라이브러리 금지 → **`numpy` 만으로 직접 구현**
- 좌표계 / 단위: **로봇 BASE 프레임, mm / mm·s⁻¹** (vision_system이 이미 이 형태로 publish)

---

## 2. 입출력 계약 (msg 2개)

우리가 쓰는 메시지는 **정확히 두 개**. 둘 다 `assembly_interfaces` 에 이미 존재.

### 2.1 입력 (구독) — `assembly_interfaces/DetectedObject`

토픽: `/vision/detected_object`

```
string  type      # "object" / "target" / ...
string  shape     # "box" 등
float64 x         # 로봇 BASE, mm
float64 y
float64 z
float64 angle     # deg (미사용)
```

- **`type == "target"` 인 메시지만** 사용
- 그 `(x, y, z)` 가 **칼만 필터의 측정값 `z_k`**
- `shape` 는 그대로 통과시켜 출력에 실어줌
- timestamp 없음 → 수신 시각을 측정 시각으로 사용 (5.4)

### 2.2 출력 (발행) — `assembly_interfaces/PredictedTarget`

토픽: `/tracking/predicted_target`

```
string  shape             # 입력에서 통과
float64 x                 # 예측 위치, 로봇 BASE, mm
float64 y
float64 z
float64 prediction_time   # 몇 초 뒤를 예측한 값인지 [s]
```

- 타이머 주기(예: 50 Hz)로 **연속 발행**
- `x, y, z` = "지금부터 `prediction_time` 초 뒤" 상자 예상 위치
- 추적 유실(coast 초과) 시 **발행 중단** (오래된 예측을 내보내지 않음)

---

## 3. 선행조건 (M0) — `assembly_interfaces` 빌드 등록

> `PredictedTarget.msg` 는 **필드는 이미 완비**돼 있으나
> [`assembly_interfaces/CMakeLists.txt`](../assembly_interfaces/CMakeLists.txt) 의
> `rosidl_generate_interfaces()` 에 **등록돼 있지 않아 빌드되지 않는다**
> (`msg/DetectedObject.msg` 만 등록됨). → `import` 불가.

`object_tracking` 밖의 변경이라 **로봇/인터페이스 담당과 협의 필요.** 필요한 건 한 줄:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/PredictedTarget.msg"      # ← 이 줄 추가
)
```

- `PredictedTarget.msg` 본문 수정 불필요 (현재 필드 그대로 사용)
- 완료 기준: `colcon build --packages-select assembly_interfaces` 성공 후
  `ros2 interface show assembly_interfaces/msg/PredictedTarget` 정상 출력
- **막히면 폴백**: `object_tracking/msg/PredictedTarget.msg` 로 동일 정의를 임시 생성해
  자체 빌드(이 경우 `object_tracking` 을 `ament_cmake` 혼합 or 별도 msg 패키지 필요) →
  가장 단순한 임시안은 `std_msgs/Float64MultiArray` 로 `[x,y,z,prediction_time]` 발행.
  M0 가 되면 즉시 `PredictedTarget` 로 교체.

---

## 4. 아키텍처

### 4.1 단일 노드

`tracking_node` **하나만** ROS 노드. 나머지 `.py` 는 순수 라이브러리 클래스.

```
object_tracking/object_tracking/
├─ tracking_node.py      # 유일한 ROS 노드: 구독 / 타이머 / 발행
├─ kalman_filter.py      # ConstantVelocityKF  (numpy only, ROS 의존 X)
└─ motion_predictor.py   # 미래 위치 예측 + 지연 보상 (numpy only)
```

`velocity_estimator.py` 는 선택 — 튜닝 시 KF 속도와 비교하는 최소제곱 추정기(부록 A).
`setup.py` `console_scripts` 는 `tracking_node` 하나만 남긴다.

### 4.2 데이터 흐름

```
/vision/detected_object (DetectedObject)
        │  type=="target" 만
        ▼
  on_detection()  ── 측정 시각까지 predict → update(x,y,z)
        │
  ConstantVelocityKF  (상태 = 위치 3 + 속도 3)
        │
  on_timer() 50Hz ── predict(dt) → MotionPredictor.predict(horizon)
        ▼
/tracking/predicted_target (PredictedTarget)   ← 연속 발행
```

### 4.3 타이머와 카메라 분리

- **측정 콜백** (카메라 ~30 Hz, 불규칙): `predict` 후 `update` 만
- **타이머** (50 Hz 고정): `predict` + 예측 계산 + `PredictedTarget` 발행
- 카메라가 잠깐 끊겨도(로봇 팔이 ArUco 가림) 타이머 `predict` 로 관성 주행(coast)

---

## 5. 칼만 필터 설계

### 5.1 모델: 3D 등속(Constant Velocity)

벨트가 가속도 0이므로 CV 모델이 물리적으로 정확. 벨트 진동·측정 노이즈는 `Q` 가 흡수.

- 상태 `x = [px, py, pz, vx, vy, vz]ᵀ`  (mm, mm/s)
- 측정 `z = [px, py, pz]ᵀ`  (BASE, mm)
- z(깊이)는 노이즈가 크므로 `rz` 를 x·y 보다 크게

### 5.2 이산화

**전이 `F(dt)`**
```
F = [ I3   dt·I3 ]
    [ 0    I3    ]
```

**프로세스 노이즈 `Q(dt)`** — 이산 백색 가속도 모델 (`G = [dt²/2, dt]ᵀ`, `Q = G Gᵀ σ_a²`, 축별):
```
Q_axis = σ_a² · [ dt⁴/4   dt³/2 ]
                [ dt³/2   dt²   ]
```
`σ_a` = 가속도 잡음 표준편차 [mm/s²]. **벨트가 진짜 등속이므로 작게** (초기값 10~30).

**측정 `H = [ I3 | 0₃ ]`**, **`R = diag(rx, ry, rz)`** [mm²] — M6에서 실측.

### 5.3 predict / update

```
# predict
x ← F x
P ← F P Fᵀ + Q

# update  (Joseph form, 수치 안정)
ŷ = z − H x
S = H P Hᵀ + R
K = P Hᵀ S⁻¹
x ← x + K ŷ
P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ
```

### 5.4 측정 시각 / dt

`DetectedObject` 에 timestamp 없음 → `tracking_node` **수신 시각**(`get_clock().now()`)을
측정 시각으로 사용. `dt` 는 `[dt_min, dt_max]` 로 clamp (프레임 드랍/스톨 시 발산 방지).

### 5.5 초기화 — 벨트 속도 50 mm/s 활용

- 1번째 target 측정: 위치만 저장
- 2번째 측정: 이동 방향 `d = (z₂ − z₁)/‖z₂ − z₁‖`,
  **속도 초기값 `v₀ = 50 · d`** (크기는 벨트 속도로 확정, 방향만 관측에서)
  → 2점 유한차분보다 노이즈에 강하고 초기 과도구간이 짧다
- `P₀`: 위치 블록 = `R`, 속도 블록 = `diag((0.3·50)²)` 정도 (방향 오차분 여유)
- `‖z₂ − z₁‖` 가 너무 작으면(정지/노이즈) 방향 불명 → `v₀ = 0`, 속도 공분산 크게

### 5.6 이상치 게이팅 (NIS)

```
NIS = ŷᵀ S⁻¹ ŷ        # ~ χ²(3)
NIS > gate_chi2 (예: 7.815 = 95%)  →  이 측정 무시 (update 스킵)
```
ArUco 순간 오검출 방어. 초기 `warmup_updates` 회는 게이트 비활성화.
추가 sanity: 추정 `‖v‖` 가 `belt_speed ± margin` 벗어나면 경고 로그.

### 5.7 Coast (가림 대응)

- 측정 없는 타이머 tick: `predict` 만 → `P` 가 `Q` 만큼 증가
- `now − t_last_meas > max_coast_time` (예: 1.0 s) → **`PredictedTarget` 발행 중단**
- 다시 측정 들어오면 자동 재개 (필터 상태 유지, 필요시 재초기화)

---

## 6. `kalman_filter.py` 스켈레톤

```python
import numpy as np


class ConstantVelocityKF:
    """
    Hand-written 6-state constant-velocity Kalman filter (numpy only).
    state x = [px, py, pz, vx, vy, vz]  (mm, mm/s), robot BASE frame
    meas  z = [px, py, pz]
    """

    def __init__(self, accel_std, meas_std, belt_speed=50.0,
                 dt_min=5e-3, dt_max=0.15):
        self.sa = np.broadcast_to(np.atleast_1d(np.asarray(accel_std, float)), (3,)).copy()
        r = np.broadcast_to(np.atleast_1d(np.asarray(meas_std, float)), (3,)).astype(float)
        self.R = np.diag(r ** 2)
        self.H = np.zeros((3, 6)); self.H[:3, :3] = np.eye(3)
        self.belt_speed = float(belt_speed)
        self.dt_min, self.dt_max = dt_min, dt_max

        self.x = None
        self.P = None
        self.initialized = False
        self._first = None          # (t, z) for direction seeding
        self.n_updates = 0
        self.last_nis = None

    # ---- discretization ----
    def _F(self, dt):
        F = np.eye(6); F[:3, 3:] = np.eye(3) * dt
        return F

    def _Q(self, dt):
        Q = np.zeros((6, 6))
        q11, q12, q22 = dt**4 / 4.0, dt**3 / 2.0, dt**2
        for i in range(3):
            s2 = self.sa[i] ** 2
            Q[i, i]         = s2 * q11
            Q[i, i + 3]     = s2 * q12
            Q[i + 3, i]     = s2 * q12
            Q[i + 3, i + 3] = s2 * q22
        return Q

    # ---- init (uses known belt speed for velocity magnitude) ----
    def init_measurement(self, z, t):
        z = np.asarray(z, float)
        if self._first is None:
            self._first = (t, z)
            return False
        t0, z0 = self._first
        d = z - z0
        dist = np.linalg.norm(d)
        if dist > 2.0:                       # 방향 신뢰 가능
            v0 = self.belt_speed * d / dist
            pv = (0.3 * self.belt_speed) ** 2
        else:                               # 방향 불명
            v0 = np.zeros(3)
            pv = self.belt_speed ** 2
        self.x = np.hstack([z, v0])
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = self.R.copy()
        self.P[3:, 3:] = np.eye(3) * pv
        self.initialized = True
        return True

    # ---- predict ----
    def predict(self, dt):
        if not self.initialized:
            return
        dt = float(np.clip(dt, self.dt_min, self.dt_max))
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)

    # ---- update ----
    def update(self, z, gate_chi2=7.815, warmup=5):
        if not self.initialized:
            return False
        z = np.asarray(z, float)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis = nis
        if gate_chi2 and self.n_updates >= warmup and nis > gate_chi2:
            return False
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T
        self.n_updates += 1
        return True

    # ---- accessors ----
    @property
    def position(self): return self.x[:3].copy()
    @property
    def velocity(self): return self.x[3:].copy()
    def speed(self): return float(np.linalg.norm(self.x[3:]))
    def pos_std(self): return np.sqrt(np.clip(np.diag(self.P)[:3], 0, None))

    def predict_future(self, horizon):
        """지금 상태에서 horizon 초 뒤 위치 (상태 불변)."""
        h = max(float(horizon), 0.0)
        return self.x[:3] + self.x[3:] * h
```

---

## 7. `motion_predictor.py` 스켈레톤

```python
class MotionPredictor:
    """수신 지연 + 로봇 이동시간을 더한 horizon 으로 미래 위치 계산."""

    def __init__(self, pipeline_latency=0.05):
        self.pipeline_latency = float(pipeline_latency)

    def horizon(self, t_state, t_now, lead):
        # t_state: 필터 상태 시각, lead: 원하는 선행시간(예: prediction_horizon)
        return (t_now - t_state) + self.pipeline_latency + float(lead)

    def predict(self, kf, t_state, t_now, lead):
        h = self.horizon(t_state, t_now, lead)
        return kf.predict_future(h), h
```

`lead` 는 파라미터 `prediction_horizon`. 나중에 로봇 제어팀이 실제 이동시간을
알려줄 수 있으면 그 값을 `lead` 로 대체.

---

## 8. `tracking_node.py` 스켈레톤

```python
import rclpy
from rclpy.node import Node
from assembly_interfaces.msg import DetectedObject, PredictedTarget

from .kalman_filter import ConstantVelocityKF
from .motion_predictor import MotionPredictor


class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking_node")

        self.declare_parameters("", [
            ("input_topic", "/vision/detected_object"),
            ("output_topic", "/tracking/predicted_target"),
            ("track_type", "target"),
            ("update_rate_hz", 50.0),
            ("belt_speed", 50.0),
            ("process_accel_std", 20.0),
            ("meas_pos_std", [3.0, 3.0, 8.0]),
            ("gate_chi2", 7.815),
            ("dt_min", 0.005), ("dt_max", 0.15),
            ("warmup_updates", 5),
            ("max_coast_time", 1.0),
            ("pipeline_latency", 0.05),
            ("prediction_horizon", 0.5),
            ("min_speed_for_prediction", 2.0),
        ])
        g = lambda k: self.get_parameter(k).value

        self.kf = ConstantVelocityKF(
            accel_std=g("process_accel_std"),
            meas_std=g("meas_pos_std"),
            belt_speed=g("belt_speed"),
            dt_min=g("dt_min"), dt_max=g("dt_max"),
        )
        self.mp = MotionPredictor(pipeline_latency=g("pipeline_latency"))

        self.track_type = g("track_type")
        self.shape = "box"
        self.t_state = None
        self.t_last_meas = None

        self.sub = self.create_subscription(
            DetectedObject, g("input_topic"), self.on_detection, 10)
        self.pub = self.create_publisher(
            PredictedTarget, g("output_topic"), 10)
        self.create_timer(1.0 / g("update_rate_hz"), self.on_timer)
        self.get_logger().info("tracking_node started")

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_detection(self, msg: DetectedObject):
        if msg.type != self.track_type:
            return
        t = self._now()
        z = [msg.x, msg.y, msg.z]
        self.shape = msg.shape

        if not self.kf.initialized:
            if self.kf.init_measurement(z, t):
                self.t_state = self.t_last_meas = t
            return

        self.kf.predict(t - self.t_state)
        self.kf.update(z,
                       gate_chi2=self.get_parameter("gate_chi2").value,
                       warmup=self.get_parameter("warmup_updates").value)
        self.t_state = t
        self.t_last_meas = t

    def on_timer(self):
        if not self.kf.initialized:
            return
        now = self._now()
        self.kf.predict(now - self.t_state)
        self.t_state = now

        if now - self.t_last_meas > self.get_parameter("max_coast_time").value:
            return                                   # 추적 유실 → 발행 중단

        lead = self.get_parameter("prediction_horizon").value
        if self.kf.speed() < self.get_parameter("min_speed_for_prediction").value:
            pos, h = self.kf.position, 0.0           # 정지로 간주
        else:
            pos, h = self.mp.predict(self.kf, self.t_state, now, lead)

        out = PredictedTarget()
        out.shape = self.shape
        out.x, out.y, out.z = float(pos[0]), float(pos[1]), float(pos[2])
        out.prediction_time = float(h)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

> 기본 `SingleThreadedExecutor` 에서 콜백·타이머 직렬 실행 → KF 상태에 락 불필요.

---

## 9. `config/tracking_params.yaml`

```yaml
tracking_node:
  ros__parameters:
    input_topic: /vision/detected_object
    output_topic: /tracking/predicted_target
    track_type: "target"

    update_rate_hz: 50.0

    # --- Kalman ---
    belt_speed: 50.0                # mm/s, 알려진 컨베이어 속도 (초기화·검증)
    process_accel_std: 20.0         # mm/s^2, 등속이라 작게 (M6 튜닝)
    meas_pos_std: [3.0, 3.0, 8.0]   # mm, z 크게. M6에서 실측
    gate_chi2: 7.815               # chi-square 95%, dof=3
    dt_min: 0.005
    dt_max: 0.15
    warmup_updates: 5

    # --- coast ---
    max_coast_time: 1.0            # s, 초과 시 예측 발행 중단

    # --- prediction ---
    pipeline_latency: 0.05         # s, 이미지 캡처→노드 수신 (M6 실측)
    prediction_horizon: 0.5        # s, PredictedTarget.prediction_time
    min_speed_for_prediction: 2.0  # mm/s 미만이면 정지로 간주
```

`setup.py` `data_files` 에 `config/*.yaml`, `launch/*` 설치. `launch` 에서 yaml 로드.

---

## 10. 튜닝 방법론 (M6)

| 항목 | 방법 |
|---|---|
| `R` (`meas_pos_std`) | 상자 **정지** bag 500+ 샘플 → `sqrt(var(x)), sqrt(var(y)), sqrt(var(z))`. bias/drift 도 확인 |
| `σ_a` (`process_accel_std`) | 50 mm/s 벨트 bag → 추정 `‖v‖` 가 **50 근처로 지연 없이** 수렴하도록. 느리면 ↑, 출렁이면 ↓ |
| NIS 일관성 | 실행 중 `last_nis` 로깅 → 평균 ≈ 3, 95% 가 7.815 이내면 정상. 크면 R/Q 과소, 작으면 과대 |
| `prediction_horizon` | horizon 별 예측 vs `t+horizon` 실제 측정 RMS 오차 곡선 → 허용오차 넘기 직전 값 |
| `pipeline_latency` | 정지 상자에서 "명령 시각 vs 반영" 로그, 또는 벨트 실험에서 예측 bias 로 역산 |

부록 A `VelocityEstimator`(슬라이딩 윈도우 최소제곱) 로 KF 속도와 교차검증.

---

## 11. 테스트 계획

### Phase 0 — 단위 테스트 (`pytest`, ROS 불필요) → M2
`test/test_kalman_filter.py`:
- 등속(50 mm/s)+가우시안 노이즈 합성 → 위치·속도 참값 수렴 (허용오차)
- 정지 궤적 → `‖v‖ → 0`
- 가변 dt(랜덤 간격) → 발산 없음
- 측정 누락(coast) 구간 → `P` 증가, `predict_future` 합리적
- 이상치 1개 주입 → NIS 게이트 reject, 상태 튐 없음
- `init_measurement` 방향 시딩: 초기 `‖v‖ ≈ 50`

`test/test_motion_predictor.py`: horizon 계산, 등속 외삽 정확도.

### Phase 1 — rosbag 리플레이 → M3, M4
- 움직이는 상자로 `/vision/detected_object` 녹화
- 노드 실행 → 측정 vs 추정 vs 예측 CSV + matplotlib
- 예측점이 이후 실제 측정과 겹치는지 시계열 비교

### Phase 2 — RViz → M5
- `visualization_msgs/MarkerArray`: 측정(빨강)/추정(초록)/예측(파랑)/속도 화살표
- 손으로 ArUco 가려 coast·발행중단·재개 육안 확인

### Phase 3 — 실장비 정지 로봇 드라이런 → M6
- 로봇 홈 정지, 벨트만 구동
- 예측 상자 위치 vs 실제 오차 측정 (여러 지점), 파라미터 보정

---

## 12. 마일스톤

### M0 — `assembly_interfaces` 에 `PredictedTarget` 등록  *(외부 협의, 선행 필수)*
- `CMakeLists.txt` `rosidl_generate_interfaces()` 에 `"msg/PredictedTarget.msg"` 추가
- 완료: `ros2 interface show assembly_interfaces/msg/PredictedTarget` 성공
- 지연되면 폴백(3장)으로 M1 진행 후 나중 교체

### M1 — 패키지 골격 + 측정 파이프라인
- `package.xml`: `<depend>assembly_interfaces</depend>` 추가
- `setup.py`: `console_scripts` → `tracking_node` 하나, `config`/`launch` 설치
- `launch/tracking.launch.py`: `tracking_node` 하나 + `tracking_params.yaml`
- `config/tracking_params.yaml` 작성
- `tracking_node.py`: `/vision/detected_object` 구독 → `type=="target"` 필터 →
  `(x,y,z)` 로깅 + **passthrough** 로 `PredictedTarget`(`x,y,z`=측정값, `prediction_time=0`) 발행
- 완료: `colcon build` 통과, `ros2 topic echo /tracking/predicted_target` 에 target 값만 나옴 (`object` 는 안 나옴)
- 산출물: `INTERFACE.md` 초안

### M2 — 칼만 필터 코어 + 단위 테스트  *(ROS 불필요)*
- `kalman_filter.py`: `ConstantVelocityKF` (6장) — `_F`/`_Q`, predict/update(Joseph), 50 mm/s 방향 시딩 초기화, NIS 게이팅, `predict_future`
- `test/test_kalman_filter.py`: Phase 0 전 항목
- 완료: `colcon test` / `pytest` 전부 통과

### M3 — 필터 통합: 실시간 위치 추정
- `tracking_node.py` 에 KF 연결: 콜백=predict+update, 타이머(50Hz)=predict
- 수신 시각 dt + clamp, 초기화 2측정 후 시작
- (아직 예측 없이) 현재 추정 위치를 `PredictedTarget`(`prediction_time≈0`) 로 발행
- 완료: rosbag 리플레이에서 측정 노이즈 대비 매끄러운 추정, 추정 `‖v‖ ≈ 50`

### M4 — 미래 위치 예측
- `motion_predictor.py` 연결: `horizon = (now−t_state) + pipeline_latency + prediction_horizon`
- `predict_future(horizon)` → `PredictedTarget{shape, x, y, z, prediction_time=horizon}` 연속 발행
- `min_speed_for_prediction` 미만 → 현재 위치, `max_coast_time` 초과 → 발행 중단
- 완료: 리플레이에서 예측점이 `t+horizon` 실제 측정과 시계열 일치

### M5 — 시각화 + 오프라인 분석 도구  *(패키지 내부)*
- `/tracking/markers` (`visualization_msgs/MarkerArray`): 측정/추정/예측/속도
- `scripts/analyze.py`: CSV 로깅, matplotlib (측정 vs 추정 vs 예측, NIS 시계열, 예측오차 vs horizon)
- 완료: RViz 로 스무딩·coast 확인, 튜닝 근거 플롯 확보

### M6 — 튜닝 & 검증  *(녹화 bag 기반)*
- 정지 bag → `R` 실측 → `meas_pos_std`
- 50 mm/s bag → `σ_a` 튜닝 (추정 속도 50 근처, 지연 최소)
- NIS 일관성 (평균 ≈ 3)
- 예측오차 vs horizon 곡선 → `prediction_horizon` 상한
- occlusion 테스트 → coast 동작
- 완료: `tracking_params.yaml` 확정값 + 튜닝 리포트

### M7 — 인계 (`INTERFACE.md` 확정)
- 토픽 계약: `/tracking/predicted_target` (`PredictedTarget`), 좌표계 BASE, 단위 mm,
  `prediction_time` 의미, 발행 주기, coast 시 발행 중단
- 로봇 제어팀용 구독 의사코드 (신선도 timeout → 폴백)
- 알려진 한계: `DetectedObject` 무 timestamp → 수신 시각 dt 지터 정량치, header 추가 제안

---

## 13. 리스크 및 대응

| 리스크 | 영향 | 대응 |
|---|---|---|
| `PredictedTarget` 미빌드 (CMakeLists 미등록) | 발행 불가 | **M0 선행**, 지연 시 `Float64MultiArray` 폴백 |
| `DetectedObject` 무 timestamp (수정 불가) | dt 지터 → 속도 추정 오차 | 수신 시각 dt + clamp, M6 에서 지터 정량화, header 추가는 제어팀에 제안 |
| 로봇 팔이 ArUco 가림 | 측정 장시간 끊김 | CV 모델 coast + `max_coast_time` 후 발행 중단, 재측정 시 재개 |
| vision BASE 변환이 `current_robot_pose` 의존 | 팔 이동 중 측정에 로봇 자세 노이즈 결합 | 관측 구간엔 팔 정지 가정, 큰 튐은 NIS 게이트가 흡수 |
| depth(z) 노이즈 큼 | z 추정 출렁임 | `rz` 크게, `σ_a` 작게 |
| `σ_a` 과대 튜닝 | 속도 노이즈 → 예측 오버슈트 | NIS 일관성으로 상한 결정, `‖v‖` 를 50±margin 로 sanity 체크 |
| 초기화 시 상자 정지/노이즈 | 방향 불명 → 초기 속도 0 | `‖z₂−z₁‖` 임계값으로 분기, 이후 필터가 수렴 |
| 예측 horizon 과대 | 오차 누적 | M6 곡선으로 상한, 로봇 실제 이동시간 확보되면 대체 |

---

## 부록 A — `velocity_estimator.py` (튜닝 보조, 선택)

```python
from collections import deque
import numpy as np


class VelocityEstimator:
    """슬라이딩 윈도우 최소제곱 기울기. KF 속도 교차검증용."""

    def __init__(self, window=8):
        self.buf = deque(maxlen=window)

    def add(self, t, p):
        self.buf.append((float(t), np.asarray(p, float)))

    def estimate(self):
        if len(self.buf) < 2:
            return np.zeros(3)
        ts = np.array([b[0] for b in self.buf]); ts -= ts[0]
        ps = np.stack([b[1] for b in self.buf])
        A = np.vstack([ts, np.ones_like(ts)]).T
        sol, *_ = np.linalg.lstsq(A, ps, rcond=None)
        return sol[0]
```
