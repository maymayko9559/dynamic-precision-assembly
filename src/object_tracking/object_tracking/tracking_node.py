# ============================================================
# tracking_node.py
# ============================================================
# object_tracking 패키지의 유일한 ROS2 노드.
#
# [M4] 칼만 필터로 "지금 이 순간"의 상자 위치를 noise 없이 추정해서 계속 발행
#   - /vision/detected_object 중 type=="target" 의 (x,y,z) 만 측정으로 사용
#   - 측정 콜백   : predict(dt) + update(z)   (등속 칼만필터로 noise 제거)
#   - 50Hz 타이머 : predict(dt) 로 상태를 현재 시각까지 당긴 뒤,
#                   noise 없는 현재 상자 위치를 PredictedTarget(prediction_time=0.0) 로 발행
#   - 상자가 가려지거나 잠깐 안 보여도 등속 예측으로 위치를 계속 발행 (중단 없음)
#
# 미래 외삽 / 로봇 속도 / 카메라 지연 / 속도 발행은 하지 않는다.
# 매끄러운 현재 위치만 주고, 그걸 따라가는 것은 로봇 제어 쪽 몫.
#
# dt 는 메시지에 타임스탬프가 없어 "노드 수신 시각" 차이로 계산.
# 기본 SingleThreadedExecutor 라 콜백/타이머가 직렬 실행 -> 락 불필요.
# ============================================================

import rclpy
from rclpy.node import Node

from assembly_interfaces.msg import DetectedObject, PredictedTarget

#같은 폴더의 칼만필터를 가져옴.
from .kalman_filter import ConstantVelocityKF


class TrackingNode(Node):

    def __init__(self):
        super().__init__("tracking_node")

        # ----- parameters -----
        # 원래 yaml파일에 적어두면 그 값 그대로 쓰는데, 없을 경우를 대비해 기본 값을 여기 적어둠.
        self.declare_parameter("input_topic", "/vision/detected_object")
        self.declare_parameter("output_topic", "/tracking/predicted_target")
        self.declare_parameter("track_type", "target")
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("belt_speed", 50.0)
        self.declare_parameter("process_accel_std", 20.0)
        self.declare_parameter("meas_pos_std", [3.0, 3.0, 8.0])
        self.declare_parameter("gate_chi2", 7.815)
        self.declare_parameter("dt_min", 0.005)
        self.declare_parameter("dt_max", 0.15)
        self.declare_parameter("warmup_updates", 5)

        # 긴 이름 짧게 저장
        gp = self.get_parameter
        # 한 번 쓰고 버릴거라 지역변수로
        input_topic = gp("input_topic").value
        output_topic = gp("output_topic").value
        # 얘는 계속 쓸거라 self로 노드에 보관
        self.track_type = gp("track_type").value

        # --------------- Kalman filter --------------
        # 필터 객체를 만들어 보관
        self.kf = ConstantVelocityKF(
            # yaml파일에서 꺼낸 값들을 필터로 넘김
            accel_std=gp("process_accel_std").value,
            meas_std=list(gp("meas_pos_std").value),
            belt_speed=gp("belt_speed").value,
            dt_min=gp("dt_min").value,
            dt_max=gp("dt_max").value,
            gate_chi2=gp("gate_chi2").value,
            warmup_updates=gp("warmup_updates").value,
        )
        # 이제 칼만필터를 불러왔으니까, self.kf로 predict, update같은 함수를 다 부를 수 있음.

        # ----- time bookkeeping (수신 시각 기반) -----
        # 추측이 이 시각 기준으로 맞다. (근데 아직 측정 전)
        self.t_state = None         # 필터 상태가 유효한 시각 [s]
        # 마지막 측정 시각 (측정 전)
        self.t_last_meas = None     # 마지막으로 측정을 반영한 시각 [s]
        # 마지막으로 본 도형 이름. 발행할 때 그대로 실어줄건데 기본은 'box'
        self.last_shape = "box"

        # ----- I/O -----
        # DetectedObject 양식으로 VisionManager에서 발행하는 것을 구독하고, 메세지가 오면 on_detection 함수 콜백
        self.sub = self.create_subscription(
            DetectedObject, input_topic, self.on_detection, 10)
        # PredictedTarget.msg 양식으로 /tracking/predicted_target에 발행    
        self.pub = self.create_publisher(
            PredictedTarget, output_topic, 10)

        # 주기 파라미터를 실수값으로. (50.0hz로 할 거임.)
        hz = float(gp("update_rate_hz").value)
        # 타이머 등록 //  1/50 = 0.02  // 0.02초마다 on_timer를 콜백해라.
        self.timer = self.create_timer(1.0 / hz, self.on_timer)

        # 측정받은 개수 카운터
        self._rx = 0
        # 튜닝 진단용 (M5) : NIS 지수이동평균, 게이트에서 버린 측정 수
        self._nis_ema = None        # 수용된 측정의 NIS EMA (일관성 지표, 목표 ≈ 3)
        self._n_rejected = 0        # gate_chi2 로 버린 이상치 누적 개수
        # 무슨 채널을 듣고 무슨 채널로 몇 HZ로 보낸다를 로그로 찍는것
        self.get_logger().info(
            f"tracking_node | sub={input_topic} type=='{self.track_type}' "
            f"-> pub={output_topic} @ {hz:.0f}Hz"
        )

    # --------------------------------------------------------

    def _now(self):
        # 현재 시각을 초 단위 float 으로
        # dt계산에 이 값들의 차이를 쓸 것임.
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------
    # 측정 콜백 : predict + update
    # 카메라 메시지가 올 때마다 실행
    # --------------------------------------------------------
    def on_detection(self, msg: DetectedObject):
        # msg : 구독해서 방금 받은 msg가 없다면
        if msg.type != self.track_type:
            return

        # 이 측정을 받은 시각 기록 (dt 계산 기준점)
        t = self._now()
        # 측정된 위치 3개를 리스트로 -> 이게 칼만 필터 측정값 z
        z = [msg.x, msg.y, msg.z]
        # 도형 이름 갱신 (발행할 때 씀)
        self.last_shape = msg.shape
        # 측정 받았으니까 측정 카운터 +1
        self._rx += 1

        # 첫 측정 -> 필터 켜기
        if not self.kf.initialized:
            # 위치 = 이 측정값, 속도 =0
            self.kf.init_measurement(z)
            # 두 시간을 지금으로 맞춤
            self.t_state = t
            self.t_last_meas = t
            # 초기화 되었다는 로그를 띄움.
            self.get_logger().info(
                f"filter initialized at "
                f"({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) mm"
            )
            return

        # 두 번째 측정부터 t - self.t_state = 이전 t_state 이후 흐른 시간(dt)
        # 그 만큼 필터를 앞으로 predict
        # 지난 상태 시각 -> 이번 측정 시각까지 예측한 뒤 측정 반영
        self.kf.predict(t - self.t_state)
        # 그 예측을 이번 측정 z로 update. 이상치면 accepted = False
        accepted = self.kf.update(z)
        # 시간을 지금으로 갱신
        self.t_state = t

        # 측정이 받아들여졌다면 마지막 측정시간 현재로 갱신 + NIS EMA 갱신
        if accepted:
            self.t_last_meas = t
            nis = self.kf.last_nis
            if self._nis_ema is None:
                self._nis_ema = nis
            else:
                self._nis_ema = 0.98 * self._nis_ema + 0.02 * nis
        # 이상치면 경고로그를 띄우고, 마지막 측정시간 갱신 안함
        else:
            self._n_rejected += 1
            self.get_logger().warn(
                f"measurement rejected (NIS={self.kf.last_nis:.1f}) "
                f"z=({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})",
                throttle_duration_sec=1.0,
            )

    # --------------------------------------------------------
    # 50Hz 타이머 : predict 로 상태를 현재까지 당긴 뒤 현재 위치 발행
    # 0.02초마다 실행(1/50=0.02)
    # 측정이 없어도 필터를 현재 시각까지 밀어 매끄러운 현재 위치를 계속 발행함
    # --------------------------------------------------------

    def on_timer(self):
        # 필터가 안켜져있으면 실행 안함
        if not self.kf.initialized:
            return

        now = self._now()
        #지난 실행 이후 지난 시간만큼 predict. 보통 0.02초
        self.kf.predict(now - self.t_state)
        # 시간 지금으로 갱신
        self.t_state = now

        # 마지막 측정 이후 흐른 시간 (참고용 로그. 가려져도 발행은 계속함)
        coast = now - self.t_last_meas

        # 위 predict 로 상태가 "지금 이 순간"까지 왔다.
        # 상자가 안 보여도 등속 예측으로 밀린 현재 위치를 그대로 발행한다.
        p = self.kf.position

        out = PredictedTarget()
        out.type = "target"             # 로봇 컨트롤이 DetectedObject 처럼 msg.type 로 분기
        out.shape = self.last_shape     # "box"
        out.x = float(p[0])
        out.y = float(p[1])
        out.z = float(p[2])
        out.angle = 0.0                 # 상자 방향 미추적
        out.prediction_time = 0.0       # 항상 "현재 위치" (미래 예측 X)
        self.pub.publish(out)

        # 튜닝 확인용 로그 (발행 안 함) : M5 에서 본다.
        #   speed  ≈ 실제 벨트 속도인지
        #   posStd < 측정 노이즈인지 (출력이 부드러운지)
        #   nis    ≈ 3 인지 (R·Q 균형. 계속 크면 과소, 계속 작으면 과대)
        #   rej    누적 (게이트가 정상 측정도 자주 버리면 gate_chi2 완화 검토)
        v = self.kf.velocity
        nis_str = f"{self._nis_ema:.1f}" if self._nis_ema is not None else "--"
        self.get_logger().info(
            f"est=({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm  "
            f"vel=({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}) mm/s  "
            f"speed={self.kf.speed():.1f}  posStd={self.kf.pos_std().max():.2f}  "
            f"coast={coast:.1f}s  nis={nis_str}  rej={self._n_rejected}",
            throttle_duration_sec=1.0,
        )


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


if __name__ == "__main__":
    main()
