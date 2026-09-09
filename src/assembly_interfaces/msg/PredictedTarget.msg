# ============================================================
# PredictedTarget.msg
# ============================================================
# object_tracking(tracking_node) 이 칼만필터로 추정한 "target(상자)" 상태를
# 발행하는 메시지. 로봇 컨트롤의 target_manager 역할을 이 메시지 하나가 대체한다.
#
#   type            : 항상 "target"
#   shape           : "box"
#   x, y, z         : 추정 위치 (로봇 BASE 좌표계, mm)
#                     prediction_time == 0.0 이면 "지금" 위치,
#                     > 0 이면 prediction_time 초 뒤의 예측 위치.
#   vx, vy, vz      : 칼만필터 상태 속도 (로봇 BASE 좌표계, mm/s)
#                     구독 측은 이걸로 임의 시점을 외삽한다:
#                     x_future = x + vx * (원하는horizon - prediction_time)
#   angle           : 상자 방향 [deg]. 현재 미추적 -> 0.0
#   prediction_time : x, y, z 가 몇 초 뒤를 나타내는지 [s]. 기본 0.0 (현재 위치)
#   coast           : 마지막으로 "실제 카메라 측정"을 반영한 뒤 흐른 시간 [s].
#                     0 에 가까우면 상자를 지금 보고 있다는 뜻.
#                     크면(예: > 0.3) 상자가 안 보여서 등속 예측으로만 밀고 있는 상태
#                     -> 구독 측은 이때의 x,y,z,vx.. 를 신뢰하면 안 된다.
#   valid           : 필터가 초기화됐고 아직 사용할만한 상태인지 (기본 True).
#
#   예) x=525, vx=50, prediction_time=0.0, coast=0.02  -> "지금 x=525, 초당 +50mm (측정 중)"
#      -> 0.5초 뒤 예측 x = 525 + 50 * 0.5 = 550
# ============================================================

string type
string shape

float64 x
float64 y
float64 z

float64 vx
float64 vy
float64 vz

float64 angle

float64 prediction_time
float64 coast
bool valid
