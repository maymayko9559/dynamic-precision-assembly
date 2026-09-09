# ============================================================
# PredictedTarget.msg
# ============================================================
# object_tracking 이 칼만필터로 추정한 "target(상자) 위치" 를 발행하는 메시지.
#
# DetectedObject 와 동일한 필드(type, shape, x, y, z, angle) + prediction_time.
# 구독 측(로봇 컨트롤)은 DetectedObject 와 똑같이 msg.type == "target" 으로 분기하면 된다.
#
#   type            : 항상 "target"
#   shape           : "box"
#   x, y, z         : 추정 위치 (로봇 BASE 좌표계, mm)
#   angle           : 상자 방향 [deg]. 현재 미추적 -> 0.0
#   prediction_time : 몇 초 뒤 위치를 예측한 값인지 [s]. 현재 구현은 0.0 (현재 위치)
#
#   예) x = 525, prediction_time = 0.5  -> "0.5초 뒤 상자는 x=525 에 있을 것"
# ============================================================

string type
string shape

float64 x
float64 y
float64 z

float64 angle

float64 prediction_time
