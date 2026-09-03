# ============================================================
# PredictedTarget.msg
# ============================================================
# prediction -> PredictedTarget.msg
#
# 현재 Target
# x = 500 mm
#
# 속도
# vx = 50 mm/s
#
# 0.5초 뒤 위치 예측 -> PredictedTarget
#
# x = 525 mm
# prediction_time = 0.5
#
# ============================================================

# Shape type
# "circle", "triangle", "square", "star"
string shape

# Predicted target position
float64 x
float64 y
float64 z

# Prediction time
float64 prediction_time