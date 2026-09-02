# ============================================================
# DetectedObject.msg
# ============================================================
# [EN]
# Custom ROS2 message used to send detected object information
# from the Vision System to other nodes.
#
# [KR]
# Vision System에서 검출한 도형 정보를 다른 Node로
# 전달하기 위한 ROS2 Custom Message.
# ============================================================

string shape
float64 x
float64 y
float64 z