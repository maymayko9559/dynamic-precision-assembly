# ============================================================
# DetectedObject.msg
# ============================================================
# [EN]
# Custom ROS2 message used to send detected object information
# from the Vision System to other nodes.
# type: "object" or "target"
# shape: The shape of the detected object (e.g., "triangle", "rectangle").
#
# [KR]
# Vision System에서 검출한 도형 정보를 다른 Node로
# 전달하기 위한 ROS2 Custom Message.
#
# Vision Detection -> DetectedObject.msg
# ============================================================

string type
string shape
float64 x
float64 y
float64 z