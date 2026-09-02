# ============================================================
# coordinate_transform.py
# ============================================================
# [EN]
# Converts the detected camera coordinates into coordinates
# that can be used by the robot.
#
# Main Responsibilities:
# - Receive detected pixel coordinates
# - Convert pixel coordinates to camera coordinates
# - Convert camera coordinates to Robot BASE coordinates
#
# [KR]
# 카메라에서 검출한 좌표를 로봇이 사용할 수 있는
# 좌표로 변환하는 파일.
#
# 주요 역할:
# - 검출된 Pixel 좌표 입력
# - Pixel 좌표를 Camera 좌표로 변환
# - Camera 좌표를 Robot BASE 좌표로 변환
# ============================================================
import rclpy
from rclpy.node import Node

class CoordinateTransform(Node):

    def __init__(self,node):
        super().__init__('coordinate_transform')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node=CoordinateTransform()
    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
