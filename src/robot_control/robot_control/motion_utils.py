# ============================================================
# motion_utils.py
# ============================================================
# [EN]
# Provides reusable utility functions for robot motion.
#
# Main Responsibilities:
# - Provide common movej / movel functions
# - Move to Home / Approach positions
# - Calculate target positions and offsets
# - Reduce duplicated robot motion code
#
# [KR]
# 로봇 이동에 반복적으로 사용되는 공통 Motion 함수를
# 모아두는 Utility 파일.
#
# 주요 역할:
# - movej / movel 등의 공통 이동 기능
# - Home / Approach 위치 이동
# - Target 위치 및 Offset 계산
# - 중복되는 Robot Motion 코드 관리
# ============================================================
import rclpy
from rclpy.node import Node

class MotionUtils(Node):

    def __init__(self,node):
        super().__init__('motion_utils')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node = MotionUtils()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()