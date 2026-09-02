# ============================================================
# robot_node.py
# ============================================================
# [EN]
# Handles initialization and connection of the Doosan Robot
# in the ROS2 environment.
#
# Main Responsibilities:
# - Configure DR_init
# - Set Robot ID and Robot Model
# - Initialize the Doosan Robot API
# - Prepare robot control functions
#
# [KR]
# ROS2 환경에서 Doosan Robot의 초기화 및 연결을
# 담당하는 파일.
#
# 주요 역할:
# - DR_init 설정
# - Robot ID / Robot Model 설정
# - Doosan Robot API 초기화
# - Robot Control에 필요한 기능 준비
# ============================================================

import rclpy
from rclpy.node import Node

class RobotNode(Node):

    def __init__(self,node):
        super().__init__('robot_node')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()