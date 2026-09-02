# ============================================================
# assembly_controller.py
# ============================================================
# [EN]
# Main controller for managing the overall assembly sequence.
# Receives target information from the Vision System and
# controls the assembly process.
#
# Main Responsibilities:
# - Receive target position
# - Approach the target
# - Align with the target
# - Perform insertion
# - Manage assembly completion
#
# [KR]
# 전체 조립 작업 순서를 관리하는 메인 Controller.
# Vision System에서 Target 정보를 받아 조립 작업을 수행한다.
#
# 주요 역할:
# - Target 위치 수신
# - Target으로 접근
# - Target 위치 정렬
# - 도형 삽입
# - 작업 완료 처리
# ============================================================

import rclpy
from rclpy.node import Node

class AssemblyController(Node):

    def __init__(self,node):
        super().__init__('assembly_controller')
        self.node = node

def main(args=None):
    rclpy.init(args=args)

    node = AssemblyController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()