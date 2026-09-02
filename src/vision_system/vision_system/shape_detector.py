# ============================================================
# shape_detector.py
# ============================================================
# [EN]
# Given a list of contours (from image_processing.get_contours),
# find the target shape, compute its center point, classify its
# type, and estimate a detection confidence.
#
# [KR]
# image_processing.get_contours() 로 얻은 Contour 목록에서
# 목표 도형을 찾아 중심점을 계산하고, 도형 종류를 분류하며,
# 검출 신뢰도를 추정한다.
# ============================================================

import rclpy
from rclpy.node import Node
class ShapeDetector(Node):

    def __init__(self,node):
        super().__init__('shape_detector')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetector()
    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
