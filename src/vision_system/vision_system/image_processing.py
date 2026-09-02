# ============================================================
# image_processing.py
# ============================================================
# [EN]
# Handles image preprocessing for object detection.
# Converts the raw camera image into a form that makes
# shape detection easier and more reliable.
#
# Main Responsibilities:
# - Grayscale conversion
# - Noise reduction using blur
# - Thresholding / edge processing
# - Prepare images for shape detection
#
# [KR]
# 도형 검출을 위한 이미지 전처리를 담당하는 파일.
# 카메라 원본 이미지를 도형을 찾기 쉬운 형태로 변환한다.
#
# 주요 역할:
# - Grayscale 변환
# - Blur를 이용한 Noise 감소
# - Threshold / Edge 처리
# - 도형 검출에 적합한 이미지 생성
# ============================================================

import rclpy
from rclpy.node import Node

class ImageProcessing(Node):

    def __init__(self,node):
        super().__init__('image_processing')
        self.node=node

def main(args=Node):
    rclpy.init(args=args)
    node = ImageProcessing()
    rclpy.spin(node)




    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

