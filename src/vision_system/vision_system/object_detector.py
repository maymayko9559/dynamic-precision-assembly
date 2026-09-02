# ============================================================
# object_detector.py
# ============================================================
# [EN]
# Detects the objects that the robot needs to pick up.
#
# Main Responsibilities:
# - Detect objects from the processed image
# - Identify the shape of each object
# - Calculate the center position of each object
# - Provide object information for robot pick operation
#
# [KR]
# 로봇이 집어야 하는 도형(Object)을 검출하는 파일.
#
# 주요 역할:
# - 전처리된 이미지에서 도형 검출
# - 각 도형의 종류 판별
# - 각 도형의 중심 위치 계산
# - Robot Pick 동작에 필요한 Object 정보 제공
# ============================================================


from .image_processing import (
    preprocess_image,
    find_contours,
)

from .shape_detector import (
    classify_shape,
    calculate_center,
)


class ObjectDetector:

    def __init__(self):
        pass

    # ========================================================
    # Detect Objects
    # ========================================================

    def detect(self, image):
        """
        Detect objects from the Pick ROI.

        Returns:
            List of detected objects.
        """

        # TODO: Implement later

        return []