# ============================================================
# target_detector.py
# ============================================================
# [EN]
# Detects the target positions where objects should be inserted.
#
# Main Responsibilities:
# - Detect insertion targets from the shape board
# - Identify the shape of each target
# - Calculate the center position of each target
# - Provide target information for robot insertion
#
# [KR]
# 도형을 삽입해야 하는 도형판의 Target을 검출하는 파일.
#
# 주요 역할:
# - 도형판에서 삽입 Target 검출
# - 각 Target의 도형 종류 판별
# - 각 Target의 중심 위치 계산
# - Robot Insert 동작에 필요한 Target 정보 제공
# ============================================================

import cv2


from .image_processing import (
    preprocess_image,
    find_contours,
)

from .shape_detector import (
    classify_shape,
    calculate_center,
    calculate_orientation,
)


class TargetDetector:

    def __init__(self):
        # 너무 작은 contour 제거용
        self.min_area = 1000

    # ========================================================
    # Detect Targets
    # ========================================================

    def detect(self, board_roi):
        """
        Detect targets from the Board ROI.

        Returns:
            List of detected targets.

        Example:
            [
                {
                    "type": "target",
                    "shape": "circle",
                    "center": (120, 100),
                    "area": 5200,
                    "contour": contour
                },

                {
                    "type": "target",
                    "shape": "triangle",
                    "center": (150, 300),
                    "area": 4800,
                    "contour": contour
                }
            ]

        NOTE:
            center is still a PIXEL coordinate inside Board ROI.

            Robot x, y, z coordinates are calculated later
            by CoordinateTransform.
        """

        targets = []

        if board_roi is None:
            return targets

        # ============================================================
        # 1. Image Preprocessing
        # ============================================================

        processed_image = preprocess_image(board_roi)

        if processed_image is None:
            return targets

        # ============================================================
        # 2. Find Contours
        # ============================================================

        contours = find_contours(processed_image)

        # ============================================================
        # 3. Analyze Contours
        # ============================================================

        for contour in contours:

            # --------------------------------------------------------
            # 3-1. Contour Area
            # --------------------------------------------------------

            area = cv2.contourArea(contour)

            if area < self.min_area:
                continue

            # --------------------------------------------------------
            # 3-2. Shape Classification
            # --------------------------------------------------------

            shape = classify_shape(contour)

            if shape == "unknown":
                continue

            # --------------------------------------------------------
            # 3-3. Center Calculation
            # --------------------------------------------------------

            center = calculate_center(contour)

            if center is None:
                continue

            # --------------------------------------------------------
            # 3-4. Orientation Calculation
            # --------------------------------------------------------

            angle = calculate_orientation(contour)

            if angle is None:
                continue

            # --------------------------------------------------------
            # 3-5. Detection results
            # --------------------------------------------------------


            target = {
                "type": "target",
                "shape": shape,
                "center": center,
                "angle": angle,
                "area": area,
                "contour": contour
            }

            targets.append(target)

        return targets