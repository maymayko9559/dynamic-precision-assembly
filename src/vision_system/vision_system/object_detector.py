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


class ObjectDetector:

    def __init__(self):

        # Ignore very small contours to avoid noise
        self.min_area = 1000



    # ========================================================
    # Detect Objects
    # ========================================================

    def detect(self, pick_roi):
        """
        Detect objects from the Pick ROI.

        Returns:
            List of detected objects.

        Example:
            [
                {
                    "type": "object",
                    "shape": "star",
                    "center": (150, 120),
                    "area": 5000,
                    "contour": contour
                }
            ]

        Note:
            center is a PIXEL coordinate inside pick ROI.
            Robot x,y,z coordinates are calculated later by CoordinatedTransform.
        """

        objects = []

        if pick_roi is None:
            return objects

        # ========================================================
        # 1. Image Preprocessing
        # ========================================================

        processed_image = preprocess_image(pick_roi)

        if processed_image is None:
            return objects


        # ========================================================
        # 2. Find Contours
        # ========================================================

        contours = find_contours(processed_image)

        # ========================================================
        # 3. Analyze Contours
        # ========================================================

        for contour in contours:

            # ------------------------------------------------
            # 3-1. Contour Area
            # ------------------------------------------------

            area = cv2.contourArea(contour)

            if area < self.min_area:
                continue

            # ------------------------------------------------
            # 3-2 Shape Classification
            # ------------------------------------------------

            shape = classify_shape(contour)

            if shape == "unknown":
                continue

            # ------------------------------------------------
            # 3-3. Calculate Center
            # ------------------------------------------------

            center = calculate_center(contour)

            if center is None:
                continue

            # ------------------------------------------------
            # 3-4. Calculate Orientation
            # ------------------------------------------------

            angle = calculate_orientation(contour)

            if angle is None:
                continue
            # =================================================
            # Detection Result
            # =================================================

            detected_object = {
                "type": "object",
                "shape": shape,
                "center": center,
                "angle": angle,
                "area": area,
                "contour": contour
            }   

            objects.append(detected_object)

        return objects
    
             