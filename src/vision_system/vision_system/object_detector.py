# ============================================================
# object_detector.py
# ============================================================
# [EN]
# Detects the objects that the robot needs to pick up.
#
# Main Responsibilities:
# - Detect objects from the processed image
# - Filter invalid contours
# - Identify the shape of each object
# - Calculate the center position of each object
# - Calculate object orientation
# - Provide object information for robot pick operation
#
# [KR]
# 로봇이 집어야 하는 도형(Object)을 검출하는 파일.
#
# 주요 역할:
# - 전처리된 이미지에서 도형 검출
# - 크기 / 비율을 이용한 잘못된 contour 제거
# - 각 도형의 종류 판별
# - 각 도형의 중심 위치 계산
# - 각 도형의 회전 각도 계산
# - Robot Pick 동작에 필요한 Object 정보 제공
# ============================================================

import cv2

from .image_processing import (
    preprocess_object_image,
    find_contours,
)

from .shape_detector import (
    classify_object_shape,
    calculate_center,
    calculate_orientation,
)


class ObjectDetector:

    def __init__(self):

        # ====================================================
        # Object Candidate Filter
        # ====================================================
        #
        # 현재 실제 object 로그:
        #
        # circle   ≈ 2578 px²
        # square   ≈ 1935 px²
        # triangle ≈ 2034 px²
        #
        # 너무 작은 contour는 noise,
        # 너무 큰 contour는 배경 / 케이블 / 주변 구조물일
        # 가능성이 높으므로 제거한다.
        # ====================================================

        self.min_area = 1500
        self.max_area = 4000

        # ====================================================
        # Aspect Ratio Filter
        # ====================================================
        #
        # 너무 길쭉한 contour는 cable 또는
        # background edge일 가능성이 높으므로 제거한다.
        #
        # 실제 triangle:
        # aspect ratio ≈ 1.46
        #
        # 실제 circle:
        # aspect ratio ≈ 1.02
        #
        # ====================================================

        self.min_aspect_ratio = 0.7
        self.max_aspect_ratio = 1.8


    # ========================================================
    # Detect Objects
    # ========================================================

    def detect(self, pick_roi):

        objects = []

        if pick_roi is None:
            return objects

        # Object 전용 preprocessing을 쓰고 있다면 이것 사용
        processed_image = preprocess_object_image(pick_roi)

        if processed_image is None:
            return objects

        # Binary Image
        # cv2.imshow(
        #     "Object Binary",
        #     processed_image
        # )
        # cv2.waitKey(1)

        contours = find_contours(processed_image)

        # print(
        #     f"[OBJECT DEBUG] contours found={len(contours)}"
        # )

        for contour in contours:

            area = cv2.contourArea(contour)

            # 너무 작은 noise만 제거
            if area < self.min_area:
                continue

            # print(
            #     f"[OBJECT CONTOUR] area={area:.0f}"
            # )

            # ============================================
            # IMPORTANT:
            # aspect ratio 기반 사전 reject 하지 않음
            # ============================================

            shape = classify_object_shape(contour)

            # print(
            #     f"[OBJECT CLASSIFY] "
            #     f"area={area:.0f}, "
            #     f"shape={shape}"
            # )

            # None / unknown 모두 reject
            if shape is None or shape == "unknown":
                continue

            center = calculate_center(contour)

            if center is None:
                continue

            angle = calculate_orientation(contour)

            if angle is None:
                continue

            detected_object = {
                "type": "object",
                "shape": shape,
                "center": center,
                "angle": angle,
                "area": area,
                "contour": contour,
            }

            objects.append(detected_object)

            # print(
            #     f"[OBJECT DETECTED] "
            #     f"shape={shape}, "
            #     f"center={center}, "
            #     f"angle={angle:.2f}, "
            #     f"area={area:.0f}"
            # )

        return objects