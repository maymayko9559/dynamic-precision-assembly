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

import cv2
import numpy as np



# ============================================================
# Calculate Orientation
# ============================================================

def calculate_orientation(contour):
    """
    Calculate the orientation angle of a contour.

    Contour의 회전 각도를 계산한다.

    Returns:
        angle: rotation angle in degrees
    """

    if contour is None or len(contour) < 3:
        return None

    rect = cv2.minAreaRect(contour)

    (_, _), (width, height), angle = rect

    if width == 0 or height == 0:
        return None

    # Normalize angle based on the longer side.
    if width < height:
        angle += 90.0

    # Normalize to [0, 180)
    angle = angle % 180.0

    return float(angle)


# ============================================================
# Get rotation difference
# ============================================================
def get_rotation_difference(shape, object_angle, target_angle):

    symmetry = {
        "circle": 360.0,
        "square": 90.0,
        "triangle": 120.0,
        "star": 72.0,
    }

    # Circle orientation does not matter.
    if shape == "circle":
        return 0.0

    symmetry_angle = symmetry[shape]

    delta = target_angle - object_angle
    delta = delta % symmetry_angle

    if delta > symmetry_angle / 2:
        delta -= symmetry_angle

    return delta

# ============================================================
# Calculate Center
# ============================================================

def calculate_center(contour):
    """
    Calculate the center pixel position of a contour.

    Returns:
        (cx, cy)

    Returns None if the contour is empty or invalid.
    """

    moments = cv2.moments(contour)

    if moments["m00"] == 0:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])

    return (cx, cy)


# ============================================================
# Shape Classification
# ============================================================

def classify_shape(contour):
    """
    Classify contour shape.

    Expected shapes:
        circle
        triangle
        star
        square
        unknown
    """

    area = cv2.contourArea(contour)

    if area < 500:
        return "unknown"

    perimeter = cv2.arcLength(
        contour,
        True
    )

    if perimeter == 0:
        return "unknown"

    # =====================================================
    # Bounding Box
    # =====================================================

    x, y, w, h = cv2.boundingRect(contour)

    if h == 0:
        return "unknown"

    aspect_ratio = float(w) / float(h)

    # Reject extremely long/thin contours
    if aspect_ratio > 2.0 or aspect_ratio < 0.5:
        return "unknown"

    # =====================================================
    # Shape Features
    # =====================================================

    approx = cv2.approxPolyDP(
        contour,
        0.025 * perimeter,
        True
    )

    vertices = len(approx)

    circularity = (
        4.0
        * np.pi
        * area
        / (perimeter * perimeter)
    )

    hull = cv2.convexHull(contour)

    hull_area = cv2.contourArea(hull)

    if hull_area == 0:
        return "unknown"

    solidity = area / hull_area

    # print(
    #     f"vertices={vertices}, "
    #     f"circularity={circularity:.2f}, "
    #     f"solidity={solidity:.2f}"
    # )

    # =====================================================
    # Triangle
    # =====================================================

    if vertices == 3:

        if (
            circularity > 0.45
            and solidity > 0.85
        ):
            return "triangle"

        return "unknown"

    # =====================================================
    # Star
    # =====================================================

    if vertices >= 6:

        if solidity < 0.85:
            return "star"

    # =====================================================
    # Circle
    # =====================================================

    if circularity > 0.80:

        return "circle"

    # =====================================================
    # Square
    # =====================================================

    if vertices == 4:

        if (
            0.75 <= aspect_ratio <= 1.30
            and solidity > 0.85
        ):
            return "square"

    return "unknown"