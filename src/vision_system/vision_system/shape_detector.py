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

    if area < 1000:
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

    print(
        f"[SHAPE DEBUG] "
        f"vertices={vertices}, "
        f"area={area:.0f}, "
        f"circularity={circularity:.2f}, "
        f"solidity={solidity:.2f}, "
        f"aspect_ratio={aspect_ratio:.2f}"
    )

    # =====================================================
    # Circle
    # =====================================================

    if (
        vertices >= 7
        and circularity > 0.82
        and solidity > 0.95
    ):
        return "circle"


    # =====================================================
    # Star
    # =====================================================

    if (
        7 <= vertices <= 12
        and 0.40 <= circularity <= 0.65
        and 0.70 <= solidity <= 0.90
    ):
        return "star"


    # =====================================================
    # Triangle
    # =====================================================

    # 실제 장난감은 rounded corner 때문에
    # vertices가 4로 잡히는 경우도 허용
    if (
        3 <= vertices <= 4
        and 0.50 <= circularity < 0.70
        and solidity > 0.95
    ):
        return "triangle"


    # =====================================================
    # Square
    # =====================================================

    if (
        vertices == 4
        and 0.70 <= circularity <= 0.82
        and solidity > 0.95
        and 0.75 <= aspect_ratio <= 1.35
    ):
        return "square"


    return "unknown"


def classify_object_shape(contour):

    area = cv2.contourArea(contour)

    if area < 1200:
        return "unknown"

    perimeter = cv2.arcLength(
        contour,
        True
    )

    if perimeter <= 0:
        return "unknown"

    approx = cv2.approxPolyDP(
        contour,
        0.025 * perimeter,
        True
    )

    vertices = len(approx)

    x, y, w, h = cv2.boundingRect(contour)

    if h == 0:
        return "unknown"

    aspect_ratio = float(w) / float(h)

    circularity = (
        4.0
        * np.pi
        * area
        / (perimeter * perimeter)
    )

    hull = cv2.convexHull(contour)

    hull_area = cv2.contourArea(hull)

    if hull_area <= 0:
        return "unknown"

    solidity = area / hull_area

    print(
        f"[OBJECT SHAPE] "
        f"vertices={vertices}, "
        f"area={area:.0f}, "
        f"circularity={circularity:.2f}, "
        f"solidity={solidity:.2f}, "
        f"aspect={aspect_ratio:.2f}"
    )

    # =====================================================
    # Circle
    # =====================================================

    if (
        vertices >= 7
        and circularity >= 0.80
        and solidity >= 0.95
        and 0.85 <= aspect_ratio <= 1.15
    ):
        return "circle"

    # =====================================================
    # Triangle
    # =====================================================
    #
    # Rounded triangle can fluctuate between
    # vertices=3 and vertices=4.
    #
    # Current measured object features:
    # circularity ~= 0.65 ~ 0.66
    # solidity    ~= 0.97
    # aspect      ~= 0.97 ~ 0.98
    # =====================================================

    if (
        3 <= vertices <= 4
        and 0.60 <= circularity <= 0.69
        and solidity >= 0.94
        and 0.85 <= aspect_ratio <= 1.10
    ):
        return "triangle"


    # =====================================================
    # Square
    # =====================================================

    if (
        4 <= vertices <= 5
        and circularity >= 0.70
        and solidity >= 0.90
        and 0.75 <= aspect_ratio <= 1.30
    ):
        return "square"

    # =====================================================
    # Star
    # =====================================================
    #
    # Star edge is slightly rounded/broken after thresholding,
    # so approxPolyDP fluctuates between 6~8 vertices.
    # =====================================================
    if (
        7 <= vertices <= 12
        and 0.35 <= circularity <= 0.70
        and 0.60 <= solidity <= 0.88
        and 0.75 <= aspect_ratio <= 1.30
    ):
        return "star"