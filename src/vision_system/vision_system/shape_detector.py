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


# ============================================================
# Shape Classification
# ============================================================

def classify_shape(contour):
    """
    Classify contour shape.

    Expected shapes:
        circle
        triangle
        square
        unknown
    """

    # TODO: Implement later

    return "unknown"


# ============================================================
# Calculate Center
# ============================================================

def calculate_center(contour):
    """
    Calculate the center pixel position of a contour.

    Returns:
        (x, y)
    """

    # TODO: Implement later

    return (0, 0)