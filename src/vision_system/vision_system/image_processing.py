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
# - GaussianBlur를 이용한 Noise 감소
# - Threshold / Edge 처리
# - 도형 검출에 적합한 이미지 생성
# ============================================================

import cv2
import numpy as np

# ============================================================
# Image Preprocessing
# ============================================================

def preprocess_image(image,
                    blur_ksize=5,
                    morph_ksize=3,
                    use_adaptive=True,
                    thresh_val=127
                     ):
    """
    Preprocess image before detection.
    """
    if image is None:
        return None
    
    # TODO:


    #===========================================================
    # Grayscale
    #===========================================================

    if len(image.shape) == 3: #RGB image일 때
        gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray_img = image


    #===========================================================
    # Blur
    #===========================================================

    # Gaussian Blur 커널이 혹시나 홀수일 경우 보정
    if blur_ksize % 2 == 0:
        blur_ksize += 1

    blurred_img = cv2.GaussianBlur(gray_img, (blur_ksize, blur_ksize), 0)


    #===========================================================
    # Threshold
    #===========================================================
    
    # 도형이 배경보다 어두울 때 -> 도형을 흰색으로 배경을 검은색으로
    if use_adaptive:
        thresh_img = cv2.adaptiveThreshold(blurred_img, 255, 
                                           cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                           cv2.THRESH_BINARY_INV, blockSize=15, C=3)

    # 도형이 배경보다 밝을 때 -> 위에 변수 False로 설정하고 도형을 검은색으로 배경을 흰색으로
    else:
        _, thresh_img = cv2.threshold(blurred_img, 0, 255, 
                                      cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,)

    # ============================================================
    # Morphology <-- 필요시 쓸 것이므로 아직 사용하지 않음.
    # ============================================================
    
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_ksize, morph_ksize))

    # # OPEN : 흰색 점 노이즈 제거
    # binary_img = cv2.morphologyEx(thresh_img, cv2.MORPH_OPEN, kernel, iterations=1)

    # # CLOSE : 구멍 메꿔서 끊어진 외곽선 연결
    # binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel, iterations=2)
     

    return thresh_img


# ============================================================
# Find Contours
# ============================================================

def find_contours(image):
    """
    Find contours from the processed image.

    Returns: List of contours
    """

    if image is None:
        return []

    contours, _ = cv2.findContours(
        image.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    return contours

# ============================================================
# Object Image Preprocessing
# ============================================================
def preprocess_object_image(image):
    """
    Object 전용 preprocessing.

    Target detection에는 영향을 주지 않는다.
    colored toy blocks를 HSV saturation/value 기반으로 분리한다.
    """

    if image is None:
        return None

    # ========================================================
    # 1. Blur
    # ========================================================

    blurred = cv2.GaussianBlur(
        image,
        (5, 5),
        0
    )

    # ========================================================
    # 2. BGR -> HSV
    # ========================================================

    hsv = cv2.cvtColor(
        blurred,
        cv2.COLOR_BGR2HSV
    )

    h, s, v = cv2.split(hsv)

    # ========================================================
    # 3. Colored object mask
    # ========================================================
    #
    # 배경은 회색/낮은 saturation,
    # 장난감은 상대적으로 saturation이 존재한다는 가정.
    #
    # square가 연한 색이라 saturation threshold를 낮게 시작.
    # ========================================================

    color_mask = cv2.inRange(
        s,
        30,
        255
    )

    # ========================================================
    # 4. Reject very dark regions
    # ========================================================
    #
    # 케이블/검은 영역 일부 제거
    # ========================================================

    bright_mask = cv2.inRange(
        v,
        70,
        255
    )

    object_mask = cv2.bitwise_and(
        color_mask,
        bright_mask
    )

    # ========================================================
    # 5. Morphology
    # ========================================================

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (3, 3)
    )

    object_mask = cv2.morphologyEx(
        object_mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=1
    )

    object_mask = cv2.morphologyEx(
        object_mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=1
    )

    return object_mask