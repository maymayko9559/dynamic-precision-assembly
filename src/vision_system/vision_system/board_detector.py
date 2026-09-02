# ============================================================
# board_detector.py
# ============================================================
# [EN]
# Detects the target board using ArUco markers.
#
# Main Responsibilities:
# - Detect ArUco markers
# - Determine the target board position
# - Calculate the target board region
# - Extract the Board ROI
#
# [KR]
# ArUco Marker를 이용하여 Target Board를 검출하는 파일.
#
# 주요 역할:
# - ArUco Marker 검출
# - Target Board 위치 계산
# - Target Board 영역 계산
# - Board ROI 추출


# LV1
# ArUco 위치 검출
# → Board 위치 계산
# → Target 검출
# → 삽입

# LV2
# ArUco 위치 계속 확인
# → Board가 이동하면 Target 좌표 Update

# LV3
# ArUco 위치 변화
# → Board 속도 계산
# → 미래 위치 예측

# LV4
# ArUco 측정값
# → Kalman Filter
# → Position + Velocity 추정
# → 미래 Target 위치 예측

# ============================================================

class BoardDetector:

    def __init__(self):
        pass

    # ========================================================
    # Detect Board
    # ========================================================

    def detect(self, frame):
        """
        Detect the target board using ArUco markers.

        Returns:
            Board information or None.
        """

        # TODO: Implement ArUco detection

        return None

    # ========================================================
    # Extract Board ROI
    # ========================================================

    def extract_board_roi(self, frame, board_info):
        """
        Extract the target board region from the image.
        """

        # TODO: Implement Board ROI extraction

        return None

    # ========================================================
    # Board Pixel -> Image Pixel
    # ========================================================

    def board_to_image_pixel(self, point, board_info):
        """
        Convert Board ROI pixel coordinates
        to full camera image pixel coordinates.
        """

        # TODO: Implement later

        return point

    # ========================================================
    # Debug
    # ========================================================

    def draw_board(self, frame, board_info):
        """
        Draw detected board information.
        """

        # TODO: Implement later

        return frame