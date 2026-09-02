# ============================================================
# board_detector.py
# ============================================================
# [EN]
# Detects the moving target board using an ArUco marker.
#
# Main Responsibilities:
# - Detect the ArUco marker attached to the target board
# - Estimate the position and orientation of the board
# - Provide the board region for target detection
# - Provide board position information for tracking
#
# [KR]
# ArUco Marker를 이용하여 움직이는 Target Board를
# 검출하는 파일.
#
# 주요 역할:
# - Target Board에 부착된 ArUco Marker 검출
# - Board의 위치 및 방향 계산
# - Target 검출을 위한 Board 영역 제공
# - Tracking을 위한 Board 위치 정보 제공


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