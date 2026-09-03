# ============================================================
# coordinate_transform.py
# ============================================================
# [EN]
# Converts the detected camera coordinates into coordinates
# that can be used by the robot.
#
# Main Responsibilities:
# - Receive detected pixel coordinates
# - Convert pixel coordinates to camera coordinates
# - Convert camera coordinates to Robot BASE coordinates
#
# [KR]
# 카메라에서 검출한 좌표를 로봇이 사용할 수 있는
# 좌표로 변환하는 파일.
#
# 주요 역할:
# - 검출된 Pixel 좌표 입력
# - Pixel 좌표를 Camera 좌표로 변환
# - Camera 좌표를 Robot BASE 좌표로 변환
# ============================================================


class CoordinateTransform:

    def __init__(self):
        pass

    # ========================================================
    # Pixel -> Camera
    # ========================================================


    def pixel_to_camera(self, u, v, depth, intrinsics):
        """
        Convert color pixel + aligned depth
        to camera 3D coordinates.

        Parameters
        ----------
        u, v : int
            Color image pixel coordinates.

        depth : float
            Raw aligned depth value.
            Current RealSense stream uses 16UC1.

        intrinsics : dict
            Camera intrinsic parameters:
            fx, fy, cx, cy

        Returns
        -------
        tuple
            (X, Y, Z) in camera coordinate system.
        """

        if depth <= 0:
            return None

        fx = intrinsics["fx"]
        fy = intrinsics["fy"]
        cx = intrinsics["cx"]
        cy = intrinsics["cy"]

        Z = float(depth)

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

        return (X, Y, Z)

    
    # ========================================================
    # Camera -> Robot
    # ========================================================

    def camera_to_robot(self, camera_position):
        """
        Convert camera coordinates to Robot BASE coordinates.
        """

        # TODO:
        # Camera-Robot calibration

        return None

    # ========================================================
    # Pixel -> Robot
    # ========================================================

    def pixel_to_robot(self, pixel, depth=None):
        """
        Convert pixel coordinates directly to Robot BASE.
        """

        # TODO: Implement later

        return None