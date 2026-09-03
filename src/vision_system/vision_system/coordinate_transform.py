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


import os
import numpy as np
from scipy.spatial.transform import Rotation


from ament_index_python.packages import get_package_share_directory

class CoordinateTransform:

    def __init__(self):
        # ====================================================
        # Load Hand-Eye Calibration Matrix
        # ====================================================

        package_share = get_package_share_directory('vision_system')

        calibration_file = os.path.join(package_share, 'config', 'T_gripper2camera.npy')

        self.T_gripper2camera = np.load(calibration_file)

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

    def camera_to_robot(self, camera_position, T_base2gripper):
            """
            Convert camera coordinates
            to Robot BASE coordinates.

            Parameters
            ----------
            camera_position : tuple
                (X, Y, Z) in camera coordinate system.

            T_base2gripper : numpy.ndarray
                4x4 transformation matrix representing
                the current robot gripper pose in BASE frame.

            Returns
            -------
            tuple
                (X, Y, Z) in Robot BASE coordinate system.
            """

            if camera_position is None:
                return None

            # ----------------------------------------------------
            # BASE -> Camera Transformation
            #
            # Same relationship used in the original
            # hand-eye calibration code.
            # ----------------------------------------------------

            T_base2camera = (T_base2gripper @ self.T_gripper2camera)

            X, Y, Z = camera_position

            camera_point = np.array([X, Y, Z, 1.0])

            base_point = (T_base2camera @ camera_point)

            return (
                float(base_point[0]),
                float(base_point[1]),
                float(base_point[2])
            )

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):

        R = Rotation.from_euler('ZYZ',[rx, ry, rz], degrees=True).as_matrix()

        T = np.eye(4)

        T[:3, :3] = R
        T[:3, 3] = [x, y, z]

        return T