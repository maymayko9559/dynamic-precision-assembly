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
# Marker Layout:
#
#   ID 0 ---------------- ID 1
#    |                      |
#    |     Target Board     |
#    |                      |
#   ID 3 ---------------- ID 2
#
#
# [KR]
# ArUco Marker를 이용하여 Target Board를 검출하는 파일.
#
# 주요 역할:
# - ArUco Marker 검출
# - Target Board 위치 계산
# - Target Board 영역 계산
# - Board ROI 추출
#
# Marker 배치:
#
#   ID 0 ---------------- ID 1
#    |                      |
#    |     Target Board     |
#    |                      |
#   ID 3 ---------------- ID 2

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


import cv2
import numpy as np
class BoardDetector:

    def __init__(self):


        # =====================================================
        # ArUco Dictionary
        # =====================================================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_250
        )

        # =====================================================
        # Detector Parameters
        # =====================================================

        self.parameters = cv2.aruco.DetectorParameters_create()


        # =====================================================
        # Board Size (in pixels)
        # This is only the output image resulution, not the real world size.
        # =====================================================
        self.board_width = 600
        self.board_height = 400


    # ========================================================
    # Detect ArUco Markers
    # ========================================================

    def detect(self, frame):
        """
        Detect the target board using ArUco markers.

        Returns:
            Board information or None.
        """

        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.parameters
        )

        return corners, ids

    
    # ========================================================
    # Extract Board ROI
    # ========================================================

    def extract_board_roi(self, frame, board_corners):
        """
        Extract the target board region from the image.
        """

        if board_corners is None:
            return None

        # =====================================================
        # Destination Coordinates
        # =====================================================

        destination = np.array(
            [
                [0, 0],
                [self.board_width - 1, 0],
                [
                    self.board_width - 1,
                    self.board_height - 1
                ],
                [0, self.board_height - 1]
            ],
            dtype=np.float32
        )

        # =====================================================
        # Perspective Transform Matrix
        # =====================================================

        matrix = cv2.getPerspectiveTransform(
            board_corners,
            destination
        )

        # =====================================================
        # Warp Image
        # =====================================================

        board_roi = cv2.warpPerspective(
            frame,
            matrix,
            (
                self.board_width,
                self.board_height
            )
        )

        return board_roi

    def board_to_camera_pixel(
        self,
        board_point,
        board_corners
    ):

        destination = np.array(
            [
                [0, 0],
                [self.board_width - 1, 0],
                [
                    self.board_width - 1,
                    self.board_height - 1
                ],
                [
                    0,
                    self.board_height - 1
                ]
            ],
            dtype=np.float32
        )

        inverse_matrix = cv2.getPerspectiveTransform(
            destination,
            board_corners
        )

        point = np.array(
            [[[board_point[0], board_point[1]]]],
            dtype=np.float32
        )

        camera_point = cv2.perspectiveTransform(
            point,
            inverse_matrix
        )

        u = int(camera_point[0][0][0])
        v = int(camera_point[0][0][1])

        return (u, v)
    # ========================================================
    # Get Board Corners
    # ========================================================

    def get_board_corners(self, corners, ids):
        """
        Get the corners of the target board based on detected ArUco markers.
        Calulate the board corners using the detected marker corners.
        ArUco IDs 0, 1,2,3

        Returns:
            
            np.array([
                top_left,
                top_right,
                bottom_right,
                bottom_left
            ])
        
            Return None if the board is not detected.
        """

        if ids is None or len(ids) < 4:
            return None 

        ids = ids.flatten() 

        # ============================================================
        # Check if all required IDs are present
        # ============================================================
        required_ids = [0, 1, 2, 3]

        for marker_id in required_ids:
            if marker_id not in ids:
                return None       

        # ===========================================================
        # Store Marker corners by Id
        # ===========================================================
        marker_corners = {}

        for marker_id, marker_coner in zip(ids, corners):
            marker_corners[int(marker_id)] = marker_coner[0]

        # 0 -------- 1
        # |          |
        # |  marker  |
        # |          |
        # 3 -------- 2 

        # ID 0: bottom-right corner
        top_left = marker_corners[0][2]

        # ID 1: bottom-left corner
        top_right = marker_corners[1][3]

        # ID 2: top-left corner
        bottom_right = marker_corners[2][0]

        # ID 3: top-right corner
        bottom_left = marker_corners[3][1]

        board_corners = np.array(
            [
                top_left,
                top_right,
                bottom_right,
                bottom_left
            ],
            dtype=np.float32
        )

        return board_corners


    # ========================================================
    # Debug
    # ========================================================

    def draw_board(self, frame, corners, ids):
        """
        Draw the detected ArUco markers on the image.
        """

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        return frame

    
    # =========================================================
    # Draw Board Boundary
    # =========================================================

    def draw_board_boundary(self, frame, board_corners):
        """
        Draw the detected board boundary.
        """

        if board_corners is None:
            return frame

        points = board_corners.astype(np.int32)

        cv2.polylines(
            frame,
            [points],
            True,
            (0, 255, 0),
            2
        )

        return frame

    # ========================================================
    # Get ArUco Marker Centers
    # ========================================================

    def get_marker_centers(self, corners, ids):

        if ids is None:
            return None

        ids = ids.flatten()

        required_ids = [0, 1, 2, 3]

        centers = {}

        for marker_id in required_ids:

            if marker_id not in ids:
                return None

        for marker_id, marker_corners in zip(ids, corners):

            marker_id = int(marker_id)

            if marker_id not in required_ids:
                continue

            # OpenCV ArUco:
            # marker_corners shape -> (1, 4, 2)
            pts = marker_corners[0]

            center = np.mean(
                pts,
                axis=0
            )

            centers[marker_id] = (
                float(center[0]),
                float(center[1])
            )

        return centers
    
    # ========================================================
    # Estimate Box Pose using solvePnP
    # ========================================================

    def estimate_box_pose(
        self,
        marker_centers,
        camera_intrinsics
    ):
        """
        Estimate the 3D pose of the box center using the
        four ArUco marker centers.

        Box center is defined as:
            (0, 0, 0)

        Marker center layout [mm]:

            ID0 (-70, -70) ---- ID1 ( 70, -70)
                |                    |
                |     BOX CENTER     |
                |       (0,0)        |
                |                    |
            ID3 (-70,  70) ---- ID2 ( 70,  70)

        Returns:
            camera_position = [x, y, z]
            rvec
            tvec

        Returns None if pose estimation fails.
        """

        if marker_centers is None:
            return None

        required_ids = [0, 1, 2, 3]

        for marker_id in required_ids:
            if marker_id not in marker_centers:
                return None

        if camera_intrinsics is None:
            return None


        # ====================================================
        # 1. Physical 3D Coordinates
        # ====================================================

        object_points = np.array(
            [
                [-70.0, -70.0, 0.0],   # ID 0
                [ 70.0, -70.0, 0.0],   # ID 1
                [ 70.0,  70.0, 0.0],   # ID 2
                [-70.0,  70.0, 0.0],   # ID 3
            ],
            dtype=np.float32
        )


        # ====================================================
        # 2. Corresponding Image Points
        # ====================================================

        image_points = np.array(
            [
                marker_centers[0],
                marker_centers[1],
                marker_centers[2],
                marker_centers[3],
            ],
            dtype=np.float32
        )


        # ====================================================
        # 3. Camera Matrix
        # ====================================================

        fx = camera_intrinsics["fx"]
        fy = camera_intrinsics["fy"]
        cx = camera_intrinsics["cx"]
        cy = camera_intrinsics["cy"]

        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64
        )


        # ====================================================
        # 4. Distortion
        # ====================================================
        #
        # 현재 CameraInfo에서 distortion coefficients를
        # 저장하지 않고 있으므로 0으로 둔다.
        #
        # 이후 msg.d 값을 저장하면 실제 distortion을
        # 사용하도록 개선할 수 있다.
        # ====================================================

        dist_coeffs = np.zeros(
            (5, 1),
            dtype=np.float64
        )


        # ====================================================
        # 5. solvePnP
        # ====================================================

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE
        )

        if not success:
            return None


        # ====================================================
        # 6. Box Center Position in Camera Coordinates
        # ====================================================
        #
        # object coordinate origin = box center
        #
        # 따라서 tvec 자체가 camera 기준
        # box-center translation이다.
        # ====================================================

        camera_position = [
            float(tvec[0][0]),
            float(tvec[1][0]),
            float(tvec[2][0]),
        ]

        return (
            camera_position,
            rvec,
            tvec
        )