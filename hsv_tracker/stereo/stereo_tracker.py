"""
Stereo Tracker with 3D Triangulation

Combines two HSV trackers for stereo vision with 3D position estimation.
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
from ..tracking.hsv_tracker import HSVTracker
from .calibration import StereoCalibration


class StereoTracker:
    """
    Stereo object tracker with 3D triangulation.

    Tracks objects in both left and right camera views and computes
    3D position using stereo triangulation.

    Attributes:
        left_tracker: HSVTracker for left camera
        right_tracker: HSVTracker for right camera
        calibration: Stereo calibration parameters
        map_left_x, map_left_y: Rectification maps for left camera
        map_right_x, map_right_y: Rectification maps for right camera
        position_3d_history: History of 3D positions
    """

    def __init__(
        self,
        calibration: StereoCalibration,
        lower_hsv: np.ndarray,
        upper_hsv: np.ndarray,
        min_radius: int = 10,
        min_area: int = 100,
        erode_iterations: int = 2,
        dilate_iterations: int = 2,
        kernel_size: Tuple[int, int] = (5, 5),
        max_trajectory_points: int = 50,
        max_3d_history: int = 100
    ):
        """
        Initialize stereo tracker.

        Args:
            calibration: StereoCalibration object
            lower_hsv: Lower HSV threshold
            upper_hsv: Upper HSV threshold
            min_radius: Minimum detection radius
            min_area: Minimum contour area
            erode_iterations: Erosion iterations
            dilate_iterations: Dilation iterations
            kernel_size: Morphological kernel size
            max_trajectory_points: Max 2D trajectory points
            max_3d_history: Max 3D position history
        """
        self.calibration = calibration

        # Create trackers for both cameras
        self.left_tracker = HSVTracker(
            lower_hsv, upper_hsv, min_radius, min_area,
            erode_iterations, dilate_iterations, kernel_size,
            max_trajectory_points
        )

        self.right_tracker = HSVTracker(
            lower_hsv, upper_hsv, min_radius, min_area,
            erode_iterations, dilate_iterations, kernel_size,
            max_trajectory_points
        )

        # Prepare rectification maps
        self._prepare_rectification_maps()

        # 3D position history
        self.position_3d_history: List[np.ndarray] = []
        self.max_3d_history = max_3d_history

        # Latest 3D position
        self.last_3d_position: Optional[np.ndarray] = None

    def _prepare_rectification_maps(self):
        """Prepare rectification maps for both cameras."""
        if self.calibration.R1 is None or self.calibration.R2 is None:
            raise ValueError("Calibration must include rectification parameters (R1, R2, P1, P2)")

        # Left camera rectification maps
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            self.calibration.left_camera.camera_matrix,
            self.calibration.left_camera.dist_coeffs,
            self.calibration.R1,
            self.calibration.P1,
            self.calibration.left_camera.image_size,
            cv2.CV_32FC1
        )

        # Right camera rectification maps
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            self.calibration.right_camera.camera_matrix,
            self.calibration.right_camera.dist_coeffs,
            self.calibration.R2,
            self.calibration.P2,
            self.calibration.right_camera.image_size,
            cv2.CV_32FC1
        )

    def rectify_frames(
        self,
        frame_left: np.ndarray,
        frame_right: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Rectify stereo image pair.

        Args:
            frame_left: Left camera frame
            frame_right: Right camera frame

        Returns:
            Tuple of (rectified_left, rectified_right)
        """
        rectified_left = cv2.remap(
            frame_left,
            self.map_left_x,
            self.map_left_y,
            cv2.INTER_LINEAR
        )

        rectified_right = cv2.remap(
            frame_right,
            self.map_right_x,
            self.map_right_y,
            cv2.INTER_LINEAR
        )

        return rectified_left, rectified_right

    def triangulate_point(
        self,
        point_left: Tuple[float, float],
        point_right: Tuple[float, float]
    ) -> Optional[np.ndarray]:
        """
        Triangulate 3D point from 2D correspondences.

        Args:
            point_left: (x, y) in left rectified image
            point_right: (x, y) in right rectified image

        Returns:
            3D point [X, Y, Z] in camera coordinates (mm), or None if invalid
        """
        if self.calibration.P1 is None or self.calibration.P2 is None:
            raise ValueError("Calibration must include projection matrices (P1, P2)")

        # Convert to homogeneous coordinates
        points_left = np.array([[point_left[0], point_left[1]]], dtype=np.float32).T
        points_right = np.array([[point_right[0], point_right[1]]], dtype=np.float32).T

        # Triangulate
        points_4d_hom = cv2.triangulatePoints(
            self.calibration.P1,
            self.calibration.P2,
            points_left,
            points_right
        )

        # Convert from homogeneous to 3D
        points_3d = points_4d_hom[:3] / points_4d_hom[3]

        return points_3d.ravel()

    def detect_and_triangulate(
        self,
        frame_left: np.ndarray,
        frame_right: np.ndarray,
        rectify: bool = True
    ) -> Tuple[
        Optional[Tuple[int, int, float]],
        Optional[Tuple[int, int, float]],
        Optional[np.ndarray]
    ]:
        """
        Detect object in both cameras and triangulate 3D position.

        Args:
            frame_left: Left camera frame
            frame_right: Right camera frame
            rectify: Whether to rectify frames before detection

        Returns:
            Tuple of (left_detection, right_detection, position_3d)
            Each detection is (x, y, radius) or None
            position_3d is [X, Y, Z] array or None
        """
        # Rectify frames if requested
        if rectify:
            frame_left, frame_right = self.rectify_frames(frame_left, frame_right)

        # Detect in both cameras
        detection_left = self.left_tracker.detect(frame_left)
        detection_right = self.right_tracker.detect(frame_right)

        # Triangulate if detected in both
        position_3d = None
        if detection_left is not None and detection_right is not None:
            point_left = (detection_left[0], detection_left[1])
            point_right = (detection_right[0], detection_right[1])

            position_3d = self.triangulate_point(point_left, point_right)

            # Add to history
            if position_3d is not None:
                self.position_3d_history.append(position_3d)
                if len(self.position_3d_history) > self.max_3d_history:
                    self.position_3d_history.pop(0)
                self.last_3d_position = position_3d

        return detection_left, detection_right, position_3d

    def get_3d_trajectory(self) -> List[np.ndarray]:
        """
        Get 3D position history.

        Returns:
            List of 3D position arrays
        """
        return self.position_3d_history.copy()

    def get_velocity_3d(self, dt: float) -> Optional[np.ndarray]:
        """
        Estimate 3D velocity from recent positions.

        Args:
            dt: Time delta between frames (seconds)

        Returns:
            3D velocity [vx, vy, vz] in mm/s, or None if insufficient data
        """
        if len(self.position_3d_history) < 2:
            return None

        # Use last two positions
        pos_current = self.position_3d_history[-1]
        pos_previous = self.position_3d_history[-2]

        # Velocity = displacement / time
        velocity = (pos_current - pos_previous) / dt

        return velocity

    def reset(self):
        """Reset both trackers and 3D history."""
        self.left_tracker.reset_trajectory()
        self.right_tracker.reset_trajectory()
        self.position_3d_history.clear()
        self.last_3d_position = None

    def get_stats(self) -> dict:
        """
        Get tracking statistics.

        Returns:
            Dictionary with tracking stats
        """
        left_stats = self.left_tracker.get_detection_stats()
        right_stats = self.right_tracker.get_detection_stats()

        return {
            'left_detections': left_stats['total_detections'],
            'right_detections': right_stats['total_detections'],
            'triangulations': len(self.position_3d_history),
            'last_3d_position': self.last_3d_position,
            'baseline_mm': self.calibration.baseline
        }
