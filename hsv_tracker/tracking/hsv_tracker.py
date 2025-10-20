"""
HSV-based Object Tracker
Detects and tracks colored objects using HSV color space thresholding
"""

import cv2
import numpy as np
from typing import Optional, Tuple, List
from collections import deque


class HSVTracker:
    """
    HSV-based object tracker for single-camera detection.

    This class provides methods for detecting and tracking objects based on
    their HSV color values. Designed to be extensible for stereo vision and
    Kalman filtering.

    Attributes:
        lower_hsv: Lower bound of HSV color range
        upper_hsv: Upper bound of HSV color range
        min_radius: Minimum detection radius in pixels
        min_area: Minimum contour area in pixels^2
        erode_iterations: Number of erosion iterations for noise reduction
        dilate_iterations: Number of dilation iterations after erosion
        kernel_size: Kernel size for morphological operations
        trajectory: Deque storing recent centroid positions
        max_trajectory_points: Maximum number of trajectory points to store
    """

    def __init__(
        self,
        lower_hsv: np.ndarray,
        upper_hsv: np.ndarray,
        min_radius: int = 10,
        min_area: int = 100,
        erode_iterations: int = 2,
        dilate_iterations: int = 2,
        kernel_size: Tuple[int, int] = (5, 5),
        max_trajectory_points: int = 50
    ):
        """
        Initialize the HSV tracker.

        Args:
            lower_hsv: Lower HSV threshold (H: 0-179, S: 0-255, V: 0-255)
            upper_hsv: Upper HSV threshold
            min_radius: Minimum detection radius in pixels
            min_area: Minimum contour area in pixels^2
            erode_iterations: Number of erosion iterations
            dilate_iterations: Number of dilation iterations
            kernel_size: Morphological operation kernel size
            max_trajectory_points: Maximum trajectory points to store
        """
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv
        self.min_radius = min_radius
        self.min_area = min_area
        self.erode_iterations = erode_iterations
        self.dilate_iterations = dilate_iterations
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)

        # Trajectory tracking
        self.trajectory: deque = deque(maxlen=max_trajectory_points)
        self.max_trajectory_points = max_trajectory_points

        # Detection state
        self.last_detection: Optional[Tuple[int, int, float]] = None
        self.detection_count = 0

    def create_mask(self, frame: np.ndarray) -> np.ndarray:
        """
        Create binary mask from HSV thresholding.

        Args:
            frame: BGR image from camera

        Returns:
            Binary mask after morphological operations
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Apply color thresholding
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Morphological operations to reduce noise
        mask = cv2.erode(mask, self.kernel, iterations=self.erode_iterations)
        mask = cv2.dilate(mask, self.kernel, iterations=self.dilate_iterations)

        return mask

    def detect(self, frame: np.ndarray) -> Optional[Tuple[int, int, float]]:
        """
        Detect object in frame and return position and radius.

        Args:
            frame: BGR image from camera

        Returns:
            Tuple of (x, y, radius) if object detected, None otherwise
        """
        # Create mask
        mask = self.create_mask(frame)

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            self.last_detection = None
            return None

        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Check if contour area is sufficient
        area = cv2.contourArea(largest_contour)
        if area < self.min_area:
            self.last_detection = None
            return None

        # Calculate minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)

        # Check if radius is sufficient
        if radius < self.min_radius:
            self.last_detection = None
            return None

        # Convert to integers
        x, y, radius = int(x), int(y), float(radius)

        # Update detection state
        self.last_detection = (x, y, radius)
        self.detection_count += 1

        # Add to trajectory
        self.trajectory.append((x, y))

        return (x, y, radius)

    def get_trajectory(self) -> List[Tuple[int, int]]:
        """
        Get current trajectory points.

        Returns:
            List of (x, y) tuples representing recent positions
        """
        return list(self.trajectory)

    def reset_trajectory(self):
        """Clear trajectory history."""
        self.trajectory.clear()

    def get_detection_stats(self) -> dict:
        """
        Get detection statistics.

        Returns:
            Dictionary with detection statistics
        """
        return {
            'total_detections': self.detection_count,
            'trajectory_length': len(self.trajectory),
            'last_position': self.last_detection[:2] if self.last_detection else None,
            'last_radius': self.last_detection[2] if self.last_detection else None
        }

    def update_color_range(self, lower_hsv: np.ndarray, upper_hsv: np.ndarray):
        """
        Update HSV color range for detection.

        Args:
            lower_hsv: New lower HSV threshold
            upper_hsv: New upper HSV threshold
        """
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv
        self.reset_trajectory()
