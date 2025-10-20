"""
Tracking Configuration
HSV color ranges and tracking parameters
"""

import numpy as np

# ============================================================================
# HSV COLOR RANGES
# ============================================================================
# HSV color space ranges for object detection
# H: 0-179, S: 0-255, V: 0-255 (OpenCV convention)

# Orange color range (default for table tennis ball)
HSV_LOWER_ORANGE = np.array([0, 100, 60])
HSV_UPPER_ORANGE = np.array([15, 255, 255])

# Additional color ranges (can be extended for multi-object tracking)
HSV_LOWER_BLUE = np.array([100, 100, 60])
HSV_UPPER_BLUE = np.array([130, 255, 255])

# ============================================================================
# MORPHOLOGICAL OPERATIONS
# ============================================================================
# Parameters for noise reduction
MORPH_ERODE_ITERATIONS = 2  # Remove small noise
MORPH_DILATE_ITERATIONS = 2  # Restore object size after erosion
MORPH_KERNEL_SIZE = (5, 5)  # Kernel size for morphological operations

# ============================================================================
# DETECTION PARAMETERS
# ============================================================================
# Minimum radius (in pixels) for valid detection
MIN_DETECTION_RADIUS_PX = 10

# Minimum contour area (in pixels^2) for valid detection
MIN_CONTOUR_AREA_PX2 = 100

# Maximum number of objects to track simultaneously
MAX_TRACKED_OBJECTS = 1  # Start with 1, can be increased for multi-object tracking

# ============================================================================
# VISUALIZATION SETTINGS
# ============================================================================
# Colors for visualization (BGR format for OpenCV)
VIS_COLOR_DETECTION_CIRCLE = (0, 255, 0)  # Green circle around detected object
VIS_COLOR_CENTROID = (0, 0, 255)  # Red dot for centroid
VIS_COLOR_TRAJECTORY = (255, 255, 0)  # Cyan for trajectory line
VIS_CIRCLE_THICKNESS = 2
VIS_CENTROID_RADIUS = 5
VIS_TRAJECTORY_THICKNESS = 2

# Maximum trajectory points to display
MAX_TRAJECTORY_POINTS = 50

# ============================================================================
# CAMERA SETTINGS
# ============================================================================
# Default camera index (0 = default camera)
CAMERA_INDEX = 0

# Camera resolution (will try to set, may fall back to camera default)
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

# Camera FPS (will try to set, may fall back to camera default)
CAMERA_FPS = 60