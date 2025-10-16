"""
Configuration Module
Contains all camera and simulation parameters
"""

from .camera_config import *

__all__ = [
    "TABLE_TENNIS_BALL_DIAMETER_MM",
    "SENSOR_WIDTH_MM",
    "SENSOR_HEIGHT_MM",
    "RESOLUTION_WIDTH_PX",
    "RESOLUTION_HEIGHT_PX",
    "FOCAL_LENGTH_MM",
    "CAMERA_DISTANCE_MM",
    "PIXEL_PITCH_UM_X",
    "PIXEL_PITCH_UM_Y",
    "ASPECT_RATIO",
    "WINDOW_TITLE",
    "BACKGROUND_COLOR",
    "SPHERE_RESOLUTION",
    "ROTATION_SPEED",
    "CORKSCREWSPIN",
    "TOPSPIN_BACKSPIN",
    "SIDESPIN",
    "COLOR_TOP_HEMISPHERE",
    "COLOR_BOTTOM_HEMISPHERE",
    "TARGET_FPS",
    "RECORDING_OUTPUT_FOLDER",
]
