"""
Simulation Module
Handles ball simulation, rendering, and recording
"""

from .geometry import generate_sphere, axis_angle_to_matrix, update_rotation
from .renderer import draw_sphere
from .recorder import save_recording_parameters, capture_frame, save_video

__all__ = [
    "generate_sphere",
    "axis_angle_to_matrix",
    "update_rotation",
    "draw_sphere",
    "save_recording_parameters",
    "capture_frame",
    "save_video",
]
