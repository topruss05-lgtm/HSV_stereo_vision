"""
Visualization utilities for object tracking
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional


def draw_detection(
    frame: np.ndarray,
    x: int,
    y: int,
    radius: float,
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
    draw_centroid: bool = True,
    centroid_color: Tuple[int, int, int] = (0, 0, 255),
    centroid_radius: int = 5
) -> np.ndarray:
    """
    Draw detection circle and centroid on frame.

    Args:
        frame: Image to draw on
        x: X-coordinate of detection center
        y: Y-coordinate of detection center
        radius: Radius of detected object
        color: BGR color for circle
        thickness: Line thickness for circle
        draw_centroid: Whether to draw centroid point
        centroid_color: BGR color for centroid
        centroid_radius: Radius of centroid circle

    Returns:
        Frame with drawn detection
    """
    # Draw detection circle
    cv2.circle(frame, (x, y), int(radius), color, thickness)

    # Draw centroid
    if draw_centroid:
        cv2.circle(frame, (x, y), centroid_radius, centroid_color, -1)

    return frame


def draw_trajectory(
    frame: np.ndarray,
    trajectory: List[Tuple[int, int]],
    color: Tuple[int, int, int] = (255, 255, 0),
    thickness: int = 2,
    draw_points: bool = True,
    point_radius: int = 2
) -> np.ndarray:
    """
    Draw trajectory line on frame.

    Args:
        frame: Image to draw on
        trajectory: List of (x, y) tuples representing positions
        color: BGR color for trajectory
        thickness: Line thickness
        draw_points: Whether to draw individual points
        point_radius: Radius of trajectory points

    Returns:
        Frame with drawn trajectory
    """
    if len(trajectory) < 2:
        return frame

    # Draw lines connecting trajectory points
    for i in range(1, len(trajectory)):
        cv2.line(frame, trajectory[i - 1], trajectory[i], color, thickness)

    # Draw individual points
    if draw_points:
        for point in trajectory:
            cv2.circle(frame, point, point_radius, color, -1)

    return frame


def draw_info_overlay(
    frame: np.ndarray,
    position: Optional[Tuple[int, int]] = None,
    radius: Optional[float] = None,
    fps: Optional[float] = None,
    detection_count: Optional[int] = None,
    font_scale: float = 0.6,
    color: Tuple[int, int, int] = (255, 255, 255),
    bg_color: Optional[Tuple[int, int, int]] = (0, 0, 0),
    alpha: float = 0.7
) -> np.ndarray:
    """
    Draw information overlay on frame.

    Args:
        frame: Image to draw on
        position: Current (x, y) position
        radius: Current radius
        fps: Current FPS
        detection_count: Total number of detections
        font_scale: Font scale for text
        color: Text color (BGR)
        bg_color: Background color (BGR), None for no background
        alpha: Background transparency (0-1)

    Returns:
        Frame with drawn overlay
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    line_height = 25
    padding = 10
    y_offset = 30

    # Prepare text lines
    lines = []
    if fps is not None:
        lines.append(f"FPS: {fps:.1f}")
    if detection_count is not None:
        lines.append(f"Detections: {detection_count}")
    if position is not None:
        lines.append(f"Position: ({position[0]}, {position[1]})")
    if radius is not None:
        lines.append(f"Radius: {radius:.1f}px")

    if not lines:
        return frame

    # Calculate background rectangle size
    max_width = 0
    for line in lines:
        (w, h), _ = cv2.getTextSize(line, font, font_scale, thickness)
        max_width = max(max_width, w)

    rect_height = len(lines) * line_height + 2 * padding
    rect_width = max_width + 2 * padding

    # Draw semi-transparent background
    if bg_color is not None:
        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (10, 10),
            (10 + rect_width, 10 + rect_height),
            bg_color,
            -1
        )
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    # Draw text lines
    for i, line in enumerate(lines):
        y = y_offset + i * line_height
        cv2.putText(
            frame,
            line,
            (10 + padding, y),
            font,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA
        )

    return frame


def draw_crosshair(
    frame: np.ndarray,
    x: int,
    y: int,
    size: int = 20,
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2
) -> np.ndarray:
    """
    Draw crosshair at specified position.

    Args:
        frame: Image to draw on
        x: X-coordinate
        y: Y-coordinate
        size: Size of crosshair arms
        color: BGR color
        thickness: Line thickness

    Returns:
        Frame with drawn crosshair
    """
    # Horizontal line
    cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
    # Vertical line
    cv2.line(frame, (x, y - size), (x, y + size), color, thickness)

    return frame


def create_mask_overlay(
    frame: np.ndarray,
    mask: np.ndarray,
    color: Tuple[int, int, int] = (0, 255, 0),
    alpha: float = 0.3
) -> np.ndarray:
    """
    Create semi-transparent overlay from binary mask.

    Args:
        frame: Original frame
        mask: Binary mask
        color: BGR color for overlay
        alpha: Transparency (0-1)

    Returns:
        Frame with mask overlay
    """
    overlay = frame.copy()

    # Create colored mask
    colored_mask = np.zeros_like(frame)
    colored_mask[mask > 0] = color

    # Blend with original frame
    result = cv2.addWeighted(overlay, 1 - alpha, colored_mask, alpha, 0)

    return result
