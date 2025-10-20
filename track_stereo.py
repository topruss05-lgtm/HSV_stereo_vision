"""
Stereo HSV Object Tracker - Main Entry Point

Real-time 3D object tracking using stereo vision with HSV color detection.
Requires stereo calibration file from calibrate_stereo.py.
"""

import cv2
import numpy as np
import time
import sys
from pathlib import Path
from hsv_tracker.config import (
    HSV_LOWER_ORANGE,
    HSV_UPPER_ORANGE,
    MIN_DETECTION_RADIUS_PX,
    MIN_CONTOUR_AREA_PX2,
    MORPH_ERODE_ITERATIONS,
    MORPH_DILATE_ITERATIONS,
    MORPH_KERNEL_SIZE,
    MAX_TRAJECTORY_POINTS,
    VIS_COLOR_DETECTION_CIRCLE,
    VIS_COLOR_CENTROID,
    VIS_CIRCLE_THICKNESS,
    VIS_CENTROID_RADIUS
)
from hsv_tracker.stereo import StereoCalibration, StereoTracker
from hsv_tracker.tracking import draw_detection, draw_info_overlay

# Camera indices
LEFT_CAMERA_INDEX = 0
RIGHT_CAMERA_INDEX = 1

# Calibration file path
CALIBRATION_FILE = "calibration/stereo_calibration.json"


def initialize_cameras() -> tuple:
    """
    Initialize both cameras.

    Returns:
        Tuple of (cap_left, cap_right)

    Raises:
        RuntimeError: If cameras cannot be opened
    """
    cap_left = cv2.VideoCapture(LEFT_CAMERA_INDEX)
    cap_right = cv2.VideoCapture(RIGHT_CAMERA_INDEX)

    if not cap_left.isOpened():
        raise RuntimeError(f"Cannot open left camera (index {LEFT_CAMERA_INDEX})")

    if not cap_right.isOpened():
        cap_left.release()
        raise RuntimeError(f"Cannot open right camera (index {RIGHT_CAMERA_INDEX})")

    return cap_left, cap_right


def main():
    """Main stereo tracking loop."""
    print("=" * 70)
    print("STEREO HSV OBJECT TRACKER")
    print("=" * 70)

    # Load calibration
    try:
        print(f"\nLoading stereo calibration from {CALIBRATION_FILE}...")
        calibration = StereoCalibration.load(CALIBRATION_FILE)
        print(f"  {calibration}")
        print(f"  Baseline: {calibration.baseline:.2f} mm")
    except FileNotFoundError:
        print(f"\nError: Calibration file not found: {CALIBRATION_FILE}")
        print("\nPlease run the calibration scripts first:")
        print("  1. python calibrate_intrinsic.py  # For left camera")
        print("  2. python calibrate_intrinsic.py  # For right camera")
        print("  3. python calibrate_stereo.py     # For stereo calibration")
        return 1

    # Initialize cameras
    try:
        print("\nInitializing cameras...")
        cap_left, cap_right = initialize_cameras()
        print(f"  Left camera: {int(cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        print(f"  Right camera: {int(cap_right.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap_right.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    except RuntimeError as e:
        print(f"\nError: {e}")
        return 1

    # Initialize stereo tracker
    print("\nInitializing stereo tracker...")
    tracker = StereoTracker(
        calibration=calibration,
        lower_hsv=HSV_LOWER_ORANGE,
        upper_hsv=HSV_UPPER_ORANGE,
        min_radius=MIN_DETECTION_RADIUS_PX,
        min_area=MIN_CONTOUR_AREA_PX2,
        erode_iterations=MORPH_ERODE_ITERATIONS,
        dilate_iterations=MORPH_DILATE_ITERATIONS,
        kernel_size=MORPH_KERNEL_SIZE,
        max_trajectory_points=MAX_TRAJECTORY_POINTS,
        max_3d_history=100
    )

    print(f"\nTracking parameters:")
    print(f"  HSV Range: {HSV_LOWER_ORANGE} - {HSV_UPPER_ORANGE}")
    print(f"  Min Radius: {MIN_DETECTION_RADIUS_PX}px")
    print(f"  Min Area: {MIN_CONTOUR_AREA_PX2}px²")
    print(f"\nControls:")
    print("  'q' - Quit")
    print("  'r' - Reset tracking")
    print("=" * 70 + "\n")

    # FPS calculation
    fps_start_time = time.time()
    fps_frame_count = 0
    fps = 0.0
    last_frame_time = time.time()

    print("Tracking started. Press 'q' to quit.\n")

    try:
        while True:
            # Capture frames
            ret_left, frame_left = cap_left.read()
            ret_right, frame_right = cap_right.read()

            if not ret_left or not ret_right:
                print("Error: Failed to capture frames. Exiting...")
                break

            # Calculate time delta
            current_time = time.time()
            dt = current_time - last_frame_time
            last_frame_time = current_time

            # Detect and triangulate
            detection_left, detection_right, position_3d = tracker.detect_and_triangulate(
                frame_left,
                frame_right,
                rectify=True
            )

            # Draw detections
            if detection_left is not None:
                x, y, radius = detection_left
                draw_detection(
                    frame_left,
                    x, y, radius,
                    color=VIS_COLOR_DETECTION_CIRCLE,
                    thickness=VIS_CIRCLE_THICKNESS,
                    draw_centroid=True,
                    centroid_color=VIS_COLOR_CENTROID,
                    centroid_radius=VIS_CENTROID_RADIUS
                )

            if detection_right is not None:
                x, y, radius = detection_right
                draw_detection(
                    frame_right,
                    x, y, radius,
                    color=VIS_COLOR_DETECTION_CIRCLE,
                    thickness=VIS_CIRCLE_THICKNESS,
                    draw_centroid=True,
                    centroid_color=VIS_COLOR_CENTROID,
                    centroid_radius=VIS_CENTROID_RADIUS
                )

            # Calculate FPS
            fps_frame_count += 1
            if fps_frame_count >= 30:
                fps_end_time = time.time()
                fps = fps_frame_count / (fps_end_time - fps_start_time)
                fps_start_time = fps_end_time
                fps_frame_count = 0

            # Get stats
            stats = tracker.get_stats()

            # Draw info overlay on left frame
            info_lines = []
            if fps > 0:
                info_lines.append(f"FPS: {fps:.1f}")
            info_lines.append(f"Triangulations: {stats['triangulations']}")

            if position_3d is not None:
                info_lines.append(f"3D Position (mm):")
                info_lines.append(f"  X: {position_3d[0]:.1f}")
                info_lines.append(f"  Y: {position_3d[1]:.1f}")
                info_lines.append(f"  Z: {position_3d[2]:.1f}")

                # Calculate velocity if possible
                velocity = tracker.get_velocity_3d(dt)
                if velocity is not None:
                    speed = np.linalg.norm(velocity)
                    info_lines.append(f"Speed: {speed:.1f} mm/s")

            # Draw basic overlay
            draw_info_overlay(
                frame_left,
                position=detection_left[:2] if detection_left else None,
                fps=fps,
                detection_count=stats['left_detections']
            )

            # Draw 3D info as text
            y_offset = 30
            for i, line in enumerate(info_lines):
                cv2.putText(
                    frame_right,
                    line,
                    (10, y_offset + i * 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )

            # Display frames
            cv2.imshow('Left Camera', frame_left)
            cv2.imshow('Right Camera', frame_right)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('r'):
                tracker.reset()
                print("Tracking reset")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        # Cleanup
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()

        # Print final statistics
        stats = tracker.get_stats()
        print("\n" + "=" * 70)
        print("TRACKING STATISTICS")
        print("=" * 70)
        print(f"Left camera detections: {stats['left_detections']}")
        print(f"Right camera detections: {stats['right_detections']}")
        print(f"3D triangulations: {stats['triangulations']}")
        if stats['last_3d_position'] is not None:
            pos = stats['last_3d_position']
            print(f"Last 3D position: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f} mm")
        print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
