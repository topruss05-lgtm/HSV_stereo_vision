"""
Mono HSV Object Tracker - Main Entry Point

Real-time object tracking using HSV color space detection.
This script provides the foundation for future stereo vision and Kalman filtering.
"""

import cv2
import time
import sys
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
    VIS_COLOR_TRAJECTORY,
    VIS_CIRCLE_THICKNESS,
    VIS_CENTROID_RADIUS,
    VIS_TRAJECTORY_THICKNESS,
    CAMERA_INDEX,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS
)
from hsv_tracker.tracking import (
    HSVTracker,
    draw_detection,
    draw_trajectory,
    draw_info_overlay
)


def initialize_camera(camera_index: int = CAMERA_INDEX) -> cv2.VideoCapture:
    """
    Initialize camera with specified parameters.

    Args:
        camera_index: Camera device index

    Returns:
        VideoCapture object

    Raises:
        RuntimeError: If camera cannot be opened
    """
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {camera_index}")

    # Try to set camera properties (may not work on all cameras)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    # Read actual camera properties
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print("=" * 70)
    print("MONO HSV OBJECT TRACKER")
    print("=" * 70)
    print(f"\nCamera initialized:")
    print(f"  Resolution: {actual_width}x{actual_height}")
    print(f"  FPS: {actual_fps:.1f}")
    print(f"\nTracking parameters:")
    print(f"  HSV Range: {HSV_LOWER_ORANGE} - {HSV_UPPER_ORANGE}")
    print(f"  Min Radius: {MIN_DETECTION_RADIUS_PX}px")
    print(f"  Min Area: {MIN_CONTOUR_AREA_PX2}px²")
    print(f"\nControls:")
    print("  'q' - Quit")
    print("  'r' - Reset trajectory")
    print("  't' - Toggle trajectory")
    print("=" * 70 + "\n")

    return cap


def main():
    """Main tracking loop."""
    try:
        # Initialize camera
        cap = initialize_camera()

        # Initialize tracker
        tracker = HSVTracker(
            lower_hsv=HSV_LOWER_ORANGE,
            upper_hsv=HSV_UPPER_ORANGE,
            min_radius=MIN_DETECTION_RADIUS_PX,
            min_area=MIN_CONTOUR_AREA_PX2,
            erode_iterations=MORPH_ERODE_ITERATIONS,
            dilate_iterations=MORPH_DILATE_ITERATIONS,
            kernel_size=MORPH_KERNEL_SIZE,
            max_trajectory_points=MAX_TRAJECTORY_POINTS
        )

        # UI state
        show_trajectory = True

        # FPS calculation
        fps_start_time = time.time()
        fps_frame_count = 0
        fps = 0.0

        print("Tracking started. Press 'q' to quit.\n")

        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame (stream end?). Exiting...")
                break

            # Detect object
            detection = tracker.detect(frame)

            # Draw detection if found
            if detection is not None:
                x, y, radius = detection
                draw_detection(
                    frame,
                    x, y, radius,
                    color=VIS_COLOR_DETECTION_CIRCLE,
                    thickness=VIS_CIRCLE_THICKNESS,
                    draw_centroid=True,
                    centroid_color=VIS_COLOR_CENTROID,
                    centroid_radius=VIS_CENTROID_RADIUS
                )

                # Draw trajectory if enabled
                if show_trajectory:
                    trajectory = tracker.get_trajectory()
                    if len(trajectory) > 1:
                        draw_trajectory(
                            frame,
                            trajectory,
                            color=VIS_COLOR_TRAJECTORY,
                            thickness=VIS_TRAJECTORY_THICKNESS,
                            draw_points=False
                        )

            # Calculate FPS
            fps_frame_count += 1
            if fps_frame_count >= 30:
                fps_end_time = time.time()
                fps = fps_frame_count / (fps_end_time - fps_start_time)
                fps_start_time = fps_end_time
                fps_frame_count = 0

            # Get detection stats
            stats = tracker.get_detection_stats()

            # Draw info overlay
            draw_info_overlay(
                frame,
                position=stats['last_position'],
                radius=stats['last_radius'],
                fps=fps,
                detection_count=stats['total_detections']
            )

            # Display frame
            cv2.imshow('HSV Tracker - Original', frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('r'):
                tracker.reset_trajectory()
                print("Trajectory reset")
            elif key == ord('t'):
                show_trajectory = not show_trajectory
                print(f"Trajectory: {'ON' if show_trajectory else 'OFF'}")

    except RuntimeError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        # Cleanup
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()

        # Print final statistics
        if 'tracker' in locals():
            stats = tracker.get_detection_stats()
            print("\n" + "=" * 70)
            print("TRACKING STATISTICS")
            print("=" * 70)
            print(f"Total detections: {stats['total_detections']}")
            print(f"Final trajectory length: {stats['trajectory_length']}")
            print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
