"""
Stereo Camera Calibration Script

Calibrates the spatial relationship between two cameras (extrinsic parameters).
Requires intrinsic calibration files for both cameras.

Usage:
    1. First calibrate both cameras individually using calibrate_intrinsic.py
    2. Run this script with both cameras connected
    3. Capture ~20-30 synchronized image pairs showing the chessboard
    4. Press SPACE to capture, 'q' when done
    5. Stereo calibration results are saved to JSON file
"""

import cv2
import numpy as np
import sys
from pathlib import Path
from hsv_tracker.stereo.calibration import (
    CameraIntrinsics,
    StereoCalibration,
    load_camera_intrinsics
)

# Chessboard parameters (MUST match intrinsic calibration!)
CHESSBOARD_SIZE = (9, 6)  # Number of inner corners (columns, rows)
SQUARE_SIZE_MM = 25.0  # Size of a square in millimeters

# Minimum number of image pairs required
MIN_IMAGE_PAIRS = 10

# Camera settings
LEFT_CAMERA_INDEX = 0
RIGHT_CAMERA_INDEX = 1

# Paths to intrinsic calibration files
LEFT_INTRINSICS_FILE = "calibration/left_camera_intrinsics.json"
RIGHT_INTRINSICS_FILE = "calibration/right_camera_intrinsics.json"


def main():
    """Main stereo calibration loop."""
    print("=" * 70)
    print("STEREO CAMERA CALIBRATION")
    print("=" * 70)
    print(f"\nChessboard configuration:")
    print(f"  Inner corners: {CHESSBOARD_SIZE[0]} x {CHESSBOARD_SIZE[1]}")
    print(f"  Square size: {SQUARE_SIZE_MM} mm")
    print(f"\nInstructions:")
    print("  1. Show the SAME chessboard to BOTH cameras simultaneously")
    print("  2. Press SPACE to capture when corners detected in BOTH views")
    print("  3. Capture from different angles and distances")
    print(f"  4. Minimum {MIN_IMAGE_PAIRS} image pairs required")
    print("  5. Press 'q' when done to start calibration")
    print("=" * 70 + "\n")

    # Load intrinsic calibrations
    try:
        print(f"Loading left camera intrinsics from {LEFT_INTRINSICS_FILE}...")
        left_intrinsics = load_camera_intrinsics(LEFT_INTRINSICS_FILE)
        print(f"  {left_intrinsics}")

        print(f"Loading right camera intrinsics from {RIGHT_INTRINSICS_FILE}...")
        right_intrinsics = load_camera_intrinsics(RIGHT_INTRINSICS_FILE)
        print(f"  {right_intrinsics}\n")

    except FileNotFoundError as e:
        print(f"\nError: {e}")
        print("\nPlease run calibrate_intrinsic.py for both cameras first:")
        print(f"  1. python calibrate_intrinsic.py  # Calibrate left camera")
        print(f"  2. Rename output to {LEFT_INTRINSICS_FILE}")
        print(f"  3. python calibrate_intrinsic.py  # Calibrate right camera")
        print(f"  4. Rename output to {RIGHT_INTRINSICS_FILE}")
        print(f"  5. Run this script again")
        return 1

    # Prepare object points
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    # Arrays to store points
    objpoints = []  # 3D points
    imgpoints_left = []  # 2D points in left camera
    imgpoints_right = []  # 2D points in right camera

    # Open cameras
    cap_left = cv2.VideoCapture(LEFT_CAMERA_INDEX)
    cap_right = cv2.VideoCapture(RIGHT_CAMERA_INDEX)

    if not cap_left.isOpened():
        print(f"Error: Cannot open left camera (index {LEFT_CAMERA_INDEX})")
        return 1

    if not cap_right.isOpened():
        print(f"Error: Cannot open right camera (index {RIGHT_CAMERA_INDEX})")
        cap_left.release()
        return 1

    print(f"Cameras opened successfully\n")

    captured_count = 0

    try:
        while True:
            # Capture frames from both cameras
            ret_left, frame_left = cap_left.read()
            ret_right, frame_right = cap_right.read()

            if not ret_left or not ret_right:
                print("Error: Failed to capture frames")
                break

            # Convert to grayscale
            gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners in both images
            ret_left_corners, corners_left = cv2.findChessboardCorners(
                gray_left,
                CHESSBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            ret_right_corners, corners_right = cv2.findChessboardCorners(
                gray_right,
                CHESSBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            # Create display frames
            display_left = frame_left.copy()
            display_right = frame_right.copy()

            # Check if corners found in BOTH images
            both_found = ret_left_corners and ret_right_corners

            if ret_left_corners:
                corners_left = cv2.cornerSubPix(
                    gray_left, corners_left, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                cv2.drawChessboardCorners(display_left, CHESSBOARD_SIZE, corners_left, ret_left_corners)

            if ret_right_corners:
                corners_right = cv2.cornerSubPix(
                    gray_right, corners_right, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                cv2.drawChessboardCorners(display_right, CHESSBOARD_SIZE, corners_right, ret_right_corners)

            # Status text
            if both_found:
                status = f"READY! Press SPACE to capture ({captured_count} pairs)"
                color = (0, 255, 0)
            else:
                status = f"Waiting for corners in both views ({captured_count} pairs)"
                color = (0, 0, 255)

            # Draw status on both frames
            cv2.putText(display_left, f"LEFT - {status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(display_right, f"RIGHT - {status}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Show frames
            cv2.imshow('Left Camera', display_left)
            cv2.imshow('Right Camera', display_right)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' ') and both_found:
                # Capture synchronized pair
                objpoints.append(objp)
                imgpoints_left.append(corners_left)
                imgpoints_right.append(corners_right)
                captured_count += 1
                print(f"Pair {captured_count} captured ({'OK' if captured_count >= MIN_IMAGE_PAIRS else f'need {MIN_IMAGE_PAIRS - captured_count} more'})")

            elif key == ord('q'):
                if captured_count < MIN_IMAGE_PAIRS:
                    print(f"\nError: Need at least {MIN_IMAGE_PAIRS} pairs (have {captured_count})")
                    continue
                else:
                    print("\nStarting stereo calibration...")
                    break

    except KeyboardInterrupt:
        print("\n\nCalibration cancelled")
        return 1

    finally:
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()

    # Perform stereo calibration
    if captured_count < MIN_IMAGE_PAIRS:
        print(f"Calibration aborted: insufficient image pairs ({captured_count} < {MIN_IMAGE_PAIRS})")
        return 1

    print(f"\nPerforming stereo calibration with {captured_count} image pairs...")

    # Stereo calibration
    flags = cv2.CALIB_FIX_INTRINSIC  # Use pre-calibrated intrinsics
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpoints_left,
        imgpoints_right,
        left_intrinsics.camera_matrix,
        left_intrinsics.dist_coeffs,
        right_intrinsics.camera_matrix,
        right_intrinsics.dist_coeffs,
        left_intrinsics.image_size,
        criteria=criteria,
        flags=flags
    )

    if not ret:
        print("Stereo calibration failed!")
        return 1

    print("Stereo calibration successful!")

    # Compute rectification transforms
    print("Computing rectification transforms...")

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        left_intrinsics.camera_matrix,
        left_intrinsics.dist_coeffs,
        right_intrinsics.camera_matrix,
        right_intrinsics.dist_coeffs,
        left_intrinsics.image_size,
        R, T,
        alpha=0  # 0 = no invalid pixels, 1 = keep all source pixels
    )

    # Create StereoCalibration object
    stereo_calib = StereoCalibration(
        left_camera=left_intrinsics,
        right_camera=right_intrinsics,
        R=R,
        T=T,
        E=E,
        F=F,
        R1=R1,
        R2=R2,
        P1=P1,
        P2=P2,
        Q=Q
    )

    # Print results
    baseline = stereo_calib.baseline
    print("\n" + "=" * 70)
    print("STEREO CALIBRATION RESULTS")
    print("=" * 70)
    print(f"\nRotation (R):")
    print(R)
    print(f"\nTranslation (T):")
    print(T.ravel())
    print(f"\nBaseline: {baseline:.2f} mm ({baseline / 10:.2f} cm)")
    print(f"\nReprojection Error: {ret:.4f}")
    print("=" * 70)

    # Save to file
    output_file = "calibration/stereo_calibration.json"
    stereo_calib.save(output_file)

    print(f"\nStereo calibration complete!")
    print(f"Next step: Run python track_stereo.py")

    return 0


if __name__ == "__main__":
    sys.exit(main())
