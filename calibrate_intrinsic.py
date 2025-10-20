"""
Intrinsic Camera Calibration Script

Calibrates a single camera using a chessboard pattern.
Calculates intrinsic parameters (camera matrix and distortion coefficients).

Usage:
    1. Print a chessboard pattern (recommended: 9x6 squares, each 25mm)
    2. Run this script and capture ~20-30 images from different angles
    3. Press SPACE to capture, 'q' when done
    4. Calibration results are saved to JSON file
"""

import cv2
import numpy as np
import sys
from pathlib import Path
from hsv_tracker.stereo.calibration import CameraIntrinsics, save_camera_intrinsics

# Chessboard parameters
CHESSBOARD_SIZE = (9, 6)  # Number of inner corners (columns, rows)
SQUARE_SIZE_MM = 25.0  # Size of a square in millimeters

# Minimum number of images required for calibration
MIN_IMAGES = 10

# Camera settings
CAMERA_INDEX = 0
CAMERA_NAME = "camera"


def main():
    """Main calibration loop."""
    print("=" * 70)
    print("INTRINSIC CAMERA CALIBRATION")
    print("=" * 70)
    print(f"\nChessboard configuration:")
    print(f"  Inner corners: {CHESSBOARD_SIZE[0]} x {CHESSBOARD_SIZE[1]}")
    print(f"  Square size: {SQUARE_SIZE_MM} mm")
    print(f"\nInstructions:")
    print("  1. Show the chessboard pattern to the camera")
    print("  2. Press SPACE to capture when corners are detected (green)")
    print("  3. Capture from different angles and distances")
    print(f"  4. Minimum {MIN_IMAGES} images required")
    print("  5. Press 'q' when done to start calibration")
    print("=" * 70 + "\n")

    # Prepare object points (3D points in real world space)
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM  # Scale to actual size

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world
    imgpoints = []  # 2D points in image plane

    # Open camera
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"Error: Cannot open camera {CAMERA_INDEX}")
        return 1

    # Get camera resolution
    img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    image_size = (img_width, img_height)

    print(f"Camera opened: {img_width}x{img_height}\n")

    captured_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame")
                break

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            ret_corners, corners = cv2.findChessboardCorners(
                gray,
                CHESSBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            # Draw corners if found
            display_frame = frame.copy()
            if ret_corners:
                # Refine corner positions for sub-pixel accuracy
                corners_refined = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )

                # Draw detected corners
                cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners_refined, ret_corners)

                # Display status
                status_text = f"Corners detected! Press SPACE to capture ({captured_count} images)"
                color = (0, 255, 0)
            else:
                status_text = f"No corners detected ({captured_count} images captured)"
                color = (0, 0, 255)

            # Draw status text
            cv2.putText(
                display_frame,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2
            )

            # Show frame
            cv2.imshow('Intrinsic Calibration', display_frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' ') and ret_corners:
                # Capture image
                objpoints.append(objp)
                imgpoints.append(corners_refined)
                captured_count += 1
                print(f"Image {captured_count} captured ({'OK' if captured_count >= MIN_IMAGES else f'need {MIN_IMAGES - captured_count} more'})")

            elif key == ord('q'):
                if captured_count < MIN_IMAGES:
                    print(f"\nError: Need at least {MIN_IMAGES} images (have {captured_count})")
                    continue
                else:
                    print("\nStarting calibration...")
                    break

    except KeyboardInterrupt:
        print("\n\nCalibration cancelled")
        return 1

    finally:
        cap.release()
        cv2.destroyAllWindows()

    # Perform calibration
    if captured_count < MIN_IMAGES:
        print(f"Calibration aborted: insufficient images ({captured_count} < {MIN_IMAGES})")
        return 1

    print(f"\nCalibrating with {captured_count} images...")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None
    )

    if not ret:
        print("Calibration failed!")
        return 1

    # Calculate reprojection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    mean_error = total_error / len(objpoints)

    # Print results
    print("\n" + "=" * 70)
    print("CALIBRATION RESULTS")
    print("=" * 70)
    print(f"\nCamera Matrix (K):")
    print(camera_matrix)
    print(f"\nDistortion Coefficients (k1, k2, p1, p2, k3):")
    print(dist_coeffs.ravel())
    print(f"\nFocal Length: fx={camera_matrix[0, 0]:.2f}, fy={camera_matrix[1, 1]:.2f}")
    print(f"Principal Point: cx={camera_matrix[0, 2]:.2f}, cy={camera_matrix[1, 2]:.2f}")
    print(f"\nMean Reprojection Error: {mean_error:.4f} pixels")
    print("=" * 70)

    # Create CameraIntrinsics object
    intrinsics = CameraIntrinsics(
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs.ravel(),
        image_size=image_size,
        camera_name=CAMERA_NAME
    )

    # Save to file
    output_dir = Path("calibration")
    output_dir.mkdir(exist_ok=True)
    output_file = output_dir / f"{CAMERA_NAME}_intrinsics.json"

    save_camera_intrinsics(intrinsics, str(output_file))

    print(f"\nCalibration complete!")
    print(f"Next step: Run this script again for the second camera (if using stereo)")
    print(f"Then run: python calibrate_stereo.py")

    return 0


if __name__ == "__main__":
    sys.exit(main())
