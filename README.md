# HSV Stereo Vision - Table Tennis Ball Tracker

A comprehensive tracking system for table tennis balls using HSV color detection and stereo vision with 3D position estimation.

## Features

### Mono Tracking
- **HSV Color Detection**: Real-time object detection using HSV color space thresholding
- **Trajectory Tracking**: Records and visualizes object movement paths
- **Configurable Parameters**: Adjustable HSV ranges, morphological operations, and detection thresholds
- **Real-time Visualization**: Live view with FPS counter, position, and detection statistics

### Stereo Vision
- **3D Triangulation**: Compute real-world 3D positions from stereo camera pairs
- **Camera Calibration**: Complete calibration workflow for intrinsic and extrinsic parameters
- **Stereo Rectification**: Automatic image rectification for efficient stereo matching
- **3D Velocity Estimation**: Calculate 3D velocity vectors from position history
- **Synchronized Tracking**: Simultaneous tracking in both camera views

### Simulation Module
- **Realistic Ball Physics**: ITTF-compliant 40mm diameter table tennis ball
- **Configurable Rotation**: Supports corkscrewspin, topspin/backspin, and sidespin
- **Camera Simulation**: Realistic camera parameters (sensor size, focal length, resolution)
- **Video Recording**: Record simulations with parameter export

## Project Structure

```
HSV_stereo_vision/
├── hsv_tracker/                      # Main package
│   ├── config/                       # Configuration
│   │   ├── camera_config.py          # Camera & simulation parameters
│   │   └── tracking_config.py        # HSV tracking parameters
│   ├── tracking/                     # Mono tracking
│   │   ├── hsv_tracker.py            # HSVTracker class
│   │   └── visualization.py          # Visualization utilities
│   ├── stereo/                       # Stereo vision
│   │   ├── calibration.py            # Calibration data structures
│   │   └── stereo_tracker.py         # StereoTracker class
│   └── simulation/                   # OpenGL simulation
│       ├── geometry.py               # Sphere generation, rotation
│       ├── renderer.py               # OpenGL rendering
│       └── recorder.py               # Video recording
├── calibration/                      # Calibration files (created by scripts)
├── recordings/                       # Simulation recordings (created by simulate.py)
├── simulate.py                       # Ball simulation
├── track_mono.py                     # Mono camera tracking
├── track_stereo.py                   # Stereo camera tracking
├── calibrate_intrinsic.py            # Intrinsic camera calibration
└── calibrate_stereo.py               # Stereo calibration
```

## Installation

### Requirements
- Python 3.8+
- Webcam(s) for tracking (2 cameras required for stereo)
- Chessboard pattern for calibration (9x6 recommended, 25mm squares)

### Dependencies

```bash
pip install -r requirements.txt
```

Required packages:
- **opencv-python**: Camera capture, image processing, calibration
- **numpy**: Numerical computations
- **pygame**: Window management (simulation)
- **PyOpenGL**: 3D rendering (simulation)

## Quick Start

### 1. Mono Tracking (Single Camera)

```bash
python track_mono.py
```

**Controls:**
- `q` - Quit
- `r` - Reset trajectory
- `t` - Toggle trajectory visualization

**Configuration:** Edit `hsv_tracker/config/tracking_config.py` to adjust HSV color ranges:
```python
HSV_LOWER_ORANGE = np.array([0, 100, 60])
HSV_UPPER_ORANGE = np.array([15, 255, 255])
```

### 2. Stereo Tracking (Two Cameras)

**Step 1: Calibrate Left Camera**
```bash
python calibrate_intrinsic.py
```
- Show chessboard pattern to camera
- Press SPACE to capture (~20 images from different angles)
- Press 'q' when done
- Rename output: `calibration/camera_intrinsics.json` → `calibration/left_camera_intrinsics.json`

**Step 2: Calibrate Right Camera**
```bash
python calibrate_intrinsic.py
```
- Repeat process for second camera
- Rename output: → `calibration/right_camera_intrinsics.json`

**Step 3: Stereo Calibration**
```bash
python calibrate_stereo.py
```
- Show SAME chessboard to BOTH cameras simultaneously
- Press SPACE to capture synchronized pairs (~20 pairs)
- Press 'q' when done
- Creates: `calibration/stereo_calibration.json`

**Step 4: Run Stereo Tracking**
```bash
python track_stereo.py
```

**Controls:**
- `q` - Quit
- `r` - Reset tracking

**Output:**
- Real-time 3D position (X, Y, Z) in millimeters
- 3D velocity in mm/s
- Detection statistics for both cameras

### 3. Ball Simulation

```bash
python simulate.py
```

**Controls:**
- `ESC` - Exit
- `SPACE` - Start/Stop recording

Recordings are saved to `recordings/` folder with corresponding parameter JSON files.

## Configuration

### Tracking Parameters

Edit `hsv_tracker/config/tracking_config.py`:

```python
# HSV color ranges (H: 0-179, S: 0-255, V: 0-255)
HSV_LOWER_ORANGE = np.array([0, 100, 60])
HSV_UPPER_ORANGE = np.array([15, 255, 255])

# Detection thresholds
MIN_DETECTION_RADIUS_PX = 10      # Minimum object radius
MIN_CONTOUR_AREA_PX2 = 100        # Minimum contour area

# Morphological operations (noise reduction)
MORPH_ERODE_ITERATIONS = 2
MORPH_DILATE_ITERATIONS = 2

# Visualization
MAX_TRAJECTORY_POINTS = 50        # Number of trajectory points to display
```

### Camera Parameters (Simulation)

Edit `hsv_tracker/config/camera_config.py`:

```python
# Physical dimensions
TABLE_TENNIS_BALL_DIAMETER_MM = 40.0

# Camera sensor
SENSOR_WIDTH_MM = 6.17
SENSOR_HEIGHT_MM = 4.55
RESOLUTION_WIDTH_PX = 1920
RESOLUTION_HEIGHT_PX = 1080

# Lens
FOCAL_LENGTH_MM = 8.0

# Position
CAMERA_DISTANCE_MM = 500.0

# Spin configuration
ROTATION_SPEED = 360.0            # Degrees per second
CORKSCREWSPIN = 0.0
TOPSPIN_BACKSPIN = 1.0
SIDESPIN = 0.0
```

## Architecture

### HSVTracker Class

The `HSVTracker` class provides mono camera tracking:

```python
from hsv_tracker.tracking import HSVTracker
import numpy as np

# Initialize tracker
tracker = HSVTracker(
    lower_hsv=np.array([0, 100, 60]),
    upper_hsv=np.array([15, 255, 255]),
    min_radius=10,
    min_area=100
)

# Detect object in frame
detection = tracker.detect(frame)  # Returns (x, y, radius) or None

# Get trajectory
trajectory = tracker.get_trajectory()  # Returns list of (x, y) tuples

# Get statistics
stats = tracker.get_detection_stats()
```

### StereoTracker Class

The `StereoTracker` class combines two cameras for 3D tracking:

```python
from hsv_tracker.stereo import StereoTracker, StereoCalibration

# Load calibration
calibration = StereoCalibration.load("calibration/stereo_calibration.json")

# Initialize tracker
tracker = StereoTracker(
    calibration=calibration,
    lower_hsv=np.array([0, 100, 60]),
    upper_hsv=np.array([15, 255, 255])
)

# Detect and triangulate
left_det, right_det, pos_3d = tracker.detect_and_triangulate(
    frame_left,
    frame_right
)

# pos_3d is [X, Y, Z] in millimeters
# Get 3D velocity
velocity = tracker.get_velocity_3d(dt)  # dt = time delta in seconds
```

### Calibration Classes

```python
from hsv_tracker.stereo import CameraIntrinsics, StereoCalibration

# Load single camera intrinsics
intrinsics = CameraIntrinsics.load("calibration/left_camera_intrinsics.json")
print(f"Focal length: fx={intrinsics.fx}, fy={intrinsics.fy}")

# Load stereo calibration
stereo = StereoCalibration.load("calibration/stereo_calibration.json")
print(f"Baseline: {stereo.baseline} mm")
```

## Coordinate Systems

### Simulation (Inertial Frame)

```
        Z (up)
        |
        |
        +------ Y (right)
       /
      /
     X (out of screen, towards viewer)
```

**Spin Directions:**
- **CORKSCREWSPIN**: Rotation around X-axis
- **TOPSPIN**: Rotation in -Y direction (leftward)
- **BACKSPIN**: Rotation in +Y direction (rightward)
- **SIDESPIN**: Rotation in +Z direction (upward)

### Stereo Vision (Camera Frame)

The 3D coordinates from stereo tracking are relative to the left camera:
- **X**: Horizontal (right is positive)
- **Y**: Vertical (down is positive, following OpenCV convention)
- **Z**: Depth (away from camera is positive)

## Calibration Details

### Chessboard Pattern

For best results, use:
- **Inner corners**: 9x6 (columns × rows)
- **Square size**: 25mm × 25mm
- **Print quality**: High-resolution, flat surface
- **Lighting**: Uniform, avoid shadows

### Calibration Tips

1. **Intrinsic Calibration:**
   - Capture ~20-30 images per camera
   - Vary angles (tilt, rotate) and distances
   - Cover entire image area
   - Ensure corners are detected (green overlay)
   - Target reprojection error < 0.5 pixels

2. **Stereo Calibration:**
   - Capture ~20-30 synchronized pairs
   - Both cameras must see the same board simultaneously
   - Maintain similar pose variety as intrinsic calibration
   - Ensure stable camera mounting (no movement between cameras)

3. **Validation:**
   - Check baseline value (should match physical distance)
   - Test with known objects at known distances
   - Verify 3D positions are reasonable

## Future Extensions

### Kalman Filtering

The architecture is designed to integrate Kalman filtering:

```python
# Current output provides:
position_3d = tracker.last_3d_position  # [X, Y, Z]
velocity_3d = tracker.get_velocity_3d(dt)  # [vX, vY, vZ]

# Can be used as input for Kalman filter:
# State: [X, Y, Z, vX, vY, vZ]
# Measurement: position_3d
```

### Multi-Object Tracking

The `HSVTracker` can be extended for multiple objects:
- Modify `detect()` to return multiple detections
- Use object ID assignment algorithms (Hungarian, SORT)
- Track multiple color ranges simultaneously

## Troubleshooting

### "Cannot open camera"
- Check camera index in config files
- Verify camera is not in use by another application
- Try different indices (0, 1, 2, ...)

### "Calibration file not found"
- Ensure you've run calibration scripts in correct order
- Check file paths in scripts match actual locations
- Verify JSON files are not corrupted

### Poor detection performance
- Adjust HSV color ranges for your lighting conditions
- Increase/decrease morphological operation iterations
- Adjust min_radius and min_area thresholds
- Ensure good lighting and contrast

### Inaccurate 3D positions
- Re-calibrate cameras (especially if moved)
- Verify baseline matches physical distance
- Check for lens distortion (use higher order distortion models)
- Ensure synchronized frame capture

## License

This project is part of the INM research initiative.

## Authors

- Tobias Pruß
- Claude Sonnet 4.5

## References

- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- Stereo Vision: https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
