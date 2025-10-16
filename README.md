# HSV Stereo Vision - Table Tennis Ball Tracker

A comprehensive tracking system for table tennis balls using HSV color detection and stereo vision.

## Project Structure

```
HSV_stereo_vision/
├── hsv_tracker/                    # Main package
│   ├── __init__.py
│   ├── config/                     # Configuration
│   │   ├── __init__.py
│   │   └── camera_config.py        # Camera & simulation parameters
│   ├── simulation/                 # Simulation module
│   │   ├── __init__.py
│   │   ├── renderer.py             # OpenGL rendering
│   │   ├── geometry.py             # Sphere generation, rotation math
│   │   └── recorder.py             # Video recording & parameter export
│   ├── tracking/                   # HSV tracking (to be implemented)
│   │   └── __init__.py
│   └── stereo/                     # Stereo vision (to be implemented)
│       └── __init__.py
├── recordings/                     # Output folder for recordings
├── simulate.py                     # Main entry point for simulation
└── README.md                       # This file
```

## Features

### Simulation Module
- **Realistic Ball Physics**: ITTF-compliant 40mm diameter table tennis ball
- **Configurable Rotation**: Supports corkscrewspin, topspin/backspin, and sidespin
- **Camera Simulation**: Realistic camera parameters (sensor size, focal length, resolution)
- **Video Recording**: Record simulations with SPACE key
- **Parameter Export**: Automatically saves all simulation parameters to JSON

### Future Modules
- **HSV Tracking**: Color-based ball detection and tracking
- **Stereo Vision**: 3D position reconstruction from stereo camera pairs

## Installation

### Requirements
```bash
pip install numpy pygame PyOpenGL opencv-python
```

### Dependencies
- Python 3.8+
- NumPy - Numerical computations
- Pygame - Window management and OpenGL context
- PyOpenGL - 3D rendering
- OpenCV (cv2) - Video recording and image processing

## Usage

### Running the Simulation

```bash
python simulate.py
```

### Controls
- **ESC**: Exit the simulation
- **SPACE**: Start/Stop recording

### Configuration

Edit camera and simulation parameters in `hsv_tracker/config/camera_config.py`:

```python
# Physical dimensions
TABLE_TENNIS_BALL_DIAMETER_MM = 40.0  # Ball diameter

# Camera sensor
SENSOR_WIDTH_MM = 6.17
SENSOR_HEIGHT_MM = 4.55
RESOLUTION_WIDTH_PX = 1920
RESOLUTION_HEIGHT_PX = 1080

# Camera lens
FOCAL_LENGTH_MM = 8.0  # Wide-angle lens

# Camera position
CAMERA_DISTANCE_MM = 500.0  # Distance to ball

# Spin configuration
ROTATION_SPEED = 360.0  # Degrees per second
CORKSCREWSPIN = 0.0
TOPSPIN_BACKSPIN = 1.0
SIDESPIN = 0.0

# Colors
COLOR_TOP_HEMISPHERE = (1.0, 0.65, 0.0, 1.0)  # Orange
COLOR_BOTTOM_HEMISPHERE = (0.0, 0.75, 1.0, 1.0)  # Sky blue
```

### Recording Output

When you record a simulation, two files are created in the `recordings/` folder:

1. **`recording_YYYYMMDD_HHMMSS.mp4`** - Video file
2. **`recording_YYYYMMDD_HHMMSS.json`** - Parameter file

Example JSON output:
```json
{
    "ROTATION_SPEED": 360.0,
    "CORKSCREWSPIN": 0.0,
    "TOPSPIN_BACKSPIN": 1.0,
    "SIDESPIN": 0.0,
    "FOCAL_LENGTH_MM": 8.0,
    "SENSOR_WIDTH_MM": 6.17,
    "SENSOR_HEIGHT_MM": 4.55,
    "CAMERA_DISTANCE_MM": 500.0,
    "RESOLUTION_WIDTH_PX": 1920,
    "RESOLUTION_HEIGHT_PX": 1080,
    "TARGET_FPS": 60,
    "PIXEL_PITCH_UM_X": 3.21,
    "PIXEL_PITCH_UM_Y": 4.21,
    "ASPECT_RATIO": 1.778,
    "TABLE_TENNIS_BALL_DIAMETER_MM": 40.0
}
```

## Coordinate System

The simulation uses an inertial reference frame:

```
        Z (up)
        |
        |
        +------ Y (right)
       /
      /
     X (out of screen, towards viewer)
```

### Spin Directions
- **CORKSCREWSPIN**: Rotation around X-axis (out of screen)
- **TOPSPIN**: Rotation in NEGATIVE Y-direction (leftward)
- **BACKSPIN**: Rotation in POSITIVE Y-direction (rightward)
- **SIDESPIN**: Rotation in POSITIVE Z-direction (upward)

## Development

### Adding New Features

The modular structure makes it easy to extend:

1. **Tracking Module** (`hsv_tracker/tracking/`)
   - Add HSV color detection
   - Implement ball tracking algorithms

2. **Stereo Module** (`hsv_tracker/stereo/`)
   - Add stereo matching
   - Implement 3D reconstruction

### Module Overview

**`hsv_tracker/config/camera_config.py`**
- All configurable parameters
- Camera specifications
- Physical constants

**`hsv_tracker/simulation/geometry.py`**
- Sphere generation
- Rotation mathematics (Rodrigues' formula)
- Axis-angle conversions

**`hsv_tracker/simulation/renderer.py`**
- OpenGL rendering
- Camera positioning
- Sphere drawing

**`hsv_tracker/simulation/recorder.py`**
- Frame capture from OpenGL
- Video encoding with OpenCV
- JSON parameter export

## License

This project is part of the INM research initiative.

## Authors

Tobias Pruß
Claude Sonnet 4.5
