"""
Camera and Simulation Configuration
All physical parameters for the TT Ball Tracker Simulator
"""

# ============================================================================
# COORDINATE SYSTEM DEFINITION (Inertial Frame I)
# ============================================================================
# X-axis (e_x^I): Points OUT OF the screen towards viewer
# Y-axis (e_y^I): Points to the RIGHT
# Z-axis (e_z^I): Points UPWARD (vertical)
#
# Visualization from viewer perspective:
#        Z (up)
#        |
#        |
#        +------ Y (right)
#       /
#      /
#     X (out of screen)
#
# Table Tennis Spin Directions:
# - CORKSCREWSPIN:    Rotation around X-axis (e_x^I) - out of screen
# - TOPSPIN:          Rotation in NEGATIVE Y-direction (-e_y^I) - leftward
# - BACKSPIN:         Rotation in POSITIVE Y-direction (+e_y^I) - rightward
# - SIDESPIN:         Rotation in POSITIVE Z-direction (+e_z^I) - upward
# ============================================================================

# ============================================================================
# PHYSICAL DIMENSIONS - All units in millimeters (mm)
# ============================================================================
TABLE_TENNIS_BALL_DIAMETER_MM = 40.0  # Official ITTF regulation: 40mm diameter

# ============================================================================
# CAMERA SPECIFICATIONS - Real camera parameters
# ============================================================================
# These parameters define a virtual camera with realistic physical properties

# Image sensor parameters
SENSOR_WIDTH_MM = 6.17      # Sensor width in millimeters (e.g., 1/2.3" sensor ~ typical action cam)
SENSOR_HEIGHT_MM = 4.55     # Sensor height in millimeters
RESOLUTION_WIDTH_PX = 1920  # Horizontal resolution in pixels (Full HD)
RESOLUTION_HEIGHT_PX = 1080 # Vertical resolution in pixels

# Lens parameters
FOCAL_LENGTH_MM = 8.0       # Focal length in millimeters (e.g., wide-angle lens)

# Camera position
CAMERA_DISTANCE_MM = 500.0  # Distance from ball center to camera in millimeters

# Derived parameters (calculated automatically - DO NOT EDIT)
PIXEL_PITCH_UM_X = (SENSOR_WIDTH_MM / RESOLUTION_WIDTH_PX) * 1000  # micrometers
PIXEL_PITCH_UM_Y = (SENSOR_HEIGHT_MM / RESOLUTION_HEIGHT_PX) * 1000
ASPECT_RATIO = RESOLUTION_WIDTH_PX / RESOLUTION_HEIGHT_PX

# ============================================================================
# SIMULATION SETTINGS
# ============================================================================

WINDOW_TITLE = "TT Ball Tracker Simulator"
BACKGROUND_COLOR = (0.1, 0.1, 0.15, 1.0)  # Dark blue-gray

# Sphere settings
SPHERE_RESOLUTION = 50  # Higher = smoother sphere, lower = better performance

# Table Tennis Spin Settings (adjust these to simulate different spins)
ROTATION_SPEED = 360.0  # Overall rotation speed in degrees per second

# Spin components (can be any value, will be normalized to rotation axis)
# These map to the coordinate system axes:
CORKSCREWSPIN = 0.0     # X-axis (e_x^I): Rotation around axis pointing out of screen
TOPSPIN_BACKSPIN = 1.0  # Y-axis (e_y^I): Negative = Topspin, Positive = Backspin
SIDESPIN = 0.0          # Z-axis (e_z^I): Rotation around vertical axis

# Color settings (RGBA format, values 0.0 - 1.0)
COLOR_TOP_HEMISPHERE = (1.0, 0.65, 0.0, 1.0)  # Orange
COLOR_BOTTOM_HEMISPHERE = (0.0, 0.75, 1.0, 1.0)  # Deep sky blue

# Performance settings
TARGET_FPS = 60

# Recording settings
RECORDING_OUTPUT_FOLDER = "recordings"  # Folder where recordings will be saved
