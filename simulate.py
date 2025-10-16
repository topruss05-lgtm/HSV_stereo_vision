"""
TT Ball Tracker Simulator - Main Entry Point
Visualizes a rotating two-colored sphere with configurable rotation parameters.
"""

import numpy as np
import pygame
from pygame.locals import QUIT, KEYDOWN, K_ESCAPE, K_SPACE, DOUBLEBUF, OPENGL
from OpenGL.GL import *
from OpenGL.GLU import gluPerspective
import time
from datetime import datetime
import os

# Import from our custom modules
from hsv_tracker.config import *
from hsv_tracker.simulation import (
    generate_sphere,
    update_rotation,
    draw_sphere,
    capture_frame,
    save_video,
    save_recording_parameters
)


def print_simulation_info(ball_radius_mm, fov_horizontal_deg, fov_vertical_deg):
    """Print simulation parameters to console."""
    print("=" * 70)
    print("TT BALL TRACKER SIMULATOR - CAMERA & PHYSICAL PARAMETERS")
    print("=" * 70)
    print("\n--- Ball Specifications ---")
    print(f"  Ball Diameter: {TABLE_TENNIS_BALL_DIAMETER_MM} mm (ITTF regulation)")
    print(f"  Ball Radius: {ball_radius_mm} mm")
    print("\n--- Camera Sensor ---")
    print(f"  Sensor Size: {SENSOR_WIDTH_MM} x {SENSOR_HEIGHT_MM} mm")
    print(f"  Resolution: {RESOLUTION_WIDTH_PX} x {RESOLUTION_HEIGHT_PX} px")
    print(f"  Pixel Pitch: {PIXEL_PITCH_UM_X:.2f} x {PIXEL_PITCH_UM_Y:.2f} μm")
    print(f"  Aspect Ratio: {ASPECT_RATIO:.3f}:1")
    print("\n--- Camera Lens ---")
    print(f"  Focal Length: {FOCAL_LENGTH_MM} mm")
    print(f"  FOV (Horizontal): {fov_horizontal_deg:.1f}°")
    print(f"  FOV (Vertical): {fov_vertical_deg:.1f}°")
    print("\n--- Camera Position ---")
    print(f"  Distance to Ball: {CAMERA_DISTANCE_MM} mm ({CAMERA_DISTANCE_MM / 10.0} cm)")
    print("\n--- Spin Configuration ---")
    print(f"  Rotation Speed: {ROTATION_SPEED}°/s")
    print(f"  Corkscrew Spin: {CORKSCREWSPIN}")
    print(f"  Topspin/Backspin: {TOPSPIN_BACKSPIN}")
    print(f"  Sidespin: {SIDESPIN}")
    print("\n" + "=" * 70)
    print("Press ESC to exit")
    print("Press SPACE to start/stop recording")
    print("=" * 70 + "\n")


def main():
    """Main application loop."""
    # Initialize Pygame and OpenGL
    pygame.init()
    screen = pygame.display.set_mode((RESOLUTION_WIDTH_PX, RESOLUTION_HEIGHT_PX), DOUBLEBUF | OPENGL)
    pygame.display.set_caption(WINDOW_TITLE)

    # Configure OpenGL
    glEnable(GL_DEPTH_TEST)
    glClearColor(*BACKGROUND_COLOR)

    # Convert physical units to OpenGL units (mm)
    ball_radius_mm = TABLE_TENNIS_BALL_DIAMETER_MM / 2.0  # 20mm
    camera_distance_mm = CAMERA_DISTANCE_MM  # Already in mm

    # Calculate Field of View (FOV) from camera parameters
    # FOV = 2 * arctan(sensor_dimension / (2 * focal_length))
    # We use the vertical sensor dimension for vertical FOV
    fov_vertical_rad = 2 * np.arctan(SENSOR_HEIGHT_MM / (2 * FOCAL_LENGTH_MM))
    fov_vertical_deg = np.degrees(fov_vertical_rad)

    # Set up perspective projection
    # Near and far clipping planes adjusted for mm scale
    near_clip_mm = 1.0  # 1mm
    far_clip_mm = 10000.0  # 10 meters (10,000mm)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(fov_vertical_deg, ASPECT_RATIO, near_clip_mm, far_clip_mm)
    glMatrixMode(GL_MODELVIEW)

    # Generate sphere geometry using physical dimensions (mm)
    vertices, colors, indices = generate_sphere(
        ball_radius_mm,
        SPHERE_RESOLUTION,
        COLOR_TOP_HEMISPHERE,
        COLOR_BOTTOM_HEMISPHERE
    )

    # Initialize rotation state
    rotation_matrix = np.identity(3)  # Start with no rotation

    # Spin components in order: [X, Y, Z] = [Corkscrewspin, Topspin/Backspin, Sidespin]
    weights = [CORKSCREWSPIN, TOPSPIN_BACKSPIN, SIDESPIN]

    # Timing
    clock = pygame.time.Clock()
    last_time = time.time()

    # Recording state
    is_recording = False
    recording_start_time = 0.0
    recorded_frames = []

    # Calculate additional derived parameters for display
    fov_horizontal_rad = 2 * np.arctan(SENSOR_WIDTH_MM / (2 * FOCAL_LENGTH_MM))
    fov_horizontal_deg = np.degrees(fov_horizontal_rad)

    # Display physical parameters
    print_simulation_info(ball_radius_mm, fov_horizontal_deg, fov_vertical_deg)

    # Main loop
    running = True
    while running:
        # Calculate time delta
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Handle events
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                running = False
            elif event.type == KEYDOWN and event.key == K_SPACE:
                if not is_recording:
                    # Start recording
                    is_recording = True
                    recording_start_time = current_time
                    recorded_frames = []
                    print(f"\n[REC] Recording started at {datetime.now().strftime('%H:%M:%S')}")
                else:
                    # Stop recording and save video
                    is_recording = False
                    recording_duration = current_time - recording_start_time
                    print(f"[REC] Recording stopped. Duration: {recording_duration:.2f}s, Frames: {len(recorded_frames)}")

                    if len(recorded_frames) > 0:
                        # Create output folder if it doesn't exist
                        os.makedirs(RECORDING_OUTPUT_FOLDER, exist_ok=True)

                        # Generate base filename with timestamp
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        base_filename = os.path.join(RECORDING_OUTPUT_FOLDER, f"recording_{timestamp}")
                        video_filename = f"{base_filename}.mp4"

                        # Save video
                        print(f"[REC] Saving video to {video_filename}...")
                        if save_video(recorded_frames, video_filename, TARGET_FPS,
                                    RESOLUTION_WIDTH_PX, RESOLUTION_HEIGHT_PX):
                            print(f"[REC] Video saved successfully! ({len(recorded_frames)} frames)")

                            # Save parameters to JSON file
                            print(f"[REC] Saving parameters...")
                            params = {
                                # Rotation parameters
                                "ROTATION_SPEED": ROTATION_SPEED,
                                "CORKSCREWSPIN": CORKSCREWSPIN,
                                "TOPSPIN_BACKSPIN": TOPSPIN_BACKSPIN,
                                "SIDESPIN": SIDESPIN,

                                # Camera lens parameters
                                "FOCAL_LENGTH_MM": FOCAL_LENGTH_MM,

                                # Camera sensor parameters
                                "SENSOR_WIDTH_MM": SENSOR_WIDTH_MM,
                                "SENSOR_HEIGHT_MM": SENSOR_HEIGHT_MM,

                                # Camera position
                                "CAMERA_DISTANCE_MM": CAMERA_DISTANCE_MM,

                                # Resolution parameters
                                "RESOLUTION_WIDTH_PX": RESOLUTION_WIDTH_PX,
                                "RESOLUTION_HEIGHT_PX": RESOLUTION_HEIGHT_PX,

                                # Performance parameters
                                "TARGET_FPS": TARGET_FPS,

                                # Additional derived parameters
                                "PIXEL_PITCH_UM_X": PIXEL_PITCH_UM_X,
                                "PIXEL_PITCH_UM_Y": PIXEL_PITCH_UM_Y,
                                "ASPECT_RATIO": ASPECT_RATIO,

                                # Ball specifications
                                "TABLE_TENNIS_BALL_DIAMETER_MM": TABLE_TENNIS_BALL_DIAMETER_MM
                            }
                            json_filename = save_recording_parameters(base_filename, params)
                            print(f"[REC] Parameters saved to {json_filename}")
                        else:
                            print("[REC] Error saving video!")
                    else:
                        print("[REC] No frames recorded!")

        # Update rotation
        rotation_matrix = update_rotation(rotation_matrix, dt, ROTATION_SPEED, weights)

        # Render
        draw_sphere(vertices, colors, indices, rotation_matrix, camera_distance_mm)

        # If recording, capture frame
        if is_recording:
            frame = capture_frame(RESOLUTION_WIDTH_PX, RESOLUTION_HEIGHT_PX)
            recorded_frames.append(frame)

        pygame.display.flip()

        # Maintain target FPS
        clock.tick(TARGET_FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
