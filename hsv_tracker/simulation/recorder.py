"""
Recorder Module
Handles video recording and parameter export
"""

import numpy as np
import cv2
import json
import os
from OpenGL.GL import *


def save_recording_parameters(filename, params):
    """Save all recording parameters to a JSON file.

    Args:
        filename: Path to the JSON file (without extension)
        params: Dictionary of parameters to save
    """
    json_filename = f"{filename}.json"
    with open(json_filename, 'w') as f:
        json.dump(params, f, indent=4)

    return json_filename


def capture_frame(width, height):
    """Capture current OpenGL frame as numpy array.

    Args:
        width: Frame width in pixels
        height: Frame height in pixels

    Returns:
        Numpy array in BGR format (OpenCV compatible)
    """
    # Read pixels from OpenGL buffer
    glReadBuffer(GL_BACK)
    pixels = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)

    # Convert to numpy array and reshape
    frame = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3)

    # Flip vertically (OpenGL coordinates vs. image coordinates)
    frame = np.flipud(frame)

    # Convert RGB to BGR for OpenCV
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    return frame


def save_video(frames, output_filename, fps, width, height):
    """Save recorded frames to MP4 video file.

    Args:
        frames: List of frames (numpy arrays in BGR format)
        output_filename: Output video file path
        fps: Frames per second
        width: Frame width in pixels
        height: Frame height in pixels

    Returns:
        True if successful, False otherwise
    """
    if len(frames) == 0:
        return False

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))

    for frame in frames:
        video_writer.write(frame)

    video_writer.release()
    return True
