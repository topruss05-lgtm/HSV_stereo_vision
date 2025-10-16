"""
Renderer Module
Handles OpenGL rendering of the sphere
"""

import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import gluLookAt


def draw_sphere(vertices, colors, indices, rotation_matrix, camera_distance_mm):
    """Render the sphere with current rotation.

    Args:
        vertices: Sphere vertex positions in mm (numpy array)
        colors: Vertex colors (numpy array)
        indices: Triangle indices (numpy array)
        rotation_matrix: Current 3x3 rotation matrix
        camera_distance_mm: Camera distance in mm
    """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    # Position camera looking along X-axis (from viewer towards screen)
    # Camera is at (camera_distance_mm, 0, 0) looking at origin (0, 0, 0)
    # Up vector is Z-axis (0, 0, 1)
    gluLookAt(camera_distance_mm, 0, 0,    # Eye position: along X-axis
              0, 0, 0,                      # Look at: origin
              0, 0, 1)                      # Up vector: Z-axis

    # Apply rotation matrix
    # OpenGL expects column-major 4x4 matrix, we have row-major 3x3
    # Convert to 4x4 homogeneous matrix and transpose for column-major
    rotation_4x4 = np.eye(4)
    rotation_4x4[:3, :3] = rotation_matrix
    glMultMatrixf(rotation_4x4.T.flatten())

    # Draw sphere
    glBegin(GL_TRIANGLES)
    for idx in indices:
        glColor4fv(colors[idx])
        glVertex3fv(vertices[idx])
    glEnd()
