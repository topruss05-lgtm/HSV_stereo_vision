"""
Geometry Module
Handles sphere generation and rotation mathematics
"""

import numpy as np


def generate_sphere(radius, resolution, color_top, color_bottom):
    """Generate sphere vertices, colors, and triangle indices.

    Args:
        radius: Sphere radius in mm
        resolution: Number of subdivisions (higher = smoother sphere)
        color_top: RGBA color for top hemisphere (z > 0)
        color_bottom: RGBA color for bottom hemisphere (z <= 0)

    Returns:
        Tuple of (vertices, colors, indices) as numpy arrays
    """
    vertices = []
    colors = []

    for i in range(resolution + 1):
        theta = i * np.pi / resolution
        for j in range(resolution + 1):
            phi = j * 2 * np.pi / resolution

            # Spherical to Cartesian coordinates
            # Map to our coordinate system: X=out, Y=right, Z=up
            x = radius * np.sin(theta) * np.cos(phi)
            y = radius * np.sin(theta) * np.sin(phi)
            z = radius * np.cos(theta)

            vertices.append([x, y, z])

            # Color based on z-coordinate (horizontal separation)
            colors.append(color_top if z > 0 else color_bottom)

    # Generate triangle indices
    indices = []
    for i in range(resolution):
        for j in range(resolution):
            first = i * (resolution + 1) + j
            second = first + resolution + 1
            indices.extend([first, second, first + 1, second, second + 1, first + 1])

    return (np.array(vertices, dtype=np.float32),
            np.array(colors, dtype=np.float32),
            np.array(indices, dtype=np.uint32))


def axis_angle_to_matrix(axis, angle):
    """
    Convert axis-angle representation to rotation matrix using Rodrigues' formula.

    Args:
        axis: Rotation axis [x, y, z] (will be normalized)
        angle: Rotation angle in radians

    Returns:
        3x3 rotation matrix
    """
    axis = np.asarray(axis, dtype=np.float64)
    axis_norm = np.linalg.norm(axis)

    if axis_norm < 1e-10:
        return np.identity(3)

    axis = axis / axis_norm

    # Rodrigues' rotation formula
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    # Cross-product matrix of the axis
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    # R = I + sin(θ)K + (1-cos(θ))K²
    R = np.identity(3) + sin_angle * K + (1 - cos_angle) * np.dot(K, K)
    return R


def update_rotation(rotation_matrix, dt, speed, weights):
    """
    Update rotation matrix by rotating around the combined axis.

    Args:
        rotation_matrix: Current 3x3 rotation matrix
        dt: Delta time in seconds
        speed: Rotation speed in degrees per second
        weights: Spin components [corkscrewspin, topspin_backspin, sidespin]
                 Maps to [X, Y, Z] axes in our coordinate system

    Returns:
        Updated 3x3 rotation matrix
    """
    total_weight = sum([abs(w) for w in weights])
    if total_weight < 1e-10:
        return rotation_matrix

    # Rotation axis from spin components
    # The weights directly define the rotation axis in our coordinate system
    axis = np.array(weights)

    # Calculate rotation angle for this time step (convert to radians)
    angle_increment = np.radians(speed * dt)

    # Get incremental rotation matrix
    R_increment = axis_angle_to_matrix(axis, angle_increment)

    # Apply incremental rotation: R_new = R_increment @ R_old
    return R_increment @ rotation_matrix
