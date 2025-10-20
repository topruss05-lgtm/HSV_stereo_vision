"""
Stereo Camera Calibration Module

Provides classes and utilities for camera calibration, including:
- Intrinsic camera parameters (focal length, principal point, distortion)
- Extrinsic stereo parameters (rotation, translation between cameras)
- Serialization to/from JSON files
"""

import json
import numpy as np
from typing import Optional, Tuple, Dict
from pathlib import Path


class CameraIntrinsics:
    """
    Intrinsic camera parameters.

    Attributes:
        camera_matrix: 3x3 camera matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        dist_coeffs: Distortion coefficients [k1, k2, p1, p2, k3]
        image_size: Image resolution (width, height)
        camera_name: Identifier for the camera (e.g., 'left', 'right')
    """

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_size: Tuple[int, int],
        camera_name: str = "camera"
    ):
        """
        Initialize camera intrinsics.

        Args:
            camera_matrix: 3x3 camera matrix
            dist_coeffs: Distortion coefficients (5 elements)
            image_size: (width, height) in pixels
            camera_name: Camera identifier
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.image_size = image_size
        self.camera_name = camera_name

    @property
    def fx(self) -> float:
        """Focal length in x direction."""
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        """Focal length in y direction."""
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        """Principal point x coordinate."""
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        """Principal point y coordinate."""
        return float(self.camera_matrix[1, 2])

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization."""
        return {
            'camera_name': self.camera_name,
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'image_size': list(self.image_size)
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'CameraIntrinsics':
        """Create from dictionary."""
        return cls(
            camera_matrix=np.array(data['camera_matrix'], dtype=np.float64),
            dist_coeffs=np.array(data['dist_coeffs'], dtype=np.float64),
            image_size=tuple(data['image_size']),
            camera_name=data.get('camera_name', 'camera')
        )

    def __repr__(self) -> str:
        return (f"CameraIntrinsics(camera_name='{self.camera_name}', "
                f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx:.2f}, cy={self.cy:.2f}, "
                f"image_size={self.image_size})")


class StereoCalibration:
    """
    Stereo camera calibration parameters.

    Contains intrinsic parameters for both cameras and extrinsic parameters
    defining the spatial relationship between them.

    Attributes:
        left_camera: Intrinsic parameters for left camera
        right_camera: Intrinsic parameters for right camera
        R: Rotation matrix from left to right camera (3x3)
        T: Translation vector from left to right camera (3x1)
        E: Essential matrix (3x3)
        F: Fundamental matrix (3x3)
        R1, R2: Rectification transforms for left and right cameras
        P1, P2: Projection matrices in rectified coordinate system
        Q: Disparity-to-depth mapping matrix (4x4)
    """

    def __init__(
        self,
        left_camera: CameraIntrinsics,
        right_camera: CameraIntrinsics,
        R: np.ndarray,
        T: np.ndarray,
        E: Optional[np.ndarray] = None,
        F: Optional[np.ndarray] = None,
        R1: Optional[np.ndarray] = None,
        R2: Optional[np.ndarray] = None,
        P1: Optional[np.ndarray] = None,
        P2: Optional[np.ndarray] = None,
        Q: Optional[np.ndarray] = None
    ):
        """
        Initialize stereo calibration.

        Args:
            left_camera: Left camera intrinsics
            right_camera: Right camera intrinsics
            R: Rotation matrix (3x3)
            T: Translation vector (3x1)
            E: Essential matrix (optional)
            F: Fundamental matrix (optional)
            R1, R2: Rectification rotations (optional)
            P1, P2: Projection matrices (optional)
            Q: Disparity-to-depth matrix (optional)
        """
        self.left_camera = left_camera
        self.right_camera = right_camera
        self.R = R
        self.T = T
        self.E = E
        self.F = F
        self.R1 = R1
        self.R2 = R2
        self.P1 = P1
        self.P2 = P2
        self.Q = Q

    @property
    def baseline(self) -> float:
        """Baseline distance between cameras (in same units as T)."""
        return float(np.linalg.norm(self.T))

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization."""
        data = {
            'left_camera': self.left_camera.to_dict(),
            'right_camera': self.right_camera.to_dict(),
            'R': self.R.tolist(),
            'T': self.T.tolist()
        }

        # Add optional parameters if available
        if self.E is not None:
            data['E'] = self.E.tolist()
        if self.F is not None:
            data['F'] = self.F.tolist()
        if self.R1 is not None:
            data['R1'] = self.R1.tolist()
        if self.R2 is not None:
            data['R2'] = self.R2.tolist()
        if self.P1 is not None:
            data['P1'] = self.P1.tolist()
        if self.P2 is not None:
            data['P2'] = self.P2.tolist()
        if self.Q is not None:
            data['Q'] = self.Q.tolist()

        return data

    @classmethod
    def from_dict(cls, data: Dict) -> 'StereoCalibration':
        """Create from dictionary."""
        return cls(
            left_camera=CameraIntrinsics.from_dict(data['left_camera']),
            right_camera=CameraIntrinsics.from_dict(data['right_camera']),
            R=np.array(data['R'], dtype=np.float64),
            T=np.array(data['T'], dtype=np.float64),
            E=np.array(data['E'], dtype=np.float64) if 'E' in data else None,
            F=np.array(data['F'], dtype=np.float64) if 'F' in data else None,
            R1=np.array(data['R1'], dtype=np.float64) if 'R1' in data else None,
            R2=np.array(data['R2'], dtype=np.float64) if 'R2' in data else None,
            P1=np.array(data['P1'], dtype=np.float64) if 'P1' in data else None,
            P2=np.array(data['P2'], dtype=np.float64) if 'P2' in data else None,
            Q=np.array(data['Q'], dtype=np.float64) if 'Q' in data else None
        )

    def save(self, filepath: str):
        """
        Save calibration to JSON file.

        Args:
            filepath: Path to output JSON file
        """
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)

        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)

        print(f"Stereo calibration saved to {filepath}")

    @classmethod
    def load(cls, filepath: str) -> 'StereoCalibration':
        """
        Load calibration from JSON file.

        Args:
            filepath: Path to JSON file

        Returns:
            StereoCalibration object

        Raises:
            FileNotFoundError: If file does not exist
        """
        filepath = Path(filepath)

        if not filepath.exists():
            raise FileNotFoundError(f"Calibration file not found: {filepath}")

        with open(filepath, 'r') as f:
            data = json.load(f)

        return cls.from_dict(data)

    def __repr__(self) -> str:
        return (f"StereoCalibration(baseline={self.baseline:.2f}, "
                f"left={self.left_camera.camera_name}, "
                f"right={self.right_camera.camera_name})")


def save_camera_intrinsics(intrinsics: CameraIntrinsics, filepath: str):
    """
    Save single camera intrinsics to JSON file.

    Args:
        intrinsics: CameraIntrinsics object
        filepath: Path to output JSON file
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    with open(filepath, 'w') as f:
        json.dump(intrinsics.to_dict(), f, indent=2)

    print(f"Camera intrinsics saved to {filepath}")


def load_camera_intrinsics(filepath: str) -> CameraIntrinsics:
    """
    Load single camera intrinsics from JSON file.

    Args:
        filepath: Path to JSON file

    Returns:
        CameraIntrinsics object

    Raises:
        FileNotFoundError: If file does not exist
    """
    filepath = Path(filepath)

    if not filepath.exists():
        raise FileNotFoundError(f"Intrinsics file not found: {filepath}")

    with open(filepath, 'r') as f:
        data = json.load(f)

    return CameraIntrinsics.from_dict(data)
