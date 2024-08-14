# pyre-unsafe
from typing import Tuple

import cv2
import numpy as np
from opensfm import transformations


def rotation_from_angle_axis(angle_axis: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(angle_axis))[0]


def rotation_from_ptr(pan: float, tilt: float, roll: float) -> np.ndarray:
    """World-to-camera rotation matrix from pan, tilt and roll."""
    R1 = rotation_from_angle_axis(np.array([0.0, 0.0, roll]))
    R2 = rotation_from_angle_axis(np.array([tilt + np.pi / 2, 0.0, 0.0]))
    R3 = rotation_from_angle_axis(np.array([0.0, 0.0, pan]))
    return R1.dot(R2).dot(R3)


def ptr_from_rotation(
    rotation_matrix: np.ndarray,
) -> Tuple[float, float, float]:
    """Pan tilt and roll from camera rotation matrix"""
    pan = pan_from_rotation(rotation_matrix)
    tilt = tilt_from_rotation(rotation_matrix)
    roll = roll_from_rotation(rotation_matrix)
    return pan, tilt, roll


def pan_from_rotation(rotation_matrix: np.ndarray) -> float:
    Rt_ez = np.dot(rotation_matrix.T, [0, 0, 1])
    return np.arctan2(Rt_ez[0], Rt_ez[1])


def tilt_from_rotation(rotation_matrix: np.ndarray) -> float:
    Rt_ez = np.dot(rotation_matrix.T, [0, 0, 1])
    l = np.linalg.norm(Rt_ez[:2])
    return np.arctan2(-Rt_ez[2], l)


def roll_from_rotation(rotation_matrix: np.ndarray) -> float:
    Rt_ex = np.dot(rotation_matrix.T, [1, 0, 0])
    Rt_ez = np.dot(rotation_matrix.T, [0, 0, 1])
    a = np.cross(Rt_ez, [0, 0, 1])
    a /= np.linalg.norm(a)
    b = np.cross(Rt_ex, a)
    return np.arcsin(np.dot(Rt_ez, b))


def rotation_from_ptr_v2(pan: float, tilt: float, roll: float) -> np.ndarray:
    """Camera rotation matrix from pan, tilt and roll.

    This is the implementation used in the Single Image Calibration code.
    """
    tilt += np.pi / 2
    return transformations.euler_matrix(pan, tilt, roll, "szxz")[:3, :3]


def ptr_from_rotation_v2(rotation_matrix: np.ndarray) -> Tuple[float, float, float]:
    """Pan tilt and roll from camera rotation matrix.

    This is the implementation used in the Single Image Calibration code.
    """
    T = np.identity(4)
    T[:3, :3] = rotation_matrix
    pan, tilt, roll = transformations.euler_from_matrix(T, "szxz")
    return pan, tilt - np.pi / 2, roll


def rotation_from_opk(omega: float, phi: float, kappa: float) -> np.ndarray:
    """World-to-camera rotation matrix from pan, tilt and roll."""

    # Omega (ω), the rotation around the Χ axis. (East)
    # Phi (φ), the rotation around the Y axis. (North)
    # Kappa (κ), the rotation around the Z axis. (UP)
    Rw = rotation_from_angle_axis(np.array([-omega, 0.0, 0.0]))
    Rp = rotation_from_angle_axis(np.array([0.0, -phi, 0.0]))
    Rk = rotation_from_angle_axis(np.array([0.0, 0.0, -kappa]))

    # OpenSfM
    # The z-axis points forward
    # The y-axis points down
    # The x-axis points to the right
    Rc = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    return Rc.dot(Rk).dot(Rp).dot(Rw)


def opk_from_rotation(
    rotation_matrix: np.ndarray,
) -> Tuple[float, float, float]:
    """Omega, phi, kappa from camera rotation matrix"""
    Rc = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    R = rotation_matrix.T.dot(Rc)
    omega = omega_from_rotation(R)
    phi = phi_from_rotation(R)
    kappa = kappa_from_rotation(R)
    return omega, phi, kappa


def omega_from_rotation(rotation_matrix: np.ndarray) -> float:
    return np.arctan2(-rotation_matrix[1, 2], rotation_matrix[2, 2])


def phi_from_rotation(rotation_matrix: np.ndarray) -> float:
    return np.arcsin(rotation_matrix[0, 2])


def kappa_from_rotation(rotation_matrix: np.ndarray) -> float:
    return np.arctan2(-rotation_matrix[0, 1], rotation_matrix[0, 0])
