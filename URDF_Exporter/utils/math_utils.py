"""Mathematical utilities for 3D transformations and matrix operations.

This module provides the Transform class for handling 3D coordinate transformations
between different reference frames, along with supporting matrix operation functions.
It handles conversions between Fusion 360's coordinate system and URDF conventions.
"""

import math
from typing import Optional

# pyright: reportMissingImports=false
import adsk
import adsk.core


class Transform:
    """Represents a 3D rigid body transformation.

    This class encapsulates 3D transformations using both translation vectors
    and Euler angles (roll, pitch, yaw). It provides methods for transformation
    composition, inversion, and conversion between different representations.

    The class handles coordinate system conversions between Fusion 360 (which uses
    centimeters) and URDF (which uses meters), and supports creation from both
    Fusion 360 Matrix3D objects and standard 4x4 transformation matrices.

    Attributes:
        homogeneous: 4x4 homogeneous transformation matrix
        translation: Translation vector [x, y, z] in meters
        rotation: Euler angles [roll, pitch, yaw] in radians (XYZ convention)
    """

    homogeneous: list[list[float]]  # 4x4 matrix
    translation: list[float]  # [x, y, z]
    rotation: list[float]  # [roll, pitch, yaw] in radians

    def __init__(
        self,
        translation: Optional[list[float]] = None,
        rotation: Optional[list[float]] = None,
    ) -> None:
        """Initialize a Transform instance.

        Args:
            translation: Translation vector [x, y, z] in meters. Defaults to [0, 0, 0]
            rotation: Euler angles [roll, pitch, yaw] in radians. Defaults to [0, 0, 0]
        """
        self.translation = translation if translation else [0.0, 0.0, 0.0]
        self.rotation = rotation if rotation else [0.0, 0.0, 0.0]
        self.homogeneous = self.as_matrix()

    def as_matrix(self) -> list[list[float]]:
        """Convert transformation to 4x4 homogeneous matrix representation.

        Constructs the transformation matrix using the ZYX Euler angle convention
        (yaw-pitch-roll), which is standard for robotics applications.

        Returns:
            list[list[float]]: 4x4 homogeneous transformation matrix where:
                               - Upper-left 3x3 is the rotation matrix
                               - Right column is the translation vector
                               - Bottom row is [0, 0, 0, 1]

        Note:
            Uses the rotation sequence R = Rz(yaw) * Ry(pitch) * Rx(roll)
        """
        tx, ty, tz = self.translation
        roll, pitch, yaw = self.rotation
        # Rotation matrices around x, y, z
        Rx = [
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)],
        ]
        Ry = [
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ]
        Rz = [
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ]
        # R = Rz * Ry * Rx
        Rzy = mat_mult(Rz, Ry)
        R = mat_mult(Rzy, Rx)
        # Build 4x4 matrix
        mat = [
            [R[0][0], R[0][1], R[0][2], tx],
            [R[1][0], R[1][1], R[1][2], ty],
            [R[2][0], R[2][1], R[2][2], tz],
            [0, 0, 0, 1],
        ]
        return mat

    @property
    def xyz(self, digits: int = 6) -> list[float]:
        """Get translation vector rounded to specified precision.

        Args:
            digits: Number of decimal places for rounding

        Returns:
            list[float]: Translation vector [x, y, z] rounded to digits precision
        """
        return [round(el, digits) for el in self.translation]

    @property
    def rpy(self, digits: int = 6) -> list[float]:
        """Get Euler angles rounded to specified precision.

        Args:
            digits: Number of decimal places for rounding

        Returns:
            list[float]: Euler angles [roll, pitch, yaw] in radians rounded to digits precision
        """
        return [round(el, digits) for el in self.rotation]

    @property
    def rotation_matrix(self) -> list[list[float]]:
        """Extract 3x3 rotation matrix from the transformation.

        Returns:
            list[list[float]]: 3x3 rotation matrix representing the orientation
        """
        return [row[:3] for row in self.as_matrix()[:3]]

    @staticmethod
    def from_Matrix3D(matrix3d: adsk.core.Matrix3D) -> "Transform":
        """Create Transform from Fusion 360 Matrix3D object.

        Converts a Fusion 360 Matrix3D (which uses centimeters) to a Transform
        object (which uses meters). Extracts translation and rotation components
        and handles the unit conversion automatically.

        Args:
            matrix3d: Fusion 360 Matrix3D object representing a transformation

        Returns:
            Transform: New Transform instance with converted units and extracted components

        Note:
            - Converts translation from centimeters to meters (divides by 100)
            - Extracts Euler angles from rotation matrix using ZYX convention
            - Handles gimbal lock singularities in angle extraction
        """
        flatten_mat = list(matrix3d.asArray())

        # reshape to 4x4
        mat = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                mat[i][j] = flatten_mat[i * 4 + j]

        # Convert units from cm to m, specifically for matrix3d from Fusion 360
        tx = mat[0][3] / 100.0  # cm to m
        ty = mat[1][3] / 100.0  # cm to m
        tz = mat[2][3] / 100.0  # cm to m

        # Extract rotation matrix
        R = [
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ]

        # Extract Euler angles (roll, pitch, yaw)
        sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(R[2][1], R[2][2])
            pitch = math.atan2(-R[2][0], sy)
            yaw = math.atan2(R[1][0], R[0][0])
        else:
            roll = math.atan2(-R[1][2], R[1][1])
            pitch = math.atan2(-R[2][0], sy)
            yaw = 0
        return Transform([tx, ty, tz], [roll, pitch, yaw])

    @staticmethod
    def from_matrix(matrix: list[list[float]]) -> "Transform":
        """Create Transform from a 4x4 homogeneous transformation matrix.

        Decomposes a 4x4 transformation matrix into translation and rotation
        components, with rotation represented as Euler angles.

        Args:
            matrix: 4x4 homogeneous transformation matrix

        Returns:
            Transform: New Transform instance with extracted components

        Note:
            - Assumes matrix is in meters (no unit conversion)
            - Extracts Euler angles using ZYX convention
            - Handles gimbal lock singularities
        """
        mat = matrix

        tx = mat[0][3]
        ty = mat[1][3]
        tz = mat[2][3]

        # Extract rotation matrix
        R = [
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ]

        # Extract Euler angles (roll, pitch, yaw)
        sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(R[2][1], R[2][2])
            pitch = math.atan2(-R[2][0], sy)
            yaw = math.atan2(R[1][0], R[0][0])
        else:
            roll = math.atan2(-R[1][2], R[1][1])
            pitch = math.atan2(-R[2][0], sy)
            yaw = 0
        return Transform([tx, ty, tz], [roll, pitch, yaw])

    def __mul__(self, other: "Transform") -> "Transform":
        """Compose two transformations (self * other).

        Performs transformation composition where the result represents
        applying 'other' transformation first, then 'self'.

        Args:
            other: Transform to compose with this one

        Returns:
            Transform: Composed transformation equivalent to self @ other

        Note:
            Matrix multiplication order: result = self.matrix @ other.matrix
        """
        mat1 = self.as_matrix()
        mat2 = other.as_matrix()
        result = mat_mult(mat1, mat2)
        return Transform.from_matrix(result)

    def inverse(self) -> "Transform":
        """Compute the inverse transformation.

        Returns a new Transform that represents the inverse of this transformation.
        If this transform converts from frame A to frame B, the inverse converts
        from frame B to frame A.

        Returns:
            Transform: Inverse transformation

        Note:
            Uses efficient inverse computation for rigid body transformations:
            R_inv = R^T, t_inv = -R^T * t
        """
        inv_mat = mat_inverse_4x4(self.as_matrix())
        return Transform.from_matrix(inv_mat)


def mat_mult(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    """Multiply two matrices using standard matrix multiplication.

    Computes the matrix product C = A * B where C[i,j] = sum(A[i,k] * B[k,j])

    Args:
        A: Left matrix (m x n)
        B: Right matrix (n x p)

    Returns:
        list[list[float]]: Result matrix (m x p)

    Note:
        The number of columns in A must equal the number of rows in B.
    """
    result = [[0.0 for _ in range(len(B[0]))] for _ in range(len(A))]
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]
    return result


def mat_transpose(A: list[list[float]]) -> list[list[float]]:
    """Compute the transpose of a matrix.

    Args:
        A: Input matrix to transpose

    Returns:
        list[list[float]]: Transposed matrix where A^T[i,j] = A[j,i]
    """
    return [list(row) for row in zip(*A)]


def mat_identity(n: int) -> list[list[float]]:
    """Create an n x n identity matrix.

    Args:
        n: Size of the square identity matrix

    Returns:
        list[list[float]]: n x n identity matrix with 1s on diagonal, 0s elsewhere
    """
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def mat_inverse_4x4(T: list[list[float]]) -> list[list[float]]:
    """Compute inverse of a 4x4 homogeneous transformation matrix.

    Uses the special structure of rigid body transformations for efficient inversion:
    If T = [R t; 0 1], then T^-1 = [R^T -R^T*t; 0 1]

    Args:
        T: 4x4 homogeneous transformation matrix

    Returns:
        list[list[float]]: Inverse transformation matrix

    Note:
        Assumes T represents a valid rigid body transformation (orthogonal rotation matrix).
    """
    R = [row[:3] for row in T[:3]]
    t = [T[0][3], T[1][3], T[2][3]]
    Rt = mat_transpose(R)
    t_inv = [-sum(Rt[i][j] * t[j] for j in range(3)) for i in range(3)]
    inv = [Rt[0] + [t_inv[0]], Rt[1] + [t_inv[1]], Rt[2] + [t_inv[2]], [0, 0, 0, 1]]
    return inv
