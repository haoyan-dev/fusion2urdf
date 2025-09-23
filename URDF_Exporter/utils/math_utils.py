import math
import adsk
import adsk.core
import adsk.fusion
from typing import Optional, Union


class Transform:
    homogeneous: list[list[float]]  # 4x4 matrix
    translation: list[float]  # [x, y, z]
    rotation: list[float]  # [roll, pitch, yaw] in radians

    def __init__(
        self,
        translation: Optional[list[float]] = None,
        rotation: Optional[list[float]] = None,
    ):
        # translation: [x, y, z]
        # rotation: [roll, pitch, yaw] in radians
        self.translation = translation if translation else [0.0, 0.0, 0.0]
        self.rotation = rotation if rotation else [0.0, 0.0, 0.0]
        self.homogeneous = self.as_matrix()

    def as_matrix(self):
        """Return 4x4 homogeneous transformation matrix."""
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
        """Return translation vector [x, y, z]."""
        return [round(el, digits) for el in self.translation]

    @property
    def rpy(self, digits: int = 6) -> list[float]:
        """Return rotation vector [roll, pitch, yaw]."""
        return [round(el, digits) for el in self.rotation]

    @property
    def rotation_matrix(self):
        """Return 3x3 rotation matrix."""
        return [row[:3] for row in self.as_matrix()[:3]]

    @staticmethod
    def from_Matrix3D(matrix3d: adsk.core.Matrix3D) -> "Transform":
        """Create Transform from a 4x4 matrix."""
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
        """Create Transform from a 4x4 matrix."""
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
        """Compose two transforms."""
        mat1 = self.as_matrix()
        mat2 = other.as_matrix()
        result = mat_mult(mat1, mat2)
        return Transform.from_matrix(result)

    def inverse(self):
        """Return the inverse transform."""
        inv_mat = mat_inverse_4x4(self.as_matrix())
        return Transform.from_matrix(inv_mat)


def mat_mult(A, B) -> list[list[float]]:
    """Multiply two matrices."""
    result = [[0.0 for _ in range(len(B[0]))] for _ in range(len(A))]
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]
    return result


def mat_transpose(A) -> list[list[float]]:
    """Transpose a matrix."""
    return [list(row) for row in zip(*A)]


def mat_identity(n) -> list[list[float]]:
    """Return n x n identity matrix."""
    return [[1 if i == j else 0 for j in range(n)] for i in range(n)]


def mat_inverse_4x4(T) -> list[list[float]]:
    """Inverse of a 4x4 homogeneous transformation matrix."""
    R = [row[:3] for row in T[:3]]
    t = [T[0][3], T[1][3], T[2][3]]
    Rt = mat_transpose(R)
    t_inv = [-sum(Rt[i][j] * t[j] for j in range(3)) for i in range(3)]
    inv = [Rt[0] + [t_inv[0]], Rt[1] + [t_inv[1]], Rt[2] + [t_inv[2]], [0, 0, 0, 1]]
    return inv
