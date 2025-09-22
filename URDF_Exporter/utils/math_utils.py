import math
from typing import Optional, Union


class Transform:
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
    def rotation_matrix(self):
        """Return 3x3 rotation matrix."""
        return [row[:3] for row in self.as_matrix()[:3]]

    @staticmethod
    def from_matrix(matrix) -> "Transform":
        """Create Transform from a 4x4 matrix."""
        matrix = validate_matrix_4x4(list(matrix))

        tx = matrix[0][3] / 100.0  # cm to m
        ty = matrix[1][3] / 100.0  # cm to m
        tz = matrix[2][3] / 100.0  # cm to m

        # Extract rotation matrix
        R = [
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]],
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

    def __mul__(self, other):
        """Compose two transforms."""
        mat1 = self.as_matrix()
        mat2 = other.as_matrix()
        result = mat_mult(mat1, mat2)
        return Transform.from_matrix(result)

    def inverse(self):
        """Return the inverse transform."""
        inv_mat = mat_inverse_4x4(self.as_matrix())
        return Transform.from_matrix(inv_mat)

def validate_matrix_4x4(matrix: Union[list[list[float]], list[float]]) -> list[list[float]]:
    """Validate and convert input to a 4x4 matrix."""
    if isinstance(matrix, list) and all(isinstance(row, list) for row in matrix):
        if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise ValueError("Matrix must be 4x4.")
        return matrix
    elif isinstance(matrix, list) and len(matrix) == 16:
        return [
            matrix[0:4],
            matrix[4:8],
            matrix[8:12],
            matrix[12:16],
        ]
    else:
        raise ValueError("Input must be a 4x4 matrix or a flat list of 16 elements.")


def mat_mult(A, B):
    """Multiply two matrices."""
    result = [[0 for _ in range(len(B[0]))] for _ in range(len(A))]
    for i in range(len(A)):
        for j in range(len(B[0])):
            for k in range(len(B)):
                result[i][j] += A[i][k] * B[k][j]
    return result


def mat_transpose(A):
    """Transpose a matrix."""
    return [list(row) for row in zip(*A)]


def mat_identity(n):
    """Return n x n identity matrix."""
    return [[1 if i == j else 0 for j in range(n)] for i in range(n)]


def mat_inverse_4x4(T):
    """Inverse of a 4x4 homogeneous transformation matrix."""
    R = [row[:3] for row in T[:3]]
    t = [T[0][3], T[1][3], T[2][3]]
    Rt = mat_transpose(R)
    t_inv = [-sum(Rt[i][j] * t[j] for j in range(3)) for i in range(3)]
    inv = [Rt[0] + [t_inv[0]], Rt[1] + [t_inv[1]], Rt[2] + [t_inv[2]], [0, 0, 0, 1]]
    return inv
