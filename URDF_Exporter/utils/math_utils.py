import math


class Transform:
    def __init__(self, translation=None, rotation=None):
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

    @staticmethod
    def from_matrix(matrix):
        """Create Transform from 4x4 matrix."""
        tx = matrix[0][3]
        ty = matrix[1][3]
        tz = matrix[2][3]
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
