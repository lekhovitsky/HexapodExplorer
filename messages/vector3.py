from dataclasses import dataclass

import numpy as np

from .message import Message


@dataclass(repr=False)
class Vector3(Message):
    x: float = 0
    y: float = 0
    z: float = 0

    def __repr__(self) -> str:
        return f"v({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

    def __add__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)

    def __matmul__(self, other: 'Vector3') -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def norm(self) -> float:
        x, y, z = self.x, self.y, self.z
        return np.sqrt(x * x + y * y + z * z)

    def dist(self, other: 'Vector3') -> float:
        return (self - other).norm()
