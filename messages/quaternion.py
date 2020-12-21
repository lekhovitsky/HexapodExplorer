from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .message import Message


@dataclass(repr=False)
class Quaternion(Message):
    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 1

    def __repr__(self) -> str:
        return f"q({self.w:.2f}, {self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

    def to_R(self) -> np.ndarray:
        """Convert quaternion into rotation matrix."""
        q = [self.w, self.x, self.y, self.z]
        R = [[q[0]**2+q[1]**2-q[2]**2-q[3]**2,     2*(q[1]*q[2]-q[0]*q[3]),      2*(q[1]*q[3]+q[0]*q[2])],
             [2*(q[1]*q[2]+q[0]*q[3]),     q[0]**2-q[1]**2+q[2]**2-q[3]**2,      2*(q[2]*q[3]-q[0]*q[1])],
             [2*(q[1]*q[3]-q[0]*q[2]),         2*(q[2]*q[3]+q[0]*q[1]),   q[0]**2-q[1]**2-q[2]**2+q[3]**2]]
        return np.asarray(R)

    def from_R(self, R: np.ndarray):
        """Convert rotation matrix into quaternion."""
        q = np.array([
            R.trace() + 1,
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ])
        # isn't the first normalization redundant?
        q /= 2.0 * np.sqrt(R.trace() + 1.0000001)
        q /= np.linalg.norm(q)
        self.w, self.x, self.y, self.z = q

    def to_Euler(self) -> Tuple[float, float ,float]:
        """Convert quaternion to (yaw, pitch, roll)."""
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = np.arctan2(t3, t4)
        return yaw, pitch, roll

    def from_Euler(self, yaw: float, pitch: float, roll: float):
        """Convert (yaw, pitch, roll) to quaternion."""
        q = np.zeros(4)
        q[0] = (np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
                + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2))
        q[1] = (np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
                - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2))
        q[2] = (np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
                + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2))
        q[3] = (np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
                - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2))
        q /= np.linalg.norm(q)
        self.w, self.x, self.y, self.z = q
