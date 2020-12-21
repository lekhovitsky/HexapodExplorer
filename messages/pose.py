import copy
from dataclasses import dataclass, field

import numpy as np
from matplotlib.axes import Axes

from .message import Message
from .vector3 import Vector3
from .quaternion import Quaternion


@dataclass
class Pose(Message):
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)

    def __repr__(self) -> str:
        return f"Pose({self.position}, {self.orientation})"

    def copy(self) -> 'Pose':
        return copy.deepcopy(self)

    def dist(self, other: 'Pose') -> float:
        return self.position.dist(other.position)

    def plot(self, ax: Axes):
        dx = np.dot(self.orientation.to_R(), np.asarray([1, 0, 0]))
        dy = np.dot(self.orientation.to_R(), np.asarray([0, 1, 0]))
        ax.quiver([self.position.x], [self.position.y], [dx[0]], [dx[1]], color='r')
        ax.quiver([self.position.x], [self.position.y], [dy[0]], [dy[1]], color='g')
