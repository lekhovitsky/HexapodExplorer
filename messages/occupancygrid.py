import copy
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from matplotlib.axes import Axes

from .message import Message
from .header import Header
from .pose import Pose


@dataclass
class OccupancyGrid(Message):
    header: Header = field(default_factory=Header)

    # map info
    resolution: Optional[float] = None
    width: Optional[int] = None
    height: Optional[int] = None
    origin: Pose = field(default_factory=Pose)

    # map data
    data: Optional[np.ndarray] = None

    def __post_init__(self):
        if self.data is not None:
            if not isinstance(self.data, np.ndarray):
                self.data = np.asarray(self.data)
            shape = (self.height, self.width)
            if self.data.shape != shape:
                self.data = self.data.reshape(shape)

    def copy(self) -> 'OccupancyGrid':
        return copy.deepcopy(self)

    def plot(self, ax: Axes):
        if self.data is None:
            return

        extent = (
            self.origin.position.x, self.origin.position.x + self.resolution * self.width,
            self.origin.position.y, self.origin.position.y + self.resolution * self.height
        )
        ax.imshow(
            self.data, cmap="Greys", interpolation='nearest',
            extent=extent, vmin=0, vmax=1, origin='lower'
        )

    def is_valid_coordinate(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def get(self, x: int, y: int) -> float:
        return self.data[y, x]

    def set(self, x: int, y: int, value: float):
        self.data[y, x] = value
