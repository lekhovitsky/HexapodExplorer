from dataclasses import dataclass, field
from typing import List

import numpy as np
from matplotlib.axes import Axes

from .message import Message
from .header import Header


@dataclass
class LaserScan(Message):
    header: Header = field(default_factory=Header)

    angle_min: float = 0
    angle_max: float = 0
    angle_increment: float = 0

    range_min: float = 0
    range_max: float = 0

    distances: List[float] = field(default_factory=list)

    def plot(self, ax: Axes):
        scan_x, scan_y = [], []
        for idx, pt in enumerate(self.distances):
            scan_x.append(pt * np.cos((self.angle_min + idx*self.angle_increment)))
            scan_y.append(pt * np.sin((self.angle_min + idx*self.angle_increment)))

        ax.scatter(scan_x, scan_y)
