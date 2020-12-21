import copy
from dataclasses import dataclass, field
from typing import List

from matplotlib.axes import Axes

from .message import Message
from .pose import Pose


@dataclass
class Path(Message):
    poses: List[Pose] = field(default_factory=list)

    def copy(self) -> 'Path':
        return copy.deepcopy(self)

    def plot(self, ax: Axes, step: int = None, line: bool = False, color: str = "purple"):
        poses = self.poses
        if isinstance(step, int) and step > 1:
            poses = poses[::step]
        if line:
            xs = [pose.position.x for pose in poses]
            ys = [pose.position.y for pose in poses]
            ax.plot(xs, ys, color=color)
        else:
            for pose in self.poses[::step]:
                pose.plot(ax)
