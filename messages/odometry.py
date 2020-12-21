import copy
from dataclasses import dataclass, field

from .message import Message
from .header import Header
from .pose import Pose


@dataclass
class Odometry(Message):
    header: Header = field(default_factory=Header)
    pose: Pose = field(default_factory=Pose)

    def copy(self) -> 'Odometry':
        return copy.deepcopy(self)
