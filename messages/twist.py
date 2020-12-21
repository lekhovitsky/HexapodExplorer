from dataclasses import dataclass, field

from .message import Message
from .vector3 import Vector3


@dataclass
class Twist(Message):
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)
