import time
from dataclasses import dataclass, field

from .message import Message


@dataclass
class Header(Message):
    frame_id: str = "base_frame"
    timestamp: float = field(default_factory=lambda: time.time())
