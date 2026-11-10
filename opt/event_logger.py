
from datetime import datetime
from enum import Enum


class EventCategory(Enum):
    COMMAND = "COMMAND"
    STATE_CHANGE = "STATE_CHANGE"
    NORMAL = "NORMAL"
    ERROR = "ERROR"
    SYSTEM = "SYSTEM"


class EventLogger:
    _message_number = 0

    def __init__(self, controller_name: str):
        self.controller_name = controller_name
        self._current_state = 'UNKNOWN'

    def set_state(self, state):
        """Update this controller's current state."""
        self._current_state = state

    def log(
        self,
        category: EventCategory,
        description: str,
        previous_state=None,
        next_state=None,
        reason=None
    ):
        EventLogger._message_number += 1

        timestamp = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )[:-3]

        message = (
            f"[{EventLogger._message_number:04d}] "
            f"[{timestamp}] "
            f"[{self.controller_name}] "
            f"[{self._current_state}] "
            f"{category.value}: "
            f"{description}"
        )

        if previous_state is not None:
            message += f" | {previous_state} -> {next_state}"

        if reason is not None:
            message += f" | Reason: {reason}"

        print(message, flush=True)
