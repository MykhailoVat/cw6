
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
    _current_state = 'UNKNOWN'

    @classmethod
    def set_state(cls, state):
        """Update current robot state."""
        cls._current_state = state

    @classmethod
    def log(
        cls,
        category: EventCategory,
        description: str,
        previous_state=None,
        next_state=None,
        reason=None
    ):
        cls._message_number += 1

        timestamp = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )[:-3]

        message = (
            f"[{cls._message_number:04d}] "
            f"[{timestamp}] "
            f"[{cls._current_state}] "
            f"{category.value}: "
            f"{description}"
        )

        if previous_state is not None:
            message += f" | {previous_state} -> {next_state}"

        if reason is not None:
            message += f" | Reason: {reason}"

        print(message, flush=True)
