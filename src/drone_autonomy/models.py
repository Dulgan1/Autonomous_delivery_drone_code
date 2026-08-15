"""Target data received from the separate CV project."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class LandingState(str, Enum):
    IDLE = "idle"
    TAKEOFF = "takeoff"
    SEARCH = "search"
    TARGET_ACQUIRED = "target_acquired"
    ALIGN = "align"
    DESCEND = "descend"
    TARGET_LOST = "target_lost"
    HOLD = "hold"
    ABORT = "abort"
    LAND = "land"


@dataclass(frozen=True)
class VisualTarget:
    """A target message from the perception system.

    The errors describe where the target is in the image, not its real-world
    position. A target must be both stable and visible before it is used.
    """

    track_id: int | str
    target_point: tuple[float, float]
    horizontal_error: float
    vertical_error: float
    normalized_radius: float
    marker_confidence: float
    stable: bool = True
    visible: bool = True

    @property
    def usable(self) -> bool:
        values = (
            *self.target_point,
            self.horizontal_error,
            self.vertical_error,
            self.normalized_radius,
            self.marker_confidence,
        )
        return (
            self.stable
            and self.visible
            and all(isfinite(value) for value in values)
            and -1.0 <= self.horizontal_error <= 1.0
            and -1.0 <= self.vertical_error <= 1.0
            and self.normalized_radius >= 0.0
            and 0.0 <= self.marker_confidence <= 1.0
        )
