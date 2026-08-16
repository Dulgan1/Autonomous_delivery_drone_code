"""Target data received from the separate CV project."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class LandingState(str, Enum):
    """Possible stages of one landing attempt.

    Attributes:
        IDLE: Autonomy is not running.
        TAKEOFF: A takeoff request has been made.
        SEARCH: Waiting for a usable visual target.
        TARGET_ACQUIRED: Checking that a new target stays usable.
        ALIGN: Asking for small image-based corrections.
        DESCEND: Asking for a slow descent while the target remains usable.
        TARGET_LOST: Holding because the target disappeared or became invalid.
        HOLD: Holding because the operator took manual control.
        ABORT: Holding because a safety rule stopped the landing attempt.
        LAND: A land request has been made.
    """
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

    Attributes:
        track_id: ID supplied by the CV tracker.
        target_point: Target centre in image pixels as ``(x, y)``.
        horizontal_error: Target position from left (-1) to right (+1).
        vertical_error: Target position from top (-1) to bottom (+1).
        normalized_radius: Apparent target size in the image; not metres.
        marker_confidence: CV quality value from 0 to 1; not a probability.
        stable: True only after the CV tracker considers the target consistent.
        visible: True only when the target is seen in the current frame.
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
        """Return True when this is safe to use as a fresh visual measurement.

        The target must be stable, visible, finite, and within the documented
        normalized ranges. Old tracked targets and malformed data return False.
        """
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
