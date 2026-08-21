"""Target data received from the separate CV project."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite


class LandingState(str, Enum):
    """Possible stages of one landing attempt.

    Attributes:
        IDLE: Autonomy is not running.
        PREFLIGHT: Checking required navigation sensors before takeoff.
        TAKEOFF: A takeoff request has been made.
        GPS_NAVIGATE: Travelling toward the configured GPS target.
        YAW_SCAN: Turning to check other directions after a blocked path.
        SEARCH_MOVE: Travelling to one point in the marker-search grid.
        SEARCH: Waiting for a usable visual target.
        TARGET_ACQUIRED: Checking that a new target stays usable.
        ALIGN: Asking for small image-based corrections.
        DESCEND: Asking for a slow descent while the target remains usable.
        DROP_READY: Checking that a centred payload marker stays safe to use.
        DROP_PAYLOAD: Requesting release and waiting for release feedback.
        RETURN_HOME: Asking the flight controller to return home after mission or abort.
        TARGET_LOST: Holding because the target disappeared or became invalid.
        HOLD: Holding because the operator took manual control.
        ABORT: Holding because a safety rule stopped the landing attempt.
        LAND: A land request has been made.
    """
    IDLE = "idle"
    PREFLIGHT = "preflight"
    TAKEOFF = "takeoff"
    GPS_NAVIGATE = "gps_navigate"
    YAW_SCAN = "yaw_scan"
    SEARCH_MOVE = "search_move"
    SEARCH = "search"
    TARGET_ACQUIRED = "target_acquired"
    ALIGN = "align"
    DESCEND = "descend"
    DROP_READY = "drop_ready"
    DROP_PAYLOAD = "drop_payload"
    RETURN_HOME = "return_home"
    TARGET_LOST = "target_lost"
    HOLD = "hold"
    ABORT = "abort"
    LAND = "land"


@dataclass(frozen=True)
class GpsPosition:
    """A GPS location in decimal degrees.

    Attributes:
        latitude_deg: Latitude from -90 to +90 degrees.
        longitude_deg: Longitude from -180 to +180 degrees.
    """

    latitude_deg: float
    longitude_deg: float

    @property
    def usable(self) -> bool:
        """Return True when both GPS coordinates are finite and in range."""
        return (
            isfinite(self.latitude_deg)
            and isfinite(self.longitude_deg)
            and -90.0 <= self.latitude_deg <= 90.0
            and -180.0 <= self.longitude_deg <= 180.0
        )


@dataclass(frozen=True)
class NavigationReadings:
    """GPS, heading, and ultrasonic readings used for safe navigation.

    ``*_fresh`` means the value was received recently. A present value is still
    unsafe to use when its matching freshness flag is False.

    Attributes:
        position: Latest GPS position, or None when unavailable.
        heading_deg: Drone heading, where 0 is north and 90 is east.
        bottom_distance_m: Distance from the bottom sensor to ground.
        side_distance_m: Distance in the direction checked by the side sensor.
        top_distance_m: Distance from the top sensor to an overhead obstacle.
        gps_fresh: Whether ``position`` is recent enough to use.
        heading_fresh: Whether ``heading_deg`` is recent enough to use.
        bottom_fresh: Whether ``bottom_distance_m`` is recent enough to use.
        side_fresh: Whether ``side_distance_m`` is recent enough to use.
        top_fresh: Whether ``top_distance_m`` is recent enough to use.
    """

    position: GpsPosition | None = None
    heading_deg: float | None = None
    bottom_distance_m: float | None = None
    side_distance_m: float | None = None
    top_distance_m: float | None = None
    gps_fresh: bool = False
    heading_fresh: bool = False
    bottom_fresh: bool = False
    side_fresh: bool = False
    top_fresh: bool = False

    @property
    def navigation_usable(self) -> bool:
        """Return True when GPS position and heading are recent and valid."""
        return (
            self.gps_fresh
            and self.heading_fresh
            and self.position is not None
            and self.position.usable
            and self.heading_deg is not None
            and isfinite(self.heading_deg)
        )


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
