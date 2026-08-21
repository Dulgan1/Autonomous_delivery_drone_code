"""Small interfaces for target input and vehicle action requests."""

from dataclasses import dataclass
from typing import Protocol

from .models import NavigationReadings, VisualTarget


class TargetProvider(Protocol):
    """Source of visual targets from the separate perception system."""

    def latest_target(self) -> VisualTarget | None:
        """Return the latest target, or None when there is no fresh target."""
        ...


class NavigationProvider(Protocol):
    """Source of GPS, heading, and ultrasonic readings."""

    def latest_navigation(self) -> NavigationReadings | None:
        """Return the newest navigation readings, or None when unavailable."""
        ...


@dataclass(frozen=True)
class VehicleTelemetry:
    """Small set of vehicle facts needed by the landing logic.

    Attributes:
        altitude_m: Height above the landing surface in metres. This value is
            supplied by a vehicle adapter or by a mock in tests.
        telemetry_fresh: True when Pixhawk telemetry is recent.
        position_hold_ready: True when Pixhawk can safely hold position.
        failsafe_active: True when Pixhawk has raised a failsafe.
        battery_remaining_percent: Battery percentage reported by Pixhawk.
        payload_released: True only when a future payload sensor confirms release.
        armed: True when the flight controller reports the vehicle is armed. A
            real adapter must report the truth; the permissive default exists
            only so mock-based tests stay short.
        flight_mode: Flight-controller mode name, for logging and diagnostics.
    """

    altitude_m: float = 0.0
    telemetry_fresh: bool = True
    position_hold_ready: bool = True
    failsafe_active: bool = False
    battery_remaining_percent: float = 100.0
    payload_released: bool = False
    armed: bool = True
    flight_mode: str = "unknown"


@dataclass(frozen=True)
class VelocitySetpoint:
    """Small image-based movement request for tests, not a motor command.

    Attributes:
        image_x: Requested correction based on left/right image error.
        image_y: Requested correction based on up/down image error.

    These are not metres per second and are not drone body-frame directions.
    """

    image_x: float
    image_y: float


class VehicleInterface(Protocol):
    """High-level vehicle requests used by the landing state machine.

    A real adapter may implement this later. It must check flight mode,
    telemetry, calibration, and its own failsafes before acting on a request.
    """

    def telemetry(self) -> VehicleTelemetry:
        """Return the latest vehicle facts needed for safety checks."""
        ...

    def takeoff(self, altitude_m: float) -> None:
        """Request takeoff to ``altitude_m`` metres; this is not a motor command."""
        ...

    def yaw_to(self, heading_deg: float) -> None:
        """Request a turn to a compass heading in degrees; not a motor command."""
        ...

    def forward(self, speed_mps: float, duration_s: float) -> None:
        """Request one short forward step after the path was checked clear."""
        ...

    def release_payload(self) -> None:
        """Request payload release; a future adapter must require its own interlock."""
        ...

    def return_home(self, reason: str) -> None:
        """Request the flight controller's configured return-to-home action."""
        ...

    def hold(self, reason: str) -> None:
        """Request the vehicle to hold position and record why it was requested."""
        ...

    def velocity(self, setpoint: VelocitySetpoint) -> None:
        """Request a bounded image-guidance correction for simulation or testing."""
        ...

    def descend(self, rate_mps: float) -> None:
        """Request a slow downward speed in metres per second."""
        ...

    def land(self) -> None:
        """Request the flight controller's landing behaviour."""
        ...
