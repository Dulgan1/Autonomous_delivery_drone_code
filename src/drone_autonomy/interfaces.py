"""Small interfaces for target input and vehicle action requests."""

from dataclasses import dataclass
from typing import Protocol

from .models import VisualTarget


class TargetProvider(Protocol):
    """Source of visual targets from the separate perception system."""

    def latest_target(self) -> VisualTarget | None:
        """Return the latest target, or None when there is no fresh target."""
        ...


@dataclass(frozen=True)
class VehicleTelemetry:
    """Small set of vehicle facts needed by the landing logic.

    Attributes:
        altitude_m: Height above the landing surface in metres. This value is
            supplied by a future vehicle adapter or by a mock in tests.
    """

    altitude_m: float = 0.0


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
