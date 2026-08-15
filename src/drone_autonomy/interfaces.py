"""Small interfaces for target input and vehicle action requests."""

from dataclasses import dataclass
from typing import Protocol

from .models import VisualTarget


class TargetProvider(Protocol):
    """Return the latest target, or None when there is no new target."""

    def latest_target(self) -> VisualTarget | None: ...


@dataclass(frozen=True)
class VehicleTelemetry:
    altitude_m: float = 0.0


@dataclass(frozen=True)
class VelocitySetpoint:
    """A small image-based movement request for tests, not a motor command."""

    image_x: float
    image_y: float


class VehicleInterface(Protocol):
    def telemetry(self) -> VehicleTelemetry: ...
    def takeoff(self, altitude_m: float) -> None: ...
    def hold(self, reason: str) -> None: ...
    def velocity(self, setpoint: VelocitySetpoint) -> None: ...
    def descend(self, rate_mps: float) -> None: ...
    def land(self) -> None: ...
