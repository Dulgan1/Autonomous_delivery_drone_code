"""Test helpers. They record requests and never control a real vehicle."""

from dataclasses import dataclass, field

from .interfaces import VehicleTelemetry, VelocitySetpoint
from .models import VisualTarget


@dataclass
class MockTargetProvider:
    """Simple target source whose ``target`` field tests can change.

    Attributes:
        target: Next target returned by :meth:`latest_target`, or None.
    """

    target: VisualTarget | None = None

    def latest_target(self) -> VisualTarget | None:
        """Return the target currently stored by the test or simulation."""
        return self.target


@dataclass
class MockVehicle:
    """Fake vehicle that records requests instead of sending them anywhere.

    Attributes:
        altitude_m: Altitude returned through :meth:`telemetry`.
        commands: Requested actions as ``(action_name, value)`` pairs.
    """

    altitude_m: float = 0.0
    commands: list[tuple[str, object]] = field(default_factory=list)

    def telemetry(self) -> VehicleTelemetry:
        """Return mock telemetry using the current ``altitude_m`` value."""
        return VehicleTelemetry(self.altitude_m)

    def takeoff(self, altitude_m: float) -> None:
        """Record a takeoff request."""
        self.commands.append(("takeoff", altitude_m))

    def hold(self, reason: str) -> None:
        """Record a hold request and its reason."""
        self.commands.append(("hold", reason))

    def velocity(self, setpoint: VelocitySetpoint) -> None:
        """Record an image-guidance velocity request."""
        self.commands.append(("velocity", setpoint))

    def descend(self, rate_mps: float) -> None:
        """Record a descent-rate request."""
        self.commands.append(("descend", rate_mps))

    def land(self) -> None:
        """Record a land request."""
        self.commands.append(("land", None))
