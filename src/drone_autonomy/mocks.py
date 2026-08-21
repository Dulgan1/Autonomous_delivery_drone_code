"""Test helpers. They record requests and never control a real vehicle."""

from dataclasses import dataclass, field

from .interfaces import VehicleTelemetry, VelocitySetpoint
from .models import NavigationReadings, VisualTarget


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
class MockNavigationProvider:
    """Simple navigation source whose ``readings`` field tests can change.

    Attributes:
        readings: Next navigation message, or None when no reading exists.
    """

    readings: NavigationReadings | None = None

    def latest_navigation(self) -> NavigationReadings | None:
        """Return the navigation readings currently stored by the test."""
        return self.readings


@dataclass
class MockVehicle:
    """Fake vehicle that records requests instead of sending them anywhere.

    Attributes:
        altitude_m: Altitude returned through :meth:`telemetry`.
        commands: Requested actions as ``(action_name, value)`` pairs.
    """

    altitude_m: float = 0.0
    telemetry_fresh: bool = True
    position_hold_ready: bool = True
    failsafe_active: bool = False
    battery_remaining_percent: float = 100.0
    payload_released: bool = False
    commands: list[tuple[str, object]] = field(default_factory=list)

    def telemetry(self) -> VehicleTelemetry:
        """Return mock telemetry using the current ``altitude_m`` value."""
        return VehicleTelemetry(
            altitude_m=self.altitude_m,
            telemetry_fresh=self.telemetry_fresh,
            position_hold_ready=self.position_hold_ready,
            failsafe_active=self.failsafe_active,
            battery_remaining_percent=self.battery_remaining_percent,
            payload_released=self.payload_released,
        )

    def takeoff(self, altitude_m: float) -> None:
        """Record a takeoff request."""
        self.commands.append(("takeoff", altitude_m))

    def yaw_to(self, heading_deg: float) -> None:
        """Record a yaw request in compass degrees."""
        self.commands.append(("yaw_to", heading_deg))

    def forward(self, speed_mps: float, duration_s: float) -> None:
        """Record a short forward request as ``(speed, duration)``."""
        self.commands.append(("forward", (speed_mps, duration_s)))

    def release_payload(self) -> None:
        """Record a payload-release request."""
        self.commands.append(("release_payload", None))

    def return_home(self, reason: str) -> None:
        """Record a return-to-home request and its reason."""
        self.commands.append(("return_home", reason))

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
