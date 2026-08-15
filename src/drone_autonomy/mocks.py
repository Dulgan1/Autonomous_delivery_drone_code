"""Test helpers. They record requests and never control a real vehicle."""

from dataclasses import dataclass, field

from .interfaces import VehicleTelemetry, VelocitySetpoint
from .models import VisualTarget


@dataclass
class MockTargetProvider:
    target: VisualTarget | None = None

    def latest_target(self) -> VisualTarget | None:
        return self.target


@dataclass
class MockVehicle:
    altitude_m: float = 0.0
    commands: list[tuple[str, object]] = field(default_factory=list)

    def telemetry(self) -> VehicleTelemetry:
        return VehicleTelemetry(self.altitude_m)

    def takeoff(self, altitude_m: float) -> None:
        self.commands.append(("takeoff", altitude_m))

    def hold(self, reason: str) -> None:
        self.commands.append(("hold", reason))

    def velocity(self, setpoint: VelocitySetpoint) -> None:
        self.commands.append(("velocity", setpoint))

    def descend(self, rate_mps: float) -> None:
        self.commands.append(("descend", rate_mps))

    def land(self) -> None:
        self.commands.append(("land", None))
