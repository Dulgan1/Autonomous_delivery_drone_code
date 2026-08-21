"""Run landing scenarios without a camera or drone."""

from dataclasses import dataclass

from .mocks import MockNavigationProvider, MockTargetProvider, MockVehicle
from .models import LandingState, NavigationReadings, VisualTarget
from .state_machine import LandingStateMachine


@dataclass(frozen=True)
class ScenarioStep:
    """One moment in a simulated flight.

    Attributes:
        time_s: Time of this step in seconds. Steps must be in time order.
        target: Fake CV target at this moment, or None when no target is seen.
        navigation: Fake GPS, heading, and ultrasonic data at this moment.
        altitude_m: Fake altitude given to the vehicle at this moment.
        manual_override: Whether the operator takes control at this moment.
        abort: Whether the operator stops the landing attempt at this moment.
        battery_percent: Fake battery level, or None to leave it unchanged.
        telemetry_fresh: Fake Pi-to-Pixhawk link health, or None to leave it.
        failsafe_active: Fake Pixhawk failsafe flag, or None to leave it.
        armed: Fake armed flag, or None to leave it unchanged.
    """

    time_s: float
    target: VisualTarget | None = None
    navigation: NavigationReadings | None = None
    altitude_m: float = 0.0
    manual_override: bool = False
    abort: bool = False
    battery_percent: float | None = None
    telemetry_fresh: bool | None = None
    failsafe_active: bool | None = None
    armed: bool | None = None


@dataclass(frozen=True)
class SimulationRecord:
    """What the system did at one scenario step.

    Attributes:
        time_s: Time of the completed scenario step.
        state: State after processing the step.
        transition_reason: Why the state changed, or None if it did not.
        actions: New vehicle requests made during this step.
    """

    time_s: float
    state: LandingState
    transition_reason: str | None
    actions: tuple[tuple[str, object], ...]


class SimulationRunner:
    """Feed scripted inputs to a state machine and keep a simple timeline."""

    def __init__(self, machine: LandingStateMachine, targets: MockTargetProvider, vehicle: MockVehicle, navigation: MockNavigationProvider | None = None):
        """Create a runner around one mock state machine.

        Args:
            machine: Landing state machine to exercise.
            targets: Mock target provider whose target is changed per step.
            vehicle: Mock vehicle whose altitude is changed per step.
            navigation: Optional mock GPS and ultrasonic source.
        """
        self.machine = machine
        self.targets = targets
        self.vehicle = vehicle
        self.navigation = navigation

    def run(self, steps: list[ScenarioStep]) -> list[SimulationRecord]:
        """Start autonomy, run steps in time order, and return a timeline.

        Args:
            steps: Fake flight moments to feed into the state machine.

        Returns:
            One record per supplied step. An empty list for no steps.

        Raises:
            RuntimeError: If the machine was already started.
        """
        if not steps:
            return []
        if self.machine.state != LandingState.IDLE:
            raise RuntimeError("simulation must start while the machine is IDLE")

        startup_command_count = len(self.vehicle.commands)
        startup_transition_count = len(self.machine.transitions)
        self.machine.start(steps[0].time_s)
        records: list[SimulationRecord] = []
        for index, step in enumerate(steps):
            self.targets.target = step.target
            if self.navigation is not None:
                self.navigation.readings = step.navigation
            self.vehicle.altitude_m = step.altitude_m
            if step.battery_percent is not None:
                self.vehicle.battery_remaining_percent = step.battery_percent
            if step.telemetry_fresh is not None:
                self.vehicle.telemetry_fresh = step.telemetry_fresh
            if step.failsafe_active is not None:
                self.vehicle.failsafe_active = step.failsafe_active
            if step.armed is not None:
                self.vehicle.armed = step.armed
            command_count = startup_command_count if index == 0 else len(self.vehicle.commands)
            transition_count = startup_transition_count if index == 0 else len(self.machine.transitions)
            state = self.machine.update(
                step.time_s,
                manual_override=step.manual_override,
                abort=step.abort,
            )
            reason = None
            if len(self.machine.transitions) > transition_count:
                reason = self.machine.transitions[-1][2]
            records.append(
                SimulationRecord(
                    time_s=step.time_s,
                    state=state,
                    transition_reason=reason,
                    actions=tuple(self.vehicle.commands[command_count:]),
                )
            )
        return records


def format_timeline(records: list[SimulationRecord]) -> str:
    """Return the simulation records as a simple plain-text table.

    Args:
        records: Records returned by :meth:`SimulationRunner.run`.

    Returns:
        A table with time, state, reason, and requested actions.
    """
    lines = ["time | state | reason | actions"]
    for record in records:
        actions = ", ".join(name for name, _ in record.actions) or "-"
        lines.append(
            f"{record.time_s:>4.1f} | {record.state.value} | "
            f"{record.transition_reason or '-'} | {actions}"
        )
    return "\n".join(lines)
