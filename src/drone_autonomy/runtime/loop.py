"""The real-time mission loop shared by simulation mode and hardware mode.

Both run modes build the same :class:`~drone_autonomy.state_machine.LandingStateMachine`
and hand it to this loop. Only the providers behind it differ: mocks on a
laptop, real adapters on the Raspberry Pi.
"""

import json
import signal
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

from ..models import LandingState
from ..state_machine import LandingStateMachine


MISSION_ENDED_STATES: frozenset[LandingState] = frozenset({
    LandingState.RETURN_HOME,
    LandingState.LAND,
    LandingState.ABORT,
})
"""States after which the flight controller, not this code, owns the aircraft."""


@dataclass(frozen=True)
class LoopRecord:
    """One decision cycle, as written to the mission log.

    Attributes:
        time_s: Loop clock reading for this cycle.
        state: State after the cycle.
        transition_reason: Why the state changed, or None if it did not.
        actions: Vehicle requests made during this cycle.
        manual_override: Whether the operator held control this cycle.
    """

    time_s: float
    state: LandingState
    transition_reason: str | None
    actions: tuple[str, ...]
    manual_override: bool


class MissionLog:
    """Append-only JSON-lines log of every decision cycle.

    A file log is the only way to review what the autonomy decided after a real
    flight, so hardware mode should always write one.
    """

    def __init__(self, path: Path | None):
        """Open the log file, creating parent directories as needed.

        Args:
            path: File to append to, or None to keep the log in memory only.
        """
        self.path = path
        self.records: list[LoopRecord] = []
        self._handle: TextIO | None = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = path.open("a", encoding="utf-8")

    def write(self, record: LoopRecord, extra: dict[str, Any] | None = None) -> None:
        """Record one cycle and flush it to disk immediately.

        Args:
            record: The completed cycle.
            extra: Additional diagnostic fields, such as telemetry values.
        """
        self.records.append(record)
        if self._handle is None:
            return
        payload = {
            "time_s": round(record.time_s, 3),
            "state": record.state.value,
            "reason": record.transition_reason,
            "actions": list(record.actions),
            "manual_override": record.manual_override,
            **(extra or {}),
        }
        self._handle.write(json.dumps(payload) + "\n")
        self._handle.flush()

    def close(self) -> None:
        """Close the log file if one is open."""
        if self._handle is not None:
            self._handle.close()
            self._handle = None


class MissionRunner:
    """Drive one state machine at a fixed rate until the mission ends.

    The loop never sleeps through a decision: if a cycle overruns its budget,
    the next cycle starts immediately rather than drifting further behind.
    """

    def __init__(
        self,
        machine: LandingStateMachine,
        *,
        rate_hz: float = 10.0,
        log: MissionLog | None = None,
        override_source: Callable[[], bool] | None = None,
        diagnostics: Callable[[], dict[str, Any]] | None = None,
        terminal_linger_s: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        on_record: Callable[[LoopRecord], None] | None = None,
    ):
        """Create a mission loop.

        Args:
            machine: The mission state machine to run.
            rate_hz: Decision cycles per second.
            log: Where cycles are recorded, or None for an in-memory log.
            override_source: Returns True while the operator holds control.
            diagnostics: Extra values to store with each logged cycle.
            terminal_linger_s: How long to keep looping after the mission ends,
                so the vehicle keeps receiving valid requests during handover.
            clock: Monotonic time source; replaced in tests.
            sleeper: Sleep function; replaced in tests.
            on_record: Called with every completed cycle, for live printing.
        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.machine = machine
        self.period_s = 1.0 / rate_hz
        self.log = log if log is not None else MissionLog(None)
        self.override_source = override_source or (lambda: False)
        self.diagnostics = diagnostics
        self.terminal_linger_s = terminal_linger_s
        self.clock = clock
        self.sleeper = sleeper
        self.on_record = on_record
        self._stop_requested = False

    def request_stop(self) -> None:
        """Ask the loop to finish after the current cycle."""
        self._stop_requested = True

    def install_signal_handlers(self) -> None:
        """Make Ctrl-C and SIGTERM end the loop cleanly instead of killing it.

        An abrupt exit would stop the Offboard setpoint stream without warning.
        Stopping cleanly lets the vehicle adapter hand control back on purpose.
        """
        for received in (signal.SIGINT, signal.SIGTERM):
            signal.signal(received, lambda *_: self.request_stop())

    def run(self, max_cycles: int | None = None) -> LandingState:
        """Start the mission and loop until it ends or a stop is requested.

        Args:
            max_cycles: Optional hard cap on cycles, used by tests.

        Returns:
            The state the mission finished in.
        """
        started_at = self.clock()
        self.machine.start(started_at)
        ended_at: float | None = None
        cycles = 0
        command_count = 0
        transition_count = len(self.machine.transitions)

        while not self._stop_requested:
            if max_cycles is not None and cycles >= max_cycles:
                break
            cycle_started = self.clock()
            override = bool(self.override_source())
            state = self.machine.update(cycle_started, manual_override=override)

            reason = None
            if len(self.machine.transitions) > transition_count:
                reason = self.machine.transitions[-1][2]
                transition_count = len(self.machine.transitions)
            actions, command_count = self._new_actions(command_count)
            record = LoopRecord(
                time_s=cycle_started - started_at,
                state=state,
                transition_reason=reason,
                actions=actions,
                manual_override=override,
            )
            self.log.write(record, self.diagnostics() if self.diagnostics else None)
            if self.on_record is not None:
                self.on_record(record)

            cycles += 1
            if state in MISSION_ENDED_STATES:
                ended_at = ended_at if ended_at is not None else cycle_started
                if cycle_started - ended_at >= self.terminal_linger_s:
                    break
            else:
                ended_at = None

            remaining = self.period_s - (self.clock() - cycle_started)
            if remaining > 0:
                self.sleeper(remaining)
        return self.machine.state

    def _new_actions(self, previous_count: int) -> tuple[tuple[str, ...], int]:
        """Return the vehicle request names made since the last cycle.

        Args:
            previous_count: Number of recorded requests before this cycle.

        Returns:
            The new request names and the updated total count. Vehicles that do
            not record their requests, such as the real Pixhawk adapter, report
            no names here; their requests appear in the flight-controller log.
        """
        commands = getattr(self.machine.vehicle, "commands", None)
        if not isinstance(commands, list):
            return (), previous_count
        return tuple(name for name, _ in commands[previous_count:]), len(commands)
