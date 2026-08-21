"""Hardware mode: run the mission on a Raspberry Pi with real sensors.

This module wires the real adapters together and hands them to the same mission
loop simulation mode uses. Hardware libraries are imported inside the functions
below, so this file can be read, imported, and type-checked on a laptop that has
neither pymavlink nor pigpio installed.

There are two ways to start it:

* ``check`` connects to Pixhawk, reads telemetry and sensors, and prints them.
  It never commands anything. This is the mode to use on the bench and with
  propellers removed.
* ``fly`` runs the real mission loop.
"""

import time
from contextlib import ExitStack
from typing import Any

from ..models import LandingState
from ..state_machine import LandingStateMachine
from .config import RuntimeConfig
from .loop import MissionLog, MissionRunner


def _log_path(config: RuntimeConfig) -> Any:
    """Return a timestamped mission-log path, or None when logging is off.

    Args:
        config: Runtime configuration holding the log directory.

    Returns:
        A :class:`pathlib.Path` for this run's log file, or None.
    """
    if config.log_dir is None:
        return None
    from pathlib import Path

    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path(config.log_dir) / f"mission-{stamp}.jsonl"


def _build_stack(config: RuntimeConfig, stack: ExitStack, *, arm_payload: bool, log) -> dict[str, Any]:
    """Open every hardware adapter and register it for orderly shutdown.

    Args:
        config: Runtime configuration.
        stack: Context stack that closes each adapter on the way out, in the
            reverse order it was opened.
        arm_payload: Whether the real servo driver is used. When False a
            stand-in is fitted that records the request and actuates nothing.
        log: Callable used for diagnostic messages.

    Returns:
        A mapping with the vehicle, navigation, target, sonar, and servo objects.
    """
    from ..hardware.cv_link import UdpTargetProvider
    from ..hardware.px4 import Px4Vehicle
    from ..hardware.servo import PayloadServo, RefusingPayloadRelease
    from ..hardware.ultrasonic import HardwareNavigationProvider, SonarArray

    servo = PayloadServo(config.servo, log=log) if arm_payload else RefusingPayloadRelease(log=log)
    stack.callback(servo.close)

    sonar = SonarArray(config.ultrasonic, log=log)
    stack.callback(sonar.close)

    targets = UdpTargetProvider(config.cv, log=log)
    stack.callback(targets.close)

    vehicle = Px4Vehicle(config.mavlink, payload_release=servo, log=log)
    stack.callback(vehicle.close)

    return {
        "vehicle": vehicle,
        "navigation": HardwareNavigationProvider(vehicle.tracker, sonar),
        "targets": targets,
        "sonar": sonar,
        "servo": servo,
    }


def check(config: RuntimeConfig, seconds: float = 30.0, log=print) -> int:
    """Read everything and command nothing, printing what the Pi can see.

    Args:
        config: Runtime configuration.
        seconds: How long to keep printing readings.
        log: Callable used for diagnostic messages.

    Returns:
        Process exit code: 0 when every input looked usable at least once.

    This is the read-only step to run before any flight. It proves the MAVLink
    link, the GPS quality, the three ultrasonic sensors, and the perception link
    all work, without giving this computer any authority over the aircraft.
    """
    with ExitStack() as stack:
        parts = _build_stack(config, stack, arm_payload=False, log=log)
        vehicle, navigation, targets = parts["vehicle"], parts["navigation"], parts["targets"]
        deadline = time.monotonic() + seconds
        ever_ready = False
        log("\nread-only check: nothing will be commanded. Ctrl-C to stop.\n")
        while time.monotonic() < deadline:
            telemetry = vehicle.telemetry()
            readings = navigation.latest_navigation()
            target = targets.latest_target()
            ever_ready = ever_ready or (telemetry.telemetry_fresh and readings.navigation_usable)
            log(
                f"mode={telemetry.flight_mode:<12} armed={telemetry.armed!s:<5} "
                f"link={'ok' if telemetry.telemetry_fresh else 'STALE':<5} "
                f"batt={telemetry.battery_remaining_percent:5.1f}% "
                f"alt={telemetry.altitude_m:5.1f}m "
                f"gps={'ok' if readings.navigation_usable else 'NOT USABLE':<10} "
                f"hdg={readings.heading_deg if readings.heading_deg is not None else float('nan'):6.1f} "
                f"bottom={_sensor(readings.bottom_distance_m, readings.bottom_fresh)} "
                f"side={_sensor(readings.side_distance_m, readings.side_fresh)} "
                f"top={_sensor(readings.top_distance_m, readings.top_fresh)} "
                f"marker={'yes' if target is not None and target.usable else 'no'}"
            )
            time.sleep(1.0)
        log("\ncheck finished.")
        log(f"sonar counters: {parts['sonar'].diagnostics()}")
        log(f"perception counters: {targets.diagnostics()}")
        if not ever_ready:
            log("\nNOT READY: telemetry or GPS never became usable. Do not fly.")
            return 1
        return 0


def _sensor(distance_m: float | None, fresh: bool) -> str:
    """Format one ultrasonic reading for the check display.

    Args:
        distance_m: Filtered distance, or None when there is no usable reading.
        fresh: Whether that reading is recent enough to trust.
    """
    if distance_m is None or not fresh:
        return "  --  "
    return f"{distance_m:5.2f}m"


def fly(config: RuntimeConfig, log=print) -> LandingState:
    """Run the real mission on the aircraft.

    Args:
        config: Runtime configuration, including the mission target.
        log: Callable used for diagnostic messages.

    Returns:
        The state the mission finished in.

    Raises:
        ValueError: If no GPS target is configured, since the delivery mission
            has nowhere to go without one.
    """
    if config.mission.target_position is None:
        raise ValueError("hardware mode needs mission.target_latitude_deg and mission.target_longitude_deg")

    with ExitStack() as stack:
        parts = _build_stack(config, stack, arm_payload=True, log=log)
        vehicle, navigation, targets = parts["vehicle"], parts["navigation"], parts["targets"]

        machine = LandingStateMachine(targets, vehicle, config.mission, navigation)
        mission_log = MissionLog(_log_path(config))
        stack.callback(mission_log.close)

        def diagnostics() -> dict[str, Any]:
            """Collect per-cycle values worth having in the flight log."""
            telemetry = vehicle.telemetry()
            readings = navigation.latest_navigation()
            return {
                "flight_mode": telemetry.flight_mode,
                "armed": telemetry.armed,
                "battery_percent": telemetry.battery_remaining_percent,
                "altitude_m": telemetry.altitude_m,
                "latitude_deg": readings.position.latitude_deg if readings.position else None,
                "longitude_deg": readings.position.longitude_deg if readings.position else None,
                "heading_deg": readings.heading_deg,
                "bottom_m": readings.bottom_distance_m if readings.bottom_fresh else None,
                "side_m": readings.side_distance_m if readings.side_fresh else None,
                "top_m": readings.top_distance_m if readings.top_fresh else None,
                **targets.diagnostics(),
            }

        runner = MissionRunner(
            machine,
            rate_hz=config.loop_rate_hz,
            log=mission_log,
            diagnostics=diagnostics,
            override_source=vehicle.pilot_in_control,
            terminal_linger_s=config.terminal_linger_s,
            on_record=lambda record: log(
                f"{record.time_s:7.1f} | {record.state.value:<15} | "
                f"{record.transition_reason or '-':<28} | {', '.join(record.actions) or '-'}"
            ),
        )
        runner.install_signal_handlers()
        log("\ntime    | state           | reason                       | actions")
        final = runner.run()
        log(f"\nmission finished in state: {final.value}")
        if mission_log.path is not None:
            log(f"flight log: {mission_log.path}")
        return final
