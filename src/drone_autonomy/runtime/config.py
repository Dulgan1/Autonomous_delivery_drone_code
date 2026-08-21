"""One configuration object shared by simulation mode and hardware mode.

Everything that differs between a laptop simulation and a real Raspberry Pi
flight lives here, so the mission logic itself never needs to know which mode
it is running in. Values are loaded from a TOML file; every field has a
conservative default so a missing section is never silently unsafe.
"""

from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any

from ..models import GpsPosition
from ..state_machine import LandingConfig


@dataclass(frozen=True)
class MavlinkConfig:
    """How the Raspberry Pi talks to the Pixhawk over MAVLink.

    Attributes:
        connection: pymavlink connection string, such as ``/dev/serial0`` for a
            wired telemetry port or ``udpin:0.0.0.0:14540`` for a PX4 simulator.
        baud: Serial baud rate; ignored for UDP and TCP connections.
        source_system: MAVLink system id this companion computer announces as.
        heartbeat_timeout_s: Telemetry older than this marks the link stale.
        setpoint_rate_hz: Rate of the PX4 Offboard setpoint stream. PX4 leaves
            Offboard mode if setpoints stop for roughly half a second, so this
            must stay comfortably above 2 Hz.
        offboard_priming_s: How long setpoints are streamed before requesting
            Offboard mode. PX4 refuses the mode switch without a live stream.
        allow_arm: Whether autonomy may arm the vehicle itself. The safe
            default is False, which requires a human to arm first.
        takeoff_timeout_s: How long to wait for the commanded takeoff altitude.
        allow_image_guidance: Whether marker image error may move the aircraft.
            The safe default is False, because image error is not metres and
            the camera has not been calibrated against the aircraft body.
        image_guidance_scale: Metres per second per unit of image error, used
            only once ``allow_image_guidance`` has been switched on.
        minimum_satellites: GPS satellites required before position hold is
            treated as trustworthy.
        maximum_eph_m: Worst reported horizontal GPS accuracy accepted.
    """

    connection: str = "/dev/serial0"
    baud: int = 921_600
    source_system: int = 255
    heartbeat_timeout_s: float = 2.0
    setpoint_rate_hz: float = 20.0
    offboard_priming_s: float = 1.0
    allow_arm: bool = False
    takeoff_timeout_s: float = 30.0
    allow_image_guidance: bool = False
    image_guidance_scale: float = 0.30
    minimum_satellites: int = 8
    maximum_eph_m: float = 5.0


@dataclass(frozen=True)
class SonarPins:
    """Trigger and echo GPIO pins for one HC-SR04 sensor.

    Pins are BCM numbers. The echo pin must reach the Pi through a level
    shifter or voltage divider: HC-SR04 echo is 5 V and Pi GPIO is 3.3 V.

    Attributes:
        trigger_pin: BCM pin driving the 10 microsecond trigger pulse.
        echo_pin: BCM pin reading the level-shifted echo pulse.
    """

    trigger_pin: int
    echo_pin: int


@dataclass(frozen=True)
class UltrasonicConfig:
    """HC-SR04 wiring, timing, and filtering rules.

    Attributes:
        bottom: Pins for the downward sensor used near the ground.
        side: Pins for the sensor that checks the direction being faced.
        top: Pins for the upward sensor that gates climbing.
        sample_rate_hz: How often the whole set of three sensors is read. The
            sensors are triggered one at a time to avoid hearing each other.
        median_window: Number of recent readings combined by the median filter.
        stale_after_s: A sensor with no accepted reading for this long is
            reported as not fresh, which the mission logic treats as unsafe.
        min_range_m: Readings below this are rejected as HC-SR04 blind zone.
        max_range_m: Readings above this are rejected as out of usable range.
        stuck_sample_limit: Identical readings in a row that mark a sensor as
            frozen or disconnected rather than genuinely steady.
        echo_timeout_s: Longest echo wait before the reading is abandoned.
    """

    bottom: SonarPins = SonarPins(trigger_pin=23, echo_pin=24)
    side: SonarPins = SonarPins(trigger_pin=17, echo_pin=27)
    top: SonarPins = SonarPins(trigger_pin=5, echo_pin=6)
    sample_rate_hz: float = 10.0
    median_window: int = 5
    stale_after_s: float = 0.5
    min_range_m: float = 0.03
    max_range_m: float = 4.0
    stuck_sample_limit: int = 20
    echo_timeout_s: float = 0.04


@dataclass(frozen=True)
class ServoConfig:
    """Payload-release servo driven directly from a Raspberry Pi GPIO pin.

    The servo needs its own regulated supply sharing ground with the Pi. It
    must not be powered from a Pi 5 V pin under load.

    Attributes:
        pin: BCM pin carrying the servo signal, driven by pigpio.
        locked_us: Pulse width in microseconds that holds the payload.
        released_us: Pulse width in microseconds that releases the payload.
        release_hold_s: How long the released pulse is held before the signal
            is switched off.
        enabled: Software interlock. While False, a release request is refused
            and logged. Keep it False for every ground and bench test.
        idle_pulse: Whether to keep sending ``locked_us`` while idle. False
            switches the signal off, which lets a sprung mechanism rely on its
            own mechanical lock rather than on continuous servo torque.
    """

    pin: int = 18
    locked_us: int = 1_000
    released_us: int = 2_000
    release_hold_s: float = 1.5
    enabled: bool = False
    idle_pulse: bool = True


@dataclass(frozen=True)
class CvLinkConfig:
    """How tracked-marker messages arrive from the separate OpenCV project.

    The default transport is one JSON datagram per frame on a local UDP port.
    UDP keeps the two programs in separate processes, so a crash or a slow
    frame in perception cannot stall the mission loop, and old frames are
    dropped instead of queueing up behind fresh ones.

    Attributes:
        host: Address to bind; ``127.0.0.1`` accepts only local messages.
        port: UDP port the perception project sends to.
        max_age_s: Messages older than this are ignored. If a message carries
            no timestamp, arrival time is used instead.
        receive_buffer_bytes: Largest accepted datagram.
    """

    host: str = "127.0.0.1"
    port: int = 5601
    max_age_s: float = 0.5
    receive_buffer_bytes: int = 4_096


@dataclass(frozen=True)
class RuntimeConfig:
    """Everything needed to start either run mode.

    Attributes:
        loop_rate_hz: How often the mission loop makes a decision.
        log_dir: Directory for the JSON-lines mission log, or None for no file.
        terminal_linger_s: How long the loop keeps running after the mission
            reaches return-home, land, or abort, so the vehicle keeps getting
            valid requests while the flight controller takes over.
        mission: The mission limits and waits used by the state machine.
        mavlink: Pixhawk link settings, hardware mode only.
        ultrasonic: HC-SR04 settings, hardware mode only.
        servo: Payload-release settings, hardware mode only.
        cv: Perception-link settings, hardware mode only.
    """

    loop_rate_hz: float = 10.0
    log_dir: str | None = "logs"
    terminal_linger_s: float = 10.0
    mission: LandingConfig = LandingConfig()
    mavlink: MavlinkConfig = MavlinkConfig()
    ultrasonic: UltrasonicConfig = UltrasonicConfig()
    servo: ServoConfig = ServoConfig()
    cv: CvLinkConfig = CvLinkConfig()


def _build(kind: type, values: dict[str, Any] | None, **nested: Any) -> Any:
    """Build one frozen config object from a TOML table, rejecting unknowns.

    Args:
        kind: Config dataclass to build.
        values: Raw table from the TOML file, or None when the section is absent.
        **nested: Already-built child objects that override raw values.

    Returns:
        An instance of ``kind`` with defaults for anything not supplied.

    Raises:
        ValueError: If the table contains a key the config does not define,
            which is almost always a typo in a safety-relevant setting.
    """
    known = {field.name for field in fields(kind)}
    supplied = dict(values or {})
    for key in nested:
        supplied.pop(key, None)
    unknown = sorted(set(supplied) - known)
    if unknown:
        raise ValueError(f"unknown {kind.__name__} setting(s): {', '.join(unknown)}")
    return kind(**supplied, **nested)


def load_config(path: str | Path | None = None) -> RuntimeConfig:
    """Read a TOML configuration file and return a checked RuntimeConfig.

    Args:
        path: TOML file to read, or None to accept every default.

    Returns:
        A fully populated configuration.

    Raises:
        FileNotFoundError: If ``path`` does not exist.
        ValueError: If a section contains an unknown key, or the mission target
            supplies only one of latitude and longitude.
    """
    if path is None:
        return RuntimeConfig()
    import tomllib

    raw = tomllib.loads(Path(path).read_text(encoding="utf-8"))
    mission_table = dict(raw.get("mission") or {})
    latitude = mission_table.pop("target_latitude_deg", None)
    longitude = mission_table.pop("target_longitude_deg", None)
    if (latitude is None) != (longitude is None):
        raise ValueError("mission target needs both target_latitude_deg and target_longitude_deg")
    target = GpsPosition(float(latitude), float(longitude)) if latitude is not None else None
    if target is not None and not target.usable:
        raise ValueError("mission target latitude/longitude are out of range")
    if "yaw_scan_offsets_deg" in mission_table:
        mission_table["yaw_scan_offsets_deg"] = tuple(float(v) for v in mission_table["yaw_scan_offsets_deg"])

    ultrasonic_table = dict(raw.get("ultrasonic") or {})
    sonar: dict[str, SonarPins] = {}
    for name in ("bottom", "side", "top"):
        pins_table = ultrasonic_table.pop(name, None)
        if pins_table is not None:
            sonar[name] = _build(SonarPins, pins_table)

    return _build(
        RuntimeConfig,
        raw,
        mission=_build(LandingConfig, mission_table, target_position=target),
        mavlink=_build(MavlinkConfig, raw.get("mavlink")),
        ultrasonic=_build(UltrasonicConfig, ultrasonic_table, **sonar),
        servo=_build(ServoConfig, raw.get("servo")),
        cv=_build(CvLinkConfig, raw.get("cv")),
    )


def with_mission(config: RuntimeConfig, **changes: Any) -> RuntimeConfig:
    """Return a copy of ``config`` with some mission settings replaced.

    Args:
        config: Configuration to copy.
        **changes: LandingConfig fields to override.

    Returns:
        A new RuntimeConfig; the original is unchanged.
    """
    return replace(config, mission=replace(config.mission, **changes))
