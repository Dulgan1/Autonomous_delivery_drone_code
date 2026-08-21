"""PX4 adapter: turns mission requests into MAVLink, and MAVLink into telemetry.

This module is the only place that knows PX4 exists. It is split in two on
purpose:

* Pure functions and :class:`TelemetryTracker` decode and encode values with no
  radio, no threads, and no pymavlink import. They are unit-tested on a laptop.
* :class:`Px4Vehicle` owns the link, the background threads, and the Offboard
  setpoint stream. It is only imported when hardware mode actually starts.

PX4 Offboard mode needs a setpoint stream faster than 2 Hz, both to enter the
mode and to stay in it. If the stream stops, PX4 leaves Offboard and runs its
own failsafe. That is the behaviour we want on a companion-computer crash, so
this adapter never tries to defeat it.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Any

from ..interfaces import VehicleTelemetry, VelocitySetpoint
from ..models import GpsPosition
from ..runtime.config import MavlinkConfig


PX4_MAIN_MODE = {
    "MANUAL": 1,
    "ALTCTL": 2,
    "POSCTL": 3,
    "AUTO": 4,
    "ACRO": 5,
    "OFFBOARD": 6,
    "STABILIZED": 7,
}
"""PX4 main flight modes, as used in the custom-mode field of a heartbeat."""

PX4_AUTO_SUB_MODE = {
    "READY": 1,
    "TAKEOFF": 2,
    "LOITER": 3,
    "MISSION": 4,
    "RTL": 5,
    "LAND": 6,
    "FOLLOW_TARGET": 8,
    "PRECLAND": 9,
}
"""PX4 sub-modes that live under the AUTO main mode."""

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
"""Heartbeat base-mode bit saying the custom-mode field carries the real mode."""

MAV_MODE_FLAG_SAFETY_ARMED = 128
"""Heartbeat base-mode bit that means the vehicle is armed."""

MAV_STATE_CRITICAL = 5
"""Heartbeat system status meaning PX4 has a serious problem."""

MAV_STATE_EMERGENCY = 6
"""Heartbeat system status meaning PX4 is in an emergency such as a crash."""

MAV_FRAME_BODY_NED = 8
"""Setpoint frame where +x is forward from the nose of the aircraft."""

VELOCITY_AND_YAW_MASK = 0b0000_1001_1100_0111
"""Setpoint type mask: use velocity and yaw, ignore position, acceleration, yaw rate."""

PILOT_MODES = frozenset({"MANUAL", "STABILIZED", "ALTCTL", "POSCTL", "ACRO"})
"""Modes that mean a human is flying the aircraft from the radio right now."""

MINIMUM_GPS_FIX_TYPE = 3
"""GPS fix type 3 is a normal 3D fix; anything lower is not safe to navigate on."""


def encode_custom_mode(main_mode: int, sub_mode: int = 0) -> int:
    """Pack a PX4 main and sub mode into one custom-mode integer.

    Args:
        main_mode: Value from :data:`PX4_MAIN_MODE`.
        sub_mode: Value from :data:`PX4_AUTO_SUB_MODE`, or 0 when unused.

    Returns:
        The custom-mode integer PX4 expects, with the main mode in bits 16-23
        and the sub mode in bits 24-31.
    """
    return (sub_mode << 24) | (main_mode << 16)


def decode_mode_name(custom_mode: int) -> str:
    """Turn a PX4 custom-mode integer back into a readable name.

    Args:
        custom_mode: Value taken from a heartbeat message.

    Returns:
        A name such as ``OFFBOARD`` or ``AUTO.RTL``, or ``UNKNOWN(n)`` when the
        combination is not one this project knows about.
    """
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    main_name = next((name for name, value in PX4_MAIN_MODE.items() if value == main), None)
    if main_name is None:
        return f"UNKNOWN({custom_mode})"
    if main_name != "AUTO":
        return main_name
    sub_name = next((name for name, value in PX4_AUTO_SUB_MODE.items() if value == sub), str(sub))
    return f"AUTO.{sub_name}"


@dataclass
class TelemetryTracker:
    """Builds vehicle facts from decoded MAVLink messages, with staleness.

    Every value keeps the time it arrived, so a link that silently dies is
    reported as stale rather than as a set of confident old numbers.

    Attributes:
        stale_after_s: Age at which a value stops counting as fresh.
        minimum_satellites: Satellites needed before position hold is trusted.
        maximum_eph_m: Worst horizontal GPS accuracy accepted, in metres.
    """

    stale_after_s: float = 2.0
    minimum_satellites: int = 8
    maximum_eph_m: float = 5.0
    armed: bool = False
    flight_mode: str = "UNKNOWN"
    system_status: int = 0
    altitude_m: float = 0.0
    position: GpsPosition | None = None
    heading_deg: float | None = None
    battery_percent: float = 0.0
    gps_fix_type: int = 0
    satellites: int = 0
    eph_m: float = 99.0
    _seen: dict[str, float] = field(default_factory=dict)

    def ingest(self, message_type: str, fields: dict[str, Any], now_s: float) -> None:
        """Take one decoded MAVLink message and update the tracked values.

        Args:
            message_type: MAVLink message name, such as ``GLOBAL_POSITION_INT``.
            fields: The message fields as a plain dictionary.
            now_s: Monotonic time the message arrived.
        """
        self._seen[message_type] = now_s
        if message_type == "HEARTBEAT":
            self.armed = bool(int(fields.get("base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
            self.flight_mode = decode_mode_name(int(fields.get("custom_mode", 0)))
            self.system_status = int(fields.get("system_status", 0))
        elif message_type == "GLOBAL_POSITION_INT":
            self.position = GpsPosition(
                latitude_deg=int(fields.get("lat", 0)) / 1e7,
                longitude_deg=int(fields.get("lon", 0)) / 1e7,
            )
            self.altitude_m = int(fields.get("relative_alt", 0)) / 1000.0
            heading = int(fields.get("hdg", 65535))
            self.heading_deg = None if heading == 65535 else (heading / 100.0) % 360.0
        elif message_type == "GPS_RAW_INT":
            self.gps_fix_type = int(fields.get("fix_type", 0))
            self.satellites = int(fields.get("satellites_visible", 0))
            eph = int(fields.get("eph", 65535))
            self.eph_m = 99.0 if eph == 65535 else eph / 100.0
        elif message_type in ("BATTERY_STATUS", "SYS_STATUS"):
            remaining = fields.get("battery_remaining", -1)
            if remaining is not None and int(remaining) >= 0:
                self.battery_percent = float(remaining)

    def fresh(self, message_type: str, now_s: float) -> bool:
        """Return True when a message type arrived recently enough to trust.

        Args:
            message_type: MAVLink message name to check.
            now_s: Current monotonic time.
        """
        seen_at = self._seen.get(message_type)
        return seen_at is not None and now_s - seen_at <= self.stale_after_s

    @property
    def link_started(self) -> bool:
        """Return True once any heartbeat has ever been received."""
        return "HEARTBEAT" in self._seen

    def position_hold_ready(self, now_s: float) -> bool:
        """Return True when GPS quality is good enough to hold position.

        Args:
            now_s: Current monotonic time.
        """
        return (
            self.fresh("GPS_RAW_INT", now_s)
            and self.fresh("GLOBAL_POSITION_INT", now_s)
            and self.gps_fix_type >= MINIMUM_GPS_FIX_TYPE
            and self.satellites >= self.minimum_satellites
            and self.eph_m <= self.maximum_eph_m
        )

    def snapshot(self, now_s: float, *, expected_mode: str | None, faulted: bool) -> VehicleTelemetry:
        """Return the vehicle facts the mission logic needs.

        Args:
            now_s: Current monotonic time.
            expected_mode: Mode this adapter believes it commanded, or None when
                it is not currently driving the aircraft. A mode that does not
                match means something else took over, which is reported as a
                failsafe so autonomy stands down.
            faulted: True when the adapter itself failed to command the vehicle.

        Returns:
            A telemetry snapshot with honest freshness and readiness flags.
        """
        mode_taken_over = expected_mode is not None and self.flight_mode != expected_mode
        return VehicleTelemetry(
            altitude_m=self.altitude_m,
            telemetry_fresh=self.fresh("HEARTBEAT", now_s) and self.fresh("GLOBAL_POSITION_INT", now_s),
            position_hold_ready=self.position_hold_ready(now_s),
            failsafe_active=(
                faulted
                or mode_taken_over
                or self.system_status in (MAV_STATE_CRITICAL, MAV_STATE_EMERGENCY)
            ),
            battery_remaining_percent=self.battery_percent,
            payload_released=False,
            armed=self.armed,
            flight_mode=self.flight_mode,
        )


@dataclass
class Setpoint:
    """The Offboard setpoint currently being streamed to PX4.

    Attributes:
        forward_mps: Body-frame forward speed; positive is out of the nose.
        right_mps: Body-frame rightward speed.
        down_mps: Body-frame downward speed; positive descends.
        yaw_deg: Compass heading being commanded, or None to hold the last one.
        expires_at_s: Time after which this setpoint reverts to a still hover,
            used so a short forward step really is short. None never expires.
    """

    forward_mps: float = 0.0
    right_mps: float = 0.0
    down_mps: float = 0.0
    yaw_deg: float | None = None
    expires_at_s: float | None = None

    def at(self, now_s: float) -> "Setpoint":
        """Return this setpoint, or a still hover once it has expired.

        Args:
            now_s: Current monotonic time.
        """
        if self.expires_at_s is not None and now_s >= self.expires_at_s:
            return Setpoint(yaw_deg=self.yaw_deg)
        return self


class Px4Vehicle:
    """Sends approved high-level requests to PX4 and reads its telemetry.

    This class implements :class:`~drone_autonomy.interfaces.VehicleInterface`.
    It never commands motors: every request becomes either a PX4 mode change,
    a PX4 command, or one Offboard velocity/yaw setpoint that PX4 is free to
    refuse, limit, or override with its own failsafes.
    """

    def __init__(self, config: MavlinkConfig, payload_release=None, log=print):
        """Open the MAVLink link and start reading telemetry.

        Args:
            config: Link settings, rates, and the arming permission.
            payload_release: Object with a ``release()`` method, or None to
                refuse payload release entirely.
            log: Callable used for diagnostic messages.

        Raises:
            ImportError: If pymavlink is not installed on this machine.
            TimeoutError: If no heartbeat arrives before the link timeout.
        """
        from pymavlink import mavutil

        self._mavutil = mavutil
        self.config = config
        self.payload_release = payload_release
        self.log = log
        self.tracker = TelemetryTracker(
            stale_after_s=config.heartbeat_timeout_s,
            minimum_satellites=config.minimum_satellites,
            maximum_eph_m=config.maximum_eph_m,
        )
        self._setpoint = Setpoint()
        self._setpoint_lock = threading.Lock()
        self._streaming = False
        self._expected_mode: str | None = None
        self._faulted = False
        self._stop = threading.Event()

        self.link = mavutil.mavlink_connection(
            config.connection,
            baud=config.baud,
            source_system=config.source_system,
        )
        self.log(f"waiting for a PX4 heartbeat on {config.connection}")
        if self.link.wait_heartbeat(timeout=30) is None:
            raise TimeoutError(f"no MAVLink heartbeat on {config.connection}")
        self.log(f"heartbeat from system {self.link.target_system}")
        self._request_streams()

        self._reader = threading.Thread(target=self._read_loop, name="px4-reader", daemon=True)
        self._streamer = threading.Thread(target=self._stream_loop, name="px4-setpoints", daemon=True)
        self._reader.start()
        self._streamer.start()

    def close(self) -> None:
        """Stop the Offboard stream and close the link.

        Stopping the stream on purpose is the safe way to finish: PX4 sees the
        setpoints end and applies its configured Offboard-loss failsafe.
        """
        self._stop.set()
        self._streaming = False
        for thread in (self._reader, self._streamer):
            if thread.is_alive():
                thread.join(timeout=2.0)
        try:
            self.link.close()
        except Exception:  # pragma: no cover - closing must never raise
            pass

    def pilot_in_control(self) -> bool:
        """Return True when the flight controller is in a pilot-flown mode.

        On PX4 a radio override does not arrive as a separate signal: the mode
        simply changes to one the pilot flies. Treating that as manual override
        is what makes the radio switch beat autonomy, exactly as required.
        """
        return self.tracker.flight_mode in PILOT_MODES

    def telemetry(self) -> VehicleTelemetry:
        """Return the latest vehicle facts, including honest staleness flags."""
        return self.tracker.snapshot(
            time.monotonic(),
            expected_mode=self._expected_mode,
            faulted=self._faulted,
        )

    def takeoff(self, altitude_m: float) -> None:
        """Ask PX4 to take off to a height, using its own takeoff behaviour.

        Args:
            altitude_m: Height above the takeoff point, in metres.
        """
        if not self.tracker.armed:
            if not self.config.allow_arm:
                self._fault("refusing to arm: allow_arm is false, so a human must arm first")
                return
            self._command(self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self._set_mode("AUTO", "TAKEOFF")
        self._command(
            self._mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, float("nan"), float("nan"), float("nan"), altitude_m,
        )
        self.log(f"takeoff requested to {altitude_m:.1f} m")

    def yaw_to(self, heading_deg: float) -> None:
        """Ask PX4 to turn to a compass heading while holding position.

        Args:
            heading_deg: Compass heading, where 0 is north.
        """
        if self._ensure_offboard():
            self._set_setpoint(Setpoint(yaw_deg=heading_deg % 360.0))

    def forward(self, speed_mps: float, duration_s: float) -> None:
        """Ask PX4 for one short forward step on the current heading.

        Args:
            speed_mps: Forward speed in metres per second.
            duration_s: How long the step lasts before hovering again.
        """
        if not self._ensure_offboard():
            return
        with self._setpoint_lock:
            yaw = self._setpoint.yaw_deg
        self._set_setpoint(Setpoint(
            forward_mps=speed_mps,
            yaw_deg=yaw,
            expires_at_s=time.monotonic() + duration_s,
        ))

    def velocity(self, setpoint: VelocitySetpoint) -> None:
        """Apply an image-guidance correction, if image guidance is permitted.

        Args:
            setpoint: Correction derived from marker position in the image.

        Image error is not metres and the camera is not yet calibrated against
        the aircraft body, so this is refused unless it has been switched on
        deliberately after calibration.
        """
        if not self.config.allow_image_guidance:
            self.log("image guidance refused: allow_image_guidance is false until the camera is calibrated")
            return
        if not self._ensure_offboard():
            return
        with self._setpoint_lock:
            yaw = self._setpoint.yaw_deg
        self._set_setpoint(Setpoint(
            forward_mps=-setpoint.image_y * self.config.image_guidance_scale,
            right_mps=setpoint.image_x * self.config.image_guidance_scale,
            yaw_deg=yaw,
        ))

    def descend(self, rate_mps: float) -> None:
        """Ask PX4 for a slow descent at a fixed rate.

        Args:
            rate_mps: Downward speed in metres per second.
        """
        if not self._ensure_offboard():
            return
        with self._setpoint_lock:
            yaw = self._setpoint.yaw_deg
        self._set_setpoint(Setpoint(down_mps=abs(rate_mps), yaw_deg=yaw))

    def hold(self, reason: str) -> None:
        """Hold position, keeping the Offboard stream alive where possible.

        Args:
            reason: Why the hold was requested; recorded in the log.

        While Offboard is running, holding means streaming a still hover, so a
        short mission pause does not drop out of the mode. When Offboard is not
        running, PX4's own loiter mode is used instead.
        """
        if self._streaming:
            with self._setpoint_lock:
                yaw = self._setpoint.yaw_deg
            self._set_setpoint(Setpoint(yaw_deg=yaw))
        elif self._expected_mode not in ("AUTO.LOITER", "AUTO.RTL", "AUTO.LAND"):
            self._set_mode("AUTO", "LOITER")
            self.log(f"holding: {reason}")

    def release_payload(self) -> None:
        """Release the payload through the servo driver, if one is fitted."""
        if self.payload_release is None:
            self._fault("payload release requested but no servo driver is configured")
            return
        self.payload_release.release()

    def return_home(self, reason: str) -> None:
        """Hand the aircraft to PX4's return-to-launch behaviour.

        Args:
            reason: Why the mission ended; recorded in the log.

        The Offboard stream is stopped first so PX4 owns the aircraft outright
        rather than competing with setpoints from this computer.
        """
        self._streaming = False
        self._set_mode("AUTO", "RTL")
        self.log(f"return home requested: {reason}")

    def land(self) -> None:
        """Hand the aircraft to PX4's landing behaviour and stop streaming."""
        self._streaming = False
        self._set_mode("AUTO", "LAND")
        self.log("land requested")

    def _read_loop(self) -> None:
        """Background thread: decode every incoming message into the tracker."""
        while not self._stop.is_set():
            try:
                message = self.link.recv_match(blocking=True, timeout=0.5)
            except Exception as error:  # pragma: no cover - link faults are rare
                self._fault(f"MAVLink receive failed: {error}")
                continue
            if message is None:
                continue
            name = message.get_type()
            if name == "STATUSTEXT":
                self.log(f"PX4: {getattr(message, 'text', '')}")
            self.tracker.ingest(name, message.to_dict(), time.monotonic())

    def _stream_loop(self) -> None:
        """Background thread: keep the Offboard setpoint stream alive.

        PX4 leaves Offboard if setpoints stop for roughly half a second, so this
        thread runs at the configured rate and never waits on mission logic.
        """
        period = 1.0 / self.config.setpoint_rate_hz
        while not self._stop.is_set():
            if self._streaming:
                with self._setpoint_lock:
                    setpoint = self._setpoint
                self._send_setpoint(setpoint.at(time.monotonic()))
            time.sleep(period)

    def _send_setpoint(self, setpoint: Setpoint) -> None:
        """Send one body-frame velocity and yaw setpoint to PX4.

        Args:
            setpoint: The velocities and heading to command right now.
        """
        from math import radians

        yaw = radians(setpoint.yaw_deg) if setpoint.yaw_deg is not None else 0.0
        try:
            self.link.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
                self.link.target_system,
                self.link.target_component,
                MAV_FRAME_BODY_NED,
                VELOCITY_AND_YAW_MASK,
                0.0, 0.0, 0.0,
                setpoint.forward_mps, setpoint.right_mps, setpoint.down_mps,
                0.0, 0.0, 0.0,
                yaw, 0.0,
            )
        except Exception as error:  # pragma: no cover - link faults are rare
            self._fault(f"setpoint send failed: {error}")

    def _set_setpoint(self, setpoint: Setpoint) -> None:
        """Replace the setpoint the stream thread is sending.

        Args:
            setpoint: The new setpoint.
        """
        with self._setpoint_lock:
            self._setpoint = setpoint

    def _ensure_offboard(self) -> bool:
        """Start the setpoint stream and enter Offboard mode if not already in it.

        Returns:
            True when Offboard is active and it is safe to command movement.

        PX4 refuses the Offboard mode switch unless setpoints are already
        arriving, so the stream is primed before the mode is requested.
        """
        if self._streaming and self.tracker.flight_mode == "OFFBOARD":
            return True
        self._streaming = True
        self._expected_mode = None
        time.sleep(self.config.offboard_priming_s)
        for attempt in range(3):
            self._set_mode("OFFBOARD")
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if self.tracker.flight_mode == "OFFBOARD":
                    self.log("offboard mode active")
                    return True
                time.sleep(0.1)
            self.log(f"offboard request {attempt + 1} not accepted yet")
        self._fault("PX4 refused offboard mode; standing down so its own control stays in charge")
        self._streaming = False
        return False

    def _set_mode(self, main: str, sub: str | None = None) -> None:
        """Request a PX4 flight mode and remember what was asked for.

        Args:
            main: Main mode name from :data:`PX4_MAIN_MODE`.
            sub: Sub-mode name from :data:`PX4_AUTO_SUB_MODE`, when main is AUTO.
        """
        custom = encode_custom_mode(PX4_MAIN_MODE[main], PX4_AUTO_SUB_MODE[sub] if sub else 0)
        self._expected_mode = decode_mode_name(custom)
        self._command(
            self._mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_MAIN_MODE[main],
            PX4_AUTO_SUB_MODE[sub] if sub else 0,
        )

    def _command(self, command: int, *params: float) -> None:
        """Send one MAVLink long command to the flight controller.

        Args:
            command: MAV_CMD identifier.
            *params: Up to seven command parameters; missing ones become zero.
        """
        values = list(params) + [0.0] * (7 - len(params))
        try:
            self.link.mav.command_long_send(
                self.link.target_system,
                self.link.target_component,
                command,
                0,
                *values[:7],
            )
        except Exception as error:  # pragma: no cover - link faults are rare
            self._fault(f"command {command} failed: {error}")

    def _request_streams(self) -> None:
        """Ask PX4 to send the telemetry messages this project depends on."""
        for message_id, rate_hz in (
            (self._mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
            (self._mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
            (self._mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1),
            (self._mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1),
        ):
            self._command(
                self._mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                message_id,
                1_000_000 / rate_hz,
            )

    def _fault(self, message: str) -> None:
        """Record an adapter failure and report it to the mission as a failsafe.

        Args:
            message: What went wrong.

        A fault means this computer could not do what the mission asked. The
        mission must then hold or come home rather than assume the request
        succeeded, so the fault is surfaced through the telemetry snapshot.
        """
        self._faulted = True
        self.log(f"FAULT: {message}")
