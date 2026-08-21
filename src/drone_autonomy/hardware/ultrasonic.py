"""HC-SR04 ultrasonic driver with filtering, timestamps, and fault detection.

The filtering rules are pure and tested on a laptop; only :class:`SonarArray`
touches GPIO. That split matters, because the rules are what stop a broken
sensor from reading as "clear".

Wiring warning: HC-SR04 echo output is 5 V and Raspberry Pi GPIO is 3.3 V. Every
echo line must pass through a level shifter or voltage divider before it
reaches the Pi. Connecting echo directly can damage the Pi.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field

from ..models import NavigationReadings
from ..runtime.config import SonarPins, UltrasonicConfig


SPEED_OF_SOUND_MPS = 343.0
"""Speed of sound in dry air near 20 C. Real range varies with temperature."""


def echo_to_distance_m(echo_seconds: float) -> float:
    """Convert an echo pulse length into a distance in metres.

    Args:
        echo_seconds: How long the echo pin stayed high.

    Returns:
        Distance to the reflecting surface; the sound travels there and back,
        so the round-trip time is halved.
    """
    return echo_seconds * SPEED_OF_SOUND_MPS / 2.0


@dataclass
class RangeFilter:
    """Accepts, rejects, and smooths readings from one ultrasonic sensor.

    A reading is only used when it is inside the sensor's honest range. Values
    outside it are dropped rather than clamped, because a clamped out-of-range
    value would look like a confident measurement.

    Attributes:
        name: Sensor name used in diagnostics, such as ``side``.
        min_range_m: Below this is the sensor's blind zone.
        max_range_m: Above this the sensor is not reliable.
        window: How many accepted readings the median is taken over.
        stale_after_s: Age at which the filtered value stops counting as fresh.
        stuck_sample_limit: Identical raw readings in a row that mean the sensor
            is frozen or unplugged rather than genuinely looking at something
            steady.
    """

    name: str
    min_range_m: float = 0.03
    max_range_m: float = 4.0
    window: int = 5
    stale_after_s: float = 0.5
    stuck_sample_limit: int = 20
    _samples: deque[float] = field(default_factory=deque)
    _updated_at_s: float | None = None
    _last_raw: float | None = None
    _repeat_count: int = 0
    rejected: int = 0
    accepted: int = 0

    def __post_init__(self) -> None:
        """Size the sample buffer to the configured median window."""
        self._samples = deque(maxlen=max(1, self.window))

    def submit(self, distance_m: float | None, now_s: float) -> None:
        """Offer one raw reading to the filter.

        Args:
            distance_m: Raw measured distance, or None when the echo timed out.
            now_s: Monotonic time the reading was taken.
        """
        if distance_m is None or not (self.min_range_m <= distance_m <= self.max_range_m):
            self.rejected += 1
            return
        if self._last_raw is not None and abs(distance_m - self._last_raw) < 1e-6:
            self._repeat_count += 1
        else:
            self._repeat_count = 0
        self._last_raw = distance_m
        if self.stuck:
            self.rejected += 1
            return
        self._samples.append(distance_m)
        self._updated_at_s = now_s
        self.accepted += 1

    @property
    def stuck(self) -> bool:
        """Return True when the sensor has repeated one value too many times."""
        return self._repeat_count >= self.stuck_sample_limit

    def value(self) -> float | None:
        """Return the median of recent accepted readings, or None if there are none."""
        if not self._samples:
            return None
        ordered = sorted(self._samples)
        return ordered[len(ordered) // 2]

    def fresh(self, now_s: float) -> bool:
        """Return True when a usable reading arrived recently enough.

        Args:
            now_s: Current monotonic time.
        """
        return (
            self._updated_at_s is not None
            and not self.stuck
            and bool(self._samples)
            and now_s - self._updated_at_s <= self.stale_after_s
        )


class SonarArray:
    """Reads three HC-SR04 sensors in turn on a background thread.

    The sensors are triggered one at a time. Firing them together lets one
    sensor hear another one's pulse, which produces a confident but wrong short
    reading, exactly the failure that would make a blocked path look clear.
    """

    def __init__(self, config: UltrasonicConfig, log=print):
        """Claim the GPIO pins and start sampling.

        Args:
            config: Pin assignments, rates, and filtering rules.
            log: Callable used for diagnostic messages.

        Raises:
            ImportError: If pigpio is not installed.
            RuntimeError: If the pigpio daemon is not running. Start it with
                ``sudo systemctl enable --now pigpiod``.
        """
        import pigpio

        self._pigpio = pigpio
        self.config = config
        self.log = log
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running; try: sudo systemctl enable --now pigpiod")

        self.filters = {
            name: RangeFilter(
                name=name,
                min_range_m=config.min_range_m,
                max_range_m=config.max_range_m,
                window=config.median_window,
                stale_after_s=config.stale_after_s,
                stuck_sample_limit=config.stuck_sample_limit,
            )
            for name in ("bottom", "side", "top")
        }
        self._pins = {"bottom": config.bottom, "side": config.side, "top": config.top}
        for pins in self._pins.values():
            self.pi.set_mode(pins.trigger_pin, pigpio.OUTPUT)
            self.pi.set_mode(pins.echo_pin, pigpio.INPUT)
            self.pi.write(pins.trigger_pin, 0)
        time.sleep(0.05)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._sample_loop, name="sonar", daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stop sampling and release the GPIO pins."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        for pins in self._pins.values():
            self.pi.write(pins.trigger_pin, 0)
        self.pi.stop()

    def distances(self, now_s: float) -> dict[str, tuple[float | None, bool]]:
        """Return each sensor's filtered distance and whether it is fresh.

        Args:
            now_s: Current monotonic time.

        Returns:
            A mapping of sensor name to ``(distance_or_none, is_fresh)``.
        """
        return {name: (f.value(), f.fresh(now_s)) for name, f in self.filters.items()}

    def diagnostics(self) -> dict[str, object]:
        """Return counters useful for spotting a failing sensor in the log."""
        return {
            f"sonar_{name}": {
                "accepted": f.accepted,
                "rejected": f.rejected,
                "stuck": f.stuck,
                "value_m": f.value(),
            }
            for name, f in self.filters.items()
        }

    def _sample_loop(self) -> None:
        """Background thread: trigger each sensor in turn and filter the result."""
        period = 1.0 / max(self.config.sample_rate_hz, 0.1)
        settle = period / len(self._pins)
        while not self._stop.is_set():
            for name, pins in self._pins.items():
                if self._stop.is_set():
                    return
                self.filters[name].submit(self._measure(pins), time.monotonic())
                time.sleep(settle)

    def _measure(self, pins: SonarPins) -> float | None:
        """Send one trigger pulse and time the echo.

        Args:
            pins: Trigger and echo pins for this sensor.

        Returns:
            The measured distance in metres, or None when no echo arrived
            before the timeout. A timeout is normal when nothing is in range
            and must never be reported as a large clear distance.
        """
        self.pi.gpio_trigger(pins.trigger_pin, 10, 1)
        deadline = time.monotonic() + self.config.echo_timeout_s
        while self.pi.read(pins.echo_pin) == 0:
            if time.monotonic() > deadline:
                return None
        started = time.monotonic()
        while self.pi.read(pins.echo_pin) == 1:
            if time.monotonic() > deadline:
                return None
        return echo_to_distance_m(time.monotonic() - started)


class HardwareNavigationProvider:
    """Combines Pixhawk GPS and heading with the three HC-SR04 sensors.

    This is what the mission logic actually reads. Nothing here invents a
    value: if the flight controller or a sensor has not produced a fresh
    reading, the matching freshness flag is False and the mission holds.
    """

    def __init__(self, tracker, sonar: SonarArray):
        """Create the combined navigation source.

        Args:
            tracker: The PX4 :class:`~drone_autonomy.hardware.px4.TelemetryTracker`.
            sonar: The running HC-SR04 array.
        """
        self.tracker = tracker
        self.sonar = sonar

    def latest_navigation(self) -> NavigationReadings:
        """Return one combined navigation message for this decision cycle."""
        now_s = time.monotonic()
        distances = self.sonar.distances(now_s)
        gps_fresh = self.tracker.fresh("GLOBAL_POSITION_INT", now_s) and self.tracker.position_hold_ready(now_s)
        bottom, bottom_fresh = distances["bottom"]
        side, side_fresh = distances["side"]
        top, top_fresh = distances["top"]
        return NavigationReadings(
            position=self.tracker.position,
            heading_deg=self.tracker.heading_deg,
            bottom_distance_m=bottom,
            side_distance_m=side,
            top_distance_m=top,
            gps_fresh=gps_fresh,
            heading_fresh=gps_fresh and self.tracker.heading_deg is not None,
            bottom_fresh=bottom_fresh,
            side_fresh=side_fresh,
            top_fresh=top_fresh,
        )
