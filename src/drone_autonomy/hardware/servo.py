"""Payload-release servo driven straight from a Raspberry Pi GPIO pin.

This is the one part of the system that physically lets go of the payload, and
it is deliberately the most reluctant. It refuses to fire unless a software
interlock has been switched on, it fires once per flight, and it turns the
signal off afterwards.

Two things about driving the servo from the Pi rather than through Pixhawk are
worth keeping in mind while testing:

* The servo needs its own regulated supply that shares ground with the Pi. A
  servo drawing stall current through a Pi 5 V pin can brown out the Pi.
* If the Pi loses power or the process is killed, the signal stops and the
  servo is left wherever it was. The mechanism should hold the payload
  mechanically at rest rather than relying on servo torque.
"""

import threading
import time

from ..runtime.config import ServoConfig


class PayloadServo:
    """Drives one release servo, once, behind a software interlock."""

    def __init__(self, config: ServoConfig, log=print):
        """Claim the servo pin and move it to the locked position.

        Args:
            config: Pin, pulse widths, and the interlock flag.
            log: Callable used for diagnostic messages.

        Raises:
            ImportError: If pigpio is not installed.
            RuntimeError: If the pigpio daemon is not running.
        """
        import pigpio

        self.config = config
        self.log = log
        self.released = False
        self._lock = threading.Lock()
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon is not running; try: sudo systemctl enable --now pigpiod")
        self.pi.set_servo_pulsewidth(config.pin, config.locked_us if config.idle_pulse else 0)
        state = "armed" if config.enabled else "LOCKED OUT (servo.enabled is false)"
        self.log(f"payload servo on GPIO {config.pin}: {state}")

    def release(self) -> bool:
        """Move the servo to its release position, once.

        Returns:
            True if the servo was actually driven to the release position.

        The request is refused when the interlock is off or when a release has
        already happened this flight. A refusal is logged loudly, because a
        silent refusal during a real delivery would be worse than a failure.
        """
        with self._lock:
            if not self.config.enabled:
                self.log("PAYLOAD RELEASE REFUSED: servo.enabled is false")
                return False
            if self.released:
                self.log("PAYLOAD RELEASE REFUSED: already released once this flight")
                return False
            self.released = True

        self.log("payload release: driving servo to the open position")
        self.pi.set_servo_pulsewidth(self.config.pin, self.config.released_us)
        time.sleep(self.config.release_hold_s)
        self.pi.set_servo_pulsewidth(self.config.pin, 0)
        self.log("payload release complete; servo signal switched off")
        return True

    def close(self) -> None:
        """Switch the servo signal off and release the pigpio connection."""
        try:
            self.pi.set_servo_pulsewidth(self.config.pin, 0)
        finally:
            self.pi.stop()


class RefusingPayloadRelease:
    """A stand-in release that only records the request.

    Used for bench and read-only flights so the mission can run all the way
    through its release decision without anything physically opening.
    """

    def __init__(self, log=print):
        """Create the stand-in.

        Args:
            log: Callable used for diagnostic messages.
        """
        self.log = log
        self.requests = 0

    def release(self) -> bool:
        """Record a release request and refuse it.

        Returns:
            Always False, because nothing was actuated.
        """
        self.requests += 1
        self.log("payload release requested; no servo is fitted in this run mode")
        return False

    def close(self) -> None:
        """Present for symmetry with :class:`PayloadServo`; does nothing."""
