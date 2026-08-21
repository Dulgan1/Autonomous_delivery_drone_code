"""Receives tracked-marker messages from the separate OpenCV project.

The perception project stays a separate program. It sends one JSON datagram per
frame to a local UDP port, and this module turns those into the
:class:`~drone_autonomy.models.VisualTarget` the mission logic already knows.

UDP is the default because it keeps the two programs independent: a crash or a
slow frame in perception cannot stall a decision cycle here, and old frames are
dropped rather than queued up behind fresh ones. If you later prefer a
different transport, replace :class:`UdpTargetProvider` only; the message shape
and :func:`parse_target_message` stay the same.

Expected message, one JSON object per datagram::

    {
      "track_id": 1,
      "target_point": [320.0, 240.0],
      "horizontal_error": -0.12,
      "vertical_error": 0.05,
      "normalized_radius": 0.18,
      "marker_confidence": 0.82,
      "stable": true,
      "visible": true,
      "sent_at_s": 12345.678
    }

``sent_at_s`` is optional. When present it must come from the same monotonic
clock as this program, which in practice means both processes run on the same
Pi. When absent, arrival time is used instead.
"""

import json
import socket
import threading
import time
from typing import Any

from ..models import VisualTarget
from ..runtime.config import CvLinkConfig


class MessageRejected(ValueError):
    """Raised when a perception message cannot be trusted as a measurement."""


def parse_target_message(payload: bytes | str) -> VisualTarget:
    """Turn one JSON datagram into a visual target.

    Args:
        payload: Raw datagram bytes or an already-decoded string.

    Returns:
        The marker message described by the datagram.

    Raises:
        MessageRejected: If the datagram is not valid JSON, is missing a
            required field, or holds a value of the wrong type. A malformed
            message is dropped rather than partly believed.
    """
    try:
        data: Any = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise MessageRejected(f"not valid JSON: {error}") from error
    if not isinstance(data, dict):
        raise MessageRejected("message must be a JSON object")

    try:
        point = data["target_point"]
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise MessageRejected("target_point must be a two-number list")
        return VisualTarget(
            track_id=data["track_id"],
            target_point=(float(point[0]), float(point[1])),
            horizontal_error=float(data["horizontal_error"]),
            vertical_error=float(data["vertical_error"]),
            normalized_radius=float(data["normalized_radius"]),
            marker_confidence=float(data["marker_confidence"]),
            stable=bool(data.get("stable", False)),
            visible=bool(data.get("visible", False)),
        )
    except KeyError as error:
        raise MessageRejected(f"missing field {error}") from error
    except (TypeError, ValueError) as error:
        raise MessageRejected(f"bad field value: {error}") from error


def message_age_s(payload: dict[str, Any], received_at_s: float, now_s: float) -> float:
    """Return how old a perception message is, in seconds.

    Args:
        payload: The decoded message.
        received_at_s: Monotonic time the datagram arrived.
        now_s: Current monotonic time.

    Returns:
        The message age. The sender's own timestamp is preferred because it
        includes the time perception spent processing the frame; arrival time
        is the fallback.
    """
    sent_at = payload.get("sent_at_s")
    if isinstance(sent_at, (int, float)):
        return max(0.0, now_s - float(sent_at))
    return max(0.0, now_s - received_at_s)


class UdpTargetProvider:
    """Serves the newest usable marker message to the mission logic.

    A background thread drains the socket so only the newest datagram survives.
    Anything older than the configured limit is discarded, because a stale
    marker position is exactly the input that would move the aircraft toward
    where the marker used to be.
    """

    def __init__(self, config: CvLinkConfig, log=print):
        """Bind the UDP port and start receiving.

        Args:
            config: Address, port, and maximum accepted message age.
            log: Callable used for diagnostic messages.
        """
        self.config = config
        self.log = log
        self.received = 0
        self.rejected = 0
        self._latest: tuple[VisualTarget, float] | None = None
        self._lock = threading.Lock()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((config.host, config.port))
        self._socket.settimeout(0.2)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._receive_loop, name="cv-link", daemon=True)
        self._thread.start()
        self.log(f"listening for marker messages on {config.host}:{config.port}")

    def close(self) -> None:
        """Stop receiving and close the socket."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._socket.close()

    def latest_target(self) -> VisualTarget | None:
        """Return the newest marker message, or None when there is no fresh one."""
        with self._lock:
            latest = self._latest
        if latest is None:
            return None
        target, received_at_s = latest
        if time.monotonic() - received_at_s > self.config.max_age_s:
            return None
        return target

    def diagnostics(self) -> dict[str, object]:
        """Return counters useful for spotting a perception link problem."""
        return {"cv_received": self.received, "cv_rejected": self.rejected}

    def _receive_loop(self) -> None:
        """Background thread: keep only the newest valid datagram."""
        while not self._stop.is_set():
            try:
                payload, _ = self._socket.recvfrom(self.config.receive_buffer_bytes)
            except socket.timeout:
                continue
            except OSError:
                return
            self.received += 1
            try:
                target = parse_target_message(payload)
            except MessageRejected as error:
                self.rejected += 1
                if self.rejected % 50 == 1:
                    self.log(f"marker message rejected: {error}")
                continue
            with self._lock:
                self._latest = (target, time.monotonic())
