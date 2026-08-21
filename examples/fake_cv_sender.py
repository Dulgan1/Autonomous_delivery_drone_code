"""Pretend to be the OpenCV project, so the perception link can be tested alone.

Run this next to ``python -m drone_autonomy check`` on the Raspberry Pi to prove
the UDP link, the message format, and the freshness rules work before a real
camera is involved.

    python examples/fake_cv_sender.py --pattern approach

Patterns:

* ``approach``  marker drifts from the left of the image to the centre
* ``centred``   marker sits still in the centre
* ``unstable``  marker is reported as not stable, so autonomy must ignore it
* ``held``      marker is reported as not visible, so autonomy must ignore it
"""

import argparse
import json
import math
import socket
import time


def build_message(pattern: str, elapsed_s: float) -> dict:
    """Return one fake tracked-marker message.

    Args:
        pattern: Which fake behaviour to produce.
        elapsed_s: Seconds since the sender started.

    Returns:
        A message in the shape :mod:`drone_autonomy.hardware.cv_link` expects.
    """
    horizontal = 0.0
    if pattern == "approach":
        horizontal = max(-0.8, -0.8 + elapsed_s * 0.08)
    message = {
        "track_id": 1,
        "target_point": [320.0 + horizontal * 320.0, 240.0],
        "horizontal_error": round(horizontal, 4),
        "vertical_error": round(0.02 * math.sin(elapsed_s), 4),
        "normalized_radius": 0.18,
        "marker_confidence": 0.85,
        "stable": pattern != "unstable",
        "visible": pattern != "held",
        "sent_at_s": time.monotonic(),
    }
    return message


def main() -> int:
    """Send fake marker messages until stopped."""
    parser = argparse.ArgumentParser(description="Send fake marker messages to the autonomy CV port.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5601)
    parser.add_argument("--rate", type=float, default=15.0, help="messages per second")
    parser.add_argument("--pattern", default="approach", choices=("approach", "centred", "unstable", "held"))
    args = parser.parse_args()

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = time.monotonic()
    print(f"sending {args.pattern!r} marker messages to {args.host}:{args.port} at {args.rate} Hz; Ctrl-C to stop")
    try:
        while True:
            message = build_message(args.pattern, time.monotonic() - started)
            sender.sendto(json.dumps(message).encode("utf-8"), (args.host, args.port))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        sender.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
