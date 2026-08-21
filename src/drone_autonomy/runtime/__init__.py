"""Run modes: scripted simulation on a laptop, real hardware on a Pi.

Both modes build the same state machine from the same configuration. Importing
this package never pulls in a hardware library; :mod:`drone_autonomy.hardware`
is loaded only when hardware mode actually starts.
"""

from .config import (
    CvLinkConfig,
    MavlinkConfig,
    RuntimeConfig,
    ServoConfig,
    SonarPins,
    UltrasonicConfig,
    load_config,
    with_mission,
)
from .loop import LoopRecord, MissionLog, MissionRunner

__all__ = [
    "CvLinkConfig",
    "LoopRecord",
    "MavlinkConfig",
    "MissionLog",
    "MissionRunner",
    "RuntimeConfig",
    "ServoConfig",
    "SonarPins",
    "UltrasonicConfig",
    "load_config",
    "with_mission",
]
