"""Safety-first, hardware-independent drone delivery autonomy.

The mission logic in this package decides what the aircraft should do next. It
never controls motors. It runs in one of two modes, both built from the same
state machine and the same configuration:

* Simulation on any computer, where every input is scripted and every request
  is recorded and thrown away.
* Hardware on a Raspberry Pi, where a PX4 adapter, HC-SR04 drivers, a servo
  driver, and a link to the separate OpenCV project stand behind the same
  interfaces.

Importing this package never pulls in a hardware library.
"""

from .models import GpsPosition, LandingState, NavigationReadings, VisualTarget
from .simulation import ScenarioStep, SimulationRunner
from .state_machine import LandingConfig, LandingStateMachine

__all__ = [
    "LandingConfig",
    "GpsPosition",
    "LandingState",
    "LandingStateMachine",
    "NavigationReadings",
    "ScenarioStep",
    "SimulationRunner",
    "VisualTarget",
]
