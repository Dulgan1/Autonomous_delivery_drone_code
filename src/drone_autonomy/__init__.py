"""Safety-first, hardware-independent drone landing autonomy.

This package accepts safe visual-target messages and produces only mockable
high-level vehicle requests. It never controls motors or flight hardware.
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
