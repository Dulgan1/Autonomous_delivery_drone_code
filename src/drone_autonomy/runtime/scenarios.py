"""A library of named fake flights for simulation mode.

Each scenario is a scripted list of moments plus the mission settings that go
with it. They exist so every safety path can be watched and re-checked on a
laptop before any of it is trusted on an aircraft.
"""

from collections.abc import Callable
from dataclasses import dataclass

from ..models import GpsPosition, NavigationReadings, VisualTarget
from ..simulation import ScenarioStep
from ..state_machine import LandingConfig


DESTINATION = GpsPosition(latitude_deg=9.0, longitude_deg=7.4)
"""Fake delivery point. Any valid coordinate works; this one is arbitrary."""

METRES_PER_DEGREE_LATITUDE = 111_111.0
"""Rough north-south scale used only to place fake positions near the target."""


def offset(position: GpsPosition, north_m: float, east_m: float) -> GpsPosition:
    """Return a fake position a few metres from another one.

    Args:
        position: Starting point.
        north_m: Metres to move north; negative moves south.
        east_m: Metres to move east; negative moves west.

    Returns:
        A nearby GPS position. This flat approximation is only good enough for
        the small distances used in these fake flights.
    """
    from math import cos, radians

    east_scale = METRES_PER_DEGREE_LATITUDE * cos(radians(position.latitude_deg))
    return GpsPosition(
        latitude_deg=position.latitude_deg + north_m / METRES_PER_DEGREE_LATITUDE,
        longitude_deg=position.longitude_deg + east_m / east_scale,
    )


def readings(
    position: GpsPosition,
    *,
    heading_deg: float = 0.0,
    side_m: float = 5.0,
    top_m: float = 5.0,
    bottom_m: float = 5.0,
    gps_fresh: bool = True,
    side_fresh: bool = True,
    top_fresh: bool = True,
) -> NavigationReadings:
    """Return one fake set of GPS, heading, and HC-SR04 readings.

    Args:
        position: Fake GPS position at this moment.
        heading_deg: Fake compass heading, where 0 is north.
        side_m: Fake side-sensor distance; below the clearance means blocked.
        top_m: Fake top-sensor distance.
        bottom_m: Fake bottom-sensor distance.
        gps_fresh: Whether GPS and heading count as recent.
        side_fresh: Whether the side reading counts as recent.
        top_fresh: Whether the top reading counts as recent.

    Returns:
        Navigation readings the state machine can use or reject.
    """
    return NavigationReadings(
        position=position,
        heading_deg=heading_deg,
        bottom_distance_m=bottom_m,
        side_distance_m=side_m,
        top_distance_m=top_m,
        gps_fresh=gps_fresh,
        heading_fresh=gps_fresh,
        bottom_fresh=True,
        side_fresh=side_fresh,
        top_fresh=top_fresh,
    )


def marker(*, horizontal: float = 0.0, vertical: float = 0.0, stable: bool = True, visible: bool = True) -> VisualTarget:
    """Return one fake tracked marker message from the perception project.

    Args:
        horizontal: Image error from left (-1) to right (+1).
        vertical: Image error from top (-1) to bottom (+1).
        stable: Whether the tracker considers the marker consistent.
        visible: Whether the marker is in the current camera frame.

    Returns:
        A visual target the state machine can use or reject.
    """
    return VisualTarget(
        track_id=1,
        target_point=(320.0, 240.0),
        horizontal_error=horizontal,
        vertical_error=vertical,
        normalized_radius=0.2,
        marker_confidence=0.9,
        stable=stable,
        visible=visible,
    )


@dataclass(frozen=True)
class Scenario:
    """One named fake flight.

    Attributes:
        name: Short identifier used on the command line.
        description: What this fake flight is meant to prove.
        config: Mission settings for this flight.
        steps: The scripted moments, in time order.
        expected_final_state: State value the flight should end in, used as a
            pass or fail check when the whole library is run.
    """

    name: str
    description: str
    config: LandingConfig
    steps: list[ScenarioStep]
    expected_final_state: str


def _quick_config(**changes) -> LandingConfig:
    """Return mission settings with the waits removed so a flight fits in seconds.

    Args:
        **changes: Extra LandingConfig fields to override.

    Returns:
        Mission settings suitable for a short scripted flight. The zeroed dwell
        times are a simulation convenience and are never real flight values.
    """
    defaults = dict(
        target_position=DESTINATION,
        takeoff_settle_s=0.0,
        acquisition_dwell_s=0.0,
        alignment_dwell_s=0.0,
        payload_alignment_dwell_s=0.0,
        search_point_dwell_s=1.0,
    )
    defaults.update(changes)
    return LandingConfig(**defaults)


def _nominal() -> Scenario:
    """Fly to the target, find the marker, release the payload, return home."""
    first_grid_point = offset(DESTINATION, -5.0, -5.0)
    steps = [ScenarioStep(t, navigation=readings(DESTINATION)) for t in (0, 1, 2)]
    steps.append(ScenarioStep(3, navigation=readings(first_grid_point)))
    steps += [
        ScenarioStep(t, target=marker(), navigation=readings(first_grid_point))
        for t in range(4, 10)
    ]
    return Scenario(
        name="nominal",
        description="Complete delivery: GPS travel, marker found, one servo release, return home.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="return_home",
    )


def _blocked_path() -> Scenario:
    """Meet an obstacle during travel, yaw-scan, and continue on a clear heading."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start)),
        ScenarioStep(1, navigation=readings(start)),
        # Facing the target, but the side sensor sees something close ahead.
        ScenarioStep(2, navigation=readings(start, side_m=0.5)),
        # Scan candidate one is the target bearing itself: still blocked.
        ScenarioStep(3, navigation=readings(start, side_m=0.5)),
        # Candidate two is 30 degrees left, so a turn is requested.
        ScenarioStep(4, navigation=readings(start, side_m=0.5)),
        # The drone is now facing 330 and that direction is clear.
        ScenarioStep(5, navigation=readings(start, heading_deg=330.0, side_m=5.0)),
        # Travel resumes with one short forward step on the clear heading.
        ScenarioStep(6, navigation=readings(start, heading_deg=330.0, side_m=5.0)),
    ]
    return Scenario(
        name="blocked_path",
        description="Side sensor blocks a forward step, yaw scan finds a clear heading, travel resumes.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="gps_navigate",
    )


def _no_clear_direction() -> Scenario:
    """Every scanned heading is blocked, so the drone returns home instead of guessing."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start)),
        ScenarioStep(1, navigation=readings(start)),
    ]
    # Heading already matches every candidate, so each scan step is rejected in turn.
    steps += [ScenarioStep(t, navigation=readings(start, side_m=0.2)) for t in range(2, 12)]
    return Scenario(
        name="no_clear_direction",
        description="All yaw-scan headings blocked, so the mission returns home rather than guessing.",
        config=_quick_config(yaw_scan_offsets_deg=(0.0,), heading_tolerance_deg=180.0),
        steps=steps,
        expected_final_state="return_home",
    )


def _marker_never_found() -> Scenario:
    """Search every grid point, see no marker, and return home."""
    grid_point = offset(DESTINATION, -5.0, -5.0)
    steps = [ScenarioStep(t, navigation=readings(DESTINATION)) for t in (0, 1, 2)]
    # Arrive at every grid point immediately and dwell without ever seeing a marker.
    steps += [ScenarioStep(3 + t, navigation=readings(grid_point)) for t in range(0, 30)]
    return Scenario(
        name="marker_never_found",
        description="Whole search grid checked with no marker, so the mission returns home.",
        config=_quick_config(search_area_side_m=4.0, search_grid_spacing_m=2.0, arrival_radius_m=50.0),
        steps=steps,
        expected_final_state="return_home",
    )


def _search_timeout() -> Scenario:
    """Prove the search clock covers the whole search, not one grid point."""
    grid_point = offset(DESTINATION, -5.0, -5.0)
    steps = [ScenarioStep(t, navigation=readings(DESTINATION)) for t in (0, 1, 2)]
    steps += [ScenarioStep(3 + t, navigation=readings(grid_point)) for t in range(0, 20)]
    return Scenario(
        name="search_timeout",
        description="Search phase exceeds its total time limit across several grid points and returns home.",
        config=_quick_config(search_timeout_s=6.0, arrival_radius_m=50.0),
        steps=steps,
        expected_final_state="return_home",
    )


def _low_battery() -> Scenario:
    """Battery reaches the return-home threshold during travel."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start), battery_percent=90.0),
        ScenarioStep(1, navigation=readings(start), battery_percent=70.0),
        ScenarioStep(2, navigation=readings(start), battery_percent=39.0),
        ScenarioStep(3, navigation=readings(start), battery_percent=39.0),
    ]
    return Scenario(
        name="low_battery",
        description="Battery at or below the 40% threshold during the mission requests return home.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="return_home",
    )


def _gps_lost() -> Scenario:
    """GPS becomes stale during travel, which must hold rather than guess a heading."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start)),
        ScenarioStep(1, navigation=readings(start)),
        ScenarioStep(2, navigation=readings(start, gps_fresh=False)),
        ScenarioStep(3, navigation=readings(start, gps_fresh=False)),
    ]
    return Scenario(
        name="gps_lost",
        description="Stale GPS or heading during travel holds position instead of guessing a direction.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="hold",
    )


def _top_blocked() -> Scenario:
    """Something overhead stops the flight before takeoff."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start, top_m=0.2)),
        ScenarioStep(1, navigation=readings(start, top_m=0.2)),
    ]
    return Scenario(
        name="top_blocked",
        description="Overhead obstacle at preflight prevents takeoff and holds.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="hold",
    )


def _not_armed() -> Scenario:
    """Autonomy refuses to start a mission on a disarmed aircraft."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start), armed=False),
        ScenarioStep(1, navigation=readings(start), armed=False),
    ]
    return Scenario(
        name="not_armed",
        description="Preflight refuses to continue while the flight controller reports the vehicle disarmed.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="hold",
    )


def _unstable_marker() -> Scenario:
    """An unstable or held-from-memory marker must never start an alignment."""
    grid_point = offset(DESTINATION, -5.0, -5.0)
    steps = [ScenarioStep(t, navigation=readings(DESTINATION)) for t in (0, 1, 2)]
    steps += [
        ScenarioStep(3, navigation=readings(grid_point)),
        ScenarioStep(4, target=marker(stable=False), navigation=readings(grid_point)),
        ScenarioStep(5, target=marker(visible=False), navigation=readings(grid_point)),
        ScenarioStep(6, target=marker(visible=False), navigation=readings(grid_point)),
    ]
    return Scenario(
        name="unstable_marker",
        description="Unstable and invisible markers are ignored, so no alignment or release begins.",
        config=_quick_config(arrival_radius_m=50.0, search_point_dwell_s=30.0),
        steps=steps,
        expected_final_state="search",
    )


def _marker_lost_before_drop() -> Scenario:
    """The marker disappears while the drone is holding for the release check."""
    grid_point = offset(DESTINATION, -5.0, -5.0)
    steps = [ScenarioStep(t, navigation=readings(DESTINATION)) for t in (0, 1, 2)]
    steps += [
        ScenarioStep(3, navigation=readings(grid_point)),
        ScenarioStep(4, target=marker(), navigation=readings(grid_point)),
        ScenarioStep(5, target=marker(), navigation=readings(grid_point)),
        ScenarioStep(6, navigation=readings(grid_point)),
        ScenarioStep(7, navigation=readings(grid_point)),
        ScenarioStep(8, navigation=readings(grid_point)),
        ScenarioStep(9, navigation=readings(grid_point)),
    ]
    return Scenario(
        name="marker_lost_before_drop",
        description="Marker lost during the pre-release check holds, then returns home without releasing.",
        config=_quick_config(payload_alignment_dwell_s=5.0, target_reacquire_timeout_s=2.0),
        steps=steps,
        expected_final_state="return_home",
    )


def _manual_override() -> Scenario:
    """The pilot takes control mid-mission; autonomy must stop immediately."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start)),
        ScenarioStep(1, navigation=readings(start)),
        ScenarioStep(2, navigation=readings(start), manual_override=True),
        ScenarioStep(3, navigation=readings(start), manual_override=True),
    ]
    return Scenario(
        name="manual_override",
        description="Operator control takes priority over every autonomous decision.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="hold",
    )


def _pixhawk_failsafe() -> Scenario:
    """Pixhawk raises its own failsafe, so this code stops competing with it."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start)),
        ScenarioStep(1, navigation=readings(start)),
        ScenarioStep(2, navigation=readings(start), failsafe_active=True),
        ScenarioStep(3, navigation=readings(start), failsafe_active=True),
    ]
    return Scenario(
        name="pixhawk_failsafe",
        description="A flight-controller failsafe holds autonomy so Pixhawk's own recovery stays in charge.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="hold",
    )


def _link_lost() -> Scenario:
    """The Pi-to-Pixhawk telemetry link goes stale during travel."""
    start = offset(DESTINATION, -40.0, 0.0)
    steps = [
        ScenarioStep(0, navigation=readings(start)),
        ScenarioStep(1, navigation=readings(start)),
        ScenarioStep(2, navigation=readings(start), telemetry_fresh=False),
        ScenarioStep(3, navigation=readings(start), telemetry_fresh=False),
    ]
    return Scenario(
        name="link_lost",
        description="Stale Pi-to-Pixhawk telemetry requests return home.",
        config=_quick_config(),
        steps=steps,
        expected_final_state="return_home",
    )


_BUILDERS: tuple[Callable[[], Scenario], ...] = (
    _nominal,
    _blocked_path,
    _no_clear_direction,
    _marker_never_found,
    _search_timeout,
    _low_battery,
    _gps_lost,
    _top_blocked,
    _not_armed,
    _unstable_marker,
    _marker_lost_before_drop,
    _manual_override,
    _pixhawk_failsafe,
    _link_lost,
)


def all_scenarios() -> list[Scenario]:
    """Return every named fake flight, in a stable order."""
    return [build() for build in _BUILDERS]


def get_scenario(name: str) -> Scenario:
    """Return one fake flight by name.

    Args:
        name: Scenario name as shown by ``--list``.

    Returns:
        The matching scenario.

    Raises:
        KeyError: If no scenario has that name.
    """
    for scenario in all_scenarios():
        if scenario.name == name:
            return scenario
    known = ", ".join(scenario.name for scenario in all_scenarios())
    raise KeyError(f"unknown scenario {name!r}; available: {known}")
