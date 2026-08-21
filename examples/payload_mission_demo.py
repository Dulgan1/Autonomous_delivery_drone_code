"""Print a fake GPS-to-payload-drop mission; it never controls hardware."""

from drone_autonomy import GpsPosition, LandingConfig, LandingStateMachine, NavigationReadings, ScenarioStep, SimulationRunner, VisualTarget
from drone_autonomy.mocks import MockNavigationProvider, MockTargetProvider, MockVehicle
from drone_autonomy.simulation import format_timeline


def navigation(position: GpsPosition) -> NavigationReadings:
    """Return safe fake GPS, heading, and HC-SR04 readings."""
    return NavigationReadings(
        position=position,
        heading_deg=0.0,
        bottom_distance_m=5.0,
        side_distance_m=5.0,
        top_distance_m=5.0,
        gps_fresh=True,
        heading_fresh=True,
        bottom_fresh=True,
        side_fresh=True,
        top_fresh=True,
    )


def marker() -> VisualTarget:
    """Return a fake stable marker exactly at image centre."""
    return VisualTarget(
        track_id=1,
        target_point=(320.0, 240.0),
        horizontal_error=0.0,
        vertical_error=0.0,
        normalized_radius=0.2,
        marker_confidence=0.9,
    )


def main() -> None:
    """Run and print one successful payload-delivery example."""
    destination = GpsPosition(0.0, 0.0)
    config = LandingConfig(
        target_position=destination,
        takeoff_settle_s=0,
        acquisition_dwell_s=0,
        alignment_dwell_s=0,
        payload_alignment_dwell_s=0,
    )
    targets = MockTargetProvider()
    nav_source = MockNavigationProvider()
    vehicle = MockVehicle()
    machine = LandingStateMachine(targets, vehicle, config, nav_source)
    runner = SimulationRunner(machine, targets, vehicle, nav_source)

    # First point in the 10 m x 10 m grid: 5 m south and 5 m west of centre.
    first_grid_point = GpsPosition(-5.0 / 111_111.0, -5.0 / 111_111.0)
    timeline = runner.run([
        ScenarioStep(0, navigation=navigation(destination)),
        ScenarioStep(1, navigation=navigation(destination)),
        ScenarioStep(2, navigation=navigation(destination)),
        ScenarioStep(3, navigation=navigation(first_grid_point)),
        ScenarioStep(4, target=marker(), navigation=navigation(first_grid_point)),
        ScenarioStep(5, target=marker(), navigation=navigation(first_grid_point)),
        ScenarioStep(6, target=marker(), navigation=navigation(first_grid_point)),
        ScenarioStep(7, target=marker(), navigation=navigation(first_grid_point)),
        ScenarioStep(8, target=marker(), navigation=navigation(first_grid_point)),
    ])

    print(format_timeline(timeline))
    print("\nRecorded requests:")
    for request in vehicle.commands:
        print(request)


if __name__ == "__main__":
    main()
