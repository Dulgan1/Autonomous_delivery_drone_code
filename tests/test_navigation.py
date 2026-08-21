import unittest

from drone_autonomy import GpsPosition, LandingConfig, LandingState, LandingStateMachine, NavigationReadings, VisualTarget
from drone_autonomy.mocks import MockNavigationProvider, MockTargetProvider, MockVehicle
from drone_autonomy.state_machine import ALLOWED_TRANSITIONS


def readings(*, latitude=0.0, longitude=0.0, heading=0.0, side=5.0, top=5.0, bottom=1.0, fresh=True):
    return NavigationReadings(
        position=GpsPosition(latitude, longitude),
        heading_deg=heading,
        side_distance_m=side,
        top_distance_m=top,
        bottom_distance_m=bottom,
        gps_fresh=fresh,
        heading_fresh=fresh,
        side_fresh=fresh,
        top_fresh=fresh,
        bottom_fresh=fresh,
    )


class GpsNavigationTests(unittest.TestCase):
    def setUp(self):
        self.targets = MockTargetProvider()
        self.navigation = MockNavigationProvider(readings())
        self.vehicle = MockVehicle()
        self.config = LandingConfig(
            target_position=GpsPosition(0.001, 0.0),
            takeoff_settle_s=0,
            arrival_radius_m=3,
        )
        self.machine = LandingStateMachine(self.targets, self.vehicle, self.config, self.navigation)

    def start_navigation(self):
        self.machine.start(0)
        self.machine.update(0)  # PREFLIGHT -> TAKEOFF
        self.machine.update(1)  # TAKEOFF -> GPS_NAVIGATE
        self.assertEqual(self.machine.state, LandingState.GPS_NAVIGATE)

    def test_clear_path_requests_one_bounded_forward_step(self):
        self.start_navigation()
        self.machine.update(2)
        self.assertIn(("forward", (self.config.forward_speed_mps, self.config.forward_step_s)), self.vehicle.commands)

    def test_arrival_starts_grid_search(self):
        self.start_navigation()
        self.navigation.readings = readings(latitude=0.001)
        self.machine.update(2)
        self.assertEqual(self.machine.state, LandingState.SEARCH_MOVE)
        self.assertEqual(self.machine.transitions[-1][2], "gps_target_reached")

    def test_blocked_path_scans_then_uses_clear_yaw_direction(self):
        self.start_navigation()
        self.navigation.readings = readings(side=1.0)
        self.machine.update(2)
        self.assertEqual(self.machine.state, LandingState.YAW_SCAN)
        self.machine.update(3)  # target direction is still blocked
        self.navigation.readings = readings(heading=330.0, side=5.0)
        self.machine.update(4)  # -30 degree candidate is clear
        self.assertEqual(self.machine.state, LandingState.GPS_NAVIGATE)
        self.assertEqual(self.machine.transitions[-1][2], "clear_scan_direction")
        self.machine.update(5)
        self.assertIn(("forward", (self.config.forward_speed_mps, self.config.forward_step_s)), self.vehicle.commands)

    def test_missing_gps_or_heading_holds(self):
        self.start_navigation()
        self.navigation.readings = readings(fresh=False)
        self.machine.update(2)
        self.assertEqual(self.machine.state, LandingState.HOLD)
        self.assertEqual(self.machine.transitions[-1][2], "navigation_unavailable")

    def test_no_clear_scan_direction_holds(self):
        config = LandingConfig(
            target_position=GpsPosition(0.001, 0.0),
            takeoff_settle_s=0,
            yaw_scan_offsets_deg=(0.0,),
        )
        machine = LandingStateMachine(self.targets, self.vehicle, config, self.navigation)
        machine.start(0)
        machine.update(0)
        machine.update(1)
        self.navigation.readings = readings(side=1.0)
        machine.update(2)
        machine.update(3)
        machine.update(4)
        self.assertEqual(machine.state, LandingState.RETURN_HOME)
        self.assertEqual(machine.transitions[-1][2], "no_clear_scan_direction")

    def test_top_sensor_blocks_preflight(self):
        self.navigation.readings = readings(top=0.2)
        self.machine.start(0)
        self.machine.update(0)
        self.assertEqual(self.machine.state, LandingState.HOLD)
        self.assertEqual(self.machine.transitions[-1][2], "preflight_top_blocked")

    def test_missing_bottom_sensor_holds_during_descent(self):
        self.machine.start(0)
        self.machine.state = LandingState.DESCEND
        self.machine.state_since_s = 0
        self.targets.target = VisualTarget(1, (0.0, 0.0), 0.0, 0.0, 0.1, 0.8)
        self.navigation.readings = NavigationReadings()
        self.machine.update(1)
        self.assertEqual(self.machine.state, LandingState.HOLD)
        self.assertEqual(self.machine.transitions[-1][2], "bottom_sensor_unavailable")


if __name__ == "__main__":
    unittest.main()


class ReturnHomeReachabilityTests(unittest.TestCase):
    """Any state that can still decide must be able to reach RETURN_HOME.

    A mission health check that raises instead of acting is worse than one that
    acts imperfectly, so this is checked directly rather than only through the
    scenarios that happen to hit each path.
    """

    def test_every_deciding_state_may_return_home(self):
        deciding = set(ALLOWED_TRANSITIONS) - {
            LandingState.HOLD,
            LandingState.ABORT,
            LandingState.RETURN_HOME,
        }
        for state in deciding:
            with self.subTest(state=state.value):
                self.assertIn(LandingState.RETURN_HOME, ALLOWED_TRANSITIONS[state])

    def test_losing_the_marker_before_the_drop_comes_home_instead_of_raising(self):
        config = LandingConfig(
            target_position=GpsPosition(0.0, 0.0),
            target_reacquire_timeout_s=1.0,
        )
        targets = MockTargetProvider()
        vehicle = MockVehicle()
        navigation = MockNavigationProvider()
        machine = LandingStateMachine(targets, vehicle, config, navigation)
        machine.state = LandingState.TARGET_LOST
        machine.state_since_s = 0.0

        machine.update(5.0)

        self.assertEqual(machine.state, LandingState.RETURN_HOME)
        self.assertIn(("return_home", "target_reacquire_timeout"), vehicle.commands)
