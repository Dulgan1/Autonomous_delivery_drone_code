import unittest

from drone_autonomy import GpsPosition, LandingConfig, LandingState, LandingStateMachine, NavigationReadings, VisualTarget
from drone_autonomy.mocks import MockNavigationProvider, MockTargetProvider, MockVehicle


def nav(*, position=GpsPosition(0.0, 0.0), heading=0.0, side=5.0):
    return NavigationReadings(
        position=position, heading_deg=heading, side_distance_m=side,
        bottom_distance_m=5.0, top_distance_m=5.0,
        gps_fresh=True, heading_fresh=True, side_fresh=True,
        bottom_fresh=True, top_fresh=True,
    )


def marker(*, x=0.0, y=0.0):
    return VisualTarget(1, (320.0, 240.0), x, y, 0.2, 0.9)


class PayloadMissionTests(unittest.TestCase):
    def setUp(self):
        self.targets = MockTargetProvider()
        self.navigation = MockNavigationProvider(nav())
        self.vehicle = MockVehicle()
        self.config = LandingConfig(
            target_position=GpsPosition(0.0, 0.0),
            takeoff_settle_s=0,
            acquisition_dwell_s=0,
            alignment_dwell_s=0,
            payload_alignment_dwell_s=0,
            search_point_dwell_s=0,
        )
        self.machine = LandingStateMachine(self.targets, self.vehicle, self.config, self.navigation)

    def reach_search_move(self):
        self.machine.start(0)
        self.machine.update(0)
        self.machine.update(1)
        self.machine.update(2)
        self.assertEqual(self.machine.state, LandingState.SEARCH_MOVE)

    def test_grid_has_serpentine_points_over_configured_area(self):
        points = self.machine._make_search_grid(GpsPosition(0.0, 0.0))
        self.assertEqual(len(points), 36)  # 6 points each way: -5, -3, -1, 1, 3, 5
        self.assertAlmostEqual(points[0].longitude_deg, -5 / 111_111, places=7)
        self.assertAlmostEqual(points[6].longitude_deg, 5 / 111_111, places=7)

    def test_centered_marker_releases_once_then_returns_home(self):
        self.reach_search_move()
        self.targets.target = marker()
        self.machine.update(3)
        self.machine.update(3)
        self.machine.update(3)
        self.assertEqual(self.machine.state, LandingState.DROP_READY)
        self.machine.update(3)
        self.assertEqual(self.machine.state, LandingState.DROP_PAYLOAD)
        self.machine.update(3)
        self.assertEqual(self.machine.state, LandingState.RETURN_HOME)
        self.assertEqual([name for name, _ in self.vehicle.commands].count("release_payload"), 1)
        self.assertIn(("return_home", "payload_release_requested"), self.vehicle.commands)

    def test_low_battery_returns_home(self):
        self.reach_search_move()
        self.vehicle.battery_remaining_percent = 40.0
        self.machine.update(3)
        self.assertEqual(self.machine.state, LandingState.RETURN_HOME)
        self.assertEqual(self.machine.transitions[-1][2], "battery_low")

    def test_search_completion_returns_home(self):
        self.reach_search_move()
        self.machine._search_points = [self.machine._active_navigation_goal]
        self.machine.state = LandingState.SEARCH
        self.machine.state_since_s = 0
        self.machine.update(1)
        self.assertEqual(self.machine.state, LandingState.RETURN_HOME)
        self.assertEqual(self.machine.transitions[-1][2], "marker_not_found_in_search_area")


if __name__ == "__main__":
    unittest.main()
