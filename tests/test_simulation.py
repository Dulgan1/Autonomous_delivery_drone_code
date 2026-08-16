import unittest

from drone_autonomy import LandingConfig, LandingState, LandingStateMachine, ScenarioStep, SimulationRunner, VisualTarget
from drone_autonomy.mocks import MockTargetProvider, MockVehicle
from drone_autonomy.simulation import format_timeline


def target(*, stable=True, visible=True):
    return VisualTarget(1, (320.0, 240.0), 0.0, 0.0, 0.2, 0.8, stable, visible)


class SimulationRunnerTests(unittest.TestCase):
    def setUp(self):
        self.targets = MockTargetProvider()
        self.vehicle = MockVehicle()
        config = LandingConfig(takeoff_settle_s=0, acquisition_dwell_s=0, alignment_dwell_s=0)
        machine = LandingStateMachine(self.targets, self.vehicle, config)
        self.runner = SimulationRunner(machine, self.targets, self.vehicle)

    def test_normal_landing_timeline(self):
        records = self.runner.run(
            [
                ScenarioStep(0, altitude_m=1.0),
                ScenarioStep(1, target(), altitude_m=1.0),
                ScenarioStep(2, target(), altitude_m=1.0),
                ScenarioStep(3, target(), altitude_m=1.0),
                ScenarioStep(4, target(), altitude_m=0.2),
            ]
        )
        self.assertEqual(
            [record.state for record in records],
            [
                LandingState.SEARCH,
                LandingState.TARGET_ACQUIRED,
                LandingState.ALIGN,
                LandingState.DESCEND,
                LandingState.LAND,
            ],
        )
        self.assertEqual(records[-1].transition_reason, "low_altitude_centered")
        self.assertIn("4.0 | land | low_altitude_centered | land", format_timeline(records))

    def test_lost_target_and_manual_override_are_visible_in_timeline(self):
        records = self.runner.run(
            [
                ScenarioStep(0, altitude_m=1.0),
                ScenarioStep(1, target(), altitude_m=1.0),
                ScenarioStep(2, target(), altitude_m=1.0),
                ScenarioStep(3, target(), altitude_m=1.0),
                ScenarioStep(4, altitude_m=1.0),
                ScenarioStep(5, manual_override=True, altitude_m=1.0),
            ]
        )
        self.assertEqual(records[4].state, LandingState.TARGET_LOST)
        self.assertEqual(records[4].transition_reason, "target_lost_during_descent")
        self.assertEqual(records[5].state, LandingState.HOLD)
        self.assertEqual(records[5].transition_reason, "manual_override")

    def test_empty_scenario_does_nothing(self):
        self.assertEqual(self.runner.run([]), [])


if __name__ == "__main__":
    unittest.main()
