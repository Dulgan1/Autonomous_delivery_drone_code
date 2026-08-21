"""Every scripted fake flight must keep ending the way it is documented to.

This is the regression net for the mission logic: if a change makes a safety
path end somewhere else, one of these fails.
"""

import unittest

from drone_autonomy.runtime.scenarios import all_scenarios
from drone_autonomy.runtime.sim_mode import run_scenario


class ScenarioTests(unittest.TestCase):
    def test_every_scenario_ends_where_it_says_it_does(self):
        for scenario in all_scenarios():
            with self.subTest(scenario=scenario.name):
                _, _, final_state = run_scenario(scenario)
                self.assertEqual(final_state, scenario.expected_final_state)

    def test_scenario_names_are_unique(self):
        names = [scenario.name for scenario in all_scenarios()]
        self.assertEqual(len(names), len(set(names)))

    def test_only_the_nominal_delivery_ever_releases_the_payload(self):
        for scenario in all_scenarios():
            with self.subTest(scenario=scenario.name):
                _, commands, _ = run_scenario(scenario)
                released = [name for name, _ in commands if name == "release_payload"]
                expected = 1 if scenario.name == "nominal" else 0
                self.assertEqual(len(released), expected)

    def test_no_scenario_requests_movement_after_manual_override(self):
        movement = {"takeoff", "forward", "yaw_to", "velocity", "descend", "land", "release_payload"}
        for scenario in all_scenarios():
            if not any(step.manual_override for step in scenario.steps):
                continue
            with self.subTest(scenario=scenario.name):
                _, commands, _ = run_scenario(scenario)
                self.assertEqual(commands[-1][0], "hold")
                self.assertNotIn(commands[-1][0], movement)


if __name__ == "__main__":
    unittest.main()
