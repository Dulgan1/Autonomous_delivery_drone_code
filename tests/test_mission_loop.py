"""Tests for the real-time mission loop shared by both run modes."""

import unittest

from drone_autonomy.interfaces import VehicleTelemetry
from drone_autonomy.mocks import MockNavigationProvider, MockTargetProvider, MockVehicle
from drone_autonomy.models import LandingState
from drone_autonomy.runtime.loop import MissionRunner
from drone_autonomy.runtime.scenarios import DESTINATION, offset, readings
from drone_autonomy.state_machine import LandingConfig, LandingStateMachine


class FakeClock:
    """A clock the tests advance by hand, so no test ever really sleeps."""

    def __init__(self, step_s: float = 0.1):
        self.now = 0.0
        self.step_s = step_s
        self.slept = 0.0

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.slept += seconds
        self.now += seconds


class MissionLoopTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.targets = MockTargetProvider()
        self.navigation = MockNavigationProvider()
        self.vehicle = MockVehicle()
        self.navigation.readings = readings(offset(DESTINATION, -40.0, 0.0))

    def build(self, **config_changes) -> MissionRunner:
        config = LandingConfig(target_position=DESTINATION, takeoff_settle_s=0.0, **config_changes)
        machine = LandingStateMachine(self.targets, self.vehicle, config, self.navigation)
        return MissionRunner(
            machine,
            rate_hz=10.0,
            terminal_linger_s=0.0,
            clock=self.clock.time,
            sleeper=self.clock.sleep,
        )

    def test_loop_starts_the_mission_and_records_every_cycle(self):
        runner = self.build()
        runner.run(max_cycles=5)
        self.assertEqual(len(runner.log.records), 5)
        self.assertEqual(runner.log.records[0].state, LandingState.TAKEOFF)
        self.assertIn("takeoff", runner.log.records[0].actions)

    def test_loop_holds_the_period_between_cycles(self):
        runner = self.build()
        runner.run(max_cycles=4)
        self.assertAlmostEqual(self.clock.slept, 0.4, places=6)

    def test_manual_override_from_the_override_source_wins(self):
        runner = self.build()
        runner.override_source = lambda: True
        runner.run(max_cycles=3)
        self.assertEqual(runner.machine.state, LandingState.HOLD)
        self.assertTrue(all(record.manual_override for record in runner.log.records))

    def test_loop_stops_once_the_flight_controller_takes_over(self):
        """A battery that falls in flight ends the mission and the loop."""
        runner = self.build()

        def drain_after_takeoff() -> bool:
            if runner.machine.state != LandingState.PREFLIGHT:
                self.vehicle.battery_remaining_percent = 10.0
            return False

        runner.override_source = drain_after_takeoff
        runner.run(max_cycles=50)
        self.assertEqual(runner.machine.state, LandingState.RETURN_HOME)
        self.assertLess(len(runner.log.records), 50)

    def test_low_battery_before_takeoff_holds_on_the_ground(self):
        """Coming home is not a sensible answer while still on the ground."""
        runner = self.build()
        self.vehicle.battery_remaining_percent = 10.0
        runner.run(max_cycles=3)
        self.assertEqual(runner.machine.state, LandingState.HOLD)

    def test_requesting_a_stop_ends_the_loop(self):
        runner = self.build()
        runner.request_stop()
        runner.run(max_cycles=10)
        self.assertEqual(runner.log.records, [])

    def test_a_vehicle_without_a_command_list_still_logs(self):
        """Real adapters do not keep a request list; the loop must cope."""

        class SilentVehicle:
            """Stands in for a real adapter, which keeps no local request list."""

            def telemetry(self):
                return VehicleTelemetry()

            def takeoff(self, altitude_m): pass
            def yaw_to(self, heading_deg): pass
            def forward(self, speed_mps, duration_s): pass
            def release_payload(self): pass
            def return_home(self, reason): pass
            def hold(self, reason): pass
            def velocity(self, setpoint): pass
            def descend(self, rate_mps): pass
            def land(self): pass

        self.vehicle = SilentVehicle()
        runner = self.build()
        runner.run(max_cycles=2)
        self.assertEqual(runner.log.records[0].actions, ())


if __name__ == "__main__":
    unittest.main()
