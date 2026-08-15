import unittest

from drone_autonomy import LandingConfig, LandingState, LandingStateMachine, VisualTarget
from drone_autonomy.mocks import MockTargetProvider, MockVehicle


def target(*, x=0.0, y=0.0, stable=True, visible=True):
    return VisualTarget(1, (320.0, 240.0), x, y, 0.2, 0.8, stable, visible)


class LandingStateMachineTests(unittest.TestCase):
    def setUp(self):
        self.provider = MockTargetProvider()
        self.vehicle = MockVehicle(altitude_m=1.0)
        self.config = LandingConfig(takeoff_settle_s=0, acquisition_dwell_s=0, alignment_dwell_s=0)
        self.machine = LandingStateMachine(self.provider, self.vehicle, self.config)

    def begin_search(self):
        self.machine.start(0)
        self.machine.update(0)
        self.assertEqual(self.machine.state, LandingState.SEARCH)

    def reach_align(self):
        self.begin_search()
        self.provider.target = target(x=0.4, y=-0.2)
        self.machine.update(1)
        self.machine.update(1)
        self.assertEqual(self.machine.state, LandingState.ALIGN)

    def test_normal_target_acquisition_and_alignment(self):
        self.reach_align()
        self.machine.update(2)
        command, setpoint = self.vehicle.commands[-1]
        self.assertEqual(command, "velocity")
        self.assertAlmostEqual(setpoint.image_x, 0.04)
        self.assertAlmostEqual(setpoint.image_y, -0.02)

    def test_centered_alignment_enters_descent(self):
        self.begin_search()
        self.provider.target = target()
        self.machine.update(1)
        self.machine.update(1)
        self.machine.update(1)
        self.assertEqual(self.machine.state, LandingState.DESCEND)

    def test_target_loss_during_descent_holds_then_aborts(self):
        self.begin_search()
        self.provider.target = target()
        self.machine.update(1); self.machine.update(1); self.machine.update(1)
        self.provider.target = None
        self.machine.update(2)
        self.assertEqual(self.machine.state, LandingState.TARGET_LOST)
        self.assertIn(("hold", "target_lost_during_descent"), self.vehicle.commands)
        self.machine.update(5)
        self.assertEqual(self.machine.state, LandingState.ABORT)

    def test_search_timeout_aborts(self):
        self.machine = LandingStateMachine(self.provider, self.vehicle, LandingConfig(takeoff_settle_s=0, search_timeout_s=1))
        self.begin_search()
        self.machine.update(2)
        self.assertEqual(self.machine.state, LandingState.ABORT)
        self.assertIn(("hold", "search_timeout"), self.vehicle.commands)

    def test_manual_override_has_priority(self):
        self.reach_align()
        self.machine.update(2, manual_override=True)
        self.assertEqual(self.machine.state, LandingState.HOLD)
        self.assertIn(("hold", "manual_override"), self.vehicle.commands)

    def test_explicit_abort_holds(self):
        self.reach_align()
        self.machine.update(2, abort=True)
        self.assertEqual(self.machine.state, LandingState.ABORT)
        self.assertIn(("hold", "operator_abort"), self.vehicle.commands)

    def test_unstable_or_missing_target_never_emits_visual_motion(self):
        self.begin_search()
        self.provider.target = target(stable=False)
        self.machine.update(1)
        self.provider.target = target(visible=False)
        self.machine.update(2)
        self.provider.target = None
        self.machine.update(3)
        motion = [name for name, _ in self.vehicle.commands if name in {"velocity", "descend"}]
        self.assertEqual(motion, [])

    def test_velocity_is_rate_limited(self):
        self.reach_align()
        self.machine.config = LandingConfig(
            takeoff_settle_s=0,
            acquisition_dwell_s=0,
            alignment_dwell_s=0,
            lateral_gain=1.0,
        )
        self.provider.target = target(x=1.0, y=-1.0)
        self.machine.update(2)
        _, setpoint = self.vehicle.commands[-1]
        self.assertEqual(setpoint.image_x, self.config.max_image_speed)
        self.assertEqual(setpoint.image_y, -self.config.max_image_speed)


if __name__ == "__main__":
    unittest.main()
