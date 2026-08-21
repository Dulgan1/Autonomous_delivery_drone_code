"""Tests for loading and checking the run-mode configuration."""

import tempfile
import unittest
from pathlib import Path

from drone_autonomy.runtime.config import RuntimeConfig, load_config, with_mission


def write_config(text: str) -> str:
    """Write a temporary TOML file and return its path.

    Args:
        text: File contents.
    """
    handle = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False, encoding="utf-8")
    handle.write(text)
    handle.close()
    return handle.name


class ConfigTests(unittest.TestCase):
    def test_defaults_are_safe_without_a_file(self):
        config = load_config(None)
        self.assertIsNone(config.mission.target_position)
        self.assertFalse(config.mavlink.allow_arm)
        self.assertFalse(config.mavlink.allow_image_guidance)
        self.assertFalse(config.servo.enabled)
        self.assertTrue(config.mission.require_armed)

    def test_example_config_loads(self):
        example = Path(__file__).resolve().parents[1] / "config" / "mission.example.toml"
        config = load_config(example)
        self.assertIsNotNone(config.mission.target_position)
        self.assertEqual(config.mission.search_area_side_m, 10.0)
        self.assertEqual(config.ultrasonic.side.trigger_pin, 17)
        self.assertEqual(config.mission.yaw_scan_offsets_deg, (0.0, -30.0, 30.0, -60.0, 60.0))

    def test_shipped_example_keeps_every_interlock_off(self):
        example = Path(__file__).resolve().parents[1] / "config" / "mission.example.toml"
        config = load_config(example)
        self.assertFalse(config.servo.enabled)
        self.assertFalse(config.mavlink.allow_arm)
        self.assertFalse(config.mavlink.allow_image_guidance)

    def test_half_a_target_is_refused(self):
        path = write_config("[mission]\ntarget_latitude_deg = 9.0\n")
        with self.assertRaises(ValueError):
            load_config(path)

    def test_out_of_range_target_is_refused(self):
        path = write_config("[mission]\ntarget_latitude_deg = 991.0\ntarget_longitude_deg = 7.0\n")
        with self.assertRaises(ValueError):
            load_config(path)

    def test_misspelled_setting_is_refused_rather_than_ignored(self):
        path = write_config("[mission]\nside_clearence_m = 2.0\n")
        with self.assertRaises(ValueError) as caught:
            load_config(path)
        self.assertIn("side_clearence_m", str(caught.exception))

    def test_with_mission_copies_rather_than_mutates(self):
        config = RuntimeConfig()
        changed = with_mission(config, side_clearance_m=9.0)
        self.assertEqual(changed.mission.side_clearance_m, 9.0)
        self.assertNotEqual(config.mission.side_clearance_m, 9.0)


if __name__ == "__main__":
    unittest.main()
