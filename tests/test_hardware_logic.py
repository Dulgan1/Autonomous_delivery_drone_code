"""Tests for the decoding, filtering, and encoding rules inside the adapters.

These run on any computer. They deliberately exercise the parts that decide
whether a reading may be trusted, because that is where a wrong answer would
let a broken sensor look healthy.
"""

import json
import unittest

from drone_autonomy.hardware.cv_link import MessageRejected, message_age_s, parse_target_message
from drone_autonomy.hardware.px4 import (
    PX4_AUTO_SUB_MODE,
    PX4_MAIN_MODE,
    PILOT_MODES,
    Setpoint,
    TelemetryTracker,
    decode_mode_name,
    encode_custom_mode,
)
from drone_autonomy.hardware.ultrasonic import RangeFilter, echo_to_distance_m


def heartbeat(mode: int, *, armed: bool = True, status: int = 4) -> tuple[str, dict]:
    """Return a fake HEARTBEAT message for the tracker.

    Args:
        mode: Packed custom mode.
        armed: Whether the armed bit is set.
        status: MAV_STATE value.
    """
    return "HEARTBEAT", {"base_mode": (128 if armed else 0) | 1, "custom_mode": mode, "system_status": status}


class Px4ModeTests(unittest.TestCase):
    def test_modes_survive_a_round_trip(self):
        for main, sub in (("OFFBOARD", None), ("AUTO", "RTL"), ("AUTO", "LAND"), ("POSCTL", None)):
            packed = encode_custom_mode(PX4_MAIN_MODE[main], PX4_AUTO_SUB_MODE[sub] if sub else 0)
            expected = f"AUTO.{sub}" if sub else main
            self.assertEqual(decode_mode_name(packed), expected)

    def test_an_unknown_mode_is_named_rather_than_guessed(self):
        self.assertIn("UNKNOWN", decode_mode_name(0x00FF0000))

    def test_pilot_flown_modes_are_recognised(self):
        self.assertIn("POSCTL", PILOT_MODES)
        self.assertNotIn("OFFBOARD", PILOT_MODES)


class TelemetryTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = TelemetryTracker(stale_after_s=2.0, minimum_satellites=8, maximum_eph_m=5.0)
        self.offboard = encode_custom_mode(PX4_MAIN_MODE["OFFBOARD"])
        self.feed(0.0)

    def feed(self, now_s: float, *, fix: int = 3, satellites: int = 12, eph: int = 120, mode=None, status: int = 4):
        """Push one healthy round of messages into the tracker."""
        name, fields = heartbeat(mode if mode is not None else self.offboard, status=status)
        self.tracker.ingest(name, fields, now_s)
        self.tracker.ingest("GLOBAL_POSITION_INT", {"lat": 90_000_000, "lon": 74_000_000, "relative_alt": 5000, "hdg": 9000}, now_s)
        self.tracker.ingest("GPS_RAW_INT", {"fix_type": fix, "satellites_visible": satellites, "eph": eph}, now_s)
        self.tracker.ingest("BATTERY_STATUS", {"battery_remaining": 77}, now_s)

    def test_healthy_telemetry_is_reported_as_usable(self):
        snapshot = self.tracker.snapshot(0.5, expected_mode="OFFBOARD", faulted=False)
        self.assertTrue(snapshot.telemetry_fresh)
        self.assertTrue(snapshot.position_hold_ready)
        self.assertTrue(snapshot.armed)
        self.assertFalse(snapshot.failsafe_active)
        self.assertEqual(snapshot.battery_remaining_percent, 77.0)
        self.assertEqual(self.tracker.heading_deg, 90.0)
        self.assertEqual(self.tracker.altitude_m, 5.0)

    def test_a_silent_link_goes_stale_rather_than_reporting_old_values(self):
        self.assertFalse(self.tracker.snapshot(30.0, expected_mode=None, faulted=False).telemetry_fresh)

    def test_a_poor_gps_fix_is_not_position_hold_ready(self):
        self.feed(1.0, fix=2)
        self.assertFalse(self.tracker.position_hold_ready(1.0))

    def test_too_few_satellites_is_not_position_hold_ready(self):
        self.feed(1.0, satellites=4)
        self.assertFalse(self.tracker.position_hold_ready(1.0))

    def test_poor_reported_accuracy_is_not_position_hold_ready(self):
        self.feed(1.0, eph=2000)
        self.assertFalse(self.tracker.position_hold_ready(1.0))

    def test_unknown_heading_is_reported_as_missing(self):
        self.tracker.ingest("GLOBAL_POSITION_INT", {"lat": 0, "lon": 0, "relative_alt": 0, "hdg": 65535}, 1.0)
        self.assertIsNone(self.tracker.heading_deg)

    def test_a_mode_change_away_from_ours_reads_as_a_takeover(self):
        self.feed(1.0, mode=encode_custom_mode(PX4_MAIN_MODE["POSCTL"]))
        snapshot = self.tracker.snapshot(1.0, expected_mode="OFFBOARD", faulted=False)
        self.assertTrue(snapshot.failsafe_active)

    def test_a_critical_system_status_reads_as_a_failsafe(self):
        self.feed(1.0, status=5)
        self.assertTrue(self.tracker.snapshot(1.0, expected_mode=None, faulted=False).failsafe_active)

    def test_an_adapter_fault_reads_as_a_failsafe(self):
        self.assertTrue(self.tracker.snapshot(0.5, expected_mode="OFFBOARD", faulted=True).failsafe_active)


class SetpointTests(unittest.TestCase):
    def test_a_short_step_becomes_a_hover_when_it_expires(self):
        step = Setpoint(forward_mps=0.25, yaw_deg=90.0, expires_at_s=10.0)
        self.assertEqual(step.at(9.9).forward_mps, 0.25)
        self.assertEqual(step.at(10.0).forward_mps, 0.0)
        self.assertEqual(step.at(10.0).yaw_deg, 90.0)

    def test_a_setpoint_without_an_expiry_persists(self):
        held = Setpoint(yaw_deg=45.0)
        self.assertEqual(held.at(1e6).yaw_deg, 45.0)


class RangeFilterTests(unittest.TestCase):
    def test_echo_time_converts_to_distance(self):
        self.assertAlmostEqual(echo_to_distance_m(0.00583), 1.0, places=2)

    def test_out_of_range_and_missing_readings_are_dropped(self):
        sensor = RangeFilter("side", min_range_m=0.03, max_range_m=4.0)
        for value in (None, 0.01, 9.0):
            sensor.submit(value, 0.0)
        self.assertIsNone(sensor.value())
        self.assertFalse(sensor.fresh(0.0))
        self.assertEqual(sensor.rejected, 3)

    def test_an_echo_timeout_never_becomes_a_clear_reading(self):
        sensor = RangeFilter("side")
        sensor.submit(0.5, 0.0)
        sensor.submit(None, 0.1)
        self.assertEqual(sensor.value(), 0.5)

    def test_the_median_ignores_a_single_noisy_reading(self):
        sensor = RangeFilter("bottom", window=5)
        for index, value in enumerate([2.0, 2.1, 0.05, 2.05, 2.0]):
            sensor.submit(value, index * 0.05)
        self.assertAlmostEqual(sensor.value(), 2.0, places=2)

    def test_a_reading_goes_stale_when_updates_stop(self):
        sensor = RangeFilter("top", stale_after_s=0.5)
        sensor.submit(1.0, 0.0)
        self.assertTrue(sensor.fresh(0.4))
        self.assertFalse(sensor.fresh(1.0))

    def test_a_frozen_sensor_is_treated_as_failed(self):
        sensor = RangeFilter("side", stuck_sample_limit=3)
        for index in range(6):
            sensor.submit(1.5, index * 0.05)
        self.assertTrue(sensor.stuck)
        self.assertFalse(sensor.fresh(0.3))

    def test_a_genuinely_varying_sensor_is_not_called_stuck(self):
        sensor = RangeFilter("side", stuck_sample_limit=3)
        for index in range(20):
            sensor.submit(1.5 + index * 0.01, index * 0.05)
        self.assertFalse(sensor.stuck)


class CvMessageTests(unittest.TestCase):
    def message(self, **changes) -> str:
        """Return a valid perception message with optional changes applied."""
        payload = {
            "track_id": 1,
            "target_point": [320.0, 240.0],
            "horizontal_error": 0.1,
            "vertical_error": -0.2,
            "normalized_radius": 0.18,
            "marker_confidence": 0.8,
            "stable": True,
            "visible": True,
        }
        payload.update(changes)
        return json.dumps(payload)

    def test_a_good_message_becomes_a_usable_target(self):
        target = parse_target_message(self.message())
        self.assertTrue(target.usable)
        self.assertEqual(target.target_point, (320.0, 240.0))

    def test_a_held_track_is_parsed_but_not_usable(self):
        self.assertFalse(parse_target_message(self.message(visible=False)).usable)

    def test_missing_stability_flags_default_to_not_usable(self):
        payload = json.loads(self.message())
        del payload["stable"], payload["visible"]
        self.assertFalse(parse_target_message(json.dumps(payload)).usable)

    def test_malformed_messages_are_rejected_rather_than_half_believed(self):
        for bad in (b"not json", b"[]", b'{"track_id": 1}', self.message(target_point=[1.0])):
            with self.assertRaises(MessageRejected):
                parse_target_message(bad)

    def test_out_of_range_errors_are_parsed_but_not_usable(self):
        self.assertFalse(parse_target_message(self.message(horizontal_error=5.0)).usable)

    def test_the_senders_timestamp_is_preferred_for_age(self):
        self.assertAlmostEqual(message_age_s({"sent_at_s": 100.0}, 100.4, 100.9), 0.9, places=6)

    def test_arrival_time_is_used_when_no_timestamp_is_sent(self):
        self.assertAlmostEqual(message_age_s({}, 100.4, 100.9), 0.5, places=6)


if __name__ == "__main__":
    unittest.main()
