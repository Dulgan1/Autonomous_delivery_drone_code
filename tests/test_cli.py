"""Tests for the command line that chooses a run mode."""

import contextlib
import io
import unittest

from drone_autonomy.__main__ import build_parser, main


def run_quietly(argv: list[str]) -> int:
    """Run the command line with its output captured.

    Args:
        argv: Arguments to pass.

    Returns:
        The exit code the command line returned.
    """
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return main(argv)


class CliTests(unittest.TestCase):
    def test_listing_scenarios_succeeds(self):
        self.assertEqual(run_quietly(["sim", "--list"]), 0)

    def test_running_one_named_scenario_succeeds(self):
        self.assertEqual(run_quietly(["sim", "nominal"]), 0)

    def test_an_unknown_scenario_is_reported(self):
        with self.assertRaises(KeyError):
            run_quietly(["sim", "no_such_flight"])

    def test_a_real_flight_needs_an_explicit_acknowledgement(self):
        self.assertEqual(run_quietly(["fly", "--config", "config/mission.example.toml"]), 2)

    def test_a_mode_must_be_chosen(self):
        with self.assertRaises(SystemExit):
            build_parser().parse_args([])


if __name__ == "__main__":
    unittest.main()
