"""Simulation mode: run the mission on a laptop with fake inputs.

Nothing in this module imports pymavlink, pigpio, or any hardware library, so
it runs anywhere Python 3.11 runs. Every request the mission makes is recorded
in a list and thrown away.
"""

from ..mocks import MockNavigationProvider, MockTargetProvider, MockVehicle
from ..simulation import SimulationRunner, format_timeline
from ..state_machine import LandingStateMachine
from .scenarios import Scenario, all_scenarios, get_scenario


def run_scenario(scenario: Scenario) -> tuple[str, list[tuple[str, object]], str]:
    """Play one fake flight and return its timeline, requests, and final state.

    Args:
        scenario: The fake flight to play.

    Returns:
        A ready-to-print timeline table, the recorded vehicle requests, and the
        state value the flight ended in.
    """
    targets = MockTargetProvider()
    navigation = MockNavigationProvider()
    vehicle = MockVehicle()
    machine = LandingStateMachine(targets, vehicle, scenario.config, navigation)
    records = SimulationRunner(machine, targets, vehicle, navigation).run(scenario.steps)
    return format_timeline(records), vehicle.commands, machine.state.value


def print_scenario(scenario: Scenario) -> bool:
    """Play one fake flight, print it, and say whether it ended as expected.

    Args:
        scenario: The fake flight to play.

    Returns:
        True when the flight ended in its expected state.
    """
    timeline, commands, final_state = run_scenario(scenario)
    passed = final_state == scenario.expected_final_state
    print(f"\n=== {scenario.name} ===")
    print(scenario.description)
    print()
    print(timeline)
    print("\nRecorded requests:")
    for command in commands:
        print(f"  {command}")
    verdict = "as expected" if passed else f"UNEXPECTED, wanted {scenario.expected_final_state}"
    print(f"\nfinal state: {final_state} ({verdict})")
    return passed


def list_scenarios() -> None:
    """Print the name and purpose of every available fake flight."""
    print("Available simulation scenarios:\n")
    width = max(len(scenario.name) for scenario in all_scenarios())
    for scenario in all_scenarios():
        print(f"  {scenario.name.ljust(width)}  {scenario.description}")


def main(names: list[str] | None) -> int:
    """Run the requested fake flights and report a pass or fail summary.

    Args:
        names: Scenario names to run, or None to run every one of them.

    Returns:
        Process exit code: 0 when every flight ended as expected.
    """
    scenarios = all_scenarios() if not names else [get_scenario(name) for name in names]
    results = [(scenario.name, print_scenario(scenario)) for scenario in scenarios]
    failed = [name for name, passed in results if not passed]
    print(f"\n{len(results) - len(failed)} of {len(results)} scenarios ended as expected.")
    if failed:
        print("unexpected endings: " + ", ".join(failed))
    return 1 if failed else 0
