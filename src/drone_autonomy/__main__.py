"""Command line for both run modes.

Simulation on a laptop::

    python -m drone_autonomy sim --list
    python -m drone_autonomy sim
    python -m drone_autonomy sim nominal blocked_path

On the Raspberry Pi, read-only first, then the real mission::

    python -m drone_autonomy check --config config/mission.toml
    python -m drone_autonomy fly --config config/mission.toml
"""

import argparse
import sys

from .runtime.config import load_config


def build_parser() -> argparse.ArgumentParser:
    """Return the argument parser for every run mode."""
    parser = argparse.ArgumentParser(
        prog="drone_autonomy",
        description="Drone delivery autonomy: scripted simulation, or a real flight on a Raspberry Pi.",
    )
    modes = parser.add_subparsers(dest="mode", required=True)

    simulate = modes.add_parser("sim", help="run scripted fake flights on any computer")
    simulate.add_argument("scenarios", nargs="*", help="scenario names; default is all of them")
    simulate.add_argument("--list", action="store_true", help="list the scenarios and exit")

    inspect = modes.add_parser("check", help="Pi: read sensors and telemetry, command nothing")
    inspect.add_argument("--config", required=True, help="TOML configuration file")
    inspect.add_argument("--seconds", type=float, default=30.0, help="how long to keep reading")

    flight = modes.add_parser("fly", help="Pi: run the real delivery mission")
    flight.add_argument("--config", required=True, help="TOML configuration file")
    flight.add_argument(
        "--i-have-completed-bench-testing",
        action="store_true",
        help="required acknowledgement that props-off and read-only checks already passed",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Parse the command line and run the requested mode.

    Args:
        argv: Arguments to parse, or None to use the real command line.

    Returns:
        Process exit code.
    """
    args = build_parser().parse_args(argv)

    if args.mode == "sim":
        from .runtime import sim_mode

        if args.list:
            sim_mode.list_scenarios()
            return 0
        return sim_mode.main(args.scenarios)

    config = load_config(args.config)
    try:
        from .runtime import hardware_mode
    except ImportError as error:  # pragma: no cover - depends on the machine
        print(_missing_hardware_message(error), file=sys.stderr)
        return 3

    if args.mode == "check":
        try:
            return hardware_mode.check(config, seconds=args.seconds)
        except ImportError as error:
            print(_missing_hardware_message(error), file=sys.stderr)
            return 3
        except KeyboardInterrupt:
            print("\ncheck stopped.")
            return 0

    if not args.i_have_completed_bench_testing:  # noqa: SIM102 - kept explicit on purpose
        print(
            "Refusing to start a real flight.\n\n"
            "Run the read-only check first:\n"
            "    python -m drone_autonomy check --config <file>\n\n"
            "Then, once props-off bench testing and every failsafe test have passed, "
            "repeat this command with --i-have-completed-bench-testing.",
            file=sys.stderr,
        )
        return 2
    try:
        return 0 if hardware_mode.fly(config).value in ("return_home", "land") else 1
    except ImportError as error:
        print(_missing_hardware_message(error), file=sys.stderr)
        return 3


def _missing_hardware_message(error: ImportError) -> str:
    """Explain how to install a missing hardware library.

    Args:
        error: The import failure.

    Returns:
        A message naming the missing library and how to install it. This runs
        on a laptop as often as on a Pi, since hardware mode is the only thing
        that needs these libraries at all.
    """
    hints = {
        "pymavlink": "pip install 'pymavlink>=2.4.37'",
        "pigpio": "sudo apt install pigpio && sudo systemctl enable --now pigpiod && pip install pigpio",
    }
    missing = getattr(error, "name", None) or "a hardware library"
    hint = hints.get(missing, "pip install -e '.[hardware]'")
    return (
        f"Hardware mode needs {missing}, which is not installed here.\n\n"
        f"On the Raspberry Pi:\n    {hint}\n\n"
        "Simulation mode needs none of this:\n"
        "    python -m drone_autonomy sim"
    )


if __name__ == "__main__":
    raise SystemExit(main())
