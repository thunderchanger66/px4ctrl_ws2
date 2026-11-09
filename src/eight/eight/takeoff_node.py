"""Run the takeoff and Offboard handoff phase without starting a trajectory."""

from .eight_node import main as run_eight


def main(args=None) -> None:
    run_eight(args=args, phase="takeoff")


if __name__ == "__main__":
    main()
