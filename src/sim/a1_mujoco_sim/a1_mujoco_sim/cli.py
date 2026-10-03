"""Command line entry point for the A1 MuJoCo simulator."""
from __future__ import annotations

import argparse
import sys

from .app import SimulatorApp, ViewerError
from .config import load_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='a1_mujoco_sim',
        description='Run the Unitree A1 MuJoCo ROS 2 simulator.')
    parser.add_argument(
        '-c', '--camera', action='store_true',
        help='make the viewer follow the robot (the viewer opens either way)')
    parser.add_argument(
        '-d', '--depth', action='store_true',
        help='open the depth camera visualization window')
    parser.add_argument(
        '--headless', action='store_true',
        help='do not open the MuJoCo viewer (servers without a display)')
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.headless and args.camera:
        parser.error('--camera cannot be combined with --headless')

    try:
        config = load_config()
    except (OSError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 2

    app = SimulatorApp(config, show_viewer=not args.headless,
                       follow_camera=args.camera, show_depth=args.depth)
    try:
        return app.run()
    except KeyboardInterrupt:
        return 0
    except ViewerError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
