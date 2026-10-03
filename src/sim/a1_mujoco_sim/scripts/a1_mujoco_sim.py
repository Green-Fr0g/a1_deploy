#!/usr/bin/env python3
"""Launch the Unitree A1 MuJoCo simulator.

Works both from a source checkout (``src/sim/a1_mujoco_sim/scripts``) and from
an installed workspace (``install/lib/a1_mujoco_sim``) by putting the sibling
``a1_mujoco_sim`` package on ``sys.path`` before importing it.
"""
import pathlib
import sys


def _bootstrap() -> None:
    here = pathlib.Path(__file__).resolve().parent
    for base in (here.parent, here):
        if (base / 'a1_mujoco_sim' / '__init__.py').is_file():
            sys.path.insert(0, str(base))
            return
    raise SystemExit('Cannot locate the a1_mujoco_sim package next to this script.')


_bootstrap()

from a1_mujoco_sim.cli import main  # noqa: E402  (imported after sys.path setup)

if __name__ == '__main__':
    sys.exit(main())
