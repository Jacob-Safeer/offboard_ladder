"""Console script shim for the circle offboard mode.

This keeps `circle_offboard.py` untouched (so it can continue to be executed
directly) while still exposing a `main` callable for the ROS2 entry point.
"""

from importlib import import_module
import runpy
from typing import Any


def main(args: Any = None) -> None:
    """Invoke the original circle_offboard module similar to `python -m`."""
    module = import_module('px4_offboard.circle_offboard')

    if hasattr(module, 'main'):
        module.main(args=args)
        return

    runpy.run_module('px4_offboard.circle_offboard', run_name='__main__')
