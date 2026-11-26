"""Read the straight pieces of a drawn SVG path, for the tests of the figure scripts.

A test module imports Segment and path_segments() after it has put this folder on sys.path. The
name of this file doesn't start with test_, so the test discovery doesn't collect it.
"""

import re
from typing import List
from typing import NamedTuple

# A command of a path with the numbers after it
COMMAND = re.compile(r'([MLHVZ])([^MLHVZ]*)')


class Segment(NamedTuple):
    """A straight piece of a drawn line, from (x1, y1) to (x2, y2)."""

    x1: float
    y1: float
    x2: float
    y2: float


def path_segments(path: str) -> List[Segment]:
    """Return the straight pieces of an SVG path made of absolute M, L, H, V and Z commands.

    Z closes a part of the path with a piece back to the point of its M command. The letter of
    any other command, such as a curve or a relative command, stands among the numbers, which
    raises a ValueError.
    """
    segments: List[Segment] = []
    x = y = start_x = start_y = 0.0
    for command, numbers in COMMAND.findall(path):
        values = [float(value) for value in re.split(r'[ ,]+', numbers.strip()) if value]
        if command == 'M':
            x, y = start_x, start_y = values[0], values[1]
            continue
        if command == 'H':
            end_x, end_y = values[0], y
        elif command == 'V':
            end_x, end_y = x, values[0]
        elif command == 'L':
            end_x, end_y = values[0], values[1]
        else:
            end_x, end_y = start_x, start_y
        segments.append(Segment(x, y, end_x, end_y))
        x, y = end_x, end_y
    return segments
