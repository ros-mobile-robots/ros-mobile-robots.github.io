"""Read the straight pieces of a drawn SVG path, for the tests of the figure scripts.

A test module imports Segment and path_segments() after it has put this folder on sys.path. The
name of this file doesn't start with test_, so the test discovery doesn't collect it.
"""

import re
from typing import Iterator
from typing import List
from typing import NamedTuple
from typing import Tuple

# A number as a figure script writes it: with a sign, a decimal point or an exponent
NUMBER = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'
# One command with exactly the coordinates that it takes: a pair after M or L, one coordinate
# after H or V, none after Z. A comma or spaces separate the two coordinates of a pair
COMMAND = re.compile(rf'\s*(?:(?P<pair>[ML])\s*(?P<x>{NUMBER})(?:\s*,\s*|\s+)(?P<y>{NUMBER})'
                     rf'|(?P<single>[HV])\s*(?P<coordinate>{NUMBER})'
                     r'|(?P<close>Z))\s*')


class Segment(NamedTuple):
    """A straight piece of a drawn line, from (x1, y1) to (x2, y2)."""

    x1: float
    y1: float
    x2: float
    y2: float


class PathCommand(NamedTuple):
    """A command of a path: its letter and the coordinates after it."""

    letter: str
    coordinates: Tuple[float, ...]


def path_commands(path: str) -> Iterator[PathCommand]:
    """Yield the commands of a path in order, and consume the whole path.

    Raise ValueError at the first text that is not M, L, H, V or Z with exactly the coordinates
    that the command takes.
    """
    position = 0
    while position < len(path):
        command = COMMAND.match(path, position)
        if command is None:
            raise ValueError('unsupported command or unexpected arguments at position '
                             f'{position} of the path {path!r}')
        numbers = (command['x'], command['y'], command['coordinate'])
        yield PathCommand(command['pair'] or command['single'] or command['close'],
                          tuple(float(number) for number in numbers if number is not None))
        position = command.end()


def path_segments(path: str) -> List[Segment]:
    """Read straight segments from a path that starts with an absolute M.

    Each M or L takes one coordinate pair, each H or V takes one coordinate, and Z closes the
    current subpath. Raise ValueError on unsupported commands or unexpected arguments.
    """
    commands = list(path_commands(path))
    if not commands or commands[0].letter != 'M':
        raise ValueError(f'the path {path!r} does not start with an absolute M')
    segments: List[Segment] = []
    x = y = start_x = start_y = 0.0
    for command in commands:
        if command.letter == 'M':
            x, y = start_x, start_y = command.coordinates
            continue
        if command.letter == 'L':
            end_x, end_y = command.coordinates
        elif command.letter == 'H':
            end_x, end_y = command.coordinates[0], y
        elif command.letter == 'V':
            end_x, end_y = x, command.coordinates[0]
        else:
            end_x, end_y = start_x, start_y
        segments.append(Segment(x, y, end_x, end_y))
        x, y = end_x, end_y
    return segments
