"""Tests for tests/svg_paths.py, the path reader of the figure scripts' tests.

Run from the repository root: python -m unittest discover -s tests
"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from svg_paths import path_segments  # noqa: E402
from svg_paths import Segment  # noqa: E402


class TestPathSegments(unittest.TestCase):
    """path_segments() follows a path from point to point and refuses what it can't read."""

    def test_each_piece_starts_where_the_piece_before_it_ends(self) -> None:
        self.assertEqual(path_segments('M10,20 H40 V60 L15.5,70'),
                         [Segment(10, 20, 40, 20), Segment(40, 20, 40, 60),
                          Segment(40, 60, 15.5, 70)])

    def test_z_closes_a_part_with_a_piece_back_to_its_start(self) -> None:
        self.assertEqual(path_segments('M10,20 H40 V60 Z'),
                         [Segment(10, 20, 40, 20), Segment(40, 20, 40, 60),
                          Segment(40, 60, 10, 20)])

    def test_a_second_m_command_starts_a_new_part_that_z_closes_to_its_own_start(self) -> None:
        self.assertEqual(path_segments('M0,0 H10 M20,5 V15 Z'),
                         [Segment(0, 0, 10, 0), Segment(20, 5, 20, 15), Segment(20, 15, 20, 5)])

    def test_a_curve_or_a_relative_command_raises_an_error(self) -> None:
        for path in ('M0,0 Q5,5 10,0', 'M0,0 l5,5', 'M0,0 A5,5 0 0 1 10,0'):
            with self.assertRaises(ValueError, msg=path):
                path_segments(path)


if __name__ == '__main__':
    unittest.main()
