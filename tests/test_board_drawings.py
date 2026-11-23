"""Tests for scripts/figures/board_drawings.py, the parts that several figures draw.

Run from the repository root: python -m unittest discover -s tests
"""

from pathlib import Path
import re
import sys
from typing import Callable
from typing import Set
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import board_drawings  # noqa: E402

DRAWINGS = (
    lambda drawing: board_drawings.draw_raspberry_pi(drawing, 0, 0),
    lambda drawing: board_drawings.draw_teensy(drawing, 0, 0),
    lambda drawing: board_drawings.draw_motor_driver_board(drawing, 0, 0),
    lambda drawing: board_drawings.draw_gear_motor_with_wheel(drawing, 0, 0),
    lambda drawing: board_drawings.draw_encoder_disc(drawing, 30, 30),
    lambda drawing: board_drawings.draw_bno055_breakout(drawing, 0, 0),
)


def classes_drawn(draw: Callable[[board_drawings.Drawing], None]) -> Set[str]:
    """Return the CSS classes of the elements that draw adds to an empty drawing."""
    drawing = board_drawings.Drawing()
    draw(drawing)
    return set(re.findall(r'class="([^"]+)"', ''.join(drawing.elements)))


class TestBoardRules(unittest.TestCase):
    """Every part keeps its colors: each class a drawing uses has a rule in the figure's scope."""

    def test_every_class_that_a_drawing_uses_has_a_color_rule(self) -> None:
        selectors = {selector for selector, _ in board_drawings.board_rules('.some-figure')}
        for draw in DRAWINGS:
            for css_class in classes_drawn(draw):
                with self.subTest(css_class=css_class):
                    self.assertIn(f'.some-figure .{css_class}', selectors)

    def test_every_rule_applies_only_inside_the_figure(self) -> None:
        for selector, _ in board_drawings.board_rules('.some-figure'):
            with self.subTest(selector=selector):
                self.assertRegex(selector, r'(^|\] )\.some-figure \.[a-z-]+$')


if __name__ == '__main__':
    unittest.main()
