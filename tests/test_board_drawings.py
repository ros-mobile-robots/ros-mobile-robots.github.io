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
    lambda drawing: board_drawings.draw_power_bank(drawing, 0, 0),
    lambda drawing: board_drawings.draw_battery_holder(drawing, 0, 0),
    lambda drawing: board_drawings.draw_lidar(drawing, 0, 0),
    lambda drawing: board_drawings.draw_camera(drawing, 0, 0),
    lambda drawing: board_drawings.draw_grove_motor_driver(drawing, 0, 0),
)


def classes_drawn(draw: Callable[[board_drawings.Drawing], None]) -> Set[str]:
    """Return the CSS classes of the elements that draw adds to an empty drawing."""
    drawing = board_drawings.Drawing()
    draw(drawing)
    return set(re.findall(r'class="([^"]+)"', ''.join(drawing.elements)))


class TestBoardRules(unittest.TestCase):
    """Every part keeps its colors: each class a drawing uses has a rule in the figure's scope."""

    def test_every_class_that_a_drawing_uses_has_a_color_rule(self) -> None:
        rules = (board_drawings.board_rules('.some-figure')
                 + board_drawings.supply_part_rules('.some-figure'))
        selectors = {selector for selector, _ in rules}
        for draw in DRAWINGS:
            for css_class in classes_drawn(draw):
                with self.subTest(css_class=css_class):
                    self.assertIn(f'.some-figure .{css_class}', selectors)

    def test_every_rule_applies_only_inside_the_figure(self) -> None:
        rules = (board_drawings.board_rules('.some-figure')
                 + board_drawings.supply_part_rules('.some-figure'))
        for selector, _ in rules:
            with self.subTest(selector=selector):
                self.assertRegex(selector, r'(^|\] )\.some-figure \.[a-z-]+$')


class TestSvgElement(unittest.TestCase):
    """svg_element() names the SVG and gives its arrows an arrowhead with the element's own ID."""

    def setUp(self) -> None:
        self.svg = board_drawings.svg_element('some-layout', 'some-figure some-layout', (340, 120),
                                              'A & B', ['<path class="arrow"/>'])

    def test_svg_element_has_its_title_as_its_accessible_name(self) -> None:
        self.assertIn('role="img" aria-labelledby="some-layout-title"><title '
                      'id="some-layout-title">A &amp; B</title>', self.svg)

    def test_svg_element_defines_the_arrowhead_with_its_own_id(self) -> None:
        self.assertIn('<defs><marker id="some-layout-head"', self.svg)

    def test_svg_element_holds_its_size_and_elements(self) -> None:
        self.assertTrue(self.svg.startswith('<svg class="some-figure some-layout" '
                                            'viewBox="0 0 340 120"'))
        self.assertTrue(self.svg.endswith('\n<path class="arrow"/>\n</svg>'))

    def test_svg_element_uses_the_figures_own_markers_instead_of_the_arrowhead(self) -> None:
        svg = board_drawings.svg_element('some-layout', 'some-figure', (340, 120), 'A', [],
                                         markers=['<marker id="one"/>', '<marker id="two"/>'])
        self.assertIn('<defs><marker id="one"/><marker id="two"/></defs>', svg)
        self.assertNotIn('some-layout-head', svg)


class TestNumberDisc(unittest.TestCase):
    """Drawing.number_disc() centers the number in its disc."""

    def test_number_disc_draws_the_disc_and_its_number_around_the_point(self) -> None:
        drawing = board_drawings.Drawing()
        drawing.number_disc(16, 20, 3, 'disc', 'number')
        self.assertEqual(drawing.elements,
                         ['<circle class="disc" cx="16" cy="20" r="8"/>',
                          '<text class="number" x="16" y="23.8" text-anchor="middle">3</text>'])


if __name__ == '__main__':
    unittest.main()
