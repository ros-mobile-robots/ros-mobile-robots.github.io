"""Tests for scripts/figures/board_drawings.py, the parts that several figures draw.

Run from the repository root: python -m unittest discover -s tests
"""

from pathlib import Path
import re
import sys
from typing import Callable
from typing import Set
import unittest
from xml.etree import ElementTree

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


class TestSvgElementWithADescription(unittest.TestCase):
    """svg_element() adds a description to the accessible name and takes the drawing's markers."""

    def test_svg_element_names_its_title_and_its_description(self) -> None:
        svg = board_drawings.svg_element('some-layout', 'some-figure', (340, 120), 'A & B', [],
                                         description='C < D')
        self.assertIn('aria-labelledby="some-layout-title some-layout-description">'
                      '<title id="some-layout-title">A &amp; B</title>'
                      '<desc id="some-layout-description">C &lt; D</desc><defs>', svg)

    def test_svg_element_has_no_definitions_without_markers(self) -> None:
        svg = board_drawings.svg_element('some-layout', 'some-figure', (340, 120), 'A', [],
                                         markers=())
        self.assertNotIn('<defs>', svg)
        self.assertNotIn('<marker', svg)

    def test_svg_element_defines_exactly_the_given_markers(self) -> None:
        markers = [board_drawings.arrowhead_marker('some-layout-flow', 'flow-head',
                                                   'auto-start-reverse'),
                   board_drawings.arrowhead_marker('some-layout-request', 'request-head')]
        svg = board_drawings.svg_element('some-layout', 'some-figure', (340, 120), 'A', [],
                                         markers=markers)
        self.assertEqual(re.findall(r'<marker id="([^"]+)"[^>]* orient="([^"]+)">'
                                    r'<path class="([^"]+)"', svg),
                         [('some-layout-flow', 'auto-start-reverse', 'flow-head'),
                          ('some-layout-request', 'auto', 'request-head')])


class TestSvgSubscript(unittest.TestCase):
    """svg_subscript() lowers the index and, on request, lifts the text after it again."""

    def test_svg_subscript_lowers_the_index_at_the_end_of_a_line(self) -> None:
        self.assertEqual(board_drawings.svg_subscript('f', 'z'),
                         'f<tspan class="sub" dy="3">z</tspan>')

    def test_svg_subscript_restores_the_baseline_for_the_text_after_it(self) -> None:
        self.assertEqual(board_drawings.svg_subscript('f', 'z', restore_baseline=True),
                         'f<tspan class="sub" dy="3">z</tspan><tspan dy="-3">\u200b</tspan>')

    def test_svg_subscript_escapes_its_symbol_and_its_index(self) -> None:
        markup = board_drawings.svg_subscript('I & Q', 'x<y')
        text = ElementTree.fromstring(f'<text>{markup}</text>')
        self.assertEqual(text.text, 'I & Q')
        self.assertEqual([(span.get('class'), span.text) for span in text], [('sub', 'x<y')])


class TestText(unittest.TestCase):
    """Drawing.text() escapes its text, and Drawing.text_markup() keeps its markup."""

    def test_text_escapes_the_characters_with_a_meaning_in_svg(self) -> None:
        drawing = board_drawings.Drawing()
        drawing.text(4, 12.5, 'a < b', 'note', 'end')
        self.assertEqual(drawing.elements,
                         ['<text class="note" x="4" y="12.5" text-anchor="end">a &lt; b</text>'])

    def test_text_markup_keeps_the_markup_as_it_is(self) -> None:
        drawing = board_drawings.Drawing()
        drawing.text_markup(4, 12.5, board_drawings.svg_subscript('f', 'z'))
        self.assertEqual(drawing.elements,
                         ['<text x="4" y="12.5" text-anchor="start">f<tspan class="sub" dy="3">z'
                          '</tspan></text>'])


class TestFigureElement(unittest.TestCase):
    """figure_element() puts the panels and the caption into one figure, sized by its widths."""

    PANELS = '<svg class="some-fig"></svg>'
    WIDTHS = board_drawings.PanelWidths(basis=280, widest=400)

    def test_figure_element_lets_markdown_read_only_the_caption(self) -> None:
        figure = board_drawings.figure_element('some', self.PANELS, 'A `caption`', self.WIDTHS)
        self.assertEqual(
            figure,
            '<figure class="some-figure" markdown="block" '
            'style="--some-basis:280px;--some-max:400px">\n'
            '<div class="some-panels">\n<svg class="some-fig"></svg>\n</div>\n'
            '<figcaption markdown="span">A `caption`</figcaption>\n</figure>')

    def test_figure_element_has_no_markdown_attribute_for_a_caption_in_html(self) -> None:
        figure = board_drawings.figure_element('some', self.PANELS, 'A <code>caption</code>',
                                               self.WIDTHS, caption_is_markdown=False)
        self.assertNotIn('markdown=', figure)
        self.assertIn('<figcaption>A <code>caption</code></figcaption>', figure)


class TestTabSet(unittest.TestCase):
    """tab_set() builds each tab's radio input, label and block from one Tab."""

    TABS = (board_drawings.Tab('First & only', '<p>one</p>'),
            board_drawings.Tab('Second', '<p>two</p>'),
            board_drawings.Tab('Third', '<p>three</p>'))

    def setUp(self) -> None:
        self.tab_set = board_drawings.tab_set('some', 'some-tabs', self.TABS)

    def test_tab_set_has_one_radio_input_per_tab_and_selects_the_first(self) -> None:
        inputs = re.findall(r'<input( checked)? id="([^"]+)" name="([^"]+)" type="radio">',
                            self.tab_set)
        self.assertEqual(inputs, [(' checked', '__tabbed_some_1', '__tabbed_some'),
                                  ('', '__tabbed_some_2', '__tabbed_some'),
                                  ('', '__tabbed_some_3', '__tabbed_some')])

    def test_tab_set_labels_each_input_with_its_tabs_escaped_label(self) -> None:
        labels = re.findall(r'<label for="([^"]+)">([^<]*)</label>', self.tab_set)
        self.assertEqual(labels, [('__tabbed_some_1', 'First &amp; only'),
                                  ('__tabbed_some_2', 'Second'), ('__tabbed_some_3', 'Third')])

    def test_tab_set_holds_the_tabs_contents_in_the_order_of_their_labels(self) -> None:
        blocks = re.findall(r'<div class="tabbed-block">\n(.*)\n</div>', self.tab_set)
        self.assertEqual(blocks, ['<p>one</p>', '<p>two</p>', '<p>three</p>'])

    def test_tab_set_names_its_class_and_its_number_of_tabs(self) -> None:
        self.assertTrue(self.tab_set.startswith(
            '<div class="tabbed-set tabbed-alternate some-tabs" data-tabs="some:3">'))


class TestBox(unittest.TestCase):
    """Box gives a rectangle's far edges and its middle from its corner and its size."""

    def setUp(self) -> None:
        """Create an offset, non-square rectangle."""
        self.box = board_drawings.Box(x=10, y=20, width=60, height=30)

    def test_box_right_and_bottom_are_the_corner_plus_the_size(self) -> None:
        """Check the right and bottom edges."""
        expected_right, expected_bottom = 70, 50
        self.assertEqual((self.box.right, self.box.bottom), (expected_right, expected_bottom))

    def test_box_middle_is_half_the_size_from_the_corner(self) -> None:
        """Check the horizontal and vertical midpoints."""
        expected_middle_x, expected_middle_y = 40, 35
        self.assertEqual((self.box.middle_x, self.box.middle_y),
                         (expected_middle_x, expected_middle_y))


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
