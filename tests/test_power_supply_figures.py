"""Tests for scripts/figures/power_supply_figures.py.

The script writes the two power flow figures of the Power Supply page.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Dict
from typing import List
import unittest
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))
sys.path.insert(0, str(ROOT / 'tests'))

import power_supply_figures  # noqa: E402
from svg_paths import path_segments  # noqa: E402
from svg_paths import Segment  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'power_supply_figures.py'
INCLUDE_NAMES = ['power-supply-flow-diffbot.html', 'power-supply-flow-remo.html']
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale() -> Dict[str, bytes]:
    """Run the script with an ASCII locale and return the includes it writes, by file name.

    The script runs in an empty folder, so the repository's includes stay as they are.
    """
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        run = subprocess.run([sys.executable, str(SCRIPT)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{SCRIPT.name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


def crosses(first: Segment, second: Segment) -> bool:
    """Return whether two horizontal or vertical segments meet or overlap."""
    return (max(min(first.x1, first.x2), min(second.x1, second.x2))
            <= min(max(first.x1, first.x2), max(second.x1, second.x2))
            and max(min(first.y1, first.y2), min(second.y1, second.y2))
            <= min(max(first.y1, first.y2), max(second.y1, second.y2)))


def ends_on_top_of(line: List[Segment], box: power_supply_figures.Box, gap: float) -> bool:
    """Return whether a line ends gap units above a box's upper edge, between its sides."""
    return box.x < line[-1].x2 < box.right and line[-1].y2 == box.y - gap


def drawn_lines(figure: power_supply_figures.Figure, css_class: str) -> List[List[Segment]]:
    """Return the segments of each path of a layout whose class is exactly css_class."""
    root = ElementTree.fromstring(figure.svg())
    return [path_segments(path.get('d', '')) for path in root.iter('path')
            if path.get('class') == css_class]


def drawn_texts(figure: power_supply_figures.Figure) -> List[str]:
    """Return the texts of a layout's text elements, in their order."""
    return [text.text or '' for text in ElementTree.fromstring(figure.svg()).iter('text')]


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    Remo's include has a character outside ASCII, the minus sign.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')


class TestStyles(unittest.TestCase):
    """Every kind of line and every source has its style, so that no line is drawn without one."""

    def setUp(self) -> None:
        self.selectors = [selector for selector, _ in power_supply_figures.RULES]

    def test_every_line_kind_has_a_style_rule(self) -> None:
        for kind in power_supply_figures.LineKind:
            self.assertIn(f'.pf-fig .line.{kind.value}', self.selectors, kind)

    def test_every_source_has_a_style_rule_for_its_supply_line(self) -> None:
        for source in power_supply_figures.Source:
            self.assertIn(f'.pf-fig .line.{source.value}', self.selectors, source)

    def test_every_source_has_a_style_rule_for_its_arrowhead(self) -> None:
        for source in power_supply_figures.Source:
            self.assertIn(f'.pf-fig .head-{source.value}', self.selectors, source)

    def test_every_sources_supply_line_ends_in_a_marker_that_the_figure_defines(self) -> None:
        figure_name = 'pf-test'
        downward_path = 'M0,0 V10'
        for source in power_supply_figures.Source:
            figure = power_supply_figures.Figure(figure_name, 1, 'title', 'pf-narrow')
            figure.supply(downward_path, source)
            marker = f'{figure_name}-{source.value}-head'
            self.assertIn(f'marker-end="url(#{marker})"', figure.svg(), source)
            self.assertIn(f'<marker id="{marker}"', figure.svg(), source)

    def test_every_class_that_a_figure_draws_has_a_style_rule(self) -> None:
        # The layouts' classes also appear in the selectors inside the container query's rule
        styled = set()
        for selector, declarations in power_supply_figures.RULES:
            styled.update(re.findall(r'\.([A-Za-z][\w-]*)', f'{selector} {declarations}'))
        for robot in (power_supply_figures.DIFFBOT, power_supply_figures.REMO):
            for figure in power_supply_figures.layouts(robot):
                for classes in re.findall(r'class="([^"]+)"', figure.svg()):
                    for css_class in classes.split():
                        self.assertIn(css_class, styled, (robot.name, css_class))


class TestLayouts(unittest.TestCase):
    """Each figure has both layouts, and its cables leave the Raspberry Pi where they can."""

    def test_each_include_holds_a_narrow_and_a_wide_layout(self) -> None:
        for name in INCLUDE_NAMES:
            include = (ROOT / 'includes' / name).read_text(encoding='utf-8')
            self.assertEqual(include.count('class="pf-fig pf-narrow"'), 1, name)
            self.assertEqual(include.count('class="pf-fig pf-wide"'), 1, name)

    def test_the_cables_to_the_lidar_and_the_camera_leave_the_raspberry_pi(self) -> None:
        pi = power_supply_figures.RASPBERRY_PI_BOX
        for box in (power_supply_figures.LIDAR_BOX, power_supply_figures.CAMERA_BOX):
            middle = box.y + box.height / 2
            for cable_y in (middle - power_supply_figures.PAIR_GAP / 2,
                            middle + power_supply_figures.PAIR_GAP / 2):
                self.assertTrue(pi.y < cable_y < pi.bottom, box)

    def test_diffbots_i2c_and_ground_wires_never_cross(self) -> None:
        figures = power_supply_figures
        i2c, ground = figures.DIFFBOT_I2C, figures.DIFFBOT_GROUND
        camera_data_y = (figures.CAMERA_BOX.y + figures.CAMERA_BOX.height / 2
                         + figures.PAIR_GAP / 2)
        # Both leave the Raspberry Pi below the camera's cable, the I2C wire first
        self.assertTrue(camera_data_y < i2c.leave_y < ground.leave_y
                        < figures.RASPBERRY_PI_BOX.bottom)
        # The I2C wire runs down farther right and turns higher, so the ground wire stays inside it
        self.assertGreater(i2c.lane_x, ground.lane_x)
        self.assertLess(i2c.arrive_y, ground.arrive_y)
        # Both lanes lie in the gap between the left column and the LiDAR and the camera
        self.assertTrue(figures.RASPBERRY_PI_BOX.right < ground.lane_x < i2c.lane_x
                        < figures.CAMERA_BOX.x)
        driver = figures.DRIVER_BOX
        self.assertTrue(driver.y < i2c.arrive_y < ground.arrive_y < driver.bottom)

    def test_diffbots_encoder_supply_stays_clear_of_the_boxes_and_the_other_wires(self) -> None:
        figures = power_supply_figures
        rail = figures.DIFFBOT_ENCODER_RAIL
        pi, mcu = figures.RASPBERRY_PI_BOX, figures.MCU_BOX
        # It leaves the Raspberry Pi's lower edge right of the MCU's USB cable and its label
        self.assertTrue(pi.x + figures.LEFT_SUPPLY_OFFSET < rail.leave_x < pi.right)
        # It turns between the Raspberry Pi and the MCU, below the I2C and the ground wires' turns
        self.assertTrue(pi.bottom < rail.turn_y < mcu.y)
        self.assertGreater(rail.turn_y,
                           max(figures.DIFFBOT_I2C.leave_y, figures.DIFFBOT_GROUND.leave_y))
        # It runs down between the MCU and the ground wire's lane, then turns above the encoders
        self.assertTrue(mcu.right < rail.lane_x < figures.DIFFBOT_GROUND.lane_x)
        self.assertTrue(mcu.bottom < rail.approach_y < figures.RIGHT_SENSOR_BOX.y)

    def test_diffbots_encoder_supply_reaches_its_box_without_crossing_its_data_wire(
            self) -> None:
        figures = power_supply_figures
        for figure in figures.layouts(figures.DIFFBOT):
            encoders = figure.box_of(figures.RIGHT_SENSOR_BOX)
            supplies = [line for line in drawn_lines(figure, 'line bank')
                        if ends_on_top_of(line, encoders, figures.HEAD_TIP)]
            data = [line for line in drawn_lines(figure, 'line data')
                    if ends_on_top_of(line, encoders, 0)]
            self.assertEqual((len(supplies), len(data)), (1, 1), figure.css_class)
            for supply_segment in supplies[0]:
                for data_segment in data[0]:
                    self.assertFalse(crosses(supply_segment, data_segment),
                                     (figure.css_class, supply_segment, data_segment))

    def test_diffbots_hub_sits_on_the_drawn_i2c_and_ground_wires_and_remo_has_none(
            self) -> None:
        figures = power_supply_figures
        for figure in figures.layouts(figures.DIFFBOT):
            elements = list(ElementTree.fromstring(figure.svg()))
            labels = [index for index, element in enumerate(elements)
                      if element.tag == 'text' and element.text == 'hub']
            self.assertEqual(len(labels), 1, figure.css_class)
            hub_rect = elements[labels[0] - 1]
            self.assertEqual(hub_rect.tag, 'rect', figure.css_class)
            left, top = float(hub_rect.get('x', 0)), float(hub_rect.get('y', 0))
            right = left + float(hub_rect.get('width', 0))
            bottom = top + float(hub_rect.get('height', 0))
            label = elements[labels[0]]
            label_x, label_y = float(label.get('x', 0)), float(label.get('y', 0))
            self.assertTrue(left < label_x < right, figure.css_class)
            self.assertTrue(top < label_y < bottom, figure.css_class)
            for css_class in ('line data', 'line ground'):
                through_the_hub = [
                    segment for line in drawn_lines(figure, css_class) for segment in line
                    if segment.x1 == segment.x2 and left < segment.x1 < right
                    and min(segment.y1, segment.y2) < top and max(segment.y1, segment.y2) > bottom]
                self.assertEqual(len(through_the_hub), 1, (figure.css_class, css_class))
        for figure in figures.layouts(figures.REMO):
            self.assertNotIn('hub', drawn_texts(figure), figure.css_class)
            hub_box = figure.box_of(figures.DIFFBOT_HUB_BOX)
            for rect in ElementTree.fromstring(figure.svg()).findall('rect'):
                rect_x, rect_y = float(rect.get('x', 0)), float(rect.get('y', 0))
                rect_right = rect_x + float(rect.get('width', 0))
                rect_bottom = rect_y + float(rect.get('height', 0))
                overlaps = (rect_x < hub_box.right and hub_box.x < rect_right
                            and rect_y < hub_box.bottom and hub_box.y < rect_bottom)
                self.assertFalse(overlaps, (figure.css_class, rect.attrib))

    def test_diffbots_hub_stays_clear_of_the_encoder_supply_and_the_camera(self) -> None:
        figures = power_supply_figures
        hub = figures.DIFFBOT_HUB_BOX
        self.assertTrue(figures.DIFFBOT_ENCODER_RAIL.lane_x < hub.x)
        self.assertTrue(hub.right < figures.CAMERA_BOX.x)

    def test_each_robot_shows_its_own_pack_and_voltage(self) -> None:
        figures = power_supply_figures
        for robot, pack, voltage in ((figures.DIFFBOT, 'Four alkaline AA cells', '6.0 V'),
                                     (figures.REMO, 'Eight NiMH AA cells', '9.6 V')):
            for figure in figures.layouts(robot):
                texts = drawn_texts(figure)
                for expected in (pack, f'{voltage} nominal',
                                 f'supply from the motor battery ({voltage})'):
                    self.assertEqual(texts.count(expected), 1,
                                     (robot.name, figure.css_class, expected))


if __name__ == '__main__':
    unittest.main()
