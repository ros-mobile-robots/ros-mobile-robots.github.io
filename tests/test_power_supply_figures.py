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
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import power_supply_figures  # noqa: E402

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
        # The layouts' classes appear in the declarations of the container query's rule
        rules = ' '.join(f'{selector} {declarations}'
                         for selector, declarations in power_supply_figures.RULES)
        for robot in (power_supply_figures.DIFFBOT, power_supply_figures.REMO):
            for figure in power_supply_figures.layouts(robot):
                for classes in re.findall(r'class="([^"]+)"', figure.svg()):
                    for css_class in classes.split():
                        self.assertIn(f'.{css_class}', rules, (robot.name, css_class))


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
        i2c_leave, i2c_lane, i2c_arrive = figures.DIFFBOT_I2C
        ground_leave, ground_lane, ground_arrive = figures.DIFFBOT_GROUND
        camera_data_y = (figures.CAMERA_BOX.y + figures.CAMERA_BOX.height / 2
                         + figures.PAIR_GAP / 2)
        # Both leave the Raspberry Pi below the camera's cable, the I2C wire first
        self.assertTrue(camera_data_y < i2c_leave < ground_leave < figures.RASPBERRY_PI_BOX.bottom)
        # The I2C wire runs down farther right and turns higher, so the ground wire stays inside it
        self.assertGreater(i2c_lane, ground_lane)
        self.assertLess(i2c_arrive, ground_arrive)
        # Both lanes lie in the gap between the left column and the LiDAR and the camera
        self.assertTrue(figures.RASPBERRY_PI_BOX.right < ground_lane < i2c_lane
                        < figures.CAMERA_BOX.x)
        driver = figures.DRIVER_BOX
        self.assertTrue(driver.y < i2c_arrive < ground_arrive < driver.bottom)

    def test_diffbots_encoder_supply_crosses_no_box_and_no_wire(self) -> None:
        figures = power_supply_figures
        leave_x, turn_y, down_x, approach_y = figures.DIFFBOT_ENCODER_RAIL
        pi, mcu = figures.RASPBERRY_PI_BOX, figures.MCU_BOX
        encoders = figures.RIGHT_SENSOR_BOX
        # It leaves the Raspberry Pi's lower edge right of the MCU's USB cable and its label
        self.assertTrue(pi.x + figures.LEFT_SUPPLY_OFFSET < leave_x < pi.right)
        # It turns between the Raspberry Pi and the MCU, below the I2C and the ground wires' turns
        self.assertTrue(pi.bottom < turn_y < mcu.y)
        self.assertGreater(turn_y, max(figures.DIFFBOT_I2C[0], figures.DIFFBOT_GROUND[0]))
        # It runs down between the MCU and the ground wire's lane
        self.assertTrue(mcu.right < down_x < figures.DIFFBOT_GROUND[1])
        # It turns again between the MCU and the encoders, and reaches the encoders' box
        self.assertTrue(mcu.bottom < approach_y < encoders.y)
        self.assertTrue(0 < figures.DIFFBOT_ENCODER_SUPPLY_OFFSET < encoders.width)

    def test_each_robots_legend_gives_its_motor_packs_voltage(self) -> None:
        for robot in (power_supply_figures.DIFFBOT, power_supply_figures.REMO):
            meanings = [entry.meaning for entry in power_supply_figures.legend_entries(robot.pack)]
            self.assertIn(f'supply from the motor battery ({robot.pack.voltage})', meanings,
                          robot.name)


if __name__ == '__main__':
    unittest.main()
