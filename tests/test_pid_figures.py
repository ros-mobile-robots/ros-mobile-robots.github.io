"""Tests for scripts/figures/pid_figures.py, which writes the PID Controllers page's figure.

Run from the repository root: python -m unittest discover -s tests
"""

import contextlib
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Dict
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import pid_figures  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'pid_figures.py'
INCLUDE_NAME = 'pid-feedback-loop.html'
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale() -> Dict[str, bytes]:
    """Run the script with an ASCII locale and return the includes it writes, by file name.

    The script runs in an empty folder, so the repository's include stays as it is.
    """
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        run = subprocess.run([sys.executable, str(SCRIPT)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{SCRIPT.name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


class TestInclude(unittest.TestCase):
    """The script writes the repository's include, whatever the locale's encoding is.

    The include has a character outside ASCII, the minus sign.
    """

    def test_script_writes_the_repository_include_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(list(written), [INCLUDE_NAME])
        committed = (ROOT / 'includes' / INCLUDE_NAME).read_bytes()
        self.assertEqual(written[INCLUDE_NAME], committed,
                         f'{INCLUDE_NAME} is not what {SCRIPT.name} writes')


class TestChecks(unittest.TestCase):
    """Each check reports a step that doesn't fit the firmware's arithmetic or the example.

    The steps below change one or two values of the script's logged step: a target of
    6.06 rad/s, a measured speed of 5.56 rad/s, an error of 0.50 rad/s and the motor command 96.
    """

    def test_checks_report_nothing_for_the_scripts_logged_step(self) -> None:
        for check in pid_figures.CHECKS:
            self.assertEqual(check(pid_figures.LOGGED_STEP), [], check.__name__)

    def test_error_check_reports_an_error_that_is_not_target_minus_measured(self) -> None:
        step = pid_figures.LOGGED_STEP._replace(error=0.40)
        self.assertEqual(
            pid_figures.check_error_is_target_minus_measured(step),
            ['the target 6.06 rad/s minus the measured 5.56 rad/s is 0.5 rad/s, not the logged '
             'error 0.4 rad/s'])

    def test_range_check_reports_a_pid_output_above_the_largest_motor_command(self) -> None:
        # K_I times this sum alone is 300, above the largest motor command of 255
        step = pid_figures.LOGGED_STEP._replace(error_sum=1000.0)
        self.assertEqual(
            pid_figures.check_pid_output_is_within_the_pwm_range(step),
            ['the PID output 300.30 is outside -255 to 255, the range that the example '
             'explains'])

    def test_motor_command_check_reports_a_command_that_the_pid_output_does_not_give(
            self) -> None:
        step = pid_figures.LOGGED_STEP._replace(motor_command=97)
        self.assertEqual(
            pid_figures.check_pid_output_gives_the_motor_command(step),
            ['the PID output 96.96 does not give the logged motor command 97'])

    def test_tick_check_reports_a_speed_that_no_whole_number_of_ticks_gives(self) -> None:
        # 23 ticks in a control period are 5.33 rad/s, and 24 ticks are 5.56 rad/s
        step = pid_figures.LOGGED_STEP._replace(measured_speed=5.50)
        self.assertEqual(
            pid_figures.check_ticks_give_the_measured_speed(step),
            ['24 ticks are 5.5644 rad/s, not the logged 5.5 rad/s'])

    def test_check_logged_step_raises_with_every_problem_of_the_step(self) -> None:
        step = pid_figures.LOGGED_STEP._replace(error=0.40, motor_command=97)
        with self.assertRaises(ValueError) as raised:
            pid_figures.check_logged_step(step)
        message = str(raised.exception)
        self.assertIn('not the logged error 0.4 rad/s', message)
        self.assertIn('does not give the logged motor command 97', message)


class TestMotorCommand(unittest.TestCase):
    """motor_command() limits a raw output as PID::compute() does and drops its fraction."""

    def test_motor_command_drops_the_fraction_of_a_raw_output_within_the_range(self) -> None:
        self.assertEqual(pid_figures.motor_command(96.96), 96)

    def test_motor_command_drops_the_fraction_towards_zero_for_a_negative_raw_output(
            self) -> None:
        self.assertEqual(pid_figures.motor_command(-96.96), -96)

    def test_motor_command_is_the_largest_command_for_a_raw_output_above_the_range(
            self) -> None:
        raw_output = 300.0
        self.assertGreater(raw_output, pid_figures.PWM_MAX)
        self.assertEqual(pid_figures.motor_command(raw_output), pid_figures.PWM_MAX)

    def test_motor_command_is_the_smallest_command_for_a_raw_output_below_the_range(
            self) -> None:
        raw_output = -300.0
        self.assertLess(raw_output, -pid_figures.PWM_MAX)
        self.assertEqual(pid_figures.motor_command(raw_output), -pid_figures.PWM_MAX)


class TestMain(unittest.TestCase):
    """main() checks the logged step before it writes anything."""

    def test_main_keeps_the_include_when_only_the_tick_check_fails(self) -> None:
        # No whole number of ticks gives this speed: 23 are 5.33 rad/s, and 24 are 5.56 rad/s
        measured_speed = 5.50  # rad/s
        logged_step = pid_figures.LOGGED_STEP
        step = logged_step._replace(measured_speed=measured_speed,
                                    target_speed=round(measured_speed + logged_step.error, 2))
        for check in pid_figures.CHECKS:
            if check is not pid_figures.check_ticks_give_the_measured_speed:
                self.assertEqual(check(step), [], check.__name__)
        include_before = b'the include of an earlier run'
        with tempfile.TemporaryDirectory() as folder:
            include = Path(folder) / 'includes' / INCLUDE_NAME
            include.parent.mkdir()
            include.write_bytes(include_before)
            with contextlib.chdir(folder), mock.patch.object(pid_figures, 'LOGGED_STEP', step), \
                    self.assertRaisesRegex(ValueError, 'ticks are .* not the logged 5.5 rad/s'):
                pid_figures.main()
            self.assertEqual(include.read_bytes(), include_before)


# The width of the PID page's article on 1280 and 1440 px screens, measured in the browser
ARTICLE_WIDTH = 688  # px
# The values of the logged step that both layouts show: target, error, motor command, duty cycle,
# ticks and measured speed
STEP_VALUES = ('6.06 rad/s', '0.50 rad/s', '>96<', '37.5 %', '>24<', '5.56 rad/s')


class TestLayouts(unittest.TestCase):
    """The include holds a wide and a tall layout of the figure, and CSS shows one of them.

    Both layouts show the logged step's values and share the caption and the accessible name.
    """

    def setUp(self) -> None:
        self.wide = pid_figures.wide_feedback_loop(pid_figures.LOGGED_STEP).svg()
        self.tall = pid_figures.tall_feedback_loop(pid_figures.LOGGED_STEP).svg()
        self.include = (ROOT / 'includes' / INCLUDE_NAME).read_text(encoding='utf-8')

    def test_include_holds_the_wide_layout_then_the_tall_one_and_one_caption(self) -> None:
        self.assertIn(self.wide, self.include)
        self.assertIn(self.tall, self.include)
        self.assertLess(self.include.index(self.wide), self.include.index(self.tall))
        self.assertEqual(self.include.count('<figcaption>'), 1)

    def test_css_shows_the_wide_layout_only_from_the_wide_screen_width(self) -> None:
        self.assertIn('.pid-loop-wide{display:none}', self.include)
        self.assertIn(f'@media screen and (min-width:{pid_figures.WIDE_FROM}){{'
                      '.md-typeset figure.pid-loop-figure{max-width:none}'
                      '.pid-loop-wide{display:block}.pid-loop-tall{display:none}}',
                      self.include)

    def test_both_layouts_have_the_same_accessible_name(self) -> None:
        for name, svg in (('pid-loop-wide', self.wide), ('pid-loop-tall', self.tall)):
            self.assertIn(f'<title id="{name}-title">{pid_figures.TITLE}</title>', svg)
            self.assertIn(f'aria-labelledby="{name}-title"', svg)

    def test_each_layout_points_its_arrows_to_its_own_arrowhead(self) -> None:
        for name, svg in (('pid-loop-wide', self.wide), ('pid-loop-tall', self.tall)):
            self.assertIn(f'<marker id="{name}-head"', svg)
            self.assertEqual(set(re.findall(r'marker-end="url\(#([\w-]+)\)"', svg)),
                             {f'{name}-head'})

    def test_both_layouts_show_the_values_of_the_logged_step(self) -> None:
        for svg in (self.wide, self.tall):
            for value in STEP_VALUES:
                self.assertIn(value, svg)

    def test_wide_layout_and_its_boxes_fit_the_pid_pages_article(self) -> None:
        self.assertLessEqual(pid_figures.WIDE_WIDTH, ARTICLE_WIDTH)
        boxes = (pid_figures.WIDE_SBC, pid_figures.WIDE_FIRMWARE, pid_figures.WIDE_PID,
                 pid_figures.WIDE_SPEED, pid_figures.WIDE_DRIVER, pid_figures.WIDE_MOTOR,
                 pid_figures.WIDE_ENCODER)
        for box in boxes:
            self.assertLessEqual(box.x + box.width, pid_figures.WIDE_WIDTH, box)
            self.assertLessEqual(box.y + box.height, pid_figures.WIDE_TIME_Y, box)


class TestFormula(unittest.TestCase):
    """Figure.formula() writes a formula's subscripts below the line and escapes its text."""

    def test_formula_writes_subscripts_below_the_line(self) -> None:
        figure = pid_figures.Figure('pid-loop-wide', 100, 100)
        figure.formula(50, 20, [('u', False), ('raw', True), (' < 255', False)])
        self.assertEqual(figure.elements, [
            '<text class="formula" x="50" y="20" text-anchor="middle">u'
            '<tspan baseline-shift="sub" font-size="8.5">raw</tspan> &lt; 255</text>'])


if __name__ == '__main__':
    unittest.main()
