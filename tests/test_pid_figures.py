"""Tests for scripts/figures/pid_figures.py, which writes the PID Controllers page's figure.

Run from the repository root: python -m unittest discover -s tests
"""

import contextlib
import os
from pathlib import Path
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
    """Each check reports a logged step that doesn't fit one of the firmware's formulas.

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


if __name__ == '__main__':
    unittest.main()
