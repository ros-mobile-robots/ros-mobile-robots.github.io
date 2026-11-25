"""Tests for scripts/figures/motor_driver_figures.py, the figures of DC Motors and Motor Drivers.

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

import motor_driver_figures as figures  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'motor_driver_figures.py'
PAGE = ROOT / 'docs' / 'theory' / 'actuation.md'
INCLUDE_NAMES = ['actuation-h-bridge.html', 'actuation-pwm.html']
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}
# A row of the page's table of the modes: the mode and the switches that are closed
MODE_ROW = re.compile(r'^\| (?P<mode>Forward|Reverse|Coast|Brake) \| (?P<closed>[^|]+) \|',
                      re.MULTILINE)


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

    The includes have characters outside ASCII, the minus sign of the commands among them.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / name).read_bytes()
            self.assertEqual(written[name], committed, f'{name} is not what {SCRIPT.name} writes')

    def test_every_drawing_has_an_accessible_name_and_both_layouts(self) -> None:
        for name in INCLUDE_NAMES:
            include = (ROOT / 'includes' / name).read_text(encoding='utf-8')
            drawings = re.findall(r'<svg [^>]*>', include)
            self.assertEqual(len(drawings), 2, name)
            for drawing in drawings:
                self.assertIn('role="img"', drawing, name)
                self.assertIn('aria-labelledby=', drawing, name)
            self.assertIn(figures.NARROW_LAYOUT, include, name)
            self.assertIn(figures.WIDE_LAYOUT, include, name)


class TestModes(unittest.TestCase):
    """The modes close the switches that AN905's Table 1 gives, and never short the supply."""

    def test_no_mode_closes_both_switches_of_a_half_bridge(self) -> None:
        for mode in figures.MODES:
            for right in (False, True):
                side = {switch.name for switch in figures.SWITCHES if switch.right == right}
                self.assertLess(len(side & mode.closed), 2, mode.name)

    def test_forward_and_reverse_drive_the_motor_in_opposite_directions(self) -> None:
        by_name = {mode.name: mode for mode in figures.MODES}
        high = {switch.name for switch in figures.SWITCHES if switch.high}
        for name in ('Forward', 'Reverse'):
            # One switch to the supply and one to ground, on opposite sides
            self.assertEqual(len(by_name[name].closed & high), 1, name)
        self.assertEqual(by_name['Forward'].direction, -by_name['Reverse'].direction)
        self.assertNotEqual(by_name['Forward'].direction, 0)

    def test_the_pages_table_closes_the_same_switches_as_the_figure(self) -> None:
        rows = {row['mode']: row['closed'].strip()
                for row in MODE_ROW.finditer(PAGE.read_text(encoding='utf-8'))}
        self.assertEqual(sorted(rows), sorted(mode.name for mode in figures.MODES))
        for mode in figures.MODES:
            named = set(re.findall(r'Q\d', rows[mode.name]))
            self.assertEqual(named, set(mode.closed), mode.name)


class TestPwm(unittest.TestCase):
    """The duty cycles, averages and frequency follow Adafruit's library and the PCA9685."""

    def test_the_commands_give_quarter_duty_cycles(self) -> None:
        duties = [pulse.duty for pulse in figures.pulses()]
        self.assertEqual(duties, [0.25, 0.5, 0.75])
        for pulse in figures.pulses():
            self.assertAlmostEqual(pulse.average, pulse.duty * figures.SUPPLY_VOLTAGE)

    def test_the_library_rounds_the_prescaler_to_the_chips_smallest(self) -> None:
        # The PCA9685 accepts no prescaler below 3, which gives its highest frequency, 1526 Hz
        self.assertEqual(figures.prescaler(), 3)
        self.assertAlmostEqual(figures.frequency_hz(), 1525.9, places=1)

    def test_the_page_repeats_the_figures_numbers(self) -> None:
        page = PAGE.read_text(encoding='utf-8')
        counts = 128 * figures.COUNTS_PER_STEP
        self.assertIn(f'The command 128, for example, gives {counts} of '
                      f'{figures.COUNTS_PER_PERIOD} counts, a duty cycle of 50 %', page)
        highest = 255 * figures.COUNTS_PER_STEP
        share = highest / figures.COUNTS_PER_PERIOD * 100
        self.assertIn(f'The highest command, 255, gives {highest} of {figures.COUNTS_PER_PERIOD} '
                      f'counts, a duty cycle of {share:.1f} %', page)
        self.assertIn(f"rounds the chip's prescaler to {figures.prescaler()}", page)
        self.assertIn(f'about {figures.frequency_hz():.0f} Hz', page)
        self.assertIn(f'about {figures.period_ms():.2f} ms', page)


if __name__ == '__main__':
    unittest.main()
