"""Tests for scripts/figures/control_systems_figures.py.

The script writes the two figures of the Introduction to Control Systems page.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Dict
from typing import Optional
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import control_systems_figures  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'control_systems_figures.py'
INCLUDE_NAMES = ['control-systems-closed-loop.html', 'control-systems-system.html']
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

    The closed loop's include has a character outside ASCII, the minus sign.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')


def signal_label(number: Optional[int] = None) -> str:
    """Return the SVG elements of one signal's label, as Figure.signal() adds them."""
    figure_height = 100
    signal_x, signal_y = 10, 20
    figure = control_systems_figures.Figure('test', figure_height, 'title')
    figure.signal(signal_x, signal_y, 'inputs', 'torque on the wheel', number)
    return ''.join(figure.elements)


class TestSignal(unittest.TestCase):
    """A signal's label has a number in a disc only when the signal has a number."""

    def test_signal_without_a_number_has_no_disc(self) -> None:
        self.assertNotIn('number-disc', signal_label())

    def test_signal_with_a_number_shows_the_number_in_a_disc(self) -> None:
        label = signal_label(number=4)
        self.assertIn('class="number-disc"', label)
        self.assertIn('>4</text>', label)

    def test_signal_with_the_number_zero_shows_the_zero_in_a_disc(self) -> None:
        label = signal_label(number=0)
        self.assertIn('class="number-disc"', label)
        self.assertIn('>0</text>', label)


if __name__ == '__main__':
    unittest.main()
