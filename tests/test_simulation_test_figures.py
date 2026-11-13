"""Tests for scripts/figures/simulation_test_figures.py.

The script writes the figures of the Simulation Test page and of the Robot Motion page.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Dict
import unittest

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / 'scripts' / 'figures' / 'simulation_test_figures.py'
INCLUDE_NAMES = ['odometry-motion-model.html', 'simulation-test-figures.html',
                 'simulation-test-pivot.html', 'simulation-test-relative-pose.html',
                 'simulation-test-wheel-radius.html']
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

    Four of the five includes have characters outside ASCII, such as the Greek letters.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')


if __name__ == '__main__':
    unittest.main()
