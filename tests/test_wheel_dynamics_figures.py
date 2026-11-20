"""Tests for scripts/figures/wheel_dynamics_figures.py.

The script writes the figures of the Wheel Dynamics in Simulation page.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Dict
import unittest

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / 'scripts' / 'figures' / 'wheel_dynamics_figures.py'
# The measured data that the script plots. The script reads it by this path from its folder
DATA_FOLDER = Path('scripts') / 'figures' / 'data'
INCLUDE_NAMES = ['wheel-dynamics-contacts.html', 'wheel-dynamics-sinkage.html',
                 'wheel-dynamics-stability.html', 'wheel-dynamics-wheel-speeds.html']
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale() -> Dict[str, bytes]:
    """Run the script with an ASCII locale and return the includes it writes, by file name.

    The script runs in a folder that holds only a copy of its data, so the repository's includes
    stay as they are.
    """
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        shutil.copytree(ROOT / DATA_FOLDER, Path(folder) / DATA_FOLDER)
        run = subprocess.run([sys.executable, str(SCRIPT)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{SCRIPT.name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the division sign.
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
