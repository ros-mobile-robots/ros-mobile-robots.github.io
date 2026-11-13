"""Tests for scripts/figures/pid_figures.py, which writes the PID Controllers page's figure.

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


if __name__ == '__main__':
    unittest.main()
