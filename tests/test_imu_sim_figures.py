"""Tests for the figure scripts of the IMU Simulation pages.

scripts/figures/imu_sim_figures.py, imu_sim_robot_figures.py and imu_sim_test_figures.py write
the pages' includes.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Dict
from typing import Sequence
import unittest

ROOT = Path(__file__).resolve().parent.parent
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale(script_name: str) -> Dict[str, bytes]:
    """Run a figure script with an ASCII locale and return the includes it writes, by file name.

    The script runs in an empty folder, so the repository's includes stay as they are.
    """
    script = ROOT / 'scripts' / 'figures' / script_name
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        run = subprocess.run([sys.executable, str(script)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{script_name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


class TestIncludes(unittest.TestCase):
    """Each script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the minus sign and the apostrophe.
    """

    def assert_script_writes_the_repository_includes(self, script_name: str,
                                                     include_names: Sequence[str]) -> None:
        written = run_with_an_ascii_locale(script_name)
        self.assertEqual(sorted(written), sorted(include_names), script_name)
        for include_name in include_names:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {script_name} writes')

    def test_thread_figures_write_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes(
            'imu_sim_figures.py',
            ['imu-sim-threads-original.html', 'imu-sim-threads-copy.html'])

    def test_robot_figures_write_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes(
            'imu_sim_robot_figures.py',
            ['imu-sim-frames.html', 'imu-sim-angular-velocity.html',
             'imu-sim-specific-force.html'])

    def test_test_figures_write_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes(
            'imu_sim_test_figures.py',
            ['imu-sim-test-at-rest.html', 'imu-sim-test-left-turn.html',
             'imu-sim-test-reset.html'])


if __name__ == '__main__':
    unittest.main()
