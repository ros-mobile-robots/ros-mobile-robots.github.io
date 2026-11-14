"""Tests for scripts/figures/localization_figures.py.

The script writes the figures of the Localization page as twelve includes.

Run from the repository root: python -m unittest discover -s tests
"""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType
from typing import Dict
import unittest

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / 'scripts' / 'figures' / 'localization_figures.py'
INCLUDE_NAMES = ['localization-covariances.html', 'localization-distance.html',
                 'localization-fused-values.html', 'localization-loop.html',
                 'localization-odom-frame.html', 'localization-stamp-check.html',
                 'localization-time-step.html', 'localization-validator-frame.html',
                 'localization-validator-order.html', 'localization-validator-time.html',
                 'localization-validator-values.html', 'localization-validator-variances.html']
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


def load_script() -> ModuleType:
    """Return the script as a module, without running its main()."""
    specification = importlib.util.spec_from_file_location(SCRIPT.stem, SCRIPT)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the Greek letters of the symbols.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')


class TestLoopFigure(unittest.TestCase):
    """The loop figure's two variants differ only above the filter's inputs.

    The page shows the variants in two tabs and says that the real robot and the simulation
    share everything from the drive controller and the IMU sample validator on.
    """

    def test_loop_variants_match_from_measurement_arrows_on(self) -> None:
        figures = load_script()
        robot = figures.loop_figure(figures.REAL_ROBOT)
        simulation = figures.loop_figure(figures.SIMULATION)
        # What both variants are to share, drawn alone: it starts with the two measurements'
        # arrows into the EKF and ends with the boxes that send velocity commands
        shared = figures.Figure('shared', 'The part that both variants share', robot.height,
                                robot.width)
        figures.filter_and_way_back(shared, figures.NODES_Y + figures.NODES_HEIGHT)
        self.assertLess(len(shared.elements), len(robot.elements))
        self.assertLess(len(shared.elements), len(simulation.elements))
        self.assertEqual(robot.elements[-len(shared.elements):], shared.elements)
        self.assertEqual(simulation.elements[-len(shared.elements):], shared.elements)


if __name__ == '__main__':
    unittest.main()
