"""Tests for scripts/figures/imu_noise_figures.py.

The script writes the figures about the IMU's bias and noise at rest as seven includes.

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
FIGURES = ROOT / 'scripts' / 'figures'
SCRIPT = FIGURES / 'imu_noise_figures.py'
INCLUDE_NAMES = ['imu-noise-test-gravity.html', 'imu-noise-test-intervals.html',
                 'imu-noise-test-measured.html', 'imu-noise-test-stamps-back.html',
                 'imu-noise-test-variance.html', 'imu-noise-three-readings.html',
                 'imu-noise-trace.html']
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
    """Return the script as a module, without running its main part.

    The script imports localization_figures.py from its own folder, which Python finds for a
    script that it runs, and which this function adds for the import.
    """
    sys.path.insert(0, str(FIGURES))
    try:
        specification = importlib.util.spec_from_file_location(SCRIPT.stem, SCRIPT)
        assert specification is not None and specification.loader is not None
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
    finally:
        sys.path.remove(str(FIGURES))
    return module


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the minus sign and the Greek delta.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')


class TestSyntheticTrace(unittest.TestCase):
    """The trace figure's readings are a synthetic recording: 180 s at 50 Hz on output steps.

    The page's caption gives the recording's mean and standard deviation, so the recording has
    to be the same in every run.
    """

    def test_synthetic_trace_has_180_s_of_readings_on_whole_output_steps(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        self.assertEqual(len(trace.readings), 180 * 50)
        for reading in trace.readings:
            steps = reading / figures.GYROSCOPE_STEP
            self.assertAlmostEqual(steps, round(steps))

    def test_synthetic_trace_is_the_same_in_every_run(self) -> None:
        figures = load_script()
        self.assertEqual(figures.synthetic_trace(), figures.synthetic_trace())


if __name__ == '__main__':
    unittest.main()
