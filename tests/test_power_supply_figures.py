"""Tests for scripts/figures/power_supply_figures.py.

The script writes the two power flow figures of the Power Supply page.

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
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import power_supply_figures  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'power_supply_figures.py'
INCLUDE_NAMES = ['power-supply-flow-diffbot.html', 'power-supply-flow-remo.html']
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

    Remo's include has a character outside ASCII, the minus sign.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')


class TestStyles(unittest.TestCase):
    """Every kind of line and every source has its style, so that no line is drawn without one."""

    def setUp(self) -> None:
        self.selectors = [selector for selector, _ in power_supply_figures.RULES]

    def test_every_line_kind_has_a_style_rule(self) -> None:
        for kind in power_supply_figures.LineKind:
            self.assertIn(f'.pf-fig .line.{kind.value}', self.selectors, kind)

    def test_every_source_has_a_style_rule_for_its_supply_line(self) -> None:
        for source in power_supply_figures.Source:
            self.assertIn(f'.pf-fig .line.{source.value}', self.selectors, source)

    def test_every_source_has_a_style_rule_for_its_arrowhead(self) -> None:
        for source in power_supply_figures.Source:
            self.assertIn(f'.pf-fig .head-{source.value}', self.selectors, source)

    def test_every_sources_supply_line_ends_in_a_marker_that_the_figure_defines(self) -> None:
        figure_name, figure_height = 'pf-test', 100
        downward_path = 'M0,0 V10'
        for source in power_supply_figures.Source:
            figure = power_supply_figures.Figure(figure_name, figure_height, 'title')
            figure.supply(downward_path, source)
            marker = f'{figure_name}-{source.value}-head'
            self.assertIn(f'marker-end="url(#{marker})"', figure.svg(), source)
            self.assertIn(f'<marker id="{marker}"', figure.svg(), source)


if __name__ == '__main__':
    unittest.main()
