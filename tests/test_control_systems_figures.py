"""Tests for scripts/figures/control_systems_figures.py.

The script writes the three figures of the Introduction to Control Systems page.

Run from the repository root: python -m unittest discover -s tests
"""

import os
from pathlib import Path
import re
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
INCLUDE_NAMES = ['control-systems-agent.html', 'control-systems-closed-loop.html',
                 'control-systems-system.html']
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


# The width of the page's article on 1280 and 1440 px screens, measured in the browser
ARTICLE_WIDTH = 688  # px


class TestClosedLoopLayouts(unittest.TestCase):
    """The closed loop's include holds a wide and a tall layout, and CSS shows one of them.

    Both layouts show the same six numbered signals and share the caption and the title.
    """

    def setUp(self) -> None:
        self.wide = control_systems_figures.wide_closed_loop_figure().svg()
        self.tall = control_systems_figures.closed_loop_figure().svg()
        self.include = (ROOT / 'includes' / 'control-systems-closed-loop.html').read_text(
            encoding='utf-8')
        self.styles = (ROOT / 'includes' / 'control-systems-system.html').read_text(
            encoding='utf-8')

    def test_include_holds_the_wide_layout_then_the_tall_one_and_one_caption(self) -> None:
        self.assertIn('<figure class="cs-figure cs-has-wide" id="closed-loop-figure">',
                      self.include)
        self.assertLess(self.include.index(self.wide), self.include.index(self.tall))
        self.assertEqual(self.include.count('<figcaption>'), 1)

    def test_css_shows_the_wide_layout_only_from_the_wide_screen_width(self) -> None:
        self.assertIn('.cs-fig-wide{display:none}', self.styles)
        self.assertIn(f'@media screen and (min-width:{control_systems_figures.WIDE_FROM}){{'
                      '.md-typeset figure.cs-figure.cs-has-wide{max-width:none}'
                      '.cs-fig-wide{display:block}.cs-fig-tall{display:none}}', self.styles)

    def test_both_layouts_have_the_same_title(self) -> None:
        for svg in (self.wide, self.tall):
            self.assertIn(f'{control_systems_figures.CLOSED_LOOP_TITLE}</title>', svg)

    def test_each_layout_points_its_arrows_to_its_own_arrowhead(self) -> None:
        for name, svg in (('cs-loop-wide', self.wide), ('cs-loop', self.tall)):
            self.assertIn(f'<marker id="{name}-head"', svg)
            self.assertEqual(set(re.findall(r'marker-end="url\(#([\w-]+)\)"', svg)),
                             {f'{name}-head'})

    def test_both_layouts_show_the_six_numbered_signals(self) -> None:
        for svg in (self.wide, self.tall):
            numbers = re.findall(r'class="number"[^>]*>(\d)</text>', svg)
            self.assertEqual(sorted(numbers), ['1', '2', '3', '4', '5', '6'])

    def test_wide_layout_fits_the_pages_article(self) -> None:
        self.assertLessEqual(control_systems_figures.WIDE_WIDTH, ARTICLE_WIDTH)
        self.assertIn(f'viewBox="0 0 {control_systems_figures.WIDE_WIDTH} ', self.wide)


if __name__ == '__main__':
    unittest.main()
