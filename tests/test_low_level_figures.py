"""Tests for scripts/figures/low_level_figures.py, which writes the Low-Level page's figures.

Run from the repository root: python -m unittest discover -s tests
"""

from collections import Counter
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Counter as CounterType
from typing import Dict
from typing import List
from typing import Set
from typing import Tuple
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import low_level_figures  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'low_level_figures.py'
INCLUDE_NAMES = ['low-level-ports.html', 'low-level-wiring.html']
TEXT = re.compile(r'<text(?: class="([^"]*)")?[^>]*>([^<]*)</text>')
CODE_TEXT = re.compile(r'<text class="code" x="([^"]+)" y="([^"]+)"[^>]*>([^<]*)</text>')
NOTE_TEXT = re.compile(r'<text class="note" x="[^"]+" y="([^"]+)"[^>]*>([^<]*)</text>')
CORE_POLYGON = re.compile(r'<polygon class="core" points="([^"]+)"/>')
# How far below or above its port's label an adapter's name stands in the tall layout, at most
ARROW_REACH = 60
# Which code each port connects to, in each environment, as diffbot_firmware/src/main.cpp and
# test/test_wheel_control/test_wheel_control.cpp at the pinned commit connect them. The tests'
# own box, which calls both primary ports, has no port of its own to pair with
ENVIRONMENT_PAIRS = {
    'On the microcontroller': {('receiveCommand()', 'RosWheelCommandSubscriber'),
                               ('step()', 'loop()'), ('Controller', 'BaseController'),
                               ('LogTimeout', 'RosTimeoutLogger')},
    'In the unit tests': {('Controller', 'FakeBaseController'),
                          ('LogTimeout', 'FakeTimeoutLog')},
}
PORT_PAIRS = set().union(*ENVIRONMENT_PAIRS.values())


def run_script() -> Dict[str, bytes]:
    """Run the script in an empty folder and return the includes it writes, by file name."""
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        run = subprocess.run([sys.executable, str(SCRIPT)], cwd=folder, capture_output=True,
                             text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{SCRIPT.name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


def texts(figure: low_level_figures.Figure, *css_classes: str) -> CounterType[str]:
    """Return how often each text of the given CSS classes appears in a layout."""
    return Counter(text for css_class, text in TEXT.findall(''.join(figure.elements))
                   if css_class in css_classes)


def port_pairs(figure: low_level_figures.Figure, wide: bool) -> Set[Tuple[str, str]]:
    """Return each port's name with the adapter drawn at the other end of its arrow.

    In the wide layout a port's label and its adapter's name share a baseline. In the tall one
    the adapter's name stands centered on the arrow, 6 px left of the port's label: above a
    primary port and below a secondary one, within ARROW_REACH, so that a name in the next
    block doesn't count.
    """
    placed = [(float(x), float(y), text)
              for x, y, text in CODE_TEXT.findall(''.join(figure.elements))]
    ports = low_level_figures.PRIMARY_PORTS + low_level_figures.SECONDARY_PORTS
    names = [(x, y, text) for x, y, text in placed if text not in ports]
    pairs = set()
    for x, y, port in ((x, y, text) for x, y, text in placed if text in ports):
        primary = port in low_level_figures.PRIMARY_PORTS
        if wide:
            matches = [(abs(name_x - x), name) for name_x, name_y, name in names
                       if name_y == y and (name_x < x if primary else name_x > x)]
        else:
            matches = [(abs(name_y - y), name) for name_x, name_y, name in names
                       if name_x == x - 6 and (name_y < y if primary else name_y > y)
                       and abs(name_y - y) <= ARROW_REACH]
        if matches:
            pairs.add((port, min(matches)[1]))
    return pairs


def environment_alone(environment: low_level_figures.Environment,
                      wide: bool) -> low_level_figures.Figure:
    """Return a layout that draws one environment alone, as the wide or the tall layout does."""
    if wide:
        figure = low_level_figures.Figure('test', 'll-ports', low_level_figures.WIDE_WIDTH, 300)
        low_level_figures.wide_environment(figure, environment, 0)
    else:
        figure = low_level_figures.Figure('test', 'll-ports', low_level_figures.TALL_WIDTH, 400)
        low_level_figures.tall_environment(figure, environment, 0)
    return figure


def core_corners(figure: low_level_figures.Figure) -> List[Tuple[float, float]]:
    """Return the corners of the first core hexagon in a layout, from the top clockwise."""
    match = CORE_POLYGON.search(''.join(figure.elements))
    if match is None:
        raise ValueError('the layout draws no core')
    return [(float(x), float(y)) for x, y in
            (corner.split(',') for corner in match.group(1).split())]


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, so that they show what the script draws."""

    def test_script_writes_the_repository_includes(self) -> None:
        written = run_script()
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for name in INCLUDE_NAMES:
            with self.subTest(include=name):
                self.assertEqual(written[name], (ROOT / 'includes' / name).read_bytes(),
                                 f'{name} is not what {SCRIPT.name} writes')

    def test_each_include_starts_with_a_comment_that_names_the_script(self) -> None:
        for name, content in run_script().items():
            with self.subTest(include=name):
                first_line = content.decode('utf-8').split('\n', 1)[0]
                self.assertIn('scripts/figures/low_level_figures.py', first_line)
                self.assertTrue(first_line.startswith('<!--'))


class TestLayouts(unittest.TestCase):
    """The wide and the tall layout of a figure tell the same: a phone shows no less."""

    def test_both_ports_layouts_name_the_same_ports_and_adapters(self) -> None:
        wide, tall = low_level_figures.wide_ports(), low_level_figures.tall_ports()
        self.assertEqual(texts(wide, 'code', 'core-name'), texts(tall, 'code', 'core-name'))

    def test_each_port_leads_to_its_adapter_in_both_layouts(self) -> None:
        self.assertEqual(port_pairs(low_level_figures.wide_ports(), wide=True), PORT_PAIRS)
        self.assertEqual(port_pairs(low_level_figures.tall_ports(), wide=False), PORT_PAIRS)

    def test_each_environment_shows_its_own_adapters_in_both_layouts(self) -> None:
        for environment in low_level_figures.ENVIRONMENTS:
            for wide in (True, False):
                with self.subTest(environment=environment.title, wide=wide):
                    figure = environment_alone(environment, wide)
                    self.assertEqual(port_pairs(figure, wide),
                                     ENVIRONMENT_PAIRS[environment.title])

    def test_the_core_note_stands_between_the_upright_edges_in_the_wide_layout(self) -> None:
        figure = environment_alone(low_level_figures.ENVIRONMENTS[0], wide=True)
        upper_corner_y, lower_corner_y = core_corners(figure)[1][1], core_corners(figure)[2][1]
        notes = {text: float(y) for y, text in NOTE_TEXT.findall(''.join(figure.elements))}
        self.assertGreater(notes['and its CommandTimeout'], upper_corner_y)
        self.assertLess(notes['and its CommandTimeout'], lower_corner_y)

    def test_both_wiring_layouts_carry_the_same_labels(self) -> None:
        wide, tall = low_level_figures.wide_wiring(), low_level_figures.tall_wiring()
        classes = ('code', 'name', 'note', 'wire-label')
        self.assertEqual(texts(wide, *classes), texts(tall, *classes))

    def test_the_wiring_names_each_motors_output_and_encoder_pins(self) -> None:
        # Remo's settings in diffbot_firmware/lib/config/diffbot_base_config.h, lines 9-21, at
        # the commit that mkdocs.yml pins as low_level_src_url. Compare them again on a repin
        labels = texts(low_level_figures.wide_wiring(), 'note')
        self.assertIn('left: M4, pins 5 and 6', labels)
        self.assertIn('right: M3, pins 7 and 8', labels)


if __name__ == '__main__':
    unittest.main()
