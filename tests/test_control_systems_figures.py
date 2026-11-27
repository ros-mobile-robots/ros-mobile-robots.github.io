"""Tests for scripts/figures/control_systems_figures.py.

The script writes the figures of the Introduction to Control Systems page.

Run from the repository root: python -m unittest discover -s tests
"""

import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
import unittest
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import control_systems_figures  # noqa: E402
from control_systems_figures import LineKind  # noqa: E402
from control_systems_figures import Point  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'control_systems_figures.py'
INCLUDE_NAMES = ['control-systems-agent.html', 'control-systems-closed-loop.html',
                 'control-systems-open-loop.html', 'control-systems-stability.html',
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


# The width of Roboto's characters as a share of the font size, by width: the regular weight,
# measured in Chromium with the font files that the site serves. A bold text is BOLD_FACTOR wider
ROBOTO_WIDTHS = {
    0.175: "'", 0.197: ',', 0.211: ';', 0.239: 'j', 0.242: ':', 0.243: 'il', 0.248: ' ',
    0.264: '.', 0.272: 'I', 0.276: '-', 0.327: 't', 0.339: 'r', 0.342: '(', 0.348: 'f)',
    0.413: '/', 0.473: 'y', 0.484: 'v', 0.496: 'xz', 0.507: 'k', 0.516: 's', 0.523: 'c',
    0.53: 'e', 0.539: 'L', 0.544: 'a', 0.551: 'hu', 0.552: 'nJ', 0.553: 'F',
    0.562: 'bgp0123456789', 0.564: 'd', 0.567: '+', 0.568: 'qE', 0.57: 'o', 0.572: '\u2212',
    0.594: 'S', 0.597: 'T', 0.599: 'Z', 0.601: 'Y', 0.616: 'R', 0.623: 'B', 0.627: 'KX',
    0.631: 'P', 0.637: 'V', 0.648: 'U', 0.651: 'C', 0.652: 'A', 0.656: 'D', 0.681: 'G',
    0.688: 'OQ', 0.713: 'HN', 0.732: '%', 0.751: 'w', 0.873: 'M', 0.877: 'm', 0.887: 'W',
}
CHARACTER_WIDTHS = {character: width for width, characters in ROBOTO_WIDTHS.items()
                    for character in characters}
BOLD_FACTOR = 1.05
# How far a text reaches above and below its baseline, as a share of the font size: the height
# of a letter such as "d", and the depth of a letter such as "p"
ASCENT = 0.75
DESCENT = 0.21


class Area(NamedTuple):
    """A rectangle in a drawing, in the drawing's units: a box, or the area a text takes up."""

    left: float
    top: float
    right: float
    bottom: float

    def holds(self, other: 'Area') -> bool:
        """Return whether the other area lies inside this one."""
        return (self.left <= other.left and other.right <= self.right
                and self.top <= other.top and other.bottom <= self.bottom)

    def holds_the_middle_of(self, other: 'Area') -> bool:
        """Return whether the middle of the other area lies inside this one."""
        middle_x = (other.left + other.right) / 2
        middle_y = (other.top + other.bottom) / 2
        return self.left < middle_x < self.right and self.top < middle_y < self.bottom

    def is_touched_at(self, x: float, y: float) -> bool:
        """Return whether a point lies on the edge of this area, or up to 1 px before it."""
        on_an_upright_edge = (min(abs(x - self.left), abs(x - self.right)) <= 1
                              and self.top <= y <= self.bottom)
        on_a_level_edge = (min(abs(y - self.top), abs(y - self.bottom)) <= 1
                           and self.left <= x <= self.right)
        return on_an_upright_edge or on_a_level_edge

    def overlaps(self, other: 'Area') -> bool:
        """Return whether the two areas share more than an edge."""
        return (self.left < other.right and other.left < self.right
                and self.top < other.bottom and other.top < self.bottom)


class DrawnText(NamedTuple):
    """A text of a drawing and the area that it takes up, estimated from Roboto's widths."""

    text: str
    area: Area


class Segment(NamedTuple):
    """A straight piece of a drawn line, from (x1, y1) to (x2, y2)."""

    x1: float
    y1: float
    x2: float
    y2: float


def css_classes(element: ElementTree.Element) -> List[str]:
    """Return the CSS classes of an SVG element."""
    return element.get('class', '').split()


def declaration_of(css_class: str, name: str) -> Optional[str]:
    """Return the value that the script's style rule for a class declares, if it has one.

    The rule for an empty class is the one for every text of a figure.
    """
    selector = f'.cs-fig .{css_class}' if css_class else '.cs-fig text'
    for rule_selector, declarations in control_systems_figures.RULES:
        match = re.search(rf'(?:^|;){name}:([^;]+)', declarations)
        if rule_selector == selector and match:
            return match.group(1)
    return None


def font_size(classes: List[str]) -> float:
    """Return the font size of a text with these classes, in px, as the style rules set it."""
    sizes = [declaration_of(css_class, 'font-size') for css_class in classes]
    size = next((size for size in sizes if size), declaration_of('', 'font-size'))
    assert size is not None
    return float(size.removesuffix('px'))


def text_width(text: str, classes: List[str]) -> float:
    """Return the estimated width of a text with these classes, in px."""
    width = sum(CHARACTER_WIDTHS[character] for character in text) * font_size(classes)
    bold = any(declaration_of(css_class, 'font-weight') == '700' for css_class in classes)
    return width * BOLD_FACTOR if bold else width


def drawn_texts(svg: str) -> List[DrawnText]:
    """Return the texts of a drawing with the area that each one takes up."""
    texts = []
    for element in ElementTree.fromstring(svg).iter('text'):
        text, classes = element.text or '', css_classes(element)
        width, size = text_width(text, classes), font_size(classes)
        x, baseline = float(element.get('x', 0)), float(element.get('y', 0))
        before_x = {'start': 0, 'middle': width / 2, 'end': width}
        left = x - before_x[element.get('text-anchor', 'start')]
        texts.append(DrawnText(text, Area(left, baseline - ASCENT * size, left + width,
                                          baseline + DESCENT * size)))
    return texts


def texts_of(svg: str) -> List[str]:
    """Return the texts of a drawing, in the order in which the drawing holds them."""
    return [drawn.text for drawn in drawn_texts(svg)]


def drawn_boxes(svg: str) -> List[Area]:
    """Return the area of each of a drawing's blocks."""
    boxes = []
    for rect in ElementTree.fromstring(svg).iter('rect'):
        if 'block' in css_classes(rect):
            left, top = float(rect.get('x', 0)), float(rect.get('y', 0))
            boxes.append(Area(left, top, left + float(rect.get('width', 0)),
                              top + float(rect.get('height', 0))))
    return boxes


def path_segments(path: str) -> List[Segment]:
    """Return the straight pieces of an SVG path made of absolute M, L, H and V commands."""
    segments: List[Segment] = []
    x = y = 0.0
    for command, numbers in re.findall(r'([MLHV])([^MLHV]*)', path):
        values = [float(value) for value in re.split(r'[ ,]+', numbers.strip())]
        if command == 'M':
            x, y = values
            continue
        if command == 'H':
            new_x, new_y = values[0], y
        elif command == 'V':
            new_x, new_y = x, values[0]
        else:
            new_x, new_y = values
        segments.append(Segment(x, y, new_x, new_y))
        x, y = new_x, new_y
    return segments


def arrow_segments(svg: str) -> List[Segment]:
    """Return the straight pieces of a drawing's arrows, in the order in which they are drawn."""
    return [segment for path in ElementTree.fromstring(svg).iter('path')
            if 'arrow' in css_classes(path) for segment in path_segments(path.get('d', ''))]


def drawn_lines(svg: str, kind: LineKind) -> List[List[Point]]:
    """Return the points of each of a drawing's lines of one kind, in the order of the lines."""
    return [[Point(*(float(number) for number in pair.split(',')))
             for pair in line.get('points', '').split()]
            for line in ElementTree.fromstring(svg).iter('polyline')
            if kind.value in css_classes(line)]


def points_along(line: List[Point]) -> List[Point]:
    """Return points on a drawn line that are at most 1 px apart, its own points included."""
    points = line[:1]
    for start, end in zip(line, line[1:]):
        steps = math.ceil(math.hypot(end.x - start.x, end.y - start.y)) or 1
        points += [Point(start.x + (end.x - start.x) * step / steps,
                         start.y + (end.y - start.y) * step / steps)
                   for step in range(1, steps + 1)]
    return points


def highest_y(line: List[Point]) -> float:
    """Return the y coordinate of a line's highest point. In a drawing, y grows downwards."""
    return min(point.y for point in line)


def height_at_its_end(line: List[Point]) -> float:
    """Return how far a signal's line ends above its lowest point, the value 0 on its time axis."""
    return max(point.y for point in line) - line[-1].y


def segments_against_the_first_arrow(svg: str) -> List[Segment]:
    """Return the pieces of a drawing's arrows that run against the direction of its first arrow.

    The first arrow of a loop's drawing is the reference's, which runs along the forward path.
    """
    segments = arrow_segments(svg)
    forward = segments[0]
    return [segment for segment in segments
            if ((segment.x2 - segment.x1) * (forward.x2 - forward.x1)
                + (segment.y2 - segment.y1) * (forward.y2 - forward.y1)) < 0]


def every_layout() -> Dict[str, str]:
    """Return every layout of every figure that the script draws, as SVG, by the layout's name."""
    figures = [control_systems_figures.system_figure(),
               control_systems_figures.wide_open_loop_figure(),
               control_systems_figures.open_loop_figure(),
               control_systems_figures.wide_stability_figure(),
               control_systems_figures.stability_figure(),
               control_systems_figures.wide_closed_loop_figure(),
               control_systems_figures.closed_loop_figure(),
               control_systems_figures.agent_figure()]
    return {figure.name: figure.svg() for figure in figures}


def viewbox(svg: str) -> Area:
    """Return the area of a drawing that its viewBox shows."""
    left, top, width, height = (float(value) for value in
                                ElementTree.fromstring(svg).get('viewBox', '').split())
    return Area(left, top, left + width, top + height)


class TestEveryDrawing(unittest.TestCase):
    """Every layout keeps its texts inside the drawing, inside their boxes and apart."""

    def test_every_text_stays_inside_its_drawing(self) -> None:
        for name, svg in every_layout().items():
            outside = [drawn.text for drawn in drawn_texts(svg)
                       if not viewbox(svg).holds(drawn.area)]
            self.assertEqual(outside, [], name)

    def test_every_text_in_a_box_stays_inside_the_box(self) -> None:
        for name, svg in every_layout().items():
            leaving = [drawn.text for drawn in drawn_texts(svg) for box in drawn_boxes(svg)
                       if box.holds_the_middle_of(drawn.area) and not box.holds(drawn.area)]
            self.assertEqual(leaving, [], name)

    def test_no_two_texts_overlap(self) -> None:
        for name, svg in every_layout().items():
            texts = drawn_texts(svg)
            overlapping = [(first.text, second.text) for index, first in enumerate(texts)
                           for second in texts[index + 1:] if first.area.overlaps(second.area)]
            self.assertEqual(overlapping, [], name)

    def test_every_arrow_stays_inside_its_drawing(self) -> None:
        for name, svg in every_layout().items():
            frame = viewbox(svg)
            outside = [segment for segment in arrow_segments(svg)
                       if not frame.holds(Area(min(segment.x1, segment.x2),
                                               min(segment.y1, segment.y2),
                                               max(segment.x1, segment.x2),
                                               max(segment.y1, segment.y2)))]
            self.assertEqual(outside, [], name)

    def test_every_line_of_a_plot_stays_inside_its_drawing(self) -> None:
        for name, svg in every_layout().items():
            frame = viewbox(svg)
            outside = [point for kind in LineKind for line in drawn_lines(svg, kind)
                       for point in line if not frame.holds(Area(*point, *point))]
            self.assertEqual(outside, [], name)

    def test_no_line_of_a_plot_crosses_a_text(self) -> None:
        for name, svg in every_layout().items():
            texts = drawn_texts(svg)
            points = [point for kind in LineKind for line in drawn_lines(svg, kind)
                      for point in points_along(line)]
            crossed = {drawn.text for drawn in texts for point in points
                       if drawn.area.holds(Area(*point, *point))}
            self.assertEqual(crossed, set(), name)


class TestLineKinds(unittest.TestCase):
    """Each kind of line in the plots has a look of its own that doesn't rest on a color."""

    def test_every_line_kind_has_a_style_rule_with_a_stroke(self) -> None:
        for kind in LineKind:
            self.assertIsNotNone(declaration_of(kind.value, 'stroke'), kind)

    def test_no_two_line_kinds_share_their_width_and_their_pattern(self) -> None:
        looks = [(declaration_of(kind.value, 'stroke-width'),
                  declaration_of(kind.value, 'stroke-dasharray')) for kind in LineKind]
        self.assertEqual(len(set(looks)), len(looks), looks)

    def test_only_the_signals_line_is_solid(self) -> None:
        solid = [kind for kind in LineKind
                 if declaration_of(kind.value, 'stroke-dasharray') is None]
        self.assertEqual(solid, [LineKind.SIGNAL])


class TestPlot(unittest.TestCase):
    """A plot maps times and values to points of its area, and samples a signal along it."""

    def setUp(self) -> None:
        self.plot = control_systems_figures.Plot(left=40, top=20, width=300, height=120,
                                                 duration=1.5, highest=2.0)

    def test_time_zero_and_value_zero_are_at_the_lower_left_corner(self) -> None:
        self.assertEqual(self.plot.point(0, 0), Point(self.plot.left, self.plot.bottom))

    def test_the_duration_and_the_highest_value_are_at_the_upper_right_corner(self) -> None:
        self.assertEqual(self.plot.point(self.plot.duration, self.plot.highest),
                         Point(self.plot.right, self.plot.top))

    def test_half_the_highest_value_is_halfway_up(self) -> None:
        halfway_up = self.plot.top + self.plot.height / 2
        self.assertEqual(self.plot.y_at(self.plot.highest / 2), halfway_up)

    def test_sampled_line_runs_from_time_zero_to_the_end_time(self) -> None:
        end_time = 0.9
        line = control_systems_figures.sampled(self.plot, math.sin, end_time)
        self.assertEqual(line[0], self.plot.point(0, math.sin(0)))
        self.assertEqual(line[-1], self.plot.point(end_time, math.sin(end_time)))

    def test_sampled_lines_points_are_no_further_apart_than_the_sample_step(self) -> None:
        line = control_systems_figures.sampled(self.plot, math.sin, self.plot.duration)
        rounding = 0.1
        for earlier, later in zip(line, line[1:]):
            self.assertLessEqual(later.x - earlier.x,
                                 control_systems_figures.SAMPLE_STEP + rounding)


class TestResponses(unittest.TestCase):
    """The example systems answer a step of their input as their formulas say."""

    def test_step_response_is_zero_until_the_step(self) -> None:
        before_the_step = -0.2
        for time in (before_the_step, 0):
            self.assertEqual(
                control_systems_figures.step_response(control_systems_figures.SETTLING, time), 0)

    def test_step_response_peaks_at_the_textbook_overshoot(self) -> None:
        system = control_systems_figures.SecondOrder(natural_frequency=8, damping_ratio=0.5)
        damped = math.sqrt(1 - system.damping_ratio ** 2)
        peak_time = math.pi / (system.natural_frequency * damped)
        overshoot = math.exp(-math.pi * system.damping_ratio / damped)
        self.assertAlmostEqual(control_systems_figures.step_response(system, peak_time),
                               1 + overshoot)

    def test_step_response_a_quarter_swing_after_the_step_has_the_textbook_value(self) -> None:
        # There the cosine part is zero, and only the sine part is left
        system = control_systems_figures.SecondOrder(natural_frequency=8, damping_ratio=0.5)
        damped = math.sqrt(1 - system.damping_ratio ** 2)
        quarter_swing = math.pi / 2 / (system.natural_frequency * damped)
        faded = math.exp(-system.damping_ratio * system.natural_frequency * quarter_swing)
        self.assertAlmostEqual(control_systems_figures.step_response(system, quarter_swing),
                               1 - faded * system.damping_ratio / damped)

    def test_step_response_settles_at_one(self) -> None:
        long_after_the_step = 10
        self.assertAlmostEqual(control_systems_figures.step_response(
            control_systems_figures.SETTLING, long_after_the_step), 1)

    def test_runaway_response_is_zero_until_the_step(self) -> None:
        before_the_step = -0.2
        for time in (before_the_step, 0):
            self.assertEqual(control_systems_figures.runaway_response(time), 0)

    def test_runaway_response_grows_by_the_same_factor_in_equal_times(self) -> None:
        # The output plus 1 is an exponential function of time
        interval = 0.25
        grown = [control_systems_figures.runaway_response(count * interval) + 1
                 for count in (1, 2, 3)]
        self.assertAlmostEqual(grown[1] / grown[0], grown[2] / grown[1])
        self.assertGreater(grown[1] / grown[0], 1)

    def test_runaway_time_is_when_the_runaway_response_reaches_the_value(self) -> None:
        value = 2.2
        reached_at = control_systems_figures.runaway_time(value)
        self.assertAlmostEqual(control_systems_figures.runaway_response(reached_at), value)


class StabilityLines(NamedTuple):
    """The lines of one layout of the stability figure."""

    input_bound: List[Point]
    input_signal: List[Point]
    output_bound: List[Point]
    stable_output: List[Point]
    unstable_output: List[Point]


def stability_lines(svg: str) -> StabilityLines:
    """Return the lines of a layout of the stability figure: the input's plot is drawn first."""
    input_bound, output_bound = drawn_lines(svg, LineKind.BOUND)
    input_signal, stable_output = drawn_lines(svg, LineKind.SIGNAL)
    unstable_output, = drawn_lines(svg, LineKind.UNSTABLE)
    return StabilityLines(input_bound, input_signal, output_bound, stable_output,
                          unstable_output)


# How far two coordinates of a drawing may differ and still count as the same: they are rounded
# to a tenth of a px
ROUNDING = 0.11


class TestStability(unittest.TestCase):
    """Both layouts show a bounded input, a stable output that stays bounded and an unstable one.

    The unstable output passes its bound and keeps growing.
    """

    def setUp(self) -> None:
        self.wide = control_systems_figures.wide_stability_figure().svg()
        self.tall = control_systems_figures.stability_figure().svg()
        self.layouts = (stability_lines(self.wide), stability_lines(self.tall))
        self.include = (ROOT / 'includes' / 'control-systems-stability.html').read_text(
            encoding='utf-8')

    def test_include_holds_the_wide_layout_then_the_tall_one_and_one_caption(self) -> None:
        self.assertIn('<figure class="cs-figure cs-has-wide" id="stability-figure">',
                      self.include)
        self.assertLess(self.include.index(self.wide), self.include.index(self.tall))
        self.assertEqual(self.include.count('<figcaption>'), 1)

    def test_caption_says_that_the_curves_are_no_measurement(self) -> None:
        self.assertIn('an example, not a measurement of the robot', self.include)

    def test_each_bound_is_a_level_line(self) -> None:
        for lines in self.layouts:
            for bound in (lines.input_bound, lines.output_bound):
                self.assertEqual(len({point.y for point in bound}), 1)

    def test_input_steps_up_once_and_stays_below_its_bound(self) -> None:
        for lines in self.layouts:
            levels = [point.y for point in lines.input_signal]
            self.assertEqual(levels, sorted(levels, reverse=True))
            self.assertEqual(len(set(levels)), 2)
            self.assertGreater(highest_y(lines.input_signal), lines.input_bound[0].y)

    def test_stable_output_stays_below_its_bound(self) -> None:
        for lines in self.layouts:
            self.assertGreater(highest_y(lines.stable_output), lines.output_bound[0].y)

    def test_stable_output_settles_at_the_inputs_level(self) -> None:
        for lines in self.layouts:
            self.assertAlmostEqual(height_at_its_end(lines.stable_output),
                                   height_at_its_end(lines.input_signal), delta=1)
            last_tenth = lines.stable_output[-len(lines.stable_output) // 10:]
            self.assertLess(max(point.y for point in last_tenth)
                            - min(point.y for point in last_tenth), 1)

    def test_unstable_output_starts_below_its_bound_and_ends_above_it(self) -> None:
        for lines in self.layouts:
            self.assertGreater(lines.unstable_output[0].y, lines.output_bound[0].y)
            self.assertLess(lines.unstable_output[-1].y, lines.output_bound[0].y)

    def test_unstable_output_ends_above_every_point_of_the_stable_output(self) -> None:
        for lines in self.layouts:
            self.assertLess(lines.unstable_output[-1].y, highest_y(lines.stable_output))

    def test_unstable_output_never_falls_and_rises_ever_faster(self) -> None:
        for lines in self.layouts:
            rises = [earlier.y - later.y for earlier, later
                     in zip(lines.unstable_output, lines.unstable_output[1:])]
            self.assertGreaterEqual(min(rises), 0)
            for earlier_rise, later_rise in zip(rises, rises[1:]):
                self.assertGreaterEqual(later_rise, earlier_rise - 2 * ROUNDING)
            self.assertGreater(rises[-1], rises[len(rises) // 2])

    def test_unstable_output_is_still_rising_where_its_line_ends(self) -> None:
        for lines in self.layouts:
            self.assertLess(lines.unstable_output[-1].y, lines.unstable_output[-2].y)
            self.assertLess(lines.unstable_output[-1].x, lines.output_bound[-1].x)

    def test_both_outputs_start_to_rise_where_the_input_steps_in_the_tall_layout(self) -> None:
        lines = self.layouts[1]
        step_x = next(point.x for point in lines.input_signal
                      if point.y < lines.input_signal[0].y)
        for output in (lines.stable_output, lines.unstable_output):
            rises_from = max(point.x for point in output if point.y == output[0].y)
            self.assertAlmostEqual(rises_from, step_x,
                                   delta=control_systems_figures.SAMPLE_STEP + ROUNDING)

    def test_both_bounds_are_at_the_same_height_in_the_wide_layout(self) -> None:
        lines = self.layouts[0]
        self.assertEqual(lines.input_bound[0].y, lines.output_bound[0].y)

    def test_both_layouts_name_the_plots_the_bounds_and_the_two_outputs(self) -> None:
        for svg in (self.wide, self.tall):
            texts = texts_of(svg)
            for text in ('input', 'target wheel speed', 'output', 'wheel speed', 'System',
                         'stable:', 'stays bounded', 'unstable:', 'keeps growing'):
                self.assertEqual(texts.count(text), 1, text)
            self.assertEqual(texts.count('bound'), 2)
            self.assertEqual(texts.count('time'), 2)

    def test_the_unstable_outputs_name_stands_next_to_the_end_of_its_line(self) -> None:
        for svg, lines in zip((self.wide, self.tall), self.layouts):
            name = next(drawn for drawn in drawn_texts(svg) if drawn.text == 'unstable:')
            line_end = lines.unstable_output[-1]
            nearest = 24
            self.assertLess(math.hypot(name.area.left - line_end.x, name.area.top - line_end.y),
                            nearest)

    def test_the_stable_outputs_name_stands_right_below_the_end_of_its_line(self) -> None:
        for svg, lines in zip((self.wide, self.tall), self.layouts):
            name = next(drawn for drawn in drawn_texts(svg) if drawn.text == 'stable:')
            line_end = lines.stable_output[-1]
            nearest = 12
            self.assertEqual(round(name.area.right), round(line_end.x))
            self.assertLess(name.area.top - line_end.y, nearest)
            self.assertGreater(name.area.top, line_end.y)


# The open loop's chain, from its first signal to its last
OPEN_LOOP_CHAIN = ['reference', 'Controller', 'commands', 'Actuators', 'inputs',
                   'Process and environment', 'outputs']
# What only a closed loop has
FEEDBACK_TEXTS = ['Sensors', 'observations', 'error', '+', '\u2212']


class TestOpenLoop(unittest.TestCase):
    """The open loop's two layouts show the chain to the outputs, and no path back."""

    def setUp(self) -> None:
        self.wide = control_systems_figures.wide_open_loop_figure().svg()
        self.tall = control_systems_figures.open_loop_figure().svg()
        self.include = (ROOT / 'includes' / 'control-systems-open-loop.html').read_text(
            encoding='utf-8')

    def test_include_holds_the_wide_layout_then_the_tall_one_and_one_caption(self) -> None:
        self.assertIn('<figure class="cs-figure cs-has-wide" id="open-loop-figure">',
                      self.include)
        self.assertLess(self.include.index(self.wide), self.include.index(self.tall))
        self.assertEqual(self.include.count('<figcaption>'), 1)

    def test_both_layouts_show_the_chain_from_the_reference_to_the_outputs(self) -> None:
        for svg in (self.wide, self.tall):
            chain = [text for text in texts_of(svg) if text in OPEN_LOOP_CHAIN]
            self.assertEqual(chain, OPEN_LOOP_CHAIN)

    def test_neither_layout_shows_sensors_observations_or_a_comparison(self) -> None:
        for svg in (self.wide, self.tall):
            self.assertEqual([text for text in texts_of(svg) if text in FEEDBACK_TEXTS], [])
            self.assertEqual(list(ElementTree.fromstring(svg).iter('circle')), [])

    def test_no_arrow_of_either_layout_runs_back(self) -> None:
        for svg in (self.wide, self.tall):
            self.assertEqual(segments_against_the_first_arrow(svg), [])

    def test_each_arrow_starts_where_the_chain_has_got_to(self) -> None:
        for svg in (self.wide, self.tall):
            segments = arrow_segments(svg)
            for earlier, later in zip(segments, segments[1:]):
                self.assertGreaterEqual(later.x1, earlier.x2)
                self.assertGreaterEqual(later.y1, earlier.y2)

    def test_every_arrow_leads_into_a_box_except_the_outputs(self) -> None:
        for svg in (self.wide, self.tall):
            ends_at_a_box = [any(box.is_touched_at(segment.x2, segment.y2)
                                 for box in drawn_boxes(svg))
                             for segment in arrow_segments(svg)]
            self.assertEqual(ends_at_a_box, [True, True, True, False])

    def test_every_arrow_leaves_a_box_except_the_reference(self) -> None:
        for svg in (self.wide, self.tall):
            starts_at_a_box = [any(box.is_touched_at(segment.x1, segment.y1)
                                   for box in drawn_boxes(svg))
                               for segment in arrow_segments(svg)]
            self.assertEqual(starts_at_a_box, [False, True, True, True])

    def test_closed_loop_has_arrows_that_run_back(self) -> None:
        closed_layouts = (control_systems_figures.wide_closed_loop_figure().svg(),
                          control_systems_figures.closed_loop_figure().svg())
        for svg in closed_layouts:
            self.assertNotEqual(segments_against_the_first_arrow(svg), [])

    def test_open_loop_differs_from_the_closed_loop_only_in_the_controllers_example(
            self) -> None:
        closed_texts = texts_of(control_systems_figures.wide_closed_loop_figure().svg())
        for svg in (self.wide, self.tall):
            own_texts = [text for text in texts_of(svg) if text not in closed_texts]
            self.assertEqual(own_texts, ['plan set in advance'])


if __name__ == '__main__':
    unittest.main()
