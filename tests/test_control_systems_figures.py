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
from typing import Tuple
import unittest
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import control_systems_figures  # noqa: E402
from control_systems_figures import LineKind  # noqa: E402
from control_systems_figures import Point  # noqa: E402

SCRIPT = ROOT / 'scripts' / 'figures' / 'control_systems_figures.py'
INCLUDE_NAMES = ['control-systems-agent.html', 'control-systems-closed-loop.html',
                 'control-systems-open-loop.html', 'control-systems-performance.html',
                 'control-systems-robustness.html', 'control-systems-stability.html',
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
# measured in Chromium with the font files that the site serves. A bold text is BOLD_FACTOR
# wider. A character that the table lacks counts as wide as the widest one: an estimate that
# is too wide can fail a test, which then asks for the character's width, but it can't hide a
# text that is too long
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
WIDEST_CHARACTER = max(ROBOTO_WIDTHS)
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
    width = sum(CHARACTER_WIDTHS.get(character, WIDEST_CHARACTER)
                for character in text) * font_size(classes)
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


def drawn_rectangles(svg: str, css_class: str) -> List[Area]:
    """Return the area of each of a drawing's rectangles with a class."""
    areas = []
    for rect in ElementTree.fromstring(svg).iter('rect'):
        if css_class in css_classes(rect):
            left, top = float(rect.get('x', 0)), float(rect.get('y', 0))
            areas.append(Area(left, top, left + float(rect.get('width', 0)),
                              top + float(rect.get('height', 0))))
    return areas


def drawn_boxes(svg: str) -> List[Area]:
    """Return the area of each of a drawing's blocks, in the order in which they are drawn."""
    return drawn_rectangles(svg, 'block')


class Labelled(NamedTuple):
    """A term of a drawing, a signal or a box, with the wheel example's words for it."""

    term: str
    example: str


def labelled_terms(svg: str) -> List[Labelled]:
    """Return each term of a drawing that has an example, with its example, in drawing order.

    A drawing holds an example right after the term that it belongs to.
    """
    pairs = []
    term = ''
    for element in ElementTree.fromstring(svg).iter('text'):
        if 'example' in css_classes(element):
            pairs.append(Labelled(term, element.text or ''))
        term = element.text or ''
    return pairs


def drawn_numbers(svg: str) -> List[Tuple[int, Point]]:
    """Return each number that a drawing shows in a disc, with the disc's center."""
    root = ElementTree.fromstring(svg)
    centers = [Point(float(circle.get('cx', 0)), float(circle.get('cy', 0)))
               for circle in root.iter('circle') if 'number-disc' in css_classes(circle)]
    numbers = [int(text.text or '') for text in root.iter('text')
               if 'number' in css_classes(text)]
    return list(zip(numbers, centers))


def drawn_dimensions(svg: str) -> List[Tuple[Point, Point]]:
    """Return the two ends of each of a drawing's dimension lines, in the order of the lines.

    A dimension line is drawn as two arrows that leave one point in opposite directions.
    """
    arrows = [path_segments(path.get('d', ''))[0]
              for path in ElementTree.fromstring(svg).iter('path')
              if 'arrow' in css_classes(path) and 'L' in path.get('d', '')]
    return [(Point(first.x2, first.y2), Point(second.x2, second.y2))
            for first, second in zip(arrows[::2], arrows[1::2])]


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


def y_on(line: List[Point], x: float) -> float:
    """Return the y coordinate of a signal's line at an x coordinate, between its points."""
    for start, end in zip(line, line[1:]):
        if start.x <= x <= end.x and end.x > start.x:
            return start.y + (end.y - start.y) * (x - start.x) / (end.x - start.x)
    raise ValueError(f'the line has no point at x = {x}')


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
               control_systems_figures.wide_performance_figure(),
               control_systems_figures.performance_figure(),
               control_systems_figures.wide_robustness_figure(),
               control_systems_figures.robustness_figure(),
               control_systems_figures.wide_closed_loop_figure(),
               control_systems_figures.closed_loop_figure(),
               control_systems_figures.agent_figure()]
    return {figure.name: figure.svg() for figure in figures}


def viewbox(svg: str) -> Area:
    """Return the area of a drawing that its viewBox shows."""
    left, top, width, height = (float(value) for value in
                                ElementTree.fromstring(svg).get('viewBox', '').split())
    return Area(left, top, left + width, top + height)


# The labels that each box of each layout has to hold, box by box in the order of drawing
ACTUATORS_BOX = ['Actuators', 'motor driver and motor']
PROCESS_BOX = ['Process and environment', 'wheel, robot and ground']
SENSORS_BOX = ['Sensors', 'wheel encoder']
OPEN_LOOP_BOXES = [['Controller', 'plan set in advance'], ACTUATORS_BOX, PROCESS_BOX]
CLOSED_LOOP_BOXES = [['Controller', 'PID in the firmware'], ACTUATORS_BOX, PROCESS_BOX,
                     SENSORS_BOX]
STABILITY_BOXES = [['System', 'wheel with its controller']]
BOX_LABELS: Dict[str, List[List[str]]] = {
    'cs-system': [ACTUATORS_BOX, PROCESS_BOX, SENSORS_BOX],
    'cs-open-wide': OPEN_LOOP_BOXES, 'cs-open': OPEN_LOOP_BOXES,
    'cs-stability-wide': STABILITY_BOXES, 'cs-stability': STABILITY_BOXES,
    'cs-performance-wide': [], 'cs-performance': [],
    'cs-robustness-wide': [], 'cs-robustness': [],
    'cs-loop-wide': CLOSED_LOOP_BOXES, 'cs-loop': CLOSED_LOOP_BOXES,
    'cs-agent': [['Sensors'], ['Perception'], ['Planner'], ['Controller'], ['Actuators']],
}


class TestTextWidth(unittest.TestCase):
    """The tests' estimate of a text's width uses the measured widths of its characters."""

    def test_width_is_the_sum_of_the_characters_widths_times_the_font_size(self) -> None:
        narrow_letters = 'il'
        self.assertAlmostEqual(text_width(narrow_letters, []),
                               2 * CHARACTER_WIDTHS['i'] * font_size([]))

    def test_a_character_without_a_measured_width_counts_as_the_widest(self) -> None:
        not_in_the_table = '\u00e9'
        self.assertNotIn(not_in_the_table, CHARACTER_WIDTHS)
        self.assertAlmostEqual(text_width(not_in_the_table, []),
                               WIDEST_CHARACTER * font_size([]))


class TestEveryDrawing(unittest.TestCase):
    """Every layout keeps its texts inside the drawing, inside their boxes and apart."""

    def test_every_text_stays_inside_its_drawing(self) -> None:
        for name, svg in every_layout().items():
            outside = [drawn.text for drawn in drawn_texts(svg)
                       if not viewbox(svg).holds(drawn.area)]
            self.assertEqual(outside, [], name)

    def test_every_box_holds_its_own_labels(self) -> None:
        layouts = every_layout()
        self.assertEqual(sorted(layouts), sorted(BOX_LABELS))
        for name, svg in layouts.items():
            boxes, texts = drawn_boxes(svg), drawn_texts(svg)
            self.assertEqual(len(boxes), len(BOX_LABELS[name]), name)
            for box, labels in zip(boxes, BOX_LABELS[name]):
                for label in labels:
                    inside_the_box = [drawn for drawn in texts
                                      if drawn.text == label and box.holds(drawn.area)]
                    self.assertEqual(len(inside_the_box), 1, (name, label))

    def test_no_text_with_its_middle_in_a_box_reaches_out_of_the_box(self) -> None:
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

    def test_no_line_of_a_plot_crosses_a_numbers_disc(self) -> None:
        clearance = control_systems_figures.DISC_RADIUS + 1
        for name, svg in every_layout().items():
            points = [point for kind in LineKind for line in drawn_lines(svg, kind)
                      for point in points_along(line)]
            crossed = {number for number, center in drawn_numbers(svg) for point in points
                       if math.hypot(point.x - center.x, point.y - center.y) < clearance}
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

    def test_only_the_signals_line_is_thick_and_solid(self) -> None:
        thick = declaration_of(LineKind.SIGNAL.value, 'stroke-width')
        thick_and_solid = [kind for kind in LineKind
                           if declaration_of(kind.value, 'stroke-width') == thick
                           and declaration_of(kind.value, 'stroke-dasharray') is None]
        self.assertEqual(thick_and_solid, [LineKind.SIGNAL])

    def test_target_speed_has_the_targets_line_in_each_of_the_three_plot_figures(self) -> None:
        # The stability figure's input is the same target speed as the other plots' reference
        plot_figures = [control_systems_figures.wide_stability_figure(),
                        control_systems_figures.stability_figure(),
                        control_systems_figures.wide_performance_figure(),
                        control_systems_figures.performance_figure(),
                        control_systems_figures.wide_robustness_figure(),
                        control_systems_figures.robustness_figure()]
        for figure in plot_figures:
            target, = drawn_lines(figure.svg(), LineKind.TARGET)
            levels = [point.y for point in target]
            self.assertEqual(levels, sorted(levels, reverse=True), figure.name)
            self.assertEqual(len(set(levels)), 2, figure.name)


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

    def test_sampled_curve_runs_from_time_zero_to_the_end_time(self) -> None:
        end_time = 0.9
        line = control_systems_figures.sample_curve(self.plot, math.sin, end_time)
        self.assertEqual(line[0], self.plot.point(0, math.sin(0)))
        self.assertEqual(line[-1], self.plot.point(end_time, math.sin(end_time)))

    def test_sampled_curves_points_are_at_most_the_sample_step_apart_along_the_time_axis(
            self) -> None:
        line = control_systems_figures.sample_curve(self.plot, math.sin, self.plot.duration)
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

    def test_disturbance_response_is_zero_until_the_load_begins(self) -> None:
        before_the_load = -0.2
        for time in (before_the_load, 0):
            self.assertEqual(control_systems_figures.disturbance_response(
                control_systems_figures.SETTLING, time), 0)

    def test_disturbance_response_moves_by_at_most_the_largest_move(self) -> None:
        system = control_systems_figures.SecondOrder(natural_frequency=8, damping_ratio=0.5)
        millisecond, two_seconds = 0.001, 2000
        moves = [control_systems_figures.disturbance_response(system, step * millisecond)
                 for step in range(two_seconds)]
        self.assertAlmostEqual(max(moves), 1, places=4)

    def test_disturbance_response_comes_back_to_zero(self) -> None:
        long_after_the_load_began = 10
        self.assertAlmostEqual(control_systems_figures.disturbance_response(
            control_systems_figures.SETTLING, long_after_the_load_began), 0)

    def test_a_rising_signal_has_settled_when_it_enters_the_band(self) -> None:
        # A signal that rises by 1 per second is within the band of 1 from 1 - ALLOWED_ERROR on
        end = 1.0
        entered_at = control_systems_figures.settled_at(lambda time: time, end)
        self.assertAlmostEqual(entered_at, 1 - control_systems_figures.ALLOWED_ERROR,
                               delta=2 * control_systems_figures.TIME_RESOLUTION)

    def test_a_signal_has_settled_after_the_last_time_it_left_the_band(self) -> None:
        # A signal that is in the band at first, leaves it at 0.5 s and is back at 0.75 s
        left_at, back_at, end = 0.5, 0.75, 1.0
        entered_at = control_systems_figures.settled_at(
            lambda time: 0 if left_at <= time < back_at else 1, end)
        self.assertAlmostEqual(entered_at, back_at,
                               delta=2 * control_systems_figures.TIME_RESOLUTION)

    def test_performance_output_is_the_step_response_until_the_disturbance(self) -> None:
        before_the_disturbance = control_systems_figures.DISTURBANCE_TIME - 0.1
        self.assertEqual(
            control_systems_figures.performance_output(before_the_disturbance),
            control_systems_figures.step_response(
                control_systems_figures.SETTLING,
                before_the_disturbance - control_systems_figures.STEP_TIME))

    def test_performance_output_dips_by_the_dip_depth_after_the_disturbance(self) -> None:
        millisecond, a_second = 0.001, 1000
        speeds = [control_systems_figures.performance_output(
            control_systems_figures.DISTURBANCE_TIME + step * millisecond)
            for step in range(a_second)]
        self.assertAlmostEqual(1 - min(speeds), control_systems_figures.DIP_DEPTH, places=2)

    def test_wheel_loop_of_the_models_value_is_the_settling_system(self) -> None:
        self.assertEqual(control_systems_figures.wheel_loop(1), control_systems_figures.SETTLING)

    def test_wheel_loop_of_twice_the_radius_has_half_the_frequency_and_half_the_damping(
            self) -> None:
        twice = control_systems_figures.wheel_loop(2)
        self.assertAlmostEqual(twice.natural_frequency,
                               control_systems_figures.SETTLING.natural_frequency / 2)
        self.assertAlmostEqual(twice.damping_ratio,
                               control_systems_figures.SETTLING.damping_ratio / 2)

    def test_wheel_speed_of_the_models_value_is_the_step_response_after_the_step(self) -> None:
        after_the_step = 0.3
        self.assertAlmostEqual(
            control_systems_figures.wheel_speed(
                1, control_systems_figures.STEP_TIME + after_the_step),
            control_systems_figures.step_response(control_systems_figures.SETTLING,
                                                  after_the_step))


class StabilityLines(NamedTuple):
    """The lines of one layout of the stability figure."""

    input_bound: List[Point]
    input_signal: List[Point]
    output_bound: List[Point]
    stable_output: List[Point]
    unstable_output: List[Point]


def stability_lines(svg: str) -> StabilityLines:
    """Return the lines of a layout of the stability figure: the input's plot is drawn first.

    The unstable output's points are in the order of time, whichever way its line is drawn.
    """
    input_bound, output_bound = drawn_lines(svg, LineKind.BOUND)
    input_signal, = drawn_lines(svg, LineKind.TARGET)
    stable_output, = drawn_lines(svg, LineKind.SIGNAL)
    unstable_output, = drawn_lines(svg, LineKind.UNSTABLE)
    return StabilityLines(input_bound, input_signal, output_bound, stable_output,
                          sorted(unstable_output, key=lambda point: point.x))


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
            rising = [rise for rise in rises if rise > 0]
            quarter = len(rising) // 4
            much_faster = 2
            self.assertGreater(sum(rising[-quarter:]), much_faster * sum(rising[:quarter]))

    def test_unstable_output_is_still_rising_where_its_line_ends(self) -> None:
        for lines in self.layouts:
            self.assertLess(lines.unstable_output[-1].y, lines.unstable_output[-2].y)
            self.assertLess(lines.unstable_output[-1].x, lines.output_bound[-1].x)

    def test_unstable_outputs_dashes_start_where_its_line_leaves_the_plot(self) -> None:
        # A line's dashes start with a whole dash at the first point that is drawn
        for svg in (self.wide, self.tall):
            drawn, = drawn_lines(svg, LineKind.UNSTABLE)
            self.assertEqual(drawn[0].y, highest_y(drawn))

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


# What each number of the performance figure marks on the plot, as the key has to name it
PERFORMANCE_KEY = {1: 'settling time', 2: 'tracking error',
                   3: 'allowed error after settling', 4: 'disturbance and recovery'}


class PerformanceDrawing(NamedTuple):
    """What one layout of the performance figure draws."""

    svg: str
    band: Area
    target: List[Point]
    speed: List[Point]
    settling_time_line: Tuple[Point, Point]
    error_line: Tuple[Point, Point]
    band_bracket: List[Point]
    numbers: List[Tuple[int, Point]]

    @property
    def target_y(self) -> float:
        """Return the y coordinate of the target's level after its step."""
        return self.target[-1].y

    @property
    def step_x(self) -> float:
        """Return the x coordinate of the target's step."""
        return next(point.x for point in self.target if point.y < self.target[0].y)

    @property
    def entered_at(self) -> int:
        """Return the index of the speed's first point from which the speed stays in the band."""
        outside = [index for index, point in enumerate(self.speed)
                   if not self.band.top <= point.y <= self.band.bottom]
        return outside[-1] + 1

    @property
    def dip(self) -> Point:
        """Return the speed's lowest point after the speed first reached the target's level."""
        reached = next(index for index, point in enumerate(self.speed)
                       if point.y <= self.target_y)
        return max(self.speed[reached:], key=lambda point: point.y)

    def mark(self, number: int) -> Point:
        """Return the center of a number's disc on the plot: the first of its two discs."""
        return next(center for drawn, center in self.numbers if drawn == number)


def performance_drawing(svg: str) -> PerformanceDrawing:
    """Return what a layout of the performance figure draws."""
    band, = drawn_rectangles(svg, 'band')
    target, = drawn_lines(svg, LineKind.TARGET)
    speed, = drawn_lines(svg, LineKind.SIGNAL)
    settling_time_line, error_line = drawn_dimensions(svg)
    band_bracket = next(line for line in drawn_lines(svg, LineKind.GUIDE) if len(line) > 2)
    return PerformanceDrawing(svg, band, target, speed, settling_time_line, error_line,
                              band_bracket, drawn_numbers(svg))


class TestPerformance(unittest.TestCase):
    """Both layouts show the speed that follows its target, the band and the four marks."""

    def setUp(self) -> None:
        self.wide = control_systems_figures.wide_performance_figure().svg()
        self.tall = control_systems_figures.performance_figure().svg()
        self.layouts = (performance_drawing(self.wide), performance_drawing(self.tall))
        self.include = (ROOT / 'includes' / 'control-systems-performance.html').read_text(
            encoding='utf-8')
        self.sample_step = control_systems_figures.SAMPLE_STEP + ROUNDING

    def test_include_holds_the_wide_layout_then_the_tall_one_and_one_caption(self) -> None:
        self.assertIn('<figure class="cs-figure cs-has-wide" id="performance-figure">',
                      self.include)
        self.assertLess(self.include.index(self.wide), self.include.index(self.tall))
        self.assertEqual(self.include.count('<figcaption>'), 1)

    def test_caption_says_that_the_curve_is_no_measurement(self) -> None:
        self.assertIn('an example response, not a measurement of the robot', self.include)

    def test_target_steps_up_once_and_stays(self) -> None:
        for drawing in self.layouts:
            levels = [point.y for point in drawing.target]
            self.assertEqual(levels, sorted(levels, reverse=True))
            self.assertEqual(len(set(levels)), 2)

    def test_band_lies_evenly_around_the_target_from_its_step_to_its_end(self) -> None:
        for drawing in self.layouts:
            self.assertAlmostEqual(drawing.target_y - drawing.band.top,
                                   drawing.band.bottom - drawing.target_y, delta=2 * ROUNDING)
            self.assertGreater(drawing.band.bottom - drawing.band.top, 20)
            self.assertEqual(drawing.band.left, drawing.step_x)
            self.assertAlmostEqual(drawing.band.right, drawing.target[-1].x, delta=ROUNDING)

    def test_speed_starts_at_rest_below_the_band(self) -> None:
        for drawing in self.layouts:
            self.assertEqual(drawing.speed[0].y, drawing.target[0].y)
            self.assertGreater(drawing.speed[0].y, drawing.band.bottom)

    def test_speed_stays_in_the_band_for_most_of_the_plot_once_it_has_entered(self) -> None:
        for drawing in self.layouts:
            in_the_band = drawing.speed[drawing.entered_at:]
            self.assertGreater(len(in_the_band), len(drawing.speed) / 2)
            self.assertEqual(drawing.speed[-1], in_the_band[-1])

    def test_speed_dips_in_the_band_and_comes_back_to_the_target(self) -> None:
        visible = 5
        for drawing in self.layouts:
            self.assertGreater(drawing.dip.y - drawing.target_y, visible)
            self.assertLess(drawing.dip.y, drawing.band.bottom)
            self.assertLess(drawing.dip.x, drawing.speed[-1].x)
            self.assertAlmostEqual(drawing.speed[-1].y, drawing.target_y, delta=1)

    def test_mark_1_spans_from_the_targets_step_to_where_the_speed_enters_the_band(
            self) -> None:
        for drawing in self.layouts:
            start, end = drawing.settling_time_line
            self.assertEqual(start.y, end.y)
            self.assertEqual(start.x, drawing.step_x)
            self.assertAlmostEqual(end.x, drawing.speed[drawing.entered_at].x,
                                   delta=self.sample_step)

    def test_a_guide_leads_from_where_the_speed_enters_the_band_down_to_mark_1(self) -> None:
        for drawing in self.layouts:
            end = drawing.settling_time_line[1]
            guide = next(line for line in drawn_lines(drawing.svg, LineKind.GUIDE)
                         if line[0].x == end.x and len(line) == 2)
            self.assertAlmostEqual(guide[0].y, drawing.band.bottom, delta=1)
            self.assertGreater(guide[1].y, end.y)

    def test_mark_2_spans_from_the_speed_up_to_the_target_at_one_time(self) -> None:
        for drawing in self.layouts:
            at_the_speed, at_the_target = drawing.error_line
            self.assertEqual(at_the_speed.x, at_the_target.x)
            self.assertEqual(at_the_target.y, drawing.target_y)
            self.assertAlmostEqual(at_the_speed.y, y_on(drawing.speed, at_the_speed.x), delta=1)
            self.assertGreater(at_the_speed.y, drawing.band.bottom)

    def test_mark_3_spans_from_the_target_to_the_bands_upper_edge(self) -> None:
        for drawing in self.layouts:
            heights = [point.y for point in drawing.band_bracket]
            self.assertEqual(min(heights), drawing.band.top)
            self.assertEqual(max(heights), drawing.target_y)
            self.assertGreater(min(point.x for point in drawing.band_bracket),
                               drawing.band.right)

    def test_mark_4_stands_right_below_the_lowest_point_of_the_dip(self) -> None:
        nearest = 2 * control_systems_figures.DISC_RADIUS
        for drawing in self.layouts:
            disc = drawing.mark(4)
            self.assertAlmostEqual(disc.x, drawing.dip.x, delta=self.sample_step)
            self.assertGreater(disc.y - control_systems_figures.DISC_RADIUS,
                               drawing.band.bottom)
            self.assertLess(disc.y - control_systems_figures.DISC_RADIUS - drawing.dip.y,
                            nearest)

    def test_each_of_the_first_three_numbers_stands_next_to_its_mark(self) -> None:
        nearest = 2 * control_systems_figures.DISC_RADIUS
        for drawing in self.layouts:
            marks = {1: drawing.settling_time_line[1],
                     2: Point(drawing.error_line[0].x,
                              (drawing.error_line[0].y + drawing.error_line[1].y) / 2),
                     3: min(drawing.band_bracket, key=lambda point: point.y)}
            for number, mark in marks.items():
                disc = drawing.mark(number)
                self.assertLess(math.hypot(disc.x - mark.x, disc.y - mark.y), nearest, number)

    def test_each_number_stands_once_on_the_plot_and_once_in_the_key(self) -> None:
        for drawing in self.layouts:
            numbers = [number for number, _ in drawing.numbers]
            self.assertEqual(numbers, [1, 2, 3, 4, 1, 2, 3, 4])

    def test_key_names_each_mark_to_the_right_of_its_number_one_row_below_the_other(
            self) -> None:
        same_row = control_systems_figures.DISC_RADIUS
        for drawing in self.layouts:
            texts = {drawn.text: drawn.area for drawn in drawn_texts(drawing.svg)}
            key_discs = drawing.numbers[len(PERFORMANCE_KEY):]
            for number, disc in key_discs:
                name = texts[PERFORMANCE_KEY[number]]
                self.assertGreater(name.left, disc.x + control_systems_figures.DISC_RADIUS)
                self.assertLess(name.left - disc.x, 3 * control_systems_figures.DISC_RADIUS)
                self.assertLess(abs((name.top + name.bottom) / 2 - disc.y), same_row)
            rows = [disc.y for _, disc in key_discs]
            self.assertEqual(rows, sorted(set(rows)))

    def test_both_layouts_name_the_two_lines_with_the_pages_words_and_the_example(
            self) -> None:
        for drawing in self.layouts:
            texts = texts_of(drawing.svg)
            for text in ('reference', 'target wheel speed', 'output', 'wheel speed', 'time'):
                self.assertEqual(texts.count(text), 1, text)


class DrawnWheel(NamedTuple):
    """A wheel of the robustness figure as it is drawn: its rim, and the name below its tick."""

    kind: LineKind
    center: Point
    radius: float

    @property
    def ground_y(self) -> float:
        """Return the y coordinate of the wheel's lowest point."""
        return self.center.y + self.radius


class RobustnessDrawing(NamedTuple):
    """What one layout of the robustness figure draws."""

    svg: str
    wheels: List[DrawnWheel]
    scale_band: Area
    plot_band: Area
    target: List[Point]
    speeds: Dict[LineKind, List[Point]]

    @property
    def rims_below_and_above(self) -> Tuple[float, float]:
        """Return how far the drawn rims differ from the middle one, in percent of its radius.

        The first number is for the smallest rim, the second one for the largest.
        """
        smallest, middle, largest = (wheel.radius for wheel in self.wheels)
        return ((middle - smallest) / middle * 100, (largest - middle) / middle * 100)

    def entered_at(self, kind: LineKind) -> int:
        """Return the index of the first point from which a wheel's speed stays in the band."""
        outside = [index for index, point in enumerate(self.speeds[kind])
                   if not self.plot_band.top <= point.y <= self.plot_band.bottom]
        return outside[-1] + 1


# How far a drawn percentage may be from a stated one, in percentage points. The rims' radii
# are rounded to a tenth of a px, which is half a point for the middle rim
PERCENT_ROUNDING = 1

# The kinds of line of the three wheels, from the smallest wheel to the largest
WHEEL_KINDS = [LineKind.SMALLEST, LineKind.SIGNAL, LineKind.LARGEST]


def robustness_drawing(svg: str) -> RobustnessDrawing:
    """Return what a layout of the robustness figure draws. The wheels' scale is drawn first."""
    wheels = [DrawnWheel(kind, Point(float(circle.get('cx', 0)), float(circle.get('cy', 0))),
                         float(circle.get('r', 0)))
              for circle in ElementTree.fromstring(svg).iter('circle')
              for kind in WHEEL_KINDS if kind.value in css_classes(circle)]
    scale_band, plot_band = drawn_rectangles(svg, 'band')
    target, = drawn_lines(svg, LineKind.TARGET)
    speeds = {kind: drawn_lines(svg, kind)[0] for kind in WHEEL_KINDS}
    return RobustnessDrawing(svg, wheels, scale_band, plot_band, target, speeds)


class TestRobustness(unittest.TestCase):
    """Both layouts show three wheels in a band of the radius, and each wheel's speed.

    All three speeds reach the band around the target and stay in it.
    """

    def setUp(self) -> None:
        self.wide = control_systems_figures.wide_robustness_figure().svg()
        self.tall = control_systems_figures.robustness_figure().svg()
        self.layouts = (robustness_drawing(self.wide), robustness_drawing(self.tall))
        self.include = (ROOT / 'includes' / 'control-systems-robustness.html').read_text(
            encoding='utf-8')

    def test_include_holds_the_wide_layout_then_the_tall_one_and_one_caption(self) -> None:
        self.assertIn('<figure class="cs-figure cs-has-wide" id="robustness-figure">',
                      self.include)
        self.assertLess(self.include.index(self.wide), self.include.index(self.tall))
        self.assertEqual(self.include.count('<figcaption>'), 1)

    def test_caption_gives_each_end_of_the_range_against_the_model_value(self) -> None:
        variation = round(control_systems_figures.RADIUS_VARIATION * 100)
        self.assertIn(f'The smallest radius is {variation} % below the model value and the '
                      f'largest is {variation} % above it.', self.include)

    def test_caption_states_the_radius_variation_that_the_rims_are_drawn_with(self) -> None:
        stated = re.search(r'The smallest radius is (\d+) % below the model value and the '
                           r'largest is (\d+) % above it\.', self.include)
        assert stated is not None
        stated_below, stated_above = (int(percent) for percent in stated.groups())
        for drawing in self.layouts:
            drawn_below, drawn_above = drawing.rims_below_and_above
            self.assertAlmostEqual(drawn_below, stated_below, delta=PERCENT_ROUNDING)
            self.assertAlmostEqual(drawn_above, stated_above, delta=PERCENT_ROUNDING)

    def test_caption_says_that_the_range_is_chosen_for_the_eye_and_not_measured(self) -> None:
        self.assertIn('The range was chosen so that the three lines can be told apart. '
                      'An example, not a measurement of the robot.', self.include)

    def test_page_states_the_radius_variation_that_the_rims_are_drawn_with(self) -> None:
        page = (ROOT / 'docs' / 'theory' / 'modeling-control'
                / 'control-systems-introduction.md').read_text(encoding='utf-8')
        stated = re.search(r'The smallest and the largest radius differ by (\d+) % from the '
                           r"model's value\.", page)
        assert stated is not None
        stated_percent = int(stated.group(1))
        for drawing in self.layouts:
            for drawn_percent in drawing.rims_below_and_above:
                self.assertAlmostEqual(drawn_percent, stated_percent, delta=PERCENT_ROUNDING)

    def test_accessible_name_is_short_and_says_angular_speed(self) -> None:
        longest = 60
        for svg in (self.wide, self.tall):
            name = ElementTree.fromstring(svg).findtext('title') or ''
            self.assertIn('angular speed', name)
            self.assertLessEqual(len(name), longest)

    def test_three_wheels_stand_side_by_side_from_the_smallest_to_the_largest(self) -> None:
        for drawing in self.layouts:
            self.assertEqual([wheel.kind for wheel in drawing.wheels], WHEEL_KINDS)
            radii = [wheel.radius for wheel in drawing.wheels]
            places = [wheel.center.x for wheel in drawing.wheels]
            self.assertEqual(radii, sorted(set(radii)))
            self.assertEqual(places, sorted(set(places)))

    def test_no_two_wheels_touch(self) -> None:
        for drawing in self.layouts:
            for left, right in zip(drawing.wheels, drawing.wheels[1:]):
                self.assertGreater(right.center.x - right.radius, left.center.x + left.radius)

    def test_all_wheels_stand_on_the_same_ground(self) -> None:
        for drawing in self.layouts:
            grounds = {round(wheel.ground_y, 1) for wheel in drawing.wheels}
            self.assertEqual(len(grounds), 1)

    def test_each_wheel_stands_on_the_scale_where_its_radius_is(self) -> None:
        # Equal steps of the radius are equal steps along the scale
        for drawing in self.layouts:
            smallest, model, largest = drawing.wheels
            self.assertAlmostEqual(
                (model.center.x - smallest.center.x) / (model.radius - smallest.radius),
                (largest.center.x - model.center.x) / (largest.radius - model.radius),
                delta=0.5)

    def test_scales_band_reaches_from_the_smallest_wheel_to_the_largest(self) -> None:
        for drawing in self.layouts:
            smallest, model, largest = drawing.wheels
            self.assertEqual(drawing.scale_band.left, smallest.center.x)
            self.assertEqual(drawing.scale_band.right, largest.center.x)
            self.assertLess(drawing.scale_band.left, model.center.x)
            self.assertLess(model.center.x, drawing.scale_band.right)
            self.assertGreater(drawing.scale_band.top, model.ground_y)

    def test_a_tick_marks_each_wheels_place_on_the_scale(self) -> None:
        for drawing in self.layouts:
            ticks = [line for line in drawn_lines(drawing.svg, LineKind.GUIDE)
                     if len(line) == 2 and line[0].x == line[1].x]
            self.assertEqual([tick[0].x for tick in ticks],
                             [wheel.center.x for wheel in drawing.wheels])
            for tick in ticks:
                self.assertLess(tick[0].y, drawing.scale_band.top)
                self.assertGreater(tick[1].y, drawing.scale_band.bottom)

    def test_each_wheels_name_stands_below_its_tick(self) -> None:
        names = ['smallest', "model's value", 'largest']
        for drawing in self.layouts:
            texts = {drawn.text: drawn.area for drawn in drawn_texts(drawing.svg)}
            for name, wheel in zip(names, drawing.wheels):
                self.assertAlmostEqual((texts[name].left + texts[name].right) / 2,
                                       wheel.center.x, delta=1)
                self.assertGreater(texts[name].top, drawing.scale_band.bottom)

    def test_a_bracket_with_the_name_bounded_variation_spans_the_scales_band(self) -> None:
        for drawing in self.layouts:
            bracket, = [line for line in drawn_lines(drawing.svg, LineKind.GUIDE)
                        if len(line) > 2]
            self.assertEqual(min(point.x for point in bracket), drawing.scale_band.left)
            self.assertEqual(max(point.x for point in bracket), drawing.scale_band.right)
            name = next(drawn.area for drawn in drawn_texts(drawing.svg)
                        if drawn.text == 'bounded variation')
            self.assertGreater(name.top, max(point.y for point in bracket))
            self.assertLess(name.top - max(point.y for point in bracket), 10)
            self.assertGreater(name.left, drawing.scale_band.left)
            self.assertLess(name.right, drawing.scale_band.right)

    def test_each_wheels_speed_has_the_line_of_its_wheel(self) -> None:
        for drawing in self.layouts:
            self.assertEqual(list(drawing.speeds), [wheel.kind for wheel in drawing.wheels])
            for kind in WHEEL_KINDS:
                self.assertEqual(len(drawn_lines(drawing.svg, kind)), 1, kind)

    def test_plots_band_lies_evenly_around_the_target(self) -> None:
        for drawing in self.layouts:
            target_y = drawing.target[-1].y
            self.assertAlmostEqual(target_y - drawing.plot_band.top,
                                   drawing.plot_band.bottom - target_y, delta=2 * ROUNDING)

    def test_every_wheels_speed_enters_the_band_once_and_stays_in_it(self) -> None:
        for drawing in self.layouts:
            band = drawing.plot_band
            for kind, speed in drawing.speeds.items():
                self.assertGreater(speed[0].y, band.bottom, kind)
                first_in_the_band = next(index for index, point in enumerate(speed)
                                         if band.top <= point.y <= band.bottom)
                self.assertEqual(drawing.entered_at(kind), first_in_the_band, kind)
                self.assertGreater(len(speed[first_in_the_band:]), len(speed) / 2, kind)

    def test_every_wheels_speed_starts_to_rise_where_the_target_steps(self) -> None:
        for drawing in self.layouts:
            step_x = next(point.x for point in drawing.target
                          if point.y < drawing.target[0].y)
            for kind, speed in drawing.speeds.items():
                rises_from = max(point.x for point in speed if point.y == speed[0].y)
                self.assertAlmostEqual(rises_from, step_x, msg=kind,
                                       delta=control_systems_figures.SAMPLE_STEP + ROUNDING)

    def test_every_wheels_speed_ends_close_to_the_target(self) -> None:
        close = 3
        for drawing in self.layouts:
            for kind, speed in drawing.speeds.items():
                self.assertAlmostEqual(speed[-1].y, drawing.target[-1].y, delta=close, msg=kind)

    def test_smaller_wheel_enters_the_band_earlier(self) -> None:
        for drawing in self.layouts:
            entries = [drawing.speeds[kind][drawing.entered_at(kind)].x for kind in WHEEL_KINDS]
            self.assertEqual(entries, sorted(set(entries)))

    def test_larger_wheel_swings_further_past_the_target(self) -> None:
        for drawing in self.layouts:
            peaks = [highest_y(drawing.speeds[kind]) for kind in WHEEL_KINDS]
            self.assertEqual(peaks, sorted(set(peaks), reverse=True))
            self.assertGreater(peaks[-1], drawing.plot_band.top)

    def test_the_three_speeds_can_be_told_apart_while_they_rise(self) -> None:
        apart = 4
        for drawing in self.layouts:
            halfway = (drawing.target[0].y + drawing.target[-1].y) / 2
            crossings = [next(point.x for point in drawing.speeds[kind] if point.y <= halfway)
                         for kind in WHEEL_KINDS]
            for earlier, later in zip(crossings, crossings[1:]):
                self.assertGreaterEqual(later - earlier, apart)

    def test_both_layouts_name_the_parameter_the_band_and_both_angular_speeds(self) -> None:
        for drawing in self.layouts:
            texts = texts_of(drawing.svg)
            for text in ('parameter', 'wheel radius', 'reference', 'target angular speed',
                         'allowed error after settling', 'output', 'wheel angular speed',
                         'time'):
                self.assertEqual(texts.count(text), 1, text)

    def test_the_bands_name_stands_right_below_the_plots_band(self) -> None:
        for drawing in self.layouts:
            name = next(drawn.area for drawn in drawn_texts(drawing.svg)
                        if drawn.text == 'allowed error after settling')
            self.assertGreater(name.top, drawing.plot_band.bottom)
            self.assertLess(name.top - drawing.plot_band.bottom, 8)
            self.assertAlmostEqual(name.right, drawing.plot_band.right, delta=1)

    def test_the_band_has_the_name_of_the_performance_figures_third_mark(self) -> None:
        for drawing in self.layouts:
            self.assertIn(PERFORMANCE_KEY[3], texts_of(drawing.svg))


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

    def test_open_loop_labels_the_forward_path_as_the_closed_loop_does_but_for_the_controller(
            self) -> None:
        # Each signal and each box of the open loop, with its example, in the chain's order
        closed_layouts = (control_systems_figures.wide_closed_loop_figure().svg(),
                          control_systems_figures.closed_loop_figure().svg())
        only_difference = (Labelled('Controller', 'plan set in advance'),
                           Labelled('Controller', 'PID in the firmware'))
        for open_svg, closed_svg in zip((self.wide, self.tall), closed_layouts):
            own = labelled_terms(open_svg)
            self.assertEqual([labelled.term for labelled in own], OPEN_LOOP_CHAIN)
            shared = [labelled for labelled in labelled_terms(closed_svg)
                      if labelled.term in OPEN_LOOP_CHAIN]
            self.assertEqual(len(own), len(shared))
            different = [pair for pair in zip(own, shared) if pair[0] != pair[1]]
            self.assertEqual(different, [only_difference])


if __name__ == '__main__':
    unittest.main()
