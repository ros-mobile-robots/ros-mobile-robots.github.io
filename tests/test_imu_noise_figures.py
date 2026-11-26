"""Tests for scripts/figures/imu_noise_figures.py.

The script writes the figures about the IMU's bias and noise at rest as nine includes.

Run from the repository root: python -m unittest discover -s tests
"""

import importlib.util
import json
from pathlib import Path
import re
import sys
from types import ModuleType
from typing import Any
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Set
from typing import Tuple
import unittest
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
FIGURES = ROOT / 'scripts' / 'figures'
sys.path.insert(0, str(ROOT / 'tests'))

import figure_script_runs  # noqa: E402

SCRIPT = FIGURES / 'imu_noise_figures.py'
TESTS_PAGE = ROOT / 'docs' / 'packages' / 'imu' / 'noise-at-rest-tests.md'
IMU_PAGE = ROOT / 'docs' / 'packages' / 'imu' / 'driver-bno055.md'
# An entry of the page's list "The test classes": the names of its classes, in a link or as
# inline code, then their numbers of tests, such as "2 + 2 tests:"
CLASS_LIST_ENTRY = re.compile(
    r'- \*\*(?:\[(?P<names>[^\]]+)\]\([^)]*\)|(?P<plain>`Test[^*]*?)), '
    r'(?P<counts>[\d +]+) tests:\*\*')
# The includes whose figure has a narrow and a wide layout
REPORT_TO_EKF_INCLUDE = 'imu-noise-report-to-ekf.html'
TWO_LAYOUT_INCLUDES = [REPORT_TO_EKF_INCLUDE, 'imu-noise-test-flow.html',
                       'imu-noise-test-intervals.html', 'imu-noise-test-stamps-back.html',
                       'imu-noise-trace.html']
INCLUDE_NAMES = [REPORT_TO_EKF_INCLUDE, 'imu-noise-test-flow.html', 'imu-noise-test-gravity.html',
                 'imu-noise-test-intervals.html', 'imu-noise-test-measured.html',
                 'imu-noise-test-stamps-back.html', 'imu-noise-test-variance.html',
                 'imu-noise-three-readings.html', 'imu-noise-trace.html']
# The drawings write their coordinates with one decimal
SVG_COORDINATE_TOLERANCE = 0.1
# The synthetic recording that the trace figure's caption describes
EXPECTED_DURATION_SECONDS = 180
EXPECTED_SAMPLE_RATE_HZ = 50
# The measured data that the script reads, as a path from the repository's root
MEASURED_DATA = 'scripts/figures/data'
MEASURED_RECORD = ROOT / MEASURED_DATA / 'imu-noise-remo-yaw-rate-at-rest.json'
# The page's section about the figure, and the equation in it that squares the setting
REPORT_TO_EKF_SECTION = 'From the report to the EKF'
PAGE_SETTING_EQUATION = re.compile(
    r'\((?P<standard_deviation>[\d.]+) rad/s\)² = (?P<decimal_variance>[\d.]+) \(rad/s\)² = '
    r'(?P<variance>\d+ · 10\S+) \(rad/s\)²')
# The page's table with the measured values, and the column of the board that the figure shows
RESULTS_SECTION = 'Results on Remo'
RESULTS_BOARD = 'Teensy 4.0'
VARIANCE_UNIT = '(rad/s)²'
SUPERSCRIPT_TO_DIGITS = str.maketrans('⁻⁰¹²³⁴⁵⁶⁷⁸⁹', '-0123456789')
# The class of the page's text column, which the theme puts around every include
PAGE_TEXT_CLASS = 'md-typeset'
# The figure from the report to the EKF: its layouts' classes with their widths in SVG units
LAYOUT_WIDTHS = {'lv-layout-narrow': 300, 'lv-layout-wide': 540}
LONGEST_NAME_WORDS = 12  # of a drawing's accessible name
# A position or a length as the drawings write it
SVG_NUMBER = r'-?\d+(?:\.\d+)?'
REPORT_CARD_TITLE = "The report's row z"
SETTING_CARD_TITLE = "The firmware's setting"
COMPARISON_NOTE_START = 'drawn to scale'
MEASURED_INTERVAL_CLASS = 'measured-interval'
SETTING_INTERVAL_CLASS = 'setting-interval'
ZERO_GUIDE_CLASS = 'zero-guide'
SCALE_NAME = 'deviation in yaw rate (rad/s)'
AXIS_TICK_COUNT = 3
# How far above an interval the baseline of its label may stand, in SVG units
INTERVAL_LABEL_REACH = 8
# The message's covariance: three rows of three elements, stored row by row
COVARIANCE_FIELD = 'angular_velocity_covariance'
COVARIANCE_SIZE = 3
COVARIANCE_DIAGONAL_INDICES = (0, 4, 8)
YAW_RATE_VARIANCE_INDEX = 8
MARKED_CELL_CLASS = 'cell-fused'
# The covariance as the end of an arrow, and how far above the baseline of its field's name
# the arrow may end, in SVG units
COVARIANCE_PART = 'covariance'
FIELD_NAME_REACH = 14
# How far from an arrow's line the anchor of its label may lie, in SVG units
ARROW_LABEL_REACH = 8
# What the tests of connection() expect for the example boxes of connection_boxes(). The upper
# and the lower box share the x positions from 50 to 110. The left and the right box share the
# y positions from 30 to 70
UPPER_BOX_BOTTOM = 20
LOWER_BOX_TOP = 60
SHARED_X_MIDDLE = 80
LEFT_BOX_RIGHT = 40
RIGHT_BOX_LEFT = 100
SHARED_Y_MIDDLE = 50
# The two variances of the figure, and the decimals with which the report prints one
SETTING_VARIANCE = 4.0e-4
REPORT_VARIANCE = 5.317e-7
REPORT_DECIMALS = 3


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


def two_steps(figures: ModuleType) -> Sequence[Any]:
    """Return a read step with nine tests in three classes and a print step with five in one."""
    return (figures.ToolStep('Read', 'read()', (figures.ClassOfTests('TestOpen', 2),
                                                figures.ClassOfTests('TestParse', 3),
                                                figures.ClassOfTests('TestCheck', 4))),
            figures.ToolStep('Print', 'show()', (figures.ClassOfTests('TestShow', 5),)))


class ConnectionBoxes(NamedTuple):
    """The example boxes of the tests of connection(): two above each other, two side by side."""

    upper: Any
    lower: Any
    left: Any
    right: Any


def connection_boxes(figures: ModuleType) -> ConnectionBoxes:
    """Return the example boxes: the lower one starts right of the upper, the right one lower."""
    return ConnectionBoxes(upper=figures.Box(x=10, y=0, width=100, height=20),
                           lower=figures.Box(x=50, y=60, width=100, height=20),
                           left=figures.Box(x=0, y=10, width=40, height=60),
                           right=figures.Box(x=100, y=30, width=40, height=100))


def page_test_counts() -> List[Tuple[str, int]]:
    """Return each test class and its number of tests, in the order of the tests page's list."""
    counts: List[Tuple[str, int]] = []
    for line in TESTS_PAGE.read_text(encoding='utf-8').splitlines():
        entry = CLASS_LIST_ENTRY.match(line)
        if entry:
            names = re.findall(r'`(Test\w+)`', entry['names'] or entry['plain'])
            test_counts = (int(count) for count in entry['counts'].split('+'))
            counts.extend(zip(names, test_counts, strict=True))
    return counts


class ReadError(ValueError):
    """The page, the include or a drawing lacks something that a test reads from it."""


class DrawnPoint(NamedTuple):
    """A position that a drawing wrote, in SVG coordinates."""

    x: float
    y: float


class DrawnBox(NamedTuple):
    """A rectangle that a drawing wrote, with its CSS class."""

    css_class: str
    x: float
    y: float
    width: float
    height: float

    @property
    def right(self) -> float:
        """Return the x coordinate of the box's right edge."""
        return self.x + self.width

    @property
    def bottom(self) -> float:
        """Return the y coordinate of the box's lower edge."""
        return self.y + self.height

    @property
    def middle_x(self) -> float:
        """Return the x coordinate of the box's middle."""
        return self.x + self.width / 2

    def holds(self, point: DrawnPoint) -> bool:
        """Return whether the point lies in the box or on its edge."""
        return (self.x - SVG_COORDINATE_TOLERANCE <= point.x
                <= self.right + SVG_COORDINATE_TOLERANCE
                and self.y - SVG_COORDINATE_TOLERANCE <= point.y
                <= self.bottom + SVG_COORDINATE_TOLERANCE)

    def has_on_its_edge(self, point: DrawnPoint) -> bool:
        """Return whether the point lies on one of the box's four edges."""
        on_a_vertical_edge = any(abs(point.x - edge) <= SVG_COORDINATE_TOLERANCE
                                 for edge in (self.x, self.right))
        on_a_horizontal_edge = any(abs(point.y - edge) <= SVG_COORDINATE_TOLERANCE
                                   for edge in (self.y, self.bottom))
        return self.holds(point) and (on_a_vertical_edge or on_a_horizontal_edge)

    def grown(self, margin: float) -> 'DrawnBox':
        """Return the box with the margin added on each of its four sides."""
        return DrawnBox(self.css_class, self.x - margin, self.y - margin,
                        self.width + 2 * margin, self.height + 2 * margin)


class DrawnText(NamedTuple):
    """A line of text that a drawing wrote: its anchor point and side, its class, its words."""

    position: DrawnPoint
    css_class: str
    anchor: str  # start, middle or end: the side of the text at which its position lies
    content: str


class DrawnArrow(NamedTuple):
    """An arrow that a drawing wrote: its line's CSS class, its start and its head's tip."""

    line_class: str
    start: DrawnPoint
    tip: DrawnPoint


class DrawnCell(NamedTuple):
    """An element of the drawn covariance: its box, the index in its corner and its value."""

    box: DrawnBox
    index_text: str
    value_text: str


class DrawnConnection(NamedTuple):
    """An arrow as a reader sees it: from which part to which part, dashed or solid, its label."""

    start: str
    end: str
    is_dashed: bool
    label: Tuple[str, ...]


# The figure's four arrows. A comparison by hand is dashed, a value in the message is solid
EXPECTED_CONNECTIONS = (
    DrawnConnection('report', 'comparison', is_dashed=True, label=('std dev',)),
    DrawnConnection('setting', 'comparison', is_dashed=True, label=('std dev',)),
    DrawnConnection('setting', COVARIANCE_PART, is_dashed=False,
                    label=('into every message,', 'at indices 0, 4 and 8')),
    DrawnConnection('yaw-rate cell', 'EKF', is_dashed=False, label=('yaw-rate variance',)),
)


def report_to_ekf_include() -> ElementTree.Element:
    """Return the repository's include from the report to the EKF, under one root element."""
    text = (ROOT / 'includes' / REPORT_TO_EKF_INCLUDE).read_text(encoding='utf-8')
    return ElementTree.fromstring(f'<root>{text}</root>')


def report_to_ekf_drawings() -> Dict[str, ElementTree.Element]:
    """Return the drawings of the include from the report to the EKF, by their layout's class."""
    drawings = {}
    for drawing in report_to_ekf_include().findall('./div/figure/svg'):
        classes = (drawing.get('class') or '').split()
        layouts = [layout for layout in LAYOUT_WIDTHS if layout in classes]
        if len(layouts) != 1:
            raise ReadError(f'a drawing with the classes {classes} has no single layout')
        drawings[layouts[0]] = drawing
    return drawings


def report_to_ekf_caption() -> str:
    """Return the caption of the include from the report to the EKF."""
    caption = report_to_ekf_include().find('./div/figure/figcaption')
    if caption is None:
        raise ReadError('the include has no caption')
    return ''.join(caption.itertext())


def drawing_ancestors(include_text: str) -> List[Set[str]]:
    """Return the classes of each element around the parts of an include's first drawing.

    From the outside in: the page's text column, the block around a figure with two layouts
    if the include has one, the figure, and the drawing itself.
    """
    root = ElementTree.fromstring(f'<root>{include_text}</root>')
    block, figure = root.find('./div'), root.find('.//figure')
    drawing = figure.find('svg') if figure is not None else None
    if figure is None or drawing is None:
        raise ReadError('the include has no figure with a drawing')
    return [{PAGE_TEXT_CLASS}] + [set((element.get('class') or '').split())
                                  for element in (block, figure, drawing) if element is not None]


def selector_applies(selector: str, css_class: str, ancestors: Sequence[Set[str]]) -> bool:
    """Return whether a selector matches a part that has only css_class, below the ancestors.

    The selector's last part has to name the class and no other one. Each part before it has
    to name classes of one of the ancestors, in their order from the outside in. A selector
    with an attribute, a pseudo-class or a combinator other than a space doesn't count: the
    includes use those for the dark scheme and for other figures.
    """
    if re.search(r'[\[:>+~]', selector):
        return False
    *outer_parts, last_part = (set(re.findall(r'\.([\w-]+)', part)) for part in selector.split())
    if last_part != {css_class}:
        return False
    remaining = list(ancestors)
    for part in outer_parts:
        while remaining and not (part and part <= remaining[0]):
            remaining.pop(0)
        if not remaining:
            return False
        remaining.pop(0)
    return True


def css_declarations(include_text: str, css_class: str) -> Dict[str, str]:
    """Return the CSS properties that an include's rules give a class of its drawing's parts.

    A rule counts when one of its selectors applies to such a part inside the include's own
    figure (selector_applies). A later rule's value replaces an earlier one's.
    """
    ancestors = drawing_ancestors(include_text)
    declarations: Dict[str, str] = {}
    for rule in re.finditer(r'^(?P<selectors>[^@{\n][^{\n]*)\{(?P<declarations>[^}]*)\}$',
                            include_text, re.MULTILINE):
        if any(selector_applies(selector.strip(), css_class, ancestors)
               for selector in rule['selectors'].split(',')):
            for declaration in filter(None, rule['declarations'].split(';')):
                name, _, value = declaration.partition(':')
                declarations[name.strip()] = value.strip()
    return declarations


def line_is_dashed(css_class: str) -> bool:
    """Return whether the include from the report to the EKF draws a class's stroke dashed."""
    text = (ROOT / 'includes' / REPORT_TO_EKF_INCLUDE).read_text(encoding='utf-8')
    declarations = css_declarations(text, css_class)
    if 'stroke' not in declarations:
        raise ReadError(f'no rule of the include gives .{css_class} a stroke')
    return declarations.get('stroke-dasharray', 'none') != 'none'


def drawn_boxes(drawing: ElementTree.Element, css_class: str) -> List[DrawnBox]:
    """Return a drawing's rectangles that have the CSS class, in the order they are drawn."""
    return [DrawnBox(css_class, *(float(rectangle.get(name) or 'nan')
                                  for name in ('x', 'y', 'width', 'height')))
            for rectangle in drawing.iter('rect')
            if css_class in (rectangle.get('class') or '').split()]


def drawn_box(drawing: ElementTree.Element, css_class: str) -> DrawnBox:
    """Return the one rectangle of a drawing that has the CSS class."""
    boxes = drawn_boxes(drawing, css_class)
    if len(boxes) != 1:
        raise ReadError(f'the drawing has {len(boxes)} rectangles of the class {css_class}')
    return boxes[0]


def drawn_texts(drawing: ElementTree.Element) -> List[DrawnText]:
    """Return a drawing's lines of text, each with the words of all its spans."""
    return [DrawnText(DrawnPoint(float(text.get('x') or 'nan'), float(text.get('y') or 'nan')),
                      text.get('class') or '', text.get('text-anchor') or 'start',
                      ''.join(text.itertext()))
            for text in drawing.iter('text')]


def texts_in(drawing: ElementTree.Element, box: DrawnBox) -> List[DrawnText]:
    """Return the lines of text of a drawing whose anchor point lies in the box.

    The anchor is the start, the middle or the end of a line's baseline. Whether the whole
    line fits in the box is a check in the browser.
    """
    return [text for text in drawn_texts(drawing) if box.holds(text.position)]


def box_with_text(drawing: ElementTree.Element, text_start: str) -> DrawnBox:
    """Return the framed box of a drawing that holds a line of text with this start."""
    for box in drawn_boxes(drawing, 'item'):
        if any(text.content.startswith(text_start) for text in texts_in(drawing, box)):
            return box
    raise ReadError(f'no framed box holds a text that starts with {text_start!r}')


def path_start(path: ElementTree.Element) -> DrawnPoint:
    """Return the point at which a path of a drawing starts."""
    start = re.match(rf'M({SVG_NUMBER}),({SVG_NUMBER})', path.get('d') or '')
    if start is None:
        raise ReadError(f"the path {path.get('d')!r} doesn't start with a point")
    return DrawnPoint(float(start[1]), float(start[2]))


def drawn_arrows(drawing: ElementTree.Element) -> List[DrawnArrow]:
    """Return a drawing's arrows: each head with the line before it, which has to match it."""
    paths = list(drawing.iter('path'))
    arrows = []
    for index, head in enumerate(paths):
        head_class = head.get('class') or ''
        if head_class.endswith('-head'):
            line_class = head_class[:-len('-head')] + '-line'
            if index == 0 or paths[index - 1].get('class') != line_class:
                raise ReadError(f'the arrowhead {head_class} has no {line_class} before it')
            arrows.append(DrawnArrow(line_class, path_start(paths[index - 1]),
                                     path_start(head)))
    return arrows


def covariance_cells(drawing: ElementTree.Element) -> List[DrawnCell]:
    """Return the cells of a drawn covariance from left to right, row after row.

    The order comes from the cells' positions, not from the order in which they are drawn.
    """
    boxes = drawn_boxes(drawing, 'cell') + drawn_boxes(drawing, MARKED_CELL_CLASS)
    cells = []
    for box in sorted(boxes, key=lambda box: (box.y, box.x)):
        index_texts = [text.content for text in texts_in(drawing, box)
                       if 'cell-index' in text.css_class.split()]
        value_texts = [text.content for text in texts_in(drawing, box)
                       if 'cell-index' not in text.css_class.split()]
        if len(index_texts) != 1 or len(value_texts) != 1:
            raise ReadError(f'a cell has the indices {index_texts} and the values {value_texts}')
        cells.append(DrawnCell(box, index_texts[0], value_texts[0]))
    return cells


def tick_positions(drawing: ElementTree.Element) -> List[float]:
    """Return the x positions of the ticks of a drawing's axis, from left to right."""
    return sorted(path_start(path).x for path in drawing.iter('path')
                  if path.get('class') == 'tick')


def zero_tick(drawing: ElementTree.Element) -> float:
    """Return the x position of the middle one of the three ticks of a drawing's axis."""
    ticks = tick_positions(drawing)
    if len(ticks) != AXIS_TICK_COUNT:
        raise ReadError(f'the axis has {len(ticks)} ticks')
    return ticks[1]


def interval_label(drawing: ElementTree.Element, interval: DrawnBox) -> str:
    """Return the text of the one line that stands right above an interval, at its middle."""
    above = DrawnBox('', interval.middle_x, interval.y - INTERVAL_LABEL_REACH, 0,
                     INTERVAL_LABEL_REACH)
    labels = [text.content for text in texts_in(drawing, above)]
    if len(labels) != 1:
        raise ReadError(f'the lines {labels} stand right above the {interval.css_class}')
    return labels[0]


def card_rows(drawing: ElementTree.Element, card: DrawnBox) -> Dict[str, str]:
    """Return the rows of a card of values: each row's label with the value on its baseline.

    A label starts at its anchor and a value ends at its anchor. A row without a label has
    the key '', and a row without a value has the value ''.
    """
    rows: Dict[str, str] = {}
    texts = texts_in(drawing, card)
    for baseline in sorted({text.position.y for text in texts}):
        row = {text.anchor: text.content for text in texts if text.position.y == baseline}
        rows[row.get('start', '')] = row.get('end', '')
    return rows


def connection_parts(drawing: ElementTree.Element) -> Dict[str, DrawnBox]:
    """Return the parts of a drawing that an arrow can start or end at, by their names.

    An arrow touches a framed box or a cell at its edge. The covariance has no frame: an arrow
    to it ends right above the name of its field, within the width of its cells.
    """
    cells = covariance_cells(drawing)
    field_names = [text for text in drawn_texts(drawing) if text.content == COVARIANCE_FIELD]
    if len(field_names) != 1:
        raise ReadError(f'the drawing names {COVARIANCE_FIELD} {len(field_names)} times')
    above_the_field_name = DrawnBox(
        '', cells[0].box.x, field_names[0].position.y - FIELD_NAME_REACH,
        cells[-1].box.right - cells[0].box.x, FIELD_NAME_REACH)
    return {'report': box_with_text(drawing, REPORT_CARD_TITLE),
            'setting': box_with_text(drawing, SETTING_CARD_TITLE),
            'comparison': box_with_text(drawing, COMPARISON_NOTE_START),
            'yaw-rate cell': cells[YAW_RATE_VARIANCE_INDEX].box,
            'EKF': box_with_text(drawing, 'EKF'),
            COVARIANCE_PART: above_the_field_name}


def part_at(parts: Dict[str, DrawnBox], point: DrawnPoint) -> str:
    """Return the name of the part that an arrow's end at the point touches, or "nothing"."""
    for name, box in parts.items():
        touches = box.holds(point) if name == COVARIANCE_PART else box.has_on_its_edge(point)
        if touches:
            return name
    return 'nothing'


def arrow_label(drawing: ElementTree.Element, arrow: DrawnArrow) -> Tuple[str, ...]:
    """Return the lines of the small text beside an arrow, from top to bottom."""
    shaft = DrawnBox('', min(arrow.start.x, arrow.tip.x), min(arrow.start.y, arrow.tip.y),
                     abs(arrow.tip.x - arrow.start.x), abs(arrow.tip.y - arrow.start.y))
    lines = [text for text in texts_in(drawing, shaft.grown(ARROW_LABEL_REACH))
             if text.css_class == 'note']
    return tuple(text.content for text in sorted(lines, key=lambda text: text.position.y))


def drawn_connections(drawing: ElementTree.Element) -> List[DrawnConnection]:
    """Return every arrow of a drawing as a connection, sorted by its two parts."""
    parts = connection_parts(drawing)
    return sorted(DrawnConnection(part_at(parts, arrow.start), part_at(parts, arrow.tip),
                                  line_is_dashed(arrow.line_class), arrow_label(drawing, arrow))
                  for arrow in drawn_arrows(drawing))


def page_section(heading: str) -> str:
    """Return the text of the IMU page's section with this heading, up to the next heading."""
    page = IMU_PAGE.read_text(encoding='utf-8')
    section = re.search(rf'^#+ {re.escape(heading)}\n(?P<text>.*?)(?=^#+ |\Z)', page,
                        re.MULTILINE | re.DOTALL)
    if section is None:
        raise ReadError(f'the page has no section {heading!r}')
    return section['text']


def power_of_ten_value(text: str) -> float:
    """Return the value of a number times a power of ten, such as "4 · 10⁻⁴"."""
    number = re.fullmatch(r'(?P<mantissa>[\d.]+) · 10(?P<exponent>[⁻⁰¹²³⁴⁵⁶⁷⁸⁹]+)', text)
    if number is None:
        raise ReadError(f'{text!r} is no number times a power of ten')
    return float(number['mantissa']) * 10 ** int(
        number['exponent'].translate(SUPERSCRIPT_TO_DIGITS))


def remo_z_value(row_start: str) -> str:
    """Return the z value of a row of the page's table "Results on Remo", for the Teensy 4.0.

    The column is the one whose header names the board. The z value is the last of the three
    values of an axis row.
    """
    rows = [[cell.strip() for cell in line.strip().strip('|').split('|')]
            for line in page_section(RESULTS_SECTION).splitlines() if line.startswith('|')]
    if not rows or RESULTS_BOARD not in rows[0]:
        raise ReadError(f'the table of {RESULTS_SECTION!r} has no column {RESULTS_BOARD!r}')
    column = rows[0].index(RESULTS_BOARD)
    for row in rows:
        if row[0].startswith(row_start):
            return row[column].split(',')[-1].strip()
    raise ReadError(f'the table of {RESULTS_SECTION!r} has no row {row_start!r}')


def remo_z_variance_text() -> str:
    """Return the variance of Remo's row z as the page's table gives it, without its unit.

    The table's row gives the power of ten in its first cell and the number before it in the
    column of the Teensy 4.0, which together are "5.317 · 10⁻⁷".
    """
    power_of_ten = re.search(r'\| Gyroscope variance x, y, z \((10\S+) \(rad/s\)²\)',
                             page_section(RESULTS_SECTION))
    if power_of_ten is None:
        raise ReadError("the table's variance row doesn't give its power of ten")
    return f"{remo_z_value('Gyroscope variance')} · {power_of_ten[1]}"


def page_setting(quantity: str) -> str:
    """Return a quantity of the equation with which the page's section squares the setting.

    The quantity is standard_deviation, such as "0.02", decimal_variance, such as "0.0004",
    or variance, such as "4 · 10⁻⁴".
    """
    equation = PAGE_SETTING_EQUATION.search(page_section(REPORT_TO_EKF_SECTION))
    if equation is None:
        raise ReadError(f'the section {REPORT_TO_EKF_SECTION!r} has no equation of the setting')
    return equation[quantity]


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the minus sign and the Greek delta.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        """Compare the generated includes with the checked-in UTF-8 bytes under an ASCII locale."""
        written = figure_script_runs.run_with_an_ascii_locale(SCRIPT.name, [MEASURED_DATA])
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

    def test_synthetic_trace_has_180_s_of_readings_at_50_hz(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        self.assertEqual(len(trace.readings), EXPECTED_DURATION_SECONDS * EXPECTED_SAMPLE_RATE_HZ)

    def test_synthetic_trace_readings_are_whole_gyroscope_output_steps(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        for reading in trace.readings:
            steps = reading / figures.GYROSCOPE_STEP
            self.assertAlmostEqual(steps, round(steps))

    def test_synthetic_trace_is_the_same_in_every_run(self) -> None:
        figures = load_script()
        self.assertEqual(figures.synthetic_trace(), figures.synthetic_trace())


class TestIncludeComment(unittest.TestCase):
    """Each include starts with the comment that names the script."""

    def test_each_include_starts_with_the_comment_that_names_the_script(self) -> None:
        for include_name in INCLUDE_NAMES:
            text = (ROOT / 'includes' / include_name).read_text(encoding='utf-8')
            self.assertTrue(text.startswith(
                '<!-- Written by scripts/figures/imu_noise_figures.py -->\n'), include_name)


class TestTexts(unittest.TestCase):
    """The figures' texts are built from the script's values."""

    def test_listed_joins_the_last_value_with_and(self) -> None:
        self.assertEqual(load_script().listed(['1', '2', '3', '4']), '1, 2, 3 and 4')

    def test_listed_gives_a_single_value_alone(self) -> None:
        self.assertEqual(load_script().listed(['1']), '1')

    def test_steps_in_words_names_zero_one_step_and_more_steps(self) -> None:
        figures = load_script()
        self.assertEqual([figures.steps_in_words(steps) for steps in (0, 1, 2)],
                         ['zero', 'one output step', 'two output steps'])

    def test_signed_value_with_decimals_gives_the_sign_a_minus_sign_and_the_decimals(
            self) -> None:
        figures = load_script()
        self.assertEqual(figures.signed_value_with_decimals(0.0014, 5), '+0.00140')
        self.assertEqual(figures.signed_value_with_decimals(-0.00148, 4), '−0.0015')

    def test_stamps_back_title_describes_the_repeat_the_step_back_and_the_last_stamp(
            self) -> None:
        figures = load_script()
        self.assertEqual(
            figures.stamps_back_title(figures.STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK),
            'Five messages in the order of the recording, each at its header stamp. The third '
            'message repeats the stamp of the second, 20 ms. The fourth is stamped 10 ms, '
            'earlier than the third. The fifth is stamped 80 ms')


class TestTraceLayouts(unittest.TestCase):
    """The trace figure has a narrow and a wide layout, each with the readings and a legend."""

    def test_trace_legend_pairs_the_mean_label_with_the_solid_mean_line(self) -> None:
        """Check the mark a reader sees beside the mean in each layout."""
        figures = load_script()
        trace = figures.synthetic_trace()
        for layout in figures.TRACE_LAYOUTS:
            elements = list(ElementTree.fromstring(figures.trace_figure(trace, layout).svg()))
            mean_label_index = next(
                index for index, element in enumerate(elements)
                if element.tag == 'text' and (element.text or '').startswith('mean:'))
            self.assertEqual(elements[mean_label_index - 1].get('class'), 'mean-line',
                             layout.name)

    def test_each_trace_layout_draws_the_solid_mean_line_between_the_dashed_spread_lines(
            self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        for layout in figures.TRACE_LAYOUTS:
            # The plot's lines start at its left edge. The legend's marks start elsewhere
            lines = list(re.finditer(
                r'<path class="(?P<kind>mean-line|spread-line)" '
                rf'd="M{figures.TRACE_PLOT_LEFT},(?P<y>[\d.]+) ',
                figures.trace_figure(trace, layout).svg()))
            self.assertEqual([line['kind'] for line in lines],
                             ['spread-line', 'mean-line', 'spread-line'], layout.name)
            upper_y, mean_y, lower_y = (float(line['y']) for line in lines)
            self.assertLess(upper_y, mean_y, layout.name)
            self.assertAlmostEqual(mean_y, figures.trace_y(trace.mean),
                                   delta=SVG_COORDINATE_TOLERANCE, msg=layout.name)
            self.assertLess(mean_y, lower_y, layout.name)

    def test_each_trace_layout_draws_the_first_seconds_readings_and_the_legends_dot(
            self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        shown = figures.TRACE_SHOWN_DURATION * figures.SAMPLE_RATE
        for layout in figures.TRACE_LAYOUTS:
            svg = figures.trace_figure(trace, layout).svg()
            self.assertEqual(svg.count('class="reading"'), shown + 1, layout.name)

    def test_each_trace_layout_uses_its_configured_viewbox_width(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        for layout in figures.TRACE_LAYOUTS:
            svg = figures.trace_figure(trace, layout).svg()
            self.assertIn(f'viewBox="0 0 {layout.width} ', svg)

    def test_trace_x_spans_the_plot_of_each_layout(self) -> None:
        figures = load_script()
        for layout in figures.TRACE_LAYOUTS:
            self.assertEqual(figures.trace_x(0, layout), figures.TRACE_PLOT_LEFT, layout.name)
            self.assertEqual(figures.trace_x(figures.TRACE_SHOWN_DURATION, layout),
                             layout.plot_right, layout.name)

    def test_each_trace_layout_has_its_expected_time_tick_count(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        # A tick every 0.5 s in the narrow layout and every 0.2 s in the wide one, over 1 s
        tick_counts = {'imu-noise-trace': 3, 'imu-noise-trace-wide': 6}
        for layout in figures.TRACE_LAYOUTS:
            svg = figures.trace_figure(trace, layout).svg()
            self.assertEqual(svg.count('class="tick"'), tick_counts[layout.name], layout.name)

    def test_each_trace_layout_shows_each_legend_entry_once(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        for layout in figures.TRACE_LAYOUTS:
            svg = figures.trace_figure(trace, layout).svg()
            for entry in figures.TRACE_LEGEND:
                self.assertEqual(svg.count(f'>{entry.label}</text>'), 1,
                                 f'{entry.label} in {layout.name}')

    def test_trace_legend_has_two_rows_fewer_in_two_columns_than_in_one(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        narrow, wide = (figures.trace_figure(trace, layout) for layout in figures.TRACE_LAYOUTS)
        self.assertEqual(narrow.height - wide.height, 2 * figures.TRACE_LEGEND_ROW_HEIGHT)

    def test_each_two_layout_include_holds_both_in_the_block_that_chooses_one(self) -> None:
        for include_name in TWO_LAYOUT_INCLUDES:
            text = (ROOT / 'includes' / include_name).read_text(encoding='utf-8')
            root = ElementTree.fromstring(f'<root>{text}</root>')
            wrapped_figures = root.findall("./div[@class='lv-space']/figure")
            self.assertEqual(len(wrapped_figures), 1, include_name)
            drawings = wrapped_figures[0].findall('svg')
            self.assertEqual([drawing.get('class') for drawing in drawings],
                             ['lv-fig lv-layout-narrow', 'lv-fig lv-layout-wide'], include_name)


class TestPlotLayouts(unittest.TestCase):
    """A stamp figure's wide layout is its narrow one with a longer time axis."""

    def test_wide_stamp_figures_put_each_mark_stretch_times_as_far_right(self) -> None:
        figures = load_script()
        narrow, wide = figures.PLOT_LAYOUTS
        for drawing, stamps in (
                (figures.intervals_figure, figures.STAMPS_WITH_ONE_LONG_INTERVAL),
                (figures.stamps_back_figure, figures.STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK)):
            narrow_x = [float(x) for x in re.findall(r'<circle[^>]* cx="([\d.]+)"',
                                                     drawing(narrow).svg())]
            wide_x = [float(x) for x in re.findall(r'<circle[^>]* cx="([\d.]+)"',
                                                   drawing(wide).svg())]
            self.assertEqual(len(narrow_x), len(stamps))
            self.assertEqual(len(wide_x), len(stamps))
            for narrow_value, wide_value in zip(narrow_x, wide_x):
                self.assertAlmostEqual(wide_value, narrow_value * wide.stretch,
                                       delta=SVG_COORDINATE_TOLERANCE)

    def test_wide_stamp_figures_are_as_wide_as_the_wide_layout(self) -> None:
        figures = load_script()
        wide = figures.PLOT_LAYOUTS[1]
        for drawing in (figures.intervals_figure, figures.stamps_back_figure):
            self.assertEqual(drawing(wide).width, figures.WIDE_WIDTH, drawing.__name__)

    def test_stamp_figures_number_their_marks_from_one_in_both_layouts(self) -> None:
        figures = load_script()
        for drawing, stamps in (
                (figures.intervals_figure, figures.STAMPS_WITH_ONE_LONG_INTERVAL),
                (figures.stamps_back_figure, figures.STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK)):
            for layout in figures.PLOT_LAYOUTS:
                numbers = re.findall(
                    r'<circle class="pass-mark"[^>]*/>\n<text class="letter"[^>]*>(\d+)</text>',
                    drawing(layout).svg())
                self.assertEqual(numbers,
                                 [str(number) for number in range(1, len(stamps) + 1)],
                                 f'{drawing.__name__} {layout.css_class}')

    def test_repeated_stamp_label_keeps_its_horizontal_offset_in_both_layouts(self) -> None:
        figures = load_script()
        distances = []
        for layout in figures.PLOT_LAYOUTS:
            svg = figures.stamps_back_figure(layout).svg()
            second_stamp_x = float(re.findall(r'<circle[^>]* cx="([\d.]+)"', svg)[1])
            label_x = float(re.findall(r'<text[^>]* x="([\d.]+)"[^>]*>repeated</text>', svg)[0])
            distances.append(label_x - second_stamp_x)
        self.assertGreater(distances[0], 0)
        self.assertAlmostEqual(distances[0], distances[1], delta=SVG_COORDINATE_TOLERANCE)


class TestFlowFigure(unittest.TestCase):
    """The flow figure's texts and columns are built from the steps and their test classes."""

    def test_wide_cards_fit_the_width_with_equal_gaps(self) -> None:
        figures = load_script()
        cards = figures.wide_cards(figures.TOOL_STEPS)
        gaps = [following.left - card.left - card.width
                for card, following in zip(cards, cards[1:])]
        self.assertAlmostEqual(gaps[0], gaps[1])
        self.assertGreaterEqual(gaps[0], figures.WIDE_FLOW_MIN_GAP)
        self.assertAlmostEqual(cards[-1].left + cards[-1].width,
                               figures.WIDE_WIDTH - figures.WIDE_FLOW_MARGIN)

    def test_both_flow_layouts_show_the_same_words(self) -> None:
        figures = load_script()
        narrow, wide = (re.sub(r'<title.*?</title>|<[^>]+>', ' ', drawing.svg()).split()
                        for drawing in (figures.flow_figure(), figures.wide_flow_figure()))
        self.assertEqual(sorted(narrow), sorted(wide))

    def test_middle_inside_margins_moves_a_card_that_would_leave_the_wide_drawing(self) -> None:
        figures = load_script()
        left, right = figures.WIDE_FLOW_MARGIN, figures.WIDE_WIDTH - figures.WIDE_FLOW_MARGIN
        left_card_width, right_card_width = 150, 170
        self.assertEqual(figures.middle_inside_margins(left, left_card_width),
                         left + left_card_width / 2)
        self.assertEqual(figures.middle_inside_margins(right, right_card_width),
                         right - right_card_width / 2)

    def test_middle_inside_margins_keeps_the_middle_of_a_card_inside_the_margins(self) -> None:
        figures = load_script()
        inside_center, inside_card_width = 270, 100
        self.assertEqual(figures.middle_inside_margins(inside_center, inside_card_width),
                         inside_center)

    def test_wide_flow_layout_keeps_every_card_inside_its_margins(self) -> None:
        figures = load_script()
        left, right = figures.WIDE_FLOW_MARGIN, figures.WIDE_WIDTH - figures.WIDE_FLOW_MARGIN
        cards = re.findall(r'<rect [^>]*x="(-?[\d.]+)"[^>]* width="([\d.]+)"',
                           figures.wide_flow_figure().svg())
        self.assertEqual(len(cards), len(figures.TOOL_STEPS) + 3)
        for x, width in cards:
            self.assertGreaterEqual(float(x), left)
            self.assertLessEqual(float(x) + float(width), right + SVG_COORDINATE_TOLERANCE)

    def test_wide_cards_fail_when_the_names_leave_no_room_for_the_arrows(self) -> None:
        figures = load_script()
        long_name = figures.ClassOfTests('Test' + 'LongName' * 4, 1)
        steps = [figures.ToolStep('Read', 'read()', (long_name,))] * 3
        with self.assertRaises(ValueError):
            figures.wide_cards(steps)

    def test_flow_caption_adds_up_the_tests_of_each_step(self) -> None:
        figures = load_script()
        self.assertTrue(figures.flow_caption(two_steps(figures)).startswith(
            "The test classes in a step's card call that step's functions: 9 tests at Read and "
            '5 tests at Print. '))

    def test_flow_title_names_each_step_with_its_function(self) -> None:
        figures = load_script()
        self.assertEqual(
            figures.flow_title(two_steps(figures)),
            "The tool's two steps from a bag file to the report, each with its test classes: "
            'Read with read() and Print with show(). A failed check leaves the first step to '
            'one line on stderr and status 1. TestToolOnBags runs the tool as a command')

    def test_class_columns_puts_the_second_column_right_of_the_first_columns_longest_name(
            self) -> None:
        figures = load_script()
        first, second = figures.class_columns(two_steps(figures))
        self.assertGreater(second, first + len('TestCheck') * figures.CLASS_NAME_WIDTH)

    def test_class_columns_fails_for_a_name_that_leaves_its_card(self) -> None:
        figures = load_script()
        too_long = figures.ClassOfTests('Test' + 'LongName' * 6, 1)
        steps = (figures.ToolStep('Read', 'read()', (figures.ClassOfTests('TestOpen', 2),
                                                     too_long)),)
        with self.assertRaises(ValueError):
            figures.class_columns(steps)

    def test_tool_steps_have_the_test_classes_and_counts_of_the_tests_page(self) -> None:
        figures = load_script()
        script_counts = [(test_class.name, test_class.test_count)
                         for step in figures.TOOL_STEPS for test_class in step.test_classes]
        script_counts.append((figures.COMMAND_TEST_CLASS, figures.COMMAND_TESTS_WITH_A_REPORT
                              + figures.COMMAND_TESTS_WITH_ONE_LINE))
        self.assertEqual(script_counts, page_test_counts())

    def test_tool_steps_have_the_labels_of_the_tests_pages_list_of_steps(self) -> None:
        page = TESTS_PAGE.read_text(encoding='utf-8')
        for number, step in enumerate(load_script().TOOL_STEPS, start=1):
            self.assertIn(f'\n{number}. **{step.label}:** ', page)


class TestReportToEkfFigure(unittest.TestCase):
    """The figure from the report to the EKF shows the page's values, to scale, in two layouts.

    The tests read the drawings, the caption and the style rules of the repository's include.
    Their expected values come from the page: the table "Results on Remo" and the equation
    that squares the firmware's setting.
    """

    def test_each_layout_draws_the_measured_interval_to_the_scale_of_the_settings(self) -> None:
        """Compare the two intervals' widths with the ratio of the page's values."""
        measured = float(remo_z_value('Gyroscope std dev'))
        setting = float(page_setting('standard_deviation'))
        for layout, drawing in report_to_ekf_drawings().items():
            measured_interval = drawn_box(drawing, MEASURED_INTERVAL_CLASS)
            setting_interval = drawn_box(drawing, SETTING_INTERVAL_CLASS)
            self.assertAlmostEqual(measured_interval.width,
                                   setting_interval.width * measured / setting,
                                   delta=SVG_COORDINATE_TOLERANCE, msg=layout)

    def test_each_layout_centres_both_intervals_on_the_zero_tick(self) -> None:
        """Compare each interval's middle with the middle tick of the axis."""
        for layout, drawing in report_to_ekf_drawings().items():
            for interval_class in (MEASURED_INTERVAL_CLASS, SETTING_INTERVAL_CLASS):
                self.assertAlmostEqual(drawn_box(drawing, interval_class).middle_x,
                                       zero_tick(drawing), delta=SVG_COORDINATE_TOLERANCE,
                                       msg=f'{interval_class} in {layout}')

    def test_each_layout_draws_both_intervals_equally_thick_in_rows_of_their_own(self) -> None:
        """Check the intervals' one height, which is less than the width of the measured one."""
        for layout, drawing in report_to_ekf_drawings().items():
            measured_interval = drawn_box(drawing, MEASURED_INTERVAL_CLASS)
            setting_interval = drawn_box(drawing, SETTING_INTERVAL_CLASS)
            self.assertEqual(measured_interval.height, setting_interval.height, layout)
            self.assertGreater(measured_interval.height, 0, layout)
            self.assertLess(measured_interval.height, measured_interval.width, layout)
            self.assertLess(measured_interval.bottom, setting_interval.y, layout)

    def test_each_layout_labels_each_interval_with_its_name_and_its_own_reach(self) -> None:
        """Read the line above each interval and compare it with the page's value for it."""
        measured = remo_z_value('Gyroscope std dev')
        setting = page_setting('standard_deviation')
        expected_labels = {MEASURED_INTERVAL_CLASS: f'Measured at rest ±{measured} rad/s',
                           SETTING_INTERVAL_CLASS: f'Firmware setting ±{setting} rad/s'}
        for layout, drawing in report_to_ekf_drawings().items():
            labels = {interval_class: interval_label(drawing, drawn_box(drawing, interval_class))
                      for interval_class in expected_labels}
            self.assertEqual(labels, expected_labels, layout)

    def test_each_layout_has_one_tick_at_each_end_of_the_settings_interval_and_at_zero(
            self) -> None:
        """Compare the three ticks and their signed labels with the setting's interval."""
        setting = page_setting('standard_deviation')
        expected_labels = (f'−{setting}', '0', f'+{setting}')
        for layout, drawing in report_to_ekf_drawings().items():
            interval = drawn_box(drawing, SETTING_INTERVAL_CLASS)
            panel = box_with_text(drawing, COMPARISON_NOTE_START)
            label_positions = {text.content: text.position.x
                               for text in texts_in(drawing, panel)}
            for tick, interval_x, label in zip(
                    tick_positions(drawing), (interval.x, interval.middle_x, interval.right),
                    expected_labels, strict=True):
                self.assertAlmostEqual(tick, interval_x, delta=SVG_COORDINATE_TOLERANCE,
                                       msg=layout)
                self.assertAlmostEqual(label_positions[label], tick,
                                       delta=SVG_COORDINATE_TOLERANCE, msg=f'{label} in {layout}')

    def test_each_layout_names_its_scale_a_deviation_in_yaw_rate_below_the_ticks(self) -> None:
        """Find the scale's name in the comparison, below the labels of the ticks."""
        for layout, drawing in report_to_ekf_drawings().items():
            panel = box_with_text(drawing, COMPARISON_NOTE_START)
            texts = {text.content: text.position.y for text in texts_in(drawing, panel)}
            self.assertIn(SCALE_NAME, texts, layout)
            self.assertGreater(texts[SCALE_NAME], texts['0'], layout)

    def test_each_layout_draws_the_zero_guide_through_both_intervals_down_to_the_ticks(
            self) -> None:
        """Check that the guide's strokes stand at zero, cover each interval, and reach a tick."""
        for layout, drawing in report_to_ekf_drawings().items():
            strokes = [re.fullmatch(rf'M({SVG_NUMBER}),({SVG_NUMBER}) V({SVG_NUMBER})',
                                    path.get('d') or '')
                       for path in drawing.iter('path') if path.get('class') == ZERO_GUIDE_CLASS]
            self.assertTrue(strokes and all(strokes), layout)
            spans = [(float(stroke[2]), float(stroke[3])) for stroke in strokes if stroke]
            for stroke in strokes:
                self.assertAlmostEqual(float(stroke[1] if stroke else 'nan'), zero_tick(drawing),
                                       delta=SVG_COORDINATE_TOLERANCE, msg=layout)
            for interval_class in (MEASURED_INTERVAL_CLASS, SETTING_INTERVAL_CLASS):
                interval = drawn_box(drawing, interval_class)
                self.assertTrue(any(top <= interval.y and interval.bottom <= bottom
                                    for top, bottom in spans), f'{interval_class} in {layout}')
            setting_interval = drawn_box(drawing, SETTING_INTERVAL_CLASS)
            ticks_top = min(path_start(path).y for path in drawing.iter('path')
                            if path.get('class') == 'tick')
            self.assertTrue(any(top <= setting_interval.y and ticks_top <= bottom
                                for top, bottom in spans), layout)

    def test_each_layouts_comparison_says_how_many_times_as_wide_the_setting_is(self) -> None:
        """Find the rounded ratio of the page's two standard deviations in the comparison."""
        ratio = (float(page_setting('standard_deviation'))
                 / float(remo_z_value('Gyroscope std dev')))
        for layout, drawing in report_to_ekf_drawings().items():
            panel = box_with_text(drawing, COMPARISON_NOTE_START)
            notes = [text.content for text in texts_in(drawing, panel)]
            self.assertEqual(
                [note for note in notes if 'times as wide' in note],
                [f'{COMPARISON_NOTE_START}: about {ratio:.0f} times as wide'], layout)

    def test_each_layouts_report_card_pairs_each_statistic_with_the_pages_value(self) -> None:
        """Compare each row's label and value with the page's table, whatever their order."""
        expected_rows = {
            "The report's row z, Remo at rest": '',
            'mean': f"{remo_z_value('Gyroscope bias')} rad/s",
            'std dev': f"{remo_z_value('Gyroscope std dev')} rad/s",
            'variance': f'{remo_z_variance_text()} {VARIANCE_UNIT}',
        }
        for layout, drawing in report_to_ekf_drawings().items():
            card = box_with_text(drawing, REPORT_CARD_TITLE)
            self.assertEqual(card_rows(drawing, card), expected_rows, layout)

    def test_each_layouts_setting_card_shows_the_variance_as_the_square_of_the_setting(
            self) -> None:
        """Compare the card's rows with the two sides of the page's equation."""
        expected_rows = {
            SETTING_CARD_TITLE: '',
            'angular_velocity_variance': '',
            'variance': f"{page_setting('variance')} {VARIANCE_UNIT}",
            '': f"= ({page_setting('standard_deviation')} rad/s)²",
        }
        for layout, drawing in report_to_ekf_drawings().items():
            card = box_with_text(drawing, SETTING_CARD_TITLE)
            self.assertEqual(card_rows(drawing, card), expected_rows, layout)

    def test_pages_equation_squares_the_standard_deviation_to_the_firmwares_variance(
            self) -> None:
        """Compare the decimal variance with the square, the power of ten and the setting."""
        decimal_variance = float(page_setting('decimal_variance'))
        self.assertAlmostEqual(decimal_variance, float(page_setting('standard_deviation')) ** 2)
        self.assertAlmostEqual(decimal_variance, power_of_ten_value(page_setting('variance')))
        self.assertAlmostEqual(decimal_variance,
                               load_script().FIRMWARE_ANGULAR_VELOCITY_VARIANCE)

    def test_each_layout_numbers_the_nine_elements_row_by_row_from_zero(self) -> None:
        """Read the indices of the cells in the order of their positions."""
        for layout, drawing in report_to_ekf_drawings().items():
            indices = [cell.index_text.lstrip('✓') for cell in covariance_cells(drawing)]
            self.assertEqual(indices, [str(index) for index in range(COVARIANCE_SIZE ** 2)],
                             layout)

    def test_each_layout_has_three_rows_with_cells_in_the_same_three_columns(self) -> None:
        """Check that each row of cells has one y and the three x positions of the first row."""
        for layout, drawing in report_to_ekf_drawings().items():
            cells = [cell.box for cell in covariance_cells(drawing)]
            rows = [cells[start:start + COVARIANCE_SIZE]
                    for start in range(0, len(cells), COVARIANCE_SIZE)]
            columns = [cell.x for cell in rows[0]]
            self.assertEqual(len(rows), COVARIANCE_SIZE, layout)
            self.assertEqual(len(set(columns)), COVARIANCE_SIZE, layout)
            self.assertEqual(len({row[0].y for row in rows}), COVARIANCE_SIZE, layout)
            for row in rows:
                self.assertEqual([cell.x for cell in row], columns, layout)
                self.assertEqual(len({cell.y for cell in row}), 1, layout)

    def test_each_layout_has_the_setting_on_the_diagonal_and_zero_elsewhere(self) -> None:
        """Compare the nine values with the page's variance at 0, 4 and 8."""
        setting = page_setting('variance')
        expected_values = [setting if index in COVARIANCE_DIAGONAL_INDICES else '0'
                           for index in range(COVARIANCE_SIZE ** 2)]
        for layout, drawing in report_to_ekf_drawings().items():
            self.assertEqual([cell.value_text for cell in covariance_cells(drawing)],
                             expected_values, layout)

    def test_each_layout_marks_only_the_yaw_rates_element_with_its_class_and_a_check_mark(
            self) -> None:
        """List the cells with the marked class and those with a check mark."""
        for layout, drawing in report_to_ekf_drawings().items():
            cells = covariance_cells(drawing)
            self.assertEqual([index for index, cell in enumerate(cells)
                              if cell.box.css_class == MARKED_CELL_CLASS],
                             [YAW_RATE_VARIANCE_INDEX], layout)
            self.assertEqual([index for index, cell in enumerate(cells)
                              if cell.index_text.startswith('✓')],
                             [YAW_RATE_VARIANCE_INDEX], layout)

    def test_marked_cell_has_a_thicker_edge_than_the_other_cells(self) -> None:
        """Compare the stroke widths that the include's rules give the two classes of cells."""
        include = (ROOT / 'includes' / REPORT_TO_EKF_INCLUDE).read_text(encoding='utf-8')
        marked_width, plain_width = (
            float(css_declarations(include, css_class)['stroke-width'])
            for css_class in (MARKED_CELL_CLASS, 'cell'))
        self.assertGreater(marked_width, plain_width)

    def test_each_layout_has_exactly_the_four_connections_with_their_lines_and_labels(
            self) -> None:
        """Compare every arrow's two parts, its dashed or solid line and its label."""
        for layout, drawing in report_to_ekf_drawings().items():
            self.assertEqual(drawn_connections(drawing), sorted(EXPECTED_CONNECTIONS), layout)

    def test_caption_gives_each_line_style_the_meaning_that_its_connections_have(self) -> None:
        """Find one sentence for the dashed and one for the solid arrow in the caption."""
        caption = report_to_ekf_caption()
        meanings = dict(re.findall(r'A (dashed|solid) arrow is a (comparison by hand|value in '
                                   r'the message)', caption))
        self.assertEqual(meanings, {'dashed': 'comparison by hand',
                                    'solid': 'value in the message'})

    def test_intervals_differ_by_a_dashed_edge_as_well_as_by_their_color(self) -> None:
        """Check the dash of the two interval classes in the include's rules."""
        self.assertTrue(line_is_dashed(SETTING_INTERVAL_CLASS))
        self.assertFalse(line_is_dashed(MEASURED_INTERVAL_CLASS))

    def test_only_the_report_to_ekf_include_has_the_rules_of_its_intervals_and_dashed_arrows(
            self) -> None:
        """Look for a stroke of the figure's own classes in each of the script's includes."""
        for include_name in INCLUDE_NAMES:
            text = (ROOT / 'includes' / include_name).read_text(encoding='utf-8')
            for css_class in (MEASURED_INTERVAL_CLASS, SETTING_INTERVAL_CLASS, 'compared-line'):
                self.assertEqual('stroke' in css_declarations(text, css_class),
                                 include_name == REPORT_TO_EKF_INCLUDE,
                                 f'{css_class} in {include_name}')

    def test_both_layouts_show_the_same_texts(self) -> None:
        """Compare the sorted lines of text of the two drawings."""
        narrow, wide = (sorted(text.content for text in drawn_texts(drawing))
                        for drawing in report_to_ekf_drawings().values())
        self.assertEqual(narrow, wide)

    def test_each_layout_keeps_its_boxes_inside_the_drawing(self) -> None:
        """Check both corners of every rectangle against the drawing's viewBox."""
        for layout, drawing in report_to_ekf_drawings().items():
            _, _, width, height = (float(number) for number in
                                   (drawing.get('viewBox') or '').split())
            whole_drawing = DrawnBox('', 0, 0, width, height)
            for css_class in ('item', 'cell', MARKED_CELL_CLASS, MEASURED_INTERVAL_CLASS,
                              SETTING_INTERVAL_CLASS):
                for box in drawn_boxes(drawing, css_class):
                    self.assertTrue(whole_drawing.holds(DrawnPoint(box.x, box.y))
                                    and whole_drawing.holds(DrawnPoint(box.right, box.bottom)),
                                    f'{box} in {layout}')

    def test_layouts_are_300_and_540_units_wide(self) -> None:
        """Read the width of each drawing's viewBox."""
        widths = {layout: int((drawing.get('viewBox') or '').split()[2])
                  for layout, drawing in report_to_ekf_drawings().items()}
        self.assertEqual(widths, LAYOUT_WIDTHS)

    def test_each_drawing_has_a_short_accessible_name(self) -> None:
        """Follow aria-labelledby to the drawing's one title and count the title's words."""
        for layout, drawing in report_to_ekf_drawings().items():
            reference = drawing.get('aria-labelledby')
            self.assertTrue(reference, layout)
            titles = [title for title in drawing.iter('title') if title.get('id') == reference]
            self.assertEqual(len(titles), 1, layout)
            word_count = len((titles[0].text or '').split())
            self.assertGreater(word_count, 0, layout)
            self.assertLessEqual(word_count, LONGEST_NAME_WORDS, layout)

    def test_pages_section_embeds_the_include(self) -> None:
        """Find the line that includes the figure in the section about the EKF."""
        self.assertIn(f'\n--8<-- "includes/{REPORT_TO_EKF_INCLUDE}"\n',
                      page_section(REPORT_TO_EKF_SECTION))

    def test_measured_record_has_the_values_of_the_pages_table_for_the_teensy_4_0(self) -> None:
        """Compare the record that the script reads with the page's table and its column."""
        record = json.loads(MEASURED_RECORD.read_text(encoding='utf-8'))
        measured = load_script().remo_yaw_rate_at_rest()
        self.assertEqual((record['board'], record['axis']), (RESULTS_BOARD, 'z'))
        self.assertEqual(measured.mean, float(remo_z_value('Gyroscope bias')))
        self.assertEqual(measured.standard_deviation, float(remo_z_value('Gyroscope std dev')))
        self.assertEqual(f'{measured.variance * 1e7:.3f}', remo_z_value('Gyroscope variance'))

    def test_connection_runs_down_between_boxes_that_stand_above_each_other(self) -> None:
        """Connect the upper example box with the lower one."""
        figures = load_script()
        self.assertEqual(
            figures.connection(connection_boxes(figures).upper, connection_boxes(figures).lower),
            figures.Connection(figures.DrawingPoint(SHARED_X_MIDDLE, UPPER_BOX_BOTTOM),
                               figures.DrawingPoint(SHARED_X_MIDDLE, LOWER_BOX_TOP)))

    def test_connection_runs_up_from_the_lower_box_to_the_upper_one(self) -> None:
        """Connect the lower example box with the upper one."""
        figures = load_script()
        self.assertEqual(
            figures.connection(connection_boxes(figures).lower, connection_boxes(figures).upper),
            figures.Connection(figures.DrawingPoint(SHARED_X_MIDDLE, LOWER_BOX_TOP),
                               figures.DrawingPoint(SHARED_X_MIDDLE, UPPER_BOX_BOTTOM)))

    def test_connection_runs_right_between_boxes_that_stand_beside_each_other(self) -> None:
        """Connect the left example box with the right one."""
        figures = load_script()
        self.assertEqual(
            figures.connection(connection_boxes(figures).left, connection_boxes(figures).right),
            figures.Connection(figures.DrawingPoint(LEFT_BOX_RIGHT, SHARED_Y_MIDDLE),
                               figures.DrawingPoint(RIGHT_BOX_LEFT, SHARED_Y_MIDDLE)))

    def test_connection_runs_left_from_the_right_box_to_the_left_one(self) -> None:
        """Connect the right example box with the left one."""
        figures = load_script()
        self.assertEqual(
            figures.connection(connection_boxes(figures).right, connection_boxes(figures).left),
            figures.Connection(figures.DrawingPoint(RIGHT_BOX_LEFT, SHARED_Y_MIDDLE),
                               figures.DrawingPoint(LEFT_BOX_RIGHT, SHARED_Y_MIDDLE)))

    def test_power_of_ten_text_gives_the_number_its_decimals_and_a_raised_exponent(self) -> None:
        """Format the setting without decimals and the report's variance with three."""
        figures = load_script()
        self.assertEqual(figures.power_of_ten_text(SETTING_VARIANCE, 0), '4 · 10⁻⁴')
        self.assertEqual(figures.power_of_ten_text(REPORT_VARIANCE, REPORT_DECIMALS),
                         '5.317 · 10⁻⁷')

    def test_firmware_standard_deviation_is_the_positive_root_of_the_settings_variance(
            self) -> None:
        """Square the positive standard deviation and compare it with the variance."""
        figures = load_script()
        self.assertGreater(figures.FIRMWARE_STANDARD_DEVIATION, 0)
        self.assertAlmostEqual(figures.FIRMWARE_STANDARD_DEVIATION ** 2,
                               figures.FIRMWARE_ANGULAR_VELOCITY_VARIANCE)


if __name__ == '__main__':
    unittest.main()
