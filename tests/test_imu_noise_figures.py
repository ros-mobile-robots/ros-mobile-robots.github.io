"""Tests for scripts/figures/imu_noise_figures.py.

The script writes the figures about the IMU's bias and noise at rest as nine includes.

Run from the repository root: python -m unittest discover -s tests
"""

import importlib.util
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from types import ModuleType
from typing import Any
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple
import unittest
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
FIGURES = ROOT / 'scripts' / 'figures'
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
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}
# The drawings write their coordinates with one decimal
SVG_COORDINATE_TOLERANCE = 0.1
# The synthetic recording that the trace figure's caption describes
EXPECTED_DURATION_SECONDS = 180
EXPECTED_SAMPLE_RATE_HZ = 50
# The page's sentence about the firmware's setting: its variance, and the standard deviation
# that the firmware's comment gives for it
PAGE_SETTING_VARIANCE = re.compile(
    r'`angular_velocity_variance`\]\([^)]*\) = (?P<variance>\S+ · 10\S+) \(rad/s\)²')
PAGE_SETTING_STANDARD_DEVIATION = re.compile(
    r'the square of a standard deviation of (?P<standard_deviation>[\d.]+) rad/s')
VARIANCE_UNIT = '(rad/s)²'
# The message's covariance: three rows of three elements, stored row by row
COVARIANCE_ELEMENT_COUNT = 9
COVARIANCE_DIAGONAL_INDICES = (0, 4, 8)
YAW_RATE_VARIANCE_INDEX = 8


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


def two_steps(figures: ModuleType) -> Sequence[Any]:
    """Return a read step with nine tests in three classes and a print step with five in one."""
    return (figures.ToolStep('Read', 'read()', (figures.ClassOfTests('TestOpen', 2),
                                                figures.ClassOfTests('TestParse', 3),
                                                figures.ClassOfTests('TestCheck', 4))),
            figures.ToolStep('Print', 'show()', (figures.ClassOfTests('TestShow', 5),)))


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


class DrawnText(NamedTuple):
    """A line of text that a drawing wrote: where it stands, its CSS class and its words."""

    position: DrawnPoint
    css_class: str
    content: str


class DrawnArrow(NamedTuple):
    """An arrow that a drawing wrote: where its line starts and where its head's tip is."""

    start: DrawnPoint
    tip: DrawnPoint


class DrawnCell(NamedTuple):
    """An element of the drawn covariance: its box, the index in its corner and its value."""

    box: DrawnBox
    index_text: str
    value_text: str


def report_to_ekf_drawings() -> Dict[str, ElementTree.Element]:
    """Return the drawings of the repository's include from the report to the EKF, by layout.

    The keys are the classes of the two layouts, lv-layout-narrow and lv-layout-wide.
    """
    text = (ROOT / 'includes' / REPORT_TO_EKF_INCLUDE).read_text(encoding='utf-8')
    root = ElementTree.fromstring(f'<root>{text}</root>')
    return {(drawing.get('class') or '').split()[-1]: drawing
            for drawing in root.findall('./div/figure/svg')}


def drawn_boxes(drawing: ElementTree.Element, css_class: str) -> List[DrawnBox]:
    """Return a drawing's rectangles that have the CSS class, in the order they are drawn."""
    return [DrawnBox(css_class, *(float(rectangle.get(name) or 'nan')
                                  for name in ('x', 'y', 'width', 'height')))
            for rectangle in drawing.iter('rect')
            if css_class in (rectangle.get('class') or '').split()]


def drawn_texts(drawing: ElementTree.Element) -> List[DrawnText]:
    """Return a drawing's lines of text, each with the words of all its spans."""
    return [DrawnText(DrawnPoint(float(text.get('x') or 'nan'), float(text.get('y') or 'nan')),
                      text.get('class') or '', ''.join(text.itertext()))
            for text in drawing.iter('text')]


def texts_in(drawing: ElementTree.Element, box: DrawnBox) -> List[DrawnText]:
    """Return the lines of text of a drawing that start, end or have their middle in the box."""
    return [text for text in drawn_texts(drawing) if box.holds(text.position)]


def box_with_text(drawing: ElementTree.Element, text_start: str) -> DrawnBox:
    """Return the framed box of a drawing that holds a line of text with this start."""
    for box in drawn_boxes(drawing, 'item'):
        if any(text.content.startswith(text_start) for text in texts_in(drawing, box)):
            return box
    raise AssertionError(f'no box holds a text that starts with {text_start!r}')


def drawn_arrows(drawing: ElementTree.Element, series: str) -> List[DrawnArrow]:
    """Return a drawing's arrows of a series, such as flow: each a line and the head after it."""
    paths = list(drawing.iter('path'))
    arrows = []
    for line, head in zip(paths, paths[1:]):
        if line.get('class') == f'{series}-line' and head.get('class') == f'{series}-head':
            start, tip = (re.match(r'M([\d.]+),([\d.]+)', path.get('d') or '')
                          for path in (line, head))
            assert start is not None and tip is not None
            arrows.append(DrawnArrow(DrawnPoint(float(start[1]), float(start[2])),
                                     DrawnPoint(float(tip[1]), float(tip[2]))))
    return arrows


def covariance_cells(drawing: ElementTree.Element) -> List[DrawnCell]:
    """Return the cells of a drawn covariance from left to right, row after row.

    The order comes from the cells' positions, not from the order in which they are drawn.
    """
    boxes = drawn_boxes(drawing, 'cell') + drawn_boxes(drawing, 'cell-fused')
    cells = []
    for box in sorted(boxes, key=lambda box: (box.y, box.x)):
        index_texts = [text.content for text in texts_in(drawing, box)
                       if 'cell-index' in text.css_class.split()]
        value_texts = [text.content for text in texts_in(drawing, box)
                       if 'cell-index' not in text.css_class.split()]
        assert len(index_texts) == 1 and len(value_texts) == 1, (index_texts, value_texts)
        cells.append(DrawnCell(box, index_texts[0], value_texts[0]))
    return cells


def tick_positions(drawing: ElementTree.Element) -> List[float]:
    """Return the x positions of the ticks of a drawing's axis, from left to right."""
    ticks = [re.match(r'M([\d.]+),', path.get('d') or '') for path in drawing.iter('path')
             if path.get('class') == 'tick']
    return sorted(float(tick[1]) for tick in ticks if tick is not None)


def remo_z_value(row_start: str) -> str:
    """Return the z value of the Teensy 4.0 column of a row of the page's "Results on Remo"."""
    for line in IMU_PAGE.read_text(encoding='utf-8').splitlines():
        if line.startswith(f'| {row_start}'):
            teensy_4_0_column = line.strip('| ').split('|')[-1]
            return teensy_4_0_column.split(',')[-1].strip()
    raise AssertionError(f'the page has no table row that starts with {row_start!r}')


def remo_z_variance_text() -> str:
    """Return the variance of Remo's row z as the page's table gives it, without its unit.

    The table's row gives the power of ten in its first cell and the number before it in the
    column of the Teensy 4.0, which together are "5.317 · 10⁻⁷".
    """
    page = IMU_PAGE.read_text(encoding='utf-8')
    power_of_ten = re.search(r'\| Gyroscope variance x, y, z \((10\S+) \(rad/s\)²\)', page)
    assert power_of_ten is not None
    return f"{remo_z_value('Gyroscope variance')} · {power_of_ten[1]}"


def page_setting_text(pattern: 're.Pattern[str]') -> str:
    """Return what the page's sentence about the firmware's setting gives for the pattern."""
    found = pattern.search(IMU_PAGE.read_text(encoding='utf-8'))
    assert found is not None, pattern.pattern
    return found[1]


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

    The tests read the drawings of the repository's include. Their expected values come from
    the page: the table "Results on Remo" and the sentence about the firmware's setting.
    """

    def test_each_layout_draws_the_measured_band_to_the_scale_of_the_settings_band(self) -> None:
        measured = float(remo_z_value('Gyroscope std dev'))
        setting = float(page_setting_text(PAGE_SETTING_STANDARD_DEVIATION))
        for layout, drawing in report_to_ekf_drawings().items():
            measured_band, = drawn_boxes(drawing, 'measured-band')
            setting_band, = drawn_boxes(drawing, 'setting-band')
            self.assertAlmostEqual(measured_band.width, setting_band.width * measured / setting,
                                   delta=SVG_COORDINATE_TOLERANCE, msg=layout)

    def test_each_layout_centres_both_bands_on_the_axis_zero(self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            _, zero_tick, _ = tick_positions(drawing)
            for band_class in ('measured-band', 'setting-band'):
                band, = drawn_boxes(drawing, band_class)
                self.assertAlmostEqual(band.middle_x, zero_tick, delta=SVG_COORDINATE_TOLERANCE,
                                       msg=f'{band_class} in {layout}')

    def test_each_layout_labels_the_ends_of_the_settings_band_with_its_standard_deviation(
            self) -> None:
        setting = page_setting_text(PAGE_SETTING_STANDARD_DEVIATION)
        for layout, drawing in report_to_ekf_drawings().items():
            setting_band, = drawn_boxes(drawing, 'setting-band')
            ticks = tick_positions(drawing)
            panel = box_with_text(drawing, 'compared by hand')
            labels = {text.content: text.position.x for text in texts_in(drawing, panel)}
            for tick, band_x, label in zip(
                    ticks, (setting_band.x, setting_band.middle_x, setting_band.right),
                    (f'−{setting} rad/s', '0', f'+{setting} rad/s')):
                self.assertAlmostEqual(tick, band_x, delta=SVG_COORDINATE_TOLERANCE, msg=layout)
                self.assertAlmostEqual(labels[label], tick, delta=SVG_COORDINATE_TOLERANCE,
                                       msg=f'{label} in {layout}')

    def test_each_layout_says_how_many_times_as_wide_the_settings_band_is(self) -> None:
        ratio = (float(page_setting_text(PAGE_SETTING_STANDARD_DEVIATION))
                 / float(remo_z_value('Gyroscope std dev')))
        for layout, drawing in report_to_ekf_drawings().items():
            setting_band, = drawn_boxes(drawing, 'setting-band')
            self.assertIn(f'about {ratio:.0f} times as wide',
                          [text.content for text in texts_in(drawing, setting_band)], layout)

    def test_each_layouts_report_card_shows_the_three_values_of_the_pages_table(self) -> None:
        expected_values = [f"{remo_z_value('Gyroscope bias')} rad/s",
                           f"{remo_z_value('Gyroscope std dev')} rad/s",
                           f'{remo_z_variance_text()} {VARIANCE_UNIT}']
        for layout, drawing in report_to_ekf_drawings().items():
            card = box_with_text(drawing, "The report's row z")
            texts = [text.content for text in texts_in(drawing, card)]
            self.assertEqual(
                texts[1:], ['mean', expected_values[0], 'std dev', expected_values[1],
                            'variance', expected_values[2]], layout)

    def test_each_layouts_setting_card_shows_the_variance_and_its_root_of_the_page(self) -> None:
        variance = f'{page_setting_text(PAGE_SETTING_VARIANCE)} {VARIANCE_UNIT}'
        standard_deviation = f'{page_setting_text(PAGE_SETTING_STANDARD_DEVIATION)} rad/s'
        for layout, drawing in report_to_ekf_drawings().items():
            card = box_with_text(drawing, "The firmware's setting")
            texts = [text.content for text in texts_in(drawing, card)]
            self.assertEqual(texts[1:], ['angular_velocity_variance', 'variance', variance,
                                         'std dev, its root', standard_deviation], layout)

    def test_each_layout_numbers_the_nine_elements_row_by_row_from_zero(self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            indices = [cell.index_text.lstrip('✓') for cell in covariance_cells(drawing)]
            self.assertEqual(indices, [str(index) for index in range(COVARIANCE_ELEMENT_COUNT)],
                             layout)

    def test_each_layout_has_three_rows_of_three_cells(self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            cells = covariance_cells(drawing)
            self.assertEqual(len({cell.box.y for cell in cells}), 3, layout)
            self.assertEqual(len({cell.box.x for cell in cells}), 3, layout)

    def test_each_layout_has_the_setting_on_the_diagonal_and_zero_elsewhere(self) -> None:
        setting = page_setting_text(PAGE_SETTING_VARIANCE)
        expected_values = [setting if index in COVARIANCE_DIAGONAL_INDICES else '0'
                           for index in range(COVARIANCE_ELEMENT_COUNT)]
        for layout, drawing in report_to_ekf_drawings().items():
            self.assertEqual([cell.value_text for cell in covariance_cells(drawing)],
                             expected_values, layout)

    def test_each_layout_marks_only_the_yaw_rates_element_with_an_edge_and_a_check_mark(
            self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            cells = covariance_cells(drawing)
            self.assertEqual([index for index, cell in enumerate(cells)
                              if cell.box.css_class == 'cell-fused'],
                             [YAW_RATE_VARIANCE_INDEX], layout)
            self.assertEqual([index for index, cell in enumerate(cells)
                              if cell.index_text.startswith('✓')],
                             [YAW_RATE_VARIANCE_INDEX], layout)

    def test_each_layouts_arrow_to_the_ekf_starts_at_the_marked_element(self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            marked_cell, = drawn_boxes(drawing, 'cell-fused')
            filter_box = box_with_text(drawing, 'EKF')
            arrows_from_the_cell = [arrow for arrow in drawn_arrows(drawing, 'flow')
                                    if marked_cell.has_on_its_edge(arrow.start)]
            self.assertEqual(len(arrows_from_the_cell), 1, layout)
            self.assertTrue(filter_box.has_on_its_edge(arrows_from_the_cell[0].tip), layout)

    def test_each_layout_compares_both_cards_with_dashed_arrows_into_the_bands_panel(
            self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            panel = box_with_text(drawing, 'compared by hand')
            arrows = drawn_arrows(drawing, 'compared')
            self.assertEqual(len(arrows), 2, layout)
            for card_title in ("The report's row z", "The firmware's setting"):
                card = box_with_text(drawing, card_title)
                tips = [arrow.tip for arrow in arrows if card.has_on_its_edge(arrow.start)]
                self.assertEqual(len(tips), 1, f'{card_title} in {layout}')
                self.assertTrue(panel.has_on_its_edge(tips[0]), f'{card_title} in {layout}')

    def test_each_layouts_solid_arrow_from_the_setting_ends_above_the_covariance(self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            card = box_with_text(drawing, "The firmware's setting")
            tips = [arrow.tip for arrow in drawn_arrows(drawing, 'flow')
                    if card.has_on_its_edge(arrow.start)]
            self.assertEqual(len(tips), 1, layout)
            cells = covariance_cells(drawing)
            self.assertLess(tips[0].y, cells[0].box.y, layout)
            self.assertGreater(tips[0].x, cells[0].box.x, layout)
            self.assertLess(tips[0].x, cells[-1].box.right, layout)

    def test_both_layouts_show_the_same_texts(self) -> None:
        narrow, wide = (sorted(text.content for text in drawn_texts(drawing))
                        for drawing in report_to_ekf_drawings().values())
        self.assertEqual(narrow, wide)

    def test_each_layout_keeps_its_boxes_inside_the_drawing(self) -> None:
        for layout, drawing in report_to_ekf_drawings().items():
            _, _, width, height = (float(number) for number in
                                   (drawing.get('viewBox') or '').split())
            whole_drawing = DrawnBox('', 0, 0, width, height)
            for css_class in ('item', 'cell', 'cell-fused', 'measured-band', 'setting-band'):
                for box in drawn_boxes(drawing, css_class):
                    self.assertTrue(whole_drawing.holds(DrawnPoint(box.x, box.y))
                                    and whole_drawing.holds(DrawnPoint(box.right, box.bottom)),
                                    f'{box} in {layout}')

    def test_each_layout_has_its_own_width(self) -> None:
        figures = load_script()
        widths = {layout: (drawing.get('viewBox') or '').split()[2]
                  for layout, drawing in report_to_ekf_drawings().items()}
        self.assertEqual(widths, {figures.NARROW_LAYOUT: str(figures.DRAWING_WIDTH),
                                  figures.WIDE_LAYOUT: str(figures.WIDE_WIDTH)})

    def test_connection_is_vertical_between_boxes_that_stand_above_each_other(self) -> None:
        figures = load_script()
        upper, lower = figures.Box(10, 0, 100, 20), figures.Box(50, 60, 100, 20)
        line = figures.connection(upper, lower)
        # The boxes share the x positions from 50 to 110, whose middle is 80
        self.assertEqual(line, figures.Connection(figures.DrawingPoint(80, 20),
                                                  figures.DrawingPoint(80, 60)))

    def test_connection_is_horizontal_between_boxes_that_stand_beside_each_other(self) -> None:
        figures = load_script()
        left, right = figures.Box(0, 10, 40, 60), figures.Box(100, 30, 40, 100)
        line = figures.connection(left, right)
        # The boxes share the y positions from 30 to 70, whose middle is 50
        self.assertEqual(line, figures.Connection(figures.DrawingPoint(40, 50),
                                                  figures.DrawingPoint(100, 50)))

    def test_connection_runs_back_from_a_box_to_a_box_above_it_or_left_of_it(self) -> None:
        figures = load_script()
        upper, lower = figures.Box(10, 0, 100, 20), figures.Box(50, 60, 100, 20)
        self.assertEqual(figures.connection(lower, upper),
                         figures.Connection(figures.DrawingPoint(80, 60),
                                            figures.DrawingPoint(80, 20)))
        left, right = figures.Box(0, 10, 40, 60), figures.Box(100, 30, 40, 100)
        self.assertEqual(figures.connection(right, left),
                         figures.Connection(figures.DrawingPoint(100, 50),
                                            figures.DrawingPoint(40, 50)))

    def test_power_of_ten_text_gives_the_number_its_decimals_and_a_raised_exponent(self) -> None:
        figures = load_script()
        self.assertEqual(figures.power_of_ten_text(4.0e-4, 0), '4 · 10⁻⁴')
        self.assertEqual(figures.power_of_ten_text(5.317e-7, 3), '5.317 · 10⁻⁷')

    def test_firmware_standard_deviation_is_the_root_of_the_settings_variance(self) -> None:
        figures = load_script()
        self.assertAlmostEqual(figures.FIRMWARE_STANDARD_DEVIATION ** 2,
                               figures.FIRMWARE_ANGULAR_VELOCITY_VARIANCE)

    def test_only_the_report_to_ekf_include_carries_the_rules_of_its_bands(self) -> None:
        for include_name in INCLUDE_NAMES:
            text = (ROOT / 'includes' / include_name).read_text(encoding='utf-8')
            self.assertEqual('.measured-band' in text, include_name == REPORT_TO_EKF_INCLUDE,
                             include_name)


if __name__ == '__main__':
    unittest.main()
