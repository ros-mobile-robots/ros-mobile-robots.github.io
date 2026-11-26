"""Tests for scripts/figures/imu_noise_figures.py.

The script writes the figures about the IMU's bias and noise at rest as eight includes.

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
from typing import Sequence
from typing import Tuple
import unittest
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
FIGURES = ROOT / 'scripts' / 'figures'
SCRIPT = FIGURES / 'imu_noise_figures.py'
TESTS_PAGE = ROOT / 'docs' / 'packages' / 'imu' / 'noise-at-rest-tests.md'
# An entry of the page's list "The test classes": the names of its classes, in a link or as
# inline code, then their numbers of tests, such as "2 + 2 tests:"
CLASS_LIST_ENTRY = re.compile(
    r'- \*\*(?:\[(?P<names>[^\]]+)\]\([^)]*\)|(?P<plain>`Test[^*]*?)), '
    r'(?P<counts>[\d +]+) tests:\*\*')
# The includes whose figure has a narrow and a wide layout
TWO_LAYOUT_INCLUDES = ['imu-noise-test-flow.html', 'imu-noise-test-intervals.html',
                       'imu-noise-test-stamps-back.html', 'imu-noise-trace.html']
INCLUDE_NAMES = ['imu-noise-test-flow.html', 'imu-noise-test-gravity.html',
                 'imu-noise-test-intervals.html', 'imu-noise-test-measured.html',
                 'imu-noise-test-stamps-back.html', 'imu-noise-test-variance.html',
                 'imu-noise-three-readings.html', 'imu-noise-trace.html']
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


def two_steps(figures: ModuleType) -> Sequence[Any]:
    """Return two made-up steps: one with three test classes and nine tests, one with one."""
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
            counts.extend(zip(names, (int(count) for count in entry['counts'].split('+'))))
    return counts


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

    def test_stamps_back_name_names_the_repeat_the_step_back_and_the_last_stamp(self) -> None:
        figures = load_script()
        self.assertEqual(
            figures.stamps_back_name(figures.STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK),
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
            label = next(index for index, element in enumerate(elements)
                         if element.tag == 'text' and (element.text or '').startswith('mean:'))
            self.assertEqual(elements[label - 1].get('class'), 'mean-line', layout.name)

    def test_each_trace_layout_draws_the_first_seconds_readings_and_the_legends_dot(
            self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        shown = figures.TRACE_SHOWN_DURATION * figures.SAMPLE_RATE
        for layout in figures.TRACE_LAYOUTS:
            svg = figures.trace_figure(trace, layout).svg()
            self.assertEqual(svg.count('class="reading"'), shown + 1, layout.name)
            self.assertIn(f'viewBox="0 0 {layout.width} ', svg)

    def test_trace_x_spans_the_plot_of_each_layout(self) -> None:
        figures = load_script()
        for layout in figures.TRACE_LAYOUTS:
            self.assertEqual(figures.trace_x(0, layout), figures.TRACE_PLOT_LEFT, layout.name)
            self.assertEqual(figures.trace_x(figures.TRACE_SHOWN_DURATION, layout),
                             layout.plot_right, layout.name)

    def test_each_trace_layout_has_its_ticks_and_every_legend_entry_once(self) -> None:
        figures = load_script()
        trace = figures.synthetic_trace()
        # A tick every 0.5 s in the narrow layout and every 0.2 s in the wide one, over 1 s
        tick_counts = {'imu-noise-trace': 3, 'imu-noise-trace-wide': 6}
        for layout in figures.TRACE_LAYOUTS:
            svg = figures.trace_figure(trace, layout).svg()
            self.assertEqual(svg.count('class="tick"'), tick_counts[layout.name], layout.name)
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
            self.assertIn('<div class="lv-space">', text, include_name)
            self.assertIn('class="lv-fig lv-layout-narrow"', text, include_name)
            self.assertIn('class="lv-fig lv-layout-wide"', text, include_name)


class TestPlotLayouts(unittest.TestCase):
    """A stamp figure's wide layout is its narrow one with a longer time axis."""

    def test_wide_stamp_figures_put_each_mark_stretch_times_as_far_right(self) -> None:
        figures = load_script()
        narrow, wide = figures.PLOT_LAYOUTS
        for drawing in (figures.intervals_figure, figures.stamps_back_figure):
            narrow_x = [float(x) for x in re.findall(r'<circle[^>]* cx="([\d.]+)"',
                                                     drawing(narrow).svg())]
            wide_x = [float(x) for x in re.findall(r'<circle[^>]* cx="([\d.]+)"',
                                                   drawing(wide).svg())]
            self.assertEqual(len(wide_x), len(narrow_x))
            for narrow_value, wide_value in zip(narrow_x, wide_x):
                self.assertAlmostEqual(wide_value, narrow_value * wide.stretch, delta=0.1)

    def test_wide_stamp_figures_are_as_wide_as_the_wide_layout(self) -> None:
        figures = load_script()
        wide = figures.PLOT_LAYOUTS[1]
        for drawing in (figures.intervals_figure, figures.stamps_back_figure):
            self.assertEqual(drawing(wide).width, figures.WIDE_WIDTH, drawing.__name__)

    def test_a_stamp_that_does_not_advance_keeps_its_labels_distance_in_both_layouts(
            self) -> None:
        figures = load_script()
        distances = []
        for layout in figures.PLOT_LAYOUTS:
            svg = figures.stamps_back_figure(layout).svg()
            second_stamp_x = float(re.findall(r'<circle[^>]* cx="([\d.]+)"', svg)[1])
            label_x = float(re.findall(r'<text[^>]* x="([\d.]+)"[^>]*>repeated</text>', svg)[0])
            distances.append(label_x - second_stamp_x)
        self.assertGreater(distances[0], 0)
        self.assertAlmostEqual(distances[0], distances[1], delta=0.1)


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
        self.assertEqual(figures.middle_inside_margins(10, 150), left + 75)
        self.assertEqual(figures.middle_inside_margins(right, 170), right - 85)
        self.assertEqual(figures.middle_inside_margins(270, 100), 270)

    def test_wide_flow_layout_keeps_every_card_inside_its_margins(self) -> None:
        figures = load_script()
        left, right = figures.WIDE_FLOW_MARGIN, figures.WIDE_WIDTH - figures.WIDE_FLOW_MARGIN
        cards = re.findall(r'<rect [^>]*x="(-?[\d.]+)"[^>]* width="([\d.]+)"',
                           figures.wide_flow_figure().svg())
        self.assertEqual(len(cards), len(figures.TOOL_STEPS) + 3)
        for x, width in cards:
            self.assertGreaterEqual(float(x), left)
            # The drawing's numbers have one decimal
            self.assertLessEqual(float(x) + float(width), right + 0.1)

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

    def test_flow_name_names_each_step_with_its_function(self) -> None:
        figures = load_script()
        self.assertEqual(
            figures.flow_name(two_steps(figures)),
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


if __name__ == '__main__':
    unittest.main()
