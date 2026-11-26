"""Tests for the figure scripts of the IMU Simulation pages.

scripts/figures/imu_sim_figures.py, imu_sim_robot_figures.py and imu_sim_test_figures.py write
the pages' includes.

Run from the repository root: python -m unittest discover -s tests
"""

from html.parser import HTMLParser
import math
from pathlib import Path
import re
import sys
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Protocol
from typing import Sequence
from typing import Tuple
import unittest
from unittest import mock
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))
sys.path.insert(0, str(ROOT / 'tests'))

import board_drawings  # noqa: E402
from figure_script_runs import run_with_an_ascii_locale  # noqa: E402
import imu_sim_figures  # noqa: E402
import imu_sim_robot_figures  # noqa: E402
import imu_sim_test_figures  # noqa: E402

# The includes that each script has to write. The list stands here on its own, so that a script
# that drops or renames an include fails a test
INCLUDE_NAMES = {
    'imu_sim_figures.py': ['imu-sim-threads.html'],
    'imu_sim_robot_figures.py': ['imu-sim-angular-velocity.html', 'imu-sim-frames.html',
                                 'imu-sim-specific-force.html'],
    'imu_sim_test_figures.py': ['imu-sim-test-at-rest.html', 'imu-sim-test-left-turn.html',
                                'imu-sim-test-reset.html'],
}


class RadioInput(NamedTuple):
    """A radio input of a tab set: its ID, its group and whether it is selected."""

    element_id: str
    group: str
    checked: bool


class TabLabel(NamedTuple):
    """A label of a tab set: the ID of the input that it switches, and its text."""

    target_id: str
    text: str


class ExpectedTab(NamedTuple):
    """What a tab has to show: its label and the title IDs of the drawings in its block."""

    label: str
    drawing_title_ids: List[str]


class TabSetParser(HTMLParser):
    """Collect the radio inputs, the labels and the drawings per block of an include's tab set."""

    def __init__(self) -> None:
        """Initialize the parser with nothing collected."""
        super().__init__(convert_charrefs=True)
        self.radio_inputs: List[RadioInput] = []
        self.labels: List[TabLabel] = []
        self.title_ids_by_block: List[List[str]] = []
        self.division_depth = 0
        self.block_depth: Optional[int] = None
        self.label_target: Optional[str] = None

    def handle_starttag(self, tag: str, attrs: List[Tuple[str, Optional[str]]]) -> None:
        """Record an input, a label, a block or a drawing's title."""
        attributes = dict(attrs)
        if tag == 'input' and attributes.get('type') == 'radio':
            self.radio_inputs.append(RadioInput(attributes.get('id') or '',
                                                attributes.get('name') or '',
                                                'checked' in attributes))
        elif tag == 'label':
            self.label_target = attributes.get('for') or ''
        elif tag == 'div':
            self.division_depth += 1
            if 'tabbed-block' in (attributes.get('class') or '').split():
                self.block_depth = self.division_depth
                self.title_ids_by_block.append([])
        elif tag == 'title' and self.block_depth is not None:
            self.title_ids_by_block[-1].append(attributes.get('id') or '')

    def handle_data(self, data: str) -> None:
        """Record the text of the label that is open."""
        if self.label_target is not None:
            self.labels.append(TabLabel(self.label_target, data))

    def handle_endtag(self, tag: str) -> None:
        """Close a label, or a block when its own div ends."""
        if tag == 'label':
            self.label_target = None
        elif tag == 'div':
            if self.division_depth == self.block_depth:
                self.block_depth = None
            self.division_depth -= 1


def assert_figure_tabs(test: unittest.TestCase, include_name: str,
                       expected_tabs: Sequence[ExpectedTab]) -> None:
    """Assert that an include's tabs switch, select the first tab and show their drawings.

    Each label has to switch a radio input of its own in one group, in the labels' order, only
    the first input may be selected, and the block at each label's position has to hold exactly
    the expected drawings.
    """
    parser = TabSetParser()
    parser.feed((ROOT / 'includes' / include_name).read_text(encoding='utf-8'))
    radio_ids = [radio.element_id for radio in parser.radio_inputs]
    test.assertEqual(len(radio_ids), len(expected_tabs), include_name)
    test.assertEqual(len(set(radio_ids)), len(radio_ids), radio_ids)
    test.assertEqual(len({radio.group for radio in parser.radio_inputs}), 1, include_name)
    test.assertEqual([radio.checked for radio in parser.radio_inputs],
                     [True] + [False] * (len(expected_tabs) - 1), include_name)
    test.assertEqual([label.target_id for label in parser.labels], radio_ids)
    test.assertEqual([label.text for label in parser.labels],
                     [tab.label for tab in expected_tabs])
    test.assertEqual(parser.title_ids_by_block, [tab.drawing_title_ids for tab in expected_tabs])


class TestIncludes(unittest.TestCase):
    """Each script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the minus sign and the apostrophe.
    """

    def assert_script_writes_the_repository_includes(self, script_name: str) -> None:
        """Assert that the script writes exactly the expected include bytes."""
        written = run_with_an_ascii_locale(script_name)
        self.assertEqual(sorted(written), INCLUDE_NAMES[script_name], script_name)
        for include_name in INCLUDE_NAMES[script_name]:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {script_name} writes')

    def test_thread_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes('imu_sim_figures.py')

    def test_robot_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes('imu_sim_robot_figures.py')

    def test_timeline_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes('imu_sim_test_figures.py')


class TestIncludeComment(unittest.TestCase):
    """Each include starts with the comment that names the script that writes it."""

    def test_each_include_starts_with_the_comment_that_names_its_script(self) -> None:
        for script_name, include_names in INCLUDE_NAMES.items():
            for include_name in include_names:
                text = (ROOT / 'includes' / include_name).read_text(encoding='utf-8')
                self.assertTrue(
                    text.startswith(f'<!-- Written by scripts/figures/{script_name} -->\n'),
                    include_name)


class TestFrameAxes(unittest.TestCase):
    """A frame's axis is an arrow along the drawing and a ring along the line of sight."""

    # A view from above, as the frames' drawing has it: SVG units per m, and where the frame's
    # origin lies on the drawing
    SCALE = 1500
    ORIGIN = board_drawings.Point(170, 152)
    DRAWING_WIDTH, DRAWING_HEIGHT = 340, 306
    FRAME_ORIGIN = imu_sim_robot_figures.Vector3(0.0, 0.0, 0.0)  # m
    AXIS_LENGTH = 42  # SVG units

    def test_only_the_upward_axis_points_along_the_line_of_sight_from_above(self) -> None:
        view = imu_sim_robot_figures.view_from_above(self.SCALE, self.ORIGIN)
        along_the_line_of_sight = [
            imu_sim_robot_figures.points_along_the_line_of_sight(view, direction)
            for direction in imu_sim_robot_figures.IMU_AXES]
        self.assertEqual(along_the_line_of_sight, [False, False, True])

    def frame_drawn(self, view: imu_sim_robot_figures.View) -> ElementTree.Element:
        """Return the SVG of chassis_link's three axes at the frame's origin, seen in a view."""
        drawing = imu_sim_robot_figures.Figure('frame', self.DRAWING_WIDTH, self.DRAWING_HEIGHT,
                                               'title', 'description')
        imu_sim_robot_figures.draw_frame(drawing, view, self.FRAME_ORIGIN,
                                         imu_sim_robot_figures.CHASSIS_AXES, self.AXIS_LENGTH)
        return svg_root(drawing)

    def test_draw_frame_rings_the_upward_axis_at_the_frames_origin_from_above(self) -> None:
        frame = self.frame_drawn(imu_sim_robot_figures.view_from_above(self.SCALE, self.ORIGIN))
        rings = elements_with_class(frame, 'circle', 'ring')
        self.assertEqual([ring.attrib['class'] for ring in rings], ['ring z'])
        self.assertEqual(circle_center(rings[0]), self.ORIGIN)

    def test_draw_frame_puts_a_dot_in_the_ring_of_an_axis_toward_the_viewer(self) -> None:
        frame = self.frame_drawn(imu_sim_robot_figures.view_from_above(self.SCALE, self.ORIGIN))
        dots = elements_with_class(frame, 'circle', 'head')
        self.assertEqual([dot.attrib['class'] for dot in dots], ['head z'])
        self.assertEqual(circle_center(dots[0]), self.ORIGIN)
        self.assertEqual(elements_with_class(frame, 'polyline', 'thin'), [])

    def test_draw_frame_draws_the_axes_along_the_drawing_as_shafts_from_the_origin(self) -> None:
        frame = self.frame_drawn(imu_sim_robot_figures.view_from_above(self.SCALE, self.ORIGIN))
        shafts = {shaft.attrib['class']: polyline_points(shaft)
                  for shaft in elements_with_class(frame, 'polyline', 'shaft')}
        self.assertEqual(sorted(shafts), ['shaft x', 'shaft y'])
        # From above, x points forward, up on the drawing, and y to the left
        self.assertEqual(shafts['shaft x'][0], self.ORIGIN)
        self.assertEqual(shafts['shaft x'][-1].x, self.ORIGIN.x)
        self.assertLess(shafts['shaft x'][-1].y, self.ORIGIN.y)
        self.assertEqual(shafts['shaft y'][0], self.ORIGIN)
        self.assertEqual(shafts['shaft y'][-1].y, self.ORIGIN.y)
        self.assertLess(shafts['shaft y'][-1].x, self.ORIGIN.x)

    def test_draw_frame_puts_a_cross_in_the_ring_of_an_axis_away_from_the_viewer(self) -> None:
        # From the right side, chassis_link's y axis points to the left, away from the viewer
        frame = self.frame_drawn(
            imu_sim_robot_figures.view_from_the_side(self.SCALE, self.ORIGIN))
        rings = elements_with_class(frame, 'circle', 'ring')
        self.assertEqual([ring.attrib['class'] for ring in rings], ['ring y'])
        self.assertEqual(circle_center(rings[0]), self.ORIGIN)
        self.assertEqual(elements_with_class(frame, 'circle', 'head'), [])
        strokes = [polyline_points(stroke)
                   for stroke in elements_with_class(frame, 'polyline', 'thin')]
        self.assertEqual(len(strokes), 2, strokes)
        for first_end, second_end in strokes:
            middle = board_drawings.Point((first_end.x + second_end.x) / 2,
                                          (first_end.y + second_end.y) / 2)
            self.assertEqual(middle, self.ORIGIN)


class TestTextsFromValues(unittest.TestCase):
    """The drawings' texts take their numbers from the scripts' constants."""

    def test_format_centimeters_drops_trailing_zeros_after_two_decimals(self) -> None:
        for meters, text in ((0.0401, '4.01'), (0.013, '1.3'), (0.08, '8'), (0.12346, '12.35')):
            self.assertEqual(imu_sim_robot_figures.format_centimeters(meters), text, f'{meters} m')

    def test_format_degrees_writes_a_minus_sign_for_a_negative_angle(self) -> None:
        quarter_turn_clockwise = -math.pi / 2  # rad
        self.assertEqual(imu_sim_robot_figures.format_degrees(quarter_turn_clockwise),
                         '\u221290°')

    def test_format_degrees_writes_no_sign_for_a_positive_angle(self) -> None:
        half_turn = math.pi  # rad
        self.assertEqual(imu_sim_robot_figures.format_degrees(half_turn), '180°')

    def test_message_values_in_a_sentence_are_along_imu_links_axes(self) -> None:
        # On Remo, imu_link's y axis points forward and its x axis to the right. So a reading
        # forward is the message's y, and a reading to the left is a negative x
        forward_left_and_up = imu_sim_robot_figures.Vector3(0.5, 0.2, 9.8)
        self.assertEqual(
            imu_sim_robot_figures.message_values_in_a_sentence(forward_left_and_up),
            '-0.2, +0.5 and +9.8')

    def test_frames_from_above_figure_names_the_distances_of_another_imu_position(self) -> None:
        # In m, in chassis_link's axes
        behind_left_and_above = imu_sim_robot_figures.Vector3(-0.05, 0.03, 0.02)
        with mock.patch.object(imu_sim_robot_figures, 'IMU_POSITION', behind_left_and_above):
            drawing = imu_sim_robot_figures.frames_from_above_figure()
        self.assertIn('5 cm behind and 3 cm left of chassis_link', drawing.description)
        self.assertIn('imu_joint puts imu_link 5 cm back, 3 cm left', drawing.svg())
        self.assertIn('and 2 cm up, and turns it by \u221290° about z', drawing.svg())

    def test_speeding_up_figure_names_another_magnification_of_the_tip(self) -> None:
        magnification = 5
        with mock.patch.object(imu_sim_robot_figures, 'TIP_MAGNIFICATION', magnification):
            drawing = imu_sim_robot_figures.specific_force_speeding_up_figure()
        self.assertIn('A circle shows the tip of f 5 times larger', drawing.description)
        self.assertIn('>5\u00d7 larger</text>', drawing.svg())

    def test_left_turn_caption_names_the_angle_of_another_turn_rate(self) -> None:
        turn_rate = 1.0  # rad/s
        with mock.patch.object(imu_sim_test_figures, 'TURN_RATE', turn_rate):
            caption = imu_sim_test_figures.left_turn_caption()
        self.assertIn(f'1.0 rad/s turns the robot by 1 rad, about {math.degrees(turn_rate):.0f}°',
                      caption)

    def test_reset_caption_names_the_changed_reset_time(self) -> None:
        run_before_reset = 4.0  # s
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET', run_before_reset):
            caption = imu_sim_test_figures.reset_caption()
        self.assertIn('the reset comes at exactly 4 s', caption)

    def test_reset_caption_names_the_changed_fresh_sample_count(self) -> None:
        fresh_sample_count = 20
        with mock.patch.object(imu_sim_test_figures, 'FRESH_SAMPLE_COUNT', fresh_sample_count):
            caption = imu_sim_test_figures.reset_caption()
        self.assertIn('The test passes with 20 fresh samples', caption)
        # Sample 20 of 50 per second
        self.assertIn('is sample 20, stamped about 0.40 s', caption)

    def test_at_rest_figure_names_the_fewest_messages_of_another_rate_tolerance(self) -> None:
        rate_tolerance = 0.8  # the share of the 50 Hz that the test would need
        with mock.patch.object(imu_sim_test_figures, 'RATE_TOLERANCE', rate_tolerance):
            drawing = imu_sim_test_figures.at_rest_figure()
        # More than 0.8 * 50 Hz * 2 s = 80 messages
        self.assertIn('pass rule: at least 81 arrivals in 2 s', drawing.svg())
        self.assertIn('and needs at least 81.', drawing.description)

    def test_left_turn_figure_names_the_fewest_messages_of_another_rate_tolerance(self) -> None:
        rate_tolerance = 0.8  # the share of the 50 Hz that the test would need
        with mock.patch.object(imu_sim_test_figures, 'RATE_TOLERANCE', rate_tolerance):
            drawing = imu_sim_test_figures.left_turn_figure()
        # More than 0.8 * 50 Hz * 2 s = 80 messages
        self.assertIn('pass rule: at least 81 arrivals in 2 s', drawing.svg())
        self.assertIn('and needs at least 81.', drawing.description)

    def test_at_rest_figure_names_another_yaw_rate_tolerance(self) -> None:
        yaw_rate_tolerance = 0.1  # rad/s
        with mock.patch.object(imu_sim_test_figures, 'YAW_RATE_AT_REST_TOLERANCE',
                               yaw_rate_tolerance):
            drawing = imu_sim_test_figures.at_rest_figure()
        self.assertIn('the mean of their yaw rates within 0.1 rad/s of 0 rad/s',
                      drawing.description)

    def test_specific_force_drawings_call_their_numbers_a_level_noise_free_example(self) -> None:
        drawings = [imu_sim_robot_figures.specific_force_at_rest_figure(),
                    imu_sim_robot_figures.specific_force_speeding_up_figure(),
                    imu_sim_robot_figures.specific_force_in_a_turn_figure()]
        labeled = ['>Level, noise-free example</text>' in drawing.svg()
                   and 'In this level, noise-free example' in drawing.description
                   for drawing in drawings]
        self.assertEqual(labeled, [True, True, True])


class Drawn(Protocol):
    """A figure of one of the three scripts: it returns its drawing as an SVG element."""

    def svg(self) -> str:
        """Return the drawing as an SVG element."""


class Edges(NamedTuple):
    """The edges of an area on a drawing, in SVG units. y grows downward."""

    left: float
    top: float
    right: float
    bottom: float


def svg_root(drawing: Drawn) -> ElementTree.Element:
    """Return the SVG element that a figure writes, parsed as XML."""
    return ElementTree.fromstring(drawing.svg())


def elements_with_class(root: ElementTree.Element, tag: str,
                        css_class: str) -> List[ElementTree.Element]:
    """Return the elements with a tag whose class tokens include css_class, as drawn."""
    return [element for element in root.iter(tag)
            if css_class in element.get('class', '').split()]


def attribute_number(element: ElementTree.Element, name: str) -> float:
    """Return an element's attribute as a number."""
    return float(element.attrib[name])


def vertical_path_positions(path: ElementTree.Element) -> List[float]:
    """Return the y positions that a path of M, V and H commands visits."""
    positions = re.findall(r'M-?[\d.]+,(-?[\d.]+)|V(-?[\d.]+)', path.attrib['d'])
    return [float(after_move or after_vertical) for after_move, after_vertical in positions]


def circle_center(circle: ElementTree.Element) -> board_drawings.Point:
    """Return the center of a circle element."""
    return board_drawings.Point(attribute_number(circle, 'cx'), attribute_number(circle, 'cy'))


def polyline_points(polyline: ElementTree.Element) -> List[board_drawings.Point]:
    """Return the points of a polyline element."""
    return [board_drawings.Point(*(float(number) for number in pair.split(',')))
            for pair in polyline.attrib['points'].split()]


def sample_markers(drawing: imu_sim_test_figures.Figure) -> List[board_drawings.Point]:
    """Return the centers of the dots that a reset panel draws on the IMU's samples."""
    return [circle_center(circle)
            for circle in elements_with_class(svg_root(drawing), 'circle', 'imu-dot')]


class TestDriveTimelineScales(unittest.TestCase):
    """The drive timelines keep what they draw inside their rows when a test's constant changes."""

    LONGER_MEASURE = 3.0  # s, a second longer than the test's: the left turn then ends at 4 s
    # The end of the turn, 4 s, on a time axis that runs from 0 to 4.3 s between x = 40 and 348
    STOP_DOT_X_FOR_THE_LONGER_MEASURE = 326.5
    FASTER_TURN_RATE = 1.0  # rad/s, twice the test's
    # 1 rad/s in a chart that runs from 0 to 1.2 rad/s between y = 256 and 196
    COMMAND_Y_FOR_THE_FASTER_TURN = 206.0
    WIDER_GRAVITY_TOLERANCE = 1.0  # m/s², twice the test's: the test would accept 8.81 to 10.81
    LOWER_GAZEBO_GRAVITY = 8.5  # m/s², below the 9 m/s² at which the chart starts otherwise
    EARLY_RESET = 0.08  # s, so early that a fifth of it is shorter than the IMU's period of 0.02 s

    def assert_rectangle_inside(self, rectangle: ElementTree.Element, edges: Edges) -> None:
        """Assert that all four edges of a rect element lie within the edges."""
        left, top = attribute_number(rectangle, 'x'), attribute_number(rectangle, 'y')
        self.assertGreaterEqual(left, edges.left)
        self.assertGreaterEqual(top, edges.top)
        self.assertLessEqual(left + attribute_number(rectangle, 'width'), edges.right)
        self.assertLessEqual(top + attribute_number(rectangle, 'height'), edges.bottom)
        self.assertGreater(attribute_number(rectangle, 'height'), 0)

    def chart_edges(self) -> Edges:
        """Return the edges of the chart's row on a drive timeline."""
        return Edges(left=imu_sim_test_figures.DRIVE_TIME_LEFT, top=imu_sim_test_figures.CHART.top,
                     right=imu_sim_test_figures.DRIVE_TIME_RIGHT,
                     bottom=imu_sim_test_figures.CHART.bottom)

    def accepted_band(self, drawing: imu_sim_test_figures.Figure) -> ElementTree.Element:
        """Return the rectangle of the accepted band: the first orange wash of a timeline."""
        return elements_with_class(svg_root(drawing), 'rect', 'imu-wash')[0]

    def test_time_axis_ends_shortly_after_the_last_command_of_the_tests(self) -> None:
        self.assertEqual(imu_sim_test_figures.drive_time_scale(),
                         imu_sim_test_figures.Scale(0.0, 3.3, 40.0, 348.0))

    def test_left_turn_puts_the_stop_dot_on_the_time_axis_for_a_longer_measure(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'MEASURE', self.LONGER_MEASURE):
            drawing = imu_sim_test_figures.left_turn_figure()
        stop_dots = elements_with_class(svg_root(drawing), 'circle', 'command-dot')
        self.assertEqual([attribute_number(dot, 'cx') for dot in stop_dots],
                         [self.STOP_DOT_X_FOR_THE_LONGER_MEASURE])
        self.assertLessEqual(self.STOP_DOT_X_FOR_THE_LONGER_MEASURE,
                             imu_sim_test_figures.DRIVE_TIME_RIGHT)

    def test_time_axis_numbers_the_fourth_second_for_a_longer_measure(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'MEASURE', self.LONGER_MEASURE):
            drawing = imu_sim_test_figures.left_turn_figure()
        numbers_y = imu_sim_test_figures.AXIS_Y + 14
        numbers = [(text.text, attribute_number(text, 'x'))
                   for text in svg_root(drawing).iter('text')
                   if attribute_number(text, 'y') == numbers_y]
        self.assertEqual([label for label, _ in numbers], ['0', '1', '2', '3', '4'])
        self.assertEqual(numbers[-1][1], self.STOP_DOT_X_FOR_THE_LONGER_MEASURE)

    def test_left_turn_keeps_the_command_trace_inside_the_chart_for_a_faster_turn(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'TURN_RATE', self.FASTER_TURN_RATE):
            drawing = imu_sim_test_figures.left_turn_figure()
        # The trace steps up and down. The legend's key of the same class is one level line
        traces = [path for path in elements_with_class(svg_root(drawing), 'path', 'command-line')
                  if 'V' in path.attrib['d']]
        self.assertEqual(len(traces), 1, traces)
        chart = self.chart_edges()
        self.assertEqual(vertical_path_positions(traces[0]),
                         [chart.bottom, self.COMMAND_Y_FOR_THE_FASTER_TURN, chart.bottom])
        self.assertGreaterEqual(self.COMMAND_Y_FOR_THE_FASTER_TURN, chart.top)

    def test_left_turn_keeps_the_accepted_band_inside_the_chart_for_a_faster_turn(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'TURN_RATE', self.FASTER_TURN_RATE):
            drawing = imu_sim_test_figures.left_turn_figure()
        self.assert_rectangle_inside(self.accepted_band(drawing), self.chart_edges())

    def test_at_rest_keeps_the_accepted_band_inside_the_chart_for_a_wider_tolerance(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'GRAVITY_TOLERANCE',
                               self.WIDER_GRAVITY_TOLERANCE):
            drawing = imu_sim_test_figures.at_rest_figure()
        self.assert_rectangle_inside(self.accepted_band(drawing), self.chart_edges())

    def test_at_rest_keeps_the_reference_line_inside_the_chart_for_a_lower_gravity(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'GAZEBO_GRAVITY', self.LOWER_GAZEBO_GRAVITY):
            drawing = imu_sim_test_figures.at_rest_figure()
        # The reference line runs to the right end of the time axis. The legend's key is short
        lines = [path for path in elements_with_class(svg_root(drawing), 'path', 'reference-line')
                 if path.attrib['d'].endswith(f'H{imu_sim_test_figures.DRIVE_TIME_RIGHT}')]
        self.assertEqual(len(lines), 1, lines)
        chart = self.chart_edges()
        self.assertEqual(vertical_path_positions(lines[0]), [chart.bottom])

    def test_old_time_panel_marks_the_next_sample_inside_the_plot_for_an_early_reset(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET', self.EARLY_RESET):
            drawing = imu_sim_test_figures.reset_with_the_old_time_figure()
        markers = elements_with_class(svg_root(drawing), 'circle', 'imu-dot')
        self.assertEqual(len(markers), 1, markers)
        self.assertLessEqual(attribute_number(markers[0], 'cx'),
                             imu_sim_test_figures.RESET_PLOT_RIGHT)
        self.assertGreaterEqual(attribute_number(markers[0], 'cy'),
                                imu_sim_test_figures.RESET_PLOT_TOP)


class TestResetPanels(unittest.TestCase):
    """The reset panels mark their samples and number their axis, also for another reset time.

    The plot lies between x = 34 and 292 and between y = 190, at stamp 0, and 60. The expected
    positions below are worked out from these edges, not with the script's scales.
    """

    LATER_RESET = 4.0  # s, a second later than the test's reset
    # With a reset at 4 s, the panels follow the simulation for 4.8 s more: 8.8 s on the
    # horizontal axis and stamps up to 4.8 s on the vertical one. A plugin that keeps the old
    # time publishes again after 8.02 s, with the stamp 4.02 s
    NEXT_SAMPLE_WITH_THE_OLD_TIME = board_drawings.Point(269.1, 81.1)
    # The numbers 0, 4 and 8 below that axis
    RUN_AXIS_NUMBERS_FOR_THE_LATER_RESET = [('0', 34.0), ('4', 151.3), ('8', 268.5)]
    RUN_AXIS_NUMBERS_Y = 204.0  # 14 below the axis at y = 190
    # With the test's reset at 3 s, the axes show 6.6 s and stamps up to 3.6 s. The tenth sample
    # of the copy arrives 3.2 s after the start, with the stamp 0.2 s
    PASSING_SAMPLE_WITH_THE_COPY = board_drawings.Point(159.1, 182.8)

    def assert_inside_the_plot(self, position: board_drawings.Point) -> None:
        """Assert that the point lies within all four plot edges."""
        self.assertGreaterEqual(position.x, imu_sim_test_figures.RESET_PLOT_LEFT)
        self.assertLessEqual(position.x, imu_sim_test_figures.RESET_PLOT_RIGHT)
        self.assertGreaterEqual(position.y, imu_sim_test_figures.RESET_PLOT_TOP)
        self.assertLessEqual(position.y, imu_sim_test_figures.RESET_PLOT_BOTTOM)

    def test_old_time_panel_marks_only_the_next_sample_for_a_later_reset(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET', self.LATER_RESET):
            markers = sample_markers(imu_sim_test_figures.reset_with_the_old_time_figure())
        self.assertEqual(markers, [self.NEXT_SAMPLE_WITH_THE_OLD_TIME])
        self.assert_inside_the_plot(markers[0])

    def test_run_axis_numbers_the_start_the_reset_and_twice_it_for_a_later_reset(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET', self.LATER_RESET):
            drawing = imu_sim_test_figures.reset_with_the_old_time_figure()
        numbers = [(text.text, attribute_number(text, 'x'))
                   for text in svg_root(drawing).iter('text')
                   if attribute_number(text, 'y') == self.RUN_AXIS_NUMBERS_Y]
        self.assertEqual(numbers, self.RUN_AXIS_NUMBERS_FOR_THE_LATER_RESET)
        self.assertLessEqual(numbers[-1][1], imu_sim_test_figures.RESET_PLOT_RIGHT)

    def test_copy_panel_marks_only_the_sample_with_which_the_test_passes(self) -> None:
        markers = sample_markers(imu_sim_test_figures.reset_with_the_copy_figure())
        self.assertEqual(markers, [self.PASSING_SAMPLE_WITH_THE_COPY])


class TestThreadDrawings(unittest.TestCase):
    """The two thread drawings stand in tabs at one place, so what they share must not move."""

    VIEW_BOX = [0.0, 0.0, 360.0, 380.0]
    # The box of the update() that has to take the change: on the physics thread's lane
    UPDATE_BOX = Edges(left=22.0, top=286.0, right=22.0 + 152.0, bottom=286.0 + 38.0)

    def boxes_at_the_awaited_update(self, drawing: imu_sim_figures.Figure) -> List[str]:
        """Return the classes of the rectangles that have exactly the update box's edges."""
        return [rectangle.attrib['class'] for rectangle in svg_root(drawing).iter('rect')
                if Edges(left=attribute_number(rectangle, 'x'),
                         top=attribute_number(rectangle, 'y'),
                         right=(attribute_number(rectangle, 'x')
                                + attribute_number(rectangle, 'width')),
                         bottom=(attribute_number(rectangle, 'y')
                                 + attribute_number(rectangle, 'height'))) == self.UPDATE_BOX]

    def texts_in_the_awaited_update(self, drawing: imu_sim_figures.Figure) -> List[Optional[str]]:
        """Return the text lines whose anchor lies inside the update box, from the top.

        The step's number sits on the box's corner, so it is not inside.
        """
        box = self.UPDATE_BOX
        return [text.text for text in svg_root(drawing).iter('text')
                if 'x' in text.attrib
                and box.left < attribute_number(text, 'x') < box.right
                and box.top < attribute_number(text, 'y') < box.bottom]

    def test_both_drawings_have_the_same_view_box(self) -> None:
        view_boxes = [[float(number) for number in svg_root(drawing).attrib['viewBox'].split()]
                      for drawing in (imu_sim_figures.threads_with_the_original_figure(),
                                      imu_sim_figures.threads_with_the_copy_figure())]
        self.assertEqual(view_boxes, [self.VIEW_BOX, self.VIEW_BOX])

    def test_original_draws_the_awaited_update_as_stuck_at_its_place(self) -> None:
        drawing = imu_sim_figures.threads_with_the_original_figure()
        self.assertEqual(self.boxes_at_the_awaited_update(drawing), ['stuck'])
        self.assertEqual(self.texts_in_the_awaited_update(drawing),
                         ['update()', 'never runs: its thread waits'])

    def test_copy_draws_the_awaited_update_as_running_at_the_same_place(self) -> None:
        drawing = imu_sim_figures.threads_with_the_copy_figure()
        self.assertEqual(self.boxes_at_the_awaited_update(drawing), ['callback'])
        self.assertEqual(self.texts_in_the_awaited_update(drawing),
                         ['update()', 'takes the change'])

    def test_every_arrow_points_to_a_marker_of_its_own_drawing(self) -> None:
        for drawing in (imu_sim_figures.threads_with_the_original_figure(),
                        imu_sim_figures.threads_with_the_copy_figure()):
            root = svg_root(drawing)
            marker_ids = [marker.attrib['id'] for marker in root.iter('marker')]
            self.assertEqual(marker_ids, [f'{drawing.name}-flow', f'{drawing.name}-request'])
            arrows = [path.attrib['marker-end'] for path in root.iter('path')
                      if 'marker-end' in path.attrib]
            self.assertGreater(len(arrows), 0)
            for marker_end in arrows:
                self.assertIn(marker_end, [f'url(#{marker_id})' for marker_id in marker_ids])

    def test_include_has_a_working_tab_per_drawing_under_its_label(self) -> None:
        assert_figure_tabs(self, 'imu-sim-threads.html', [
            ExpectedTab(imu_sim_figures.ORIGINAL_TAB_LABEL, ['th-original-title']),
            ExpectedTab(imu_sim_figures.COPY_TAB_LABEL, ['th-copy-title'])])

    def test_page_has_a_tab_with_each_label_of_the_include(self) -> None:
        page = (ROOT / 'docs' / 'packages' / 'imu' / 'simulation' / 'plugin.md').read_text(
            encoding='utf-8')
        page_tabs = re.findall(r'^=== "([^"]*)"$', page, re.MULTILINE)
        self.assertEqual(page_tabs, [imu_sim_figures.ORIGINAL_TAB_LABEL,
                                     imu_sim_figures.COPY_TAB_LABEL])


class TestFrameTabs(unittest.TestCase):
    """The frames' include has one tab per robot, with the labels of the page's tabs."""

    def test_include_has_a_working_tab_per_robot_under_its_label(self) -> None:
        assert_figure_tabs(self, 'imu-sim-frames.html', [
            ExpectedTab(imu_sim_robot_figures.REMO_TAB_LABEL,
                        ['imu-frames-above-title', 'imu-frames-side-title']),
            ExpectedTab(imu_sim_robot_figures.DIFFBOT_TAB_LABEL, ['imu-frames-diffbot-title'])])

    def test_page_has_a_tab_with_each_label_of_the_include(self) -> None:
        page = (ROOT / 'docs' / 'packages' / 'imu' / 'simulation'
                / 'position-and-axes.md').read_text(encoding='utf-8')
        page_tabs = re.findall(r'^=== "([^"]*)"$', page, re.MULTILINE)
        self.assertEqual(page_tabs, [imu_sim_robot_figures.REMO_TAB_LABEL,
                                     imu_sim_robot_figures.DIFFBOT_TAB_LABEL])


class TestDiffbotMounting(unittest.TestCase):
    """DiffBot's drawing puts imu_link straight above chassis_link, by the model's height."""

    PLACEHOLDER_HEIGHT_M = 0.01  # imu_link above chassis_link in DiffBot's URDF model
    # The magnified box draws 1 m as 1500 units, four times larger: 0.01 m rises by 60 units
    EXPECTED_INSET_RISE = 60.0
    OTHER_HEIGHT_M = 0.015
    EXPECTED_OTHER_RISE = 90.0

    def joint_in_the_magnified_box(
            self, drawing: imu_sim_robot_figures.Figure) -> List[board_drawings.Point]:
        """Return the two ends of the one dashed line that the drawing has for imu_joint."""
        joints = elements_with_class(svg_root(drawing), 'polyline', 'joint')
        self.assertEqual(len(joints), 1, joints)
        return polyline_points(joints[0])

    def test_diffbot_drawing_draws_imu_joint_straight_up_by_the_models_height(self) -> None:
        self.assertEqual(imu_sim_robot_figures.DIFFBOT_IMU_POSITION.z, self.PLACEHOLDER_HEIGHT_M)
        chassis, imu = self.joint_in_the_magnified_box(
            imu_sim_robot_figures.frames_on_diffbot_figure())
        self.assertEqual(imu.x - chassis.x, 0.0)
        self.assertAlmostEqual(chassis.y - imu.y, self.EXPECTED_INSET_RISE, places=2)

    def test_diffbot_drawing_draws_imu_joint_as_long_as_another_height(self) -> None:
        other_position = imu_sim_robot_figures.Vector3(0.0, 0.0, self.OTHER_HEIGHT_M)
        with mock.patch.object(imu_sim_robot_figures, 'DIFFBOT_IMU_POSITION', other_position):
            chassis, imu = self.joint_in_the_magnified_box(
                imu_sim_robot_figures.frames_on_diffbot_figure())
        self.assertEqual(imu.x - chassis.x, 0.0)
        self.assertAlmostEqual(chassis.y - imu.y, self.EXPECTED_OTHER_RISE, places=2)

    def test_diffbot_drawing_names_the_models_height(self) -> None:
        drawing = imu_sim_robot_figures.frames_on_diffbot_figure()
        self.assertIn('>1 cm up, not turned</text>', drawing.svg())

    def test_diffbot_description_says_that_the_mounting_is_not_measured(self) -> None:
        drawing = imu_sim_robot_figures.frames_on_diffbot_figure()
        self.assertIn('It is not measured on the robot.', drawing.description)


class TestTurnReading(unittest.TestCase):
    """In a left turn on the spot, the IMU's acceleration points to the middle of the axle."""

    # Remo's IMU lies 0.0761 m behind the axle and 0.0234 m to its left. At 0.5 rad/s, the
    # acceleration is 0.25 1/s² times that offset, toward the axle: forward and to the right
    EXPECTED_ACCELERATION = imu_sim_robot_figures.Vector3(0.019025, -0.00585, 0.0)  # m/s²
    # imu_link's x axis points to the robot's right and its y axis forward
    EXPECTED_MESSAGE = '(+0.01, +0.02, +9.8)'
    # On the drawing, 900 units per m around (108, 142), front at the top: the IMU and the
    # middle of the axle
    SENSOR = board_drawings.Point(86.94, 178.09)
    AXLE_MIDDLE = board_drawings.Point(108.0, 109.6)

    def test_turn_acceleration_points_from_the_imu_to_the_middle_of_the_axle(self) -> None:
        acceleration = imu_sim_robot_figures.turn_acceleration()
        for component, expected in zip(acceleration, self.EXPECTED_ACCELERATION):
            self.assertAlmostEqual(component, expected, places=6)

    def test_centripetal_acceleration_is_the_turn_accelerations_length(self) -> None:
        self.assertAlmostEqual(imu_sim_robot_figures.centripetal_acceleration(),
                               math.hypot(self.EXPECTED_ACCELERATION.x,
                                          self.EXPECTED_ACCELERATION.y), places=6)

    def test_left_turn_reading_points_toward_the_axle_in_remos_imu_axes(self) -> None:
        root = svg_root(imu_sim_robot_figures.specific_force_in_a_turn_figure())
        readings = [text.text for text in elements_with_class(root, 'text', 'reading')]
        self.assertIn(self.EXPECTED_MESSAGE, readings)

    def test_left_turn_draws_the_acceleration_from_the_imu_toward_the_axle(self) -> None:
        root = svg_root(imu_sim_robot_figures.specific_force_in_a_turn_figure())
        shafts = [polyline_points(shaft)
                  for shaft in elements_with_class(root, 'polyline', 'acceleration')]
        self.assertEqual(len(shafts), 1, shafts)
        start, end = shafts[0]
        self.assertEqual(start, self.SENSOR)
        to_the_axle = math.atan2(self.AXLE_MIDDLE.y - self.SENSOR.y,
                                 self.AXLE_MIDDLE.x - self.SENSOR.x)
        self.assertAlmostEqual(math.atan2(end.y - start.y, end.x - start.x), to_the_axle,
                               places=2)


if __name__ == '__main__':
    unittest.main()
