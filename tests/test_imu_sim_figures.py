"""Tests for the figure scripts of the IMU Simulation pages.

scripts/figures/imu_sim_figures.py, imu_sim_robot_figures.py and imu_sim_test_figures.py write
the pages' includes.

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
from typing import Sequence
from typing import Tuple
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import board_drawings  # noqa: E402
import imu_sim_figures  # noqa: E402
import imu_sim_robot_figures  # noqa: E402
import imu_sim_test_figures  # noqa: E402

# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale(script_name: str) -> Dict[str, bytes]:
    """Run a figure script with an ASCII locale and return the includes it writes, by file name.

    The script runs in an empty folder, so the repository's includes stay as they are.
    """
    script = ROOT / 'scripts' / 'figures' / script_name
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        run = subprocess.run([sys.executable, str(script)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{script_name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


class TestIncludes(unittest.TestCase):
    """Each script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the minus sign and the apostrophe.
    """

    def assert_script_writes_the_repository_includes(self, script_name: str,
                                                     include_names: Sequence[str]) -> None:
        written = run_with_an_ascii_locale(script_name)
        self.assertEqual(sorted(written), sorted(include_names), script_name)
        for include_name in include_names:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {script_name} writes')

    def test_thread_figures_write_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes(
            'imu_sim_figures.py', ['imu-sim-threads.html'])

    def test_robot_figures_write_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes(
            'imu_sim_robot_figures.py',
            ['imu-sim-frames.html', 'imu-sim-angular-velocity.html',
             'imu-sim-specific-force.html'])

    def test_test_figures_write_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes(
            'imu_sim_test_figures.py',
            ['imu-sim-test-at-rest.html', 'imu-sim-test-left-turn.html',
             'imu-sim-test-reset.html'])


class TestIncludeComment(unittest.TestCase):
    """Each include starts with the comment that names the script that writes it."""

    def test_each_include_starts_with_the_comment_that_names_its_script(self) -> None:
        scripts = {
            'imu-sim-threads.html': 'imu_sim_figures.py',
            'imu-sim-frames.html': 'imu_sim_robot_figures.py',
            'imu-sim-angular-velocity.html': 'imu_sim_robot_figures.py',
            'imu-sim-specific-force.html': 'imu_sim_robot_figures.py',
            'imu-sim-test-at-rest.html': 'imu_sim_test_figures.py',
            'imu-sim-test-left-turn.html': 'imu_sim_test_figures.py',
            'imu-sim-test-reset.html': 'imu_sim_test_figures.py',
        }
        for include_name, script_name in scripts.items():
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

    def test_draw_frame_adds_a_ring_for_the_axis_toward_the_viewer(self) -> None:
        view = imu_sim_robot_figures.view_from_above(self.SCALE, self.ORIGIN)
        drawing = imu_sim_robot_figures.Figure('frame', self.DRAWING_WIDTH, self.DRAWING_HEIGHT,
                                               'title', 'description')
        imu_sim_robot_figures.draw_frame(drawing, view, self.FRAME_ORIGIN,
                                         imu_sim_robot_figures.CHASSIS_AXES, self.AXIS_LENGTH)
        rings = [element for element in drawing.elements if 'class="ring ' in element]
        self.assertEqual(len(rings), 1, rings)
        self.assertIn('class="ring z"', rings[0])


class TestTextsFromValues(unittest.TestCase):
    """The drawings' texts take their numbers from the scripts' constants."""

    def test_centimeters_drops_trailing_zeros_after_two_decimals(self) -> None:
        for meters, text in ((0.0401, '4.01'), (0.013, '1.3'), (0.08, '8'), (0.12346, '12.35')):
            self.assertEqual(imu_sim_robot_figures.centimeters(meters), text, f'{meters} m')

    def test_degrees_writes_a_minus_sign_for_a_negative_angle(self) -> None:
        self.assertEqual(imu_sim_robot_figures.degrees(-math.pi / 2), '\u221290°')
        self.assertEqual(imu_sim_robot_figures.degrees(math.pi), '180°')

    def test_message_values_in_a_sentence_are_along_imu_links_axes(self) -> None:
        # On Remo, imu_link's y axis points forward: a reading forward is the message's y
        forward_and_up = imu_sim_robot_figures.Vector3(0.5, 0.0, 9.8)
        self.assertEqual(imu_sim_robot_figures.message_values_in_a_sentence(forward_and_up),
                         '0, +0.5 and +9.8')

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

    def test_reset_caption_names_another_reset_time_and_sample_count(self) -> None:
        run_before_reset = 4.0  # s
        fresh_sample_count = 20
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET', run_before_reset), \
                mock.patch.object(imu_sim_test_figures, 'FRESH_SAMPLE_COUNT', fresh_sample_count):
            caption = imu_sim_test_figures.reset_caption()
        self.assertIn('the reset comes at exactly 4 s', caption)
        self.assertIn('The test passes with 20 fresh samples', caption)

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


def sample_markers(drawing: imu_sim_test_figures.Figure) -> List[Tuple[float, float]]:
    """Return the positions of the dots that a reset panel draws on the IMU's samples."""
    return [(float(x), float(y)) for x, y in re.findall(
        r'<circle class="imu-dot" cx="([0-9.]+)" cy="([0-9.]+)"', drawing.svg())]


class TestResetPanels(unittest.TestCase):
    """The reset panels keep their marks inside the plot, also for another reset time."""

    OTHER_RUN_BEFORE_RESET = 4.0  # s, later than the test's 3 s

    def assert_inside_the_plot(self, position: Tuple[float, float]) -> None:
        x, y = position
        self.assertGreaterEqual(x, imu_sim_test_figures.RESET_PLOT_LEFT)
        self.assertLessEqual(x, imu_sim_test_figures.RESET_PLOT_RIGHT)
        self.assertGreaterEqual(y, imu_sim_test_figures.RESET_PLOT_TOP)
        self.assertLessEqual(y, imu_sim_test_figures.RESET_PLOT_BOTTOM)

    def test_old_time_panel_marks_the_next_sample_inside_the_plot_for_a_later_reset(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET',
                               self.OTHER_RUN_BEFORE_RESET):
            markers = sample_markers(imu_sim_test_figures.reset_with_the_old_time_figure())
        self.assertEqual(len(markers), 1, markers)
        self.assert_inside_the_plot(markers[0])

    def test_run_axis_numbers_no_time_beyond_the_plot_for_a_later_reset(self) -> None:
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET',
                               self.OTHER_RUN_BEFORE_RESET):
            run = imu_sim_test_figures.reset_run()
            last_tick = imu_sim_test_figures.run_axis_ticks()[-1]
            first_sample_with_the_old_time = (
                2 * self.OTHER_RUN_BEFORE_RESET + imu_sim_test_figures.IMU_PERIOD)
        self.assertEqual(last_tick, 2 * self.OTHER_RUN_BEFORE_RESET)
        self.assertLessEqual(run.position(last_tick), imu_sim_test_figures.RESET_PLOT_RIGHT)
        self.assertLess(first_sample_with_the_old_time, run.last_value)

    def test_copy_panel_marks_the_sample_with_which_the_test_passes(self) -> None:
        markers = sample_markers(imu_sim_test_figures.reset_with_the_copy_figure())
        passing_stamp = (imu_sim_test_figures.FRESH_SAMPLE_COUNT
                         * imu_sim_test_figures.IMU_PERIOD)  # s after the reset
        expected = (
            imu_sim_test_figures.reset_run().position(
                imu_sim_test_figures.RUN_BEFORE_RESET + passing_stamp),
            imu_sim_test_figures.reset_stamp().position(passing_stamp))
        self.assertEqual(markers, [expected])


class TestThreadDrawings(unittest.TestCase):
    """The two thread drawings stand in tabs at one place, so what they share must not move."""

    def test_both_drawings_have_the_same_size(self) -> None:
        original = imu_sim_figures.threads_with_the_original_figure()
        copy = imu_sim_figures.threads_with_the_copy_figure()
        self.assertEqual(original.height, copy.height)

    def test_both_drawings_put_the_awaited_update_at_the_same_place(self) -> None:
        # The box of the update() that has to take the change: its upper left corner
        box = (f'x="{imu_sim_figures.PHYSICS_LANE.left + 4}" '
               f'y="{imu_sim_figures.UPDATE_BOX_Y}"')
        drawings = [imu_sim_figures.threads_with_the_original_figure(),
                    imu_sim_figures.threads_with_the_copy_figure()]
        boxes_at_that_place = [drawing.svg().count(f'<rect class="stuck" {box}')
                               + drawing.svg().count(f'<rect class="callback" {box}')
                               for drawing in drawings]
        self.assertEqual(boxes_at_that_place, [1, 1])

    def test_include_has_one_tab_per_drawing_with_the_pages_labels(self) -> None:
        include = (ROOT / 'includes' / 'imu-sim-threads.html').read_text(encoding='utf-8')
        labels = re.findall(r'<label for="__tabbed_th_\d">([^<]*)</label>', include)
        tab_labels = [imu_sim_figures.ORIGINAL_TAB_LABEL, imu_sim_figures.COPY_TAB_LABEL]
        self.assertEqual(labels, tab_labels)
        page = (ROOT / 'docs' / 'packages' / 'imu' / 'simulation' / 'plugin.md').read_text(
            encoding='utf-8')
        page_tabs = re.findall(r'^=== "([^"]*)"$', page, re.MULTILINE)
        self.assertEqual(page_tabs, tab_labels)


class TestFrameTabs(unittest.TestCase):
    """The frames' include has one tab per robot, with the labels of the page's tabs."""

    def test_include_and_page_use_the_same_robot_labels(self) -> None:
        include = (ROOT / 'includes' / 'imu-sim-frames.html').read_text(encoding='utf-8')
        labels = re.findall(r'<label for="__tabbed_imuframes_\d">([^<]*)</label>', include)
        tab_labels = [imu_sim_robot_figures.REMO_TAB_LABEL,
                      imu_sim_robot_figures.DIFFBOT_TAB_LABEL]
        self.assertEqual(labels, tab_labels)
        page = (ROOT / 'docs' / 'packages' / 'imu' / 'simulation'
                / 'position-and-axes.md').read_text(encoding='utf-8')
        page_tabs = re.findall(r'^=== "([^"]*)"$', page, re.MULTILINE)
        self.assertEqual(page_tabs, tab_labels)

    def test_diffbot_drawing_puts_imu_link_above_chassis_link_by_the_models_height(self) -> None:
        drawing = imu_sim_robot_figures.frames_on_diffbot_figure()
        self.assertIn('>1 cm up, not turned</text>', drawing.svg())
        self.assertIn('It is not measured on the robot.', drawing.description)


if __name__ == '__main__':
    unittest.main()
