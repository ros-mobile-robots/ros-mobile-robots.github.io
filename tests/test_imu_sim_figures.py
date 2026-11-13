"""Tests for the figure scripts of the IMU Simulation pages.

scripts/figures/imu_sim_figures.py, imu_sim_robot_figures.py and imu_sim_test_figures.py write
the pages' includes.

Run from the repository root: python -m unittest discover -s tests
"""

import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Dict
from typing import Sequence
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

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
            'imu_sim_figures.py',
            ['imu-sim-threads-original.html', 'imu-sim-threads-copy.html'])

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


class TestFrameAxes(unittest.TestCase):
    """A frame's axis is an arrow along the drawing and a ring along the line of sight."""

    def test_only_the_upward_axis_points_along_the_line_of_sight_from_above(self) -> None:
        view = imu_sim_robot_figures.view_from_above(scale=1500, origin=(170, 152))
        along_the_line_of_sight = [
            imu_sim_robot_figures.points_along_the_line_of_sight(view, direction)
            for direction in imu_sim_robot_figures.IMU_AXES]
        self.assertEqual(along_the_line_of_sight, [False, False, True])

    def test_draw_frame_adds_a_ring_for_the_axis_toward_the_viewer(self) -> None:
        view = imu_sim_robot_figures.view_from_above(scale=1500, origin=(170, 152))
        drawing = imu_sim_robot_figures.Figure('frame', 340, 306, 'title', 'description')
        imu_sim_robot_figures.draw_frame(drawing, view, (0.0, 0.0, 0.0),
                                         imu_sim_robot_figures.CHASSIS_AXES, length=42)
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
        forward_and_up = (0.5, 0.0, 9.8)
        self.assertEqual(imu_sim_robot_figures.message_values_in_a_sentence(forward_and_up),
                         '0, +0.5 and +9.8')

    def test_frames_from_above_figure_names_the_distances_of_another_imu_position(self) -> None:
        behind_left_and_above = (-0.05, 0.03, 0.02)  # m, in chassis_link's axes
        with mock.patch.object(imu_sim_robot_figures, 'IMU_POSITION', behind_left_and_above):
            drawing = imu_sim_robot_figures.frames_from_above_figure()
        self.assertIn('5 cm behind and 3 cm left of chassis_link', drawing.description)
        self.assertIn('imu_joint puts imu_link 5 cm back, 3 cm left', drawing.svg())
        self.assertIn('and 2 cm up, and turns it by \u221290° about z', drawing.svg())

    def test_speeding_up_figure_names_another_magnification_of_the_tip(self) -> None:
        with mock.patch.object(imu_sim_robot_figures, 'TIP_MAGNIFICATION', 5):
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
        with mock.patch.object(imu_sim_test_figures, 'RUN_BEFORE_RESET', 4.0), \
                mock.patch.object(imu_sim_test_figures, 'FRESH_SAMPLE_COUNT', 20):
            caption = imu_sim_test_figures.reset_caption()
        self.assertIn('the reset comes at exactly 4 s', caption)
        self.assertIn('The test passes with 20 fresh samples', caption)


if __name__ == '__main__':
    unittest.main()
