"""Tests for scripts/figures/motion_lab.py, which writes the interactive figure's include.

Run from the repository root: python -m unittest discover -s tests
"""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import motion_lab  # noqa: E402


def drive_of(example_id):
    """Return the drive of the guided example with this id."""
    example = next(e for e in motion_lab.EXAMPLES if e.key == example_id)
    return example, motion_lab.example_drive(example)


class TestGuidedExamples(unittest.TestCase):
    """The examples' results equal values worked out by hand from the kinematics.

    Remo's wheels are 0.144 m apart and 0.025 m wide. The test's drives are 0.6 m and 1.5 rad.
    """

    def test_radius_too_big_shortens_the_true_drive_only(self):
        example, drive = drive_of('radius')
        # The controller commands 1 / 1.1 of the wheel turns and multiplies them by 1.1 again
        self.assertAlmostEqual(drive.odometry.forward, 0.6, places=9)
        self.assertAlmostEqual(drive.model.forward, 0.6 / 1.1, places=9)
        self.assertEqual(
            motion_lab.failing_checks(example.command, drive),
            ['odometry and true pose, forward'],
        )

    def test_separation_too_small_turns_the_robot_less_and_passes(self):
        example, drive = drive_of('separation')
        self.assertAlmostEqual(drive.odometry.turned, 1.5, places=9)
        self.assertAlmostEqual(drive.model.turned, 1.5 * 0.95, places=9)
        self.assertEqual(motion_lab.failing_checks(example.command, drive), [])

    def test_frame_behind_the_axle_moves_on_an_arc(self):
        example, drive = drive_of('offset')
        # A point d behind the pivot moves along a chord of 2 d sin(angle / 2)
        chord = 2 * 0.020 * math.sin(1.5 / 2)
        self.assertAlmostEqual(math.hypot(drive.model.forward, drive.model.left), chord, places=9)
        self.assertAlmostEqual(
            math.hypot(drive.odometry.forward, drive.odometry.left), 0.0, places=9
        )
        self.assertEqual(
            motion_lab.failing_checks(example.command, drive),
            ['distance from the start, true pose'],
        )

    def test_contact_at_the_outer_edges_turns_the_robot_less(self):
        example, drive = drive_of('contact')
        self.assertAlmostEqual(drive.model.turned, 1.5 * 0.144 / (0.144 + 0.025), places=9)
        self.assertAlmostEqual(drive.odometry.turned, 1.5, places=9)
        self.assertEqual(
            motion_lab.failing_checks(example.command, drive),
            ['odometry and true pose, turned'],
        )

    def test_slow_wheels_shorten_both_views_alike(self):
        example, drive = drive_of('tracking')
        self.assertAlmostEqual(drive.odometry.forward, 0.6 * 0.8, places=9)
        self.assertAlmostEqual(drive.model.forward, 0.6 * 0.8, places=9)
        self.assertEqual(
            motion_lab.failing_checks(example.command, drive),
            ['forward, odometry', 'forward, true pose'],
        )

    def test_each_example_changes_one_setting(self):
        for example in motion_lab.EXAMPLES:
            self.assertEqual(len(example.settings), 1, example.key)


class TestRobotsOwnValues(unittest.TestCase):
    """With each robot's own values, the three views agree and every check passes."""

    def test_all_checks_pass_for_both_robots_and_both_commands(self):
        for robot in motion_lab.ROBOTS.values():
            for kind, command in motion_lab.COMMANDS.items():
                drive = motion_lab.simulate(robot, command, motion_lab.DEFAULT_SETTINGS)
                self.assertEqual(motion_lab.failing_checks(kind, drive), [], (robot.name, kind))
                for a, b in zip(drive.odometry, drive.model):
                    self.assertAlmostEqual(a, b, places=9)


class TestInclude(unittest.TestCase):
    """The include in the repository is what the script writes, and MkDocs can insert it."""

    def test_include_is_up_to_date(self):
        self.assertEqual(
            motion_lab.INCLUDE.read_text(encoding='utf-8'),
            motion_lab.build(),
            'run python3 scripts/figures/motion_lab.py',
        )

    def test_include_has_no_blank_line_or_template_marker(self):
        output = motion_lab.build()
        self.assertNotIn('\n\n', output)
        for pair in ('{{', '{#', '{%'):
            self.assertNotIn(pair, output)


if __name__ == '__main__':
    unittest.main()
