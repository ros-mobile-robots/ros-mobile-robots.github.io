"""Tests for scripts/figures/motion_lab_figures.py, which writes the interactive figure's include.

Run from the repository root: python -m unittest discover -s tests
"""

import json
import math
from pathlib import Path
import re
import sys
from typing import Any
from typing import Dict
from typing import Tuple
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))

import motion_lab_figures as figures  # noqa: E402

# The model's results are exact up to floating-point rounding
PLACES = 9
REMO = figures.ROBOTS[figures.EXAMPLE_ROBOT]
FORWARD = figures.COMMANDS[figures.CommandKind.FORWARD]
TURN = figures.COMMANDS[figures.CommandKind.TURN]


def drive_of(example_key: str) -> Tuple[figures.Example, figures.Drive]:
    """Return the guided example with this key and its drive."""
    example = next(example for example in figures.EXAMPLES if example.key == example_key)
    return example, figures.example_drive(example)


def drive_with(command: figures.CommandKind, **settings: int) -> figures.Drive:
    """Return Remo's drive of a command with some settings changed."""
    return figures.simulate(REMO, figures.COMMANDS[command], figures.Settings(**settings))


class TestGuidedExamples(unittest.TestCase):
    """The examples' results equal values worked out by hand from the kinematics.

    The drives are the test's commands, FORWARD.distance and TURN.angle, with Remo's wheels.
    """

    def test_a_radius_too_big_shortens_only_the_true_drive(self) -> None:
        example, drive = drive_of('radius')
        factor = example.settings.radius_percent / 100
        # The controller commands 1 / factor of the wheel turns, and the odometry multiplies them
        # by the factor again
        self.assertAlmostEqual(drive.odometry.forward, FORWARD.distance, places=PLACES)
        self.assertAlmostEqual(drive.model.forward, FORWARD.distance / factor, places=PLACES)
        self.assertEqual(
            figures.failing_checks(example.command, drive), ['odometry and true pose, forward']
        )

    def test_a_separation_too_small_turns_the_robot_less_and_every_check_passes(self) -> None:
        example, drive = drive_of('separation')
        factor = example.settings.separation_percent / 100
        self.assertAlmostEqual(drive.odometry.turned, TURN.angle, places=PLACES)
        self.assertAlmostEqual(drive.model.turned, TURN.angle * factor, places=PLACES)
        self.assertEqual(figures.failing_checks(example.command, drive), [])

    def test_a_frame_behind_the_axle_ends_a_chord_away_and_fails_the_distance_check(self) -> None:
        example, drive = drive_of('offset')
        offset = example.settings.offset_mm / 1000
        # A point at the distance d from the pivot moves along a chord of 2 d sin(angle / 2)
        chord = 2 * offset * math.sin(TURN.angle / 2)
        self.assertAlmostEqual(math.hypot(drive.model.forward, drive.model.left), chord,
                               places=PLACES)
        self.assertAlmostEqual(math.hypot(drive.odometry.forward, drive.odometry.left), 0.0,
                               places=PLACES)
        self.assertEqual(
            figures.failing_checks(example.command, drive), ['distance from the start, true pose']
        )

    def test_contact_at_the_outer_edges_turns_the_robot_less_than_its_odometry(self) -> None:
        example, drive = drive_of('contact')
        wheels = REMO.wheels
        # At 100 % the wheels push at their outer edges, a wheel's width further apart
        contact_separation = wheels.separation + wheels.width
        self.assertAlmostEqual(drive.model.turned, TURN.angle * wheels.separation
                               / contact_separation, places=PLACES)
        self.assertAlmostEqual(drive.odometry.turned, TURN.angle, places=PLACES)
        self.assertEqual(
            figures.failing_checks(example.command, drive), ['odometry and true pose, turned']
        )

    def test_slow_wheels_shorten_both_views_alike(self) -> None:
        example, drive = drive_of('tracking')
        share = example.settings.tracking_percent / 100
        self.assertAlmostEqual(drive.odometry.forward, FORWARD.distance * share, places=PLACES)
        self.assertAlmostEqual(drive.model.forward, FORWARD.distance * share, places=PLACES)
        self.assertEqual(
            figures.failing_checks(example.command, drive),
            ['forward, odometry', 'forward, true pose'],
        )

    def test_each_example_changes_one_setting(self) -> None:
        for example in figures.EXAMPLES:
            self.assertEqual(len(figures.changed_settings(example.settings)), 1, example.key)

    def test_every_example_text_has_all_its_values_filled_in(self) -> None:
        for example in figures.EXAMPLES:
            values = figures.example_values(example)
            for text in (example.question, example.answer):
                filled = text.format(**values)
                self.assertNotRegex(filled, r'[{}]', example.key)


class TestChecks(unittest.TestCase):
    """The checks follow test_drive.py: their order, their tolerances and their boundaries."""

    def test_the_rows_follow_the_tests_order_each_view_then_the_agreement(self) -> None:
        names = {
            kind: [row.name for row in figures.check_rows(kind)] for kind in figures.CommandKind
        }
        self.assertEqual(names[figures.CommandKind.FORWARD], [
            'forward, odometry', 'left, odometry', 'turned, odometry',
            'forward, true pose', 'left, true pose', 'turned, true pose',
            'odometry and true pose, forward',
        ])
        self.assertEqual(names[figures.CommandKind.TURN], [
            'turned, odometry', 'distance from the start, odometry',
            'turned, true pose', 'distance from the start, true pose',
            'odometry and true pose, turned',
        ])

    def test_a_value_on_a_tolerance_boundary_passes(self) -> None:
        turned = figures.CHECKS[figures.CommandKind.TURN].each_view[0]
        boundary = turned.expected - turned.tolerance
        # The wheels reach the share of their speed that ends the turn exactly on the boundary
        tracking = round(boundary / TURN.angle * 100)
        drive = drive_with(figures.CommandKind.TURN, tracking_percent=tracking)
        self.assertAlmostEqual(drive.model.turned, boundary, places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.TURN, drive), [])

    def test_the_turn_test_catches_a_separation_off_by_more_than_the_stated_share(self) -> None:
        share = int(figures.check_values(figures.CommandKind.TURN)['agreement_share'])
        agreement = 'odometry and true pose, turned'
        inside = drive_with(figures.CommandKind.TURN, separation_percent=100 - (share - 1))
        outside = drive_with(figures.CommandKind.TURN, separation_percent=100 - share)
        self.assertNotIn(agreement, figures.failing_checks(figures.CommandKind.TURN, inside))
        self.assertIn(agreement, figures.failing_checks(figures.CommandKind.TURN, outside))


class TestRobotsOwnValues(unittest.TestCase):
    """With each robot's own values, the three views agree and every check passes."""

    def test_every_check_passes_for_both_robots_and_both_commands(self) -> None:
        for robot in figures.ROBOTS.values():
            for kind, command in figures.COMMANDS.items():
                drive = figures.simulate(robot, command, figures.DEFAULT_SETTINGS)
                self.assertEqual(figures.failing_checks(kind, drive), [], (robot.name, kind))
                for odometry, model in zip(drive.odometry, drive.model):
                    self.assertAlmostEqual(odometry, model, places=PLACES)


class TestDrawing(unittest.TestCase):
    """The static drawing's helpers: straight runs, the reach and the grid."""

    def test_a_straight_run_keeps_only_its_ends(self) -> None:
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 1.0)]
        self.assertEqual(
            figures.without_straight_runs(points), [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0)]
        )

    def test_a_point_off_the_line_by_more_than_a_tenth_of_a_pixel_stays(self) -> None:
        points = [(0.0, 0.0), (1.0, 0.2), (2.0, 0.0)]
        self.assertEqual(figures.without_straight_runs(points), points)

    def test_the_reach_takes_in_the_wider_configured_outline(self) -> None:
        own = figures.model_inputs(REMO, figures.DEFAULT_SETTINGS)
        wider = figures.model_inputs(REMO, figures.Settings(separation_percent=120))
        self.assertGreater(figures.reach(wider), figures.reach(own))

    def test_grid_lines_lie_at_least_the_least_spacing_apart(self) -> None:
        drive = figures.example_drive(figures.EXAMPLES[0])
        inputs = figures.model_inputs(REMO, figures.EXAMPLES[0].settings)
        scale = figures.drawing_scale(figures.DRAWING.static_width_px, drive.samples,
                                      figures.reach(inputs))
        step = figures.grid_step(scale)
        self.assertGreaterEqual(step * scale.pixels_per_metre, figures.DRAWING.grid_spacing_px)
        for smaller in (spacing for spacing in figures.DRAWING.grid_steps if spacing < step):
            self.assertLess(smaller * scale.pixels_per_metre, figures.DRAWING.grid_spacing_px)


def include_data(text: str) -> Dict[str, Any]:
    """Return the data that the include carries for motion-lab.js."""
    match = re.search(r'<script type="application/json" class="ix-data">(.*?)</script>', text)
    if match is None:
        raise ValueError('the include has no data')
    data: Dict[str, Any] = json.loads(match.group(1))
    return data


class TestInclude(unittest.TestCase):
    """The include in the repository is what the script writes, and MkDocs can insert it."""

    def setUp(self) -> None:
        self.include = figures.INCLUDE.read_text(encoding='utf-8')

    def test_include_is_the_scripts_output(self) -> None:
        self.assertEqual(self.include, figures.build(),
                         'run python3 scripts/figures/motion_lab_figures.py')

    def test_include_names_its_script_on_its_first_line(self) -> None:
        self.assertEqual(self.include.splitlines()[0],
                         '<!-- Written by scripts/figures/motion_lab_figures.py -->')

    def test_include_has_no_blank_line_or_template_marker(self) -> None:
        figures.check_include(self.include)

    def test_check_include_rejects_a_blank_line_and_a_template_marker(self) -> None:
        for broken in ('<p>one</p>\n\n<p>two</p>', '<p>{{ value }}</p>', '<p>{% if %}</p>'):
            with self.assertRaises(ValueError, msg=broken):
                figures.check_include(broken)

    def test_include_data_carries_the_drawing_sizes_and_the_check_rows(self) -> None:
        data = include_data(self.include)
        self.assertEqual(data['drawing'], json.loads(json.dumps({
            figures.camel_case(name): value for name, value in figures.DRAWING._asdict().items()
        })))
        for kind in figures.CommandKind:
            self.assertEqual([row['name'] for row in data['checks'][kind.value]],
                             [row.name for row in figures.check_rows(kind)])


if __name__ == '__main__':
    unittest.main()
