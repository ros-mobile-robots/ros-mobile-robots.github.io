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
from typing import List
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


def example_with_key(example_key: str) -> figures.Example:
    """Return the guided example with this key."""
    return next(example for example in figures.EXAMPLES if example.key == example_key)


def other_drive(example: figures.Example) -> figures.Drive:
    """Return a guided example's drive on the test's other command."""
    return figures.example_drive(example, figures.other_command(example.command))


def formatted_answer(example: figures.Example) -> str:
    """Return a guided example's answer for the drive it opens with, with its values filled in."""
    return example.primary_prompt.answer.format(**figures.example_text_values(example))


def turned_check() -> figures.Check:
    """Return the turn test's check of the angle turned, whatever its place in the test."""
    checks = figures.CHECKS[figures.CommandKind.TURN]
    return next(check for check in checks.each_view if check.quantity is figures.Quantity.TURNED)


def pixel_points(*pairs: Tuple[float, float]) -> List[figures.PixelPoint]:
    """Return the drawing's points of (x, y) pairs in px."""
    return [figures.PixelPoint(x, y) for x, y in pairs]


def simulate_remo_with_settings(command: figures.CommandKind, **settings: int) -> figures.Drive:
    """Return Remo's drive of a command with some settings changed."""
    return figures.simulate(REMO, figures.COMMANDS[command], figures.Settings(**settings))


class TestGuidedExamples(unittest.TestCase):
    """The examples' results equal values worked out by hand from the kinematics.

    The drives are the test's commands, FORWARD.distance and TURN.angle, with Remo's wheels.
    """

    def test_a_radius_too_big_shortens_only_the_true_drive(self) -> None:
        example = example_with_key('radius')
        drive = figures.example_drive(example)
        factor = example.settings.radius_percent / 100
        # The controller commands 1 / factor of the wheel turns, and the odometry multiplies them
        # by the factor again
        self.assertAlmostEqual(drive.odometry.forward, FORWARD.distance, places=PLACES)
        self.assertAlmostEqual(drive.model.forward, FORWARD.distance / factor, places=PLACES)
        self.assertEqual(
            figures.failing_checks(example.command, drive), ['odometry and true pose, forward']
        )

    def test_a_separation_too_small_turns_the_robot_less_and_every_check_passes(self) -> None:
        example = example_with_key('separation')
        drive = figures.example_drive(example)
        factor = example.settings.separation_percent / 100
        self.assertAlmostEqual(drive.odometry.turned, TURN.angle, places=PLACES)
        self.assertAlmostEqual(drive.model.turned, TURN.angle * factor, places=PLACES)
        self.assertEqual(figures.failing_checks(example.command, drive), [])

    def test_a_frame_behind_the_axle_ends_a_chord_away_and_fails_the_distance_check(self) -> None:
        example = example_with_key('offset')
        drive = figures.example_drive(example)
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
        example = example_with_key('contact')
        drive = figures.example_drive(example)
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
        example = example_with_key('tracking')
        drive = figures.example_drive(example)
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
            for kind in figures.CommandKind:
                values = figures.example_text_values(example, kind)
                prompt = figures.example_prompt(example, kind)
                for text in (prompt.question, prompt.answer):
                    filled = text.format(**values)
                    self.assertNotRegex(filled, r'[{}]', f'{example.key}, {kind.value}')


class TestAnswerTexts(unittest.TestCase):
    """Each example's answer states the numbers worked out by hand, in the right sentence."""

    def test_the_radius_answer_states_the_commanded_and_the_shortened_distance(self) -> None:
        example = example_with_key('radius')
        true_distance = FORWARD.distance / (example.settings.radius_percent / 100)
        answer = formatted_answer(example)
        self.assertIn(f'The odometry reports {FORWARD.distance:.3f} m.', answer)
        self.assertIn(f'The robot drives {true_distance:.3f} m.', answer)

    def test_the_separation_answer_states_both_angles_and_their_difference(self) -> None:
        example = example_with_key('separation')
        true_angle = TURN.angle * example.settings.separation_percent / 100
        answer = formatted_answer(example)
        self.assertIn(f'The odometry reports {TURN.angle:.3f} rad, and the robot turns'
                      f' {true_angle:.3f} rad.', answer)
        self.assertIn(f'The two differ by {TURN.angle - true_angle:.3f} rad', answer)

    def test_the_offset_answer_states_the_chord_in_centimetres(self) -> None:
        example = example_with_key('offset')
        chord = 2 * example.settings.offset_mm / 1000 * math.sin(TURN.angle / 2)
        self.assertIn(f'ends {chord * 100:.1f} cm from its start', formatted_answer(example))

    def test_the_contact_answer_states_the_smaller_turn_and_both_separations(self) -> None:
        example = example_with_key('contact')
        wheels = REMO.wheels
        contact_separation = wheels.separation + wheels.width
        share = wheels.separation / contact_separation
        answer = formatted_answer(example)
        self.assertIn(f'The robot turns {TURN.angle * share:.3f} rad,'
                      f' {(1 - share) * 100:.0f} % less', answer)
        self.assertIn(f'{contact_separation:.3f} m apart instead of {wheels.separation:.3f} m',
                      answer)

    def test_the_tracking_answer_states_the_distance_that_both_views_show(self) -> None:
        example = example_with_key('tracking')
        reached = FORWARD.distance * example.settings.tracking_percent / 100
        self.assertIn(f'Both show {reached:.3f} m', formatted_answer(example))


class TestOtherDrives(unittest.TestCase):
    """On the other drive, each example's results equal values worked out by hand too."""

    def test_a_radius_too_big_turns_the_robot_less_and_fails_the_comparison(self) -> None:
        example = example_with_key('radius')
        drive = other_drive(example)
        factor = example.settings.radius_percent / 100
        # Both wheels get 1 / factor of their turns: the robot turns 1 / factor of the angle
        self.assertAlmostEqual(drive.odometry.turned, TURN.angle, places=PLACES)
        self.assertAlmostEqual(drive.model.turned, TURN.angle / factor, places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.TURN, drive),
                         ['odometry and true pose, turned'])

    def test_a_separation_too_small_changes_nothing_on_a_straight_drive(self) -> None:
        drive = other_drive(example_with_key('separation'))
        self.assertAlmostEqual(drive.odometry.forward, FORWARD.distance, places=PLACES)
        self.assertAlmostEqual(drive.model.forward, FORWARD.distance, places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.FORWARD, drive), [])

    def test_a_frame_behind_the_axle_drives_as_far_as_the_axle(self) -> None:
        drive = other_drive(example_with_key('offset'))
        self.assertAlmostEqual(drive.model.forward, FORWARD.distance, places=PLACES)
        self.assertAlmostEqual(drive.model.left, 0.0, places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.FORWARD, drive), [])

    def test_contact_at_the_outer_edges_changes_nothing_on_a_straight_drive(self) -> None:
        drive = other_drive(example_with_key('contact'))
        self.assertAlmostEqual(drive.model.forward, FORWARD.distance, places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.FORWARD, drive), [])

    def test_wheels_at_part_of_their_speed_turn_exactly_to_the_tolerance_boundary(self) -> None:
        example = example_with_key('tracking')
        drive = other_drive(example)
        share = example.settings.tracking_percent / 100
        turned = turned_check()
        self.assertAlmostEqual(drive.odometry.turned, TURN.angle * share, places=PLACES)
        self.assertAlmostEqual(drive.model.turned, TURN.angle * share, places=PLACES)
        self.assertAlmostEqual(drive.model.turned, turned.expected - turned.tolerance,
                               places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.TURN, drive), [])


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
        turned = turned_check()
        boundary = turned.expected - turned.tolerance
        # The wheels reach the share of their speed that ends the turn exactly on the boundary
        tracking = round(boundary / TURN.angle * 100)
        drive = simulate_remo_with_settings(figures.CommandKind.TURN, tracking_percent=tracking)
        self.assertAlmostEqual(drive.model.turned, boundary, places=PLACES)
        self.assertEqual(figures.failing_checks(figures.CommandKind.TURN, drive), [])

    def test_the_turn_test_catches_a_separation_off_by_more_than_the_stated_share(self) -> None:
        share = int(figures.check_text_values(figures.CommandKind.TURN)['agreement_share'])
        agreement = 'odometry and true pose, turned'
        turn = figures.CommandKind.TURN
        inside = simulate_remo_with_settings(turn, separation_percent=100 - (share - 1))
        outside = simulate_remo_with_settings(turn, separation_percent=100 - share)
        self.assertNotIn(agreement, figures.failing_checks(turn, inside))
        self.assertIn(agreement, figures.failing_checks(turn, outside))


class TestModelLimits(unittest.TestCase):
    """The model's two numerical limits decide as their constants say."""

    # Values that floating-point numbers hold exactly, so the boundary is the boundary
    CHECK = figures.Check(figures.Quantity.FORWARD, expected=1.0, tolerance=0.25)
    BOUNDARY = CHECK.expected + CHECK.tolerance
    SPEED = 1.0
    DURATION = 1.0

    def test_a_value_on_the_tolerance_boundary_passes(self) -> None:
        self.assertTrue(figures.passes(self.CHECK, self.BOUNDARY))

    def test_a_value_beyond_the_boundary_by_less_than_the_allowance_passes(self) -> None:
        value = self.BOUNDARY + figures.ROUNDING_ALLOWANCE / 2
        self.assertTrue(figures.passes(self.CHECK, value))

    def test_a_value_beyond_the_boundary_by_more_than_the_allowance_fails(self) -> None:
        value = self.BOUNDARY + 2 * figures.ROUNDING_ALLOWANCE
        self.assertFalse(figures.passes(self.CHECK, value))

    def test_an_angular_velocity_below_the_straight_limit_drives_straight(self) -> None:
        velocity = figures.Velocity(self.SPEED, figures.STRAIGHT_ANGULAR_VELOCITY / 2)
        self.assertEqual(figures.pose_after(velocity, self.DURATION),
                         figures.Pose(self.SPEED * self.DURATION, 0.0, 0.0))

    def test_an_angular_velocity_above_the_straight_limit_turns(self) -> None:
        angular = 2 * figures.STRAIGHT_ANGULAR_VELOCITY
        pose = figures.pose_after(figures.Velocity(self.SPEED, angular), self.DURATION)
        self.assertEqual(pose.theta, angular * self.DURATION)


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
        points = pixel_points((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 1.0))
        self.assertEqual(
            figures.without_straight_runs(points), pixel_points((0.0, 0.0), (2.0, 0.0), (3.0, 1.0))
        )

    def test_a_point_off_the_line_by_more_than_a_tenth_of_a_pixel_stays(self) -> None:
        points = pixel_points((0.0, 0.0), (1.0, 0.2), (2.0, 0.0))
        self.assertEqual(figures.without_straight_runs(points), points)

    def test_a_path_that_turns_back_keeps_its_turning_point(self) -> None:
        points = pixel_points((0.0, 0.0), (2.0, 0.0), (1.0, 0.0))
        self.assertEqual(figures.without_straight_runs(points), points)

    def test_a_path_that_returns_to_its_start_keeps_its_far_point(self) -> None:
        points = pixel_points((0.0, 0.0), (1.0, 0.0), (0.0, 0.0))
        self.assertEqual(figures.without_straight_runs(points), points)

    def test_a_single_point_stays_a_single_point(self) -> None:
        point = pixel_points((0.0, 0.0))
        self.assertEqual(figures.without_straight_runs(point), point)

    def test_the_reach_takes_in_the_wider_configured_outline(self) -> None:
        own = figures.model_inputs(REMO, figures.DEFAULT_SETTINGS)
        wider = figures.model_inputs(REMO, figures.Settings(separation_percent=120))
        self.assertGreater(figures.outline_reach_metres(wider), figures.outline_reach_metres(own))

    def test_grid_lines_lie_at_least_the_least_spacing_apart(self) -> None:
        drive = figures.example_drive(figures.EXAMPLES[0])
        inputs = figures.model_inputs(REMO, figures.EXAMPLES[0].settings)
        scale = figures.drawing_scale(figures.DRAWING.static_width_px, drive.samples,
                                      figures.outline_reach_metres(inputs))
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
        self.assertEqual(self.include, figures.motion_lab_include_html(),
                         'run python3 scripts/figures/motion_lab_figures.py')

    def test_include_names_its_script_on_its_first_line(self) -> None:
        self.assertEqual(self.include.splitlines()[0],
                         '<!-- Written by scripts/figures/motion_lab_figures.py -->')

    def test_include_has_no_blank_line_or_template_marker(self) -> None:
        figures.check_include(self.include)

    def test_check_include_rejects_a_blank_line(self) -> None:
        with self.assertRaises(ValueError):
            figures.check_include('<p>one</p>\n\n<p>two</p>')

    def test_check_include_rejects_each_marker_that_starts_a_template_expression(self) -> None:
        # Written out here, not taken from the script's list: a marker missing there fails
        for broken in ('<p>{{ value }}</p>', '<p>{% if %}</p>', '<p>{# comment #}</p>'):
            with self.subTest(broken=broken), self.assertRaises(ValueError):
                figures.check_include(broken)

    def test_check_include_accepts_single_braces_and_single_line_breaks(self) -> None:
        figures.check_include('<p>{one}</p>\n<p>100 %</p>\n')

    def test_include_data_carries_the_models_two_limits(self) -> None:
        self.assertEqual(include_data(self.include)['modelLimits'], {
            'straightAngularVelocity': figures.STRAIGHT_ANGULAR_VELOCITY,
            'roundingAllowance': figures.ROUNDING_ALLOWANCE,
        })

    def test_include_data_carries_the_drawing_sizes(self) -> None:
        data = include_data(self.include)
        self.assertEqual(data['drawing'], json.loads(json.dumps({
            figures.camel_case(name): value for name, value in figures.DRAWING._asdict().items()
        })))

    def test_include_data_carries_each_check_row_whole_in_the_tests_order(self) -> None:
        data = include_data(self.include)
        for kind in figures.CommandKind:
            expected = [
                {
                    'view': row.view.value,
                    'quantity': row.check.quantity.value,
                    'expected': row.check.expected,
                    'tolerance': row.check.tolerance,
                    'unit': figures.UNITS[row.check.quantity],
                    'name': row.name,
                    'nameInSentence': row.name_in_sentence,
                    'allowed': row.allowed,
                }
                for row in figures.check_rows(kind)
            ]
            self.assertEqual(data['checks'][kind.value], expected, kind.value)

    def test_the_forward_tests_first_check_row_has_the_values_of_test_drive_py(self) -> None:
        # Written out from test_drive.py: assertAlmostEqual(odometry.forward, 0.6, delta=0.1).
        # The expected distance is the product of 0.2 m/s and 3 s, which rounding takes off 0.6
        first = include_data(self.include)['checks'][figures.CommandKind.FORWARD.value][0]
        self.assertAlmostEqual(first.pop('expected'), 0.6, places=PLACES)
        self.assertEqual(first, {
            'view': 'odometry', 'quantity': 'forward', 'tolerance': 0.1, 'unit': 'm',
            'name': 'forward, odometry', 'nameInSentence': 'forward (odometry)',
            'allowed': '0.6 ± 0.1 m',
        })

    def test_include_carries_no_expected_results_for_the_tests(self) -> None:
        self.assertNotIn('data-expected', self.include)


if __name__ == '__main__':
    unittest.main()
