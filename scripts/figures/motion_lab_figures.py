"""Write includes/motion-lab.html, the interactive figure "Three views of one drive".

The figure is on docs/development/simulation-test.md. Its include holds everything the figure
needs:

- the data for docs/custom_theme/js/interactive/motion-lab.js: the robots' wheel geometry, the
  test's two commands, the rows of its checks with their texts, the guided examples and the
  drawing's sizes;
- a static drawing of the first example and the answers of all examples, computed here. They are
  what the page shows without JavaScript, and until the script has drawn the figure;
- the controls, which the script shows once the figure works.

The values come from the diffbot repository at SOURCE_COMMIT: wheel_radius and wheel_separation
from diffbot_control/config/<robot>/geometry.yaml, the wheel width from the robots' descriptions
(front_wheel.yaml, length), the commands and tolerances from diffbot_gazebo/test/test_drive.py.
Update them here when they change there, and run from the repository root:

    python3 scripts/figures/motion_lab_figures.py

tests/test_motion_lab_figures.py checks the computed answers against values worked out by hand,
and that the include is the output of this script.
"""

from enum import Enum
import html
import json
import math
from pathlib import Path
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

from board_drawings import svg_element

ROOT = Path(__file__).resolve().parent.parent.parent
INCLUDE = ROOT / 'includes' / 'motion-lab.html'

SOURCE_COMMIT = 'f5f282eed7b6a2b146d40c7d5fd641b8a339a77d'
SOURCE_URL = f'https://github.com/ros-mobile-robots/diffbot/blob/{SOURCE_COMMIT}'

# Below this angular velocity (rad/s), a drive counts as straight: the radius of its circle,
# linear / angular velocity, would overflow. diffdrive.js has the same limit
STRAIGHT_ANGULAR_VELOCITY = 1e-12
# A value on a tolerance's boundary can miss it by floating-point rounding: 1.5 - 1.2 gives
# 0.30000000000000004. This small allowance keeps such a value inside its range, as in
# diffdrive.js. It is a choice of the figure only. The ROS test compares without it
ROUNDING_ALLOWANCE = 1e-9
# MkDocs inserts the include into a Markdown page: a blank line would end its HTML block, and
# these character pairs would start an expression of the macros plugin
TEMPLATE_MARKERS = ('{{', '{#', '{%')
# The time slider's steps over one drive
TIME_SLIDER_STEPS = 1000


# --- The robots, the test's commands and its checks ------------------------------------------

class WheelGeometry(NamedTuple):
    """A robot's wheels: their radius, the distance of their center planes, their width (m)."""

    radius: float
    separation: float
    width: float


class Robot(NamedTuple):
    """A robot's name and its wheels."""

    name: str
    wheels: WheelGeometry


class CommandKind(Enum):
    """The two drives of the simulation test."""

    FORWARD = 'forward'
    TURN = 'turn'


class Command(NamedTuple):
    """A velocity command of the test, held for its duration.

    linear_velocity is in m/s, angular_velocity in rad/s and duration in s.
    """

    linear_velocity: float
    angular_velocity: float
    duration: float

    @property
    def distance(self) -> float:
        """Return how far (m) the command alone drives."""
        return self.linear_velocity * self.duration

    @property
    def angle(self) -> float:
        """Return how far (rad) the command alone turns."""
        return self.angular_velocity * self.duration


class Quantity(Enum):
    """What a check of the test measures in a motion from the start pose."""

    FORWARD = 'forward'
    LEFT = 'left'
    TURNED = 'turned'
    DISTANCE = 'distance'  # From the start, in any direction


class View(Enum):
    """Whose motion a check measures: the odometry's, the true pose's, or their difference."""

    ODOMETRY = 'odometry'
    MODEL = 'model'
    AGREEMENT = 'agreement'


class Check(NamedTuple):
    """One assertAlmostEqual of test_drive.py: the quantity, its expected value and its delta."""

    quantity: Quantity
    expected: float
    tolerance: float


class DriveChecks(NamedTuple):
    """A test's checks: those on each view's motion, and the one on how far the views are apart.

    The agreement check compares the odometry's value of its quantity with the true pose's.
    """

    each_view: Tuple[Check, ...]
    agreement: Check


ROBOTS = {
    'remo': Robot('Remo', WheelGeometry(radius=0.0330, separation=0.144, width=0.025)),
    'diffbot': Robot('DiffBot', WheelGeometry(radius=0.0333, separation=0.138, width=0.025)),
}
EXAMPLE_ROBOT = 'remo'
# test_drive.py: FORWARD_VELOCITY (m/s), TURN_VELOCITY (rad/s) and DURATION (s)
COMMANDS = {
    CommandKind.FORWARD: Command(linear_velocity=0.2, angular_velocity=0.0, duration=3.0),
    CommandKind.TURN: Command(linear_velocity=0.0, angular_velocity=0.5, duration=3.0),
}
# test_drive.py: the expected value and the delta of each assertAlmostEqual, in its order
CHECKS = {
    CommandKind.FORWARD: DriveChecks(
        each_view=(
            Check(
                Quantity.FORWARD, expected=COMMANDS[CommandKind.FORWARD].distance, tolerance=0.1
            ),
            Check(Quantity.LEFT, expected=0.0, tolerance=0.1),
            Check(Quantity.TURNED, expected=0.0, tolerance=0.2),
        ),
        agreement=Check(Quantity.FORWARD, expected=0.0, tolerance=0.03),
    ),
    CommandKind.TURN: DriveChecks(
        each_view=(
            Check(Quantity.TURNED, expected=COMMANDS[CommandKind.TURN].angle, tolerance=0.3),
            Check(Quantity.DISTANCE, expected=0.0, tolerance=0.02),
        ),
        agreement=Check(Quantity.TURNED, expected=0.0, tolerance=0.1),
    ),
}
UNITS = {Quantity.FORWARD: 'm', Quantity.LEFT: 'm', Quantity.TURNED: 'rad', Quantity.DISTANCE: 'm'}
QUANTITY_NAMES = {
    Quantity.FORWARD: 'forward',
    Quantity.LEFT: 'left',
    Quantity.TURNED: 'turned',
    Quantity.DISTANCE: 'distance from the start',
}
VIEW_NAMES = {View.ODOMETRY: 'odometry', View.MODEL: 'true pose'}


def number(value: float) -> str:
    """Return a value as the figure's texts write it: 0.6, 1.5, 0.02."""
    return f'{value:g}'


def command_text(kind: CommandKind) -> str:
    """Return what a command tells the robot, as a sentence says it: "drive 0.6 m"."""
    command = COMMANDS[kind]
    if kind is CommandKind.FORWARD:
        return f'drive {number(command.distance)} m'
    return f'turn {number(command.angle)} rad on the spot'


def command_button_text(kind: CommandKind) -> str:
    """Return a command's button text: "Drive 0.6 m"."""
    command = COMMANDS[kind]
    if kind is CommandKind.FORWARD:
        return f'Drive {number(command.distance)} m'
    return f'Turn {number(command.angle)} rad'


class CheckRow(NamedTuple):
    """A row of the figure's table of checks: the check, the view it measures, and its texts.

    name is the row's name in the table, name_in_sentence its name in the summary's list of
    failing checks, and allowed the values that pass, as the table shows them.
    """

    check: Check
    view: View
    name: str
    name_in_sentence: str
    allowed: str


def view_check_row(check: Check, view: View) -> CheckRow:
    """Return the row of a check on one view's motion."""
    quantity, unit = QUANTITY_NAMES[check.quantity], UNITS[check.quantity]
    return CheckRow(
        check,
        view,
        name=f'{quantity}, {VIEW_NAMES[view]}',
        name_in_sentence=f'{quantity} ({VIEW_NAMES[view]})',
        allowed=f'{number(check.expected)} ± {number(check.tolerance)} {unit}',
    )


def agreement_check_row(check: Check) -> CheckRow:
    """Return the row of the check that compares the odometry with the true pose."""
    odometry, model = VIEW_NAMES[View.ODOMETRY], VIEW_NAMES[View.MODEL]
    return CheckRow(
        check,
        View.AGREEMENT,
        name=f'{odometry} and {model}, {QUANTITY_NAMES[check.quantity]}',
        name_in_sentence=f'the comparison of the {odometry} with the {model}',
        allowed=f'at most {number(check.tolerance)} {UNITS[check.quantity]} apart',
    )


def check_rows(kind: CommandKind) -> List[CheckRow]:
    """Return a test's checks in the test's order: each view's checks, then the agreement."""
    checks = CHECKS[kind]
    rows = [
        view_check_row(check, view)
        for view in (View.ODOMETRY, View.MODEL)
        for check in checks.each_view
    ]
    return rows + [agreement_check_row(checks.agreement)]


# --- The kinematic model ---------------------------------------------------------------------

class Settings(NamedTuple):
    """The values that the figure's sliders set, with their units in their names.

    The configured wheel radius and separation are percent of the robot's own values. offset_mm
    is how far base_footprint lies behind the axle's midpoint. contact_percent is where the wheels
    push on the ground: 0 in the middle of each wheel, 100 at the wheels' outer edges.
    tracking_percent is the share of its commanded speed that each wheel reaches.
    """

    radius_percent: int = 100
    separation_percent: int = 100
    offset_mm: int = 0
    contact_percent: int = 0
    tracking_percent: int = 100


DEFAULT_SETTINGS = Settings()


class ModelInputs(NamedTuple):
    """The kinematic model's inputs in SI units, from a robot and the figure's settings.

    configured is the geometry that the controller is configured with. contact_separation is the
    distance (m) at which the wheels push on the ground, frame_offset how far (m) base_footprint
    lies behind the axle's midpoint, and tracking the share of its commanded speed that each wheel
    reaches.
    """

    wheels: WheelGeometry
    configured: WheelGeometry
    contact_separation: float
    frame_offset: float
    tracking: float


class Pose(NamedTuple):
    """A pose on the ground: x and y (m) and the heading theta (rad)."""

    x: float
    y: float
    theta: float


class Velocity(NamedTuple):
    """A robot's linear (m/s) and angular (rad/s) velocity."""

    linear: float
    angular: float


class WheelSpeeds(NamedTuple):
    """The angular velocities (rad/s) of the left and the right wheel."""

    left: float
    right: float


class ViewVelocities(NamedTuple):
    """The velocities of a drive's three views: the command, the odometry and the model."""

    command: Velocity
    odometry: Velocity
    model: Velocity


class Poses(NamedTuple):
    """A drive's poses at one time.

    command is where the command alone puts the robot, odometry where the odometry says it is.
    axle is the model's axle midpoint, and frame the model's base_footprint, which the test
    reports as the true pose.
    """

    command: Pose
    odometry: Pose
    axle: Pose
    frame: Pose


class Motion(NamedTuple):
    """An end pose relative to a start pose, as test_drive.py's RelativePose: m, m and rad."""

    forward: float
    left: float
    turned: float


class Drive(NamedTuple):
    """One drive: the motions that the test measures, and the poses for the drawing's paths."""

    command: Motion
    odometry: Motion
    model: Motion
    samples: Tuple[Poses, ...]


def model_inputs(robot: Robot, settings: Settings) -> ModelInputs:
    """Return the model's inputs for a robot with the figure's settings."""
    wheels = robot.wheels
    return ModelInputs(
        wheels=wheels,
        configured=wheels._replace(
            radius=wheels.radius * settings.radius_percent / 100,
            separation=wheels.separation * settings.separation_percent / 100,
        ),
        contact_separation=wheels.separation + wheels.width * settings.contact_percent / 100,
        frame_offset=settings.offset_mm / 1000,
        tracking=settings.tracking_percent / 100,
    )


def wheel_speeds_for(velocity: Velocity, radius: float, separation: float) -> WheelSpeeds:
    """Return the wheel speeds that diff_drive_controller commands with its geometry."""
    return WheelSpeeds(
        left=(velocity.linear - velocity.angular * separation / 2) / radius,
        right=(velocity.linear + velocity.angular * separation / 2) / radius,
    )


def velocity_from(speeds: WheelSpeeds, radius: float, separation: float) -> Velocity:
    """Return the velocity that two wheel speeds give with a wheel radius and separation."""
    return Velocity(
        linear=radius * (speeds.right + speeds.left) / 2,
        angular=radius * (speeds.right - speeds.left) / separation,
    )


def view_velocities(inputs: ModelInputs, command: Command) -> ViewVelocities:
    """Return the velocities of a drive's command, odometry and model.

    The controller turns the command into wheel speeds with its configured radius and separation.
    The wheels reach `tracking` of them. The odometry converts the wheel turns back with the
    configured values. The model converts them with the robot's own radius and the separation at
    which the wheels push on the ground.
    """
    commanded = Velocity(command.linear_velocity, command.angular_velocity)
    configured = inputs.configured
    commanded_speeds = wheel_speeds_for(commanded, configured.radius, configured.separation)
    reached = WheelSpeeds(
        inputs.tracking * commanded_speeds.left, inputs.tracking * commanded_speeds.right
    )
    return ViewVelocities(
        command=commanded,
        odometry=velocity_from(reached, configured.radius, configured.separation),
        model=velocity_from(reached, inputs.wheels.radius, inputs.contact_separation),
    )


def pose_after(velocity: Velocity, duration: float) -> Pose:
    """Return the pose after a constant velocity for the duration, from 0, 0 with heading 0.

    A robot with an angular velocity drives on a circle of radius linear / angular velocity. The
    heading isn't wrapped, so a path can be drawn through more than half a turn.
    """
    if abs(velocity.angular) < STRAIGHT_ANGULAR_VELOCITY:
        return Pose(velocity.linear * duration, 0.0, 0.0)
    theta = velocity.angular * duration
    circle_radius = velocity.linear / velocity.angular
    return Pose(circle_radius * math.sin(theta), circle_radius * (1 - math.cos(theta)), theta)


def poses_at(velocities: ViewVelocities, frame_offset: float, time: float) -> Poses:
    """Return a drive's poses at a time (s).

    base_footprint starts at 0, 0 and lies frame_offset behind the axle's midpoint, so the axle's
    midpoint starts frame_offset ahead of it.
    """
    axle_from_start = pose_after(velocities.model, time)
    axle = axle_from_start._replace(x=axle_from_start.x + frame_offset)
    frame = Pose(
        axle.x - frame_offset * math.cos(axle.theta),
        axle.y - frame_offset * math.sin(axle.theta),
        axle.theta,
    )
    return Poses(
        command=pose_after(velocities.command, time),
        odometry=pose_after(velocities.odometry, time),
        axle=axle,
        frame=frame,
    )


def wrap(angle: float) -> float:
    """Return the same angle in -pi..pi, as the function of this name in test_drive.py."""
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_motion(start: Pose, end: Pose) -> Motion:
    """Return the end pose relative to the start pose, in the start's frame, as test_drive.py."""
    dx, dy = end.x - start.x, end.y - start.y
    cos, sin = math.cos(start.theta), math.sin(start.theta)
    return Motion(cos * dx + sin * dy, -sin * dx + cos * dy, wrap(end.theta - start.theta))


def simulate(robot: Robot, command: Command, settings: Settings) -> Drive:
    """Compute one drive's three views with the figure's kinematic model."""
    inputs = model_inputs(robot, settings)
    velocities = view_velocities(inputs, command)
    samples = tuple(
        poses_at(velocities, inputs.frame_offset, command.duration * index / DRAWING.samples)
        for index in range(DRAWING.samples + 1)
    )
    start, end = samples[0], samples[-1]
    return Drive(
        command=relative_motion(start.command, end.command),
        odometry=relative_motion(start.odometry, end.odometry),
        model=relative_motion(start.frame, end.frame),
        samples=samples,
    )


def measured(quantity: Quantity, motion: Motion) -> float:
    """Return the value of a quantity in a motion."""
    values = {
        Quantity.FORWARD: motion.forward,
        Quantity.LEFT: motion.left,
        Quantity.TURNED: motion.turned,
        Quantity.DISTANCE: math.hypot(motion.forward, motion.left),
    }
    return values[quantity]


def row_value(row: CheckRow, drive: Drive) -> float:
    """Return the value that a row's check measures in a drive."""
    quantity = row.check.quantity
    if row.view is View.AGREEMENT:
        return abs(measured(quantity, drive.odometry) - measured(quantity, drive.model))
    return measured(quantity, drive.odometry if row.view is View.ODOMETRY else drive.model)


def passes(check: Check, value: float) -> bool:
    """Return whether a value lies within the check's tolerance of its expected value."""
    return abs(value - check.expected) <= check.tolerance + ROUNDING_ALLOWANCE


def failing_checks(kind: CommandKind, drive: Drive) -> List[str]:
    """Return the names of the checks that a drive fails, in the test's order."""
    return [row.name for row in check_rows(kind) if not passes(row.check, row_value(row, drive))]


# --- The guided examples ---------------------------------------------------------------------

class Example(NamedTuple):
    """A guided example: a drive, its settings, its question and its answer.

    question and answer are HTML. Their {placeholders} are the names of example_values().
    """

    key: str
    command: CommandKind
    settings: Settings
    question: str
    answer: str


# Each example changes one setting of the example robot
EXAMPLES = (
    Example(
        key='radius',
        command=CommandKind.FORWARD,
        settings=Settings(radius_percent=110),
        question=(
            '<b>The configured wheel radius is {radius_error} % too big.</b> The robot is told to'
            ' {command}. How far does the odometry say the robot drove, and how far did the robot'
            ' drive?'
        ),
        answer=(
            'The odometry reports {odometry_forward} m. The robot drives {model_forward} m. The'
            ' controller commands fewer wheel turns, and multiplies them by the same too-big'
            ' radius. Only the check that compares the odometry with the true pose fails.'
        ),
    ),
    Example(
        key='separation',
        command=CommandKind.TURN,
        settings=Settings(separation_percent=95),
        question=(
            '<b>The configured wheel separation is {separation_error} % too small.</b> The robot'
            ' is told to {command}. Does a check fail?'
        ),
        answer=(
            'No. The odometry reports {odometry_turned} rad, and the robot turns {model_turned}'
            ' rad. The two differ by {gap_turned} rad, and the check allows {agreement_tolerance}'
            ' rad. The test only catches a separation that is off by more than about'
            ' {agreement_share} %.'
        ),
    ),
    Example(
        key='offset',
        command=CommandKind.TURN,
        settings=Settings(offset_mm=20),
        question=(
            '<b>What if <code>base_footprint</code> were {offset_cm} cm behind the axle?</b> The'
            ' robot is told to {command}. Does <code>base_footprint</code> stay where it was?'
        ),
        answer=(
            'No. The robot turns about the midpoint of its axle. A frame {offset_cm} cm behind'
            ' that midpoint follows an arc and ends {model_distance_cm} cm from its start. The'
            ' check that allows {distance_tolerance} m fails. The odometry reports no movement. On'
            " DiffBot and Remo, <code>base_footprint</code> is at the axle's midpoint and stays in"
            ' place.'
        ),
    ),
    Example(
        key='contact',
        command=CommandKind.TURN,
        settings=Settings(contact_percent=100),
        question=(
            '<b>What if the wheels pushed on the ground at their outer edges?</b> The robot is'
            ' told to {command}. How far does the robot turn?'
        ),
        answer=(
            'The robot turns {model_turned} rad, {less_percent} % less, because its wheels push'
            ' {contact_separation} m apart instead of {separation} m. The odometry still reports'
            ' {odometry_turned} rad, so the check that compares the two fails. In the simulation,'
            " the wheels' collision shapes are spheres, which touch the ground in the middle of"
            ' the wheel.'
        ),
    ),
    Example(
        key='tracking',
        command=CommandKind.FORWARD,
        settings=Settings(tracking_percent=80),
        question=(
            '<b>The wheels reach only {tracking} % of their commanded speed.</b> The robot is told'
            ' to {command}. Do the odometry and the true pose still agree?'
        ),
        answer=(
            'Yes. Both show {model_forward} m, because the odometry counts the wheel turns that'
            ' really happened. The two checks against the expected {expected_forward} m fail, and'
            ' the check that compares the odometry with the true pose passes.'
        ),
    ),
)
NUMBER_WORDS = {3: 'Three', 4: 'Four', 5: 'Five', 6: 'Six', 7: 'Seven'}


def changed_settings(settings: Settings) -> Dict[str, int]:
    """Return the settings that differ from the robot's own values, by name."""
    return {
        name: value
        for name, value in settings._asdict().items()
        if value != getattr(DEFAULT_SETTINGS, name)
    }


def example_drive(example: Example) -> Drive:
    """Return the drive of a guided example."""
    return simulate(ROBOTS[EXAMPLE_ROBOT], COMMANDS[example.command], example.settings)


def check_values(kind: CommandKind) -> Dict[str, str]:
    """Return the texts of a test's expected values and tolerances, for an example's texts.

    expected_turned is the expected angle, turned_tolerance its delta, agreement_tolerance the
    delta of the check that compares the odometry with the true pose. agreement_share is that
    delta as a share (percent) of the commanded distance or angle: a wheel separation or radius
    that is off by more than this share makes the agreement check fail.
    """
    checks, command = CHECKS[kind], COMMANDS[kind]
    values = {}
    for check in checks.each_view:
        values[f'expected_{check.quantity.value}'] = number(check.expected)
        values[f'{check.quantity.value}_tolerance'] = number(check.tolerance)
    commanded = command.distance if kind is CommandKind.FORWARD else command.angle
    values['agreement_tolerance'] = number(checks.agreement.tolerance)
    values['agreement_share'] = f'{checks.agreement.tolerance / commanded * 100:.0f}'
    return values


def example_values(example: Example) -> Dict[str, str]:
    """Return the texts that fill an example's question and answer."""
    robot = ROBOTS[EXAMPLE_ROBOT]
    drive = example_drive(example)
    settings = example.settings
    inputs = model_inputs(robot, settings)
    # How much less the robot turns than its odometry reports
    less = 1 - drive.model.turned / drive.odometry.turned if drive.odometry.turned else 0.0
    return {
        **check_values(example.command),
        'command': command_text(example.command),
        'radius_error': f'{settings.radius_percent - 100}',
        'separation_error': f'{100 - settings.separation_percent}',
        'offset_cm': number(settings.offset_mm / 10),
        'tracking': f'{settings.tracking_percent}',
        'odometry_forward': f'{drive.odometry.forward:.3f}',
        'model_forward': f'{drive.model.forward:.3f}',
        'odometry_turned': f'{drive.odometry.turned:.3f}',
        'model_turned': f'{drive.model.turned:.3f}',
        'gap_turned': f'{abs(drive.odometry.turned - drive.model.turned):.3f}',
        'model_distance_cm': f'{measured(Quantity.DISTANCE, drive.model) * 100:.1f}',
        'less_percent': f'{less * 100:.0f}',
        'contact_separation': f'{inputs.contact_separation:.3f}',
        'separation': f'{robot.wheels.separation:.3f}',
    }


# --- The drawing -----------------------------------------------------------------------------

class DrawingSizes(NamedTuple):
    """The drawing's sizes, which motion-lab.js uses too.

    Lengths on the ground are in m, lengths on the screen in px of the drawing. aspect is the
    drawing's height as a share of its width. The static drawing is static_width_px wide, the
    live one as wide as the figure, at least minimum_width_px. margin lies around the robots,
    padding_px around the drawn area. nose is the length of the arrow from the axle's midpoint to
    the robot's front, with its head. command_heading is the length of the line from the
    command's cross in the command's heading. The grid's spacing is the smallest of grid_steps
    that puts its lines at least grid_spacing_px apart. samples is the number of poses per drive
    for the paths.
    """

    aspect: float
    static_width_px: int
    minimum_width_px: int
    padding_px: float
    margin: float
    nose: float
    head_length: float
    head_half_width: float
    command_heading: float
    cross_half_px: float
    dot_radius_px: float
    ring_radius_px: float
    axle_dot_radius_px: float
    grid_steps: Tuple[float, ...]
    grid_spacing_px: float
    samples: int


DRAWING = DrawingSizes(
    aspect=0.5,
    static_width_px=600,
    minimum_width_px=240,
    padding_px=14,
    margin=0.02,
    nose=0.05,
    head_length=0.012,
    head_half_width=0.008,
    command_heading=0.085,
    cross_half_px=5,
    dot_radius_px=5,
    ring_radius_px=5.5,
    axle_dot_radius_px=4,
    grid_steps=(0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0),
    grid_spacing_px=22,
    samples=150,
)
DRAWING_TITLE = 'One drive from above: the command, the odometry and the true pose'
# A point of the drawing in px, x to the right and y down
Point = Tuple[float, float]


class DrawingScale(NamedTuple):
    """How a drawing maps the ground (m) onto its pixels.

    The ground's x axis, the robot's forward at the start, points to the right of the drawing,
    and its y axis, the robot's left, points up.
    """

    width_px: float
    height_px: float
    center_x: float
    center_y: float
    pixels_per_metre: float

    def to_pixels(self, x: float, y: float) -> Point:
        """Return the drawing's point of a point on the ground."""
        return (
            self.width_px / 2 + (x - self.center_x) * self.pixels_per_metre,
            self.height_px / 2 - (y - self.center_y) * self.pixels_per_metre,
        )


class StaticDrawing(NamedTuple):
    """The static drawing as SVG, and its grid's spacing as the note under it says it."""

    svg: str
    grid_note: str


def reach(inputs: ModelInputs) -> float:
    """Return how far (m) the drawing reaches beyond the paths' poses.

    It takes in the wheels' outer edges of both outlines, the robot's arrow, the command's line
    and base_footprint behind the axle, with the margin.
    """
    outer_edge = max(
        wheels.separation / 2 + wheels.width / 2 for wheels in (inputs.wheels, inputs.configured)
    )
    front = max(
        inputs.wheels.radius,
        DRAWING.nose + DRAWING.head_length,
        DRAWING.command_heading,
        inputs.frame_offset,
    )
    return max(outer_edge, front) + DRAWING.margin


def drawing_scale(width_px: float, samples: Sequence[Poses], reach_m: float) -> DrawingScale:
    """Return the scale that fits every pose of the samples into a drawing, with the reach."""
    poses = [pose for sample in samples for pose in sample]
    left = min(pose.x for pose in poses) - reach_m
    right = max(pose.x for pose in poses) + reach_m
    bottom = min(pose.y for pose in poses) - reach_m
    top = max(pose.y for pose in poses) + reach_m
    height_px = round(width_px * DRAWING.aspect)
    pixels_per_metre = min(
        (width_px - 2 * DRAWING.padding_px) / (right - left),
        (height_px - 2 * DRAWING.padding_px) / (top - bottom),
    )
    return DrawingScale(width_px, height_px, (left + right) / 2, (bottom + top) / 2,
                        pixels_per_metre)


def grid_step(scale: DrawingScale) -> float:
    """Return the grid's spacing (m): the smallest step whose lines lie far enough apart."""
    return next(
        (
            step
            for step in DRAWING.grid_steps
            if step * scale.pixels_per_metre >= DRAWING.grid_spacing_px
        ),
        DRAWING.grid_steps[-1],
    )


def grid_lines(scale: DrawingScale, step: float) -> str:
    """Return the grid as SVG: lines at the multiples of the step across the whole drawing."""
    half_width = scale.width_px / 2 / scale.pixels_per_metre
    half_height = scale.height_px / 2 / scale.pixels_per_metre
    lines = []
    index = math.ceil((scale.center_x - half_width) / step)
    while index * step < scale.center_x + half_width:
        x_px = scale.to_pixels(index * step, 0)[0]
        lines.append(f'<line x1="{x_px:.1f}" y1="0" x2="{x_px:.1f}" y2="{scale.height_px}"/>')
        index += 1
    index = math.ceil((scale.center_y - half_height) / step)
    while index * step < scale.center_y + half_height:
        y_px = scale.to_pixels(0, index * step)[1]
        lines.append(f'<line x1="0" y1="{y_px:.1f}" x2="{scale.width_px}" y2="{y_px:.1f}"/>')
        index += 1
    return f'<g class="ix-grid">{"".join(lines)}</g>'


def points_attribute(points: Sequence[Point]) -> str:
    """Return points as an SVG points attribute, to a tenth of a pixel."""
    return ' '.join(f'{x:.1f},{y:.1f}' for x, y in points)


def without_straight_runs(points: Sequence[Point]) -> List[Point]:
    """Return the points without each one that lies on the line between the kept one and the next.

    A straight path keeps its two ends. A tenth of a pixel counts as on the line.
    """
    kept = list(points[:1])
    for point, following in zip(points[1:-1], points[2:]):
        (x0, y0), (x1, y1), (x2, y2) = kept[-1], point, following
        # Twice the area of the triangle, divided by its base: the distance from the line
        cross = abs((x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0))
        if cross > 0.1 * math.hypot(x2 - x0, y2 - y0):
            kept.append(point)
    return kept + list(points[-1:])


def path_line(scale: DrawingScale, poses: Sequence[Pose], css_class: str) -> str:
    """Return a view's path through its poses as an SVG polyline."""
    points = without_straight_runs([scale.to_pixels(pose.x, pose.y) for pose in poses])
    return f'<polyline class="ix-path {css_class}" points="{points_attribute(points)}"/>'


def line_element(start: Point, end: Point) -> str:
    """Return an SVG line between two points of the drawing."""
    return (
        f'<line x1="{start[0]:.1f}" y1="{start[1]:.1f}" x2="{end[0]:.1f}" y2="{end[1]:.1f}"/>'
    )


def on_robot(pose: Pose, forward: float, left: float) -> Tuple[float, float]:
    """Return the ground position (m) of a point given in the frame of a robot at a pose."""
    cos, sin = math.cos(pose.theta), math.sin(pose.theta)
    return pose.x + forward * cos - left * sin, pose.y + forward * sin + left * cos


def robot_outline(
    scale: DrawingScale, pose: Pose, wheels: WheelGeometry, css_class: str, markers: str = ''
) -> str:
    """Return a robot from above as SVG: two wheels, the axle and an arrow to its front.

    The pose is the axle's midpoint. markers are further SVG elements of the robot's group.
    """
    def at(forward: float, left: float) -> Point:
        return scale.to_pixels(*on_robot(pose, forward, left))

    parts = [f'<g class="ix-robot {css_class}">']
    for side in (1, -1):
        middle = side * wheels.separation / 2
        corners = [
            at(wheels.radius, middle + wheels.width / 2),
            at(wheels.radius, middle - wheels.width / 2),
            at(-wheels.radius, middle - wheels.width / 2),
            at(-wheels.radius, middle + wheels.width / 2),
        ]
        parts.append(f'<polygon class="ix-wheel" points="{points_attribute(corners)}"/>')
    parts.append(line_element(at(0, wheels.separation / 2), at(0, -wheels.separation / 2)))
    parts.append(line_element(at(0, 0), at(DRAWING.nose, 0)))
    head = [
        at(DRAWING.nose + DRAWING.head_length, 0),
        at(DRAWING.nose, DRAWING.head_half_width),
        at(DRAWING.nose, -DRAWING.head_half_width),
    ]
    parts.append(f'<polygon class="ix-head" points="{points_attribute(head)}"/>')
    parts.append(f'{markers}</g>')
    return ''.join(parts)


def true_pose_markers(scale: DrawingScale, axle: Pose, frame_offset: float) -> str:
    """Return the model's markers: a dot at base_footprint, and with an offset the axle's midpoint.

    With an offset, a thin line joins base_footprint to the axle's midpoint.
    """
    frame_x, frame_y = scale.to_pixels(*on_robot(axle, -frame_offset, 0))
    markers = []
    if frame_offset > 0:
        axle_x, axle_y = scale.to_pixels(axle.x, axle.y)
        markers.append(
            f'<line class="ix-link" x1="{axle_x:.1f}" y1="{axle_y:.1f}" x2="{frame_x:.1f}"'
            f' y2="{frame_y:.1f}"/><circle class="ix-dot ix-axle" cx="{axle_x:.1f}"'
            f' cy="{axle_y:.1f}" r="{DRAWING.axle_dot_radius_px:g}"/>'
        )
    markers.append(
        f'<circle class="ix-dot" cx="{frame_x:.1f}" cy="{frame_y:.1f}"'
        f' r="{DRAWING.dot_radius_px:g}"/>'
    )
    return ''.join(markers)


def odometry_marker(scale: DrawingScale, pose: Pose) -> str:
    """Return the odometry's marker: a ring at its base_footprint."""
    x_px, y_px = scale.to_pixels(pose.x, pose.y)
    return (
        f'<circle class="ix-ring" cx="{x_px:.1f}" cy="{y_px:.1f}" r="{DRAWING.ring_radius_px:g}"/>'
    )


def command_marker(scale: DrawingScale, pose: Pose) -> str:
    """Return the command's end as SVG: a cross where the command alone puts the robot.

    A line from the cross shows the command's heading.
    """
    cross_x, cross_y = scale.to_pixels(pose.x, pose.y)
    heading_x, heading_y = scale.to_pixels(*on_robot(pose, DRAWING.command_heading, 0))
    half = DRAWING.cross_half_px
    return (
        f'<g class="ix-robot ix-command"><line x1="{cross_x:.1f}" y1="{cross_y:.1f}"'
        f' x2="{heading_x:.1f}" y2="{heading_y:.1f}"/><path class="ix-cross"'
        f' d="M{cross_x - half:.1f},{cross_y - half:.1f} l{2 * half:g},{2 * half:g}'
        f' m0,{-2 * half:g} l{-2 * half:g},{2 * half:g}"/></g>'
    )


def static_drawing(example: Example) -> StaticDrawing:
    """Return the end of an example's drive as a static drawing, in the live drawing's look."""
    robot = ROBOTS[EXAMPLE_ROBOT]
    inputs = model_inputs(robot, example.settings)
    drive = example_drive(example)
    scale = drawing_scale(DRAWING.static_width_px, drive.samples, reach(inputs))
    step = grid_step(scale)
    start, end = drive.samples[0], drive.samples[-1]
    elements = [grid_lines(scale, step)]
    if inputs.frame_offset > 0:
        elements.append(path_line(scale, [sample.axle for sample in drive.samples], 'ix-axle'))
    elements += [
        path_line(scale, [sample.frame for sample in drive.samples], 'ix-model'),
        path_line(scale, [sample.odometry for sample in drive.samples], 'ix-odometry'),
        path_line(scale, [sample.command for sample in drive.samples], 'ix-command'),
        robot_outline(scale, start.axle, inputs.wheels, 'ix-start'),
        robot_outline(
            scale, end.axle, inputs.wheels, 'ix-model',
            true_pose_markers(scale, end.axle, inputs.frame_offset),
        ),
        robot_outline(
            scale, end.odometry, inputs.configured, 'ix-odometry',
            odometry_marker(scale, end.odometry),
        ),
        command_marker(scale, end.command),
    ]
    svg = svg_element(
        'motion-lab', 'ix-static', (scale.width_px, scale.height_px), DRAWING_TITLE, elements
    )
    return StaticDrawing(svg, f'Grid: {round(step * 100)} cm')


# --- The include -----------------------------------------------------------------------------

class LegendEntry(NamedTuple):
    """An entry of the legend: its view, its text, its sample of line and symbol (SVG elements).

    The entry of the axle's midpoint starts hidden: the script shows it while base_footprint
    lies behind the axle.
    """

    view: str
    text: str
    glyph: str
    starts_hidden: bool = False


LEGEND = (
    LegendEntry(
        'command',
        'command',
        '<line class="ix-path ix-command" x1="1" y1="6" x2="19" y2="6"/>'
        '<path class="ix-cross" d="M20.5,2.5 l7,7 m0,-7 l-7,7"/>',
    ),
    LegendEntry(
        'odometry',
        'odometry',
        '<line class="ix-path ix-odometry" x1="1" y1="6" x2="17" y2="6"/>'
        '<circle class="ix-ring" cx="23" cy="6" r="4"/>',
    ),
    LegendEntry(
        'model',
        'true pose (model)',
        '<line class="ix-path ix-model" x1="1" y1="6" x2="18" y2="6"/>'
        '<circle class="ix-dot" cx="23" cy="6" r="4"/>',
    ),
    LegendEntry(
        'axle',
        "axle's midpoint",
        '<line class="ix-path ix-axle" x1="2" y1="6" x2="18" y2="6"/>'
        '<circle class="ix-dot ix-axle" cx="23" cy="6" r="3.5"/>',
        starts_hidden=True,
    ),
)


class Slider(NamedTuple):
    """A slider of the figure: the setting it changes, its range and step, its texts.

    help_text is HTML with the placeholders {lowest}, {highest} and {default}.
    """

    setting: str
    lowest: int
    highest: int
    step: int
    label: str
    help_text: str

    @property
    def element_id(self) -> str:
        """Return the slider's HTML id."""
        return 'motion-lab-' + self.setting.replace('_', '-')


CONFIGURATION_SLIDERS = (
    Slider(
        'radius_percent', 80, 120, 1, 'Configured wheel radius',
        "<code>wheel_radius</code> in <code>geometry.yaml</code>, as a share of the robot's wheel"
        ' radius',
    ),
    Slider(
        'separation_percent', 80, 120, 1, 'Configured wheel separation',
        '<code>wheel_separation</code> in <code>geometry.yaml</code>, as a share of the distance'
        " between the robot's wheels",
    ),
)
WHAT_IF_SLIDERS = (
    Slider(
        'offset_mm', 0, 60, 1, '<code>base_footprint</code> behind the axle',
        "On DiffBot and Remo this is {default} mm: <code>base_footprint</code> lies at the axle's"
        ' midpoint.',
    ),
    Slider(
        'contact_percent', 0, 100, 5, 'Where the wheels push on the ground',
        "{lowest} %: in the middle of each wheel. {highest} %: at the wheels' outer edges. An"
        ' assumption that only changes how far apart the wheels push when the robot turns. No'
        ' contact forces are computed.',
    ),
    Slider(
        'tracking_percent', 70, 110, 1, 'Wheel speed reached',
        'The share of its commanded speed that each wheel reaches, the same for both wheels. A'
        ' fixed share, not a PID controller.',
    ),
)


def camel_case(name: str) -> str:
    """Return a Python name in JavaScript's form: wheel_width becomes wheelWidth."""
    first, *rest = name.split('_')
    return first + ''.join(word.capitalize() for word in rest)


def robots_data() -> Dict[str, Dict[str, object]]:
    """Return each robot's name and wheel geometry, by its key."""
    return {
        key: {
            'name': robot.name,
            'radius': robot.wheels.radius,
            'separation': robot.wheels.separation,
            'wheelWidth': robot.wheels.width,
        }
        for key, robot in ROBOTS.items()
    }


def commands_data() -> Dict[str, Dict[str, object]]:
    """Return each command's velocities, duration and text, by its kind."""
    return {
        kind.value: {
            'linearVelocity': command.linear_velocity,
            'angularVelocity': command.angular_velocity,
            'duration': command.duration,
            'text': command_text(kind),
        }
        for kind, command in COMMANDS.items()
    }


def checks_data() -> Dict[str, List[Dict[str, object]]]:
    """Return each test's checks in its order, with their values and texts, by the test's kind."""
    return {
        kind.value: [
            {
                'view': row.view.value,
                'quantity': row.check.quantity.value,
                'expected': row.check.expected,
                'tolerance': row.check.tolerance,
                'unit': UNITS[row.check.quantity],
                'name': row.name,
                'nameInSentence': row.name_in_sentence,
                'allowed': row.allowed,
            }
            for row in check_rows(kind)
        ]
        for kind in CommandKind
    }


def figure_data() -> Dict[str, object]:
    """Return the data that motion-lab.js reads from the include."""
    first = EXAMPLES[0]
    return {
        'source': SOURCE_COMMIT,
        'robots': robots_data(),
        'commands': commands_data(),
        'checks': checks_data(),
        'viewNames': {entry.view: entry.text for entry in LEGEND if entry.view != 'axle'},
        'drawing': {camel_case(name): value for name, value in DRAWING._asdict().items()},
        'default': {'robot': EXAMPLE_ROBOT, 'command': first.command.value},
        'examples': [
            {
                'key': example.key,
                'command': example.command.value,
                'settings': changed_settings(example.settings),
            }
            for example in EXAMPLES
        ],
    }


def glyph(entry: LegendEntry) -> str:
    """Return the small line-and-symbol sample of a legend entry."""
    return f'<svg class="ix-glyph" viewBox="0 0 28 12" aria-hidden="true">{entry.glyph}</svg>'


def legend_html() -> str:
    """Return the legend: a sample of line and symbol per view."""
    entries = ''.join(
        f'<li data-view="{entry.view}"{" hidden" if entry.starts_hidden else ""}>'
        f'{glyph(entry)}{entry.text}</li>'
        for entry in LEGEND
    )
    return f'<ul class="ix-legend">{entries}</ul>'


def example_item_html(example: Example) -> str:
    """Return an example's list item: its question, its button and its answer.

    The item carries the drive's expected results for the browser tests: the odometry's and the
    true pose's motion, the failing checks and the number of checks.
    """
    drive = example_drive(example)
    expected = {
        'odometry': list(drive.odometry),
        'model': list(drive.model),
        'failing': failing_checks(example.command, drive),
        'checks': len(check_rows(example.command)),
    }
    texts = example_values(example)
    return (
        f'<li data-example-item="{example.key}"'
        f' data-expected="{html.escape(json.dumps(expected))}"><span'
        f' class="ix-question">{example.question.format(**texts)}</span> <button type="button"'
        f' class="ix-show" data-example="{example.key}" data-live hidden>Show what'
        f' happens</button><span class="ix-answer">{example.answer.format(**texts)}</span></li>'
    )


def steps_html() -> str:
    """Return the guided examples: the introduction, the stepper and the examples' list."""
    robot = ROBOTS[EXAMPLE_ROBOT].name
    return ''.join([
        '<div class="ix-steps">',
        f'<p><b>{NUMBER_WORDS[len(EXAMPLES)]} drives to try.</b> Each one changes a single value'
        f' of {robot}. Guess the answer first.</p>',
        '<div class="ix-stepper" data-live hidden><button type="button"'
        ' class="ix-previous">Previous</button><span class="ix-step-count"'
        ' aria-live="polite"></span><button type="button" class="ix-next">Next</button><button'
        ' type="button" class="ix-reset" title="Back to the robot\'s own values">Reset'
        ' values</button></div>',
        f'<ol class="ix-examples">{"".join(example_item_html(e) for e in EXAMPLES)}</ol>',
        '</div>',
    ])


def toolbar_html() -> str:
    """Return the row above the drawing: the command's buttons and the legend."""
    first = EXAMPLES[0].command
    buttons = ''.join(
        f'<button type="button" data-command="{kind.value}"'
        f' aria-pressed="{"true" if kind is first else "false"}">{command_button_text(kind)}'
        '</button>'
        for kind in CommandKind
    )
    return (
        '<div class="ix-toolbar"><div class="ix-choice" role="group" aria-label="Command"'
        f' data-live hidden><span>Command</span>{buttons}</div>{legend_html()}</div>'
    )


def plot_html(drawing: StaticDrawing) -> str:
    """Return the drawing, the note under it and the play bar."""
    duration = COMMANDS[EXAMPLES[0].command].duration
    robot = ROBOTS[EXAMPLE_ROBOT].name
    return '\n'.join([
        f'<div class="ix-plot" style="aspect-ratio: 1 / {DRAWING.aspect}">{drawing.svg}</div>',
        f'<p class="ix-plotnote"><span class="ix-gridnote">{drawing.grid_note}</span><span'
        f' class="ix-static-note">The first example, with {robot}\'s values</span><span'
        f' class="ix-time">t = {duration:.1f} s, the end</span></p>',
        '<div class="ix-playbar" data-live hidden><button type="button"'
        ' class="ix-play">Play</button><input type="range" class="ix-scrub" min="0"'
        f' max="{TIME_SLIDER_STEPS}" value="{TIME_SLIDER_STEPS}" aria-label="Time in the'
        ' drive"></div>',
    ])


def results_html() -> str:
    """Return the places of the answer, the summary and the two tables, filled by the script."""
    return ''.join([
        '<div class="ix-results" data-live hidden>',
        '<p class="ix-shown-answer" tabindex="-1"></p>',
        '<p class="ix-summary" id="motion-lab-summary"></p>',
        '<table class="ix-table"><caption>Motion at the end of the drive, from the start pose, as'
        ' the test measures it</caption><thead><tr><th scope="col">View</th><th'
        ' scope="col">forward</th><th scope="col">left</th><th scope="col">turned</th></tr>'
        '</thead><tbody class="ix-motion-rows"></tbody></table>',
        '<table class="ix-table ix-checks"><caption>The test\'s checks, predicted for this'
        ' drive</caption><thead><tr><th scope="col">Check</th><th scope="col">Allowed</th><th'
        ' scope="col">This drive</th><th scope="col">Result</th></tr></thead><tbody'
        ' class="ix-check-rows"></tbody></table>',
        '</div>',
    ])


def slider_html(slider: Slider) -> str:
    """Return one labelled slider with its value and help text."""
    default = getattr(DEFAULT_SETTINGS, slider.setting)
    help_text = slider.help_text.format(
        lowest=slider.lowest, highest=slider.highest, default=default
    )
    return (
        f'<div class="ix-control"><label for="{slider.element_id}">{slider.label}</label><output'
        f' for="{slider.element_id}"></output><input type="range" id="{slider.element_id}"'
        f' data-setting="{slider.setting}" min="{slider.lowest}" max="{slider.highest}"'
        f' step="{slider.step}" value="{default}" aria-describedby="{slider.element_id}-help">'
        f'<small id="{slider.element_id}-help">{help_text}</small></div>'
    )


def settings_html() -> str:
    """Return the closed block with the robot's choice and the sliders."""
    robots = ''.join(
        f'<option value="{key}">{robot.name}</option>' for key, robot in ROBOTS.items()
    )
    return '\n'.join([
        '<details class="ix-settings" data-live hidden><summary>Change the values'
        ' yourself</summary>',
        '<div class="ix-controls">',
        '<div class="ix-control ix-choice"><label for="motion-lab-robot">Robot</label><select'
        f' id="motion-lab-robot" class="ix-robot-select">{robots}</select></div>',
        '<p class="ix-group">The controller\'s configuration</p>',
        *(slider_html(slider) for slider in CONFIGURATION_SLIDERS),
        '<p class="ix-group">What if</p>',
        *(slider_html(slider) for slider in WHAT_IF_SLIDERS),
        '<div><button type="button" class="ix-reset">Reset to the robot\'s values</button>'
        ' <small>Double-click a slider to reset only that one.</small></div>',
        '</div>',
        '</details>',
    ])


def caption_html() -> str:
    """Return the figure's caption: what the drawing shows, and what the figure computes."""
    return (
        '<figcaption><p>One drive of the simulation test from above. Solid orange: where the'
        " robot really is, in this figure's model. Dashed blue: where the odometry says the robot"
        ' is, drawn with the wheel radius and separation that the controller is configured with.'
        ' Gray cross: where the command alone would put the robot. Faint: the start. Both frames'
        ' are <code>base_footprint</code>.</p><p class="ix-model-note"><b>What this figure'
        ' computes.</b> Kinematics only: the wheels roll without slipping and reach their speed'
        ' at once, and the robot moves exactly as its wheel speeds and its geometry dictate.'
        " Gazebo's true pose also depends on the motor control, the contacts and the timing of"
        ' the commands, so the simulation test measures slightly different values. The browser'
        " doesn't run the test: the checks are predicted with the test's tolerances. Geometry and"
        f' tolerances: <a href="{SOURCE_URL}/diffbot_control/config"><code>geometry.yaml</code>'
        f'</a> and <a href="{SOURCE_URL}/diffbot_gazebo/test/test_drive.py"><code>test_drive.py'
        f'</code></a> at diffbot <code>{SOURCE_COMMIT[:7]}</code>.</p></figcaption>'
    )


def build() -> str:
    """Return the include's HTML."""
    data = json.dumps(figure_data(), separators=(',', ':'))
    lines = [
        '<!-- Written by scripts/figures/motion_lab_figures.py -->',
        '<figure class="ix ix-motion" data-ix="motion-lab">',
        f'<script type="application/json" class="ix-data">{data}</script>',
        steps_html(),
        toolbar_html(),
        plot_html(static_drawing(EXAMPLES[0])),
        results_html(),
        settings_html(),
        '<div class="ix-announce" aria-live="polite"></div>',
        caption_html(),
        '</figure>',
    ]
    return '\n'.join(lines) + '\n'


def check_include(text: str) -> None:
    """Raise ValueError if the include has a blank line or a template marker."""
    if '\n\n' in text:
        raise ValueError('the include has a blank line, which would end its HTML block')
    markers = [marker for marker in TEMPLATE_MARKERS if marker in text]
    if markers:
        raise ValueError(f'the include has the template markers {markers}')


def main() -> None:
    """Build the include, check it and write it."""
    text = build()
    check_include(text)
    INCLUDE.write_text(text, encoding='utf-8')
    print(f'wrote {INCLUDE.relative_to(ROOT)}')


if __name__ == '__main__':
    main()
