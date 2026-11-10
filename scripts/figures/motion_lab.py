"""Generate includes/motion-lab.html, the interactive figure on the Simulation Test page.

The figure "Three views of one drive" is on docs/development/simulation-test.md.

The include holds everything the figure needs:

- the robots' wheel geometry, the test's two commands and its checks, as data for the script
  docs/custom_theme/js/interactive/motion-lab.js;
- a static drawing of the first example and the answers of all examples, computed here. They
  are what the page shows without JavaScript, and until the script has drawn the figure;
- the controls, which the script shows once the figure works.

The values come from the diffbot repository at SOURCE_COMMIT: wheel_radius and wheel_separation
from diffbot_control/config/<robot>/geometry.yaml, the wheel width from the robots' descriptions
(front_wheel.yaml, length), the commands and tolerances from diffbot_gazebo/test/test_drive.py.
Update them here when they change there, and run from the repository root:

    python3 scripts/figures/motion_lab.py

tests/test_motion_lab.py checks the computed answers against values worked out by hand, and
that the include is the output of this script.
"""

import html
import json
import math
from pathlib import Path
from typing import Callable, Dict, List, NamedTuple, Tuple

ROOT = Path(__file__).resolve().parent.parent.parent
INCLUDE = ROOT / 'includes' / 'motion-lab.html'

SOURCE_COMMIT = 'f5f282eed7b6a2b146d40c7d5fd641b8a339a77d'
SOURCE_URL = f'https://github.com/ros-mobile-robots/diffbot/blob/{SOURCE_COMMIT}'


class Robot(NamedTuple):
    """A robot's name and wheel geometry: radius, separation and width of its wheels (m).

    The separation is the distance between the two wheels' center planes.
    """

    name: str
    radius: float
    separation: float
    wheel_width: float


class Command(NamedTuple):
    """A velocity command of the test: v (m/s) and w (rad/s), sent for the duration (s)."""

    name: str
    v: float
    w: float
    duration: float


class Example(NamedTuple):
    """A guided example: a command and the one setting that the example changes.

    question and answer are HTML. The answer's {placeholders} are filled with the computed
    values.
    """

    key: str
    command: str
    settings: Dict[str, int]
    question: str
    answer: str


ROBOTS = {
    'remo': Robot('Remo', radius=0.0330, separation=0.144, wheel_width=0.025),
    'diffbot': Robot('DiffBot', radius=0.0333, separation=0.138, wheel_width=0.025),
}
# test_drive.py: FORWARD_VELOCITY (m/s), TURN_VELOCITY (rad/s) and DURATION (s)
COMMANDS = {
    'forward': Command('drive 0.6 m', v=0.2, w=0.0, duration=3.0),
    'turn': Command('turn 1.5 rad on the spot', v=0.0, w=0.5, duration=3.0),
}
# test_drive.py: the expected values and the delta of each assertAlmostEqual
CHECKS = {
    'forward': {
        'forward': {'expected': 0.6, 'tolerance': 0.1},
        'left': {'expected': 0.0, 'tolerance': 0.1},
        'turned': {'expected': 0.0, 'tolerance': 0.2},
        'agreement': {'tolerance': 0.03},
    },
    'turn': {
        'turned': {'expected': 1.5, 'tolerance': 0.3},
        'distance': {'expected': 0.0, 'tolerance': 0.02},
        'agreement': {'tolerance': 0.1},
    },
}
# The sliders: lowest, highest and step. radius and separation are percent of the robot's own
# value, offset is in mm, contact in percent of the wheel width, tracking in percent
SLIDERS = {
    'radius': (80, 120, 1),
    'separation': (80, 120, 1),
    'offset': (0, 60, 1),
    'contact': (0, 100, 5),
    'tracking': (70, 110, 1),
}
DEFAULT_SETTINGS = {'radius': 100, 'separation': 100, 'offset': 0, 'contact': 0, 'tracking': 100}
EXAMPLE_ROBOT = 'remo'
NUMBER_WORDS = {3: 'Three', 4: 'Four', 5: 'Five', 6: 'Six', 7: 'Seven'}
ASPECT = 0.5  # Height of the drawing as a share of its width


class Pose(NamedTuple):
    """A pose on the ground: x and y (m) and the heading theta (rad)."""

    x: float
    y: float
    theta: float


class Motion(NamedTuple):
    """An end pose relative to a start pose, as test_drive.py's RelativePose."""

    forward: float
    left: float
    turned: float


class Drive(NamedTuple):
    """The end of one drive: the three views' motions and the poses for the drawing."""

    command: Motion
    odometry: Motion
    model: Motion
    command_pose: Pose
    odometry_pose: Pose
    axle_pose: Pose


def wrap(angle: float) -> float:
    """Return the same angle in -pi..pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_after(v: float, w: float, t: float) -> Pose:
    """Return the pose after a constant velocity for the time t, from 0, 0 with heading 0."""
    if abs(w) < 1e-12:
        return Pose(v * t, 0.0, 0.0)
    theta = w * t
    return Pose(v / w * math.sin(theta), v / w * (1 - math.cos(theta)), theta)


def motion_from_origin(pose: Pose) -> Motion:
    """Return the motion from the pose 0, 0, 0 to a pose."""
    return Motion(pose.x, pose.y, wrap(pose.theta))


def simulate(robot: Robot, command: Command, settings: Dict[str, int]) -> Drive:
    """Compute one drive's three views at its end, with the figure's kinematic model.

    The controller turns the command into wheel speeds with its configured radius and
    separation. The wheels reach `tracking` of them. The odometry converts the wheel turns back
    with the configured values. The model converts them with the robot's own radius and the
    separation at which the wheels push on the ground.
    """
    configured_radius = robot.radius * settings['radius'] / 100
    configured_separation = robot.separation * settings['separation'] / 100
    contact_separation = robot.separation + robot.wheel_width * settings['contact'] / 100
    offset = settings['offset'] / 1000
    tracking = settings['tracking'] / 100
    v, w, duration = command.v, command.w, command.duration

    left = tracking * (v - w * configured_separation / 2) / configured_radius
    right = tracking * (v + w * configured_separation / 2) / configured_radius
    odometry_v = configured_radius * (right + left) / 2
    odometry_w = configured_radius * (right - left) / configured_separation
    model_v = robot.radius * (right + left) / 2
    model_w = robot.radius * (right - left) / contact_separation

    command_pose = pose_after(v, w, duration)
    odometry_pose = pose_after(odometry_v, odometry_w, duration)
    axle = pose_after(model_v, model_w, duration)
    # The reported frame starts at 0, 0 and lies `offset` behind the axle's midpoint
    axle_pose = Pose(axle.x + offset, axle.y, axle.theta)
    frame = Pose(
        axle_pose.x - offset * math.cos(axle.theta),
        axle_pose.y - offset * math.sin(axle.theta),
        axle.theta,
    )
    return Drive(
        motion_from_origin(command_pose),
        motion_from_origin(odometry_pose),
        motion_from_origin(frame),
        command_pose,
        odometry_pose,
        axle_pose,
    )


def failing_checks(kind: str, drive: Drive) -> List[str]:
    """Return the names of the test's checks that the drive fails, in the test's order."""
    checks = CHECKS[kind]
    failing = []

    def check(name: str, value: float, expected: float, tolerance: float) -> None:
        # The same small allowance for rounding as in diffdrive.js
        if abs(value - expected) > tolerance + 1e-9:
            failing.append(name)

    for source, motion in (('odometry', drive.odometry), ('true pose', drive.model)):
        if kind == 'forward':
            check(
                f'forward, {source}',
                motion.forward,
                checks['forward']['expected'],
                checks['forward']['tolerance'],
            )
            check(f'left, {source}', motion.left, 0.0, checks['left']['tolerance'])
            check(f'turned, {source}', motion.turned, 0.0, checks['turned']['tolerance'])
        else:
            check(
                f'turned, {source}',
                motion.turned,
                checks['turned']['expected'],
                checks['turned']['tolerance'],
            )
            check(
                f'distance from the start, {source}',
                math.hypot(motion.forward, motion.left),
                0.0,
                checks['distance']['tolerance'],
            )
    if kind == 'forward':
        check(
            'odometry and true pose, forward',
            abs(drive.odometry.forward - drive.model.forward),
            0.0,
            checks['agreement']['tolerance'],
        )
    else:
        check(
            'odometry and true pose, turned',
            abs(drive.odometry.turned - drive.model.turned),
            0.0,
            checks['agreement']['tolerance'],
        )
    return failing


def check_count(kind: str) -> int:
    """Return how many checks the test makes for a drive."""
    return 7 if kind == 'forward' else 5


# The guided examples. Each changes one setting of the example robot
EXAMPLES = [
    Example(
        key='radius',
        command='forward',
        settings={'radius': 110},
        question=(
            '<b>The configured wheel radius is 10 % too big.</b> The robot is told to drive 0.6 m.'
            ' How far does the odometry say the robot drove, and how far did the robot drive?'
        ),
        answer=(
            'The odometry reports {odometry_forward} m. The robot drives {model_forward} m. The'
            ' controller commands fewer wheel turns, and multiplies them by the same too-big'
            ' radius. Only the check that compares the odometry with the true pose fails.'
        ),
    ),
    Example(
        key='separation',
        command='turn',
        settings={'separation': 95},
        question=(
            '<b>The configured wheel separation is 5 % too small.</b> The robot is told to turn'
            ' 1.5 rad on the spot. Does a check fail?'
        ),
        answer=(
            'No. The odometry reports {odometry_turned} rad, and the robot turns {model_turned}'
            ' rad. The two differ by {gap_turned} rad, and the check allows 0.1 rad. The test only'
            ' catches a separation that is off by more than about 7 %.'
        ),
    ),
    Example(
        key='offset',
        command='turn',
        settings={'offset': 20},
        question=(
            '<b>What if <code>base_footprint</code> were 2 cm behind the axle?</b> The robot turns'
            ' 1.5 rad on the spot. Does <code>base_footprint</code> stay where it was?'
        ),
        answer=(
            'No. The robot turns about the midpoint of its axle. A frame 2 cm behind that midpoint'
            ' moves {model_distance_cm} cm on an arc, and the check that allows 0.02 m fails. The'
            ' odometry reports no movement. On DiffBot and Remo, <code>base_footprint</code> is at'
            " the axle's midpoint and stays in place."
        ),
    ),
    Example(
        key='contact',
        command='turn',
        settings={'contact': 100},
        question=(
            '<b>What if the wheels pushed on the ground at their outer edges?</b> The robot is'
            ' told to turn 1.5 rad on the spot. How far does the robot turn?'
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
        command='forward',
        settings={'tracking': 80},
        question=(
            '<b>The wheels reach only 80 % of their commanded speed.</b> The robot is told to'
            ' drive 0.6 m. Do the odometry and the true pose still agree?'
        ),
        answer=(
            'Yes. Both show {model_forward} m, because the odometry counts the wheel turns that'
            ' really happened. The two checks against the expected 0.6 m fail, and the check that'
            ' compares the odometry with the true pose passes.'
        ),
    ),
]


def example_drive(example: Example) -> Drive:
    """Return the drive of a guided example."""
    return simulate(
        ROBOTS[EXAMPLE_ROBOT],
        COMMANDS[example.command],
        {**DEFAULT_SETTINGS, **example.settings},
    )


def example_values(example: Example) -> Dict[str, str]:
    """Return the values that an example's answer shows, formatted."""
    robot = ROBOTS[EXAMPLE_ROBOT]
    drive = example_drive(example)
    settings = {**DEFAULT_SETTINGS, **example.settings}
    contact_separation = robot.separation + robot.wheel_width * settings['contact'] / 100
    # How much less the robot turns than its odometry reports, for the turns
    less = 1 - drive.model.turned / drive.odometry.turned if drive.odometry.turned else 0.0
    return {
        'odometry_forward': f'{drive.odometry.forward:.3f}',
        'model_forward': f'{drive.model.forward:.3f}',
        'odometry_turned': f'{drive.odometry.turned:.3f}',
        'model_turned': f'{drive.model.turned:.3f}',
        'gap_turned': f'{abs(drive.odometry.turned - drive.model.turned):.3f}',
        'model_distance_cm': f'{math.hypot(drive.model.forward, drive.model.left) * 100:.1f}',
        'less_percent': f'{less * 100:.0f}',
        'contact_separation': f'{contact_separation:.3f}',
        'separation': f'{robot.separation:.3f}',
    }


# --- The static drawing ----------------------------------------------------------------------

Point = Tuple[float, float]


def robot_outline(
    pose: Pose,
    radius: float,
    separation: float,
    width: float,
    to_px: Callable[[float, float], Point],
    css: str,
    extra: str = '',
) -> str:
    """Return the SVG of a robot from above: two wheels, the axle and an arrow to its front."""
    cos, sin = math.cos(pose.theta), math.sin(pose.theta)

    def at(forward: float, left: float) -> Point:
        return to_px(pose.x + forward * cos - left * sin, pose.y + forward * sin + left * cos)

    def points(corners: List[Point]) -> str:
        return ' '.join(f'{x:.1f},{y:.1f}' for x, y in corners)

    parts = [f'<g class="ix-robot {css}">']
    for side in (1, -1):
        y = side * separation / 2
        corners = [
            at(radius, y + width / 2),
            at(radius, y - width / 2),
            at(-radius, y - width / 2),
            at(-radius, y + width / 2),
        ]
        parts.append(f'<polygon class="ix-wheel" points="{points(corners)}"/>')
    (lx, ly), (rx, ry) = at(0, separation / 2), at(0, -separation / 2)
    parts.append(f'<line x1="{lx:.1f}" y1="{ly:.1f}" x2="{rx:.1f}" y2="{ry:.1f}"/>')
    nose = 0.05
    (ox, oy), (nx, ny) = at(0, 0), at(nose, 0)
    parts.append(f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{nx:.1f}" y2="{ny:.1f}"/>')
    head = [at(nose + 0.012, 0), at(nose, 0.008), at(nose, -0.008)]
    parts.append(f'<polygon class="ix-head" points="{points(head)}"/>')
    parts.append(extra + '</g>')
    return ''.join(parts)


def static_drawing(example: Example, width: int = 600) -> Tuple[str, str]:
    """Return a straight drive's end as static SVG and the drawing's grid note.

    The drawing has the look of the one that motion-lab.js draws.
    """
    robot = ROBOTS[EXAMPLE_ROBOT]
    settings = {**DEFAULT_SETTINGS, **example.settings}
    drive = example_drive(example)
    assert COMMANDS[example.command].w == 0, 'the static drawing draws its paths as straight lines'
    height = round(width * ASPECT)
    pad = 14
    reach = max(robot.separation / 2 + robot.wheel_width, robot.radius + 0.06) + 0.02
    xs = [0.0, drive.command_pose.x, drive.odometry_pose.x, drive.axle_pose.x]
    ys = [0.0, drive.command_pose.y, drive.odometry_pose.y, drive.axle_pose.y]
    x_min, x_max = min(xs) - reach, max(xs) + reach
    y_min, y_max = min(ys) - reach, max(ys) + reach
    scale = min((width - 2 * pad) / (x_max - x_min), (height - 2 * pad) / (y_max - y_min))
    cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2

    def to_px(x: float, y: float) -> Point:
        return width / 2 + (x - cx) * scale, height / 2 - (y - cy) * scale

    step = next((s for s in (0.01, 0.02, 0.05, 0.1, 0.2, 0.5) if s * scale >= 22), 1.0)
    parts = [
        (
            f'<svg class="ix-static" viewBox="0 0 {width} {height}" role="img"'
            ' aria-labelledby="motion-lab-title">'
        ),
        (
            '<title id="motion-lab-title">One drive from above: the command, the odometry and the'
            ' true pose</title>'
        ),
        '<g class="ix-grid">',
    ]
    x = math.ceil((cx - width / 2 / scale) / step) * step
    while x < cx + width / 2 / scale:
        parts.append(
            f'<line x1="{to_px(x, 0)[0]:.1f}" y1="0" x2="{to_px(x, 0)[0]:.1f}" y2="{height}"/>'
        )
        x += step
    y = math.ceil((cy - height / 2 / scale) / step) * step
    while y < cy + height / 2 / scale:
        parts.append(
            f'<line x1="0" y1="{to_px(0, y)[1]:.1f}" x2="{width}" y2="{to_px(0, y)[1]:.1f}"/>'
        )
        y += step
    parts.append('</g>')

    def path(end: Pose, css: str) -> str:
        (x0, y0), (x1, y1) = to_px(0, 0), to_px(end.x, end.y)
        return f'<polyline class="ix-path {css}" points="{x0:.1f},{y0:.1f} {x1:.1f},{y1:.1f}"/>'

    parts.append(path(drive.axle_pose, 'ix-model'))
    parts.append(path(drive.odometry_pose, 'ix-odometry'))
    parts.append(path(drive.command_pose, 'ix-command'))
    own = (robot.radius, robot.separation, robot.wheel_width)
    parts.append(robot_outline(Pose(0, 0, 0), *own, to_px, 'ix-start'))
    fx, fy = to_px(drive.axle_pose.x, drive.axle_pose.y)
    parts.append(
        robot_outline(
            drive.axle_pose,
            *own,
            to_px,
            'ix-model',
            f'<circle class="ix-dot" cx="{fx:.1f}" cy="{fy:.1f}" r="5"/>',
        )
    )
    configured = (
        robot.radius * settings['radius'] / 100,
        robot.separation * settings['separation'] / 100,
        robot.wheel_width,
    )
    qx, qy = to_px(drive.odometry_pose.x, drive.odometry_pose.y)
    parts.append(
        robot_outline(
            drive.odometry_pose,
            *configured,
            to_px,
            'ix-odometry',
            f'<circle class="ix-ring" cx="{qx:.1f}" cy="{qy:.1f}" r="5.5"/>',
        )
    )
    kx, ky = to_px(drive.command_pose.x, drive.command_pose.y)
    hx, hy = to_px(drive.command_pose.x + 0.085, drive.command_pose.y)
    parts.append(
        f'<g class="ix-robot ix-command"><line x1="{kx:.1f}" y1="{ky:.1f}" x2="{hx:.1f}"'
        f' y2="{hy:.1f}"/><path class="ix-cross" d="M{kx - 5:.1f},{ky - 5:.1f} l10,10 m0,-10'
        ' l-10,10"/></g>'
    )
    parts.append('</svg>')
    return ''.join(parts), f'Grid: {round(step * 100)} cm'


# --- The include -----------------------------------------------------------------------------

GLYPHS = {
    'command': (
        '<line class="ix-path ix-command" x1="1" y1="6" x2="19" y2="6"/>'
        '<path class="ix-cross" d="M20.5,2.5 l7,7 m0,-7 l-7,7"/>'
    ),
    'odometry': (
        '<line class="ix-path ix-odometry" x1="1" y1="6" x2="17" y2="6"/>'
        '<circle class="ix-ring" cx="23" cy="6" r="4"/>'
    ),
    'model': (
        '<line class="ix-path ix-model" x1="1" y1="6" x2="18" y2="6"/><circle class="ix-dot"'
        ' cx="23" cy="6" r="4"/>'
    ),
    'axle': (
        '<line class="ix-path ix-axle" x1="2" y1="6" x2="18" y2="6"/>'
        '<circle class="ix-dot ix-axle" cx="23" cy="6" r="3.5"/>'
    ),
}
# View, legend text, and whether the entry starts hidden
LEGEND = [
    ('command', 'command', False),
    ('odometry', 'odometry', False),
    ('model', 'true pose (model)', False),
    ('axle', "axle's midpoint", True),
]


def glyph(view: str) -> str:
    """Return the small line-and-symbol sample of a view, for the legend and the table."""
    return f'<svg class="ix-glyph" viewBox="0 0 28 12" aria-hidden="true">{GLYPHS[view]}</svg>'


def slider(name: str, label: str, help_text: str) -> str:
    """Return one labelled slider with its value and help text."""
    low, high, step = SLIDERS[name]
    return (
        f'<div class="ix-control"><label for="motion-lab-{name}">{label}</label><output'
        f' for="motion-lab-{name}"></output><input type="range" id="motion-lab-{name}"'
        f' data-setting="{name}" min="{low}" max="{high}" step="{step}"'
        f' value="{DEFAULT_SETTINGS[name]}" aria-describedby="motion-lab-{name}-help"><small'
        f' id="motion-lab-{name}-help">{help_text}</small></div>'
    )


def build() -> str:
    """Return the include's HTML."""
    data = {
        'source': SOURCE_COMMIT,
        'aspect': ASPECT,
        'robots': {
            key: {
                'name': robot.name,
                'radius': robot.radius,
                'separation': robot.separation,
                'wheelWidth': robot.wheel_width,
            }
            for key, robot in ROBOTS.items()
        },
        'commands': {key: command._asdict() for key, command in COMMANDS.items()},
        'checks': CHECKS,
        'default': {
            'robot': EXAMPLE_ROBOT,
            'command': EXAMPLES[0].command,
            'example': EXAMPLES[0].key,
        },
        'examples': [
            {'id': e.key, 'command': e.command, 'settings': e.settings} for e in EXAMPLES
        ],
    }
    drawing, grid_note = static_drawing(EXAMPLES[0])
    legend = ''.join(
        f'<li data-view="{view}"{" hidden" if hidden else ""}>{glyph(view)}{text}</li>'
        for view, text, hidden in LEGEND
    )
    examples = []
    for example in EXAMPLES:
        drive = example_drive(example)
        expected = {
            'odometry': list(drive.odometry),
            'model': list(drive.model),
            'failing': failing_checks(example.command, drive),
            'checks': check_count(example.command),
        }
        answer = example.answer.format(**example_values(example))
        examples.append(
            f'<li data-example-item="{example.key}"'
            f' data-expected="{html.escape(json.dumps(expected))}"><span'
            f' class="ix-question">{example.question}</span> <button type="button"'
            f' class="ix-show" data-example="{example.key}" data-live hidden>Show what'
            f' happens</button><span class="ix-answer">{answer}</span></li>'
        )
    robots = ''.join(
        f'<option value="{key}">{robot.name}</option>' for key, robot in ROBOTS.items()
    )
    lines = [
        '<figure class="ix ix-motion" data-ix="motion-lab">',
        (
            '<script type="application/json"'
            f' class="ix-data">{json.dumps(data, separators=(",", ":"))}</script>'
        ),
        f'<ul class="ix-legend">{legend}</ul>',
        f'<div class="ix-plot" style="aspect-ratio: 1 / {ASPECT}">{drawing}</div>',
        (
            f'<p class="ix-plotnote"><span class="ix-gridnote">{grid_note}</span> '
            '<span class="ix-time">t = 3.0 s, the end</span></p>'
        ),
        (
            '<div class="ix-playbar" data-live hidden><button type="button"'
            ' class="ix-play">Play</button><input type="range" class="ix-scrub" min="0" max="1000"'
            ' value="1000" aria-label="Time in the drive"></div>'
        ),
        (
            '<figcaption>One drive of the simulation test from above. Solid orange: where the'
            " robot really is, in this figure's model. Dashed blue: where the odometry says the"
            ' robot is, drawn with the wheel radius and separation that the controller is'
            ' configured with. Gray cross: where the command alone would put the robot. Faint:'
            ' the start. Both frames are <code>base_footprint</code>.<span'
            ' class="ix-static-note"> The picture shows the first example, with'
            f" {ROBOTS[EXAMPLE_ROBOT].name}'s values.</span></figcaption>"
        ),
        '<div class="ix-steps">',
        (
            f'<p><b>{NUMBER_WORDS[len(EXAMPLES)]} drives to try.</b> Each one changes a single'
            f' value of {ROBOTS[EXAMPLE_ROBOT].name}. Guess the answer first.</p>'
        ),
        (
            '<div class="ix-stepper" data-live hidden><button type="button"'
            ' class="ix-previous">Previous</button><span class="ix-step-count"'
            ' aria-live="polite"></span><button type="button" class="ix-next">Next</button></div>'
        ),
        f'<ol class="ix-examples">{"".join(examples)}</ol>',
        '</div>',
        '<div class="ix-results" data-live hidden>',
        '<p class="ix-summary" id="motion-lab-summary"></p>',
        (
            '<table class="ix-table"><caption>Motion at the end of the drive, from the start pose,'
            ' as the test measures it</caption><thead><tr><th scope="col">View</th><th'
            ' scope="col">forward</th><th scope="col">left</th><th'
            ' scope="col">turned</th></tr></thead><tbody class="ix-motion-rows"></tbody></table>'
        ),
        (
            '<table class="ix-table ix-checks"><caption>The test\'s checks, predicted for this'
            ' drive</caption><thead><tr><th scope="col">Check</th><th scope="col">Allowed</th><th'
            ' scope="col">This drive</th><th scope="col">Result</th></tr></thead><tbody'
            ' class="ix-check-rows"></tbody></table>'
        ),
        '</div>',
        (
            '<details class="ix-settings" data-live hidden><summary>Change the values'
            ' yourself</summary>'
        ),
        '<div class="ix-controls">',
        (
            '<div class="ix-control ix-choice"><label for="motion-lab-robot">Robot</label>'
            f'<select id="motion-lab-robot" class="ix-robot-select">{robots}</select></div>'
        ),
        (
            '<div class="ix-control ix-choice" role="group"'
            ' aria-label="Command"><span>Command</span><button type="button"'
            ' data-command="forward" aria-pressed="true">Drive 0.6 m</button><button type="button"'
            ' data-command="turn" aria-pressed="false">Turn 1.5 rad</button></div>'
        ),
        '<p class="ix-group">The controller\'s configuration</p>',
        slider(
            'radius',
            'Configured wheel radius',
            "<code>wheel_radius</code> in <code>geometry.yaml</code>, as a share of the robot's"
            ' wheel radius',
        ),
        slider(
            'separation',
            'Configured wheel separation',
            '<code>wheel_separation</code> in <code>geometry.yaml</code>, as a share of the'
            " distance between the robot's wheels",
        ),
        '<p class="ix-group">What if</p>',
        slider(
            'offset',
            '<code>base_footprint</code> behind the axle',
            "On DiffBot and Remo this is 0 mm: <code>base_footprint</code> lies at the axle's"
            ' midpoint.',
        ),
        slider(
            'contact',
            'Where the wheels push on the ground',
            "0 %: in the middle of each wheel. 100 %: at the wheels' outer edges. An assumption"
            ' that only changes how far apart the wheels push when the robot turns. No contact'
            ' forces are computed.',
        ),
        slider(
            'tracking',
            'Wheel speed reached',
            'The share of its commanded speed that each wheel reaches, the same for both wheels. A'
            ' fixed share, not a PID controller.',
        ),
        (
            '<div><button type="button" class="ix-reset">Reset to the robot\'s values</button> '
            '<small>Double-click a slider to reset only that one.</small></div>'
        ),
        '</div>',
        '</details>',
        (
            '<p class="ix-model-note"><b>What this figure computes.</b> Kinematics only: the'
            ' wheels roll without slipping and reach their speed at once, and the robot moves'
            " exactly as its wheel speeds and its geometry dictate. Gazebo's true pose also"
            ' depends on the motor control, the contacts and the timing of the commands, so the'
            " simulation test measures slightly different values. The browser doesn't run the"
            " test: the checks are predicted with the test's tolerances. Geometry and"
            ' tolerances: <a'
            f' href="{SOURCE_URL}/diffbot_control/config"><code>geometry.yaml</code></a> and <a'
            f' href="{SOURCE_URL}/diffbot_gazebo/test/test_drive.py"><code>test_drive.py</code>'
            f'</a> at diffbot <code>{SOURCE_COMMIT[:7]}</code>.</p>'
        ),
        '<div class="ix-announce" aria-live="polite"></div>',
        '</figure>',
    ]
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    output = build()
    # MkDocs inserts the include into a Markdown page: a blank line would end its HTML block, and
    # these character pairs would start a template expression
    assert '\n\n' not in output and not any(pair in output for pair in ('{{', '{#', '{%'))
    INCLUDE.write_text(output, encoding='utf-8')
    print(f'wrote {INCLUDE.relative_to(ROOT)}')
