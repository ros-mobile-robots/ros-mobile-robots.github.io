"""Generate the figures of the Localization chapter's test pages on diffbot_gazebo's tests.

The pages are docs/packages/imu/robot_localization/tests/gazebo.md, switch.md and odom-tf.md.
The script writes three includes that follow the site's light and dark colors:

- includes/localization-test-gazebo.html: the left turn of test_ekf.py from above, one drawing
  for DiffBot and one for Remo, each with the true turn, the EKF's turn and the wheel odometry's
  turn. The two drawings stand side by side on a wide screen and one below the other on a phone.
- includes/localization-test-switch.html: how test_ekf_switch.py works. The test gives a launch
  file to roslaunch's loader and asserts on the configuration that the loader returns.
- includes/localization-test-odom-tf.html: who publishes the transform odom → base_footprint
  on the topic /tf without the EKF (test_odom_tf.py) and with it (test_ekf.py).

A drawing carries short labels only. Headings, results and legends are HTML text around the
drawings, so that they keep the page's text size on a phone.

The tests' constants are copied from the tests in the diffbot repository (diffbot_gazebo/test/).
A comment names each. The measured values are those of a saved run of the tests. They are in
scripts/figures/data/localization-gazebo-test-runs.jsonl, one line per rostest file, with the
commit of the diffbot repository, the command and the test file.

Run from the repository root after changing a figure:

    python3 scripts/figures/localization_gazebo_test_figures.py
"""

import json
import math
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

from localization_figures import band
from localization_figures import BG
from localization_figures import escape
from localization_figures import FG
from localization_figures import FG_LIGHT
from localization_figures import Figure
from localization_figures import LARGE_RULES
from localization_figures import RULES
from localization_figures import write_parts

RUNS_FILE = 'scripts/figures/data/localization-gazebo-test-runs.jsonl'

# test_ekf.py: the limits of the turn test
MIN_ODOMETRY_ERROR = 0.15  # rad, the least by which the odometry's turn must fall behind
EKF_TOLERANCE = 0.05  # rad, how far the EKF's turn may differ from the true one
# wheel_separation in diffbot_control/config/<model>/geometry.yaml (m)
ROBOT_WHEEL_SEPARATION = {'DiffBot': 0.138, 'Remo': 0.144}

# The node that each simulation expects as the transform's only publisher: the publisher
# parameter in sim_drive_<model>.test, and EKF_NODE in test_ekf.py
CONTROLLER_NODE = '/gazebo'
EKF_NODE = '/diffbot/ekf_localization'

# test_ekf_switch.py: one of the test's calls of load(), as its description writes it
LOADED_EXAMPLE = 'diffbot_gazebo diffbot.launch ekf:=true'

LINE_HEIGHT = 14  # between the baselines of a box's lines
HEAD_LENGTH = 9  # of an arrowhead
HEAD_HALF_WIDTH = 3.8

# The turn figure: one drawing per robot, the robot at the center of rotation
TURN_WIDTH = 208
TURN_HEIGHT = 126
TURN_CENTER_X = 120
TURN_CENTER_Y = 104
BODY_RADIUS = 14
TRUE_RADIUS = 40  # the radii of the three arcs, one for each turn
EKF_RADIUS = 60
ODOMETRY_RADIUS = 80
TOLERANCE_HALF_WIDTH = 7  # of the sector around the EKF's arc that shows EKF_TOLERANCE
START_RADIUS = 86

# The transform figure: two publishers on the left, the observer on the right
TF_WIDTH = 300
PUBLISHER_X = 2
PUBLISHER_WIDTH = 186
PUBLISHER_GAP = 8
OBSERVER_X = 226

# The switch figure: the test's flow from top to bottom
FLOW_WIDTH = 300
FLOW_X = 2
FLOW_GAP = 22  # between two boxes, for an arrow
FLOW_CELL_WIDTH = 88
FLOW_CELL_HEIGHT = 26
ASSERTIONS_WIDTH = 156

EXTRA_RULES = (
    ('.lv-fig .true-turn path', f'stroke:{FG};stroke-width:2.2;fill:none'),
    ('.lv-fig .true-turn polygon', f'fill:{FG}'),
    ('.lv-fig .ekf-turn path', 'stroke:var(--lv-estimate);stroke-width:2.2;fill:none'),
    ('.lv-fig .ekf-turn polygon', 'fill:var(--lv-estimate)'),
    ('.lv-fig .odometry-turn path', 'stroke:var(--lv-wheels);stroke-width:2.2;fill:none'),
    ('.lv-fig .odometry-turn polygon', 'fill:var(--lv-wheels)'),
    ('.lv-fig .start-heading', f'stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:3 3;'
                               'fill:none'),
    ('.lv-fig .tolerance', 'fill:var(--lv-good-fill);stroke:var(--lv-good);stroke-width:.8'),
    ('.lv-fig .body', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:1.2'),
    ('.lv-fig .nose', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2'),
    ('.lv-fig .wheel', f'fill:{FG_LIGHT}'),
    ('.lv-fig .flow', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-fig .flow-head', f'fill:{FG}'),
    ('.lv-fig .flow-skipped', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2;'
                              'stroke-dasharray:3 3'),
    ('.lv-fig .flow-head-skipped', f'fill:{FG_LIGHT}'),
    ('.lv-fig .active', 'fill:var(--lv-good-fill);stroke:var(--lv-good);stroke-width:1.6'),
    ('.lv-fig .inactive', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:3 3'),
    # Each robot of the turn figure: its heading, its drawing and its results in one column
    ('.lv-grids .lv-pair .lv-case', 'width:250px;max-width:100%'),
    ('.lv-grids .lv-pair .lv-case .lv-fig', 'width:100%'),
    ('.lv-legend .lv-swatch.lv-inactive', 'border-style:dashed'),
    # The legend's sample of an arc of the turn figure
    ('.lv-legend .lv-arc', 'display:inline-block;box-sizing:border-box;width:1.6em;height:0;'
                           f'vertical-align:.3em;border-top:3px solid {FG};text-indent:0;'
                           'margin-right:.4em'),
    ('.lv-legend .lv-arc.lv-ekf', 'border-top-color:var(--lv-estimate)'),
    ('.lv-legend .lv-arc.lv-odometry', 'border-top-color:var(--lv-wheels)'),
    # A line under a part's heading, and a list of results whose lines start with a sign
    ('.md-typeset .lv-note', 'margin:0 0 .4em;font-size:.7rem;text-align:left'),
    ('.md-typeset ul.lv-legend.lv-results li', 'padding-left:1.4em;text-indent:-1.4em'),
    ('.lv-legend .lv-sign', 'display:inline-block;width:1.4em;text-indent:0;font-weight:700'),
)
ALL_RULES = RULES + LARGE_RULES + EXTRA_RULES


class Turn(NamedTuple):
    """One robot's left turn in a saved run of sim_ekf_<model>.test, as test_ekf.py printed it."""

    robot: str
    controller_separation: float  # m, the drive controller's wheel_separation parameter
    true_turn: float  # rad, from Gazebo
    odometry_turn: float  # rad, from the drive controller's odometry
    ekf_turn: float  # rad, from the EKF's estimate


class Run(NamedTuple):
    """A saved run of one rostest file: one line of RUNS_FILE."""

    commit: str  # of the diffbot repository
    robot: str
    with_ekf: bool
    odom_tf_counts: Dict[str, int]  # transforms odom → base_footprint by publishing node
    turn: Optional[Turn]  # of the rostest files that run test_ekf.py


class Publisher(NamedTuple):
    """A node that can publish odom → base_footprint, as one drawing of the figure shows it."""

    name: str
    lines: Sequence[Tuple[str, str]]  # under the name, each a (text, CSS class) pair
    publishes: bool
    tooltip: str


class TfCase(NamedTuple):
    """One simulation of the transform figure: with the EKF or without it."""

    name: str  # of the drawing
    heading: str  # HTML, above the drawing
    with_ekf: bool
    expected_node: str
    publishers: Sequence[Publisher]


# test_odom_tf.py in sim_drive_<model>.test and test_ekf_is_the_only_odom_tf_publisher() in
# test_ekf.py. enable_odom_tf is the drive controller's parameter (ODOM_TF_PARAMETER)
TF_CASES = (
    TfCase('lv-tf-without-ekf', 'Without the EKF, checked by <code>test_odom_tf.py</code>',
           False, CONTROLLER_NODE,
           (Publisher('drive controller',
                      ((CONTROLLER_NODE, 'mono'), ('enable_odom_tf: true', 'mono')), True,
                      f'The drive controller runs in the node {CONTROLLER_NODE} and '
                      'publishes the transform: enable_odom_tf is true'),
            Publisher('EKF', (('not started', 'note'),), False,
                      'The EKF is not started and publishes nothing'))),
    TfCase('lv-tf-with-ekf', 'With the EKF, checked by <code>test_ekf.py</code>',
           True, EKF_NODE,
           (Publisher('drive controller',
                      ((CONTROLLER_NODE, 'mono'), ('enable_odom_tf: false', 'mono')), False,
                      f'The drive controller runs in the node {CONTROLLER_NODE} and does '
                      'not publish the transform: enable_odom_tf is false'),
            Publisher('EKF', ((EKF_NODE, 'mono'),), True,
                      f'The EKF runs in the node {EKF_NODE} and publishes the transform'))),
)


def load_runs() -> List[Run]:
    """Return the saved runs of the simulation tests, one for each line of RUNS_FILE."""
    runs = []
    with open(RUNS_FILE) as runs_file:
        for line in runs_file:
            record = json.loads(line)
            printed = record.get('turn')
            turn = None
            if printed:
                turn = Turn(record['robot'], printed['controller_wheel_separation_m'],
                            printed['true_rad'], printed['odometry_rad'], printed['ekf_rad'])
            runs.append(Run(record['commit'], record['robot'], record['ekf'],
                            record['odom_tf_counts'], turn))
    return runs


def short_commit(runs: Sequence[Run]) -> str:
    """Return the short form of the one commit that all runs were made at."""
    commits = {run.commit for run in runs}
    if len(commits) != 1:
        raise ValueError(f'the runs in {RUNS_FILE} are from different commits: {commits}')
    return commits.pop()[:7]


def code(text: str) -> str:
    """Return text as an HTML code element."""
    return f'<code>{escape(text)}</code>'


def quantity(value: str, unit: str) -> str:
    """Return a value and its unit as HTML, with a space that keeps both on one line."""
    return f'{value}&nbsp;{unit}'


def mark(css_class: str, content: str = '') -> str:
    """Return the mark at the start of a list's line as an HTML span: a sample or a sign."""
    return f'<span class="{css_class}">{content}</span>'


def legend(lines: Sequence[str]) -> str:
    """Return an HTML list for below a drawing, with one item for each line of HTML."""
    items = '\n'.join(f'<li>{line}</li>' for line in lines)
    return f'<ul class="lv-legend">\n{items}\n</ul>'


def results(lines: Sequence[Tuple[str, str]]) -> str:
    """Return an HTML list of results, each a (sign, HTML) pair: ✓ for a passed check, or ''."""
    items = '\n'.join(f'<li>{mark("lv-sign", sign)}{line}</li>' for sign, line in lines)
    return f'<ul class="lv-legend lv-results">\n{items}\n</ul>'


def point(center_x: float, center_y: float, heading: float, radius: float) -> Tuple[float, float]:
    """Return the point at radius from the center in the direction of heading (rad).

    A heading of 0 points to the right, and a left turn goes counterclockwise, as seen from
    above.
    """
    return (center_x + radius * math.cos(heading), center_y - radius * math.sin(heading))


def draw_turn_arc(figure: Figure, turn: float, radius: float, css_class: str,
                  tooltip: str) -> None:
    """Add an arc from the start heading over turn (rad), with an arrowhead at its end."""
    center_x, center_y = TURN_CENTER_X, TURN_CENTER_Y
    head_angle = HEAD_LENGTH / radius  # the part of the turn that the arrowhead covers
    base_x, base_y = point(center_x, center_y, turn - head_angle, radius)
    tip_x, tip_y = point(center_x, center_y, turn, radius)
    across_x = HEAD_HALF_WIDTH * math.cos(turn - head_angle)
    across_y = -HEAD_HALF_WIDTH * math.sin(turn - head_angle)
    large_arc = 1 if turn - head_angle > math.pi else 0
    figure.elements.append(
        f'<g class="{css_class}"><title>{escape(tooltip)}</title>'
        f'<path d="M{center_x + radius},{center_y} A{radius},{radius} 0 {large_arc} 0 '
        f'{base_x:.1f},{base_y:.1f}"/>'
        f'<polygon points="{tip_x:.1f},{tip_y:.1f} {base_x + across_x:.1f},'
        f'{base_y + across_y:.1f} {base_x - across_x:.1f},{base_y - across_y:.1f}"/></g>')


def draw_robot(figure: Figure, heading: float) -> None:
    """Add the robot from above, turned to heading (rad): its body, wheels and nose line."""
    center_x, center_y = TURN_CENTER_X, TURN_CENTER_Y
    degrees = -math.degrees(heading)  # SVG turns clockwise for positive angles
    wheel_y = (center_y - BODY_RADIUS - 2, center_y + BODY_RADIUS - 2)
    wheels = ''.join(f'<rect class="wheel" x="{center_x - 7:.1f}" y="{y:.1f}" width="14" '
                     'height="4" rx="1.5"/>' for y in wheel_y)
    figure.elements.append(
        f'<g transform="rotate({degrees:.1f} {center_x} {center_y})">{wheels}'
        f'<circle class="body" cx="{center_x}" cy="{center_y}" r="{BODY_RADIUS}"/>'
        f'<path class="nose" d="M{center_x + 4},{center_y} H{center_x + BODY_RADIUS}"/></g>')


def draw_tolerance(figure: Figure, turn: Turn) -> None:
    """Add the sector in which the EKF's arc has to end: EKF_TOLERANCE around the true turn."""
    center_x, center_y = TURN_CENTER_X, TURN_CENTER_Y
    inner = EKF_RADIUS - TOLERANCE_HALF_WIDTH
    outer = EKF_RADIUS + TOLERANCE_HALF_WIDTH
    first = turn.true_turn - EKF_TOLERANCE
    last = turn.true_turn + EKF_TOLERANCE
    inner_first_x, inner_first_y = point(center_x, center_y, first, inner)
    outer_first_x, outer_first_y = point(center_x, center_y, first, outer)
    outer_last_x, outer_last_y = point(center_x, center_y, last, outer)
    inner_last_x, inner_last_y = point(center_x, center_y, last, inner)
    tooltip = (f"{turn.robot}: the EKF's turn has to end between {first:.3f} rad and "
               f'{last:.3f} rad, {EKF_TOLERANCE} rad to each side of the true turn')
    figure.elements.append(
        f'<path class="tolerance" d="M{inner_first_x:.1f},{inner_first_y:.1f} '
        f'L{outer_first_x:.1f},{outer_first_y:.1f} '
        f'A{outer},{outer} 0 0 0 {outer_last_x:.1f},{outer_last_y:.1f} '
        f'L{inner_last_x:.1f},{inner_last_y:.1f} '
        f'A{inner},{inner} 0 0 1 {inner_first_x:.1f},{inner_first_y:.1f} Z">'
        f'<title>{escape(tooltip)}</title></path>')


def turn_drawing(turn: Turn) -> Figure:
    """Return one robot's left turn from above: its three turns from the same start."""
    center_x, center_y = TURN_CENTER_X, TURN_CENTER_Y
    figure = Figure(f'lv-turn-{turn.robot.lower()}',
                    f'The left turn of the simulated {turn.robot} from above: the robot turned '
                    f"{turn.true_turn:.3f} rad, the EKF's estimate {turn.ekf_turn:.3f} rad and "
                    f'the wheel odometry {turn.odometry_turn:.3f} rad',
                    TURN_HEIGHT, TURN_WIDTH)
    figure.path(f'M{center_x},{center_y} H{center_x + START_RADIUS}', 'start-heading')
    figure.text(center_x + START_RADIUS, center_y + 14, 'start', 'note', 'end')
    draw_tolerance(figure, turn)
    draw_robot(figure, turn.true_turn)
    draw_turn_arc(figure, turn.true_turn, TRUE_RADIUS, 'true-turn',
                  f'{turn.robot}: the robot turned {turn.true_turn:.3f} rad in Gazebo')
    draw_turn_arc(figure, turn.ekf_turn, EKF_RADIUS, 'ekf-turn',
                  f"{turn.robot}: the EKF's estimate turned {turn.ekf_turn:.3f} rad")
    draw_turn_arc(figure, turn.odometry_turn, ODOMETRY_RADIUS, 'odometry-turn',
                  f'{turn.robot}: the wheel odometry turned {turn.odometry_turn:.3f} rad')

    true_x, true_y = point(center_x, center_y, turn.true_turn, TRUE_RADIUS)
    # The labels are the words of the line that test_ekf.py prints: true, odometry, EKF
    figure.text(true_x - 9, true_y + 5, f'true {turn.true_turn:.3f} rad', '', 'end')
    ekf_x, ekf_y = point(center_x, center_y, turn.ekf_turn, EKF_RADIUS)
    figure.text(ekf_x - 13, ekf_y + 4, f'EKF {turn.ekf_turn:.3f} rad', '', 'end')
    odometry_x, odometry_y = point(center_x, center_y, turn.odometry_turn, ODOMETRY_RADIUS)
    figure.text(odometry_x, odometry_y - 9, f'odometry {turn.odometry_turn:.3f} rad', '',
                'middle')
    return figure


def turn_setup(turn: Turn) -> str:
    """Return the HTML line under a robot's name: its two wheel separations."""
    robot_separation = quantity(f'{ROBOT_WHEEL_SEPARATION[turn.robot]:g}', 'm')
    controller_separation = quantity(f'{turn.controller_separation:g}', 'm')
    return (f'<p class="lv-note">Wheel separation: <i>b</i> = {robot_separation} on the '
            f'simulated robot, <i>b</i><sub>c</sub> = {controller_separation} in the drive '
            'controller</p>')


def ekf_error(turn: Turn) -> float:
    """Return how far the EKF's turn is from the true turn (rad)."""
    return abs(turn.ekf_turn - turn.true_turn)


def odometry_error(turn: Turn) -> float:
    """Return by how much the wheel odometry's turn falls behind the true turn (rad)."""
    return turn.true_turn - turn.odometry_turn


def require_passing_turn(turn: Turn) -> None:
    """Raise a ValueError if the saved turn doesn't pass test_ekf.py's two limits on a turn.

    The figure marks both checks with a ✓, which holds for a passing run only.
    """
    if ekf_error(turn) > EKF_TOLERANCE or odometry_error(turn) <= MIN_ODOMETRY_ERROR:
        raise ValueError(f"the saved run of {turn.robot} doesn't pass test_ekf.py's limits")


def turn_results(turn: Turn) -> str:
    """Return the HTML list under a robot's drawing: the two checks on its turn."""
    return results((
        ('✓', '<strong>The EKF follows:</strong> its turn is '
              f'{quantity(f"{ekf_error(turn):.3f}", "rad")} from the true turn. Allowed: '
              f'{quantity(str(EKF_TOLERANCE), "rad")}'),
        ('✓', '<strong>The wheel odometry falls behind:</strong> by '
              f'{quantity(f"{odometry_error(turn):.3f}", "rad")}. Required: more than '
              f'{quantity(str(MIN_ODOMETRY_ERROR), "rad")}'),
    ))


def turn_case(turn: Turn) -> str:
    """Return one robot's part of the turn figure: its name, setup, drawing and results."""
    parts = (band(turn.robot), turn_setup(turn), turn_drawing(turn).svg(), turn_results(turn))
    return '<div class="lv-case">\n' + '\n'.join(parts) + '\n</div>'


def write_turn_test(runs: Sequence[Run]) -> None:
    """Write the include with test_ekf.py's left turn for both robots."""
    turns = [run.turn for run in runs if run.turn]
    for turn in turns:
        require_passing_turn(turn)
    pair = '<div class="lv-pair">\n' + '\n'.join(turn_case(turn) for turn in turns) + '\n</div>'
    marks = legend((
        mark('lv-arc') + "<strong>true:</strong> the robot's turn in Gazebo, "
                         'Δ<i>θ</i><sub>true</sub>',
        mark('lv-arc lv-ekf') + "<strong>EKF:</strong> the turn of the EKF's estimate, "
                                'Δ<i>θ</i><sub>EKF</sub>',
        mark('lv-arc lv-odometry') + '<strong>odometry:</strong> the turn of the wheel '
                                     'odometry, Δ<i>θ</i><sub>odom</sub>',
        mark('lv-swatch lv-fused') + "where the EKF's arc has to end: "
                                     f'{code("EKF_TOLERANCE")}, '
                                     f'{quantity(str(EKF_TOLERANCE), "rad")}, to each side of '
                                     'the true turn',
    ))
    write_parts(
        'includes/localization-test-gazebo.html', [pair, marks],
        f'The left turn of {code("test_ekf.py")} from above, in a run at the commit '
        f'{code(short_commit(runs))}. Each arc is one turn, from the dashed start heading.',
        ALL_RULES, 'lv-grids')


def box_height(line_count: int) -> float:
    """Return the height of a box with line_count lines of text."""
    return line_count * LINE_HEIGHT + 10


def draw_lines(figure: Figure, x: float, top: float, lines: Sequence[Tuple[str, str]],
               anchor: str = 'start') -> None:
    """Add a box's lines of text below top, each a (text, CSS class) pair."""
    for index, (content, css_class) in enumerate(lines):
        figure.text(x, top + 15.5 + index * LINE_HEIGHT, content, css_class, anchor)


def draw_arrow(figure: Figure, start: Tuple[float, float], end: Tuple[float, float],
               skipped: bool = False) -> None:
    """Add a straight arrow to the right or down, dashed when the test skips this step."""
    (start_x, start_y), (end_x, end_y) = start, end
    suffix = '-skipped' if skipped else ''
    if start_y == end_y:
        base = f'{end_x - HEAD_LENGTH},{end_y - HEAD_HALF_WIDTH} ' \
               f'{end_x - HEAD_LENGTH},{end_y + HEAD_HALF_WIDTH}'
        line_end = f'{end_x - HEAD_LENGTH + 1},{end_y}'
    else:
        base = f'{end_x - HEAD_HALF_WIDTH},{end_y - HEAD_LENGTH} ' \
               f'{end_x + HEAD_HALF_WIDTH},{end_y - HEAD_LENGTH}'
        line_end = f'{end_x},{end_y - HEAD_LENGTH + 1}'
    figure.path(f'M{start_x},{start_y} L{line_end}', f'flow{suffix}')
    figure.elements.append(
        f'<polygon class="flow-head{suffix}" points="{end_x},{end_y} {base}"/>')


def draw_publisher(figure: Figure, y: float, publisher: Publisher) -> float:
    """Add a publisher's box and return its height: solid green when it publishes, else dashed."""
    height = box_height(1 + len(publisher.lines))
    css_class = 'active' if publisher.publishes else 'inactive'
    name = (f'✓ {publisher.name}', 'strong') if publisher.publishes else (publisher.name, '')
    figure.elements.append(f'<g><title>{escape(publisher.tooltip)}</title>')
    figure.rect(PUBLISHER_X, y, PUBLISHER_WIDTH, height, css_class, corner_radius=4)
    draw_lines(figure, PUBLISHER_X + 8, y, (name,) + tuple(publisher.lines))
    figure.elements.append('</g>')
    return height


def tf_drawing(case: TfCase) -> Figure:
    """Return one simulation's drawing: the two publishers, the topic /tf and the observer."""
    publisher_names = ' and '.join(publisher.name for publisher in case.publishers
                                   if publisher.publishes)
    figure = Figure(case.name,
                    f'{"With" if case.with_ekf else "Without"} the EKF, only the '
                    f'{publisher_names} publishes the transform odom → base_footprint on the '
                    f'topic /tf, from the node {case.expected_node}. The observer counts the '
                    'transforms by publishing node', 0, TF_WIDTH)
    publishers_right = PUBLISHER_X + PUBLISHER_WIDTH
    y = 2.0
    for publisher in case.publishers:
        height = draw_publisher(figure, y, publisher)
        if publisher.publishes:
            arrow_y = y + height / 2
            draw_arrow(figure, (publishers_right, arrow_y), (OBSERVER_X, arrow_y))
            figure.text((publishers_right + OBSERVER_X) / 2 - 2, arrow_y - 6, '/tf', 'mono',
                        'middle')
        y += height + PUBLISHER_GAP
    bottom = y - PUBLISHER_GAP
    figure.rect(OBSERVER_X, 2, TF_WIDTH - 2 - OBSERVER_X, bottom - 2, 'item', corner_radius=5)
    observer_lines = (('observer', 'name'), ('counts', 'note'), ('by node', 'note'))
    draw_lines(figure, (OBSERVER_X + TF_WIDTH - 2) / 2,
               (2 + bottom) / 2 - box_height(len(observer_lines)) / 2, observer_lines,
               'middle')
    figure.height = round(bottom + 2)
    return figure


def require_only_publisher(run: Run, node: str) -> None:
    """Raise a ValueError if another node than node published the transform in the saved run.

    The figure marks the saved run with a ✓, which holds for a passing run only.
    """
    if list(run.odom_tf_counts) != [node]:
        raise ValueError(f'the saved run of {run.robot} has other publishers than {node}: '
                         f'{run.odom_tf_counts}')


def counted_transforms(case: TfCase, runs: Sequence[Run]) -> str:
    """Return what the observer counted in the simulation's saved runs, one count per robot."""
    counts: List[str] = []
    for run in runs:
        if run.with_ekf == case.with_ekf:
            unit = 'for' if counts else 'transforms for'
            counts.append(f'{run.odom_tf_counts[case.expected_node]} {unit} {run.robot}')
    return ' and '.join(counts)


def tf_results(case: TfCase, runs: Sequence[Run]) -> str:
    """Return the HTML list under a simulation's drawing: the expected node and the counts."""
    return results((
        ('', f'<strong>Expected:</strong> only {code(case.expected_node)} publishes the '
             'transform'),
        ('✓', f'<strong>Saved run:</strong> the observer counted '
              f'{counted_transforms(case, runs)}, all from {code(case.expected_node)}'),
    ))


def write_odom_tf_test(runs: Sequence[Run]) -> None:
    """Write the include of who publishes odom → base_footprint with and without the EKF."""
    parts = []
    for case in TF_CASES:
        for run in runs:
            if run.with_ekf == case.with_ekf:
                require_only_publisher(run, case.expected_node)
        parts += [f'<p class="lv-band">{case.heading}</p>', tf_drawing(case).svg(),
                  tf_results(case, runs)]
    parts.append(legend((
        mark('lv-swatch lv-fused', '✓') + 'publishes the transform '
                                          f'{code("odom → base_footprint")} on the topic '
                                          f'{code("/tf")}',
        mark('lv-swatch lv-inactive') + "doesn't publish the transform",
    )))
    write_parts(
        'includes/localization-test-odom-tf.html', parts,
        f'Who publishes the transform {code("odom → base_footprint")}. The observer is '
        f'{code("OdomTfObserver")} in the test node. The counts are from a run at the commit '
        f'{code(short_commit(runs))}.',
        ALL_RULES, 'lv-large')


def switch_flow() -> Figure:
    """Return the drawing of how test_ekf_switch.py works, from the launch file to the checks."""
    figure = Figure('lv-switch-flow',
                    'How test_ekf_switch.py works: load() gives a launch file and its '
                    "arguments to roslaunch's loader, load_config_default(). The loader "
                    'returns what roslaunch would start, which the test keeps as a '
                    'LaunchConfiguration with the nodes, the parameters and a description. '
                    "The test's assertions look at this result. The ROS master and the "
                    'nodes are not started', 0, FLOW_WIDTH)
    width = FLOW_WIDTH - 2 * FLOW_X
    middle = FLOW_WIDTH / 2
    # The test's own path runs straight down on the left, above the box of the assertions
    path_x = FLOW_X + ASSERTIONS_WIDTH / 2
    skipped_x = FLOW_X + ASSERTIONS_WIDTH + 12
    skipped_width = FLOW_WIDTH - FLOW_X - skipped_x
    skipped_middle = skipped_x + skipped_width / 2

    y = 2.0
    launch_lines = (('launch file with arguments', 'name'), (LOADED_EXAMPLE, 'mono'))
    figure.rect(FLOW_X, y, width, box_height(len(launch_lines)), 'item', corner_radius=5)
    draw_lines(figure, middle, y, launch_lines, 'middle')
    y += box_height(len(launch_lines))
    draw_arrow(figure, (path_x, y), (path_x, y + FLOW_GAP))
    figure.text(path_x + 9, y + 14, 'load()', 'mono')

    y += FLOW_GAP
    loader_lines = (("roslaunch's loader", 'name'), ('load_config_default()', 'mono'),
                    ('reads the file and its includes', 'note'))
    figure.rect(FLOW_X, y, width, box_height(len(loader_lines)), 'item', corner_radius=5)
    draw_lines(figure, middle, y, loader_lines, 'middle')
    y += box_height(len(loader_lines))
    draw_arrow(figure, (path_x, y), (path_x, y + FLOW_GAP))
    figure.text(path_x + 9, y + 14, 'what roslaunch would start', 'note')

    y += FLOW_GAP
    result_height = box_height(1) + FLOW_CELL_HEIGHT + 6
    figure.rect(FLOW_X, y, width, result_height, 'item', corner_radius=5)
    draw_lines(figure, middle, y, (('LaunchConfiguration', 'mono'),), 'middle')
    cells_y = y + box_height(1)
    cell_gap = (width - 16 - 3 * FLOW_CELL_WIDTH) / 2
    for index, field in enumerate(('nodes', 'parameters', 'description')):
        cell_x = FLOW_X + 8 + index * (FLOW_CELL_WIDTH + cell_gap)
        figure.rect(cell_x, cells_y, FLOW_CELL_WIDTH, FLOW_CELL_HEIGHT, 'cell', corner_radius=3)
        figure.text(cell_x + FLOW_CELL_WIDTH / 2, cells_y + FLOW_CELL_HEIGHT / 2 + 4, field,
                    'mono', 'middle')
    y += result_height
    draw_arrow(figure, (path_x, y), (path_x, y + FLOW_GAP))
    draw_arrow(figure, (skipped_middle, y), (skipped_middle, y + FLOW_GAP), skipped=True)

    y += FLOW_GAP
    assertion_lines = (("the test's assertions", 'strong'), ('look at these fields', 'note'))
    figure.rect(FLOW_X, y, ASSERTIONS_WIDTH, box_height(len(assertion_lines)), 'item',
                corner_radius=5)
    draw_lines(figure, path_x, y, assertion_lines, 'middle')
    skipped_lines = (('ROS master, nodes', ''), ('not started', 'note'))
    figure.rect(skipped_x, y, skipped_width, box_height(len(skipped_lines)), 'inactive',
                corner_radius=5)
    draw_lines(figure, skipped_middle, y, skipped_lines, 'middle')
    figure.height = round(y + box_height(len(assertion_lines)) + 2)
    return figure


def write_switch_test() -> None:
    """Write the include of how test_ekf_switch.py reads a launch file without starting it."""
    parts = [
        switch_flow().svg(),
        legend((
            mark('lv-swatch lv-inactive') + 'what roslaunch would do next and the test leaves '
                                            'out',
        )),
    ]
    write_parts(
        'includes/localization-test-switch.html', parts,
        f'How {code("test_ekf_switch.py")} works, from top to bottom, for one of its launch '
        'files.',
        ALL_RULES, 'lv-large')


def main() -> None:
    """Write the three includes of the test pages."""
    runs = load_runs()
    write_turn_test(runs)
    write_switch_test()
    write_odom_tf_test(runs)


if __name__ == '__main__':
    main()
