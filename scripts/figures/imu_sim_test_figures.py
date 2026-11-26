"""Generate the timelines on docs/packages/imu/simulation/tests.md.

It writes three includes, as inline SVG that follows the site's light and dark colors. Each one
shows what one test of the simulated IMU does over simulation time:

- includes/imu-sim-test-at-rest.html: the test at rest. The test sends the stop command for
  SETTLE and MEASURE, the robot stands still, and the test keeps the IMU's messages of MEASURE.
- includes/imu-sim-test-left-turn.html: the left turn. The test sends the turn command for
  SPEED_UP and MEASURE, the robot turns left on the spot, and the test keeps the IMU's messages
  of MEASURE.
- includes/imu-sim-test-reset.html: the reset test, in two panels with the same axes. The stamps
  of the IMU's samples before and after the reset, with the robots' IMU plugin and with a
  plugin that keeps the time from before the reset. The panels are an idealized example: the
  reset comes at exactly RUN_BEFORE_RESET, and the IMU publishes exactly IMU_RATE samples per
  second. The test itself only needs FRESH_SAMPLE_COUNT samples after the reset with advancing
  stamps below RECOVERY. Both axes end a fifth of RUN_BEFORE_RESET after the time at which a
  plugin that keeps the old time publishes again, so that this sample lies inside the plot for
  any RUN_BEFORE_RESET.

The durations, rates and tolerances below are the constants of diffbot_gazebo/test/test_imu.py
and test_imu_reset.py in the diffbot repository, at the commit that mkdocs.yml pins as
imu_sim_src_raw. Change them together with the pin. The drawings show each test's plan, not a
recorded run: the robot's headings follow from the commanded turn rate.

The two series colors are the first two slots of the categorical palette that the Wheel Dynamics
figures use. They were checked for color vision deficiencies against the site's light and dark
backgrounds, with their own steps for the dark scheme. Every series also has a legend entry, and
the page has the numbers as text.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_sim_test_figures.py
"""

import math
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

from board_drawings import Drawing
from board_drawings import figure_element
from board_drawings import PanelWidths
from board_drawings import style_element
from board_drawings import svg_element
from board_drawings import svg_subscript

# The first line of each include: the script that writes it
INCLUDE_COMMENT = '<!-- Written by scripts/figures/imu_sim_test_figures.py -->'


# test_imu.py
COMMAND_RATE = 20  # Hz, how often the test sends its command
IMU_RATE = 50.0  # Hz, the test's RATE
RATE_TOLERANCE = 0.9
SETTLE = 1.0  # s of simulation time
SPEED_UP = 1.0  # s of simulation time
MEASURE = 2.0  # s of simulation time
GRAVITY = 9.81  # m/s²
GRAVITY_TOLERANCE = 0.5  # m/s²
TURN_RATE = 0.5  # rad/s
TURN_RATE_TOLERANCE = 0.15  # rad/s
# How far the mean yaw rate may lie from zero at rest: a tenth of TURN_RATE
YAW_RATE_AT_REST_TOLERANCE = TURN_RATE / 10  # rad/s
# test_imu_reset.py
RUN_BEFORE_RESET = 3.0  # s of simulation time
RECOVERY = 1.0  # s of simulation time
FRESH_SAMPLE_COUNT = 10
RECOVERY_TIMEOUT = 30.0  # s of wall time
# Gazebo's gravity in the robots' worlds, SDF's default
GAZEBO_GRAVITY = 9.8  # m/s²

LINE_HEIGHT = 14

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
FG_LIGHTER = 'var(--md-default-fg-color--lighter)'
FG_LIGHTEST = 'var(--md-default-fg-color--lightest)'
BG = 'var(--md-default-bg-color)'
RULES = (
    ('.md-typeset figure.tl-figure', 'margin:1em auto;width:100%;max-width:none'),
    ('.tl-figure .tl-panels', 'display:flex;flex-wrap:wrap;justify-content:center;'
                              'align-items:flex-start;gap:.8rem 1.6rem'),
    ('.tl-figure .tl-fig', 'flex:1 1 var(--tl-basis,300px);max-width:var(--tl-max,480px)'),
    ('.md-typeset .tl-figure figcaption', 'max-width:36rem'),
    ('.tl-fig', 'width:100%;height:auto;--tl-command:#2a78d6;--tl-imu:#eb6834;'
                'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .tl-fig', '--tl-command:#3987e5;--tl-imu:#d95926'),
    ('.tl-fig text', f'fill:{FG};font-size:12px'),
    ('.tl-fig .row-title', 'font-weight:700'),
    ('.tl-fig .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                      'font-size:11.5px'),
    ('.tl-fig .note', f'font-size:11.5px;fill:{FG_LIGHT}'),
    ('.tl-fig .sub', 'font-size:10px'),
    ('.tl-fig .halo', f'paint-order:stroke;stroke:{BG};stroke-width:3px;stroke-linejoin:round'),
    ('.tl-fig .strong', 'font-weight:700'),
    ('.tl-fig .grid', f'fill:none;stroke:{FG_LIGHTEST};stroke-width:1'),
    ('.tl-fig .axis', f'fill:none;stroke:{FG_LIGHTER};stroke-width:1'),
    ('.tl-fig .span', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    ('.tl-fig .command-wash', 'fill:var(--tl-command);fill-opacity:.16;stroke:none'),
    ('.tl-fig .command-line', 'fill:none;stroke:var(--tl-command);stroke-width:2;'
                              'stroke-linejoin:round;stroke-linecap:round'),
    ('.tl-fig .command-dot', f'fill:var(--tl-command);stroke:{BG};stroke-width:2'),
    ('.tl-fig .imu-wash', 'fill:var(--tl-imu);fill-opacity:.16;stroke:none'),
    ('.tl-fig .imu-bar', 'fill:var(--tl-imu);stroke:none'),
    ('.tl-fig .imu-line', 'fill:none;stroke:var(--tl-imu);stroke-width:3;stroke-linecap:round'),
    ('.tl-fig .imu-dot', f'fill:var(--tl-imu);stroke:{BG};stroke-width:2'),
    ('.tl-fig .clock-line', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2'),
    ('.tl-fig .reference-line', f'fill:none;stroke:{FG};stroke-width:1.6'),
    ('.tl-fig .fresh', f'fill:{FG_LIGHTEST};stroke:none'),
    ('.tl-fig .body', f'fill:{FG_LIGHTEST};stroke:{FG_LIGHT};stroke-width:1'),
    ('.tl-fig .wheel', f'fill:{FG_LIGHT};stroke:none'),
    ('.tl-fig .heading', f'fill:none;stroke:{FG};stroke-width:1.6;stroke-linecap:round'),
    ('.tl-fig .heading-head', f'fill:{FG};stroke:none'),
    ('.tl-fig .start-heading', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    ('.tl-fig .turned', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2'),
    ('.tl-fig .turned-head', f'fill:{FG_LIGHT};stroke:none'),
)


class Scale(NamedTuple):
    """Map a value range onto a range of SVG coordinates."""

    first_value: float
    last_value: float
    first_position: float
    last_position: float

    def position(self, value: float) -> float:
        """Return the SVG coordinate of a value, rounded to a tenth of a pixel."""
        share = (value - self.first_value) / (self.last_value - self.first_value)
        position = self.first_position + share * (self.last_position - self.first_position)
        return round(position, 1)


class Figure:
    """One timeline: its name, size, accessible texts and the elements of its drawing.

    The figure draws with the shared Drawing and adds what the timelines have of their own:
    groups with a transform.
    """

    def __init__(self, name: str, width: float, height: float, title: str,
                 description: str) -> None:
        """Initialize the figure metadata and its drawing."""
        self.name, self.width, self.height = name, width, height
        self.title, self.description = title, description
        self.drawing = Drawing()

    @property
    def elements(self) -> List[str]:
        """Return the SVG elements of the drawing, in the order they are drawn."""
        return self.drawing.elements

    def rectangle(self, x: float, y: float, width: float, height: float, css_class: str,
                  radius: float = 0) -> None:
        """Add a rectangle with its upper left corner at (x, y)."""
        self.drawing.rect(x, y, width, height, css_class, radius)

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of text at the requested anchor.

        anchor is start, middle or end.
        """
        self.drawing.text(x, y, text, css_class, anchor)

    def text_markup(self, x: float, y: float, markup: str, css_class: str = '') -> None:
        """Add one line of text from SVG markup, such as a symbol with a subscript."""
        self.drawing.text_markup(x, y, markup, css_class)

    def path(self, path: str, css_class: str) -> None:
        """Add a line along an SVG path."""
        self.drawing.line(path, css_class)

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle around (x, y)."""
        self.drawing.circle(x, y, radius, css_class)

    def group(self, transform: str, elements: Sequence[str]) -> None:
        """Add elements in a group with a transform."""
        self.elements.append(f'<g transform="{transform}">' + ''.join(elements) + '</g>')

    def svg(self) -> str:
        """Return the drawing as an SVG element without markers."""
        return svg_element(self.name, 'tl-fig', (self.width, self.height), self.title,
                           self.elements, description=self.description, markers=())


class Row(NamedTuple):
    """A row of a timeline: the y position of its upper edge and its height."""

    top: float
    height: float

    @property
    def bottom(self) -> float:
        """Return the y position of the row's lower edge."""
        return self.top + self.height


# The two drive tests: the layout that both timelines share
DRIVE_WIDTH = 360
DRIVE_HEIGHT = 394
# Simulation time since the test's first command, with room after the last phase
DRIVE_TIME = Scale(0.0, 3.3, 40.0, 348.0)
COMMAND_BAR = Row(top=22, height=20)
PHASES_Y = 54
ROBOT_TITLE_Y = 90
ROBOT_Y = 126
ROBOT_LABEL_Y = 164
CHART_TITLE_Y = 186
CHART = Row(top=196, height=60)
LEGEND_Y = 274
MESSAGES_TITLE_Y = 314
MESSAGES_BAR = Row(top=322, height=8)
AXIS_Y = 362


class Phase(NamedTuple):
    """One phase of a drive test: the constant's name in the test and its duration."""

    name: str
    seconds: float


class Robots(NamedTuple):
    """The row with the robot from above: its title, the robot at each time and notes."""

    title: str
    robots: Sequence['Robot']
    notes: Sequence[Tuple[float, Sequence[str]]]  # (time, lines of text)


class Robot(NamedTuple):
    """The robot from above at one time: its heading and the label below it."""

    time: float
    heading: float  # rad, counterclockwise from the heading at the test's start
    label: str
    anchor: str = 'middle'


class Band(NamedTuple):
    """The means of a reading that a drive test accepts: the lowest and the highest one."""

    lowest: float
    highest: float


class Reading(NamedTuple):
    """The chart of the one reading that a drive test checks."""

    title: str  # SVG markup
    values: Scale
    ticks: Sequence[Tuple[float, str]]
    accepted: Band
    band_label: str  # what the band is, written inside it
    band_label_offset: float  # from the middle of the band down to the label's baseline
    legend: Sequence[Tuple[str, str]]  # (CSS class of the key, text)


def draw_guide(figure: Figure, time: float) -> None:
    """Add a vertical guide at a time, in the rows without a label that the guide would cross."""
    x = DRIVE_TIME.position(time)
    figure.path(f'M{x},{COMMAND_BAR.bottom + 4} V{PHASES_Y + 4} M{x},{CHART.top - 4} '
                f'V{CHART.bottom} M{x},{MESSAGES_BAR.top - 4} V{MESSAGES_BAR.bottom + 2} '
                f'M{x},{AXIS_Y - 4} V{AXIS_Y}', 'grid')


def draw_command(figure: Figure, text: str, phases: Sequence[Phase], stops: bool) -> None:
    """Add the command that the test sends, the phases below it and the guides at their ends."""
    figure.text(8, 14, f'The test sends this command, {COMMAND_RATE} times per second',
                'row-title')
    top, height = COMMAND_BAR.top, COMMAND_BAR.height
    start = DRIVE_TIME.position(0.0)
    end = DRIVE_TIME.position(sum(phase.seconds for phase in phases))
    figure.rectangle(start, top, end - start, height, 'command-wash', 4)
    figure.text((start + end) / 2, top + 14, text, anchor='middle')
    if stops:
        figure.circle(end, top + height / 2, 4.5, 'command-dot')
        figure.text(end + 8, top + 14, 'stop', 'note')
    phase_start = 0.0
    for phase in phases:
        left = DRIVE_TIME.position(phase_start)
        right = DRIVE_TIME.position(phase_start + phase.seconds)
        draw_guide(figure, phase_start)
        figure.path(f'M{left + 3},{PHASES_Y - 4} V{PHASES_Y + 4} M{left + 3},{PHASES_Y} '
                    f'H{right - 3} M{right - 3},{PHASES_Y - 4} V{PHASES_Y + 4}', 'span')
        figure.text((left + right) / 2, PHASES_Y + 16, f'{phase.name}, {phase.seconds:g} s',
                    'mono', 'middle')
        phase_start += phase.seconds
    draw_guide(figure, phase_start)


def robot_elements(heading: float) -> List[str]:
    """Return the robot from above around the origin, turned by heading (rad)."""
    degrees = round(math.degrees(heading), 1)
    robot = ('<rect class="wheel" x="-5" y="-15" width="13" height="4" rx="1.5"/>'
             '<rect class="wheel" x="-5" y="11" width="13" height="4" rx="1.5"/>'
             '<rect class="body" x="-14" y="-11" width="28" height="22" rx="4"/>'
             '<path class="heading" d="M0,0 H17"/>'
             '<path class="heading-head" d="M23,0 L15,-4 L15,4 z"/>')
    # SVG's y axis points down, so a counterclockwise turn is a negative rotation
    elements = [f'<g transform="rotate({-degrees})">{robot}</g>']
    if heading > 0.0:
        radius = 29
        end_x = round(radius * math.cos(heading), 1)
        end_y = round(-radius * math.sin(heading), 1)
        elements.append('<path class="start-heading" d="M0,0 H34"/>')
        elements.append(f'<path class="turned" d="M{radius},0 A{radius},{radius} 0 0 0 '
                        f'{end_x},{end_y}"/>')
        elements.append(f'<path class="turned-head" transform="translate({end_x},{end_y}) '
                        f'rotate({-degrees - 90})" d="M3,0 L-4,-3.5 L-4,3.5 z"/>')
    return elements


def draw_robots(figure: Figure, row: Robots) -> None:
    """Add the row's title, the robot from above at each time, and the notes at their times."""
    figure.text(8, ROBOT_TITLE_Y, row.title, 'row-title')
    for robot in row.robots:
        x = DRIVE_TIME.position(robot.time)
        figure.group(f'translate({x},{ROBOT_Y})', robot_elements(robot.heading))
        label_x = x if robot.anchor == 'middle' else DRIVE_WIDTH - 6
        figure.text(label_x, ROBOT_LABEL_Y, robot.label, 'note', robot.anchor)
    for time, lines in row.notes:
        for index, line in enumerate(lines):
            figure.text(DRIVE_TIME.position(time), ROBOT_Y - 3 + index * LINE_HEIGHT, line,
                        'note', 'middle')


def draw_reading(figure: Figure, reading: Reading, measure_start: float) -> Scale:
    """Add the reading's chart with its accepted band and legend, and return its value scale."""
    figure.text_markup(8, CHART_TITLE_Y, reading.title, 'row-title')
    left, right = DRIVE_TIME.first_position, DRIVE_TIME.last_position
    for value, label in reading.ticks:
        y = reading.values.position(value)
        figure.path(f'M{left},{y} H{right}', 'grid')
        figure.text(left - 6, y + 4, label, 'note', 'end')
    band_left = DRIVE_TIME.position(measure_start)
    band_right = DRIVE_TIME.position(measure_start + MEASURE)
    band_top = reading.values.position(reading.accepted.highest)
    band_bottom = reading.values.position(reading.accepted.lowest)
    figure.rectangle(band_left, band_top, band_right - band_left, band_bottom - band_top,
                     'imu-wash')
    figure.text(band_left - 4, band_bottom + 4, f'{reading.accepted.lowest:.2f}', 'note', 'end')
    figure.text(band_left - 4, band_top + 4, f'{reading.accepted.highest:.2f}', 'note', 'end')
    figure.text((band_left + band_right) / 2,
                (band_top + band_bottom) / 2 + reading.band_label_offset, reading.band_label,
                'note', 'middle')
    for index, (key_class, text) in enumerate(reading.legend):
        y = LEGEND_Y + index * LINE_HEIGHT
        if key_class == 'imu-wash':
            figure.rectangle(left, y - 9, 16, 10, key_class)
        else:
            figure.path(f'M{left},{y - 4} H{left + 16}', key_class)
        figure.text(left + 22, y, text, 'note')
    return reading.values


def draw_messages(figure: Figure, measure_start: float, count_rule: str) -> None:
    """Add the IMU's messages, the ones the test keeps, and the time axis below them.

    count_rule says how many of the kept messages the test needs.
    """
    figure.text(8, MESSAGES_TITLE_Y, f'The IMU publishes, {IMU_RATE:g} messages per second',
                'row-title')
    top, height = MESSAGES_BAR.top, MESSAGES_BAR.height
    left, right = DRIVE_TIME.first_position, DRIVE_TIME.last_position
    figure.rectangle(left, top, right - left, height, 'imu-wash', 4)
    kept_left = DRIVE_TIME.position(measure_start)
    kept_right = DRIVE_TIME.position(measure_start + MEASURE)
    figure.rectangle(kept_left, top, kept_right - kept_left, height, 'imu-bar', 4)
    figure.text((kept_left + kept_right) / 2, top - 4 + height + 16,
                f'the test keeps these, about {MEASURE * IMU_RATE:g}', 'note', 'middle')
    figure.text((kept_left + kept_right) / 2, top - 4 + height + 16 + LINE_HEIGHT, count_rule,
                'note', 'middle')
    figure.path(f'M{left},{AXIS_Y} H{right}', 'axis')
    for second in range(4):
        figure.text(DRIVE_TIME.position(second), AXIS_Y + 14, str(second), 'note', 'middle')
    figure.text((left + right) / 2, AXIS_Y + 28,
                'simulation time since the test’s first command, in s', 'note', 'middle')


def fewest_messages_in_measure() -> int:
    """Return the fewest messages of MEASURE with which a test of test_imu.py passes the rate.

    Both tests need a rate above RATE_TOLERANCE times its RATE, so more messages than that rate
    gives in MEASURE.
    """
    return math.floor(RATE_TOLERANCE * IMU_RATE * MEASURE) + 1


def at_rest_figure() -> Figure:
    """Return the timeline of the test at rest."""
    figure = Figure(
        'tl-at-rest', DRIVE_WIDTH, DRIVE_HEIGHT, 'The test at rest over simulation time',
        f'The test sends the stop command {COMMAND_RATE} times per second, for SETTLE, '
        f'{SETTLE:g} s, and then for MEASURE, {MEASURE:g} s. The robot stands still. The test '
        f'keeps the IMU’s messages of MEASURE, about {MEASURE * IMU_RATE:g}, and needs at least '
        f'{fewest_messages_in_measure()}. The mean of their readings on the upward axis must lie '
        f'between {GRAVITY - GRAVITY_TOLERANCE:.2f} and {GRAVITY + GRAVITY_TOLERANCE:.2f} m/s², '
        f'and the mean of their yaw rates within {YAW_RATE_AT_REST_TOLERANCE:g} rad/s of 0 rad/s. '
        f'A black line marks a reference value, gravity in the simulation, {GAZEBO_GRAVITY} '
        'm/s². It is not a reading.')
    draw_command(figure, 'stand still: all velocities 0',
                 [Phase('SETTLE', SETTLE), Phase('MEASURE', MEASURE)], stops=False)
    draw_robots(figure, Robots(
        title='The robot, seen from above',
        robots=[Robot(SETTLE + MEASURE / 2, 0.0, 'stands still')],
        notes=[(SETTLE / 2, ['may still brake', 'after the left turn'])]))
    reading = Reading(
        title=('Reading on the upward axis, ' + svg_subscript('f', 'z', restore_baseline=True)
               + ', in m/s²'),
        values=Scale(9.0, 10.6, CHART.bottom, CHART.top),
        ticks=[(9.0, '9'), (10.0, '10')],
        accepted=Band(lowest=GRAVITY - GRAVITY_TOLERANCE, highest=GRAVITY + GRAVITY_TOLERANCE),
        band_label=f'the mean over {MEASURE:g} s must lie here',
        band_label_offset=14,
        legend=[('reference-line', f'reference value: gravity in the simulation, '
                                   f'{GAZEBO_GRAVITY} m/s²'),
                ('imu-wash', f'accepted mean of the IMU’s readings: {GRAVITY} ± '
                             f'{GRAVITY_TOLERANCE}')])
    values = draw_reading(figure, reading, SETTLE)
    figure.path(f'M{DRIVE_TIME.first_position},{values.position(GAZEBO_GRAVITY)} '
                f'H{DRIVE_TIME.last_position}', 'reference-line')
    draw_messages(figure, SETTLE,
                  f'pass rule: at least {fewest_messages_in_measure()} arrivals in {MEASURE:g} s')
    return figure


def left_turn_figure() -> Figure:
    """Return the timeline of the left turn."""
    turn_end = SPEED_UP + MEASURE
    figure = Figure(
        'tl-left-turn', DRIVE_WIDTH, DRIVE_HEIGHT, 'The left turn over simulation time',
        f'The test sends a turn command of {TURN_RATE} rad/s about the upward axis, '
        f'{COMMAND_RATE} times per second, for SPEED_UP, {SPEED_UP:g} s, and then for MEASURE, '
        f'{MEASURE:g} s. Then it sends one stop command. The robot turns left on the spot, by '
        f'about {TURN_RATE * SPEED_UP:g} rad after SPEED_UP and about {TURN_RATE * turn_end:g} '
        f'rad after MEASURE. These headings are ideal: they follow from the command, not from '
        f'a run. The test keeps the IMU’s messages of MEASURE, about {MEASURE * IMU_RATE:g}, '
        f'and needs at least {fewest_messages_in_measure()}. The mean of their yaw rates must '
        'lie between '
        f'{TURN_RATE - TURN_RATE_TOLERANCE:.2f} and {TURN_RATE + TURN_RATE_TOLERANCE:.2f} rad/s.')
    draw_command(figure, f'turn left: angular.z = {TURN_RATE} rad/s',
                 [Phase('SPEED_UP', SPEED_UP), Phase('MEASURE', MEASURE)], stops=True)
    robots = [Robot(0.0, 0.0, 'start')]
    for time, anchor in ((SPEED_UP, 'middle'), (turn_end, 'end')):
        turned = TURN_RATE * time
        robots.append(Robot(time, turned, f'about {turned:g} rad, {math.degrees(turned):.0f}°',
                            anchor))
    draw_robots(figure, Robots(title='The robot from above: ideal headings from the command',
                               robots=robots, notes=[]))
    reading = Reading(
        title=('Turn rate about the upward axis, '
               + svg_subscript('ω', 'z', restore_baseline=True) + ', in rad/s'),
        values=Scale(0.0, 0.7, CHART.bottom, CHART.top),
        ticks=[(0.0, '0'), (TURN_RATE, f'{TURN_RATE}')],
        accepted=Band(lowest=TURN_RATE - TURN_RATE_TOLERANCE,
                      highest=TURN_RATE + TURN_RATE_TOLERANCE),
        band_label=f'the mean over {MEASURE:g} s must lie here',
        band_label_offset=11,
        legend=[('command-line', f'the test’s command: {TURN_RATE} rad/s'),
                ('imu-wash', f'accepted mean of the IMU’s yaw rates: {TURN_RATE} ± '
                             f'{TURN_RATE_TOLERANCE}')])
    values = draw_reading(figure, reading, SPEED_UP)
    start, end = DRIVE_TIME.position(0.0), DRIVE_TIME.position(turn_end)
    figure.path(f'M{start},{values.position(0.0)} V{values.position(TURN_RATE)} H{end} '
                f'V{values.position(0.0)} H{DRIVE_TIME.last_position}', 'command-line')
    draw_messages(figure, SPEED_UP,
                  f'pass rule: at least {fewest_messages_in_measure()} arrivals in {MEASURE:g} s')
    return figure


# The reset test: the layout that both panels share
RESET_WIDTH = 300
RESET_HEIGHT = 262
# The plot's edges on the drawing
RESET_PLOT_LEFT = 34.0
RESET_PLOT_RIGHT = 292.0
RESET_PLOT_BOTTOM = 190.0
RESET_PLOT_TOP = 60.0
IMU_PERIOD = 1.0 / IMU_RATE


def run_after_reset() -> float:
    """Return how long the panels follow the simulation after the reset, in s.

    A plugin that keeps the old time publishes again RUN_BEFORE_RESET and one period after the
    reset. The panels show a fifth of RUN_BEFORE_RESET more, so that this sample and the
    samples after it lie inside the plot.
    """
    return 1.2 * RUN_BEFORE_RESET


def reset_run() -> Scale:
    """Return the horizontal scale: how long the simulation has run, in s."""
    return Scale(0.0, RUN_BEFORE_RESET + run_after_reset(), RESET_PLOT_LEFT, RESET_PLOT_RIGHT)


def reset_stamp() -> Scale:
    """Return the vertical scale: the stamps of the IMU's samples, in s."""
    return Scale(0.0, run_after_reset(), RESET_PLOT_BOTTOM, RESET_PLOT_TOP)


def run_axis_ticks() -> Tuple[float, float, float]:
    """Return the run times that the horizontal axis numbers: the start, the reset and twice it."""
    return (0.0, RUN_BEFORE_RESET, 2 * RUN_BEFORE_RESET)


def draw_reset_heading(figure: Figure, heading: str) -> None:
    """Add a reset panel's heading and, below it, what the vertical axis shows."""
    figure.text(8, 14, heading, 'row-title')
    figure.text(8, 32, 'Stamp of the IMU’s samples, in s of simulation time', 'note')


def draw_fresh_region(figure: Figure) -> None:
    """Add the region in which a sample counts as fresh: after the reset, below RECOVERY."""
    run, stamp = reset_run(), reset_stamp()
    reset_x = run.position(RUN_BEFORE_RESET)
    figure.rectangle(reset_x, stamp.position(RECOVERY), run.last_position - reset_x,
                     stamp.position(0.0) - stamp.position(RECOVERY), 'fresh')


def draw_stamp_grid(figure: Figure) -> None:
    """Add a horizontal grid line with its stamp at 0, RECOVERY and RUN_BEFORE_RESET."""
    run, stamp = reset_run(), reset_stamp()
    for value in (0.0, RECOVERY, RUN_BEFORE_RESET):
        y = stamp.position(value)
        figure.path(f'M{run.first_position},{y} H{run.last_position}', 'grid')
        figure.text(run.first_position - 6, y + 4, f'{value:g}', 'note', 'end')


def draw_run_axis(figure: Figure) -> None:
    """Add the horizontal axis as a line at stamp 0."""
    run = reset_run()
    figure.path(f'M{run.first_position},{reset_stamp().position(0.0)} H{run.last_position}',
                'axis')


def draw_reset_guide(figure: Figure) -> None:
    """Add a vertical line where the test resets the simulation, with its label."""
    stamp = reset_stamp()
    reset_x = reset_run().position(RUN_BEFORE_RESET)
    figure.path(f'M{reset_x},{stamp.position(0.0)} V{stamp.last_position - 8}', 'span')
    for index, line in enumerate(('the test resets', 'the simulation')):
        figure.text(reset_x - 6, stamp.last_position - 3 + index * LINE_HEIGHT, line, 'note',
                    'end')


def draw_run_axis_labels(figure: Figure) -> None:
    """Add the horizontal axis's numbers and, below them, what the axis shows."""
    run = reset_run()
    axis_y = reset_stamp().position(0.0)
    for seconds in run_axis_ticks():
        figure.text(run.position(seconds), axis_y + 14, f'{seconds:g}', 'note', 'middle')
    figure.text((run.first_position + run.last_position) / 2, axis_y + 28,
                'seconds that the simulation has run', 'note', 'middle')


def draw_reset_legend(figure: Figure) -> None:
    """Add the legend below the horizontal axis: Gazebo's clock and the IMU's samples."""
    left = RESET_PLOT_LEFT
    axis_y = reset_stamp().position(0.0)
    figure.path(f'M{left},{axis_y + 44} H{left + 16}', 'clock-line')
    figure.text(left + 22, axis_y + 48, 'Gazebo’s clock', 'note')
    figure.path(f'M{left + 118},{axis_y + 44} H{left + 134}', 'imu-line')
    figure.text(left + 140, axis_y + 48, 'the IMU’s samples', 'note')


def draw_reset_chart(figure: Figure, heading: str) -> None:
    """Add what both reset panels share: the chart without the clock and the samples.

    The parts lie on top of each other in this order, so the axis and the reset's guide cover
    the fresh region and the grid.
    """
    draw_reset_heading(figure, heading)
    draw_fresh_region(figure)
    draw_stamp_grid(figure)
    draw_run_axis(figure)
    draw_reset_guide(figure)
    draw_run_axis_labels(figure)
    draw_reset_legend(figure)


def clock_path(first_run: float, last_run: float, clock_start: float) -> str:
    """Return the path of a clock that started at clock_start, between two run times."""
    run, stamp = reset_run(), reset_stamp()
    return (f'M{run.position(first_run)},{stamp.position(first_run - clock_start)} '
            f'L{run.position(last_run)},{stamp.position(last_run - clock_start)}')


def first_stamp_after_reset(silent_until: Optional[float]) -> float:
    """Return the stamp of the IMU's first sample after the reset, in s.

    silent_until is the stamp up to which the plugin publishes nothing after the reset. None
    means that the plugin publishes from its first period on.
    """
    return IMU_PERIOD if silent_until is None else silent_until + IMU_PERIOD


def draw_clock_and_samples(figure: Figure, silent_until: Optional[float]) -> None:
    """Add Gazebo's clock and the IMU's samples, before and after the reset."""
    end = RUN_BEFORE_RESET + run_after_reset()
    figure.path(clock_path(0.0, RUN_BEFORE_RESET, 0.0)
                + f' V{reset_stamp().position(0.0)} '
                + clock_path(RUN_BEFORE_RESET, end, RUN_BEFORE_RESET).replace('M', 'L', 1),
                'clock-line')
    figure.path(clock_path(IMU_PERIOD, RUN_BEFORE_RESET, 0.0), 'imu-line')
    first_run = RUN_BEFORE_RESET + first_stamp_after_reset(silent_until)
    figure.path(clock_path(first_run, end, RUN_BEFORE_RESET), 'imu-line')


def draw_sample_marker(figure: Figure, stamp_after_reset: float) -> Tuple[float, float]:
    """Add a dot at the sample with a stamp after the reset, and return the dot's position."""
    x = reset_run().position(RUN_BEFORE_RESET + stamp_after_reset)
    y = reset_stamp().position(stamp_after_reset)
    figure.circle(x, y, 4.5, 'imu-dot')
    return x, y


def draw_recovery_with_the_copy(figure: Figure) -> None:
    """Add the copy's run: the sample with which the test passes, and the window it lies in."""
    draw_clock_and_samples(figure, None)
    passing_stamp = FRESH_SAMPLE_COUNT * IMU_PERIOD
    draw_sample_marker(figure, passing_stamp)
    # Two short lines at the right end of the fresh region, clear of the samples' line
    right = reset_run().last_position - 4
    middle = reset_stamp().position(RECOVERY / 2) + 5
    figure.text(right, middle - LINE_HEIGHT / 2,
                f'sample {FRESH_SAMPLE_COUNT}: {passing_stamp:.2f} s', 'note', 'end')
    # "<" keeps the line short enough to end clear of the dot and the samples' line
    figure.text(right, middle + LINE_HEIGHT / 2, f'counted: rising, < {RECOVERY:g} s', 'note',
                'end')


def draw_silence_with_the_old_time(figure: Figure) -> None:
    """Add the run of a plugin that keeps the old time: silent until the clock is back there."""
    draw_clock_and_samples(figure, RUN_BEFORE_RESET)
    first_stamp = first_stamp_after_reset(RUN_BEFORE_RESET)
    first_x, first_y = draw_sample_marker(figure, first_stamp)
    run, stamp = reset_run(), reset_stamp()
    reset_x = run.position(RUN_BEFORE_RESET)
    figure.text(first_x - 8, first_y - 8, f'next: about {first_stamp:.2f} s', 'note', 'end')
    figure.text(run.last_position - 6, stamp.position(RECOVERY / 2) + 4, 'no fresh sample',
                'note', 'end')
    figure.text((reset_x + first_x) / 2 - 6, stamp.position(0.75 * RUN_BEFORE_RESET), 'silent',
                'note', 'end')


def reset_with_the_copy_figure() -> Figure:
    """Return the reset panel with the robots' IMU plugin: fresh samples right away."""
    figure = Figure(
        'tl-reset-copy', RESET_WIDTH, RESET_HEIGHT, 'The reset test with the robots’ IMU plugin',
        f'The stamps of the IMU’s samples rise with Gazebo’s clock to {RUN_BEFORE_RESET:g} s. '
        'Then the test resets the simulation, and the clock starts from 0 again. The first '
        f'sample after the reset is stamped about {IMU_PERIOD:.2f} s. Samples that arrive after '
        'the test saved its count, just before the reset call, count as fresh when each stamp is '
        f'later than 0 s, later than the last fresh stamp and below {RECOVERY:g} s. The test '
        'passes with '
        f'{FRESH_SAMPLE_COUNT} of them. A dot marks sample {FRESH_SAMPLE_COUNT}, stamped about '
        f'{FRESH_SAMPLE_COUNT * IMU_PERIOD:.2f} s.')
    draw_reset_chart(figure, 'With the robots’ IMU plugin')
    draw_recovery_with_the_copy(figure)
    figure.text(8, RESET_HEIGHT - 8, f'{FRESH_SAMPLE_COUNT} fresh samples: the test passes',
                'strong')
    return figure


def reset_with_the_old_time_figure() -> Figure:
    """Return the reset panel with a plugin that keeps the old time: no fresh sample."""
    figure = Figure(
        'tl-reset-old-time', RESET_WIDTH, RESET_HEIGHT,
        'The reset test with a plugin that keeps the old time',
        f'The stamps of the IMU’s samples rise with Gazebo’s clock to {RUN_BEFORE_RESET:g} s. '
        'Then the test resets the simulation, and the clock starts from 0 again. The plugin '
        f'stays silent until the clock has passed {RUN_BEFORE_RESET:g} s again. No sample after '
        f'the reset has a stamp below {RECOVERY:g} s, so none counts as fresh, and the test '
        'fails.')
    draw_reset_chart(figure, 'With a plugin that keeps the old time')
    draw_silence_with_the_old_time(figure)
    figure.text(8, RESET_HEIGHT - 8, 'no fresh sample: the test fails', 'strong')
    return figure


def figure_html(drawings: Sequence[Figure], caption: str, widths: PanelWidths) -> str:
    """Return the drawings' style and a figure with the drawings side by side and one caption.

    The drawings are the figure's panels, without an element around each. caption is Markdown.
    """
    panels = '\n'.join(drawing.svg() for drawing in drawings)
    return f'{style_element(RULES)}\n{figure_element("tl", panels, caption, widths)}\n'


def at_rest_caption() -> str:
    """Return the caption of the test at rest's timeline, as Markdown."""
    return ('The test at rest, as a plan over simulation time. Blue is the command that the '
            'test sends, orange the IMU’s messages. The test keeps only the messages of '
            '`MEASURE` and checks them.')


def left_turn_caption() -> str:
    """Return the caption of the left turn's timeline, as Markdown."""
    return ('The left turn, as a plan over simulation time. The robot’s headings follow from '
            f'the commanded turn rate: {TURN_RATE} rad/s turns the robot by {TURN_RATE:g} rad, '
            f'about {math.degrees(TURN_RATE):.0f}°, in each second. The simulated robot needs a '
            'moment to reach the turn rate, so it turns a little less.')


def reset_caption() -> str:
    """Return the caption of the reset test's two panels, as Markdown."""
    return ('The reset test, as an idealized example: the reset comes at exactly '
            f'{RUN_BEFORE_RESET:g} s, and the IMU publishes exactly {IMU_RATE:g} samples per '
            'second. The robot stands still, and the test sends no command. Each panel shows '
            'the stamps of the IMU’s samples while the simulation runs. The horizontal axis '
            'doesn’t start over at the reset. The test counts a sample as fresh when the sample '
            'arrives after the test saved its count of samples, just before the reset call, '
            'with a stamp later than 0 s, later than the last fresh stamp and below `RECOVERY`, '
            f'{RECOVERY:g} s. In this ideal run, these are the '
            'samples in the gray area. The test '
            f'passes with {FRESH_SAMPLE_COUNT} fresh samples. It polls with a wall-time timeout '
            f'of {RECOVERY_TIMEOUT:g} s. The dot in the first panel is sample '
            f'{FRESH_SAMPLE_COUNT}, stamped about {FRESH_SAMPLE_COUNT * IMU_PERIOD:.2f} s. A '
            'repeated stamp doesn’t count, so a stream that repeats one stamp fails.')


def main() -> None:
    """Write the three includes."""
    with open('includes/imu-sim-test-at-rest.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(figure_html([at_rest_figure()], at_rest_caption(),
                                  PanelWidths(basis=300, widest=480)))
    with open('includes/imu-sim-test-left-turn.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(figure_html([left_turn_figure()], left_turn_caption(),
                                  PanelWidths(basis=300, widest=480)))
    with open('includes/imu-sim-test-reset.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(figure_html([reset_with_the_copy_figure(), reset_with_the_old_time_figure()],
                                  reset_caption(), PanelWidths(basis=280, widest=400)))


if __name__ == '__main__':
    main()
