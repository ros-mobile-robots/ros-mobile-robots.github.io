"""Generate the figures of the page DC Motors and Motor Drivers, docs/theory/actuation.md.

The script writes two includes, inline SVG that follows the site's light and dark colors. Each
holds a narrow layout for phones and a wide one for wider screens, and CSS shows one of them:

- includes/actuation-h-bridge.html: an H-bridge in its four modes, forward, reverse, coast and
  brake, with the switches that are closed and the path of the current. The modes and their
  switches are those of Table 1 in Microchip's application note AN905, Brushed DC Motor
  Fundamentals, which also names the switches Q1 to Q4 in this order.
- includes/actuation-pwm.html: the voltage at a motor for three duty cycles of pulse-width
  modulation (PWM), with the average voltage of each. The three duty cycles are those of three
  commands of Remo's firmware, and the supply is the nominal voltage of the robots' eight NiMH
  cells.

The page's table of the modes names the same switches as MODES, and the page's text repeats the
numbers of the PWM figure, so change them together. tests/test_motor_driver_figures.py checks
both.

Run from the repository root after changing a figure:

    python3 scripts/figures/motor_driver_figures.py
"""

import math
from typing import FrozenSet
from typing import List
from typing import NamedTuple
from typing import Tuple

from localization_figures import BG
from localization_figures import FG
from localization_figures import FG_LIGHT
from localization_figures import Figure
from localization_figures import LAYOUT_SPACE
from localization_figures import NARROW_LAYOUT
from localization_figures import RULES
from localization_figures import WIDE_FROM
from localization_figures import WIDE_LAYOUT
from localization_figures import WIDE_WIDTH
from localization_figures import WIDTH
from localization_figures import write_layout_parts

# --- The H-bridge -------------------------------------------------------------------------------


class Switch(NamedTuple):
    """One of the bridge's four switches: its name and its place in the bridge."""

    name: str
    right: bool  # in the right half-bridge, else in the left one
    high: bool  # between the supply and the motor, else between the motor and ground


SWITCHES = (Switch('Q1', right=False, high=True), Switch('Q2', right=False, high=False),
            Switch('Q3', right=True, high=True), Switch('Q4', right=True, high=False))


class Mode(NamedTuple):
    """A mode of the bridge: the switches that are closed and the direction of the current."""

    name: str
    closed: FrozenSet[str]
    # 1: the current flows through the motor from left to right, -1: from right to left,
    # 0: the drawing marks the path without a direction (coast: no path at all)
    direction: int


MODES = (Mode('Forward', frozenset({'Q1', 'Q4'}), 1),
         Mode('Reverse', frozenset({'Q2', 'Q3'}), -1),
         Mode('Coast', frozenset(), 0),
         Mode('Brake', frozenset({'Q2', 'Q4'}), 0))

# One bridge in a panel PANEL_HEIGHT units tall: the supply rail at the top, ground at the
# bottom, the two half-bridges at the sides, and the motor between their middles. The narrow
# layout has two columns of wide panels, the wide layout one row of narrower ones, so that both
# fill their drawing's width, WIDTH and WIDE_WIDTH
PANEL_HEIGHT = 186
RAIL_TOP = 48
RAIL_BOTTOM = 166
SIDE_INSET = 34  # from a panel's edge to its half-bridge
MOTOR_RADIUS = 14
SWITCH_GAP = 16  # the length of a switch along its half-bridge
HIGH_SWITCH_Y = 66
LOW_SWITCH_Y = 124
GAP = 20  # between the panels


class BridgeLayout(NamedTuple):
    """A layout of the four panels: columns per row, panel width, and the drawing's class."""

    columns: int
    panel_width: float
    css_class: str


BRIDGE_LAYOUTS = (BridgeLayout(2, (WIDTH - GAP) / 2, NARROW_LAYOUT),
                  BridgeLayout(4, (WIDE_WIDTH - 3 * GAP) / 4, WIDE_LAYOUT))


def half_bridge_x(left: float, panel_width: float, right: bool) -> float:
    """Return the x of a half-bridge in the panel that starts at left."""
    return left + panel_width - SIDE_INSET if right else left + SIDE_INSET


def middle_y() -> float:
    """Return the y of the motor and of the two middles of the half-bridges."""
    return (HIGH_SWITCH_Y + SWITCH_GAP + LOW_SWITCH_Y) / 2


def switch_y(switch: Switch) -> float:
    """Return the y where a switch starts, at its upper contact."""
    return HIGH_SWITCH_Y if switch.high else LOW_SWITCH_Y


def draw_switch(figure: Figure, x: float, top: float, switch: Switch, closed: bool) -> None:
    """Add a switch: two contacts and a lever that touches the lower contact when closed."""
    upper = top + switch_y(switch)
    lower = upper + SWITCH_GAP
    # An open lever points toward the motor, away from the switch's label
    lever_end_x = x if closed else x + (-9 if switch.right else 9)
    lever_end_y = lower if closed else lower - 5
    figure.path(f'M{x},{upper} L{lever_end_x},{lever_end_y}',
                'switch closed' if closed else 'switch')
    for contact_y in (upper, lower):
        figure.circle(x, contact_y, 2, 'contact')
    label_x = x - 8 if not switch.right else x + 8
    figure.text(label_x, upper + SWITCH_GAP / 2 + 4, switch.name, 'note',
                'end' if not switch.right else 'start')


def current_path(left: float, top: float, panel_width: float, mode: Mode) -> str:
    """Return the path of the current through the closed switches and the motor."""
    middle = top + middle_y()
    rail_top = top + RAIL_TOP
    rail_bottom = top + RAIL_BOTTOM
    left_x = half_bridge_x(left, panel_width, right=False)
    right_x = half_bridge_x(left, panel_width, right=True)
    if mode.direction == 1:
        return (f'M{left_x},{rail_top} V{middle} H{right_x} V{rail_bottom}')
    if mode.direction == -1:
        return (f'M{right_x},{rail_top} V{middle} H{left_x} V{rail_bottom}')
    # Brake: the motor's terminals through the two low switches and the ground rail
    return (f'M{left_x},{middle} H{right_x} V{rail_bottom} H{left_x} Z')


def draw_panel(figure: Figure, left: float, top: float, panel_width: float, mode: Mode) -> None:
    """Add one bridge in one mode, with the mode's name and its closed switches above it."""
    left_x = half_bridge_x(left, panel_width, right=False)
    right_x = half_bridge_x(left, panel_width, right=True)
    rail_top = top + RAIL_TOP
    rail_bottom = top + RAIL_BOTTOM
    middle = top + middle_y()
    center = left + panel_width / 2
    figure.text(center, top + 12, mode.name, 'name strong', 'middle')
    closed = ' and '.join(sorted(mode.closed)) + ' closed' if mode.closed else 'all open'
    figure.text(center, top + 25, closed, 'note', 'middle')
    if mode.closed:
        figure.path(current_path(left, top, panel_width, mode), 'current')
    figure.path(f'M{left_x},{rail_top} H{right_x}', 'rail')
    figure.path(f'M{left_x},{rail_bottom} H{right_x}', 'rail')
    figure.text(center, rail_top - 5, '+V', 'note', 'middle')
    figure.text(center, rail_bottom + 13, 'GND', 'note', 'middle')
    for x in (left_x, right_x):
        figure.path(f'M{x},{rail_top} V{top + HIGH_SWITCH_Y} '
                    f'M{x},{top + HIGH_SWITCH_Y + SWITCH_GAP} V{top + LOW_SWITCH_Y} '
                    f'M{x},{top + LOW_SWITCH_Y + SWITCH_GAP} V{rail_bottom}', 'wire')
    motor_x = center
    figure.path(f'M{left_x},{middle} H{motor_x - MOTOR_RADIUS} '
                f'M{motor_x + MOTOR_RADIUS},{middle} H{right_x}', 'wire')
    for switch in SWITCHES:
        x = right_x if switch.right else left_x
        draw_switch(figure, x, top, switch, switch.name in mode.closed)
    figure.circle(motor_x, middle, MOTOR_RADIUS, 'motor')
    figure.text(motor_x, middle + 4, 'M', 'name', 'middle')
    if mode.direction:
        arrow_x = motor_x + mode.direction * (MOTOR_RADIUS + 10)
        figure.path(f'M{arrow_x - mode.direction * 6},{middle - 5} L{arrow_x},{middle} '
                    f'L{arrow_x - mode.direction * 6},{middle + 5} Z', 'current-head')


def h_bridge_figure(layout: BridgeLayout) -> Figure:
    """Return the four modes in a grid of the layout's columns."""
    rows = -(-len(MODES) // layout.columns)
    width = round(layout.columns * layout.panel_width + (layout.columns - 1) * GAP)
    height = rows * PANEL_HEIGHT + (rows - 1) * GAP
    title = ('An H-bridge in its four modes: forward with Q1 and Q4 closed, reverse with Q2 and '
             'Q3 closed, coast with all switches open, and brake with Q2 and Q4 closed, which '
             'connects both motor terminals to ground')
    figure = Figure(f'ac-bridge-{layout.columns}', title, height, width, layout.css_class)
    for index, mode in enumerate(MODES):
        row, column = divmod(index, layout.columns)
        draw_panel(figure, column * (layout.panel_width + GAP), row * (PANEL_HEIGHT + GAP),
                   layout.panel_width, mode)
    return figure


# --- PWM ----------------------------------------------------------------------------------------

# Remo's firmware passes a command from 0 to 255 to Adafruit_DCMotor::setSpeed(), which sets the
# PWM channel of the FeatherWing's PCA9685 to 16 counts per step of the command, of 4096 counts
# per period (Adafruit_MotorShield 1.1.4). The firmware asks for 1600 Hz. The library lowers the
# request by 10 % and rounds the chip's prescaler (Adafruit_MS_PWMServoDriver::setPWMFreq()), and
# the PCA9685 divides its 25 MHz oscillator by 4096 times the prescaler plus 1 (its datasheet,
# section 7.3.5)
COUNTS_PER_STEP = 16
COUNTS_PER_PERIOD = 4096
REQUESTED_FREQUENCY_HZ = 1600
OSCILLATOR_HZ = 25_000_000
COMMANDS = (64, 128, 192)
SUPPLY_VOLTAGE = 9.6  # eight NiMH cells, 1.2 V nominal each
PERIODS = 3

PLOT_LEFT = 90  # the waveforms start here, after the row's label
PLOT_RIGHT = WIDTH - 12
ROW_TOP = 18
ROW_HEIGHT = 80
WAVE_HEIGHT = 40  # the supply voltage, in units
AVERAGE_LABEL_DROP = 16  # from the bottom of a row's wave to the baseline of its average


class Pulse(NamedTuple):
    """One command's duty cycle and the average voltage that it gives."""

    command: int
    duty: float  # from 0 to 1
    average: float  # in V


def pulses() -> Tuple[Pulse, ...]:
    """Return the duty cycle and the average voltage of each command in COMMANDS."""
    result = []
    for command in COMMANDS:
        duty = command * COUNTS_PER_STEP / COUNTS_PER_PERIOD
        result.append(Pulse(command, duty, duty * SUPPLY_VOLTAGE))
    return tuple(result)


def volts(value: float) -> str:
    """Return a voltage with one decimal."""
    return f'{value:.1f} V'


def prescaler() -> int:
    """Return the PCA9685's prescaler as Adafruit's library computes it for the request."""
    return math.floor(OSCILLATOR_HZ / COUNTS_PER_PERIOD / (REQUESTED_FREQUENCY_HZ * 0.9) - 1 + 0.5)


def frequency_hz() -> float:
    """Return the PWM frequency that the PCA9685 runs at with this prescaler."""
    return OSCILLATOR_HZ / (COUNTS_PER_PERIOD * (prescaler() + 1))


def period_ms() -> float:
    """Return the PWM period in ms."""
    return 1000 / frequency_hz()


def draw_pulse_row(figure: Figure, top: float, pulse: Pulse) -> None:
    """Add one row: its label, three periods of the voltage and the average as a dashed line."""
    low = top + WAVE_HEIGHT
    high = top
    period = (PLOT_RIGHT - PLOT_LEFT) / PERIODS
    figure.text(PLOT_LEFT - figure.fixed(10), top + 14, f'{pulse.duty * 100:.0f} %',
                'name strong', 'end')
    figure.text(PLOT_LEFT - figure.fixed(10), top + 28, f'command {pulse.command}', 'note',
                'end')
    commands: List[str] = [f'M{PLOT_LEFT},{low}']
    for index in range(PERIODS):
        start = PLOT_LEFT + index * period
        commands.append(f'V{high} H{start + pulse.duty * period:.1f} V{low} '
                        f'H{start + period:.1f}')
    figure.path(' '.join(commands), 'wave')
    average_y = low - pulse.duty * WAVE_HEIGHT
    figure.path(f'M{PLOT_LEFT},{average_y:.1f} H{PLOT_RIGHT}', 'average')
    figure.text(PLOT_RIGHT, low + AVERAGE_LABEL_DROP, f'average {volts(pulse.average)}',
                'note average-label', 'end')


def pwm_figure(stretch: float, layout: str) -> Figure:
    """Return the three rows of waveforms, stretched along the time axis by stretch."""
    rows = pulses()
    height = ROW_TOP + len(rows) * ROW_HEIGHT + 30
    figure = Figure(f'ac-pwm-{layout}', 'The voltage at a motor for duty cycles of 25, 50 and 75 '
                    f'percent at a supply of {volts(SUPPLY_VOLTAGE)}: the averages are '
                    + ', '.join(volts(pulse.average) for pulse in rows),
                    height, round(WIDTH * stretch), layout, stretch)
    for index, pulse in enumerate(rows):
        draw_pulse_row(figure, ROW_TOP + index * ROW_HEIGHT, pulse)
    period = (PLOT_RIGHT - PLOT_LEFT) / PERIODS
    first = rows[0]
    axis_y = ROW_TOP + len(rows) * ROW_HEIGHT - 12
    figure.dimension(PLOT_LEFT, PLOT_LEFT + period, axis_y)
    figure.text(PLOT_LEFT + period / 2, axis_y + 14,
                f'one period, about {period_ms():.2f} ms', 'note', 'middle')
    top_label_y = ROW_TOP - 6
    figure.text(PLOT_LEFT + first.duty * period / 2, top_label_y, f'{volts(SUPPLY_VOLTAGE)}',
                'note', 'middle')
    return figure


# --- The includes -------------------------------------------------------------------------------

FIGURE_RULES = RULES + (
    ('.md-typeset figure.lv-figure.ac-figure', 'max-width:24rem'),
    # The wide layouts are WIDE_WIDTH units wide and get a figure that is as much wider, so that
    # their text is as large as the narrow layouts'
    (f'@container (min-width:{WIDE_FROM})',
     f'.md-typeset .{LAYOUT_SPACE} figure.lv-figure.ac-figure.lv-layouts'
     f'{{max-width:{24 * WIDE_WIDTH / WIDTH:g}rem}}'),
    ('.ac-figure .lv-fig text', 'font-size:11.5px'),
    ('.ac-figure .lv-fig .name', 'font-size:12.5px'),
    ('.ac-figure .lv-fig .note', 'font-size:11px'),
    ('.ac-figure', '--ac-current:#eb6834'),
    ('[data-md-color-scheme=slate] .ac-figure', '--ac-current:#ff9a62'),
    ('.ac-figure .rail', f'fill:none;stroke:{FG};stroke-width:2'),
    ('.ac-figure .wire', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.4'),
    ('.ac-figure .switch', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.6;stroke-linecap:round'),
    ('.ac-figure .switch.closed', f'stroke:{FG};stroke-width:2.2'),
    ('.ac-figure .contact', f'fill:{BG};stroke:{FG};stroke-width:1.2'),
    ('.ac-figure .motor', f'fill:{BG};stroke:{FG};stroke-width:1.6'),
    ('.ac-figure .current', 'fill:none;stroke:var(--ac-current);stroke-width:6;opacity:.45;'
                            'stroke-linejoin:round'),
    ('.ac-figure .current-head', 'fill:var(--ac-current)'),
    ('.ac-figure .wave', f'fill:none;stroke:{FG};stroke-width:1.8'),
    ('.ac-figure .average', 'fill:none;stroke:var(--ac-current);stroke-width:1.8;'
                            'stroke-dasharray:5 3'),
    ('.ac-figure .average-label', 'fill:var(--ac-current);font-weight:700'),
)
WIDE_STRETCH = WIDE_WIDTH / WIDTH


def write_h_bridge() -> None:
    """Write the include of the H-bridge's four modes."""
    parts = [h_bridge_figure(layout).svg() for layout in BRIDGE_LAYOUTS]
    write_layout_parts(
        'includes/actuation-h-bridge.html', parts,
        'An H-bridge in its four modes. The orange band is the path of the current, and the '
        'arrow shows its direction through the motor M. Forward and reverse connect the motor '
        'to the supply +V in opposite directions. In coast, all switches are open and the motor '
        'turns on freely. In brake, the two low switches connect both motor terminals to '
        'ground, and a turning motor drives a current through this loop that slows it down.',
        FIGURE_RULES, 'ac-figure')


def write_pwm() -> None:
    """Write the include of the three duty cycles."""
    parts = [pwm_figure(1, NARROW_LAYOUT).svg(), pwm_figure(WIDE_STRETCH, WIDE_LAYOUT).svg()]
    write_layout_parts(
        'includes/actuation-pwm.html', parts,
        f'The voltage at a motor for three duty cycles, at a supply of '
        f'{volts(SUPPLY_VOLTAGE)}. Each period, the driver connects the motor to the supply for '
        'the share of the period that the duty cycle gives, and the dashed line is the average '
        "voltage. The commands are those of Remo's firmware that give these duty cycles. The "
        'drawing treats the switches as ideal, without the voltage that a real driver loses.',
        FIGURE_RULES, 'ac-figure')


def main() -> None:
    """Write both includes."""
    write_h_bridge()
    write_pwm()


if __name__ == '__main__':
    main()
