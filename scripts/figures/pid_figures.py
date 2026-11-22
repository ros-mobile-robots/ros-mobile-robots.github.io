"""Generate the figure on docs/packages/diffbot_base/pid.md.

It writes includes/pid-feedback-loop.html: the speed control loop of one wheel as the
microcontroller's firmware runs it, and a time line of one control step, as inline SVG that follows
the site's light and dark colors. The include holds two layouts of the same figure: a wide one for
wider screens, with the forward path to the right and the feedback path back below it, and a tall
one for phones. CSS shows one of them. The values on the arrows are one control step of Remo's
left wheel, read from the firmware's debug output (BaseController::printDebug()) during a run
with the wheels off the ground:

    measured_ang_vel_left: 5.56
    wheel_cmd_velocity_left_: 6.06
    motor_cmd_left_: 96
    pid_left_errors (p, i, d): 0.50 322.20 0.00

LOGGED_STEP holds these values. The script derives the other numbers from them with the firmware's
formulas and stops if a derived number disagrees with a logged one. The ticks of the step aren't
logged: the figure shows the count that fits the measured speed at the nominal control period. The
page's text repeats the numbers, so change them together.

Run from the repository root after changing the figure:

    python3 scripts/figures/pid_figures.py
"""

import math
from typing import Callable
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

from board_drawings import board_rules
from board_drawings import draw_encoder_disc
from board_drawings import draw_gear_motor_with_wheel
from board_drawings import draw_motor_driver_board
from board_drawings import draw_raspberry_pi
from board_drawings import draw_teensy
from board_drawings import Drawing
from board_drawings import ENCODER_BOARD_OFFSET
from board_drawings import escape
from board_drawings import style_element

WIDTH = 360
HEIGHT = 538
# The forward path runs down at FORWARD_X, the feedback path back up at FEEDBACK_X. They meet in
# the comparison's circle at COMPARISON_Y
FORWARD_X = 90
FEEDBACK_X = 270
COMPARISON_Y = 140
# The boxes of the forward path and of the feedback path
LEFT_X = 15
LEFT_WIDTH = 150
RIGHT_X = 205
RIGHT_WIDTH = 130
# The time line under the loop: from TIME_START_X to TIME_END_X, the control step in the middle
TIME_Y = 498
TIME_START_X = 20
TIME_END_X = 340

# The wide layout, for screens at least WIDE_FROM wide: the forward path runs right along WIDE_ROW,
# the feedback path back left along WIDE_BACK, and the motor driver, the motor and the encoder
# stand in a column on the right. WIDE_WIDTH fits the PID page's article, which is 688 px wide on
# 1280 and 1440 px screens, so the text keeps its size. Narrower screens get the tall layout above
WIDE_FROM = '60em'
WIDE_WIDTH = 680
WIDE_HEIGHT = 486
WIDE_ROW = 128
WIDE_BACK = 346
WIDE_COMPARISON_X = 200
WIDE_TIME_Y = 444
WIDE_TIME_START_X = 20
WIDE_TIME_END_X = 660


class Box(NamedTuple):
    """A rectangle of the wide layout: its top left corner, its width and its height."""

    x: float
    y: float
    width: float
    height: float


WIDE_SBC = Box(4, WIDE_ROW - 40, 74, 80)
WIDE_FIRMWARE = Box(176, 40, 284, 346)
WIDE_PID = Box(296, WIDE_ROW - 44, 150, 88)
WIDE_SPEED = Box(230, WIDE_BACK - 22, 130, 44)
WIDE_DRIVER = Box(500, WIDE_ROW - 22, 176, 44)
WIDE_MOTOR = Box(500, 204, 176, 84)
WIDE_ENCODER = Box(500, WIDE_BACK - 28, 176, 56)

# The firmware's values, from the diffbot repository (https://github.com/ros-mobile-robots/diffbot)
# at commit f5f282e, which pid_src_raw and pid_src_url in mkdocs.yml pin for the page as well.
# diffbot_firmware/lib/config/diffbot_base_config.h
K_P = 0.6
K_I = 0.3
K_D = 0.5
PWM_MAX = 255
CONTROL_RATE = 20  # Hz, UPDATE_RATE_CONTROL
# diffbot_base/config/base.yaml, encoder_resolution: ticks per turn of the wheel
ENCODER_RESOLUTION = 542
# Adafruit Motor Shield V2 Library 1.1.4, Adafruit_MotorShield.cpp: Adafruit_DCMotor::setSpeed()
# sets speed * 16 of the PCA9685's 4096 steps per PWM period
PWM_STEPS_PER_SPEED = 16
PWM_STEPS_PER_PERIOD = 4096


class LoggedStep(NamedTuple):
    """One control step as the firmware's debug output prints it, rounded to two decimals."""

    target_speed: float    # rad/s, wheel_cmd_velocity_left_
    measured_speed: float  # rad/s, measured_ang_vel_left
    error: float           # rad/s, the first value of pid_left_errors (p, i, d)
    error_sum: float       # rad/s, the second value
    error_change: float    # rad/s, the third value
    motor_command: int     # motor_cmd_left_


LOGGED_STEP = LoggedStep(target_speed=6.06, measured_speed=5.56, error=0.50, error_sum=322.20,
                         error_change=0.00, motor_command=96)

# The accessible name of both layouts
TITLE = ("One wheel's speed control loop in the microcontroller firmware, with the motor driver, "
         'the motor, the wheel and the encoder, and a time line of one control step')

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.pid-loop-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.pid-loop', 'width:100%;height:auto;--pid-value:#1e63c4;'
                  'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('.pid-loop-wide', 'display:none'),
    (f'@media screen and (min-width:{WIDE_FROM})',
     '.md-typeset figure.pid-loop-figure{max-width:none}'
     '.pid-loop-wide{display:block}.pid-loop-tall{display:none}'),
    ('[data-md-color-scheme=slate] .pid-loop', '--pid-value:#82b1ff'),
    ('.pid-loop text', f'fill:{FG};font-size:12px'),
    ('.pid-loop .firmware', 'fill:var(--md-default-fg-color--lightest);'
                            'stroke:var(--md-default-fg-color--lighter);stroke-width:1'),
    ('.pid-loop .code', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.pid-loop .hardware', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.pid-loop .arrow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.pid-loop .arrow-head', f'fill:{FG_LIGHT}'),
    ('.pid-loop .number-disc', f'fill:{FG}'),
    ('.pid-loop .number', 'fill:var(--md-default-bg-color);font-size:11px;font-weight:700'),
    ('.pid-loop .name', 'font-size:13px'),
    ('.pid-loop .note', f'font-size:11px;fill:{FG_LIGHT}'),
    ('.pid-loop .symbol', 'font-style:italic'),
    ('.pid-loop .formula', 'font-size:11.5px;font-style:italic'),
    ('.pid-loop .container-name', 'font-weight:700'),
    ('.pid-loop .sign', 'font-size:15px;font-weight:700'),
    ('.pid-loop .value', 'fill:var(--pid-value);font-weight:700'),
    ('.pid-loop .time-line', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.pid-loop .time-step', f'fill:none;stroke:{FG};stroke-width:2.5'),
    # The drawings of the wide layout, in the colors of the real parts in both color schemes
    *board_rules('.pid-loop'),
)
# A piece of a formula: its text and whether it is a subscript
FormulaPart = Tuple[str, bool]


class Figure(Drawing):
    """Collect the SVG elements of one layout of the figure.

    name is the layout's CSS class, pid-loop-tall or pid-loop-wide, and the prefix of its IDs.
    """

    def __init__(self, name: str, width: int, height: int) -> None:
        super().__init__()
        self.name = name
        self.width = width
        self.height = height

    def arrow(self, path: str, css_class: str = 'arrow') -> None:
        """Add a line along an SVG path, with an arrowhead at its end."""
        self.elements.append(
            f'<path class="{css_class}" d="{path}" marker-end="url(#{self.name}-head)"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start', symbol: str = '') -> None:
        """Add one line of text, and the symbol in italics after it if one is given.

        anchor is start, middle or end.
        """
        class_attribute = f' class="{css_class}"' if css_class else ''
        symbol_element = f' <tspan class="symbol">{escape(symbol)}</tspan>' if symbol else ''
        self.elements.append(f'<text{class_attribute} x="{x:g}" y="{y:g}" text-anchor="{anchor}">'
                             f'{escape(text)}{symbol_element}</text>')

    def formula(self, x: float, y: float, parts: Sequence[FormulaPart],
                anchor: str = 'middle') -> None:
        """Add one line of a formula, with its subscripts below the line and smaller."""
        pieces = [f'<tspan baseline-shift="sub" font-size="8.5">{escape(text)}</tspan>'
                  if subscript else escape(text) for text, subscript in parts]
        self.elements.append(f'<text class="formula" x="{x:g}" y="{y:g}" '
                             f'text-anchor="{anchor}">{"".join(pieces)}</text>')

    def number(self, x: float, y: float, number: int) -> None:
        """Add a signal's number in a filled disc around (x, y)."""
        self.circle(x, y, 8, 'number-disc')
        self.text(x, y + 3.8, str(number), 'number', 'middle')

    def signal(self, x: float, y: float, number: int, names: Sequence[str], value: str = '',
               symbol: str = '') -> None:
        """Add a signal's label: its number, its name on one or more lines, and its value.

        The symbol, if one is given, follows the name's last line. The value is the line after.
        """
        self.number(x + 8, y - 4, number)
        for index, name in enumerate(names):
            last = index == len(names) - 1
            self.text(x + 20, y + 14 * index, name, symbol=symbol if last else '')
        if value:
            self.text(x + 20, y + 14 * len(names), value, 'value')

    def svg(self) -> str:
        """Return the layout as an SVG element."""
        head = (f'<marker id="{self.name}-head" viewBox="0 0 10 10" refX="9" refY="5" '
                'markerWidth="7" markerHeight="7" orient="auto">'
                '<path class="arrow-head" d="M0,0 L10,5 L0,10 z"/></marker>')
        return (f'<svg class="pid-loop {self.name}" viewBox="0 0 {self.width} {self.height}" '
                f'role="img" aria-labelledby="{self.name}-title"><title id="{self.name}-title">'
                f'{escape(TITLE)}</title><defs>{head}</defs>\n'
                + '\n'.join(self.elements) + '\n</svg>')


def measured_speed(ticks: int) -> float:
    """Return the wheel speed in rad/s for ticks counted in one control period.

    As Encoder::jointState() computes it: the ticks' angle divided by the time.
    """
    return ticks * 2 * math.pi / ENCODER_RESOLUTION * CONTROL_RATE


def ticks_in_period(step: LoggedStep) -> int:
    """Return the whole number of ticks that comes closest to the step's measured speed.

    The debug output doesn't print the ticks of a single step. This is the count that fits the
    logged speed when the step took the nominal control period.
    check_ticks_give_the_measured_speed() checks that the count gives the logged speed.
    """
    return round(step.measured_speed / measured_speed(1))


def pid_output(step: LoggedStep) -> float:
    """Return the PID's raw output for the step, as PID::compute() adds up its three terms."""
    return K_P * step.error + K_I * step.error_sum + K_D * step.error_change


def motor_command(raw_output: float) -> int:
    """Return the motor command that the firmware makes of a PID's raw output.

    PID::compute() limits the raw output to PWM_MIN through PWM_MAX. BaseController::write()
    assigns the result to an int, which drops the fractional part, towards zero.
    """
    return int(min(max(raw_output, -PWM_MAX), PWM_MAX))


def check_error_is_target_minus_measured(step: LoggedStep) -> List[str]:
    """Check that the logged error is the target speed minus the measured speed."""
    error = round(step.target_speed - step.measured_speed, 2)
    if error == step.error:
        return []
    return [f'the target {step.target_speed} rad/s minus the measured {step.measured_speed} '
            f'rad/s is {error} rad/s, not the logged error {step.error} rad/s']


def check_pid_output_is_within_the_pwm_range(step: LoggedStep) -> List[str]:
    """Check that the PID's raw output lies within the motor command's range.

    This is a condition of the example, not a formula of the firmware: the firmware also runs
    with a raw output beyond the range and limits it, but the page explains a step without that
    limit.
    """
    if -PWM_MAX <= pid_output(step) <= PWM_MAX:
        return []
    return [f'the PID output {pid_output(step):.2f} is outside {-PWM_MAX} to {PWM_MAX}, the '
            'range that the example explains']


def check_pid_output_gives_the_motor_command(step: LoggedStep) -> List[str]:
    """Check that the PID's output, limited and cut to a whole number, is the logged command."""
    if motor_command(pid_output(step)) == step.motor_command:
        return []
    return [f'the PID output {pid_output(step):.2f} does not give the logged motor command '
            f'{step.motor_command}']


def check_ticks_give_the_measured_speed(step: LoggedStep) -> List[str]:
    """Check that a whole number of ticks in one control period gives the logged speed."""
    ticks = ticks_in_period(step)
    if round(measured_speed(ticks), 2) == step.measured_speed:
        return []
    return [f'{ticks} ticks are {measured_speed(ticks):.4f} rad/s, not the logged '
            f'{step.measured_speed} rad/s']


# Each check takes the logged step and returns its problems as messages
CHECKS: Tuple[Callable[[LoggedStep], List[str]], ...] = (
    check_error_is_target_minus_measured,
    check_pid_output_is_within_the_pwm_range,
    check_pid_output_gives_the_motor_command,
    check_ticks_give_the_measured_speed,
)


def check_logged_step(step: LoggedStep) -> None:
    """Stop with every problem of a logged step.

    A problem is a value that doesn't fit the firmware's arithmetic, or a step that doesn't meet
    the worked example's assumptions.
    """
    problems = [problem for check in CHECKS for problem in check(step)]
    if problems:
        raise ValueError('. '.join(problems))


def on_time_percent(motor_command: int) -> float:
    """Return for how much of a PWM period the motor driver switches the motor on, in percent."""
    return abs(motor_command) * PWM_STEPS_PER_SPEED / PWM_STEPS_PER_PERIOD * 100


def draw_target(figure: Figure, step: LoggedStep) -> None:
    """Add the single-board computer, which sends the target wheel speed (signal 1)."""
    figure.box(LEFT_X, 6, LEFT_WIDTH, 32, 'hardware', [('SBC: Raspberry Pi 4 B', 'name')])
    figure.arrow(f'M{FORWARD_X},38 V{COMPARISON_Y - 14}')
    figure.signal(FORWARD_X + 6, 56, 1, ['target wheel speed'], f'{step.target_speed:.2f} rad/s')


def draw_firmware(figure: Figure, step: LoggedStep) -> None:
    """Add the firmware's part of the loop: the comparison, the PID and the measured speed.

    These are the error (signal 2) and the measured wheel speed (signal 7).
    """
    figure.box(4, 78, 352, 186, 'firmware')
    figure.text(348, 94, f'Microcontroller firmware, about every {1000 // CONTROL_RATE} ms',
                'container-name', 'end')
    figure.circle(FORWARD_X, COMPARISON_Y, 13, 'code')
    figure.text(FORWARD_X - 13, COMPARISON_Y - 12, '+', 'sign', 'end')
    figure.text(FORWARD_X + 17, COMPARISON_Y + 16, '\u2212', 'sign')
    figure.arrow(f'M{FEEDBACK_X},198 V{COMPARISON_Y} H{FORWARD_X + 14}')
    figure.signal(FORWARD_X + 20, 116, 7, ['measured wheel speed'],
                  f'{step.measured_speed:.2f} rad/s')
    figure.arrow(f'M{FORWARD_X},{COMPARISON_Y + 13} V197')
    figure.signal(FORWARD_X + 6, 172, 2, ['error'], f'{step.error:.2f} rad/s', symbol='e')
    figure.box(LEFT_X, 198, LEFT_WIDTH, 56, 'code', [
        ('PID controller', 'name'), (f'K_P {K_P}, K_I {K_I}, K_D {K_D}', 'note'),
        (f'output limited to \u00b1{PWM_MAX}', 'note')])
    figure.box(RIGHT_X, 198, RIGHT_WIDTH, 56, 'code', [
        ('speed from ticks', 'name'), ('angle \u00f7 time', 'note')])


def draw_hardware(figure: Figure, step: LoggedStep) -> None:
    """Add the motor driver, the motor with the wheel and the encoder (signals 3 to 6)."""
    figure.arrow(f'M{FORWARD_X},254 V321')
    figure.signal(FORWARD_X + 6, 286, 3, ['motor command'], str(step.motor_command), symbol='u')
    figure.box(LEFT_X, 322, LEFT_WIDTH, 42, 'hardware', [
        ('Motor driver', 'name'), ('Adafruit FeatherWing', 'note')])
    figure.arrow(f'M{FORWARD_X},364 V419')
    figure.signal(FORWARD_X + 6, 388, 4, ['PWM duty cycle'],
                  f'{on_time_percent(step.motor_command):.1f} %')
    figure.box(LEFT_X, 420, LEFT_WIDTH, 42, 'hardware', [
        ('Motor and wheel', 'name'), ('DG01D-E gear motor', 'note')])
    figure.arrow(f'M{LEFT_X + LEFT_WIDTH},441 H{FEEDBACK_X} V365')
    figure.signal(LEFT_X + LEFT_WIDTH + 4, 434, 5, ['rotation'])
    figure.box(RIGHT_X, 322, RIGHT_WIDTH, 42, 'hardware', [
        ('Encoder', 'name'), (f'{ENCODER_RESOLUTION} ticks per wheel turn', 'note')])
    figure.arrow(f'M{FEEDBACK_X},321 V255')
    figure.signal(FEEDBACK_X + 4, 280, 6, ['ticks since', 'last step'], str(ticks_in_period(step)))


def draw_time_line(figure: Figure, y: float, start_x: float, end_x: float) -> None:
    """Add a time line of one control step: what the step measures and what it commands.

    The measured speed (signal 7) is the mean over the interval before the step. The motor command
    (signal 3) normally acts during the interval after it, until the next step's write(). The
    scheduler counts whole milliseconds and the encoder measures the actual time between two
    reads, so the time line doesn't name a length.
    """
    step_x = (start_x + end_x) / 2
    figure.arrow(f'M{start_x},{y} H{end_x}', 'time-line')
    for x in (start_x, step_x):
        figure.line(f'M{x:g},{y - 5} V{y + 5}', 'time-line')
    figure.line(f'M{step_x:g},{y - 9} V{y + 9}', 'time-step')
    figure.text(step_x, y - 14, 'control step', 'container-name', 'middle')
    # The interval names sit away from the control step's name, which is in the middle
    figure.text((start_x + step_x) / 2 - 12, y - 6, 'previous interval', 'note', 'middle')
    figure.text((step_x + end_x) / 2 + 12, y - 6, 'next interval', 'note', 'middle')
    figure.signal(start_x + 4, y + 24, 7, ['measured over this time'])
    figure.signal(step_x + 8, y + 24, 3, ['acts during this time'])


def tall_feedback_loop(step: LoggedStep) -> Figure:
    """Return the tall layout, for phones: the loop runs down and back up."""
    figure = Figure('pid-loop-tall', WIDTH, HEIGHT)
    draw_target(figure, step)
    draw_firmware(figure, step)
    draw_hardware(figure, step)
    draw_time_line(figure, TIME_Y, TIME_START_X, TIME_END_X)
    return figure


def draw_motor_with_wheel(figure: Figure, box: Box) -> None:
    """Add the motor box: the DG01D-E from above, schematic, with the wheel on its shaft.

    The gearbox, the motor's can and the encoder board at the can's end lie along the motor's
    axis. The shaft leaves the gearbox sideways, and the wheel is seen from above, tyre first.
    """
    figure.box(*box, 'hardware')
    figure.text(box.x + 8, box.y + 22, 'Motor and', 'name')
    figure.text(box.x + 8, box.y + 36, 'wheel', 'name')
    figure.text(box.x + 8, box.y + 52, 'DG01D-E', 'note')
    draw_gear_motor_with_wheel(figure, box.x + 88, box.y + 12)


def encoder_board_x(motor: Box) -> float:
    """Return the x of the encoder board at the end of the motor's can, in the motor box."""
    return motor.x + 88 + ENCODER_BOARD_OFFSET


def draw_encoder(figure: Figure, box: Box) -> None:
    """Add the encoder box: the encoder seen from the motor's end, enlarged and schematic.

    The magnet disc on the motor shaft has six alternating poles. The ring-shaped board around it
    carries the two Hall sensors, both on the upper arc, and the connector below.
    """
    figure.box(*box, 'hardware')
    figure.text(box.x + 8, box.y + 19, 'Encoder', 'name')
    figure.text(box.x + 8, box.y + 34, f'{ENCODER_RESOLUTION} ticks per', 'note')
    figure.text(box.x + 8, box.y + 47, 'wheel turn', 'note')
    center_x, center_y = box.x + box.width - 34, box.y + box.height / 2 - 3
    draw_encoder_disc(figure, center_x, center_y)


def draw_wide_target(figure: Figure, step: LoggedStep) -> None:
    """Add the single-board computer with its drawing, and the target wheel speed (signal 1)."""
    box = WIDE_SBC
    center_x = box.x + box.width / 2
    figure.box(*box, 'hardware')
    draw_raspberry_pi(figure, box.x + 13, box.y + 6)
    figure.text(center_x, WIDE_ROW + 10, 'SBC', 'name', 'middle')
    figure.text(center_x, WIDE_ROW + 23, 'Raspberry', 'note', 'middle')
    figure.text(center_x, WIDE_ROW + 35, 'Pi 4 B', 'note', 'middle')
    figure.arrow(f'M{box.x + box.width},{WIDE_ROW} H{WIDE_COMPARISON_X - 14}')
    figure.signal(82, WIDE_ROW - 54, 1, ['target wheel', 'speed'],
                  f'{step.target_speed:.2f} rad/s')


def draw_wide_firmware(figure: Figure, step: LoggedStep) -> None:
    """Add the firmware with the Teensy at its title, the comparison, the PID and the speed.

    These are the error (signal 2) and the measured wheel speed (signal 7). The PID box shows the
    firmware's formula: the raw output, then its limit and the cut to a whole number.
    """
    box = WIDE_FIRMWARE
    right = box.x + box.width
    figure.box(*box, 'firmware')
    draw_teensy(figure, right - 233, box.y + 7)  # right before the title's first word
    figure.text(right - 8, box.y + 18, 'Microcontroller (MCU) firmware', 'container-name', 'end')
    figure.text(right - 8, box.y + 32,
                f'about every {1000 // CONTROL_RATE} ms, here a Teensy 4.0', 'note', 'end')
    figure.circle(WIDE_COMPARISON_X, WIDE_ROW, 13, 'code')
    figure.text(WIDE_COMPARISON_X - 12, WIDE_ROW - 12, '+', 'sign', 'end')
    figure.text(WIDE_COMPARISON_X - 10, WIDE_ROW + 30, '\u2212', 'sign', 'end')
    pid = WIDE_PID
    figure.arrow(f'M{WIDE_COMPARISON_X + 13},{WIDE_ROW} H{pid.x}')
    figure.signal(WIDE_COMPARISON_X + 18, WIDE_ROW + 26, 2, ['error'], f'{step.error:.2f} rad/s',
                  symbol='e')
    pid_x = pid.x + pid.width / 2
    figure.box(*pid, 'code')
    figure.text(pid_x, WIDE_ROW - 26, 'PID controller', 'name', 'middle')
    figure.formula(pid_x, WIDE_ROW - 7, [('u', False), ('raw', True), (' = K', False),
                                         ('P', True), ('\u00b7e + K', False), ('I', True),
                                         ('\u00b7\u03a3e', False)])
    figure.formula(pid_x, WIDE_ROW + 12, [('+ K', False), ('D', True),
                                          ('\u00b7(e \u2212 e', False), ('previous', True),
                                          (')', False)])
    figure.text(pid_x, WIDE_ROW + 33, f'limit to \u00b1{PWM_MAX}, truncate \u2192 u', 'note',
                'middle')
    speed = WIDE_SPEED
    figure.box(*speed, 'code', [('speed from ticks', 'name'), ('angle \u00f7 time', 'note')])
    figure.arrow(f'M{speed.x},{WIDE_BACK} H{WIDE_COMPARISON_X} V{WIDE_ROW + 13}')
    figure.signal(WIDE_COMPARISON_X + 8, 252, 7, ['measured', 'wheel speed'],
                  f'{step.measured_speed:.2f} rad/s')


def draw_wide_hardware(figure: Figure, step: LoggedStep) -> None:
    """Add the motor driver, the motor with the wheel and the encoder (signals 3 to 6)."""
    driver, motor, encoder, speed = WIDE_DRIVER, WIDE_MOTOR, WIDE_ENCODER, WIDE_SPEED
    pid_right = WIDE_PID.x + WIDE_PID.width
    figure.arrow(f'M{pid_right},{WIDE_ROW} H{driver.x}')
    figure.signal(pid_right + 18, WIDE_ROW - 54, 3, ['motor', 'command'],
                  str(step.motor_command), symbol='u')
    figure.box(*driver, 'hardware')
    figure.text(driver.x + 8, driver.y + 19, 'Motor driver', 'name')
    figure.text(driver.x + 8, driver.y + 33, 'Adafruit FeatherWing', 'note')
    draw_motor_driver_board(figure, driver.x + driver.width - 50, driver.y + 10)
    center_x = motor.x + motor.width / 2
    figure.arrow(f'M{center_x},{driver.y + driver.height} V{motor.y}')
    figure.signal(center_x + 6, WIDE_ROW + 36, 4, ['PWM duty', 'cycle'],
                  f'{on_time_percent(step.motor_command):.1f} %')
    draw_motor_with_wheel(figure, motor)
    board_x = encoder_board_x(motor)
    figure.arrow(f'M{board_x},{motor.y + 34} V{encoder.y}')
    figure.signal(board_x - 92, motor.y + motor.height + 14, 5, ['motor shaft', 'rotation'])
    draw_encoder(figure, encoder)
    firmware_right = WIDE_FIRMWARE.x + WIDE_FIRMWARE.width
    figure.arrow(f'M{encoder.x},{WIDE_BACK} H{speed.x + speed.width}')
    figure.text((firmware_right + encoder.x) / 2, WIDE_BACK - 6, 'pulses', 'note', 'middle')
    figure.signal(speed.x + speed.width + 6, WIDE_BACK - 48, 6, ['ticks since', 'last step'],
                  str(ticks_in_period(step)))


def wide_feedback_loop(step: LoggedStep) -> Figure:
    """Return the wide layout: target on the left, forward path right, feedback path below."""
    figure = Figure('pid-loop-wide', WIDE_WIDTH, WIDE_HEIGHT)
    draw_wide_target(figure, step)
    draw_wide_firmware(figure, step)
    draw_wide_hardware(figure, step)
    draw_time_line(figure, WIDE_TIME_Y, WIDE_TIME_START_X, WIDE_TIME_END_X)
    return figure


def caption() -> str:
    """Return the figure's caption, which both layouts share."""
    period = f'{1000 // CONTROL_RATE} ms'
    return ("One wheel's speed control loop. Start at the target wheel speed (1) from the "
            'single-board computer (SBC) and follow the numbers around the loop, back to the '
            'comparison in the circle. The boxes inside the microcontroller firmware are code. '
            'The motor driver, the motor with the wheel and the encoder are hardware. The blue '
            "values are one control step of Remo's left wheel. The firmware doesn't log the ticks "
            '(6). They are calculated from the measured speed (7). The time line shows that a '
            'control step measures the speed over the previous interval and sets the motor '
            f'command for the next one. The example takes each interval as the nominal {period}. '
            'The encoder senses the motor shaft before the gearbox, and the firmware uses '
            f'{ENCODER_RESOLUTION} ticks per wheel turn.')


def main() -> None:
    """Write the include with both layouts of the figure and their caption."""
    check_logged_step(LOGGED_STEP)
    with open('includes/pid-feedback-loop.html', 'w', encoding='utf-8') as include:
        include.write(f'{style_element(RULES)}\n<figure class="pid-loop-figure">\n'
                      f'{wide_feedback_loop(LOGGED_STEP).svg()}\n'
                      f'{tall_feedback_loop(LOGGED_STEP).svg()}\n'
                      f'<figcaption>{caption()}</figcaption>\n</figure>\n')


if __name__ == '__main__':
    main()
