"""Generate the figure on docs/packages/diffbot_base/pid.md.

It writes includes/pid-feedback-loop.html: the speed control loop of one wheel as the Teensy
firmware runs it, as inline SVG that follows the site's light and dark colors. The numbers on the
arrows are one control step of Remo's left wheel, read from the firmware's debug output
(BaseController::printDebug()) during a run with the wheels off the ground:

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
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

WIDTH = 360
HEIGHT = 480
# The forward path runs down at FORWARD_X, the feedback path back up at FEEDBACK_X. They meet in
# the comparison's circle at COMPARISON_Y
FORWARD_X = 105
FEEDBACK_X = 265
COMPARISON_Y = 126

# diffbot_firmware/lib/config/diffbot_base_config.h
K_P = 0.6
K_I = 0.3
K_D = 0.5
PWM_MAX = 255
CONTROL_RATE = 20  # Hz, UPDATE_RATE_CONTROL
# diffbot_base/config/base.yaml, encoder_resolution: ticks per turn of the wheel
ENCODER_RESOLUTION = 542
# Adafruit_DCMotor::setSpeed() sets speed * 16 of the PCA9685's 4096 steps per PWM period
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

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.pid-loop-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.pid-loop', 'width:100%;height:auto;--pid-value:#1e63c4;'
                  'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .pid-loop', '--pid-value:#82b1ff'),
    ('.pid-loop text', f'fill:{FG};font-size:10px'),
    ('.pid-loop .firmware', 'fill:var(--md-default-fg-color--lightest);'
                            'stroke:var(--md-default-fg-color--lighter);stroke-width:1'),
    ('.pid-loop .code', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.pid-loop .hardware', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.pid-loop .arrow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.pid-loop .arrow-head', f'fill:{FG_LIGHT}'),
    ('.pid-loop .number-disc', f'fill:{FG}'),
    ('.pid-loop .number', 'fill:var(--md-default-bg-color);font-size:9.5px;font-weight:700'),
    ('.pid-loop .name', 'font-size:11px'),
    ('.pid-loop .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                        'font-size:9.5px'),
    ('.pid-loop .note', f'font-size:9.5px;fill:{FG_LIGHT}'),
    ('.pid-loop .container-name', 'font-weight:700'),
    ('.pid-loop .sign', 'font-size:13px;font-weight:700'),
    ('.pid-loop .value', 'fill:var(--pid-value);font-weight:700'),
)


def escape(text: str) -> str:
    """Return text with the characters escaped that have a meaning in SVG."""
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def style_element(rules: Sequence[Tuple[str, str]]) -> str:
    """Return a style element with one CSS rule per (selector, declarations) pair."""
    lines = [f'{selector}{{{declarations}}}' for selector, declarations in rules]
    return '<style>\n' + '\n'.join(lines) + '\n</style>'


class Figure:
    """Collect the SVG elements of the figure."""

    def __init__(self, title: str) -> None:
        self.title = title
        self.elements: List[str] = []

    def box(self, x: float, y: float, width: float, height: float, css_class: str,
            lines: Sequence[Line] = ()) -> None:
        """Add a rectangle with its text lines centered in it, 13 px apart."""
        self.elements.append(f'<rect class="{css_class}" x="{x}" y="{y}" width="{width}" '
                             f'height="{height}" rx="5"/>')
        for index, (text, text_class) in enumerate(lines):
            baseline = y + height / 2 + (index - (len(lines) - 1) / 2) * 13 + 4
            self.text(x + width / 2, baseline, text, text_class, 'middle')

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle around (x, y)."""
        self.elements.append(f'<circle class="{css_class}" cx="{x}" cy="{y}" r="{radius}"/>')

    def arrow(self, path: str) -> None:
        """Add a line along an SVG path, with an arrowhead at its end."""
        self.elements.append(f'<path class="arrow" d="{path}" marker-end="url(#pid-loop-head)"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of text. anchor is start, middle or end."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(
            f'<text{class_attribute} x="{x}" y="{y}" text-anchor="{anchor}">{escape(text)}</text>')

    def number(self, x: float, y: float, number: int) -> None:
        """Add a signal's number in a filled disc around (x, y)."""
        self.circle(x, y, 7, 'number-disc')
        self.text(x, y + 3.2, str(number), 'number', 'middle')

    def signal(self, x: float, y: float, number: int, name: str, value: str = '') -> None:
        """Add a signal's label: its number and name, and its example value on a second line."""
        self.number(x + 7, y - 3.5, number)
        self.text(x + 18, y, name)
        if value:
            self.text(x + 18, y + 12, value, 'value')

    def svg(self) -> str:
        """Return the figure as an SVG element."""
        head = ('<marker id="pid-loop-head" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
                'markerHeight="7" orient="auto"><path class="arrow-head" d="M0,0 L10,5 L0,10 z"/>'
                '</marker>')
        return (f'<svg class="pid-loop" viewBox="0 0 {WIDTH} {HEIGHT}" role="img" '
                f'aria-labelledby="pid-loop-title"><title id="pid-loop-title">'
                f'{escape(self.title)}</title><defs>{head}</defs>\n'
                + '\n'.join(self.elements) + '\n</svg>')


def measured_speed(ticks: int) -> float:
    """Return the wheel speed in rad/s for ticks counted in one control period.

    As Encoder::jointState() computes it: the ticks' angle divided by the time.
    """
    return ticks * 2 * math.pi / ENCODER_RESOLUTION * CONTROL_RATE


def ticks_in_period(step: LoggedStep) -> int:
    """Return the whole number of ticks that gives the step's measured speed.

    The debug output doesn't print the ticks of a single step. This is the count that fits the
    logged speed when the step took the nominal control period.
    """
    ticks = round(step.measured_speed / measured_speed(1))
    if round(measured_speed(ticks), 2) != step.measured_speed:
        raise ValueError(f'{ticks} ticks are {measured_speed(ticks):.4f} rad/s, '
                         f'not the logged {step.measured_speed} rad/s')
    return ticks


def pid_output(step: LoggedStep) -> float:
    """Return the PID's output for the step, as PID::compute() adds up its three terms."""
    return K_P * step.error + K_I * step.error_sum + K_D * step.error_change


def check(step: LoggedStep) -> None:
    """Stop if the logged values don't fit the firmware's formulas."""
    if round(step.target_speed - step.measured_speed, 2) != step.error:
        raise ValueError('the error is not the target minus the measured speed')
    if not -PWM_MAX <= pid_output(step) <= PWM_MAX:
        raise ValueError('the PID output is outside the range that the example explains')
    if int(pid_output(step)) != step.motor_command:
        raise ValueError(f'the PID output {pid_output(step):.2f} does not give the logged '
                         f'motor command {step.motor_command}')


def on_time_percent(motor_command: int) -> float:
    """Return for how much of a PWM period the motor driver switches the motor on, in percent."""
    return abs(motor_command) * PWM_STEPS_PER_SPEED / PWM_STEPS_PER_PERIOD * 100


def draw_target(figure: Figure, step: LoggedStep) -> None:
    """Add the Raspberry Pi, which sends the target wheel speed (signal 1)."""
    figure.box(30, 6, 150, 34, 'code', [
        ('Raspberry Pi', 'name'), ('wheel_cmd_velocities', 'mono')])
    figure.arrow(f'M{FORWARD_X},40 V{COMPARISON_Y - 14}')
    figure.signal(FORWARD_X + 8, 56, 1, 'target wheel speed', f'{step.target_speed:.2f} rad/s')


def draw_firmware(figure: Figure, step: LoggedStep) -> None:
    """Add the firmware's part of the loop: the comparison, the PID and the measured speed.

    These are the error (signal 2) and the measured wheel speed (signal 7).
    """
    figure.box(4, 74, 352, 176, 'firmware')
    figure.text(348, 89, f'Teensy firmware, every {1000 // CONTROL_RATE} ms', 'container-name',
                'end')
    figure.circle(FORWARD_X, COMPARISON_Y, 13, 'code')
    figure.text(FORWARD_X - 13, COMPARISON_Y - 12, '+', 'sign', 'end')
    figure.text(FORWARD_X + 17, COMPARISON_Y + 15, '\u2212', 'sign')
    figure.arrow(f'M{FEEDBACK_X},180 V{COMPARISON_Y} H{FORWARD_X + 14}')
    figure.signal(FORWARD_X + 24, COMPARISON_Y - 18, 7, 'measured wheel speed',
                  f'{step.measured_speed:.2f} rad/s')
    figure.arrow(f'M{FORWARD_X},{COMPARISON_Y + 13} V179')
    figure.signal(FORWARD_X + 8, 156, 2, 'error', f'{step.error:.2f} rad/s')
    figure.box(30, 180, 150, 52, 'code', [
        ('PID controller', 'name'), ('PID::compute()', 'mono'),
        (f'K_P {K_P}, K_I {K_I}, K_D {K_D}', 'note')])
    figure.box(200, 180, 130, 52, 'code', [
        ('speed from ticks', 'name'), ('Encoder::jointState()', 'mono')])


def draw_hardware(figure: Figure, step: LoggedStep) -> None:
    """Add the motor driver, the motor with the wheel and the encoder (signals 3 to 6)."""
    figure.arrow(f'M{FORWARD_X},232 V303')
    figure.signal(FORWARD_X + 8, 270, 3, 'motor command',
                  f'{step.motor_command} (\u2212{PWM_MAX} to {PWM_MAX})')
    figure.box(30, 304, 150, 40, 'hardware', [
        ('Motor driver', 'name'), ('Adafruit FeatherWing', 'note')])
    figure.arrow(f'M{FORWARD_X},344 V397')
    figure.signal(FORWARD_X + 8, 364, 4, 'PWM voltage to the motor',
                  f'on {on_time_percent(step.motor_command):.1f} % of the time')
    figure.box(30, 398, 150, 40, 'hardware', [
        ('Motor and wheel', 'name'), ('DG01D-E gear motor', 'note')])
    figure.arrow(f'M180,418 H{FEEDBACK_X} V345')
    figure.signal(186, 412, 5, 'rotation')
    figure.box(200, 304, 130, 40, 'hardware', [
        ('Encoder', 'name'), (f'{ENCODER_RESOLUTION} ticks per wheel turn', 'note')])
    figure.arrow(f'M{FEEDBACK_X},304 V233')
    figure.signal(FEEDBACK_X + 6, 270, 6, 'ticks',
                  f'{ticks_in_period(step)} in {1000 // CONTROL_RATE} ms')


def feedback_loop(step: LoggedStep) -> Figure:
    """Return the figure of one wheel's speed control loop with the step's values."""
    figure = Figure("One wheel's speed control loop in the Teensy firmware, with the motor "
                    'driver, the motor, the wheel and the encoder')
    draw_target(figure, step)
    draw_firmware(figure, step)
    draw_hardware(figure, step)
    figure.text(WIDTH / 2, 458, "Blue: one control step of Remo's left wheel", 'value', 'middle')
    figure.text(WIDTH / 2, 471, 'The ticks (6) are calculated from the measured speed (7)',
                'value', 'middle')
    return figure


def main() -> None:
    """Write the include with the figure and its caption."""
    check(LOGGED_STEP)
    caption = ("One wheel's speed control loop. Start at the target wheel speed (1) and follow "
               'the numbers around the loop, back to the comparison in the circle. The boxes '
               'inside the Teensy firmware are code. The motor driver, the motor with the wheel '
               'and the encoder are hardware.')
    with open('includes/pid-feedback-loop.html', 'w', encoding='utf-8') as include:
        include.write(f'{style_element(RULES)}\n<figure class="pid-loop-figure">\n'
                      f'{feedback_loop(LOGGED_STEP).svg()}\n'
                      f'<figcaption>{caption}</figcaption>\n</figure>\n')


if __name__ == '__main__':
    main()
