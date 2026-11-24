"""Generate the figures on docs/packages/diffbot_base/low-level.md.

It writes two includes, each with a wide layout and a tall one for phones, as inline SVG that
follows the site's light and dark colors. CSS shows one of the layouts.

- includes/low-level-wiring.html: what Remo's microcontroller (MCU), a Teensy 4.0, connects to,
  and over which wire: the robot's computer over USB, the BNO055 and the motor FeatherWing over
  the I2C bus, the motors through the FeatherWing's outputs, and the motors' encoders on four pins.
- includes/low-level-ports.html: WheelControl, the firmware's core, with its ports on its edge and
  the adapters that connect them, once as the MCU runs it and once as the unit tests run it.

The names, pins and motor outputs come from the firmware at the commit that the page pins:
WheelControl's template parameters and functions in diffbot_firmware/lib/wheel_control/
wheel_control.h, the adapters that src/main.cpp defines or connects, the fakes in test/fakes/
fake_base_controller.h, which test/test_wheel_control/test_wheel_control.cpp uses, and the pins
and outputs in lib/config/diffbot_base_config.h. The I2C pins are the ones of the Teensy's bus
Wire, which the BNO055's page lists.

Run from the repository root after changing a figure:

    python3 scripts/figures/low_level_figures.py
"""

from enum import Enum
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

from board_drawings import board_rules
from board_drawings import draw_bno055_breakout
from board_drawings import draw_gear_motor_with_wheel
from board_drawings import draw_motor_driver_board
from board_drawings import draw_raspberry_pi
from board_drawings import draw_teensy
from board_drawings import Drawing
from board_drawings import ENCODER_BOARD_OFFSET
from board_drawings import style_element
from board_drawings import svg_element

SCRIPT = 'scripts/figures/low_level_figures.py'

# Screens at least WIDE_FROM wide get the wide layouts, which fit the page's article
WIDE_FROM = '60em'
WIDE_WIDTH = 680
TALL_WIDTH = 360

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
CODE_FONT = 'var(--md-code-font-family,"Roboto Mono"),monospace'

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]


class Point(NamedTuple):
    """A point of a layout, in px from the top left corner."""

    x: float
    y: float


class Box(NamedTuple):
    """A rectangle of a layout: its top left corner, its width and its height."""

    x: float
    y: float
    width: float
    height: float


class LineKind(Enum):
    """The kinds of wire in the wiring figure, each with its CSS class."""

    USB = 'usb'
    I2C = 'i2c'
    MOTOR_OUTPUT = 'motor-output'
    ENCODER = 'encoder-signal'


def path_through(points: Sequence[Point]) -> str:
    """Return an SVG path along straight lines through the points."""
    return ' '.join(f'{"M" if index == 0 else "L"}{x:g},{y:g}'
                    for index, (x, y) in enumerate(points))


class Figure(Drawing):
    """Collect the SVG elements of one layout of a figure.

    name is the layout's CSS class, such as ll-ports-wide, and the prefix of its IDs. css_class is
    the class that the layouts of one figure share, such as ll-ports.
    """

    def __init__(self, name: str, css_class: str, width: float, height: float) -> None:
        super().__init__()
        self.name = name
        self.css_class = css_class
        self.width = width
        self.height = height

    def polygon(self, points: Sequence[Point], css_class: str) -> None:
        """Add a closed shape through the points."""
        corners = ' '.join(f'{x:g},{y:g}' for x, y in points)
        self.elements.append(f'<polygon class="{css_class}" points="{corners}"/>')

    def arrow(self, points: Sequence[Point]) -> None:
        """Add a line through the points, with an arrowhead at the last one."""
        self.elements.append(f'<path class="arrow" d="{path_through(points)}" '
                             f'marker-end="url(#{self.name}-head)"/>')

    def wire(self, points: Sequence[Point], kind: LineKind) -> None:
        """Add a wire of the given kind through the points."""
        self.line(path_through(points), kind.value)

    def svg(self, title: str) -> str:
        """Return the layout as an SVG element whose accessible name is title."""
        return svg_element(self.name, f'{self.css_class} {self.name}', (self.width, self.height),
                           title, self.elements)


def layout_rules(css_class: str) -> Tuple[Tuple[str, str], ...]:
    """Return the CSS rules that size a figure, show one of its layouts and style its text."""
    return (
        (f'.md-typeset figure.{css_class}-figure', 'margin:1em auto;width:100%;max-width:480px'),
        (f'.{css_class}', 'width:100%;height:auto;'
                          'font-family:var(--md-text-font-family,Roboto),sans-serif'),
        (f'.{css_class}-wide', 'display:none'),
        (f'@media screen and (min-width:{WIDE_FROM})',
         f'.md-typeset figure.{css_class}-figure{{max-width:none}}'
         f'.{css_class}-wide{{display:block}}.{css_class}-tall{{display:none}}'),
        (f'.{css_class} text', f'fill:{FG};font-size:12px'),
        (f'.{css_class} .name', 'font-size:12.5px'),
        (f'.{css_class} .note', f'font-size:11px;fill:{FG_LIGHT}'),
        (f'.{css_class} .container-name', 'font-weight:700'),
        (f'.{css_class} .code', f'font-family:{CODE_FONT};font-size:11px'),
    )


def labels(figure: Figure, x: float, y: float, lines: Sequence[Line],
           anchor: str = 'middle') -> None:
    """Add text lines from (x, y) down, 14 px apart."""
    for index, (text, css_class) in enumerate(lines):
        figure.text(x, y + index * 14, text, css_class, anchor)


# The wiring figure: Remo's MCU in the middle, the robot's computer on one side and the devices
# on the other. The MCU's connections are tabs on the edge of its box. The figure names the MCU's
# role, not the board, and draws Remo's Teensy 4.0. The caption names the board

class Motor(NamedTuple):
    """One of Remo's motors: its side, the FeatherWing output and the encoder pins it uses."""

    side: str
    output: str
    encoder_pins: Tuple[int, int]


MOTORS = (Motor('left', 'M4', (5, 6)), Motor('right', 'M3', (7, 8)))
WIRE_PINS = 'SDA 18, SCL 19'
ENCODER_PINS = f'pins {MOTORS[0].encoder_pins[0]} to {MOTORS[-1].encoder_pins[-1]}'
MCU_NAME = 'Microcontroller (MCU)'
WIRING_TITLE = "What Remo's microcontroller (MCU) connects to"
WIRING_RULES = (
    *layout_rules('ll-wiring'),
    ('.ll-wiring .mcu', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.ll-wiring .tab', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.ll-wiring .usb', f'fill:none;stroke:{FG};stroke-width:3.5;stroke-linecap:round'),
    ('.ll-wiring .i2c', f'fill:none;stroke:{FG};stroke-width:2;stroke-dasharray:7 3'),
    ('.ll-wiring .motor-output', f'fill:none;stroke:{FG};stroke-width:2;'
                                 'stroke-dasharray:10 3 2 3'),
    ('.ll-wiring .encoder-signal', f'fill:none;stroke:{FG};stroke-width:2;'
                                   'stroke-dasharray:2 3'),
    ('.ll-wiring .wire-label', 'font-size:11px;font-weight:700'),
    *board_rules('.ll-wiring'),
)


def motor_lines() -> List[Line]:
    """Return the motor's label: what it is, then each motor's output and encoder pins."""
    lines: List[Line] = [('Two DG01D-E with encoders', 'name')]
    for motor in MOTORS:
        first, second = motor.encoder_pins
        lines.append((f'{motor.side}: {motor.output}, pins {first} and {second}', 'note'))
    return lines


def output_names() -> str:
    """Return the FeatherWing's motor outputs that the firmware uses, such as M4 and M3."""
    return ' and '.join(motor.output for motor in MOTORS)


def mcu(figure: Figure, box: Box, tabs: Sequence[Tuple[Box, str]], centered: bool) -> None:
    """Add the MCU's box with its name, the drawing of Remo's board and its connections as tabs.

    The name and the drawing stand in the middle of the box if centered is true, else at its left.
    """
    figure.rect(*box, 'mcu', 6)
    if centered:
        middle = box.x + box.width / 2
        figure.text(middle, box.y + 22, MCU_NAME, 'container-name', 'middle')
        draw_teensy(figure, middle - 21, box.y + 32)
    else:
        figure.text(box.x + 14, box.y + 22, MCU_NAME, 'container-name')
        draw_teensy(figure, box.x + 14, box.y + 32)
    for tab, text in tabs:
        figure.box(*tab, 'tab', [(text, 'code')])


def wide_wiring() -> Figure:
    """Return the wide layout: the computer, the MCU and the devices from left to right."""
    figure = Figure('ll-wiring-wide', 'll-wiring', WIDE_WIDTH, 350)
    draw_raspberry_pi(figure, 40, 125)
    labels(figure, 64, 186, [("Robot's computer", 'name'), ('single-board computer', 'note')])
    usb_tab = Box(150, 138, 44, 22)
    wire_tab = Box(296, 118, 104, 22)
    encoder_tab = Box(296, 178, 104, 22)
    mcu(figure, Box(170, 66, 170, 150),
        [(usb_tab, 'USB'), (wire_tab, WIRE_PINS), (encoder_tab, ENCODER_PINS)], False)
    figure.text(290, 112, 'Wire', 'note', 'end')
    figure.text(290, 172, 'encoders', 'note', 'end')
    figure.wire((Point(88, 149), Point(usb_tab.x, 149)), LineKind.USB)
    figure.text(119, 167, 'USB cable', 'wire-label', 'middle')
    bus_y = wire_tab.y + wire_tab.height / 2
    figure.wire((Point(wire_tab.x + wire_tab.width, bus_y), Point(460, bus_y)), LineKind.I2C)
    figure.wire((Point(430, bus_y), Point(430, 54), Point(460, 54)), LineKind.I2C)
    figure.text(424, 92, 'I2C bus', 'wire-label', 'end')
    draw_bno055_breakout(figure, 460, 40)
    labels(figure, 492, 52, [('BNO055', 'name'), ('IMU breakout', 'note')], 'start')
    draw_motor_driver_board(figure, 460, 118)
    labels(figure, 514, 126, [('FeatherWing', 'name'), ('motor driver', 'note')], 'start')
    motor = Point(520, 200)
    output_x = 482
    figure.wire((Point(output_x, 140), Point(output_x, motor.y + 11),
                 Point(motor.x, motor.y + 11)), LineKind.MOTOR_OUTPUT)
    labels(figure, output_x + 6, 170, [('motor outputs', 'wire-label'),
                                       (output_names(), 'note')], 'start')
    draw_gear_motor_with_wheel(figure, *motor)
    encoder_right = motor.x + ENCODER_BOARD_OFFSET + 4
    encoder_y = encoder_tab.y + encoder_tab.height / 2
    figure.wire((Point(encoder_right, motor.y + 11), Point(640, motor.y + 11), Point(640, 270),
                 Point(420, 270), Point(420, encoder_y),
                 Point(encoder_tab.x + encoder_tab.width, encoder_y)), LineKind.ENCODER)
    figure.text(530, 286, 'encoder signals', 'wire-label', 'middle')
    labels(figure, 530, 306, motor_lines())
    return figure


def tall_wiring() -> Figure:
    """Return the tall layout: the computer, the MCU and the devices from top to bottom."""
    figure = Figure('ll-wiring-tall', 'll-wiring', TALL_WIDTH, 462)
    draw_raspberry_pi(figure, 40, 14)
    labels(figure, 100, 28, [("Robot's computer", 'name'), ('single-board computer', 'note')],
           'start')
    usb_tab = Box(42, 73, 44, 22)
    wire_tab = Box(30, 193, 120, 22)
    encoder_tab = Box(232, 193, 116, 22)
    mcu(figure, Box(20, 84, 320, 120),
        [(usb_tab, 'USB'), (wire_tab, WIRE_PINS), (encoder_tab, ENCODER_PINS)], True)
    figure.text(90, 186, 'Wire', 'note', 'middle')
    figure.text(290, 186, 'encoders', 'note', 'middle')
    figure.wire((Point(64, 46), Point(64, usb_tab.y)), LineKind.USB)
    figure.text(72, 62, 'USB cable', 'wire-label')
    pins_bottom = wire_tab.y + wire_tab.height
    figure.wire((Point(90, pins_bottom), Point(90, 240)), LineKind.I2C)
    figure.wire((Point(41, 262), Point(41, 240), Point(152, 240), Point(152, 266)), LineKind.I2C)
    figure.text(96, 234, 'I2C bus', 'wire-label')
    draw_bno055_breakout(figure, 30, 262)
    labels(figure, 41, 306, [('BNO055', 'name'), ('IMU breakout', 'note')])
    draw_motor_driver_board(figure, 130, 266)
    labels(figure, 152, 306, [('FeatherWing', 'name'), ('motor driver', 'note')])
    motor = Point(230, 330)
    figure.wire((Point(174, 277), Point(200, 277), Point(200, motor.y + 11),
                 Point(motor.x, motor.y + 11)), LineKind.MOTOR_OUTPUT)
    labels(figure, 206, 296, [('motor outputs', 'wire-label'), (output_names(), 'note')],
           'start')
    draw_gear_motor_with_wheel(figure, *motor)
    encoder_x = motor.x + ENCODER_BOARD_OFFSET
    figure.wire((Point(encoder_x, motor.y), Point(encoder_x, pins_bottom)), LineKind.ENCODER)
    figure.text(encoder_x - 6, 262, 'encoder signals', 'wire-label', 'end')
    labels(figure, 260, 410, motor_lines())
    return figure


def wiring_caption() -> str:
    """Return the wiring figure's caption, which both layouts share."""
    return ("Remo's microcontroller (MCU), here a Teensy 4.0, and what it connects to. The thick "
            "line is the USB cable to the robot's computer. The dashed lines are the I2C bus "
            '<code>Wire</code>, which the BNO055 and the motor FeatherWing share. The dash-dotted '
            "line is the FeatherWing's motor outputs, and the dotted line the encoder signals "
            'back to the MCU. The figure draws one of the two motors.')


# The ports figure: WheelControl as a hexagon, the shape that gives the pattern its other name,
# with its ports on its edge

class Environment(NamedTuple):
    """Where WheelControl runs, and the adapters there.

    primary holds the code that calls the primary ports, one name for each port or a single name
    that calls both. secondary holds the code that implements each secondary port.
    """

    title: str
    primary: Tuple[Sequence[Line], ...]
    secondary: Tuple[str, str]


PRIMARY_PORTS = ('receiveCommand()', 'step()')
SECONDARY_PORTS = ('Controller', 'LogTimeout')
ENVIRONMENTS = (
    Environment('On the microcontroller',
                ((('RosWheelCommandSubscriber', 'code'),), (('loop()', 'code'),)),
                ('BaseController', 'RosTimeoutLogger')),
    Environment('In the unit tests',
                ((('Tests in', ''), ('test_wheel_control.cpp', 'code')),),
                ('FakeBaseController', 'FakeTimeoutLog')),
)
PORT_SIZE = 12
PORTS_TITLE = "WheelControl's ports on the microcontroller and in the unit tests"
PORTS_RULES = (
    *layout_rules('ll-ports'),
    ('.ll-ports .environment', 'fill:var(--md-default-fg-color--lightest);'
                               'stroke:var(--md-default-fg-color--lighter);stroke-width:1'),
    ('.ll-ports .core', f'fill:var(--md-default-bg-color);stroke:{FG};stroke-width:1.5'),
    ('.ll-ports .adapter', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.ll-ports .port', f'fill:{FG}'),
    ('.ll-ports .arrow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.ll-ports .arrow-head', f'fill:{FG_LIGHT}'),
    ('.ll-ports .core-name', f'font-family:{CODE_FONT};font-size:13px;font-weight:700'),
    ('.ll-ports-tall .code', 'font-size:10.5px'),
)


def port(figure: Figure, center: Point) -> None:
    """Add a port: a small square on the core's edge, centered on the edge."""
    half = PORT_SIZE / 2
    figure.rect(center.x - half, center.y - half, PORT_SIZE, PORT_SIZE, 'port', 1.5)


# The wide layout: the environments are rows. In each, the code that calls the primary ports
# stands on the left, the core in the middle and the code of the secondary ports on the right.
# The core is a hexagon with a corner at the top and bottom, and the ports sit on its two upright
# edges, PORT_GAP apart
WIDE_ROW_HEIGHT = 186
WIDE_ROW_GAP = 10
WIDE_TOP = 42
WIDE_CORE_X = 340
WIDE_CORE_WIDTH = 220
WIDE_CORE_HEIGHT = 170
WIDE_ADAPTER_WIDTH = 180
WIDE_LEFT_X = 16
WIDE_RIGHT_X = WIDE_WIDTH - 16 - WIDE_ADAPTER_WIDTH
PORT_GAP = 36


def wide_environment(figure: Figure, environment: Environment, top: float) -> None:
    """Add one environment as a row of the wide layout, from top down."""
    figure.rect(4, top, WIDE_WIDTH - 8, WIDE_ROW_HEIGHT, 'environment', 6)
    figure.text(14, top + 18, environment.title, 'container-name')
    center = Point(WIDE_CORE_X, top + WIDE_ROW_HEIGHT / 2 + 4)
    half_width, half_height = WIDE_CORE_WIDTH / 2, WIDE_CORE_HEIGHT / 2
    figure.polygon((Point(center.x, center.y - half_height),
                    Point(center.x + half_width, center.y - half_height / 2),
                    Point(center.x + half_width, center.y + half_height / 2),
                    Point(center.x, center.y + half_height),
                    Point(center.x - half_width, center.y + half_height / 2),
                    Point(center.x - half_width, center.y - half_height / 2)), 'core')
    figure.text(center.x, center.y - half_height / 2 - 6, 'WheelControl', 'core-name', 'middle')
    figure.text(center.x, center.y - half_height / 2 + 8, 'and its CommandTimeout', 'note',
                'middle')
    port_ys = (center.y - PORT_GAP / 2, center.y + PORT_GAP / 2)
    left_edge, right_edge = center.x - half_width, center.x + half_width
    adapter_right = WIDE_LEFT_X + WIDE_ADAPTER_WIDTH
    for name, y in zip(PRIMARY_PORTS, port_ys):
        port(figure, Point(left_edge, y))
        figure.text(left_edge + 12, y + 4, name, 'code')
        figure.arrow((Point(adapter_right, y), Point(left_edge - PORT_SIZE / 2, y)))
    for name, y in zip(SECONDARY_PORTS, port_ys):
        port(figure, Point(right_edge, y))
        figure.text(right_edge - 12, y + 4, name, 'code', 'end')
        figure.arrow((Point(right_edge + PORT_SIZE / 2, y), Point(WIDE_RIGHT_X, y)))
    if len(environment.primary) == len(PRIMARY_PORTS):
        for lines, y in zip(environment.primary, port_ys):
            figure.box(WIDE_LEFT_X, y - 15, WIDE_ADAPTER_WIDTH, 30, 'adapter', lines)
    else:
        figure.box(WIDE_LEFT_X, port_ys[0] - 15, WIDE_ADAPTER_WIDTH, PORT_GAP + 30, 'adapter',
                   environment.primary[0])
    for name, y in zip(environment.secondary, port_ys):
        figure.box(WIDE_RIGHT_X, y - 15, WIDE_ADAPTER_WIDTH, 30, 'adapter', [(name, 'code')])


def wide_ports() -> Figure:
    """Return the wide layout: the calls go from left to right, one row per environment."""
    height = WIDE_TOP + len(ENVIRONMENTS) * (WIDE_ROW_HEIGHT + WIDE_ROW_GAP) - WIDE_ROW_GAP + 4
    figure = Figure('ll-ports-wide', 'll-ports', WIDE_WIDTH, height)
    left_middle = WIDE_LEFT_X + WIDE_ADAPTER_WIDTH / 2
    right_middle = WIDE_RIGHT_X + WIDE_ADAPTER_WIDTH / 2
    figure.text(left_middle, 16, 'Primary ports:', 'container-name', 'middle')
    figure.text(left_middle, 31, 'an adapter calls the core', 'note', 'middle')
    figure.text(right_middle, 16, 'Secondary ports:', 'container-name', 'middle')
    figure.text(right_middle, 31, 'the core calls an adapter', 'note', 'middle')
    for index, environment in enumerate(ENVIRONMENTS):
        wide_environment(figure, environment, WIDE_TOP + index * (WIDE_ROW_HEIGHT + WIDE_ROW_GAP))
    return figure


# The tall layout: the environments are blocks, one below the other. In each, the code that calls
# the primary ports stands at the top, the core in the middle and the code of the secondary ports
# at the bottom. The core is a hexagon with a corner at the left and right, and the ports sit on
# its top and bottom edges, below and above the adapters' middles
TALL_BLOCK_HEIGHT = 276
TALL_BLOCK_GAP = 10
TALL_ADAPTER_WIDTH = 168
TALL_ADAPTER_XS = (8, TALL_WIDTH - 8 - TALL_ADAPTER_WIDTH)
TALL_CORE_HEIGHT = 98
TALL_CORE_SLANT = 62


def tall_environment(figure: Figure, environment: Environment, top: float) -> None:
    """Add one environment as a block of the tall layout, from top down."""
    figure.rect(4, top, TALL_WIDTH - 8, TALL_BLOCK_HEIGHT, 'environment', 6)
    figure.text(14, top + 18, environment.title, 'container-name')
    port_xs = [x + TALL_ADAPTER_WIDTH / 2 for x in TALL_ADAPTER_XS]
    adapter_top = top + 28
    core_top = adapter_top + 30 + 42
    core_bottom = core_top + TALL_CORE_HEIGHT
    lower_top = core_bottom + 42
    if len(environment.primary) == len(PRIMARY_PORTS):
        for lines, adapter_x in zip(environment.primary, TALL_ADAPTER_XS):
            figure.box(adapter_x, adapter_top, TALL_ADAPTER_WIDTH, 30, 'adapter', lines)
    else:
        figure.box(TALL_ADAPTER_XS[0], adapter_top - 4,
                   TALL_ADAPTER_XS[1] + TALL_ADAPTER_WIDTH - TALL_ADAPTER_XS[0], 38, 'adapter',
                   environment.primary[0])
    left, right = 8, TALL_WIDTH - 8
    middle = (core_top + core_bottom) / 2
    figure.polygon((Point(left + TALL_CORE_SLANT, core_top),
                    Point(right - TALL_CORE_SLANT, core_top), Point(right, middle),
                    Point(right - TALL_CORE_SLANT, core_bottom),
                    Point(left + TALL_CORE_SLANT, core_bottom), Point(left, middle)), 'core')
    figure.text(TALL_WIDTH / 2, core_top + 15, 'primary ports', 'note', 'middle')
    figure.text(TALL_WIDTH / 2, middle + 4, 'WheelControl', 'core-name', 'middle')
    figure.text(TALL_WIDTH / 2, middle + 19, 'and its CommandTimeout', 'note', 'middle')
    figure.text(TALL_WIDTH / 2, core_bottom - 7, 'secondary ports', 'note', 'middle')
    for name, port_x in zip(PRIMARY_PORTS, port_xs):
        port(figure, Point(port_x, core_top))
        figure.arrow((Point(port_x, adapter_top + 30), Point(port_x, core_top - PORT_SIZE / 2)))
        figure.text(port_x + 6, core_top - 18, name, 'code')
    for name, port_x in zip(SECONDARY_PORTS, port_xs):
        port(figure, Point(port_x, core_bottom))
        figure.arrow((Point(port_x, core_bottom + PORT_SIZE / 2), Point(port_x, lower_top)))
        figure.text(port_x + 6, core_bottom + 26, name, 'code')
    for name, adapter_x in zip(environment.secondary, TALL_ADAPTER_XS):
        figure.box(adapter_x, lower_top, TALL_ADAPTER_WIDTH, 30, 'adapter', [(name, 'code')])


def tall_ports() -> Figure:
    """Return the tall layout: the calls go from top to bottom, one block per environment."""
    height = len(ENVIRONMENTS) * (TALL_BLOCK_HEIGHT + TALL_BLOCK_GAP) - TALL_BLOCK_GAP + 8
    figure = Figure('ll-ports-tall', 'll-ports', TALL_WIDTH, height)
    for index, environment in enumerate(ENVIRONMENTS):
        tall_environment(figure, environment, 4 + index * (TALL_BLOCK_HEIGHT + TALL_BLOCK_GAP))
    return figure


def ports_caption() -> str:
    """Return the ports figure's caption, which both layouts share."""
    return ('<code>WheelControl</code>, the core, with its ports on its edge. An arrow points '
            'from the code that calls to the code that is called: adapters call the primary '
            'ports, and the core calls the secondary ports, which adapters implement. The core is '
            'the same on the microcontroller and in the unit tests. Only the adapters change.')


def write_include(path: str, rules: Sequence[Tuple[str, str]], figure_class: str,
                  layouts: Sequence[str], caption: str) -> None:
    """Write an include: a comment that names this script, the style, then the figure."""
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'<!-- Generated by {SCRIPT}. Change the script and run it. -->\n'
                      f'{style_element(rules)}\n<figure class="{figure_class}">\n'
                      + '\n'.join(layouts)
                      + f'\n<figcaption>{caption}</figcaption>\n</figure>\n')


def main() -> None:
    """Write the includes of the Low-Level page's generated figures."""
    write_include('includes/low-level-wiring.html', WIRING_RULES, 'll-wiring-figure',
                  [wide_wiring().svg(WIRING_TITLE), tall_wiring().svg(WIRING_TITLE)],
                  wiring_caption())
    write_include('includes/low-level-ports.html', PORTS_RULES, 'll-ports-figure',
                  [wide_ports().svg(PORTS_TITLE), tall_ports().svg(PORTS_TITLE)],
                  ports_caption())


if __name__ == '__main__':
    main()
