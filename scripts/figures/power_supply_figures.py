"""Generate the power flow figures on docs/power-supply.md.

Both are inline SVG that follows the site's light and dark colors:

- includes/power-supply-flow-diffbot.html: DiffBot's two supplies and the parts on them. It has
  the styles for both figures;
- includes/power-supply-flow-remo.html: Remo's two supplies and the parts on them.

A thick line is a supply, in the color of its source, with its voltage next to it. A dashed line is
a data connection. A figure draws a supply only where it is checked on the robot or in the board's
schematic. On Remo, the camera has only its documented data connection. DiffBot's figure leaves out
every part whose supply isn't checked.

Run from the repository root after changing a figure:

    python3 scripts/figures/power_supply_figures.py
"""

from typing import List
from typing import Sequence
from typing import Tuple

WIDTH = 360
# The parts on the power bank are in the left column, the motor battery's parts in the right one
LEFT_X = 10
LEFT_WIDTH = 140
RIGHT_X = 220
RIGHT_WIDTH = 130
# The supply and the data line of one cable run side by side, this far apart
PAIR_GAP = 7
# A supply line ends this far before the part it feeds, and its arrowhead's tip closes the gap
HEAD_TIP = 5

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]
# A legend entry: the CSS class of its line sample and its text
LegendEntry = Tuple[str, str]

LEGEND_BANK: LegendEntry = ('bank', 'supply from the power bank')
LEGEND_BATTERY: LegendEntry = ('battery', 'supply from the motor battery')
LEGEND_DATA: LegendEntry = ('data', 'data connection')
LEGEND_GROUND: LegendEntry = ('ground', 'ground wire (GND)')

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.pf-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.pf-fig', 'width:100%;height:auto;--pf-bank:#1e63c4;--pf-battery:#b45309;'
                'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .pf-fig', '--pf-bank:#82b1ff;--pf-battery:#ffab70'),
    ('.pf-fig text', f'fill:{FG};font-size:10px'),
    ('.pf-fig .part', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.pf-fig .source', 'fill:var(--md-code-bg-color);stroke-width:2'),
    ('.pf-fig .source.bank', 'stroke:var(--pf-bank)'),
    ('.pf-fig .source.battery', 'stroke:var(--pf-battery)'),
    ('.pf-fig .line', 'fill:none'),
    ('.pf-fig .line.bank', 'stroke:var(--pf-bank);stroke-width:3'),
    ('.pf-fig .line.battery', 'stroke:var(--pf-battery);stroke-width:3'),
    ('.pf-fig .line.data', f'stroke:{FG_LIGHT};stroke-width:1.3;stroke-dasharray:4 3'),
    ('.pf-fig .line.ground', f'stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.pf-fig .head-bank', 'fill:var(--pf-bank)'),
    ('.pf-fig .head-battery', 'fill:var(--pf-battery)'),
    ('.pf-fig .name', 'font-size:11px'),
    ('.pf-fig .note', f'font-size:9.5px;fill:{FG_LIGHT}'),
    ('.pf-fig .volt-bank', 'fill:var(--pf-bank);font-weight:700'),
    ('.pf-fig .volt-battery', 'fill:var(--pf-battery);font-weight:700'),
)


def escape(text: str) -> str:
    """Return text with the characters escaped that have a meaning in SVG."""
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def style_element(rules: Sequence[Tuple[str, str]]) -> str:
    """Return a style element with one CSS rule per (selector, declarations) pair."""
    lines = [f'{selector}{{{declarations}}}' for selector, declarations in rules]
    return '<style>\n' + '\n'.join(lines) + '\n</style>'


class Figure:
    """Collect the SVG elements of one figure."""

    def __init__(self, name: str, height: int, title: str) -> None:
        self.name = name
        self.height = height
        self.title = title
        self.elements: List[str] = []

    def box(self, x: float, y: float, width: float, height: float, lines: Sequence[Line],
            css_class: str = 'part') -> None:
        """Add a box with its text lines centered in it, 13 px apart."""
        self.elements.append(f'<rect class="{css_class}" x="{x}" y="{y}" width="{width}" '
                             f'height="{height}" rx="5"/>')
        for index, (text, text_class) in enumerate(lines):
            baseline = y + height / 2 + (index - (len(lines) - 1) / 2) * 13 + 4
            self.text(x + width / 2, baseline, text, text_class, 'middle')

    def line(self, path: str, kind: str) -> None:
        """Add a line along an SVG path. kind is bank, battery, data or ground."""
        self.elements.append(f'<path class="line {kind}" d="{path}"/>')

    def supply(self, path: str, source: str) -> None:
        """Add a supply line with an arrowhead toward the part it feeds.

        source is bank or battery. The arrowhead's tip is HEAD_TIP px beyond the path's end.
        """
        self.elements.append(f'<path class="line {source}" d="{path}" '
                             f'marker-end="url(#{self.name}-{source}-head)"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of text. anchor is start, middle or end."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(
            f'<text{class_attribute} x="{x}" y="{y}" text-anchor="{anchor}">{escape(text)}</text>')

    def legend(self, y: float, entries: Sequence[LegendEntry]) -> None:
        """Add the legend's rows below y: a sample of each line and what it means."""
        for row, (kind, meaning) in enumerate(entries):
            baseline = y + 14 * row
            self.line(f'M{LEFT_X},{baseline - 3.5} H{LEFT_X + 26}', kind)
            self.text(LEFT_X + 34, baseline, meaning)

    def svg(self) -> str:
        """Return the figure as an SVG element."""
        heads = ''.join(
            f'<marker id="{self.name}-{source}-head" viewBox="0 0 10 10" refX="{10 - HEAD_TIP}" '
            'refY="5" markerUnits="userSpaceOnUse" markerWidth="10" markerHeight="10" '
            f'orient="auto"><path class="head-{source}" d="M0,0 L10,5 L0,10 z"/></marker>'
            for source in ('bank', 'battery'))
        return (f'<svg class="pf-fig" viewBox="0 0 {WIDTH} {self.height}" role="img" '
                f'aria-labelledby="{self.name}-title"><title id="{self.name}-title">'
                f'{escape(self.title)}</title><defs>{heads}</defs>\n'
                + '\n'.join(self.elements) + '\n</svg>')


def draw_power_bank_and_pi(figure: Figure) -> None:
    """Add the power bank and the Raspberry Pi on it."""
    figure.box(LEFT_X, 6, LEFT_WIDTH, 38, [('USB power bank', 'name')], 'source bank')
    figure.supply(f'M60,44 V{70 - HEAD_TIP}', 'bank')
    figure.text(69, 61, '5 V, USB-C', 'volt-bank')
    figure.box(LEFT_X, 70, LEFT_WIDTH, 60, [('Raspberry Pi 4 B', 'name')])


def draw_lidar_and_camera(figure: Figure) -> None:
    """Add the LiDAR and the camera on the Raspberry Pi.

    The LiDAR's USB adapter gets the supply and the data over one cable. The camera has a data
    line only.
    """
    left_edge, box_x = LEFT_X + LEFT_WIDTH, 196
    figure.supply(f'M{left_edge},79 H{box_x - HEAD_TIP}', 'bank')
    figure.line(f'M{left_edge},{79 + PAIR_GAP} H{box_x}', 'data')
    figure.text((left_edge + box_x) / 2, 73, '5 V', 'volt-bank', 'middle')
    figure.box(box_x, 64, 122, 34, [('RPLidar A2M8', 'name'), ('USB adapter', 'note')])
    figure.line(f'M{left_edge},118 H{box_x}', 'data')
    figure.box(box_x, 106, 122, 34, [('Raspberry Pi Camera', 'name'), ('camera cable', 'note')])


def draw_battery_and_motors(figure: Figure, battery: Sequence[Line], voltage: str,
                            driver: Sequence[Line], driver_y: float = 186) -> None:
    """Add the motor battery, the motor driver on it and the motors.

    battery and driver are the lines of text in the battery's and the motor driver's box,
    voltage is the battery's nominal voltage, and driver_y is the top of the motor driver's box.
    """
    figure.box(RIGHT_X, 6, RIGHT_WIDTH, 38, battery, 'source battery')
    figure.supply(f'M338,44 V{driver_y - HEAD_TIP}', 'battery')
    figure.text(330, driver_y - 20, voltage, 'volt-battery', 'end')
    figure.box(RIGHT_X, driver_y, RIGHT_WIDTH, 54, driver)
    figure.supply(f'M285,{driver_y + 54} V{driver_y + 104 - HEAD_TIP}', 'battery')
    figure.text(277, driver_y + 77, 'switched on and off', 'volt-battery', 'end')
    figure.text(277, driver_y + 89, 'by the driver (PWM)', 'volt-battery', 'end')
    figure.box(RIGHT_X, driver_y + 104, RIGHT_WIDTH, 38, [('Two DG01D-E motors', 'name')])


def remo() -> Figure:
    """Return Remo's figure: every part's supply is checked."""
    figure = Figure('pf-remo', 400, "Remo's power flow: the power bank supplies the Raspberry Pi, "
                    'the LiDAR, the Teensy and its sensors, and the six NiMH cells supply the '
                    'motors')
    draw_power_bank_and_pi(figure)
    draw_lidar_and_camera(figure)
    draw_battery_and_motors(figure, [('Six NiMH AA cells', 'name')], '7.2 V nominal', [
        ('Motor driver', 'name'), ('Adafruit FeatherWing', 'note'),
        ('joins battery − and GND', 'note')])

    # The Teensy on the Raspberry Pi's USB port: one cable for the supply and the data
    figure.supply(f'M60,130 V{186 - HEAD_TIP}', 'bank')
    figure.line(f'M{60 + PAIR_GAP},130 V186', 'data')
    figure.text(76, 162, '5 V and data, USB', 'volt-bank')
    figure.box(LEFT_X, 186, LEFT_WIDTH, 54, [('Teensy 4.0', 'name')])

    # The motor driver's logic on the Teensy: 3.3 V, I2C and ground
    left_edge, right_edge = LEFT_X + LEFT_WIDTH, RIGHT_X
    middle = (left_edge + right_edge) / 2
    figure.supply(f'M{left_edge},199 H{right_edge - HEAD_TIP}', 'bank')
    figure.text(middle, 194, '3.3 V', 'volt-bank', 'middle')
    figure.line(f'M{left_edge},214 H{right_edge}', 'data')
    figure.text(middle, 210, 'I2C', 'note', 'middle')
    figure.line(f'M{left_edge},229 H{right_edge}', 'ground')
    figure.text(middle, 225, 'GND', 'note', 'middle')

    # The Teensy's sensors
    for x, width, lines in ((LEFT_X, 66, [('Encoders', 'name'), ('both wheels', 'note')]),
                            (LEFT_X + 74, 66, [('IMU', 'name'), ('BNO055', 'note')])):
        figure.supply(f'M{x + 18},240 V{290 - HEAD_TIP}', 'bank')
        figure.line(f'M{x + 18 + PAIR_GAP},240 V290', 'data')
        figure.text(x + 31, 269, '3.3 V', 'volt-bank')
        figure.box(x, 290, width, 38, lines)

    figure.legend(352, (LEGEND_BANK, LEGEND_BATTERY, LEGEND_DATA, LEGEND_GROUND))
    return figure


def diffbot() -> Figure:
    """Return DiffBot's figure: its two supplies, which are the parts that are checked."""
    figure = Figure('pf-diffbot', 292, "DiffBot's power flow: the power bank supplies the "
                    'Raspberry Pi, and the four AA batteries supply the motors')
    draw_power_bank_and_pi(figure)
    draw_battery_and_motors(figure, [('Four AA batteries', 'name'), ('alkaline', 'note')],
                            '6.0 V nominal', [
        ('Motor driver', 'name'), ('Grove I2C Motor Driver', 'note'),
        ('makes 5 V for its logic', 'note')], driver_y=96)
    figure.legend(264, (LEGEND_BANK, LEGEND_BATTERY))
    return figure


def write_include(path: str, figure: Figure, caption: str, style: str = '') -> None:
    """Write a figure with its caption to an include, after the style element if one is given."""
    with open(path, 'w') as include:
        include.write(f'{style}<figure class="pf-figure">\n{figure.svg()}\n'
                      f'<figcaption>{caption}</figcaption>\n</figure>\n')


def main() -> None:
    """Write both includes."""
    write_include('includes/power-supply-flow-diffbot.html', diffbot(),
                  "DiffBot's two supplies. Follow each color from its source at the top.",
                  style_element(RULES) + '\n')
    write_include('includes/power-supply-flow-remo.html', remo(),
                  "Remo's two supplies. Follow each color from its source at the top. The motor "
                  'driver is the only part on both supplies.')


if __name__ == '__main__':
    main()
