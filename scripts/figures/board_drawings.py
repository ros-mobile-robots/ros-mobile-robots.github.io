"""Draw the robots' boards and parts for the generated figures, from above and schematic.

A figure that shows the hardware draws its parts with these functions, so that a part looks the
same in every figure: the Raspberry Pi 4 B, the Teensy 4.0, the Adafruit DC Motor FeatherWing, the
DG01D-E gear motor with its wheel, the motor's encoder seen from the motor's end, and Adafruit's
BNO055 breakout. The parts keep the colors of the real ones in both color schemes. board_rules()
returns their CSS.

The module also holds what a figure script needs to write SVG: escape(), style_element(), Box
for a rectangle with its edges and its middle, svg_subscript(), svg_element() with its arrowhead
markers, and Drawing, which collects the elements of one layout and draws its rectangles,
circles, lines, text, boxes with text and numbers in discs.
For the HTML around a drawing it has figure_element(), the figure with its panels and its caption,
and tab_set(), a set of content tabs.
"""

import math
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

# The x of the middle of the encoder board, from the left edge of the gear motor's gearbox. The
# encoder sits at the end of the motor's can, where a figure can attach its signal line
ENCODER_BOARD_OFFSET = 76
# The radius of the disc around a step's or a check's number, in px
NUMBER_DISC_RADIUS = 8


def escape(text: str) -> str:
    """Return text with the characters escaped that have a meaning in SVG."""
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def style_element(rules: Sequence[Tuple[str, str]]) -> str:
    """Return a style element with one CSS rule per (selector, declarations) pair."""
    lines = [f'{selector}{{{declarations}}}' for selector, declarations in rules]
    return '<style>\n' + '\n'.join(lines) + '\n</style>'


class Point(NamedTuple):
    """A point on a drawing, in SVG units from the upper left corner."""

    x: float
    y: float


class Box(NamedTuple):
    """A box on a drawing, in SVG units: its top left corner, its width and its height."""

    x: float
    y: float
    width: float
    height: float

    @property
    def right(self) -> float:
        """Return the x coordinate of the box's right edge."""
        return self.x + self.width

    @property
    def bottom(self) -> float:
        """Return the y coordinate of the box's lower edge."""
        return self.y + self.height

    @property
    def middle_x(self) -> float:
        """Return the x coordinate of the box's middle."""
        return self.x + self.width / 2

    @property
    def middle_y(self) -> float:
        """Return the y coordinate of the box's middle."""
        return self.y + self.height / 2


def svg_subscript(symbol: str, index: str, *, restore_baseline: bool = False) -> str:
    """Return SVG markup for a symbol with a subscript of the CSS class sub.

    symbol and index are text, which the markup holds escaped. The subscript lowers the baseline
    of the text line. With restore_baseline, the markup moves the baseline back up, for a line
    that goes on after the subscript.
    """
    lowered = f'{escape(symbol)}<tspan class="sub" dy="3">{escape(index)}</tspan>'
    # A shift applies to a character, so the second tspan holds one that has no width: U+200B
    return lowered + '<tspan dy="-3">\u200b</tspan>' if restore_baseline else lowered


def arrowhead_marker(marker_id: str, head_class: str = 'arrow-head',
                     orientation: str = 'auto') -> str:
    """Return an SVG marker that ends a line with an arrowhead of the CSS class head_class.

    orientation is the marker's orient attribute: auto, or auto-start-reverse for a marker that
    may also start a line.
    """
    return (f'<marker id="{marker_id}" viewBox="0 0 10 10" refX="9" refY="5" '
            f'markerWidth="7" markerHeight="7" orient="{orientation}">'
            f'<path class="{head_class}" d="M0,0 L10,5 L0,10 z"/></marker>')


def svg_element(name: str, css_classes: str, size: Tuple[float, float], title: str,
                elements: Sequence[str], *, description: Optional[str] = None,
                markers: Optional[Sequence[str]] = None) -> str:
    """Return an SVG element of size (width, height) with the elements, named title.

    name starts the element's IDs: name-title for its accessible name and, with a description,
    name-description for its accessible description, the longer text that a screen reader offers
    after the name. markers are the
    marker elements that the drawing's lines point to. Without them, the element has one
    arrowhead marker, name-head. An empty sequence gives an element without markers.
    """
    width, height = size
    described_by = ''
    described = ''
    if description is not None:
        described_by = f' aria-describedby="{name}-description"'
        described = f'<desc id="{name}-description">{escape(description)}</desc>'
    if markers is None:
        markers = (arrowhead_marker(f'{name}-head'),)
    definitions = f'<defs>{"".join(markers)}</defs>' if markers else ''
    return (f'<svg class="{css_classes}" viewBox="0 0 {width:g} {height:g}" role="img" '
            f'aria-labelledby="{name}-title"{described_by}>'
            f'<title id="{name}-title">{escape(title)}</title>'
            f'{described}{definitions}\n'
            + '\n'.join(elements) + '\n</svg>')


class Drawing:
    """Collect the SVG elements of one layout of a figure, in the order they are drawn."""

    def __init__(self) -> None:
        self.elements: List[str] = []

    def rect(self, x: float, y: float, width: float, height: float, css_class: str,
             corner_radius: float = 0) -> None:
        """Add a rectangle, with rounded corners if corner_radius is given."""
        self.elements.append(f'<rect class="{css_class}" x="{x:g}" y="{y:g}" width="{width:g}" '
                             f'height="{height:g}" rx="{corner_radius:g}"/>')

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle around (x, y)."""
        self.elements.append(f'<circle class="{css_class}" cx="{x:g}" cy="{y:g}" r="{radius:g}"/>')

    def line(self, path: str, css_class: str) -> None:
        """Add a line or a shape along an SVG path, without an arrowhead."""
        self.elements.append(f'<path class="{css_class}" d="{path}"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of escaped text at the requested anchor."""
        self.text_markup(x, y, escape(text), css_class, anchor)

    def text_markup(self, x: float, y: float, markup: str, css_class: str = '',
                    anchor: str = 'start') -> None:
        """Add one line of text from SVG markup, such as a symbol with a subscript.

        The markup goes into the drawing as it is, so escape() the text in it. anchor is start,
        middle or end.
        """
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(f'<text{class_attribute} x="{x:g}" y="{y:g}" '
                             f'text-anchor="{anchor}">{markup}</text>')

    def number_disc(self, x: float, y: float, number: int, disc_class: str,
                    number_class: str) -> None:
        """Add a number centered in a filled disc around (x, y), such as a step's or a check's."""
        self.circle(x, y, NUMBER_DISC_RADIUS, disc_class)
        self.text(x, y + 3.8, str(number), number_class, 'middle')

    def box(self, x: float, y: float, width: float, height: float, css_class: str,
            lines: Sequence[Tuple[str, str]] = ()) -> None:
        """Add a rectangle with its text lines centered in it, 14 px apart.

        Each line is its text and its CSS class.
        """
        self.rect(x, y, width, height, css_class, 5)
        for index, (text, text_class) in enumerate(lines):
            baseline = y + height / 2 + (index - (len(lines) - 1) / 2) * 14 + 4
            self.text(x + width / 2, baseline, text, text_class, 'middle')


class PanelWidths(NamedTuple):
    """The widths of a figure's panels on the page, in px."""

    basis: int  # the width from which a panel takes a row of its own
    widest: int  # a panel's largest width


def figure_element(css_prefix: str, panels_html: str, caption: str, widths: PanelWidths, *,
                   caption_is_markdown: bool = True) -> str:
    """Return a figure element with panels side by side and one caption below them.

    css_prefix starts the classes and the CSS variables: prefix-figure, prefix-panels,
    --prefix-basis and --prefix-max. panels_html is the markup of the panels, which wrap below
    each other on a narrow screen. caption is Markdown, or HTML without caption_is_markdown, for
    a figure inside raw HTML. The panels stand in a div without the markdown attribute, so that
    Markdown leaves the SVG's text as it is and the glossary doesn't wrap a term such as IMU in
    an abbr element.
    """
    figure_markdown, caption_markdown = (
        (' markdown="block"', ' markdown="span"') if caption_is_markdown else ('', ''))
    style = f'--{css_prefix}-basis:{widths.basis}px;--{css_prefix}-max:{widths.widest}px'
    return (f'<figure class="{css_prefix}-figure"{figure_markdown} style="{style}">\n'
            f'<div class="{css_prefix}-panels">\n{panels_html}\n</div>\n'
            f'<figcaption{caption_markdown}>{caption}</figcaption>\n</figure>')


class Tab(NamedTuple):
    """One tab of a tab set: its label as text and its content as HTML."""

    label: str
    content_html: str


def tab_set(name: str, css_class: str, tabs: Sequence[Tab]) -> str:
    """Return a set of content tabs as HTML, the first one selected.

    The HTML has the elements and classes of the tabs that the Markdown extension
    pymdownx.tabbed writes with alternate_style, so the site's theme shows, links and prints the
    set like a set from Markdown. The set is raw HTML, so Markdown leaves the text of a drawing
    in a tab as it is. name makes the IDs, which keeps them apart from the numbered IDs of the
    sets from Markdown. css_class is the set's own class. Each radio input, its label and its
    block come from one Tab, so a tab's content can't stand under another tab's label.
    """
    group = f'__tabbed_{name}'
    inputs = ''.join(f'<input{" checked" if index == 0 else ""} id="{group}_{index + 1}" '
                     f'name="{group}" type="radio">' for index in range(len(tabs)))
    labels = ''.join(f'<label for="{group}_{index + 1}">{escape(tab.label)}</label>'
                     for index, tab in enumerate(tabs))
    blocks = ''.join(f'<div class="tabbed-block">\n{tab.content_html}\n</div>\n' for tab in tabs)
    return (f'<div class="tabbed-set tabbed-alternate {css_class}" '
            f'data-tabs="{name}:{len(tabs)}">{inputs}<div class="tabbed-labels">{labels}</div>\n'
            f'<div class="tabbed-content">\n{blocks}</div>\n</div>')


def board_rules(scope: str) -> Tuple[Tuple[str, str], ...]:
    """Return the CSS rules of the parts' colors, for the figure whose SVG has the class scope.

    scope is a class selector, such as .pid-loop. The colors are those of the real parts in both
    color schemes. Only the tyre and the Hall sensors change in the dark scheme, so that they
    stand out from its background.
    """
    return (
        (f'{scope} .sbc-board', 'fill:#3f9a4e;stroke:#1f5a2a;stroke-width:0.8'),
        (f'{scope} .mcu-board', 'fill:#2f7d3c;stroke:#1a4d25;stroke-width:0.8'),
        (f'{scope} .driver-board', 'fill:#26408b;stroke:#152657;stroke-width:0.8'),
        (f'{scope} .pin-header', 'fill:#222'),
        (f'{scope} .metal', 'fill:#c8ccd2;stroke:#7d838c;stroke-width:0.6'),
        (f'{scope} .chip', 'fill:#2b2b2b'),
        (f'{scope} .pad', 'fill:#e3c15a'),
        (f'{scope} .terminal', 'fill:#3aa856;stroke:#1d6a33;stroke-width:0.6'),
        (f'{scope} .gearbox', 'fill:#8fc9ec;stroke:#3b7ea8;stroke-width:1'),
        (f'{scope} .motor-can', 'fill:#b8bec6;stroke:#6b737d;stroke-width:1'),
        (f'{scope} .encoder-board', 'fill:#3c8d4a;stroke:#245a2e;stroke-width:1'),
        (f'{scope} .ring-hole', 'fill:var(--md-code-bg-color)'),
        (f'{scope} .tyre', 'fill:#3a3a3a;stroke:#1d1d1d;stroke-width:1'),
        (f'[data-md-color-scheme=slate] {scope} .tyre', 'fill:#5a5f6a;stroke:#9aa0aa'),
        (f'{scope} .tyre-groove', 'fill:none;stroke:#6a6a6a;stroke-width:1'),
        (f'{scope} .shaft', 'fill:#d9d9d9;stroke:#777;stroke-width:1'),
        (f'{scope} .magnet-north', 'fill:#d9534f'),
        (f'{scope} .magnet-south', 'fill:#4a78c2'),
        (f'{scope} .hall-sensor', 'fill:#222;stroke:#888;stroke-width:0.6'),
        (f'[data-md-color-scheme=slate] {scope} .hall-sensor',
         'fill:#111;stroke:#e0e0e0;stroke-width:0.8'),
        (f'{scope} .connector', 'fill:#f4f4f4;stroke:#999;stroke-width:0.8'),
        (f'{scope} .imu-board', 'fill:#2450a6;stroke:#142e66;stroke-width:0.8'),
    )


def supply_part_rules(scope: str) -> Tuple[Tuple[str, str], ...]:
    """Return the CSS rules of the colors that only the supply's parts use, in scope.

    The power bank, the battery holder, the LiDAR, the camera and the Grove board use these
    together with board_rules(). The cases get a lighter fill and edge in the dark scheme, so
    that they stand out from its background.
    """
    return (
        (f'{scope} .case', 'fill:#3b3f45;stroke:#1f2226;stroke-width:0.8'),
        (f'[data-md-color-scheme=slate] {scope} .case', 'fill:#4a4f57;stroke:#a0a6ae'),
        (f'{scope} .led', 'fill:#4fc3f7'),
        (f'{scope} .holder-lid', 'fill:none;stroke:#6b7078;stroke-width:0.8'),
        (f'{scope} .lead-plus', 'fill:none;stroke:#d32f2f;stroke-width:1.6'),
        (f'{scope} .lead-minus', 'fill:none;stroke:#222;stroke-width:1.6'),
        (f'[data-md-color-scheme=slate] {scope} .lead-minus', 'stroke:#d0d4da'),
        (f'{scope} .lidar-window', 'fill:#8a1c1c'),
        (f'{scope} .lens', 'fill:#1b3f6b;stroke:#c8ccd2;stroke-width:0.6'),
        (f'{scope} .ribbon', 'fill:#e9d8a6;stroke:#b39a55;stroke-width:0.6'),
    )


def draw_raspberry_pi(drawing: Drawing, x: float, y: float) -> None:
    """Add a Raspberry Pi 4 B at (x, y): the board, its pin header, USB, Ethernet, processor."""
    drawing.rect(x, y, 48, 32, 'sbc-board', 2.5)
    drawing.rect(x + 4, y + 1.5, 34, 4, 'pin-header')
    for index, height in enumerate((6, 6, 7)):
        drawing.rect(x + 40, y + 8 + index * 8, 10, height, 'metal')
    drawing.rect(x + 14, y + 12, 11, 11, 'chip')


def draw_teensy(drawing: Drawing, x: float, y: float) -> None:
    """Add a Teensy 4.0 at (x, y): the board, its USB connector, processor and pads."""
    drawing.rect(x + 4, y, 34, 14, 'mcu-board', 1.5)
    drawing.rect(x, y + 3.5, 6, 7, 'metal', 1)
    drawing.rect(x + 18, y + 3, 8, 8, 'chip')
    for index in range(10):
        drawing.circle(x + 8 + index * 3, y + 1.6, 0.8, 'pad')
        drawing.circle(x + 8 + index * 3, y + 12.4, 0.8, 'pad')


def draw_motor_driver_board(drawing: Drawing, x: float, y: float) -> None:
    """Add the Adafruit DC Motor FeatherWing at (x, y): the board, its terminals and driver."""
    drawing.rect(x, y, 44, 22, 'driver-board', 2)
    for index in range(3):
        drawing.rect(x + 3 + index * 13, y + 2, 11, 7, 'terminal')
    drawing.rect(x + 16, y + 12, 10, 7, 'chip')


def draw_gear_motor_with_wheel(drawing: Drawing, x: float, y: float) -> None:
    """Add the DG01D-E gear motor with the wheel on its shaft, the gearbox's top left at (x, y).

    The gearbox, the motor's can and the encoder board at the can's end lie along the motor's
    axis. The shaft leaves the gearbox sideways, and the wheel is seen from above, tyre first.
    """
    drawing.rect(x, y, 44, 22, 'gearbox', 2)
    drawing.rect(x + 44, y + 2, 8, 18, 'gearbox')
    drawing.rect(x + 52, y + 1, 20, 20, 'motor-can', 3)
    drawing.rect(x + 72, y, 8, 22, 'encoder-board', 1)
    shaft_x = x + 12
    drawing.rect(shaft_x - 2.7, y + 22, 5.4, 6, 'shaft')
    drawing.rect(shaft_x - 32.5, y + 28, 65, 26, 'tyre', 5)
    for index in range(1, 8):
        groove_x = shaft_x - 32.5 + index * 65 / 8
        drawing.line(f'M{groove_x:.1f},{y + 30} V{y + 52}', 'tyre-groove')


def draw_encoder_disc(drawing: Drawing, center_x: float, center_y: float) -> None:
    """Add the motor's encoder seen from the motor's end, enlarged, around (center_x, center_y).

    The magnet disc on the motor shaft has six alternating poles. The ring-shaped board around it
    carries the two Hall sensors, both on the upper arc, and the connector below.
    """
    drawing.circle(center_x, center_y, 19, 'encoder-board')
    drawing.circle(center_x, center_y, 13, 'ring-hole')
    drawing.rect(center_x - 9, center_y + 17, 18, 8, 'connector', 1)
    radius = 11
    for pole in range(6):
        start = math.radians(pole * 60 - 90)
        end = math.radians(pole * 60 - 30)
        drawing.line(f'M{center_x:g},{center_y:g} '
                     f'L{center_x + radius * math.cos(start):.2f},'
                     f'{center_y + radius * math.sin(start):.2f} '
                     f'A{radius},{radius} 0 0 1 {center_x + radius * math.cos(end):.2f},'
                     f'{center_y + radius * math.sin(end):.2f} Z',
                     'magnet-north' if pole % 2 == 0 else 'magnet-south')
    drawing.circle(center_x, center_y, 2, 'shaft')
    for angle in (-135, -45):
        sensor_x = center_x + 16 * math.cos(math.radians(angle))
        sensor_y = center_y + 16 * math.sin(math.radians(angle))
        rotation = f'rotate({angle + 90} {sensor_x:.1f} {sensor_y:.1f})'
        drawing.elements.append(
            f'<rect class="hall-sensor" x="{sensor_x - 3:.1f}" y="{sensor_y - 2.5:.1f}" '
            f'width="6" height="5" transform="{rotation}"/>')


def draw_bno055_breakout(drawing: Drawing, x: float, y: float) -> None:
    """Add Adafruit's BNO055 breakout at (x, y), upright: the board, its holes, pads and chips.

    The four mounting holes sit in the corners, six pads along the left edge (VIN to RST) and four
    along the right edge (PS0 to ADR). The BNO055 is the square chip in the middle.
    """
    drawing.rect(x, y, 22, 28, 'imu-board', 2)
    for corner_x, corner_y in ((4, 4), (18, 4), (4, 24), (18, 24)):
        drawing.circle(x + corner_x, y + corner_y, 2.4, 'pad')
        drawing.circle(x + corner_x, y + corner_y, 1.2, 'ring-hole')
    for index in range(6):
        drawing.circle(x + 2.4, y + 8.5 + index * 2.3, 0.9, 'pad')
    for index in range(4):
        drawing.circle(x + 19.6, y + 10.5 + index * 2.3, 0.9, 'pad')
    drawing.rect(x + 13, y + 3, 4, 8, 'chip', 0.5)
    drawing.rect(x + 8, y + 11.5, 7, 7, 'chip', 0.5)


def draw_power_bank(drawing: Drawing, x: float, y: float) -> None:
    """Add a USB power bank at (x, y): its case, a USB port at its end and four charge LEDs."""
    drawing.rect(x, y, 44, 24, 'case', 4)
    drawing.rect(x + 44, y + 8, 4, 8, 'metal', 1)
    for index in range(4):
        drawing.circle(x + 8 + index * 5, y + 18, 1.4, 'led')


def draw_battery_holder(drawing: Drawing, x: float, y: float) -> None:
    """Add the series motor pack at (x, y): a closed holder with its two leads.

    The drawing shows no cells, so that it fits any number of cells and any holder. The red lead
    is the plus pole, the black one the minus pole.
    """
    drawing.rect(x, y + 2, 48, 24, 'case', 3)
    drawing.rect(x + 5, y + 7, 38, 14, 'holder-lid', 2)
    drawing.line(f'M{x + 48},{y + 9} H{x + 52} Q{x + 56},{y + 9} {x + 56},{y + 5} V{y + 2}',
                 'lead-plus')
    drawing.line(f'M{x + 48},{y + 19} H{x + 52} Q{x + 56},{y + 19} {x + 56},{y + 23} V{y + 26}',
                 'lead-minus')


def draw_lidar(drawing: Drawing, x: float, y: float) -> None:
    """Add an RPLIDAR A2 at (x, y), from the side: its base and the round head that turns on it."""
    drawing.rect(x, y + 14, 38, 12, 'case', 2)
    drawing.rect(x + 4, y, 30, 15, 'case', 6)
    drawing.rect(x + 8, y + 5, 22, 4, 'lidar-window', 1)


def draw_camera(drawing: Drawing, x: float, y: float) -> None:
    """Add a Raspberry Pi Camera Module 2 at (x, y): its board, the lens and the ribbon cable."""
    drawing.rect(x, y, 25, 24, 'mcu-board', 2)
    drawing.rect(x + 7.5, y + 5, 10, 10, 'chip', 1)
    drawing.circle(x + 12.5, y + 10, 3, 'lens')
    drawing.rect(x + 5, y + 24, 15, 6, 'ribbon')


def draw_grove_motor_driver(drawing: Drawing, x: float, y: float) -> None:
    """Add the Grove I2C Motor Driver V1.3 at (x, y), from above, as the board's photo shows it.

    Two motor terminals on the left, the power terminal at the top right, the flat L298 with its
    pins, the square microcontroller, the jumper J4 and the white Grove socket below it.
    """
    drawing.rect(x, y, 46, 26, 'driver-board', 2)
    for terminal_x, terminal_y in ((x + 1.5, y + 2), (x + 1.5, y + 16), (x + 37.5, y + 2)):
        drawing.rect(terminal_x, terminal_y, 7, 8, 'terminal')
    for index in range(6):
        drawing.rect(x + 12 + index * 2.2, y + 7, 1.2, 1.6, 'metal')
        drawing.rect(x + 12 + index * 2.2, y + 18.4, 1.2, 1.6, 'metal')
    drawing.rect(x + 11, y + 8.5, 14, 10, 'chip', 0.8)
    drawing.rect(x + 28, y + 11, 6.5, 6.5, 'chip')
    drawing.rect(x + 38.5, y + 11, 5, 2, 'chip')
    drawing.rect(x + 38.5, y + 14, 6, 10, 'connector', 0.6)
