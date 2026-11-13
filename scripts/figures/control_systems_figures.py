"""Generate the figures on docs/theory/modeling-control/control-systems-introduction.md.

Both are inline SVG that follows the site's light and dark colors:

- includes/control-systems-system.html: a system as actuators, process and sensors, with its
  inputs, outputs and observations. It has the styles for both figures;
- includes/control-systems-closed-loop.html: the same system with a controller that closes the
  loop from the observations back to the commands.

The black words are the page's general terms. The blue words are one example, the speed control of
a wheel of the robots, which docs/packages/diffbot_base/pid.md explains with the firmware's code.

Run from the repository root after changing a figure:

    python3 scripts/figures/control_systems_figures.py
"""

from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

WIDTH = 360
# The forward path runs down at FORWARD_X, the feedback path back up at FEEDBACK_X
FORWARD_X = 105
FEEDBACK_X = 265
BOX_X = 30
BOX_WIDTH = 150
BOX_HEIGHT = 40

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]

ACTUATORS: Sequence[Line] = (('Actuators', 'name'), ('motor driver and motor', 'example'))
PROCESS: Sequence[Line] = (('Process and environment', 'name'),
                           ('wheel, robot and ground', 'example'))
SENSORS: Sequence[Line] = (('Sensors', 'name'), ('wheel encoder', 'example'))
LEGEND = "Blue: an example, the speed of one of the robot's wheels"

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.cs-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.cs-fig', 'width:100%;height:auto;--cs-example:#1e63c4;'
                'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .cs-fig', '--cs-example:#82b1ff'),
    ('.cs-fig text', f'fill:{FG};font-size:10px'),
    ('.cs-fig .block', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.cs-fig .arrow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.cs-fig .arrow-head', f'fill:{FG_LIGHT}'),
    ('.cs-fig .number-disc', f'fill:{FG}'),
    ('.cs-fig .number', 'fill:var(--md-default-bg-color);font-size:9.5px;font-weight:700'),
    ('.cs-fig .name', 'font-size:11px'),
    ('.cs-fig .sign', 'font-size:13px;font-weight:700'),
    ('.cs-fig .example', 'fill:var(--cs-example);font-size:9.5px'),
    ('.cs-fig .legend', 'fill:var(--cs-example)'),
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

    def box(self, y: float, lines: Sequence[Line], x: float = BOX_X,
            width: float = BOX_WIDTH) -> None:
        """Add a block with its text lines centered in it, 13 px apart."""
        self.elements.append(f'<rect class="block" x="{x}" y="{y}" width="{width}" '
                             f'height="{BOX_HEIGHT}" rx="5"/>')
        for index, (text, text_class) in enumerate(lines):
            baseline = y + BOX_HEIGHT / 2 + (index - (len(lines) - 1) / 2) * 13 + 4
            self.text(x + width / 2, baseline, text, text_class, 'middle')

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle around (x, y)."""
        self.elements.append(f'<circle class="{css_class}" cx="{x}" cy="{y}" r="{radius}"/>')

    def arrow(self, path: str) -> None:
        """Add a line along an SVG path, with an arrowhead at its end."""
        self.elements.append(
            f'<path class="arrow" d="{path}" marker-end="url(#{self.name}-head)"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of text. anchor is start, middle or end."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(
            f'<text{class_attribute} x="{x}" y="{y}" text-anchor="{anchor}">{escape(text)}</text>')

    def signal(self, x: float, y: float, name: str, example: str,
               number: Optional[int] = None) -> None:
        """Add a signal's label: its name, its example below, and its number if it has one."""
        if number is not None:
            self.circle(x + 7, y - 3.5, 7, 'number-disc')
            self.text(x + 7, y - 0.3, str(number), 'number', 'middle')
            x += 18
        self.text(x, y, name)
        self.text(x, y + 12, example, 'example')

    def svg(self) -> str:
        """Return the figure as an SVG element."""
        head = (f'<marker id="{self.name}-head" viewBox="0 0 10 10" refX="9" refY="5" '
                'markerWidth="7" markerHeight="7" orient="auto">'
                '<path class="arrow-head" d="M0,0 L10,5 L0,10 z"/></marker>')
        return (f'<svg class="cs-fig" viewBox="0 0 {WIDTH} {self.height}" role="img" '
                f'aria-labelledby="{self.name}-title"><title id="{self.name}-title">'
                f'{escape(self.title)}</title><defs>{head}</defs>\n'
                + '\n'.join(self.elements) + '\n</svg>')


def system_figure() -> Figure:
    """Return the figure of a system: actuators, process and sensors in a chain."""
    figure = Figure('cs-system', 294,
                    'A system: actuators give inputs to the process, and sensors turn its '
                    'outputs into observations')
    figure.box(6, ACTUATORS)
    figure.arrow(f'M{FORWARD_X},46 V96')
    figure.signal(FORWARD_X + 8, 66, 'inputs', 'torque on the wheel')
    figure.box(97, PROCESS)
    figure.arrow(f'M{FORWARD_X},137 V187')
    figure.signal(FORWARD_X + 8, 157, 'outputs', 'wheel speed')
    figure.box(188, SENSORS)
    figure.arrow(f'M{FORWARD_X},228 V268')
    figure.signal(FORWARD_X + 8, 248, 'observations', 'measured wheel speed')
    figure.text(WIDTH / 2, 286, LEGEND, 'legend', 'middle')
    return figure


def closed_loop_figure() -> Figure:
    """Return the figure of the system with a controller that closes the loop."""
    figure = Figure('cs-loop', 384,
                    'A closed control loop: reference, comparison, controller, actuators, '
                    'process, sensors, and the observations fed back to the comparison')
    comparison_y = 74
    figure.arrow(f'M{FORWARD_X},6 V{comparison_y - 14}')
    figure.signal(FORWARD_X + 8, 22, 'reference', 'target wheel speed', 1)
    figure.circle(FORWARD_X, comparison_y, 13, 'block')
    figure.text(FORWARD_X - 13, comparison_y - 12, '+', 'sign', 'end')
    figure.text(FORWARD_X + 17, comparison_y + 15, '−', 'sign')
    figure.arrow(f'M{FORWARD_X},{comparison_y + 13} V127')
    figure.signal(FORWARD_X + 8, 104, 'error', 'target minus measured', 2)
    figure.box(128, (('Controller', 'name'), ('PID controller in the firmware', 'example')))
    figure.arrow(f'M{FORWARD_X},168 V218')
    figure.signal(FORWARD_X + 8, 188, 'commands', 'motor command', 3)
    figure.box(219, ACTUATORS)
    figure.arrow(f'M{FORWARD_X},259 V309')
    figure.signal(FORWARD_X + 8, 279, 'inputs', 'torque on the wheel', 4)
    figure.box(310, PROCESS)
    figure.arrow(f'M{BOX_X + BOX_WIDTH},330 H{FEEDBACK_X} V260')
    figure.signal(FEEDBACK_X + 8, 290, 'outputs', 'wheel speed', 5)
    figure.box(219, SENSORS, x=200, width=130)
    figure.arrow(f'M{FEEDBACK_X},219 V{comparison_y} H{FORWARD_X + 14}')
    figure.signal(FORWARD_X + 24, comparison_y - 18, 'observations', 'measured wheel speed', 6)
    figure.text(WIDTH / 2, 374, LEGEND, 'legend', 'middle')
    return figure


def write_include(path: str, figure: Figure, caption: str, style: str = '') -> None:
    """Write a figure with its caption to an include, after the style element if one is given."""
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{style}<figure class="cs-figure">\n{figure.svg()}\n'
                      f'<figcaption>{caption}</figcaption>\n</figure>\n')


def main() -> None:
    """Write both includes."""
    write_include('includes/control-systems-system.html', system_figure(),
                  'A system from its actuators to its sensors. Read the figure from top to '
                  'bottom.', style_element(RULES) + '\n')
    write_include('includes/control-systems-closed-loop.html', closed_loop_figure(),
                  'A closed loop. Start at the reference (1) and follow the numbers around '
                  'the loop, back to the comparison in the circle.')


if __name__ == '__main__':
    main()
