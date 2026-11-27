"""Generate the figures on docs/theory/modeling-control/control-systems-introduction.md.

All are inline SVG that follows the site's light and dark colors:

- includes/control-systems-system.html: a system as actuators, process and sensors, with its
  inputs, outputs and observations. It has the styles for all figures;
- includes/control-systems-open-loop.html: an open loop, the chain from the reference through the
  controller, the actuators and the process to the outputs, with no path back. It holds a wide
  layout, with the chain to the right, and a tall one for phones, with the chain down;
- includes/control-systems-closed-loop.html: the same system with a controller that closes the
  loop from the observations back to the commands. It holds two layouts of the figure, a wide one
  for wider screens, with the forward path to the right and the feedback path back below it, and
  a tall one for phones. CSS shows one of them;
- includes/control-systems-agent.html: an agent of perception, planner and controller between the
  sensors and the actuators. Its title sits in the group's top left corner, clear of the arrow
  that enters the group from the sensors.

Each figure element has an id (FIGURE_IDS), so that the page can link to the figure itself.

The black words are the page's general terms. The blue words in the first two figures are one
example, the speed control of a wheel of the robots, which docs/packages/diffbot_base/pid.md
explains with the firmware's code.

Run from the repository root after changing a figure:

    python3 scripts/figures/control_systems_figures.py
"""

from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

WIDTH = 360
# The closed loop's forward path runs down at FORWARD_X through the boxes of the left column, its
# feedback path back up at FEEDBACK_X through the sensors' box of the right column
FORWARD_X = 105
FEEDBACK_X = 275
BOX_X = 20
BOX_WIDTH = 170
RIGHT_BOX_X = 215
RIGHT_BOX_WIDTH = 120
BOX_HEIGHT = 44
# The system's chain runs down the middle of its figure
SYSTEM_X = WIDTH / 2
# The closed loop's wide layout, for screens at least WIDE_FROM wide: the forward path runs right
# along WIDE_ROW to the actuators, the process and the sensors stand in a column at WIDE_RIGHT_X,
# and the feedback path runs back left along WIDE_BOTTOM. WIDE_WIDTH fits the page's article, which
# is 688 px wide on 1280 and 1440 px screens, so the text keeps its size
WIDE_FROM = '60em'
WIDE_WIDTH = 680
WIDE_HEIGHT = 344
WIDE_ROW = 100
WIDE_COMPARISON_X = 170
WIDE_CONTROLLER_X = 330
WIDE_CONTROLLER_WIDTH = 140
WIDE_BOTTOM = 290
WIDE_RIGHT_X = 520
WIDE_RIGHT_WIDTH = 156

# The id of each figure element, for links to the figure
FIGURE_IDS = {
    'includes/control-systems-system.html': 'system-figure',
    'includes/control-systems-open-loop.html': 'open-loop-figure',
    'includes/control-systems-closed-loop.html': 'closed-loop-figure',
    'includes/control-systems-agent.html': 'agent-figure',
}
# The agent figure's boxes, all as wide as AGENT_BOX_WIDTH and as high as AGENT_BOX_HEIGHT. The
# sensors, perception, controller and actuators are centered at SYSTEM_X, the planner at
# PLANNER_X
AGENT_BOX_WIDTH = 130
AGENT_BOX_HEIGHT = 36
PLANNER_X = 85

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]

ACTUATORS: Sequence[Line] = (('Actuators', 'name'), ('motor driver and motor', 'example'))
PROCESS: Sequence[Line] = (('Process and environment', 'name'),
                           ('wheel, robot and ground', 'example'))
SENSORS: Sequence[Line] = (('Sensors', 'name'), ('wheel encoder', 'example'))
LEGEND = "Blue: an example, the speed of one of the robot's wheels"
CLOSED_LOOP_TITLE = ('A closed control loop: reference, comparison, controller, actuators, '
                     'process, sensors, and the observations fed back to the comparison')


class Signal(NamedTuple):
    """A signal's label: the page's word for the signal, and the wheel example's word below."""

    name: str
    example: str


class Stage(NamedTuple):
    """One step of the open loop's chain: the signal into a box, and the box."""

    signal: Signal
    box_lines: Sequence[Line]
    wide_width: float  # the box's width in the wide layout


# The open loop is the closed loop's forward path: the same signals and boxes, without the
# comparison and the sensors. Its controller has no error to work with, so it follows a plan.
# In the tall layout, an arrow of OPEN_LOOP_GAP leads down into each box. In the wide layout,
# the chain runs right along OPEN_WIDE_ROW, and an arrow of OPEN_WIDE_GAP leads into each box
OPEN_LOOP_TITLE = ('An open control loop: reference, controller, actuators and process in a '
                   'chain to the outputs, with no path back to the controller')
OPEN_LOOP_STAGES = (
    Stage(Signal('reference', 'target wheel speed'),
          (('Controller', 'name'), ('plan set in advance', 'example')), WIDE_CONTROLLER_WIDTH),
    Stage(Signal('commands', 'motor command'), ACTUATORS, WIDE_RIGHT_WIDTH),
    Stage(Signal('inputs', 'torque on the wheel'), PROCESS, WIDE_RIGHT_WIDTH),
)
OPEN_LOOP_OUTPUTS = Signal('outputs', 'wheel speed')
OPEN_LOOP_TOP = 6
OPEN_LOOP_GAP = 54
OPEN_LOOP_LAST_ARROW = 46
OPEN_WIDE_HEIGHT = 124
OPEN_WIDE_ROW = 66
OPEN_WIDE_START = 4
OPEN_WIDE_GAP = 48

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.cs-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.cs-fig', 'width:100%;height:auto;--cs-example:#1e63c4;'
                'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('.cs-fig-wide', 'display:none'),
    (f'@media screen and (min-width:{WIDE_FROM})',
     '.md-typeset figure.cs-figure.cs-has-wide{max-width:none}'
     '.cs-fig-wide{display:block}.cs-fig-tall{display:none}'),
    ('[data-md-color-scheme=slate] .cs-fig', '--cs-example:#82b1ff'),
    ('.cs-fig text', f'fill:{FG};font-size:12px'),
    ('.cs-fig .block', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.cs-fig .group', 'fill:var(--md-default-fg-color--lightest);'
                       'stroke:var(--md-default-fg-color--lighter);stroke-width:1'),
    ('.cs-fig .group-name', 'font-size:13px;font-weight:700'),
    ('.cs-fig .arrow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.cs-fig .arrow-head', f'fill:{FG_LIGHT}'),
    ('.cs-fig .number-disc', f'fill:{FG}'),
    ('.cs-fig .number', 'fill:var(--md-default-bg-color);font-size:11px;font-weight:700'),
    ('.cs-fig .name', 'font-size:13px'),
    ('.cs-fig .sign', 'font-size:15px;font-weight:700'),
    ('.cs-fig .example', 'fill:var(--cs-example)'),
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
    """Collect the SVG elements of one figure, or of one layout of a figure.

    layout is the CSS class of a layout, cs-fig-wide or cs-fig-tall, and empty for a figure that
    has one layout only.
    """

    def __init__(self, name: str, height: int, title: str, width: int = WIDTH,
                 layout: str = '') -> None:
        self.name = name
        self.height = height
        self.title = title
        self.width = width
        self.layout = layout
        self.elements: List[str] = []

    def box(self, y: float, lines: Sequence[Line], x: float = BOX_X,
            width: float = BOX_WIDTH, height: float = BOX_HEIGHT,
            css_class: str = 'block') -> None:
        """Add a block with its text lines centered in it, 15 px apart."""
        self.elements.append(f'<rect class="{css_class}" x="{x}" y="{y}" width="{width}" '
                             f'height="{height}" rx="5"/>')
        for index, (text, text_class) in enumerate(lines):
            baseline = y + height / 2 + (index - (len(lines) - 1) / 2) * 15 + 4
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
            self.circle(x + 8, y - 4, 8, 'number-disc')
            self.text(x + 8, y - 0.2, str(number), 'number', 'middle')
            x += 20
        self.text(x, y, name)
        self.text(x, y + 14, example, 'example')

    def svg(self) -> str:
        """Return the figure as an SVG element."""
        head = (f'<marker id="{self.name}-head" viewBox="0 0 10 10" refX="9" refY="5" '
                'markerWidth="7" markerHeight="7" orient="auto">'
                '<path class="arrow-head" d="M0,0 L10,5 L0,10 z"/></marker>')
        css_class = f'cs-fig {self.layout}' if self.layout else 'cs-fig'
        return (f'<svg class="{css_class}" viewBox="0 0 {self.width} {self.height}" role="img" '
                f'aria-labelledby="{self.name}-title"><title id="{self.name}-title">'
                f'{escape(self.title)}</title><defs>{head}</defs>\n'
                + '\n'.join(self.elements) + '\n</svg>')


def system_figure() -> Figure:
    """Return the figure of a system: actuators, process and sensors in a chain."""
    figure = Figure('cs-system', 318,
                    'A system: actuators give inputs to the process, and sensors turn its '
                    'outputs into observations')
    box_x = SYSTEM_X - BOX_WIDTH / 2
    figure.box(6, ACTUATORS, x=box_x)
    figure.arrow(f'M{SYSTEM_X},50 V103')
    figure.signal(SYSTEM_X + 8, 72, 'inputs', 'torque on the wheel')
    figure.box(104, PROCESS, x=box_x)
    figure.arrow(f'M{SYSTEM_X},148 V201')
    figure.signal(SYSTEM_X + 8, 170, 'outputs', 'wheel speed')
    figure.box(202, SENSORS, x=box_x)
    figure.arrow(f'M{SYSTEM_X},246 V292')
    figure.signal(SYSTEM_X + 8, 266, 'observations', 'measured wheel speed')
    figure.text(WIDTH / 2, 312, LEGEND, 'legend', 'middle')
    return figure


def open_loop_figure() -> Figure:
    """Return the tall layout of the open loop, for phones: the chain runs down the middle."""
    legend_space = 26
    height = (OPEN_LOOP_TOP + len(OPEN_LOOP_STAGES) * (OPEN_LOOP_GAP + BOX_HEIGHT)
              + OPEN_LOOP_LAST_ARROW + legend_space)
    figure = Figure('cs-open', height, OPEN_LOOP_TITLE, layout='cs-fig-tall')
    label_x = SYSTEM_X + 8
    arrow_start = OPEN_LOOP_TOP
    for stage in OPEN_LOOP_STAGES:
        box_y = arrow_start + OPEN_LOOP_GAP
        figure.arrow(f'M{SYSTEM_X},{arrow_start} V{box_y - 1}')
        figure.signal(label_x, arrow_start + 22, stage.signal.name, stage.signal.example)
        figure.box(box_y, stage.box_lines, x=SYSTEM_X - BOX_WIDTH / 2)
        arrow_start = box_y + BOX_HEIGHT
    figure.arrow(f'M{SYSTEM_X},{arrow_start} V{arrow_start + OPEN_LOOP_LAST_ARROW}')
    figure.signal(label_x, arrow_start + 20, OPEN_LOOP_OUTPUTS.name, OPEN_LOOP_OUTPUTS.example)
    figure.text(WIDTH / 2, height - 6, LEGEND, 'legend', 'middle')
    return figure


def wide_open_loop_figure() -> Figure:
    """Return the wide layout of the open loop: the chain runs right, and nothing returns.

    Each signal's label starts above the start of its arrow, as in the closed loop's wide layout.
    """
    figure = Figure('cs-open-wide', OPEN_WIDE_HEIGHT, OPEN_LOOP_TITLE, width=WIDE_WIDTH,
                    layout='cs-fig-wide')
    label_y = OPEN_WIDE_ROW - 44
    arrow_start: float = OPEN_WIDE_START
    for stage in OPEN_LOOP_STAGES:
        box_x = arrow_start + OPEN_WIDE_GAP
        figure.arrow(f'M{arrow_start},{OPEN_WIDE_ROW} H{box_x}')
        figure.signal(arrow_start + 6, label_y, stage.signal.name, stage.signal.example)
        figure.box(OPEN_WIDE_ROW - BOX_HEIGHT / 2, stage.box_lines, x=box_x,
                   width=stage.wide_width)
        arrow_start = box_x + stage.wide_width
    figure.arrow(f'M{arrow_start},{OPEN_WIDE_ROW} H{WIDE_WIDTH - OPEN_WIDE_START}')
    figure.signal(arrow_start + 6, label_y, OPEN_LOOP_OUTPUTS.name, OPEN_LOOP_OUTPUTS.example)
    figure.text(WIDE_WIDTH / 2, OPEN_WIDE_HEIGHT - 8, LEGEND, 'legend', 'middle')
    return figure


def closed_loop_figure() -> Figure:
    """Return the tall layout of the closed loop, for phones: the loop runs down and back up."""
    figure = Figure('cs-loop', 428, CLOSED_LOOP_TITLE, layout='cs-fig-tall')
    comparison_y = 84
    figure.arrow(f'M{FORWARD_X},6 V{comparison_y - 14}')
    figure.signal(FORWARD_X + 6, 24, 'reference', 'target wheel speed', 1)
    figure.circle(FORWARD_X, comparison_y, 13, 'block')
    figure.text(FORWARD_X - 13, comparison_y - 12, '+', 'sign', 'end')
    figure.text(FORWARD_X + 17, comparison_y + 16, '\u2212', 'sign')
    figure.arrow(f'M{FORWARD_X},{comparison_y + 13} V143')
    figure.signal(FORWARD_X + 6, 118, 'error', 'target minus measured', 2)
    figure.box(144, (('Controller', 'name'), ('PID in the firmware', 'example')))
    figure.arrow(f'M{FORWARD_X},188 V239')
    figure.signal(FORWARD_X + 6, 210, 'commands', 'motor command', 3)
    figure.box(240, ACTUATORS)
    figure.arrow(f'M{FORWARD_X},284 V335')
    figure.signal(FORWARD_X + 6, 306, 'inputs', 'torque on the wheel', 4)
    figure.box(336, PROCESS)
    figure.arrow(f'M{BOX_X + BOX_WIDTH},358 H{FEEDBACK_X} V285')
    figure.signal(BOX_X + BOX_WIDTH + 8, 380, 'outputs', 'wheel speed', 5)
    figure.box(240, SENSORS, x=RIGHT_BOX_X, width=RIGHT_BOX_WIDTH)
    figure.arrow(f'M{FEEDBACK_X},240 V{comparison_y} H{FORWARD_X + 14}')
    figure.signal(FORWARD_X + 22, comparison_y - 20, 'observations', 'measured wheel speed', 6)
    figure.text(WIDTH / 2, 420, LEGEND, 'legend', 'middle')
    return figure


def wide_closed_loop_figure() -> Figure:
    """Return the wide layout of the closed loop: reference on the left, feedback path below.

    The forward path runs right from the reference through the comparison and the controller to
    the actuators. The process and the sensors follow down a column on the right, and the
    observations run back left along the bottom into the comparison's minus input.
    """
    figure = Figure('cs-loop-wide', WIDE_HEIGHT, CLOSED_LOOP_TITLE, width=WIDE_WIDTH,
                    layout='cs-fig-wide')
    right_center_x = WIDE_RIGHT_X + WIDE_RIGHT_WIDTH / 2
    process_y = WIDE_BOTTOM - 114
    figure.arrow(f'M4,{WIDE_ROW} H{WIDE_COMPARISON_X - 14}')
    figure.signal(8, 56, 'reference', 'target wheel speed', 1)
    figure.circle(WIDE_COMPARISON_X, WIDE_ROW, 13, 'block')
    figure.text(WIDE_COMPARISON_X - 13, WIDE_ROW - 12, '+', 'sign', 'end')
    figure.text(WIDE_COMPARISON_X + 8, WIDE_ROW + 30, '\u2212', 'sign')
    figure.arrow(f'M{WIDE_COMPARISON_X + 13},{WIDE_ROW} H{WIDE_CONTROLLER_X}')
    figure.signal(WIDE_COMPARISON_X + 20, 56, 'error', 'target minus measured', 2)
    controller = (('Controller', 'name'), ('PID in the firmware', 'example'))
    figure.box(WIDE_ROW - BOX_HEIGHT / 2, controller, x=WIDE_CONTROLLER_X,
               width=WIDE_CONTROLLER_WIDTH)
    figure.arrow(f'M{WIDE_CONTROLLER_X + WIDE_CONTROLLER_WIDTH},{WIDE_ROW} H{WIDE_RIGHT_X}')
    figure.signal(WIDE_CONTROLLER_X + WIDE_CONTROLLER_WIDTH + 6, 56, 'commands', 'motor command',
                  3)
    figure.box(WIDE_ROW - BOX_HEIGHT / 2, ACTUATORS, x=WIDE_RIGHT_X, width=WIDE_RIGHT_WIDTH)
    figure.arrow(f'M{right_center_x},{WIDE_ROW + BOX_HEIGHT / 2} V{process_y}')
    figure.signal(right_center_x - 136, 146, 'inputs', 'torque on the wheel', 4)
    figure.box(process_y, PROCESS, x=WIDE_RIGHT_X, width=WIDE_RIGHT_WIDTH)
    figure.arrow(f'M{right_center_x},{process_y + BOX_HEIGHT} V{WIDE_BOTTOM - BOX_HEIGHT / 2}')
    figure.signal(right_center_x - 104, WIDE_BOTTOM - 54, 'outputs', 'wheel speed', 5)
    figure.box(WIDE_BOTTOM - BOX_HEIGHT / 2, SENSORS, x=WIDE_RIGHT_X, width=WIDE_RIGHT_WIDTH)
    figure.arrow(f'M{WIDE_RIGHT_X},{WIDE_BOTTOM} H{WIDE_COMPARISON_X} V{WIDE_ROW + 13}')
    figure.signal(WIDE_COMPARISON_X + 20, WIDE_BOTTOM - 26, 'observations', 'measured wheel speed',
                  6)
    figure.text(WIDE_WIDTH / 2, WIDE_HEIGHT - 8, LEGEND, 'legend', 'middle')
    return figure


def agent_box(figure: Figure, y: float, name: str, center_x: float = SYSTEM_X) -> None:
    """Add one of the agent figure's boxes with its name, centered at center_x."""
    figure.box(y, ((name, 'name'),), x=center_x - AGENT_BOX_WIDTH / 2, width=AGENT_BOX_WIDTH,
               height=AGENT_BOX_HEIGHT)


def agent_figure() -> Figure:
    """Return the figure of an agent: perception, planner and controller, in a group.

    The sensors' observations enter the group at the top, the commands leave it at the bottom.
    """
    figure = Figure('cs-agent', 414,
                    'An agent: perception turns the observations into an estimate, the planner '
                    'makes a reference path from it, and the controller computes the commands')
    left_of_center, right_of_center = SYSTEM_X - 30, SYSTEM_X + 30
    agent_box(figure, 6, 'Sensors')
    figure.arrow(f'M{SYSTEM_X},42 V103')
    figure.text(SYSTEM_X + 8, 62, 'observations')
    figure.box(74, (), x=10, width=340, height=258, css_class='group')
    figure.text(22, 94, 'Agent', 'group-name')
    agent_box(figure, 104, 'Perception')
    figure.arrow(f'M{left_of_center},140 V156 H{PLANNER_X} V187')
    figure.text(PLANNER_X + 8, 176, 'estimate')
    agent_box(figure, 188, 'Planner', PLANNER_X)
    figure.arrow(f'M{right_of_center},140 V279')
    figure.text(right_of_center + 8, 214, 'estimate')
    figure.arrow(f'M{PLANNER_X},224 V298 H{SYSTEM_X - AGENT_BOX_WIDTH / 2 - 1}')
    figure.text(PLANNER_X + 8, 254, 'reference path')
    agent_box(figure, 280, 'Controller')
    figure.arrow(f'M{SYSTEM_X},316 V371')
    figure.text(SYSTEM_X + 8, 356, 'commands')
    agent_box(figure, 372, 'Actuators')
    return figure


def write_include(path: str, figures: Sequence[Figure], caption: str, style: str = '') -> None:
    """Write a figure with its caption to an include, after the style element if one is given.

    figures holds the figure's layouts: one, or the wide and the tall one, which CSS switches.
    """
    figure_class = 'cs-figure cs-has-wide' if len(figures) > 1 else 'cs-figure'
    svgs = '\n'.join(figure.svg() for figure in figures)
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{style}<figure class="{figure_class}" id="{FIGURE_IDS[path]}">\n'
                      f'{svgs}\n<figcaption>{caption}</figcaption>\n</figure>\n')


def main() -> None:
    """Write the includes."""
    write_include('includes/control-systems-system.html', [system_figure()],
                  'A system from its actuators to its sensors. Read the figure from top to '
                  'bottom. In the example, the measured wheel speed comes about like this: the '
                  "wheel turns, the encoder's two Hall sensors give pulses, the Teensy counts "
                  'their edges, and the firmware computes the angular velocity from the change of '
                  'the count and the time it took.',
                  style_element(RULES) + '\n')
    write_include('includes/control-systems-open-loop.html',
                  [wide_open_loop_figure(), open_loop_figure()],
                  'An open loop. Follow the arrows from the reference to the outputs. No arrow '
                  "leads back from the wheel's speed to the controller.")
    write_include('includes/control-systems-closed-loop.html',
                  [wide_closed_loop_figure(), closed_loop_figure()],
                  'A closed loop. Start at the reference (1) and follow the numbers around '
                  'the loop, back to the comparison in the circle.')
    write_include('includes/control-systems-agent.html', [agent_figure()],
                  'An agent between the sensors and the actuators. Perception gives its estimate '
                  'to the planner and to the controller, and the planner gives the reference path '
                  'to the controller.')


if __name__ == '__main__':
    main()
