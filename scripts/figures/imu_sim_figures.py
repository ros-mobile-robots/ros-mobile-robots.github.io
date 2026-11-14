"""Generate the thread figures on docs/packages/imu/simulation/plugin.md.

It writes two includes, as inline SVG that follows the site's light and dark colors. Both follow
one switch_controller request of the controller spawner through the Gazebo server, with the same
two lanes, Gazebo's physics thread and one of the ROS spinner threads, and time running downward:

- includes/imu-sim-threads-original.html: gazebo_plugins' original IMU plugin runs the service's
  callback in the physics thread with ros::spinOnce(), and the physics thread waits for itself.
- includes/imu-sim-threads-copy.html: with the robots' copy of the plugin, a spinner thread runs
  the callback, and the physics thread goes on.

The numbers in the discs are the steps of the numbered lists on the page, so change them
together. Each caption is Markdown.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_sim_figures.py
"""

from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple


class Lane(NamedTuple):
    """A thread's lane on the drawing: the x positions of its left and right edges."""

    left: float
    right: float

    @property
    def width(self) -> float:
        """Return the lane's width."""
        return self.right - self.left

    @property
    def center(self) -> float:
        """Return the x position of the lane's middle."""
        return (self.left + self.right) / 2


class Box(NamedTuple):
    """A box on the drawing: its upper left corner and its size."""

    x: float
    y: float
    width: float
    height: float

    @property
    def right(self) -> float:
        """Return the x position of the box's right edge."""
        return self.x + self.width

    @property
    def bottom(self) -> float:
        """Return the y position of the box's lower edge."""
        return self.y + self.height

    @property
    def center_x(self) -> float:
        """Return the x position of the box's middle."""
        return self.x + self.width / 2

    @property
    def center_y(self) -> float:
        """Return the y position of the box's middle."""
        return self.y + self.height / 2


WIDTH = 360
PHYSICS_LANE = Lane(left=18, right=178)
SPINNER_LANE = Lane(left=186, right=352)
LANE_TOP = 62
# The boxes above the lanes
SPAWNER_BOX = Box(x=18, y=8, width=152, height=38)
QUEUE_BOX = Box(x=200, y=8, width=152, height=38)
# Where the request leaves the queue downward, to the right of the spinner lane's name
REQUEST_X = 338
LINE_HEIGHT = 14

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
BG = 'var(--md-default-bg-color)'
RULES = (
    ('.md-typeset figure.th-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.th-fig', 'width:100%;height:auto;--th-request:#1e63c4;--th-stuck:#c62828;'
                'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .th-fig', '--th-request:#82b1ff;--th-stuck:#ef6a6a'),
    ('.th-fig text', f'fill:{FG};font-size:12px'),
    ('.th-fig .lane-name', f'font-weight:700;fill:{FG_LIGHT}'),
    ('.th-fig .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                      'font-size:11.5px'),
    ('.th-fig .note', f'font-size:11px;fill:{FG_LIGHT}'),
    ('.th-fig .stuck-text', 'fill:var(--th-stuck)'),
    ('.th-fig .strong', 'font-weight:700'),
    ('.th-fig .lane', 'fill:var(--md-default-fg-color--lightest);stroke:none'),
    ('.th-fig .box', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:1'),
    ('.th-fig .queue', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1;'
                       'stroke-dasharray:4 3'),
    ('.th-fig .callback', f'fill:{BG};stroke:var(--th-request);stroke-width:1.8'),
    ('.th-fig .idle', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:3 3'),
    ('.th-fig .stuck', 'fill:none;stroke:var(--th-stuck);stroke-width:1.6;stroke-dasharray:5 3'),
    ('.th-fig .open', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:2 3'),
    ('.th-fig .flow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.4'),
    ('.th-fig .flow-head', f'fill:{FG_LIGHT}'),
    ('.th-fig .request', 'fill:none;stroke:var(--th-request);stroke-width:1.8'),
    ('.th-fig .request-head', 'fill:var(--th-request)'),
    ('.th-fig .step-disc', f'fill:{FG}'),
    ('.th-fig .step', f'fill:{BG};font-size:11px;font-weight:700'),
)


def escape(text: str) -> str:
    """Return text with the characters escaped that have a meaning in SVG."""
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def style_element(rules: Sequence[Tuple[str, str]]) -> str:
    """Return a style element with one CSS rule per (selector, declarations) pair."""
    lines = [f'{selector}{{{declarations}}}' for selector, declarations in rules]
    return '<style>\n' + '\n'.join(lines) + '\n</style>'


class Figure:
    """Collect the SVG elements of one drawing."""

    def __init__(self, name: str, height: float, title: str, description: str) -> None:
        self.name, self.height = name, height
        self.title, self.description = title, description
        self.elements: List[str] = []

    def rectangle(self, x: float, y: float, width: float, height: float, css_class: str,
                  radius: float = 4) -> None:
        """Add a rectangle with its upper left corner at (x, y)."""
        self.elements.append(f'<rect class="{css_class}" x="{x}" y="{y}" width="{width}" '
                             f'height="{height}" rx="{radius}"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'middle') -> None:
        """Add one line of text. anchor is start, middle or end."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(
            f'<text{class_attribute} x="{x}" y="{y}" text-anchor="{anchor}">{escape(text)}</text>')

    def lines(self, x: float, y: float, lines: Sequence[Line]) -> None:
        """Add centered text lines, the first with its baseline at y."""
        for index, (text, css_class) in enumerate(lines):
            self.text(x, y + index * LINE_HEIGHT, text, css_class)

    def box(self, x: float, y: float, width: float, css_class: str,
            lines: Sequence[Line]) -> float:
        """Add a box around centered text lines and return the y position of its lower edge."""
        height = len(lines) * LINE_HEIGHT + 10
        self.rectangle(x, y, width, height, css_class)
        self.lines(x + width / 2, y + 17, lines)
        return y + height

    def path(self, path: str, css_class: str) -> None:
        """Add a line along an SVG path, without an arrowhead."""
        self.elements.append(f'<path class="{css_class}" d="{path}"/>')

    def arrow(self, path: str, css_class: str) -> None:
        """Add a line along an SVG path, with an arrowhead at its end."""
        self.elements.append(f'<path class="{css_class}" d="{path}" '
                             f'marker-end="url(#{self.name}-{css_class})"/>')

    def step(self, x: float, y: float, number: int) -> None:
        """Add a step's number in a filled disc around (x, y)."""
        self.elements.append(f'<circle class="step-disc" cx="{x}" cy="{y}" r="8"/>')
        self.text(x, y + 4, str(number), 'step')

    def svg(self) -> str:
        """Return the drawing as an SVG element."""
        heads = ''.join(
            f'<marker id="{self.name}-{css_class}" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path class="{css_class}-head" d="M0,0 L10,5 L0,10 z"/></marker>'
            for css_class in ('flow', 'request'))
        return (f'<svg class="th-fig" viewBox="0 0 {WIDTH} {self.height}" role="img" '
                f'aria-labelledby="{self.name}-title {self.name}-description">'
                f'<title id="{self.name}-title">{escape(self.title)}</title>'
                f'<desc id="{self.name}-description">{escape(self.description)}</desc>'
                f'<defs>{heads}</defs>\n' + '\n'.join(self.elements) + '\n</svg>')


def draw_request(figure: Figure) -> None:
    """Add the spawner, the callback queue and the request between them (step 1)."""
    spawner, queue = SPAWNER_BOX, QUEUE_BOX
    figure.rectangle(spawner.x, spawner.y, spawner.width, spawner.height, 'box')
    figure.lines(spawner.center_x, spawner.y + 16, [('controller spawner', ''),
                                                    ('another ROS node', 'note')])
    figure.rectangle(queue.x, queue.y, queue.width, queue.height, 'queue')
    figure.lines(queue.center_x, queue.y + 16, [('global callback queue', ''),
                                                ('a waiting list', 'note')])
    figure.arrow(f'M{spawner.right},{spawner.center_y} H{queue.x - 1}', 'request')
    figure.step((spawner.right + queue.x) / 2, spawner.center_y - 12, 1)


def draw_lanes(figure: Figure) -> None:
    """Add the two lanes with their names."""
    for lane, name in ((PHYSICS_LANE, 'physics thread'), (SPINNER_LANE, 'ROS spinner thread')):
        figure.rectangle(lane.left, LANE_TOP, lane.width, figure.height - LANE_TOP - 4, 'lane',
                         6)
        figure.text(lane.center, LANE_TOP + 17, name, 'lane-name')
    figure.elements.append(f'<text class="note" transform="translate(13,{LANE_TOP + 4}) '
                           'rotate(-90)" text-anchor="end">time</text>')
    figure.arrow(f'M9,{LANE_TOP + 34} V{figure.height - 10}', 'flow')


def threads_with_the_original_figure() -> Figure:
    """Return the drawing of the request with gazebo_plugins' original plugin: a deadlock."""
    figure = Figure('th-original', 352, 'The request with the original IMU plugin',
                    'Step 1: the controller spawner, another ROS node, sends a switch_controller '
                    'request, which waits in the global callback queue. Step 2: the physics '
                    'thread calls the IMU plugin’s UpdateChild(), which publishes the IMU '
                    'message and calls ros::spinOnce(). Step 3: ros::spinOnce() takes the '
                    'request from the queue and runs the callback in the physics thread. Step 4: '
                    'the callback notes the change and waits for update(). Step 5: update() '
                    'never runs, because its thread is waiting. This is a deadlock: the clock '
                    'stops. The ROS spinner thread finds the queue empty.')
    draw_request(figure)
    draw_lanes(figure)
    left, right, center = PHYSICS_LANE.left, PHYSICS_LANE.right, PHYSICS_LANE.center
    # UpdateChild() holds ros::spinOnce(), which holds the callback. The two outer frames stay
    # open at the bottom: these functions never return
    figure.path(f'M{left + 4},262 V88 H{right - 4} V262', 'box')
    figure.path(f'M{left + 4},262 V278 M{right - 4},262 V278', 'open')
    figure.lines(center, 104, [('UpdateChild()', 'mono'), ('publishes the IMU message', 'note')])
    figure.step(left + 4, 88, 2)
    figure.path(f'M{left + 10},258 V126 H{right - 10} V258', 'box')
    figure.path(f'M{left + 10},258 V272 M{right - 10},258 V272', 'open')
    figure.lines(center, 142, [('ros::spinOnce()', 'mono'), ('runs the callback here', 'note')])
    figure.box(left + 16, 166, right - left - 32, 'callback',
               [('switch_controller', 'mono'), ('callback', 'note'), ('notes the change,', ''),
                ('waits for update()', '')])
    figure.step(left + 16, 166, 4)
    # The request goes from the queue down the spinner lane and into ros::spinOnce()
    figure.arrow(f'M{REQUEST_X},{QUEUE_BOX.bottom} V150 H{right - 9}', 'request')
    figure.step(REQUEST_X, 112, 3)
    figure.box(left + 4, 286, right - left - 8, 'stuck',
               [('update()', 'mono stuck-text'), ('never runs: its thread waits', 'note')])
    figure.step(left + 4, 286, 5)
    figure.text(center, 340, 'deadlock: the clock stops', 'stuck-text strong')
    figure.box(SPINNER_LANE.left + 20, 176, SPINNER_LANE.width - 40, 'idle',
               [('finds the queue', 'note'), ('empty', 'note')])
    return figure


def threads_with_the_copy_figure() -> Figure:
    """Return the drawing of the request with the robots' copy of the plugin: no deadlock."""
    figure = Figure('th-copy', 322, 'The request with the robots’ copy of the IMU plugin',
                    'Step 1: the controller spawner sends a switch_controller request, which '
                    'waits in the global callback queue. Step 2: the physics thread calls the IMU '
                    'plugin’s UpdateChild(), which publishes the IMU message and returns. '
                    'Step 3: a ROS spinner thread takes the request and runs the callback, which '
                    'notes the change and waits for update(). Step 4: the physics thread goes '
                    'on and calls update(), which takes the change. Step 5: the callback '
                    'returns, and the spawner gets its answer.')
    draw_request(figure)
    draw_lanes(figure)
    left, right, center = PHYSICS_LANE.left, PHYSICS_LANE.right, PHYSICS_LANE.center
    width = right - left - 8
    figure.box(left + 4, 88, width, 'box',
               [('UpdateChild()', 'mono'), ('publishes the IMU message', 'note'),
                ('and returns', 'note')])
    figure.step(left + 4, 88, 2)
    figure.box(left + 4, 150, width, 'box', [('update()', 'mono'),
                                             ('runs the controllers', 'note')])
    figure.box(left + 4, 222, width, 'callback', [('update()', 'mono'),
                                                  ('takes the change', 'note')])
    figure.step(left + 4, 222, 4)
    figure.lines(center, 282, [('next steps: the clock', 'note'), ('runs on', 'note')])
    spinner_left, spinner_center = SPINNER_LANE.left, SPINNER_LANE.center
    figure.arrow(f'M{REQUEST_X},{QUEUE_BOX.bottom} V87', 'request')
    figure.rectangle(spinner_left + 4, 88, SPINNER_LANE.width - 8, 172, 'callback')
    figure.lines(spinner_center, 105, [('switch_controller', 'mono'), ('callback', 'note'),
                                       ('notes the change,', ''), ('waits for update()', '')])
    figure.step(spinner_left + 4, 88, 3)
    figure.arrow(f'M{right - 4},241 H{spinner_left + 3}', 'request')
    figure.text(spinner_center, 245, 'the change is taken', 'note')
    figure.arrow(f'M{spinner_center},260 V280', 'flow')
    figure.step(spinner_center - 62, 296, 5)
    figure.lines(spinner_center + 8, 294, [('returns: the spawner', 'note'),
                                           ('gets its answer', 'note')])
    return figure


def figure_html(drawing: Figure, caption: str) -> str:
    """Return a figure element with a drawing and its caption. caption is Markdown.

    The drawing stands in a div without the markdown attribute, so that Markdown leaves the
    SVG's text as it is. Otherwise the glossary would wrap a term such as IMU in an abbr element,
    which a browser doesn't draw inside SVG text.
    """
    return (f'{style_element(RULES)}\n<figure class="th-figure" markdown="block">\n'
            f'<div class="th-panel">\n{drawing.svg()}\n</div>\n'
            f'<figcaption markdown="span">{caption}</figcaption>\n</figure>\n')


def main() -> None:
    """Write the two includes."""
    original = ('With gazebo_plugins’ original IMU plugin. Time runs downward, and the '
                'numbers are the steps in the text. Blue is the `switch_controller` request and '
                'its callback. The callback runs in the physics thread and waits for an '
                '`update()` that only this thread can run.')
    copy = ('With the robots’ copy of the plugin. A ROS spinner thread runs the callback and '
            'waits. The physics thread goes on and calls `update()`.')
    with open('includes/imu-sim-threads-original.html', 'w', encoding='utf-8') as include:
        include.write(figure_html(threads_with_the_original_figure(), original))
    with open('includes/imu-sim-threads-copy.html', 'w', encoding='utf-8') as include:
        include.write(figure_html(threads_with_the_copy_figure(), copy))


if __name__ == '__main__':
    main()
