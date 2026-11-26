"""Generate the thread figures on docs/packages/imu/simulation/plugin.md.

It writes one include, includes/imu-sim-threads.html, with two drawings as inline SVG that
follows the site's light and dark colors. Both follow one switch_controller request of the
controller spawner through the Gazebo server, with the same two lanes, Gazebo's physics thread
and one of the ROS spinner threads, and time running downward:

- With gazebo_plugins' original IMU plugin, ros::spinOnce() runs the service's callback in the
  physics thread, and the physics thread waits for itself.
- With the robots' copy of the plugin, a spinner thread runs the callback, and the physics thread
  goes on.

The two drawings stand in two tabs at the same place, so that a reader can switch between them.
So both have the same size, and what they share lies at the same position in both: the request,
the lanes, the update() that has to take the change (UPDATE_BOX_Y) and what happens to the clock
(CLOCK_NOTE_Y). The tabs are written as HTML, in the markup that pymdownx.tabbed produces with
alternate_style, as scripts/figures/simulation_test_figures.py does. The page's numbered lists
stand in Markdown tabs with the same labels (TAB_LABELS), and the theme's content.tabs.link
switches all tabs with one label together. So change the labels here and on the page together.
The numbers in the discs are the steps of those lists. Each caption is HTML.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_sim_figures.py
"""

from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

from board_drawings import Drawing
from board_drawings import escape
from board_drawings import style_element

# The first line of each include: the script that writes it
INCLUDE_COMMENT = '<!-- Written by scripts/figures/imu_sim_figures.py -->'


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
HEIGHT = 380
PHYSICS_LANE = Lane(left=18, right=178)
SPINNER_LANE = Lane(left=186, right=352)
LANE_TOP = 62
# The boxes above the lanes
SPAWNER_BOX = Box(x=18, y=8, width=152, height=38)
QUEUE_BOX = Box(x=200, y=8, width=152, height=38)
# Where the request leaves the queue downward, to the right of the spinner lane's name
REQUEST_X = 338
# The upper edge of the box of the controller manager's update() that has to take the change
UPDATE_BOX_Y = 286
# The baseline of the line below that box, which says what happens to the simulation's clock
CLOCK_NOTE_Y = 342
LINE_HEIGHT = 14

# The tabs' labels, the same as those of the numbered lists on the page
TAB_LABELS = ('With the original plugin', 'With the copy')

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
BG = 'var(--md-default-bg-color)'
RULES = (
    ('.md-typeset .th-figs figure', 'margin:.6em auto 0;width:100%;max-width:480px'),
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


class Figure:
    """Collect the SVG elements of one drawing."""

    def __init__(self, name: str, title: str, description: str) -> None:
        self.name, self.height = name, HEIGHT
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
        """Add a step's number in a filled disc around (x, y), as the shared drawings do."""
        disc = Drawing()
        disc.number_disc(x, y, number, 'step-disc', 'step')
        self.elements.extend(disc.elements)

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
    figure = Figure('th-original', 'The request with the original IMU plugin',
                    'Step 1: the controller spawner, another ROS node, sends a switch_controller '
                    'request, which waits in the global callback queue. Step 2: the physics '
                    'thread calls the IMU plugin’s UpdateChild(), which publishes the IMU '
                    'message and calls ros::spinOnce(). Step 3: ros::spinOnce() takes the '
                    'request from the queue and runs the callback in the physics thread. Step 4: '
                    'the callback notes the change and waits here for update(). Step 5: '
                    'update() never runs, because its thread is waiting. This is a deadlock: '
                    'the clock stops. The ROS spinner thread finds the queue empty.')
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
                ('waits here for update()', '')])
    figure.step(left + 16, 166, 4)
    # The request goes from the queue down the spinner lane and into ros::spinOnce()
    figure.arrow(f'M{REQUEST_X},{QUEUE_BOX.bottom} V150 H{right - 9}', 'request')
    figure.step(REQUEST_X, 112, 3)
    figure.box(left + 4, UPDATE_BOX_Y, right - left - 8, 'stuck',
               [('update()', 'mono stuck-text'), ('never runs: its thread waits', 'note')])
    figure.step(left + 4, UPDATE_BOX_Y, 5)
    figure.text(center, CLOCK_NOTE_Y, 'deadlock: the clock stops', 'stuck-text strong')
    figure.box(SPINNER_LANE.left + 20, 176, SPINNER_LANE.width - 40, 'idle',
               [('finds the queue', 'note'), ('empty', 'note')])
    return figure


def threads_with_the_copy_figure() -> Figure:
    """Return the drawing of the request with the robots' copy of the plugin: no deadlock."""
    figure = Figure('th-copy', 'The request with the robots’ copy of the IMU plugin',
                    'Step 1: the controller spawner sends a switch_controller request, which '
                    'waits in the global callback queue. Step 2: the physics thread calls the IMU '
                    'plugin’s UpdateChild(), which publishes the IMU message and returns. '
                    'Step 3: a ROS spinner thread takes the request and runs the callback, which '
                    'notes the change and waits for update(). The callback waits here, in the '
                    'spinner thread, not in the physics thread. Step 4: the physics thread goes '
                    'on and calls update(), which takes the change. The clock runs on. Step 5: '
                    'the callback returns, and the spawner gets its answer.')
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
    figure.arrow(f'M{center},198 V{UPDATE_BOX_Y - 30}', 'flow')
    figure.text(center, UPDATE_BOX_Y - 12, 'this step or the next', 'note')
    update_bottom = figure.box(left + 4, UPDATE_BOX_Y, width, 'callback',
                               [('update()', 'mono'), ('takes the change', 'note')])
    figure.step(left + 4, UPDATE_BOX_Y, 4)
    figure.text(center, CLOCK_NOTE_Y, 'the clock runs on', 'strong')
    spinner_left, spinner_center = SPINNER_LANE.left, SPINNER_LANE.center
    figure.arrow(f'M{REQUEST_X},{QUEUE_BOX.bottom} V87', 'request')
    figure.rectangle(spinner_left + 4, 88, SPINNER_LANE.width - 8, update_bottom - 88, 'callback')
    figure.lines(spinner_center, 105, [('switch_controller', 'mono'), ('callback', 'note'),
                                       ('notes the change,', ''), ('waits for update()', '')])
    figure.lines(spinner_center, 204, [('the callback waits here,', 'strong'),
                                       ('not in the physics thread', 'note')])
    figure.step(spinner_left + 4, 88, 3)
    update_center = (UPDATE_BOX_Y + update_bottom) / 2
    figure.arrow(f'M{right - 4},{update_center} H{spinner_left + 3}', 'request')
    figure.text(spinner_center, update_center + 4, 'the change is taken', 'note')
    figure.arrow(f'M{spinner_center},{update_bottom} V{update_bottom + 14}', 'flow')
    figure.step(spinner_center - 62, update_bottom + 30, 5)
    figure.lines(spinner_center + 8, update_bottom + 28, [('returns: the spawner', 'note'),
                                                          ('gets its answer', 'note')])
    return figure


def tabs_html(drawings: Sequence[Figure], captions: Sequence[str]) -> str:
    """Return a set of tabs, one per drawing, each with the drawing and its caption.

    The labels are TAB_LABELS. captions are HTML. The tab set is raw HTML, so Markdown leaves
    the SVG's text as it is. Otherwise the glossary would wrap a term such as IMU in an abbr
    element, which a browser doesn't draw inside SVG text.
    """
    inputs = ''.join(
        f'<input{" checked" if index == 0 else ""} id="__tabbed_th_{index + 1}" '
        'name="__tabbed_th" type="radio">' for index in range(len(drawings)))
    labels = ''.join(f'<label for="__tabbed_th_{index + 1}">{escape(label)}</label>'
                     for index, label in enumerate(TAB_LABELS))
    blocks = ''.join(
        f'<div class="tabbed-block">\n<figure>\n{drawing.svg()}\n'
        f'<figcaption>{caption}</figcaption>\n</figure>\n</div>\n'
        for drawing, caption in zip(drawings, captions))
    return (f'{style_element(RULES)}\n<div class="tabbed-set tabbed-alternate th-figs" '
            f'data-tabs="th:{len(drawings)}">{inputs}<div class="tabbed-labels">{labels}</div>\n'
            f'<div class="tabbed-content">\n{blocks}</div>\n</div>\n')


def main() -> None:
    """Write the include."""
    original = ('With gazebo_plugins’ original IMU plugin. Time runs downward, and the '
                'numbers are the steps below. Blue is the <code>switch_controller</code> '
                'request and its callback. The callback runs in the physics thread and waits '
                'for an <code>update()</code> that only this thread can run.')
    copy = ('With the robots’ copy of the plugin. Time runs downward, and the numbers are the '
            'steps below. A ROS spinner thread runs the callback, and the callback waits '
            'there. The physics thread goes on and calls <code>update()</code>.')
    with open('includes/imu-sim-threads.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(tabs_html([threads_with_the_original_figure(),
                                 threads_with_the_copy_figure()], [original, copy]))


if __name__ == '__main__':
    main()
