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
(CLOCK_NOTE_Y). The tabs are written as HTML by board_drawings.tab_set(), in the markup that
pymdownx.tabbed produces with alternate_style. The page's numbered lists stand in Markdown tabs
with the same labels (ORIGINAL_TAB_LABEL and COPY_TAB_LABEL), and the theme's content.tabs.link
switches all tabs with one label together. So change the labels here and on the page together.
The numbers in the discs are the steps of those lists. Each caption is HTML.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_sim_figures.py
"""

import enum
from typing import List
from typing import NamedTuple
from typing import Sequence

from board_drawings import arrowhead_marker
from board_drawings import Drawing
from board_drawings import style_element
from board_drawings import svg_element
from board_drawings import Tab
from board_drawings import tab_set

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
ORIGINAL_TAB_LABEL = 'With the original plugin'
COPY_TAB_LABEL = 'With the copy'


class TextLine(NamedTuple):
    """A line of text on the drawing: the text and its CSS class."""

    text: str
    css_class: str = ''


class ThreadArrowKind(enum.Enum):
    """What an arrow shows. The value is the arrow's CSS class and ends its marker's ID."""

    FLOW = 'flow'  # what a thread does next
    REQUEST = 'request'  # the way of the switch_controller request


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
    """One thread drawing: its name, its accessible texts and the elements of its drawing.

    The figure draws with the shared Drawing and adds what the thread drawings have of their
    own: centered text, rounded rectangles, boxes as high as their text, and arrows.
    """

    def __init__(self, name: str, title: str, description: str) -> None:
        """Initialize the figure metadata and its drawing."""
        self.name, self.height = name, HEIGHT
        self.title, self.description = title, description
        self.drawing = Drawing()

    @property
    def elements(self) -> List[str]:
        """Return the SVG elements of the drawing, in the order they are drawn."""
        return self.drawing.elements

    def rectangle(self, x: float, y: float, width: float, height: float, css_class: str,
                  radius: float = 4) -> None:
        """Add a rectangle with rounded corners and its upper left corner at (x, y)."""
        self.drawing.rect(x, y, width, height, css_class, radius)

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'middle') -> None:
        """Add one line of text, centered at x.

        anchor is start, middle or end.
        """
        self.drawing.text(x, y, text, css_class, anchor)

    def lines(self, x: float, y: float, lines: Sequence[TextLine]) -> None:
        """Add centered text lines, the first with its baseline at y."""
        for index, line in enumerate(lines):
            self.text(x, y + index * LINE_HEIGHT, line.text, line.css_class)

    def box(self, x: float, y: float, width: float, css_class: str,
            lines: Sequence[TextLine]) -> float:
        """Add a box around centered text lines and return the y position of its lower edge."""
        height = len(lines) * LINE_HEIGHT + 10
        self.rectangle(x, y, width, height, css_class)
        self.lines(x + width / 2, y + 17, lines)
        return y + height

    def path(self, path: str, css_class: str) -> None:
        """Add a line along an SVG path, without an arrowhead."""
        self.drawing.line(path, css_class)

    def marker_id(self, kind: ThreadArrowKind) -> str:
        """Return the ID of this drawing's arrowhead marker for a kind of arrow."""
        return f'{self.name}-{kind.value}'

    def arrow(self, path: str, kind: ThreadArrowKind) -> None:
        """Add a line along an SVG path, with an arrowhead of its kind at its end."""
        self.elements.append(f'<path class="{kind.value}" d="{path}" '
                             f'marker-end="url(#{self.marker_id(kind)})"/>')

    def draw_step_number(self, x: float, y: float, number: int) -> None:
        """Add a step's number in a filled disc around (x, y), as the shared drawings do."""
        self.drawing.number_disc(x, y, number, 'step-disc', 'step')

    def svg(self) -> str:
        """Return the drawing as an SVG element with one arrowhead marker per kind of arrow."""
        heads = [arrowhead_marker(self.marker_id(kind), f'{kind.value}-head',
                                  'auto-start-reverse') for kind in ThreadArrowKind]
        return svg_element(self.name, 'th-fig', (WIDTH, self.height), self.title, self.elements,
                           description=self.description, markers=heads)


def draw_request(figure: Figure) -> None:
    """Add the spawner, the callback queue and the request between them (step 1)."""
    spawner, queue = SPAWNER_BOX, QUEUE_BOX
    figure.rectangle(spawner.x, spawner.y, spawner.width, spawner.height, 'box')
    figure.lines(spawner.center_x, spawner.y + 16, [TextLine('controller spawner'),
                                                    TextLine('another ROS node', 'note')])
    figure.rectangle(queue.x, queue.y, queue.width, queue.height, 'queue')
    figure.lines(queue.center_x, queue.y + 16, [TextLine('global callback queue'),
                                                TextLine('a waiting list', 'note')])
    figure.arrow(f'M{spawner.right},{spawner.center_y} H{queue.x - 1}',
                 ThreadArrowKind.REQUEST)
    figure.draw_step_number((spawner.right + queue.x) / 2, spawner.center_y - 12, 1)


def draw_lanes(figure: Figure) -> None:
    """Add the two lanes with their names."""
    for lane, name in ((PHYSICS_LANE, 'physics thread'), (SPINNER_LANE, 'ROS spinner thread')):
        figure.rectangle(lane.left, LANE_TOP, lane.width, figure.height - LANE_TOP - 4, 'lane',
                         6)
        figure.text(lane.center, LANE_TOP + 17, name, 'lane-name')
    figure.elements.append(f'<text class="note" transform="translate(13,{LANE_TOP + 4}) '
                           'rotate(-90)" text-anchor="end">time</text>')
    figure.arrow(f'M9,{LANE_TOP + 34} V{figure.height - 10}', ThreadArrowKind.FLOW)


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
    figure.lines(center, 104, [TextLine('UpdateChild()', 'mono'),
                               TextLine('publishes the IMU message', 'note')])
    figure.draw_step_number(left + 4, 88, 2)
    figure.path(f'M{left + 10},258 V126 H{right - 10} V258', 'box')
    figure.path(f'M{left + 10},258 V272 M{right - 10},258 V272', 'open')
    figure.lines(center, 142, [TextLine('ros::spinOnce()', 'mono'),
                               TextLine('runs the callback here', 'note')])
    figure.box(left + 16, 166, right - left - 32, 'callback',
               [TextLine('switch_controller', 'mono'), TextLine('callback', 'note'),
                TextLine('notes the change,'), TextLine('waits here for update()')])
    figure.draw_step_number(left + 16, 166, 4)
    # The request goes from the queue down the spinner lane and into ros::spinOnce()
    figure.arrow(f'M{REQUEST_X},{QUEUE_BOX.bottom} V150 H{right - 9}', ThreadArrowKind.REQUEST)
    figure.draw_step_number(REQUEST_X, 112, 3)
    figure.box(left + 4, UPDATE_BOX_Y, right - left - 8, 'stuck',
               [TextLine('update()', 'mono stuck-text'),
                TextLine('never runs: its thread waits', 'note')])
    figure.draw_step_number(left + 4, UPDATE_BOX_Y, 5)
    figure.text(center, CLOCK_NOTE_Y, 'deadlock: the clock stops', 'stuck-text strong')
    figure.box(SPINNER_LANE.left + 20, 176, SPINNER_LANE.width - 40, 'idle',
               [TextLine('finds the queue', 'note'), TextLine('empty', 'note')])
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
               [TextLine('UpdateChild()', 'mono'), TextLine('publishes the IMU message', 'note'),
                TextLine('and returns', 'note')])
    figure.draw_step_number(left + 4, 88, 2)
    figure.box(left + 4, 150, width, 'box', [TextLine('update()', 'mono'),
                                             TextLine('runs the controllers', 'note')])
    figure.arrow(f'M{center},198 V{UPDATE_BOX_Y - 30}', ThreadArrowKind.FLOW)
    figure.text(center, UPDATE_BOX_Y - 12, 'this step or the next', 'note')
    update_bottom = figure.box(left + 4, UPDATE_BOX_Y, width, 'callback',
                               [TextLine('update()', 'mono'),
                                TextLine('takes the change', 'note')])
    figure.draw_step_number(left + 4, UPDATE_BOX_Y, 4)
    figure.text(center, CLOCK_NOTE_Y, 'the clock runs on', 'strong')
    spinner_left, spinner_center = SPINNER_LANE.left, SPINNER_LANE.center
    figure.arrow(f'M{REQUEST_X},{QUEUE_BOX.bottom} V87', ThreadArrowKind.REQUEST)
    figure.rectangle(spinner_left + 4, 88, SPINNER_LANE.width - 8, update_bottom - 88, 'callback')
    figure.lines(spinner_center, 105,
                 [TextLine('switch_controller', 'mono'), TextLine('callback', 'note'),
                  TextLine('notes the change,'), TextLine('waits for update()')])
    figure.lines(spinner_center, 204, [TextLine('the callback waits here,', 'strong'),
                                       TextLine('not in the physics thread', 'note')])
    figure.draw_step_number(spinner_left + 4, 88, 3)
    update_center = (UPDATE_BOX_Y + update_bottom) / 2
    figure.arrow(f'M{right - 4},{update_center} H{spinner_left + 3}', ThreadArrowKind.REQUEST)
    figure.text(spinner_center, update_center + 4, 'the change is taken', 'note')
    figure.arrow(f'M{spinner_center},{update_bottom} V{update_bottom + 14}', ThreadArrowKind.FLOW)
    figure.draw_step_number(spinner_center - 62, update_bottom + 30, 5)
    figure.lines(spinner_center + 8, update_bottom + 28,
                 [TextLine('returns: the spawner', 'note'), TextLine('gets its answer', 'note')])
    return figure


def figure_tab(label: str, drawing: Figure, caption_html: str) -> Tab:
    """Return a tab with a drawing and, below it, the drawing's caption as HTML."""
    return Tab(label, f'<figure>\n{drawing.svg()}\n'
                      f'<figcaption>{caption_html}</figcaption>\n</figure>')


def thread_tabs() -> List[Tab]:
    """Return the two tabs, each with its label, its drawing and its caption."""
    original = ('With gazebo_plugins’ original IMU plugin. Time runs downward, and the '
                'numbers are the steps below. Blue is the <code>switch_controller</code> '
                'request and its callback. The callback runs in the physics thread and waits '
                'for an <code>update()</code> that only this thread can run.')
    copy = ('With the robots’ copy of the plugin. Time runs downward, and the numbers are the '
            'steps below. A ROS spinner thread runs the callback, and the callback waits '
            'there. The physics thread goes on and calls <code>update()</code>.')
    return [figure_tab(ORIGINAL_TAB_LABEL, threads_with_the_original_figure(), original),
            figure_tab(COPY_TAB_LABEL, threads_with_the_copy_figure(), copy)]


def main() -> None:
    """Write the include."""
    with open('includes/imu-sim-threads.html', 'w', encoding='utf-8') as include:
        include.write(f'{INCLUDE_COMMENT}\n{style_element(RULES)}\n'
                      f'{tab_set("th", "th-figs", thread_tabs())}\n')


if __name__ == '__main__':
    main()
