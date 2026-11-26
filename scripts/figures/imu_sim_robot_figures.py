"""Generate the figures of the IMU Simulation pages that show a robot.

It writes three includes, as inline SVG that follows the site's light and dark colors:

- includes/imu-sim-frames.html, for docs/packages/imu/simulation/position-and-axes.md:
  chassis_link, imu_joint and imu_link, in one tab per robot. Remo from above and from the
  side, and DiffBot from the side, with the placeholder mounting of DiffBot's URDF model. The
  tabs are written as HTML by board_drawings.tab_set(), in the markup that pymdownx.tabbed
  produces with alternate_style. Their labels (REMO_TAB_LABEL and DIFFBOT_TAB_LABEL) are those
  of the page's Markdown tabs, and the theme's content.tabs.link switches all tabs with one
  label together. So change the labels here and on the page together.
- includes/imu-sim-angular-velocity.html, for docs/packages/imu/simulation/index.md: the angular
  velocity about each of imu_link's axes, with the direction that counts as positive.
- includes/imu-sim-specific-force.html, for the same page: gravity, the acceleration and what the
  sensor reads, at rest, while the robot speeds up and in a turn. The three drawings are a
  level, noise-free example: the floor is level, the motion is exactly the named one, and the
  readings have no noise. They don't show a recorded run.

Remo is a schematic, drawn to scale from the numbers of its public model, remo_description 2.5.0
(urdf/remo.urdf.xacro and config/remo/*.yaml). DiffBot is a schematic from the numbers of
diffbot_description (urdf/diffbot.urdf.xacro and config/diffbot/*.yaml) at the commit that
mkdocs.yml pins as imu_sim_src_raw. The axes have ROS's colors: x red, y green and z blue. The
pages' text repeats the numbers, so change them together.

Each caption is Markdown, so a link in it is relative to the page that includes the figure.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_sim_robot_figures.py
"""

import enum
import math
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

from board_drawings import Drawing
from board_drawings import figure_element
from board_drawings import PanelWidths
from board_drawings import Point
from board_drawings import style_element
from board_drawings import svg_element
from board_drawings import svg_subscript
from board_drawings import Tab
from board_drawings import tab_set

# The first line of each include: the script that writes it
INCLUDE_COMMENT = '<!-- Written by scripts/figures/imu_sim_robot_figures.py -->'


class Vector3(NamedTuple):
    """A position in m or a direction without a unit, in chassis_link's axes."""

    x: float  # forward
    y: float  # to the left
    z: float  # up


class Box(NamedTuple):
    """A box with its edges along chassis_link's axes, in m."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


class Axis(NamedTuple):
    """One axis of a frame: its letter, the frame's origin in m and the axis's direction."""

    name: str
    origin: Vector3
    direction: Vector3


class WheelSide(enum.Enum):
    """A drive wheel's side of the robot. The value is the sign of the wheel's y position."""

    LEFT = 1
    RIGHT = -1


class InsetBounds(NamedTuple):
    """The edges of a box on the drawing that shows a part of it magnified, in SVG units."""

    left: float
    top: float
    right: float
    bottom: float


# Remo at remo_description 2.5.0, in chassis_link's axes, in m.
# config/remo/base.yaml, base: 0.14 x 0.08 x 0.03, with chassis_link at the center of its top
PLATE = Box(-0.07, 0.07, -0.04, 0.04, -0.03, 0.0)
# base.yaml, rpi_deck: 0.14 x 0.09 x 0.002, on the plate
DECK = Box(-0.07, 0.07, -0.045, 0.045, 0.0, 0.002)
# config/remo/components.yaml, breadboard: 0.055 x 0.08 x 0.01. remo.urdf.xacro puts its center
# 4 cm behind chassis_link. Drawn lying on the deck
BREADBOARD = Box(-0.0675, -0.0125, -0.04, 0.04, 0.002, 0.012)
# remo.urdf.xacro, axle_x: the wheel axle lies 3.6 cm in front of chassis_link
AXLE_X = 0.036
# config/remo/front_wheel.yaml: z_offset, radius, length, and the wheel separation that its
# comment names
AXLE_Z = -0.0336
WHEEL_RADIUS = 0.033
WHEEL_WIDTH = 0.025
WHEEL_SEPARATION = 0.144
FLOOR_Z = AXLE_Z - WHEEL_RADIUS
# The hub that the schematic draws in the middle of a wheel
HUB_RADIUS = 0.3 * WHEEL_RADIUS
# config/remo/caster_wheel.yaml: dx and the ball's radius
CASTER_X = -0.051
CASTER_RADIUS = 0.012
# remo.urdf.xacro, imu_xyz and imu_rpy: where imu_joint puts imu_link on chassis_link
IMU_POSITION = Vector3(-0.0401, 0.0234, 0.013)
IMU_YAW = -math.pi / 2
# The BNO055 board along imu_link's x, y and z axes: diffbot's config/diffbot/sensors.yaml
IMU_BOARD_SIZE = Vector3(0.027, 0.020, 0.004)

# DiffBot at the pinned commit of diffbot_description, in chassis_link's axes, in m.
# config/diffbot/base.yaml, base: 0.195 x 0.08 x 0.005, with chassis_link at its center
DIFFBOT_PLATE = Box(-0.0975, 0.0975, -0.04, 0.04, -0.0025, 0.0025)
# config/diffbot/front_wheel.yaml: x_offset, the wheels' distance from the plate's front edge.
# diffbot.urdf.xacro, axle_x: the wheel axle lies this far in front of chassis_link
DIFFBOT_AXLE_X = 0.195 / 2 - 0.079
# front_wheel.yaml: z_offset and radius
DIFFBOT_AXLE_Z = -0.017
DIFFBOT_WHEEL_RADIUS = 0.0333
DIFFBOT_FLOOR_Z = DIFFBOT_AXLE_Z - DIFFBOT_WHEEL_RADIUS
# config/diffbot/caster_wheel.yaml: the caster wheel's center, which is the hub's dx and dz plus
# the wheel's dx and dz, and the wheel's radius
DIFFBOT_CASTER_X = -0.077 - 0.016
DIFFBOT_CASTER_Z = 0.0045 - 0.04
DIFFBOT_CASTER_RADIUS = 0.015
# diffbot.urdf.xacro, imu_xyz and imu_rpy: placeholders, the mounting on DiffBot isn't measured
DIFFBOT_IMU_POSITION = Vector3(0.0, 0.0, 0.01)

# chassis_link's origin and the directions of its x, y and z axes
CHASSIS_ORIGIN = Vector3(0.0, 0.0, 0.0)
FORWARD = Vector3(1.0, 0.0, 0.0)
LEFT = Vector3(0.0, 1.0, 0.0)
UP = Vector3(0.0, 0.0, 1.0)
# A frame's x, y and z axes as directions in chassis_link's axes
FrameAxes = Tuple[Vector3, Vector3, Vector3]
CHASSIS_AXES: FrameAxes = (FORWARD, LEFT, UP)
IMU_AXES: FrameAxes = (
    Vector3(math.cos(IMU_YAW), math.sin(IMU_YAW), 0.0),
    Vector3(-math.sin(IMU_YAW), math.cos(IMU_YAW), 0.0),
    UP)
AXIS_NAMES = ('x', 'y', 'z')
# The labels of the frames' tabs, the same as those of the Markdown tabs on the page
REMO_TAB_LABEL = 'Remo'
DIFFBOT_TAB_LABEL = 'DiffBot'
# An axis whose arrow on the drawing would be shorter than this share of its length points along
# the viewer's line of sight. The drawing shows it as a ring
RING_SHARE = 0.2

# The examples of the specific force figure
GAZEBO_GRAVITY = 9.8  # m/s², Gazebo's gravity in the robots' worlds
# Below the drive controller's limit of 0.6 m/s² (diffbot_control.yaml, max_acceleration)
FORWARD_ACCELERATION = 0.5  # m/s²
TURN_RATE = 0.5  # rad/s, the left turn of test_imu.py

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
FG_LIGHTER = 'var(--md-default-fg-color--lighter)'
FG_LIGHTEST = 'var(--md-default-fg-color--lightest)'
BG = 'var(--md-default-bg-color)'
RULES = (
    ('.md-typeset figure.imu-figure', 'margin:1em auto;width:100%;max-width:none'),
    ('.md-typeset .imu-tabs figure.imu-figure', 'margin:.6em auto 0'),
    ('.imu-figure .imu-panels', 'display:flex;flex-wrap:wrap;justify-content:center;'
                                'align-items:flex-start;gap:.8rem 1.6rem'),
    ('.imu-figure .imu-panel', 'flex:1 1 var(--imu-basis,300px);max-width:var(--imu-max,400px)'),
    ('.md-typeset .imu-figure .imu-panel p', 'margin:.3em 0 0;font-size:.7rem;text-align:left'),
    ('.md-typeset .imu-figure figcaption', 'max-width:36rem'),
    ('.imu-fig', 'display:block;width:100%;height:auto;--imu-x:#d32f2f;--imu-y:#2e7d32;'
                 '--imu-z:#1565c0;--imu-board:#9db9e6;--imu-gravity:#546e7a;'
                 '--imu-acceleration:#e65100;--imu-reading:#7b1fa2;'
                 'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .imu-fig',
     '--imu-x:#ff7b72;--imu-y:#6fcf73;--imu-z:#6cb2ff;--imu-board:#3d5a8c;'
     '--imu-gravity:#a7b6be;--imu-acceleration:#ffa94d;--imu-reading:#d79bea'),
    ('.imu-fig text', f'fill:{FG};font-size:12px'),
    ('.imu-fig .title', 'font-size:13px;font-weight:700'),
    ('.imu-fig .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                       'font-size:11.5px'),
    ('.imu-fig .note', f'font-size:11.5px;fill:{FG_LIGHT}'),
    ('.imu-fig .strong', 'font-weight:700'),
    ('.imu-fig .sub', 'font-size:10px'),
    ('.imu-fig .halo', f'paint-order:stroke;stroke:{BG};stroke-width:3.5px;'
                       'stroke-linejoin:round'),
    ('.imu-fig .body', f'fill:{FG_LIGHTEST};stroke:{FG_LIGHTER};stroke-width:1'),
    ('.imu-fig .wheel', f'fill:{FG_LIGHTER};stroke:{FG_LIGHT};stroke-width:1'),
    ('.imu-fig .part', f'fill:{BG};stroke:{FG_LIGHTER};stroke-width:1'),
    ('.imu-fig .board', f'fill:var(--imu-board);stroke:{FG_LIGHT};stroke-width:1'),
    ('.imu-fig .floor', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.5'),
    ('.imu-fig .joint', f'fill:none;stroke:{FG};stroke-width:1.6;stroke-dasharray:5 3'),
    ('.imu-fig .guide', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:2 3'),
    ('.imu-fig .lens', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    ('.imu-fig .inset', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:1'),
    ('.imu-fig .dot', f'fill:{FG}'),
    ('.imu-fig .x', 'stroke:var(--imu-x);fill:var(--imu-x)'),
    ('.imu-fig .y', 'stroke:var(--imu-y);fill:var(--imu-y)'),
    ('.imu-fig .z', 'stroke:var(--imu-z);fill:var(--imu-z)'),
    ('.imu-fig .gravity', 'stroke:var(--imu-gravity);fill:var(--imu-gravity)'),
    ('.imu-fig .acceleration', 'stroke:var(--imu-acceleration);fill:var(--imu-acceleration)'),
    ('.imu-fig .reading', 'stroke:var(--imu-reading);fill:var(--imu-reading)'),
    ('.imu-fig .motion', f'stroke:{FG_LIGHT};fill:{FG_LIGHT}'),
    ('.imu-fig .context', 'opacity:.62'),
    ('.imu-fig .shaft', 'fill:none;stroke-width:2.4;stroke-linecap:round'),
    ('.imu-fig .turn', 'stroke-width:2'),
    ('.imu-fig .thin', 'stroke-width:1.5'),
    ('.imu-fig .dashed', 'stroke-dasharray:4 3'),
    ('.imu-fig .head', 'stroke:none'),
    ('.imu-fig .ring', f'fill:{BG};stroke-width:1.8'),
    ('.imu-fig text.x,.imu-fig text.y,.imu-fig text.z,.imu-fig text.gravity,'
     '.imu-fig text.acceleration,.imu-fig text.reading', 'stroke:none;font-weight:700'),
    ('.imu-fig text.halo', f'stroke:{BG}'),
)


def dot(first: Vector3, second: Vector3) -> float:
    """Return the dot product of two vectors."""
    return sum(first_component * second_component
               for first_component, second_component in zip(first, second))


def cross(first: Vector3, second: Vector3) -> Vector3:
    """Return the cross product of two vectors."""
    return Vector3(first.y * second.z - first.z * second.y,
                   first.z * second.x - first.x * second.z,
                   first.x * second.y - first.y * second.x)


def offset_point(start: Vector3, direction: Vector3, length: float = 1.0) -> Vector3:
    """Return the position that lies length times direction away from start."""
    return Vector3(start.x + length * direction.x, start.y + length * direction.y,
                   start.z + length * direction.z)


def normalized(vector: Vector3) -> Vector3:
    """Return the vector with length 1."""
    length = math.sqrt(dot(vector, vector))
    return Vector3(vector.x / length, vector.y / length, vector.z / length)


class View(NamedTuple):
    """A parallel projection of chassis_link's axes onto the drawing.

    right and up are the directions that point to the right and upward on the drawing. scale is
    in SVG units per m, and origin is where chassis_link's origin lies on the drawing.
    """

    right: Vector3
    up: Vector3
    scale: float
    origin: Point

    def project(self, position: Vector3) -> Point:
        """Return where a position in m lies on the drawing."""
        return Point(self.origin.x + self.scale * dot(position, self.right),
                     self.origin.y - self.scale * dot(position, self.up))

    def on_drawing(self, direction: Vector3, length: float) -> Point:
        """Return a direction as an offset on the drawing, for an arrow of length SVG units."""
        return Point(length * dot(direction, self.right), -length * dot(direction, self.up))

    def toward_viewer(self, direction: Vector3) -> float:
        """Return how much of a direction points out of the drawing, toward the viewer."""
        return dot(direction, cross(self.right, self.up))


def view_from_above(scale: float, origin: Point) -> View:
    """Return the view from above, with the robot's front at the top of the drawing."""
    return View(right=Vector3(0.0, -1.0, 0.0), up=FORWARD, scale=scale, origin=origin)


def view_from_the_side(scale: float, origin: Point) -> View:
    """Return the view from the robot's right side, with the robot's front on the right."""
    return View(right=FORWARD, up=UP, scale=scale, origin=origin)


def view_from_behind(scale: float, origin: Point, azimuth: float, elevation: float) -> View:
    """Return a view from behind and above the robot.

    azimuth turns the viewer to the robot's right, and elevation lifts the viewer, both in
    degrees.
    """
    turn, lift = math.radians(azimuth), math.radians(elevation)
    right = Vector3(math.sin(turn), -math.cos(turn), 0.0)
    up = Vector3(math.sin(lift) * math.cos(turn), math.sin(lift) * math.sin(turn),
                 math.cos(lift))
    return View(right, up, scale, origin)


class Figure:
    """One drawing of a robot: its name, size, accessible texts and the elements of its drawing.

    The figure draws with the shared Drawing, at points that a View projects, rounded to a tenth
    of an SVG unit. It adds what these drawings have of their own: shapes through projected
    points, and arrows whose heads are as large as each arrow needs.
    """

    def __init__(self, name: str, width: float, height: float, title: str,
                 description: str) -> None:
        """Initialize the figure metadata and its drawing."""
        self.name, self.width, self.height = name, width, height
        self.title, self.description = title, description
        self.drawing = Drawing()

    @property
    def elements(self) -> List[str]:
        """Return the SVG elements of the drawing, in the order they are drawn."""
        return self.drawing.elements

    def polygon(self, points: Sequence[Point], css_class: str) -> None:
        """Add a closed shape through the points."""
        corners = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.elements.append(f'<polygon class="{css_class}" points="{corners}"/>')

    def rectangle(self, first: Point, second: Point, css_class: str, radius: float = 0) -> None:
        """Add a rectangle between two opposite corners."""
        x, y = min(first.x, second.x), min(first.y, second.y)
        width, height = abs(first.x - second.x), abs(first.y - second.y)
        self.drawing.rect(round(x, 1), round(y, 1), round(width, 1), round(height, 1), css_class,
                          radius)

    def circle(self, center: Point, radius: float, css_class: str) -> None:
        """Add a circle around center."""
        self.drawing.circle(round(center.x, 1), round(center.y, 1), round(radius, 1), css_class)

    def polyline(self, points: Sequence[Point], css_class: str) -> None:
        """Add an open line through the points."""
        path = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.elements.append(f'<polyline class="{css_class}" points="{path}"/>')

    def draw_arrowhead(self, tip: Point, direction: Point, css_class: str,
                       size: float = 8.0) -> None:
        """Add an arrowhead whose tip points along direction."""
        length = math.hypot(*direction)
        along = Point(direction.x / length, direction.y / length)
        across = Point(-along.y, along.x)
        base = Point(tip.x - size * along.x, tip.y - size * along.y)
        half_width = 0.42 * size
        self.polygon([tip,
                      Point(base.x + half_width * across.x, base.y + half_width * across.y),
                      Point(base.x - half_width * across.x, base.y - half_width * across.y)],
                     f'head {css_class}')

    def arrow(self, start: Point, end: Point, css_class: str, size: float = 8.0) -> None:
        """Add a straight arrow from start to end."""
        direction = Point(end.x - start.x, end.y - start.y)
        length = math.hypot(*direction)
        shaft_end = Point(end.x - 0.8 * size * direction.x / length,
                          end.y - 0.8 * size * direction.y / length)
        self.polyline([start, shaft_end], f'shaft {css_class}')
        self.draw_arrowhead(end, direction, css_class, size)

    def curved_arrow(self, points: Sequence[Point], css_class: str, size: float = 7.0) -> None:
        """Add an arrow along the points, with its head at the last one."""
        self.polyline(points[:-1], f'shaft {css_class}')
        tip, before_the_tip = points[-1], points[-3]
        direction = Point(tip.x - before_the_tip.x, tip.y - before_the_tip.y)
        self.draw_arrowhead(tip, direction, css_class, size)

    def text(self, position: Point, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of text at the requested anchor.

        anchor is start, middle or end.
        """
        self.drawing.text(round(position.x, 1), round(position.y, 1), text, css_class, anchor)

    def text_markup(self, position: Point, markup: str, css_class: str = '',
                    anchor: str = 'start') -> None:
        """Add one line of text from SVG markup, such as a symbol with a subscript.

        anchor is start, middle or end.
        """
        self.drawing.text_markup(round(position.x, 1), round(position.y, 1), markup, css_class,
                                 anchor)

    def svg(self) -> str:
        """Return the drawing as an SVG element without markers: its arrows draw their heads."""
        return svg_element(self.name, 'imu-fig', (self.width, self.height), self.title,
                           self.elements, description=self.description, markers=())


def box_corners(box: Box) -> List[Vector3]:
    """Return the eight corners of a box."""
    return [Vector3(x, y, z) for x in (box.x_min, box.x_max) for y in (box.y_min, box.y_max)
            for z in (box.z_min, box.z_max)]


def remo_imu_board() -> Box:
    """Return the BNO055 board as a box in chassis_link's axes.

    imu_link is turned by -90 degrees about z, so the board's x edge lies along the robot's y axis.
    """
    length, width, height = IMU_BOARD_SIZE
    x, y, z = IMU_POSITION
    return Box(x - width / 2, x + width / 2, y - length / 2, y + length / 2,
               z - height / 2, z + height / 2)


def draw_box_outline(figure: Figure, view: View, box: Box, css_class: str,
                     radius: float = 0) -> None:
    """Add a box as the rectangle that a view along one of the axes shows."""
    points = [view.project(corner) for corner in box_corners(box)]
    figure.rectangle(Point(min(point.x for point in points), min(point.y for point in points)),
                     Point(max(point.x for point in points), max(point.y for point in points)),
                     css_class, radius)


def wheel_center(side: WheelSide) -> Vector3:
    """Return the center of the left or the right wheel."""
    return Vector3(AXLE_X, side.value * WHEEL_SEPARATION / 2, AXLE_Z)


def draw_remo_from_above(figure: Figure, view: View) -> None:
    """Add Remo as the view from above shows it: wheels, deck, breadboard and IMU board."""
    for side in WheelSide:
        x, y, z = wheel_center(side)
        wheel_from_above = Box(x - WHEEL_RADIUS, x + WHEEL_RADIUS, y - WHEEL_WIDTH / 2,
                               y + WHEEL_WIDTH / 2, z, z)
        draw_box_outline(figure, view, wheel_from_above, 'wheel', 4)
    draw_box_outline(figure, view, DECK, 'body', 5)
    draw_box_outline(figure, view, BREADBOARD, 'part', 2)
    draw_box_outline(figure, view, remo_imu_board(), 'board', 1.5)


def draw_remo_from_the_side(figure: Figure, view: View) -> None:
    """Add Remo as the view from its right side shows it, standing on the floor."""
    caster = view.project(Vector3(CASTER_X, 0.0, FLOOR_Z + CASTER_RADIUS))
    figure.circle(caster, CASTER_RADIUS * view.scale, 'wheel')
    draw_box_outline(figure, view, PLATE, 'body', 2)
    draw_box_outline(figure, view, DECK, 'body')
    draw_box_outline(figure, view, BREADBOARD, 'part', 1.5)
    draw_box_outline(figure, view, remo_imu_board(), 'board', 1)
    wheel = view.project(wheel_center(WheelSide.RIGHT))
    figure.circle(wheel, WHEEL_RADIUS * view.scale, 'wheel')
    figure.circle(wheel, HUB_RADIUS * view.scale, 'body')


def draw_floor(figure: Figure, view: View, left: float, right: float,
               floor_z: float = FLOOR_Z) -> None:
    """Add the floor as a line under the wheels, between two x positions on the drawing.

    floor_z is the floor's height in chassis_link's axes, in m. The default is Remo's.
    """
    height = view.project(Vector3(0.0, 0.0, floor_z)).y
    figure.polyline([Point(left, height), Point(right, height)], 'floor')


def visible_faces(view: View, box: Box) -> List[List[Vector3]]:
    """Return the corners of each face of a box that faces the viewer."""
    low = Vector3(box.x_min, box.y_min, box.z_min)
    high = Vector3(box.x_max, box.y_max, box.z_max)
    faces = []
    for axis in range(3):
        first, second = [other for other in range(3) if other != axis]
        for value, outward in ((low[axis], -1.0), (high[axis], 1.0)):
            normal = [0.0, 0.0, 0.0]
            normal[axis] = outward
            if view.toward_viewer(Vector3(*normal)) <= 0:
                continue
            corners = []
            for first_coordinate, second_coordinate in (
                    (low[first], low[second]), (high[first], low[second]),
                    (high[first], high[second]), (low[first], high[second])):
                corner = [0.0, 0.0, 0.0]
                corner[axis], corner[first], corner[second] = (
                    value, first_coordinate, second_coordinate)
                corners.append(Vector3(*corner))
            faces.append(corners)
    return faces


def draw_box(figure: Figure, view: View, box: Box, css_class: str) -> None:
    """Add the faces of a box that the viewer sees."""
    for face in visible_faces(view, box):
        figure.polygon([view.project(corner) for corner in face], css_class)


def convex_hull(points: Sequence[Point]) -> List[Point]:
    """Return the corners of the smallest convex shape around the points."""
    ordered = sorted(set(points))

    def turns_left(start: Point, middle: Point, end: Point) -> bool:
        """Return whether the ordered vertices make a left turn."""
        return ((middle.x - start.x) * (end.y - start.y)
                - (middle.y - start.y) * (end.x - start.x)) > 0

    def hull_chain(sequence: Sequence[Point]) -> List[Point]:
        """Build one hull chain and omit its shared endpoint."""
        hull: List[Point] = []
        for point in sequence:
            while len(hull) >= 2 and not turns_left(hull[-2], hull[-1], point):
                hull.pop()
            hull.append(point)
        return hull[:-1]

    return hull_chain(ordered) + hull_chain(ordered[::-1])


def wheel_rim(view: View, side: WheelSide, y_offset: float, radius: float) -> List[Point]:
    """Return a circle around a wheel's axle on the drawing, y_offset from the wheel's center."""
    x, y, z = wheel_center(side)
    return [view.project(Vector3(x + radius * math.cos(math.radians(angle)), y + y_offset,
                                 z + radius * math.sin(math.radians(angle))))
            for angle in range(0, 360, 10)]


def draw_wheel(figure: Figure, view: View, side: WheelSide) -> None:
    """Add a wheel as a cylinder, with the face that the viewer sees."""
    to_the_right = Vector3(0.0, -1.0, 0.0)
    near = -WHEEL_WIDTH / 2 if view.toward_viewer(to_the_right) > 0 else WHEEL_WIDTH / 2
    rims = wheel_rim(view, side, near, WHEEL_RADIUS) + wheel_rim(view, side, -near, WHEEL_RADIUS)
    figure.polygon(convex_hull(rims), 'wheel')
    figure.polygon(wheel_rim(view, side, near, WHEEL_RADIUS), 'wheel')
    figure.polygon(wheel_rim(view, side, near, HUB_RADIUS), 'body')


def draw_remo_from_behind(figure: Figure, view: View) -> None:
    """Add Remo as a view from behind and above shows it, the far parts first."""
    draw_wheel(figure, view, WheelSide.LEFT)
    caster = view.project(Vector3(CASTER_X, 0.0, FLOOR_Z + CASTER_RADIUS))
    figure.circle(caster, CASTER_RADIUS * view.scale, 'wheel')
    draw_box(figure, view, PLATE, 'body')
    draw_box(figure, view, DECK, 'body')
    draw_box(figure, view, BREADBOARD, 'part')
    draw_box(figure, view, remo_imu_board(), 'board')
    draw_wheel(figure, view, WheelSide.RIGHT)


def points_along_the_line_of_sight(view: View, direction: Vector3) -> bool:
    """Return whether a direction points at the viewer or away from the viewer.

    A direction counts as along the line of sight within the projected-length threshold: when
    its arrow on the drawing would be shorter than RING_SHARE of its length.
    """
    return math.hypot(*view.on_drawing(direction, 1.0)) < RING_SHARE


def draw_axis_ring(figure: Figure, view: View, axis: Axis,
                   label_offset: Optional[Point]) -> None:
    """Add an axis along the line of sight as a ring around the frame's origin.

    The ring holds a dot when the axis points out of the drawing, and a cross when it points
    into the drawing. label_offset is where the letter goes, measured from the ring's center.
    Without it, the ring gets no letter.
    """
    center = view.project(axis.origin)
    figure.circle(center, 5.5, f'ring {axis.name}')
    if view.toward_viewer(axis.direction) > 0:
        figure.circle(center, 2.0, f'head {axis.name}')
    else:
        for sign in (1, -1):
            figure.polyline([Point(center.x - 3.2, center.y - 3.2 * sign),
                             Point(center.x + 3.2, center.y + 3.2 * sign)],
                            f'shaft thin {axis.name}')
    if label_offset:
        figure.text(Point(center.x + label_offset.x, center.y + label_offset.y), axis.name,
                    f'{axis.name} halo', 'middle')


def draw_axis_arrow(figure: Figure, view: View, axis: Axis, length: float,
                    label_offset: Optional[Point]) -> None:
    """Add an axis as an arrow of length SVG units from the frame's origin, with its letter.

    label_offset is where the letter goes, measured from the arrow's tip. Without it, the letter
    goes beyond the tip.
    """
    start = view.project(axis.origin)
    offset = view.on_drawing(axis.direction, length)
    end = Point(start.x + offset.x, start.y + offset.y)
    figure.arrow(start, end, axis.name)
    reach = math.hypot(*offset)
    beyond = Point(9 * offset.x / reach, 9 * offset.y / reach + 4)
    label = label_offset or beyond
    figure.text(Point(end.x + label.x, end.y + label.y), axis.name, f'{axis.name} halo',
                'middle')


def draw_frame(figure: Figure, view: View, origin: Vector3, axes: FrameAxes, length: float,
               label_offsets: Optional[Dict[str, Point]] = None) -> None:
    """Add a frame's three axes in ROS's colors, the arrows first and then the rings.

    An axis along the drawing is an arrow of length SVG units, and an axis along the line of
    sight is a ring. label_offsets moves an axis's letter, as draw_axis_arrow() and
    draw_axis_ring() describe.
    """
    offsets = label_offsets or {}
    frame_axes = [Axis(name=name, origin=origin, direction=direction)
                  for name, direction in zip(AXIS_NAMES, axes)]
    rings = [axis for axis in frame_axes if points_along_the_line_of_sight(view, axis.direction)]
    for axis in frame_axes:
        if axis not in rings:
            draw_axis_arrow(figure, view, axis, length, offsets.get(axis.name))
    for axis in rings:
        draw_axis_ring(figure, view, axis, offsets.get(axis.name))


def draw_joint(figure: Figure, view: View, imu_position: Optional[Vector3] = None) -> None:
    """Add imu_joint as a dashed line from chassis_link's origin to imu_link's origin.

    imu_position is imu_link's origin in chassis_link's axes, in m. The default is Remo's.
    """
    end = IMU_POSITION if imu_position is None else imu_position
    figure.polyline([view.project(CHASSIS_ORIGIN), view.project(end)], 'joint')


def format_number(value: float) -> str:
    """Return a value with at most two decimals, without a plus sign."""
    return f'{value:.2f}'.rstrip('0').rstrip('.')


def format_centimeters(meters: float) -> str:
    """Return a length in cm with at most two decimals, without the unit."""
    return format_number(100 * meters)


def format_degrees(radians: float) -> str:
    """Return an angle in whole degrees with the degree sign, and a minus sign below zero."""
    return f'{math.degrees(radians):.0f}°'.replace('-', '\u2212')


def frames_from_above_figure() -> Figure:
    """Return the drawing of the two frames and their joint on Remo, from above."""
    x, y, z = IMU_POSITION
    back, left, up = format_centimeters(-x), format_centimeters(y), format_centimeters(z)
    figure = Figure('imu-frames-above', 340, 306, 'chassis_link, imu_joint and imu_link on Remo, '
                    'from above',
                    'Remo with the Raspberry Pi deck, from above, with its front at the top. '
                    'chassis_link is at the center '
                    'of the base plate, with its x axis forward and its y axis to the left. '
                    f'imu_link is at the IMU board on the breadboard, {back} cm behind and '
                    f'{left} cm left of chassis_link. Its x axis points to the right and its y '
                    'axis forward. The z axes of both point up, out of the drawing. A dashed '
                    'line, the fixed joint imu_joint, connects the two origins.')
    view = view_from_above(1500, Point(170, 152))
    figure.text(Point(10, 18), 'Remo with the Raspberry Pi deck, from above', 'title')
    figure.text(Point(170, 38), 'front', 'note', 'middle')
    draw_remo_from_above(figure, view)
    draw_joint(figure, view)
    draw_frame(figure, view, CHASSIS_ORIGIN, CHASSIS_AXES, 42,
               {'y': Point(0, -8), 'z': Point(11, -7)})
    draw_frame(figure, view, IMU_POSITION, IMU_AXES, 42, {'y': Point(-11, 12), 'z': Point(0, 28)})
    figure.text(Point(184, 164), 'chassis_link', 'mono halo')
    figure.text(Point(98, 216), 'imu_link', 'mono halo', 'end')
    figure.text(Point(162, 191), 'imu_joint', 'mono halo')
    figure.text(Point(162, 204), 'a fixed joint', 'note halo')
    figure.text(Point(226, 248), 'breadboard', 'note', 'end')
    figure.text(Point(170, 279), f'imu_joint puts imu_link {back} cm back, {left} cm left', 'note',
                'middle')
    figure.text(Point(170, 295),
                f'and {up} cm up, and turns it by {format_degrees(IMU_YAW)} about z', 'note',
                'middle')
    return figure


def frames_from_the_side_figure() -> Figure:
    """Return the drawing of the two frames and their joint on Remo, from its right side."""
    up = format_centimeters(IMU_POSITION.z)
    figure = Figure('imu-frames-side', 340, 270, 'chassis_link, imu_joint and imu_link on Remo, '
                    'from the right side',
                    'Remo with the Raspberry Pi deck, from its right side, with its front on '
                    'the right. chassis_link is on '
                    'top of the base plate, with its x axis forward and its z axis up. imu_link '
                    f'is {up} cm higher, on the breadboard. Its y axis points forward and its z '
                    'axis up. chassis_link’s y axis points into the drawing, and '
                    'imu_link’s x axis out of it.')
    view = view_from_the_side(1500, Point(190, 128))
    figure.text(Point(10, 18), 'Remo with the Raspberry Pi deck, from the right side', 'title')
    figure.text(Point(318, 114), 'front', 'note', 'end')
    draw_floor(figure, view, 24, 330)
    figure.text(Point(30, 240), 'floor', 'note')
    draw_remo_from_the_side(figure, view)
    figure.text(Point(84, 121), 'breadboard', 'note', 'end')
    draw_joint(figure, view)
    draw_frame(figure, view, CHASSIS_ORIGIN, CHASSIS_AXES, 42,
               {'x': Point(5, -7), 'y': Point(12, 16)})
    draw_frame(figure, view, IMU_POSITION, IMU_AXES, 42, {'y': Point(0, -8), 'x': Point(-11, 15)})
    figure.text(Point(198, 102), 'chassis_link', 'mono halo')
    figure.text(Point(121, 88), 'imu_link', 'mono halo', 'end')
    figure.text(Point(146, 150), 'imu_joint', 'mono halo', 'middle')
    figure.text(Point(170, 262), f'imu_link lies {up} cm above chassis_link, on the breadboard',
                'note', 'middle')
    return figure


def diffbot_imu_board() -> Box:
    """Return the BNO055 board on DiffBot as a box in chassis_link's axes.

    DiffBot's placeholder mounting has no rotation, so the board's edges lie along the robot's.
    """
    length, width, height = IMU_BOARD_SIZE
    x, y, z = DIFFBOT_IMU_POSITION
    return Box(x - length / 2, x + length / 2, y - width / 2, y + width / 2,
               z - height / 2, z + height / 2)


def draw_diffbot_from_the_side(figure: Figure, view: View) -> None:
    """Add DiffBot as the view from its right side shows it, standing on the floor."""
    caster = view.project(Vector3(DIFFBOT_CASTER_X, 0.0, DIFFBOT_CASTER_Z))
    figure.circle(caster, DIFFBOT_CASTER_RADIUS * view.scale, 'wheel')
    wheel = view.project(Vector3(DIFFBOT_AXLE_X, 0.0, DIFFBOT_AXLE_Z))
    figure.circle(wheel, DIFFBOT_WHEEL_RADIUS * view.scale, 'wheel')
    figure.circle(wheel, 0.3 * DIFFBOT_WHEEL_RADIUS * view.scale, 'body')
    draw_box_outline(figure, view, DIFFBOT_PLATE, 'body', 1)
    draw_box_outline(figure, view, diffbot_imu_board(), 'board', 1)


# DiffBot's drawing shows the two frames again in a box, DIFFBOT_MAGNIFICATION times larger: at the
# robot's scale, the 1 cm between them is too short to tell them apart
DIFFBOT_SCALE = 1500  # SVG units per m
DIFFBOT_MAGNIFICATION = 4


def draw_magnified_diffbot_frames(figure: Figure, inset: InsetBounds) -> None:
    """Add a box with DiffBot's base plate, IMU board and the two frames, magnified.

    The plate and the board are bands across the box, cut off at its edges.
    """
    view = view_from_the_side(DIFFBOT_SCALE * DIFFBOT_MAGNIFICATION,
                              Point(inset.left + 65, inset.bottom - 15))
    board = diffbot_imu_board()
    figure.rectangle(Point(inset.left, inset.top), Point(inset.right, inset.bottom), 'inset', 4)
    figure.elements.append('<g class="context">')
    for box, css_class in ((DIFFBOT_PLATE, 'body'), (board, 'board')):
        band_top = view.project(Vector3(0.0, 0.0, box.z_max)).y
        band_bottom = view.project(Vector3(0.0, 0.0, box.z_min)).y
        band_right = view.project(Vector3(box.x_max, 0.0, 0.0)).x
        figure.rectangle(Point(inset.left + 1, band_top),
                         Point(min(inset.right - 1, band_right),
                               min(inset.bottom - 1, band_bottom)), css_class)
    figure.elements.append('</g>')
    draw_joint(figure, view, DIFFBOT_IMU_POSITION)
    draw_frame(figure, view, DIFFBOT_IMU_POSITION, CHASSIS_AXES, 40,
               {'x': Point(4, -8), 'y': Point(-12, -9), 'z': Point(10, 6)})
    chassis = view.project(CHASSIS_ORIGIN)
    imu = view.project(DIFFBOT_IMU_POSITION)
    figure.circle(chassis, 3, 'dot')
    up = format_centimeters(DIFFBOT_IMU_POSITION.z)
    joint_middle = (chassis.y + imu.y) / 2
    figure.text(Point(inset.left + 8, inset.top + 15), f'{DIFFBOT_MAGNIFICATION}\u00d7 larger',
                'note')
    figure.text(Point(inset.right - 8, imu.y - 26), 'imu_link', 'mono halo', 'end')
    figure.text(Point(chassis.x + 9, joint_middle - 3), 'imu_joint', 'mono halo')
    figure.text(Point(chassis.x + 9, joint_middle + 11), f'{up} cm up, not turned', 'note halo')
    figure.text(Point(inset.right - 8, chassis.y + 4), 'chassis_link', 'mono halo', 'end')


def frames_on_diffbot_figure() -> Figure:
    """Return the drawing of the two frames and their joint on DiffBot, from its right side.

    imu_link has chassis_link's directions and lies straight above it. So the drawing shows
    imu_link's axes and marks chassis_link's origin with a dot, in a magnified box.
    """
    up = format_centimeters(DIFFBOT_IMU_POSITION.z)
    figure = Figure('imu-frames-diffbot', 340, 318, 'chassis_link, imu_joint and imu_link on '
                    'DiffBot, from the right side',
                    'DiffBot from its right side, with its front on the right. chassis_link is '
                    'at the center of the base plate. imu_link lies '
                    f'{up} cm straight above it, and its axes point the same way as '
                    'chassis_link’s: x forward, z up, and y into the drawing. A dashed line, the '
                    'fixed joint imu_joint, connects the two origins. A box shows the two '
                    f'frames {DIFFBOT_MAGNIFICATION} times larger. This mounting is a '
                    'placeholder of DiffBot’s URDF model. It is not measured on the robot.')
    view = view_from_the_side(DIFFBOT_SCALE, Point(165, 196))
    figure.text(Point(10, 18), 'DiffBot’s placeholder mounting, from the right side', 'title')
    figure.text(Point(326, 184), 'front', 'note', 'end')
    draw_floor(figure, view, 4, 336, DIFFBOT_FLOOR_Z)
    figure.text(Point(60, 288), 'floor', 'note')
    figure.elements.append('<g class="context">')
    draw_diffbot_from_the_side(figure, view)
    figure.elements.append('</g>')
    # A ring around the two frames on the robot, and a line from it to the magnified box
    frames = view.project(Vector3(0.0, 0.0, DIFFBOT_IMU_POSITION.z / 2))
    figure.circle(frames, 13, 'lens')
    inset = InsetBounds(left=150.0, top=30.0, right=330.0, bottom=150.0)
    figure.polyline([Point(frames.x + 3, frames.y - 13), Point(inset.left + 24, inset.bottom)],
                    'lens')
    draw_magnified_diffbot_frames(figure, inset)
    figure.text(Point(170, 308), f'Placeholder: imu_link {up} cm above chassis_link, not turned',
                'note', 'middle')
    return figure


def rotation_arrow_points(view: View, origin: Vector3, axis: Vector3, distance: float,
                          radius: float, first_angle: float) -> List[Point]:
    """Return the points of an arrow that turns about an axis in the positive direction.

    Positive follows the right-hand rule: with the right thumb along the axis, the fingers curl
    in the arrow's direction. distance is how far along the axis the arrow's circle lies, and
    radius is the circle's radius, both in m. The arrow starts at first_angle, in degrees, and
    covers 290 degrees of the circle.
    """
    helper = UP if abs(axis.z) < 0.9 else FORWARD
    first = normalized(cross(axis, helper))
    second = cross(axis, first)
    center = offset_point(origin, axis, distance)
    points = []
    for step in range(0, 291, 10):
        angle = math.radians(first_angle + step)
        position = offset_point(offset_point(center, first, radius * math.cos(angle)), second,
                                radius * math.sin(angle))
        points.append(view.project(position))
    return points


def angular_velocity_figure() -> Figure:
    """Return the drawing of the angular velocity about each of imu_link's axes."""
    figure = Figure('imu-angular-velocity', 360, 330, 'The angular velocity about imu_link’s '
                    'three axes',
                    'Remo from behind and above. At the IMU board, imu_link’s three axes: x '
                    'in red to the robot’s right, y in green forward and z in blue up. A '
                    'curved arrow turns about each axis in the direction that counts as '
                    'positive: counterclockwise when the axis points at the viewer. The arrow '
                    'about z is the yaw rate, which is positive in a left turn: a left turn at '
                    f'{TURN_RATE} rad/s reads {format_signed_number(TURN_RATE)} rad/s. The EKF '
                    'fuses this value.')
    view = view_from_behind(1300, Point(166, 168), 30, 35)
    figure.elements.append('<g class="context">')
    draw_remo_from_behind(figure, view)
    figure.elements.append('</g>')
    figure.text(view.project(Vector3(0.086, -0.045, 0.0)), 'front', 'note', 'middle')
    length = 118 / view.scale
    start = view.project(IMU_POSITION)
    # Where each curved arrow starts on its circle, so that its head lies on the viewer's side
    first_angles = {'x': 140.0, 'y': 60.0, 'z': -170.0}
    for axis, name in zip(IMU_AXES, AXIS_NAMES):
        end = view.project(offset_point(IMU_POSITION, axis, length))
        figure.arrow(start, end, name, 9)
        offset = Point(end.x - start.x, end.y - start.y)
        reach = math.hypot(*offset)
        figure.text(Point(end.x + 10 * offset.x / reach, end.y + 10 * offset.y / reach + 4),
                    name, f'{name} halo', 'middle')
        figure.curved_arrow(rotation_arrow_points(view, IMU_POSITION, axis, 0.68 * length,
                                                  0.2 * length, first_angles[name]),
                            f'turn {name}', 8)
    figure.circle(start, 3, 'dot')
    figure.text_markup(Point(70, 28), svg_subscript('ω', 'z'), 'z', 'end')
    figure.text(Point(78, 28), f'yaw rate. Left turn at {TURN_RATE} rad/s: '
                               f'{format_signed_number(TURN_RATE)} rad/s.')
    figure.text(Point(78, 44), 'The EKF fuses this value.')
    figure.text_markup(Point(186, 150), svg_subscript('ω', 'y'), 'y halo')
    figure.text_markup(Point(199, 246), svg_subscript('ω', 'x'), 'x halo')
    figure.text(Point(180, 306), 'Each curved arrow shows the direction that counts', 'note',
                'middle')
    figure.text(Point(180, 321), 'as positive for the angular velocity about its axis', 'note',
                'middle')
    return figure


# The specific force drawings are FORCE_WIDTH wide. They draw g and f to one scale, FORCE_SCALE,
# and in their true directions. A forward acceleration of 0.5 m/s² is 2 units long at that scale,
# so a circle shows the tip of f TIP_MAGNIFICATION times larger
FORCE_WIDTH = 216
FORCE_HEIGHT = 326
GRAVITY_LENGTH = 46.0
FORCE_SCALE = GRAVITY_LENGTH / GAZEBO_GRAVITY  # SVG units per m/s²
TIP_MAGNIFICATION = 10
# The turn's drawing shows only the direction of a. At FORCE_SCALE, its arrow would be 0.1 units
# long
TURN_ACCELERATION_LENGTH = 26.0
# On the page, a drawing is at most FORCE_PANEL_WIDTH px wide, and it shares a row with another
# drawing only at that full width. So its text is as large on a wide screen as on a phone, where
# each drawing has a row of its own
FORCE_PANEL_WIDTH = 290


def draw_vector(figure: Figure, start: Point, offset: Point, css_class: str) -> Point:
    """Add an arrow from start along an offset on the drawing, and return its tip."""
    end = Point(start.x + offset.x, start.y + offset.y)
    figure.arrow(start, end, css_class, 9)
    return end


def factor_in_words(factor: int) -> str:
    """Return a whole factor for a sentence: ten as a word, another factor in digits."""
    return 'ten' if factor == 10 else str(factor)


def format_signed_number(value: float) -> str:
    """Return a value with at most two decimals and its sign, without a unit.

    A value that rounds to zero has no sign.
    """
    return '0' if abs(value) < 0.005 else f'{value:+.2f}'.rstrip('0').rstrip('.')


def specific_force(acceleration: Vector3) -> Vector3:
    """Return what the sensor reads for an acceleration: the acceleration minus gravity."""
    return offset_point(acceleration, UP, GAZEBO_GRAVITY)


def tilt_from_vertical(reading: Vector3) -> float:
    """Return the angle between a reading and the upward direction, in degrees."""
    return math.degrees(math.atan2(math.hypot(reading.x, reading.y), reading.z))


# What the three numbers of a specific force drawing are: a level floor, the named motion and no
# noise, along imu_link's axes on Remo. A recorded message differs a little
EXAMPLE_IN_A_SENTENCE = ('In this level, noise-free example, the reading along imu_link’s x, y '
                         'and z axes on Remo is')


def formatted_imu_components(reading: Vector3) -> List[str]:
    """Return an ideal reading as x, y and z on Remo, along imu_link's axes.

    These are the message's three numbers when the floor is level and the reading has no noise.
    """
    return [format_signed_number(dot(reading, axis)) for axis in IMU_AXES]


def message_values(reading: Vector3) -> str:
    """Return an ideal reading's x, y and z on Remo as a drawing shows them: (x, y, z)."""
    return '(' + ', '.join(formatted_imu_components(reading)) + ')'


def message_values_in_a_sentence(reading: Vector3) -> str:
    """Return an ideal reading's x, y and z on Remo as a sentence names them: x, y and z."""
    x, y, z = formatted_imu_components(reading)
    return f'{x}, {y} and {z}'


def draw_reading_lines(figure: Figure, first: str, second: str, reading: Vector3) -> None:
    """Add what the sensor reads at the bottom of a specific force drawing.

    first and second say it in the robot's directions. reading, in chassis_link's axes, becomes
    three numbers along imu_link's axes on Remo, labeled as a level, noise-free example.
    """
    center = FORCE_WIDTH / 2
    figure.text(Point(center, 232), 'The sensor reads', 'note', 'middle')
    figure.text(Point(center, 249), first, 'reading', 'middle')
    figure.text(Point(center, 265), second, 'reading', 'middle')
    figure.text(Point(center, 284), 'Level, noise-free example', 'note', 'middle')
    figure.text(Point(center, 299), 'in Remo’s IMU axes x, y, z', 'note', 'middle')
    figure.text(Point(center, 316), message_values(reading), 'reading', 'middle')


def draw_magnified_tip(figure: Figure, tip: Point, acceleration: float) -> None:
    """Add a circle that shows the tip of f TIP_MAGNIFICATION times larger.

    At the tip, f = a - g reads as a sum: straight up from the sensor by gravity's amount, then
    forward by the acceleration. The circle shows the end of that upward line, dotted, the arrow
    of a from there to the tip of f, and the end of f's arrow in its true direction.
    """
    center, radius = Point(160.0, 100.0), 32.0
    toward = Point(center.x - tip.x, center.y - tip.y)
    reach = math.hypot(*toward)
    figure.circle(tip, 7, 'lens')
    figure.polyline([Point(tip.x + 7 * toward.x / reach, tip.y + 7 * toward.y / reach),
                     Point(center.x - radius * toward.x / reach,
                           center.y - radius * toward.y / reach)], 'lens')
    figure.circle(center, radius, 'inset')
    figure.text(Point(center.x, center.y - radius - 6), f'{TIP_MAGNIFICATION}\u00d7 larger',
                'note', 'middle')
    forward = acceleration * FORCE_SCALE * TIP_MAGNIFICATION
    reading_tip = Point(center.x + forward / 2, center.y - 4)
    above_sensor = Point(reading_tip.x - forward, reading_tip.y)
    shown_height = 30.0
    figure.polyline([Point(above_sensor.x, above_sensor.y + shown_height), above_sensor],
                    'guide')
    figure.arrow(Point(reading_tip.x - shown_height * acceleration / GAZEBO_GRAVITY,
                       reading_tip.y + shown_height), reading_tip, 'reading', 9)
    figure.arrow(above_sensor, reading_tip, 'acceleration', 7)
    figure.text(Point(center.x, reading_tip.y - 8), f'a = {format_number(acceleration)}',
                'acceleration', 'middle')
    figure.text(Point(reading_tip.x + 6, reading_tip.y + 17), 'f', 'reading')


def specific_force_from_the_side_figure(name: str, title: str, description: str,
                                        acceleration: float) -> Figure:
    """Return a drawing of the specific force for a robot on the floor, seen from its right side.

    acceleration is the robot's forward acceleration in m/s². g and f are drawn to FORCE_SCALE.
    """
    figure = Figure(name, FORCE_WIDTH, FORCE_HEIGHT, title, description)
    view = view_from_the_side(900, Point(118, 152))
    figure.text(Point(6, 18), title, 'title')
    draw_floor(figure, view, 10, FORCE_WIDTH - 10)
    figure.text(Point(FORCE_WIDTH - 10, 146), 'front', 'note', 'end')
    figure.elements.append('<g class="context">')
    draw_remo_from_the_side(figure, view)
    figure.elements.append('</g>')
    sensor = view.project(IMU_POSITION)
    reading = specific_force(Vector3(acceleration, 0.0, 0.0))
    draw_vector(figure, sensor, Point(0, GAZEBO_GRAVITY * FORCE_SCALE), 'gravity')
    figure.text(Point(sensor.x - 8, sensor.y + GRAVITY_LENGTH / 2 + 8), f'g = {GAZEBO_GRAVITY}',
                'gravity halo', 'end')
    tip = draw_vector(figure, sensor, Point(reading.x * FORCE_SCALE, -reading.z * FORCE_SCALE),
                      'reading')
    figure.text(Point(tip.x, tip.y - 9), 'f', 'reading halo', 'middle')
    if acceleration:
        draw_magnified_tip(figure, tip, acceleration)
    else:
        figure.text(Point(sensor.x + 7, sensor.y - 5), 'a = 0', 'acceleration halo')
    figure.circle(sensor, 3, 'dot')
    return figure


def specific_force_at_rest_figure() -> Figure:
    """Return the drawing of the specific force while the robot stands still."""
    reading = specific_force(Vector3(0.0, 0.0, 0.0))
    figure = specific_force_from_the_side_figure(
        'imu-force-rest', 'At rest',
        'Remo stands still on the floor, seen from its right side. At the IMU, gravity g points '
        f'down with {GAZEBO_GRAVITY} m/s². The acceleration a is zero. The sensor reads '
        f'f = a − g, which points up with {GAZEBO_GRAVITY} m/s². {EXAMPLE_IN_A_SENTENCE} '
        f'{message_values_in_a_sentence(reading)}.', 0.0)
    draw_reading_lines(figure, 'forward 0,', f'up +{GAZEBO_GRAVITY} m/s²', reading)
    return figure


def specific_force_speeding_up_figure() -> Figure:
    """Return the drawing of the specific force while the robot speeds up forward."""
    reading = specific_force(Vector3(FORWARD_ACCELERATION, 0.0, 0.0))
    forward = format_number(FORWARD_ACCELERATION)
    figure = specific_force_from_the_side_figure(
        'imu-force-speeding-up', 'Speeding up forward',
        f'Remo speeds up forward with {forward} m/s², seen from its right side. At the IMU, '
        f'gravity g points down with {GAZEBO_GRAVITY} m/s², and the acceleration a points '
        f'forward with {forward} m/s². The sensor reads f = a − g: {forward} m/s² forward and '
        f'{GAZEBO_GRAVITY} m/s² up, so f leans forward by {tilt_from_vertical(reading):.0f} '
        f'degrees. A circle shows the tip of f {factor_in_words(TIP_MAGNIFICATION)} times '
        f'larger: f ends {forward} m/s² in front of the point straight above the sensor. '
        f'{EXAMPLE_IN_A_SENTENCE} {message_values_in_a_sentence(reading)}.',
        FORWARD_ACCELERATION)
    figure.arrow(Point(132, 46), Point(178, 46), 'motion thin', 7)
    figure.text(Point(155, 39), 'speeds up', 'note', 'middle')
    draw_reading_lines(figure, f'forward {format_signed_number(FORWARD_ACCELERATION)},',
                       f'up +{GAZEBO_GRAVITY} m/s²', reading)
    return figure


def turn_radius() -> float:
    """Return the IMU's distance from the wheel axle's midpoint, which Remo turns about, in m."""
    return math.hypot(IMU_POSITION.x - AXLE_X, IMU_POSITION.y)


def circle_points(center: Point, radius: float, first_angle: int, last_angle: int) -> List[Point]:
    """Return points on a circle on the drawing, counterclockwise between two angles in degrees."""
    return [Point(center.x + radius * math.cos(math.radians(angle)),
                  center.y - radius * math.sin(math.radians(angle)))
            for angle in range(first_angle, last_angle + 1, 6)]


def turn_acceleration() -> Vector3:
    """Return the IMU's acceleration in a turn on the spot: toward the wheel axle's midpoint."""
    return Vector3(TURN_RATE ** 2 * (AXLE_X - IMU_POSITION.x), TURN_RATE ** 2 * -IMU_POSITION.y,
                   0.0)


def centripetal_acceleration() -> float:
    """Return the size of the IMU's acceleration in a turn on the spot, in m/s²."""
    return TURN_RATE ** 2 * turn_radius()


def specific_force_in_a_turn_figure() -> Figure:
    """Return the drawing of the specific force while the robot turns left on the spot."""
    centripetal = f'{centripetal_acceleration():.2f}'
    reading = specific_force(turn_acceleration())
    figure = Figure('imu-force-turn', FORCE_WIDTH, FORCE_HEIGHT, 'Turning left, from above',
                    'Remo turns left on the spot, seen from above. The IMU moves on a circle '
                    f'around the midpoint of the wheel axle, {100 * turn_radius():.0f} cm '
                    f'away. Its acceleration a points to that midpoint. At {TURN_RATE} rad/s, a '
                    f'is {centripetal} m/s², tiny next to gravity. Its arrow shows only its '
                    f'direction. The sensor reads this {centripetal} m/s² toward the center '
                    f'and {GAZEBO_GRAVITY} m/s² up, out of the drawing, so f leans toward the '
                    f'center by {tilt_from_vertical(reading):.1f} degrees. '
                    f'{EXAMPLE_IN_A_SENTENCE} {message_values_in_a_sentence(reading)}.')
    view = view_from_above(900, Point(FORCE_WIDTH / 2, 142))
    figure.text(Point(6, 18), 'Turning left, from above', 'title')
    figure.elements.append('<g class="context">')
    draw_remo_from_above(figure, view)
    figure.elements.append('</g>')
    center = view.project(Vector3(AXLE_X, 0.0, 0.0))
    sensor = view.project(IMU_POSITION)
    figure.polyline(circle_points(center, turn_radius() * view.scale, 0, 360), 'guide')
    figure.curved_arrow(circle_points(center, 13, -60, 222), 'motion thin', 7)
    figure.circle(center, 2.5, 'dot')
    figure.text(Point(center.x, center.y - 20), f'{TURN_RATE} rad/s', 'note', 'middle')
    toward = Point(center.x - sensor.x, center.y - sensor.y)
    reach = math.hypot(*toward)
    tip = draw_vector(figure, sensor, Point(TURN_ACCELERATION_LENGTH * toward.x / reach,
                                            TURN_ACCELERATION_LENGTH * toward.y / reach),
                      'acceleration')
    figure.text(Point(tip.x + 7, tip.y + 8), f'a = {centripetal}', 'acceleration halo')
    figure.circle(sensor, 5.5, 'ring reading')
    figure.circle(sensor, 2.0, 'head reading')
    figure.text(Point(sensor.x - 10, sensor.y + 4), 'f', 'reading halo', 'end')
    draw_reading_lines(figure, f'toward the center {centripetal},', f'up +{GAZEBO_GRAVITY} m/s²',
                       reading)
    return figure


class Panel(NamedTuple):
    """One panel of a figure: a drawing and the text below it, as HTML. None is no text."""

    drawing: Figure
    explanation_html: Optional[str] = None


def panels_html(panels: Sequence[Panel]) -> str:
    """Return the panels' markup: each drawing in its own div, with its text below it."""
    divisions = []
    for panel in panels:
        explanation = ('' if panel.explanation_html is None
                       else f'\n<p>{panel.explanation_html}</p>')
        divisions.append(f'<div class="imu-panel">\n{panel.drawing.svg()}{explanation}\n</div>')
    return '\n'.join(divisions)


def write_include(path: str, html: str) -> None:
    """Write an include: the comment that names this script, the drawings' style and html."""
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{INCLUDE_COMMENT}\n{style_element(RULES)}\n{html}\n')


def frame_tabs() -> List[Tab]:
    """Return one tab per robot, each with its label, its drawings and its caption.

    The tab set is raw HTML, so the captions are HTML.
    """
    up = format_centimeters(DIFFBOT_IMU_POSITION.z)
    remo = ('Remo’s frames, to scale. <code>chassis_link</code> is at the center of the base '
            'plate’s top. <code>imu_joint</code>, the dashed line, is a fixed joint. It puts '
            '<code>imu_link</code> on <code>chassis_link</code>, at the IMU’s board (blue) on '
            'the breadboard. Red is x, green is y and blue is z. A ring with a dot is an axis '
            'that points at you, a ring with a cross one that points away.')
    diffbot = ('DiffBot’s frames in its URDF model, to scale. <code>imu_joint</code>, the '
               f'dashed line, puts <code>imu_link</code> {up} cm straight above '
               '<code>chassis_link</code>, the dot, without a rotation. So '
               '<code>imu_link</code>’s axes point the same way as <code>chassis_link</code>’s: '
               'x forward (red), z up (blue), and y to the left, away from you (the ring with '
               'a cross). The mounting is a placeholder. It isn’t measured on the robot.')
    widths = PanelWidths(basis=300, widest=400)
    remo_panels = [Panel(frames_from_above_figure()), Panel(frames_from_the_side_figure())]
    diffbot_panels = [Panel(frames_on_diffbot_figure())]
    return [
        Tab(REMO_TAB_LABEL, figure_element('imu', panels_html(remo_panels), remo, widths,
                                           caption_is_markdown=False)),
        Tab(DIFFBOT_TAB_LABEL, figure_element('imu', panels_html(diffbot_panels), diffbot,
                                              widths, caption_is_markdown=False))]


def write_frames() -> None:
    """Write the include with the frames on Remo and on DiffBot, one tab per robot."""
    write_include('includes/imu-sim-frames.html', tab_set('imuframes', 'imu-tabs', frame_tabs()))


def write_angular_velocity() -> None:
    """Write the include with the angular velocity about imu_link's axes."""
    caption = ('`angular_velocity` on Remo: one number per axis of `imu_link`, in rad/s. Red is '
               'x, green is y and blue is z. Point your right thumb along an axis: your fingers '
               'curl in the positive direction. On Remo, `imu_link`’s y axis points forward and '
               'its x axis to the robot’s right, because the board is mounted turned by '
               f'{format_degrees(IMU_YAW)} about z, as the [figure of Remo’s frames]'
               '(position-and-axes.md) shows.')
    write_include('includes/imu-sim-angular-velocity.html', figure_element(
        'imu', panels_html([Panel(angular_velocity_figure())]), caption,
        PanelWidths(basis=300, widest=480)))


def specific_force_panels() -> List[Panel]:
    """Return the three specific force drawings, each with the text below it, as HTML."""
    forward = format_number(FORWARD_ACCELERATION)
    forward_tilt = tilt_from_vertical(specific_force(Vector3(FORWARD_ACCELERATION, 0.0, 0.0)))
    turn_tilt = tilt_from_vertical(specific_force(turn_acceleration()))
    a, f, g = '<strong>a</strong>', '<strong>f</strong>', '<strong>g</strong>'
    return [
        Panel(specific_force_at_rest_figure(),
              f'{a} is zero, so the sensor reads {f} = \u2212{g}: {GAZEBO_GRAVITY} m/s² straight '
              f'up. {g} and {f} are drawn to one scale.'),
        Panel(specific_force_speeding_up_figure(),
              f'{f} leans forward by only {forward_tilt:.0f}°. So the circle shows the tip of '
              f'{f} {TIP_MAGNIFICATION} times larger: {f} ends {forward} m/s² in front of the '
              'dotted line, which points straight up from the sensor.'),
        Panel(specific_force_in_a_turn_figure(),
              f'The dotted circle is the sensor’s path. {f} points up, at you, and leans toward '
              f'the center by {turn_tilt:.1f}°. The arrow of {a} shows only its direction: at '
              f'the scale of {g}, its {centripetal_acceleration():.2f} m/s² would be too short '
              'to see.'),
    ]


def write_specific_force() -> None:
    """Write the include with the specific force in three situations."""
    caption = ('`linear_acceleration` is the specific force **f** = **a** − **g**: the '
               'acceleration **a** of the sensor minus gravity **g**, in m/s². The three '
               'drawings are a level, noise-free example: the floor is level, the robot moves '
               'exactly as named, and the readings have no noise. The last line of each drawing '
               'gives the reading along `imu_link`’s x, y and z axes on Remo, where the y axis '
               'points forward. A message of the simulation differs a little, as the text '
               'below the figure explains.')
    write_include('includes/imu-sim-specific-force.html', figure_element(
        'imu', panels_html(specific_force_panels()), caption,
        PanelWidths(basis=FORCE_PANEL_WIDTH, widest=FORCE_PANEL_WIDTH)))


def main() -> None:
    """Write the three includes."""
    write_frames()
    write_angular_velocity()
    write_specific_force()


if __name__ == '__main__':
    main()
