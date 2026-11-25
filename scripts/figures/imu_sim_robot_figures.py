"""Generate the figures of the IMU Simulation pages that show a robot.

It writes three includes, as inline SVG that follows the site's light and dark colors:

- includes/imu-sim-frames.html, for docs/packages/imu/simulation/position-and-axes.md:
  chassis_link, imu_joint and imu_link, in one tab per robot. Remo from above and from the
  side, and DiffBot from the side, with the placeholder mounting of DiffBot's URDF model. The
  tabs are written as HTML, in the markup that pymdownx.tabbed produces with alternate_style,
  as scripts/figures/simulation_test_figures.py does. Their labels (ROBOT_TAB_LABELS) are those
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

import math
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple

from board_drawings import escape
from board_drawings import style_element

# The first line of each include: the script that writes it
INCLUDE_COMMENT = '<!-- Written by scripts/figures/imu_sim_robot_figures.py -->'

# A point on the drawing, in SVG units
Point = Tuple[float, float]
# A position in m or a direction without a unit, in chassis_link's axes: x forward, y left, z up
Vector = Tuple[float, float, float]


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
    origin: Vector
    direction: Vector


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
IMU_POSITION: Vector = (-0.0401, 0.0234, 0.013)
IMU_YAW = -math.pi / 2
# The BNO055 board along imu_link's x, y and z axes: diffbot's config/diffbot/sensors.yaml
IMU_BOARD_SIZE: Vector = (0.027, 0.020, 0.004)

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
DIFFBOT_IMU_POSITION: Vector = (0.0, 0.0, 0.01)

CHASSIS_AXES: Tuple[Vector, Vector, Vector] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
IMU_AXES: Tuple[Vector, Vector, Vector] = (
    (math.cos(IMU_YAW), math.sin(IMU_YAW), 0.0),
    (-math.sin(IMU_YAW), math.cos(IMU_YAW), 0.0),
    (0.0, 0.0, 1.0))
AXIS_NAMES = ('x', 'y', 'z')
# The labels of the frames' tabs, the same as those of the Markdown tabs on the page
ROBOT_TAB_LABELS = ('Remo', 'DiffBot')
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


def dot(first: Vector, second: Vector) -> float:
    """Return the dot product of two vectors."""
    return sum(a * b for a, b in zip(first, second))


def cross(first: Vector, second: Vector) -> Vector:
    """Return the cross product of two vectors."""
    return (first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0])


def add(start: Vector, direction: Vector, length: float = 1.0) -> Vector:
    """Return the position that lies length times direction away from start."""
    return (start[0] + length * direction[0], start[1] + length * direction[1],
            start[2] + length * direction[2])


def normalized(vector: Vector) -> Vector:
    """Return the vector with length 1."""
    length = math.sqrt(dot(vector, vector))
    return (vector[0] / length, vector[1] / length, vector[2] / length)


class View(NamedTuple):
    """A parallel projection of chassis_link's axes onto the drawing.

    right and up are the directions that point to the right and upward on the drawing. scale is
    in SVG units per m, and origin is where chassis_link's origin lies on the drawing.
    """

    right: Vector
    up: Vector
    scale: float
    origin: Point

    def project(self, position: Vector) -> Point:
        """Return where a position in m lies on the drawing."""
        return (self.origin[0] + self.scale * dot(position, self.right),
                self.origin[1] - self.scale * dot(position, self.up))

    def on_drawing(self, direction: Vector, length: float) -> Point:
        """Return a direction as an offset on the drawing, for an arrow of length SVG units."""
        return (length * dot(direction, self.right), -length * dot(direction, self.up))

    def toward_viewer(self, direction: Vector) -> float:
        """Return how much of a direction points out of the drawing, toward the viewer."""
        return dot(direction, cross(self.right, self.up))


def view_from_above(scale: float, origin: Point) -> View:
    """Return the view from above, with the robot's front at the top of the drawing."""
    return View((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), scale, origin)


def view_from_the_side(scale: float, origin: Point) -> View:
    """Return the view from the robot's right side, with the robot's front on the right."""
    return View((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), scale, origin)


def view_from_behind(scale: float, origin: Point, azimuth: float, elevation: float) -> View:
    """Return a view from behind and above the robot.

    azimuth turns the viewer to the robot's right, and elevation lifts the viewer, both in
    degrees.
    """
    turn, lift = math.radians(azimuth), math.radians(elevation)
    right = (math.sin(turn), -math.cos(turn), 0.0)
    up = (math.sin(lift) * math.cos(turn), math.sin(lift) * math.sin(turn), math.cos(lift))
    return View(right, up, scale, origin)


class Figure:
    """Collect the SVG elements of one drawing."""

    def __init__(self, name: str, width: float, height: float, title: str,
                 description: str) -> None:
        self.name, self.width, self.height = name, width, height
        self.title, self.description = title, description
        self.elements: List[str] = []

    def polygon(self, points: Sequence[Point], css_class: str) -> None:
        """Add a closed shape through the points."""
        corners = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.elements.append(f'<polygon class="{css_class}" points="{corners}"/>')

    def rectangle(self, first: Point, second: Point, css_class: str, radius: float = 0) -> None:
        """Add a rectangle between two opposite corners."""
        x, y = min(first[0], second[0]), min(first[1], second[1])
        width, height = abs(first[0] - second[0]), abs(first[1] - second[1])
        self.elements.append(f'<rect class="{css_class}" x="{x:.1f}" y="{y:.1f}" '
                             f'width="{width:.1f}" height="{height:.1f}" rx="{radius}"/>')

    def circle(self, center: Point, radius: float, css_class: str) -> None:
        """Add a circle around center."""
        self.elements.append(f'<circle class="{css_class}" cx="{center[0]:.1f}" '
                             f'cy="{center[1]:.1f}" r="{radius:.1f}"/>')

    def polyline(self, points: Sequence[Point], css_class: str) -> None:
        """Add an open line through the points."""
        path = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.elements.append(f'<polyline class="{css_class}" points="{path}"/>')

    def head(self, tip: Point, direction: Point, css_class: str, size: float = 8.0) -> None:
        """Add an arrowhead whose tip points along direction."""
        length = math.hypot(*direction)
        along = (direction[0] / length, direction[1] / length)
        across = (-along[1], along[0])
        base = (tip[0] - size * along[0], tip[1] - size * along[1])
        half = 0.42 * size
        self.polygon([tip, (base[0] + half * across[0], base[1] + half * across[1]),
                      (base[0] - half * across[0], base[1] - half * across[1])],
                     f'head {css_class}')

    def arrow(self, start: Point, end: Point, css_class: str, size: float = 8.0) -> None:
        """Add a straight arrow from start to end."""
        direction = (end[0] - start[0], end[1] - start[1])
        length = math.hypot(*direction)
        shaft_end = (end[0] - 0.8 * size * direction[0] / length,
                     end[1] - 0.8 * size * direction[1] / length)
        self.polyline([start, shaft_end], f'shaft {css_class}')
        self.head(end, direction, css_class, size)

    def curved_arrow(self, points: Sequence[Point], css_class: str, size: float = 7.0) -> None:
        """Add an arrow along the points, with its head at the last one."""
        self.polyline(points[:-1], f'shaft {css_class}')
        direction = (points[-1][0] - points[-3][0], points[-1][1] - points[-3][1])
        self.head(points[-1], direction, css_class, size)

    def text(self, position: Point, text: str, css_class: str = '', anchor: str = 'start',
             markup: bool = False) -> None:
        """Add one line of text. anchor is start, middle or end. markup keeps SVG tags."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        content = text if markup else escape(text)
        self.elements.append(f'<text{class_attribute} x="{position[0]:.1f}" '
                             f'y="{position[1]:.1f}" text-anchor="{anchor}">{content}</text>')

    def svg(self) -> str:
        """Return the drawing as an SVG element."""
        return (f'<svg class="imu-fig" viewBox="0 0 {self.width} {self.height}" role="img" '
                f'aria-labelledby="{self.name}-title {self.name}-description">'
                f'<title id="{self.name}-title">{escape(self.title)}</title>'
                f'<desc id="{self.name}-description">{escape(self.description)}</desc>\n'
                + '\n'.join(self.elements) + '\n</svg>')


def subscript(symbol: str, index: str) -> str:
    """Return SVG markup for a symbol with a subscript, for the end of a text line."""
    return f'{symbol}<tspan class="sub" dy="3">{index}</tspan>'


def box_corners(box: Box) -> List[Vector]:
    """Return the eight corners of a box."""
    return [(x, y, z) for x in (box.x_min, box.x_max) for y in (box.y_min, box.y_max)
            for z in (box.z_min, box.z_max)]


def imu_board() -> Box:
    """Return the BNO055 board as a box in chassis_link's axes.

    imu_link is turned by -90 degrees about z, so the board's x edge lies along the robot's y axis.
    """
    length, width, height = IMU_BOARD_SIZE
    x, y, z = IMU_POSITION
    return Box(x - width / 2, x + width / 2, y - length / 2, y + length / 2,
               z - height / 2, z + height / 2)


def outline(figure: Figure, view: View, box: Box, css_class: str, radius: float = 0) -> None:
    """Add a box as the rectangle that a view along one of the axes shows."""
    points = [view.project(corner) for corner in box_corners(box)]
    figure.rectangle((min(x for x, _ in points), min(y for _, y in points)),
                     (max(x for x, _ in points), max(y for _, y in points)), css_class, radius)


def wheel_center(side: int) -> Vector:
    """Return the center of the left (side 1) or the right (side -1) wheel."""
    return (AXLE_X, side * WHEEL_SEPARATION / 2, AXLE_Z)


def draw_remo_from_above(figure: Figure, view: View) -> None:
    """Add Remo as the view from above shows it: wheels, deck, breadboard and IMU board."""
    for side in (1, -1):
        x, y, z = wheel_center(side)
        outline(figure, view, Box(x - WHEEL_RADIUS, x + WHEEL_RADIUS, y - WHEEL_WIDTH / 2,
                                  y + WHEEL_WIDTH / 2, z, z), 'wheel', 4)
    outline(figure, view, DECK, 'body', 5)
    outline(figure, view, BREADBOARD, 'part', 2)
    outline(figure, view, imu_board(), 'board', 1.5)


def draw_remo_from_the_side(figure: Figure, view: View) -> None:
    """Add Remo as the view from its right side shows it, standing on the floor."""
    caster = view.project((CASTER_X, 0.0, FLOOR_Z + CASTER_RADIUS))
    figure.circle(caster, CASTER_RADIUS * view.scale, 'wheel')
    outline(figure, view, PLATE, 'body', 2)
    outline(figure, view, DECK, 'body')
    outline(figure, view, BREADBOARD, 'part', 1.5)
    outline(figure, view, imu_board(), 'board', 1)
    figure.circle(view.project(wheel_center(-1)), WHEEL_RADIUS * view.scale, 'wheel')
    figure.circle(view.project(wheel_center(-1)), HUB_RADIUS * view.scale, 'body')


def draw_floor(figure: Figure, view: View, left: float, right: float,
               floor_z: float = FLOOR_Z) -> None:
    """Add the floor as a line under the wheels, between two x positions on the drawing.

    floor_z is the floor's height in chassis_link's axes, in m. The default is Remo's.
    """
    height = view.project((0.0, 0.0, floor_z))[1]
    figure.polyline([(left, height), (right, height)], 'floor')


def visible_faces(view: View, box: Box) -> List[List[Vector]]:
    """Return the corners of each face of a box that faces the viewer."""
    low = (box.x_min, box.y_min, box.z_min)
    high = (box.x_max, box.y_max, box.z_max)
    faces = []
    for axis in range(3):
        first, second = [other for other in range(3) if other != axis]
        for value, outward in ((low[axis], -1.0), (high[axis], 1.0)):
            normal = [0.0, 0.0, 0.0]
            normal[axis] = outward
            if view.toward_viewer((normal[0], normal[1], normal[2])) <= 0:
                continue
            corners = []
            for a, b in ((low[first], low[second]), (high[first], low[second]),
                         (high[first], high[second]), (low[first], high[second])):
                corner = [0.0, 0.0, 0.0]
                corner[axis], corner[first], corner[second] = value, a, b
                corners.append((corner[0], corner[1], corner[2]))
            faces.append(corners)
    return faces


def draw_box(figure: Figure, view: View, box: Box, css_class: str) -> None:
    """Add the faces of a box that the viewer sees."""
    for face in visible_faces(view, box):
        figure.polygon([view.project(corner) for corner in face], css_class)


def convex_hull(points: Sequence[Point]) -> List[Point]:
    """Return the corners of the smallest convex shape around the points."""
    ordered = sorted(set(points))

    def turns_left(a: Point, b: Point, c: Point) -> bool:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0

    def half(sequence: Sequence[Point]) -> List[Point]:
        hull: List[Point] = []
        for point in sequence:
            while len(hull) >= 2 and not turns_left(hull[-2], hull[-1], point):
                hull.pop()
            hull.append(point)
        return hull[:-1]

    return half(ordered) + half(ordered[::-1])


def wheel_rim(view: View, side: int, y_offset: float, radius: float) -> List[Point]:
    """Return a circle around a wheel's axle on the drawing, y_offset from the wheel's center."""
    x, y, z = wheel_center(side)
    return [view.project((x + radius * math.cos(math.radians(angle)), y + y_offset,
                          z + radius * math.sin(math.radians(angle))))
            for angle in range(0, 360, 10)]


def draw_wheel(figure: Figure, view: View, side: int) -> None:
    """Add a wheel as a cylinder, with the face that the viewer sees."""
    near = -WHEEL_WIDTH / 2 if view.toward_viewer((0.0, -1.0, 0.0)) > 0 else WHEEL_WIDTH / 2
    rims = wheel_rim(view, side, near, WHEEL_RADIUS) + wheel_rim(view, side, -near, WHEEL_RADIUS)
    figure.polygon(convex_hull(rims), 'wheel')
    figure.polygon(wheel_rim(view, side, near, WHEEL_RADIUS), 'wheel')
    figure.polygon(wheel_rim(view, side, near, HUB_RADIUS), 'body')


def draw_remo_from_behind(figure: Figure, view: View) -> None:
    """Add Remo as a view from behind and above shows it, the far parts first."""
    draw_wheel(figure, view, 1)
    caster = view.project((CASTER_X, 0.0, FLOOR_Z + CASTER_RADIUS))
    figure.circle(caster, CASTER_RADIUS * view.scale, 'wheel')
    draw_box(figure, view, PLATE, 'body')
    draw_box(figure, view, DECK, 'body')
    draw_box(figure, view, BREADBOARD, 'part')
    draw_box(figure, view, imu_board(), 'board')
    draw_wheel(figure, view, -1)


def points_along_the_line_of_sight(view: View, direction: Vector) -> bool:
    """Return whether a direction points at the viewer or away from the viewer."""
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
            figure.polyline([(center[0] - 3.2, center[1] - 3.2 * sign),
                             (center[0] + 3.2, center[1] + 3.2 * sign)],
                            f'shaft thin {axis.name}')
    if label_offset:
        figure.text((center[0] + label_offset[0], center[1] + label_offset[1]), axis.name,
                    f'{axis.name} halo', 'middle')


def draw_axis_arrow(figure: Figure, view: View, axis: Axis, length: float,
                    label_offset: Optional[Point]) -> None:
    """Add an axis as an arrow of length SVG units from the frame's origin, with its letter.

    label_offset is where the letter goes, measured from the arrow's tip. Without it, the letter
    goes beyond the tip.
    """
    start = view.project(axis.origin)
    offset = view.on_drawing(axis.direction, length)
    end = (start[0] + offset[0], start[1] + offset[1])
    figure.arrow(start, end, axis.name)
    reach = math.hypot(*offset)
    beyond = (9 * offset[0] / reach, 9 * offset[1] / reach + 4)
    label = label_offset or beyond
    figure.text((end[0] + label[0], end[1] + label[1]), axis.name, f'{axis.name} halo', 'middle')


def draw_frame(figure: Figure, view: View, origin: Vector, axes: Tuple[Vector, Vector, Vector],
               length: float, label_offsets: Optional[Dict[str, Point]] = None) -> None:
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


def draw_joint(figure: Figure, view: View, imu_position: Optional[Vector] = None) -> None:
    """Add imu_joint as a dashed line from chassis_link's origin to imu_link's origin.

    imu_position is imu_link's origin in chassis_link's axes, in m. The default is Remo's.
    """
    end = IMU_POSITION if imu_position is None else imu_position
    figure.polyline([view.project((0.0, 0.0, 0.0)), view.project(end)], 'joint')


def number(value: float) -> str:
    """Return a value with at most two decimals, without a plus sign."""
    return f'{value:.2f}'.rstrip('0').rstrip('.')


def centimeters(meters: float) -> str:
    """Return a length in cm with at most two decimals, without the unit."""
    return number(100 * meters)


def degrees(radians: float) -> str:
    """Return an angle in whole degrees with the degree sign, and a minus sign below zero."""
    return f'{math.degrees(radians):.0f}°'.replace('-', '\u2212')


def frames_from_above_figure() -> Figure:
    """Return the drawing of the two frames and their joint on Remo, from above."""
    x, y, z = IMU_POSITION
    back, left, up = centimeters(-x), centimeters(y), centimeters(z)
    figure = Figure('imu-frames-above', 340, 306, 'chassis_link, imu_joint and imu_link on Remo, '
                    'from above',
                    'Remo with the Raspberry Pi deck, from above, with its front at the top. '
                    'chassis_link is at the center '
                    'of the base plate, with its x axis forward and its y axis to the left. '
                    f'imu_link is at the IMU board on the breadboard, {back} cm behind and '
                    f'{left} cm left of chassis_link. Its x axis points to the right and its y '
                    'axis forward. The z axes of both point up, out of the drawing. A dashed '
                    'line, the fixed joint imu_joint, connects the two origins.')
    view = view_from_above(1500, (170, 152))
    figure.text((10, 18), 'Remo with the Raspberry Pi deck, from above', 'title')
    figure.text((170, 38), 'front', 'note', 'middle')
    draw_remo_from_above(figure, view)
    draw_joint(figure, view)
    draw_frame(figure, view, (0.0, 0.0, 0.0), CHASSIS_AXES, 42, {'y': (0, -8), 'z': (11, -7)})
    draw_frame(figure, view, IMU_POSITION, IMU_AXES, 42, {'y': (-11, 12), 'z': (0, 28)})
    figure.text((184, 164), 'chassis_link', 'mono halo')
    figure.text((98, 216), 'imu_link', 'mono halo', 'end')
    figure.text((162, 191), 'imu_joint', 'mono halo')
    figure.text((162, 204), 'a fixed joint', 'note halo')
    figure.text((226, 248), 'breadboard', 'note', 'end')
    figure.text((170, 279), f'imu_joint puts imu_link {back} cm back, {left} cm left', 'note',
                'middle')
    figure.text((170, 295), f'and {up} cm up, and turns it by {degrees(IMU_YAW)} about z',
                'note', 'middle')
    return figure


def frames_from_the_side_figure() -> Figure:
    """Return the drawing of the two frames and their joint on Remo, from its right side."""
    up = centimeters(IMU_POSITION[2])
    figure = Figure('imu-frames-side', 340, 270, 'chassis_link, imu_joint and imu_link on Remo, '
                    'from the right side',
                    'Remo with the Raspberry Pi deck, from its right side, with its front on '
                    'the right. chassis_link is on '
                    'top of the base plate, with its x axis forward and its z axis up. imu_link '
                    f'is {up} cm higher, on the breadboard. Its y axis points forward and its z '
                    'axis up. chassis_link’s y axis points into the drawing, and '
                    'imu_link’s x axis out of it.')
    view = view_from_the_side(1500, (190, 128))
    figure.text((10, 18), 'Remo with the Raspberry Pi deck, from the right side', 'title')
    figure.text((318, 114), 'front', 'note', 'end')
    draw_floor(figure, view, 24, 330)
    figure.text((30, 240), 'floor', 'note')
    draw_remo_from_the_side(figure, view)
    figure.text((84, 121), 'breadboard', 'note', 'end')
    draw_joint(figure, view)
    draw_frame(figure, view, (0.0, 0.0, 0.0), CHASSIS_AXES, 42, {'x': (5, -7), 'y': (12, 16)})
    draw_frame(figure, view, IMU_POSITION, IMU_AXES, 42, {'y': (0, -8), 'x': (-11, 15)})
    figure.text((198, 102), 'chassis_link', 'mono halo')
    figure.text((121, 88), 'imu_link', 'mono halo', 'end')
    figure.text((146, 150), 'imu_joint', 'mono halo', 'middle')
    figure.text((170, 262), f'imu_link lies {up} cm above chassis_link, on the breadboard',
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
    caster = view.project((DIFFBOT_CASTER_X, 0.0, DIFFBOT_CASTER_Z))
    figure.circle(caster, DIFFBOT_CASTER_RADIUS * view.scale, 'wheel')
    wheel = view.project((DIFFBOT_AXLE_X, 0.0, DIFFBOT_AXLE_Z))
    figure.circle(wheel, DIFFBOT_WHEEL_RADIUS * view.scale, 'wheel')
    figure.circle(wheel, 0.3 * DIFFBOT_WHEEL_RADIUS * view.scale, 'body')
    outline(figure, view, DIFFBOT_PLATE, 'body', 1)
    outline(figure, view, diffbot_imu_board(), 'board', 1)


# DiffBot's drawing shows the two frames again in a box, DIFFBOT_MAGNIFICATION times larger: at the
# robot's scale, the 1 cm between them is too short to tell them apart
DIFFBOT_SCALE = 1500  # SVG units per m
DIFFBOT_MAGNIFICATION = 4


def draw_magnified_diffbot_frames(figure: Figure, left: float, top: float, right: float,
                                  bottom: float) -> None:
    """Add a box with DiffBot's base plate, IMU board and the two frames, magnified.

    The box lies between left, top, right and bottom on the drawing. The plate and the board
    are bands across the box, cut off at its edges.
    """
    view = view_from_the_side(DIFFBOT_SCALE * DIFFBOT_MAGNIFICATION, (left + 65, bottom - 15))
    board = diffbot_imu_board()
    figure.rectangle((left, top), (right, bottom), 'inset', 4)
    figure.elements.append('<g class="context">')
    for box, css_class in ((DIFFBOT_PLATE, 'body'), (board, 'board')):
        figure.rectangle((left + 1, view.project((0.0, 0.0, box.z_max))[1]),
                         (min(right - 1, view.project((box.x_max, 0.0, 0.0))[0]),
                          min(bottom - 1, view.project((0.0, 0.0, box.z_min))[1])), css_class)
    figure.elements.append('</g>')
    draw_joint(figure, view, DIFFBOT_IMU_POSITION)
    draw_frame(figure, view, DIFFBOT_IMU_POSITION, CHASSIS_AXES, 40,
               {'x': (4, -8), 'y': (-12, -9), 'z': (10, 6)})
    chassis = view.project((0.0, 0.0, 0.0))
    imu = view.project(DIFFBOT_IMU_POSITION)
    figure.circle(chassis, 3, 'dot')
    up = centimeters(DIFFBOT_IMU_POSITION[2])
    figure.text((left + 8, top + 15), f'{DIFFBOT_MAGNIFICATION}\u00d7 larger', 'note')
    figure.text((right - 8, imu[1] - 26), 'imu_link', 'mono halo', 'end')
    figure.text((chassis[0] + 9, (chassis[1] + imu[1]) / 2 - 3), 'imu_joint', 'mono halo')
    figure.text((chassis[0] + 9, (chassis[1] + imu[1]) / 2 + 11), f'{up} cm up, not turned',
                'note halo')
    figure.text((right - 8, chassis[1] + 4), 'chassis_link', 'mono halo', 'end')


def frames_on_diffbot_figure() -> Figure:
    """Return the drawing of the two frames and their joint on DiffBot, from its right side.

    imu_link has chassis_link's directions and lies straight above it. So the drawing shows
    imu_link's axes and marks chassis_link's origin with a dot, in a magnified box.
    """
    up = centimeters(DIFFBOT_IMU_POSITION[2])
    figure = Figure('imu-frames-diffbot', 340, 318, 'chassis_link, imu_joint and imu_link on '
                    'DiffBot, from the right side',
                    'DiffBot from its right side, with its front on the right. chassis_link is '
                    'at the center of the base plate. imu_link lies '
                    f'{up} cm straight above it, and its axes point the same way as '
                    'chassis_link’s: x forward, z up, and y into the drawing. A dashed line, the '
                    'fixed joint imu_joint, connects the two origins. A box shows the two '
                    f'frames {DIFFBOT_MAGNIFICATION} times larger. This mounting is a '
                    'placeholder of DiffBot’s URDF model. It is not measured on the robot.')
    view = view_from_the_side(DIFFBOT_SCALE, (165, 196))
    figure.text((10, 18), 'DiffBot’s placeholder mounting, from the right side', 'title')
    figure.text((326, 184), 'front', 'note', 'end')
    draw_floor(figure, view, 4, 336, DIFFBOT_FLOOR_Z)
    figure.text((60, 288), 'floor', 'note')
    figure.elements.append('<g class="context">')
    draw_diffbot_from_the_side(figure, view)
    figure.elements.append('</g>')
    # A ring around the two frames on the robot, and a line from it to the magnified box
    frames = view.project((0.0, 0.0, DIFFBOT_IMU_POSITION[2] / 2))
    figure.circle(frames, 13, 'lens')
    box_left, box_top, box_right, box_bottom = 150.0, 30.0, 330.0, 150.0
    figure.polyline([(frames[0] + 3, frames[1] - 13), (box_left + 24, box_bottom)], 'lens')
    draw_magnified_diffbot_frames(figure, box_left, box_top, box_right, box_bottom)
    figure.text((170, 308), f'Placeholder: imu_link {up} cm above chassis_link, not turned',
                'note', 'middle')
    return figure


def rotation_arrow(view: View, origin: Vector, axis: Vector, distance: float, radius: float,
                   first_angle: float) -> List[Point]:
    """Return the points of an arrow that turns about an axis in the positive direction.

    Positive follows the right-hand rule: with the right thumb along the axis, the fingers curl
    in the arrow's direction. distance is how far along the axis the arrow's circle lies, and
    radius is the circle's radius, both in m. The arrow starts at first_angle, in degrees, and
    covers 290 degrees of the circle.
    """
    helper = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
    first = normalized(cross(axis, helper))
    second = cross(axis, first)
    center = add(origin, axis, distance)
    points = []
    for step in range(0, 291, 10):
        angle = math.radians(first_angle + step)
        position = add(add(center, first, radius * math.cos(angle)), second,
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
                    f'{TURN_RATE} rad/s reads {signed(TURN_RATE)} rad/s. The EKF fuses this '
                    'value.')
    view = view_from_behind(1300, (166, 168), 30, 35)
    figure.elements.append('<g class="context">')
    draw_remo_from_behind(figure, view)
    figure.elements.append('</g>')
    figure.text(view.project((0.086, -0.045, 0.0)), 'front', 'note', 'middle')
    length = 118 / view.scale
    start = view.project(IMU_POSITION)
    # Where each curved arrow starts on its circle, so that its head lies on the viewer's side
    first_angles = {'x': 140.0, 'y': 60.0, 'z': -170.0}
    for axis, name in zip(IMU_AXES, AXIS_NAMES):
        end = view.project(add(IMU_POSITION, axis, length))
        figure.arrow(start, end, name, 9)
        offset = (end[0] - start[0], end[1] - start[1])
        reach = math.hypot(*offset)
        figure.text((end[0] + 10 * offset[0] / reach, end[1] + 10 * offset[1] / reach + 4),
                    name, f'{name} halo', 'middle')
        figure.curved_arrow(rotation_arrow(view, IMU_POSITION, axis, 0.68 * length,
                                           0.2 * length, first_angles[name]), f'turn {name}', 8)
    figure.circle(start, 3, 'dot')
    figure.text((70, 28), subscript('ω', 'z'), 'z', 'end', markup=True)
    figure.text((78, 28), f'yaw rate. Left turn at {TURN_RATE} rad/s: {signed(TURN_RATE)} rad/s.')
    figure.text((78, 44), 'The EKF fuses this value.')
    figure.text((186, 150), subscript('ω', 'y'), 'y halo', markup=True)
    figure.text((199, 246), subscript('ω', 'x'), 'x halo', markup=True)
    figure.text((180, 306), 'Each curved arrow shows the direction that counts', 'note', 'middle')
    figure.text((180, 321), 'as positive for the angular velocity about its axis', 'note',
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
    end = (start[0] + offset[0], start[1] + offset[1])
    figure.arrow(start, end, css_class, 9)
    return end


def factor_in_words(factor: int) -> str:
    """Return a whole factor for a sentence: ten as a word, another factor in digits."""
    return 'ten' if factor == 10 else str(factor)


def signed(value: float) -> str:
    """Return a value with at most two decimals and its sign, without a unit. Zero has no sign."""
    return '0' if abs(value) < 0.005 else f'{value:+.2f}'.rstrip('0').rstrip('.')


def specific_force(acceleration: Vector) -> Vector:
    """Return what the sensor reads for an acceleration: the acceleration minus gravity."""
    return add(acceleration, (0.0, 0.0, 1.0), GAZEBO_GRAVITY)


def tilt_from_vertical(reading: Vector) -> float:
    """Return the angle between a reading and the upward direction, in degrees."""
    return math.degrees(math.atan2(math.hypot(reading[0], reading[1]), reading[2]))


# What the three numbers of a specific force drawing are: a level floor, the named motion and no
# noise, along imu_link's axes on Remo. A recorded message differs a little
EXAMPLE_IN_A_SENTENCE = ('In this level, noise-free example, the reading along imu_link’s x, y '
                         'and z axes on Remo is')


def message_numbers(reading: Vector) -> List[str]:
    """Return an ideal reading as x, y and z on Remo, along imu_link's axes.

    These are the message's three numbers when the floor is level and the reading has no noise.
    """
    return [signed(dot(reading, axis)) for axis in IMU_AXES]


def message_values(reading: Vector) -> str:
    """Return an ideal reading's x, y and z on Remo as a drawing shows them: (x, y, z)."""
    return '(' + ', '.join(message_numbers(reading)) + ')'


def message_values_in_a_sentence(reading: Vector) -> str:
    """Return an ideal reading's x, y and z on Remo as a sentence names them: x, y and z."""
    x, y, z = message_numbers(reading)
    return f'{x}, {y} and {z}'


def reading_lines(figure: Figure, first: str, second: str, reading: Vector) -> None:
    """Add what the sensor reads at the bottom of a specific force drawing.

    first and second say it in the robot's directions. reading, in chassis_link's axes, becomes
    three numbers along imu_link's axes on Remo, labeled as a level, noise-free example.
    """
    center = FORCE_WIDTH / 2
    figure.text((center, 232), 'The sensor reads', 'note', 'middle')
    figure.text((center, 249), first, 'reading', 'middle')
    figure.text((center, 265), second, 'reading', 'middle')
    figure.text((center, 284), 'Level, noise-free example', 'note', 'middle')
    figure.text((center, 299), 'in Remo’s IMU axes x, y, z', 'note', 'middle')
    figure.text((center, 316), message_values(reading), 'reading', 'middle')


def draw_magnified_tip(figure: Figure, tip: Point, acceleration: float) -> None:
    """Add a circle that shows the tip of f TIP_MAGNIFICATION times larger.

    At the tip, f = a - g reads as a sum: straight up from the sensor by gravity's amount, then
    forward by the acceleration. The circle shows the end of that upward line, dotted, the arrow
    of a from there to the tip of f, and the end of f's arrow in its true direction.
    """
    center, radius = (160.0, 100.0), 32.0
    toward = (center[0] - tip[0], center[1] - tip[1])
    reach = math.hypot(*toward)
    figure.circle(tip, 7, 'lens')
    figure.polyline([(tip[0] + 7 * toward[0] / reach, tip[1] + 7 * toward[1] / reach),
                     (center[0] - radius * toward[0] / reach,
                      center[1] - radius * toward[1] / reach)], 'lens')
    figure.circle(center, radius, 'inset')
    figure.text((center[0], center[1] - radius - 6), f'{TIP_MAGNIFICATION}\u00d7 larger', 'note',
                'middle')
    forward = acceleration * FORCE_SCALE * TIP_MAGNIFICATION
    reading_tip = (center[0] + forward / 2, center[1] - 4)
    above_sensor = (reading_tip[0] - forward, reading_tip[1])
    shown_height = 30.0
    figure.polyline([(above_sensor[0], above_sensor[1] + shown_height), above_sensor], 'guide')
    figure.arrow((reading_tip[0] - shown_height * acceleration / GAZEBO_GRAVITY,
                  reading_tip[1] + shown_height), reading_tip, 'reading', 9)
    figure.arrow(above_sensor, reading_tip, 'acceleration', 7)
    figure.text((center[0], reading_tip[1] - 8), f'a = {number(acceleration)}', 'acceleration',
                'middle')
    figure.text((reading_tip[0] + 6, reading_tip[1] + 17), 'f', 'reading')


def specific_force_from_the_side_figure(name: str, title: str, description: str,
                                        acceleration: float) -> Figure:
    """Return a drawing of the specific force for a robot on the floor, seen from its right side.

    acceleration is the robot's forward acceleration in m/s². g and f are drawn to FORCE_SCALE.
    """
    figure = Figure(name, FORCE_WIDTH, FORCE_HEIGHT, title, description)
    view = view_from_the_side(900, (118, 152))
    figure.text((6, 18), title, 'title')
    draw_floor(figure, view, 10, FORCE_WIDTH - 10)
    figure.text((FORCE_WIDTH - 10, 146), 'front', 'note', 'end')
    figure.elements.append('<g class="context">')
    draw_remo_from_the_side(figure, view)
    figure.elements.append('</g>')
    sensor = view.project(IMU_POSITION)
    reading = specific_force((acceleration, 0.0, 0.0))
    draw_vector(figure, sensor, (0, GAZEBO_GRAVITY * FORCE_SCALE), 'gravity')
    figure.text((sensor[0] - 8, sensor[1] + GRAVITY_LENGTH / 2 + 8), f'g = {GAZEBO_GRAVITY}',
                'gravity halo', 'end')
    tip = draw_vector(figure, sensor, (reading[0] * FORCE_SCALE, -reading[2] * FORCE_SCALE),
                      'reading')
    figure.text((tip[0], tip[1] - 9), 'f', 'reading halo', 'middle')
    if acceleration:
        draw_magnified_tip(figure, tip, acceleration)
    else:
        figure.text((sensor[0] + 7, sensor[1] - 5), 'a = 0', 'acceleration halo')
    figure.circle(sensor, 3, 'dot')
    return figure


def specific_force_at_rest_figure() -> Figure:
    """Return the drawing of the specific force while the robot stands still."""
    reading = specific_force((0.0, 0.0, 0.0))
    figure = specific_force_from_the_side_figure(
        'imu-force-rest', 'At rest',
        'Remo stands still on the floor, seen from its right side. At the IMU, gravity g points '
        f'down with {GAZEBO_GRAVITY} m/s². The acceleration a is zero. The sensor reads '
        f'f = a − g, which points up with {GAZEBO_GRAVITY} m/s². {EXAMPLE_IN_A_SENTENCE} '
        f'{message_values_in_a_sentence(reading)}.', 0.0)
    reading_lines(figure, 'forward 0,', f'up +{GAZEBO_GRAVITY} m/s²', reading)
    return figure


def specific_force_speeding_up_figure() -> Figure:
    """Return the drawing of the specific force while the robot speeds up forward."""
    reading = specific_force((FORWARD_ACCELERATION, 0.0, 0.0))
    forward = number(FORWARD_ACCELERATION)
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
    figure.arrow((132, 46), (178, 46), 'motion thin', 7)
    figure.text((155, 39), 'speeds up', 'note', 'middle')
    reading_lines(figure, f'forward {signed(FORWARD_ACCELERATION)},',
                  f'up +{GAZEBO_GRAVITY} m/s²', reading)
    return figure


def turn_radius() -> float:
    """Return the IMU's distance from the wheel axle's midpoint, which Remo turns about, in m."""
    return math.hypot(IMU_POSITION[0] - AXLE_X, IMU_POSITION[1])


def circle_points(center: Point, radius: float, first_angle: int, last_angle: int) -> List[Point]:
    """Return points on a circle on the drawing, counterclockwise between two angles in degrees."""
    return [(center[0] + radius * math.cos(math.radians(angle)),
             center[1] - radius * math.sin(math.radians(angle)))
            for angle in range(first_angle, last_angle + 1, 6)]


def turn_acceleration() -> Vector:
    """Return the IMU's acceleration in a turn on the spot: toward the wheel axle's midpoint."""
    return (TURN_RATE ** 2 * (AXLE_X - IMU_POSITION[0]), TURN_RATE ** 2 * -IMU_POSITION[1], 0.0)


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
    view = view_from_above(900, (FORCE_WIDTH / 2, 142))
    figure.text((6, 18), 'Turning left, from above', 'title')
    figure.elements.append('<g class="context">')
    draw_remo_from_above(figure, view)
    figure.elements.append('</g>')
    center = view.project((AXLE_X, 0.0, 0.0))
    sensor = view.project(IMU_POSITION)
    figure.polyline(circle_points(center, turn_radius() * view.scale, 0, 360), 'guide')
    figure.curved_arrow(circle_points(center, 13, -60, 222), 'motion thin', 7)
    figure.circle(center, 2.5, 'dot')
    figure.text((center[0], center[1] - 20), f'{TURN_RATE} rad/s', 'note', 'middle')
    toward = (center[0] - sensor[0], center[1] - sensor[1])
    reach = math.hypot(*toward)
    tip = draw_vector(figure, sensor, (TURN_ACCELERATION_LENGTH * toward[0] / reach,
                                       TURN_ACCELERATION_LENGTH * toward[1] / reach),
                      'acceleration')
    figure.text((tip[0] + 7, tip[1] + 8), f'a = {centripetal}', 'acceleration halo')
    figure.circle(sensor, 5.5, 'ring reading')
    figure.circle(sensor, 2.0, 'head reading')
    figure.text((sensor[0] - 10, sensor[1] + 4), 'f', 'reading halo', 'end')
    reading_lines(figure, f'toward the center {centripetal},', f'up +{GAZEBO_GRAVITY} m/s²',
                  reading)
    return figure


def figure_element(drawings: Sequence[Figure], caption: str, basis: int, widest: int,
                   explanations: Sequence[str] = (), caption_is_html: bool = False) -> str:
    """Return a figure element with the drawings side by side and one caption.

    The drawings wrap below each other on a narrow screen. basis and widest are the width in
    px from which a drawing wraps and its largest width. explanations has one text per drawing,
    as HTML, which stands below its drawing. Without it, the drawings have no text of their
    own. caption is Markdown, or HTML with caption_is_html, for a figure inside raw HTML. The
    drawings stand in a div without the markdown attribute, so that Markdown leaves the SVG's
    text as it is and the glossary doesn't wrap a term such as IMU in an abbr element.
    """
    panels = []
    for index, drawing in enumerate(drawings):
        explanation = f'\n<p>{explanations[index]}</p>' if explanations else ''
        panels.append(f'<div class="imu-panel">\n{drawing.svg()}{explanation}\n</div>')
    figure_markdown, caption_markdown = ('', '') if caption_is_html else (
        ' markdown="block"', ' markdown="span"')
    return (f'<figure class="imu-figure"{figure_markdown} '
            f'style="--imu-basis:{basis}px;--imu-max:{widest}px">\n'
            '<div class="imu-panels">\n' + '\n'.join(panels) + '\n</div>\n'
            f'<figcaption{caption_markdown}>{caption}</figcaption>\n</figure>\n')


def figure_html(drawings: Sequence[Figure], caption: str, basis: int, widest: int,
                explanations: Sequence[str] = ()) -> str:
    """Return an include: the drawings' style and a figure element with a Markdown caption."""
    return (f'{style_element(RULES)}\n'
            + figure_element(drawings, caption, basis, widest, explanations))


def tabs_html(name: str, labels: Sequence[str], figures: Sequence[str]) -> str:
    """Return an include: the drawings' style and a set of tabs with one figure element each.

    name makes the tabs' ids, and labels are the tabs' labels. The tab set is raw HTML, so the
    figure elements need HTML captions.
    """
    inputs = ''.join(
        f'<input{" checked" if index == 0 else ""} id="__tabbed_{name}_{index + 1}" '
        f'name="__tabbed_{name}" type="radio">' for index in range(len(figures)))
    tab_labels = ''.join(f'<label for="__tabbed_{name}_{index + 1}">{escape(label)}</label>'
                         for index, label in enumerate(labels))
    blocks = ''.join(f'<div class="tabbed-block">\n{figure}</div>\n' for figure in figures)
    return (f'{style_element(RULES)}\n<div class="tabbed-set tabbed-alternate imu-tabs" '
            f'data-tabs="{name}:{len(figures)}">{inputs}'
            f'<div class="tabbed-labels">{tab_labels}</div>\n'
            f'<div class="tabbed-content">\n{blocks}</div>\n</div>\n')


def write_frames() -> None:
    """Write the include with the frames on Remo and on DiffBot, one tab per robot."""
    up = centimeters(DIFFBOT_IMU_POSITION[2])
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
    figures = [
        figure_element([frames_from_above_figure(), frames_from_the_side_figure()], remo, 300,
                       400, caption_is_html=True),
        figure_element([frames_on_diffbot_figure()], diffbot, 300, 400, caption_is_html=True)]
    with open('includes/imu-sim-frames.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(tabs_html('imuframes', ROBOT_TAB_LABELS, figures))


def write_angular_velocity() -> None:
    """Write the include with the angular velocity about imu_link's axes."""
    caption = ('`angular_velocity` on Remo: one number per axis of `imu_link`, in rad/s. Red is '
               'x, green is y and blue is z. Point your right thumb along an axis: your fingers '
               'curl in the positive direction. On Remo, `imu_link`’s y axis points forward and '
               'its x axis to the robot’s right, because the board is mounted turned by '
               f'{degrees(IMU_YAW)} about z, as the [figure of Remo’s frames]'
               '(position-and-axes.md) shows.')
    with open('includes/imu-sim-angular-velocity.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(figure_html([angular_velocity_figure()], caption, 300, 480))


def specific_force_explanations() -> List[str]:
    """Return the text below each specific force drawing, as HTML, in the drawings' order."""
    forward = number(FORWARD_ACCELERATION)
    forward_tilt = tilt_from_vertical(specific_force((FORWARD_ACCELERATION, 0.0, 0.0)))
    turn_tilt = tilt_from_vertical(specific_force(turn_acceleration()))
    a, f, g = '<strong>a</strong>', '<strong>f</strong>', '<strong>g</strong>'
    return [
        f'{a} is zero, so the sensor reads {f} = \u2212{g}: {GAZEBO_GRAVITY} m/s² straight up. '
        f'{g} and {f} are drawn to one scale.',
        f'{f} leans forward by only {forward_tilt:.0f}°. So the circle shows the tip of {f} '
        f'{TIP_MAGNIFICATION} times larger: {f} ends {forward} m/s² in front of the dotted '
        'line, which points straight up from the sensor.',
        f'The dotted circle is the sensor’s path. {f} points up, at you, and leans toward the '
        f'center by {turn_tilt:.1f}°. The arrow of {a} shows only its direction: at the scale '
        f'of {g}, its {centripetal_acceleration():.2f} m/s² would be too short to see.',
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
    drawings = [specific_force_at_rest_figure(), specific_force_speeding_up_figure(),
                specific_force_in_a_turn_figure()]
    with open('includes/imu-sim-specific-force.html', 'w', encoding='utf-8') as include:
        include.write(INCLUDE_COMMENT + '\n')
        include.write(figure_html(drawings, caption, FORCE_PANEL_WIDTH, FORCE_PANEL_WIDTH,
                                  specific_force_explanations()))


def main() -> None:
    """Write the three includes."""
    write_frames()
    write_angular_velocity()
    write_specific_force()


if __name__ == '__main__':
    main()
