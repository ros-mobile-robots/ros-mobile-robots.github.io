"""Generate the figures of the two pages with the math basics of the IMU pages as inline SVG.

The pages are docs/theory/preliminaries/noise-and-covariance.md, which shows the first four
includes, and docs/theory/preliminaries/quaternions.md, which shows the last one. The script
writes these includes, which follow the site's light and dark colors:

- includes/math-normal-distribution.html, in "The normal distribution": the density of the
  simulated IMU's noise, a normal distribution with the mean 0 and the standard deviation of the
  plugin's gaussianNoise, with the shares within one, two and three standard deviations.
- includes/math-covariance-scatter.html, in "The covariance of two values": two clouds of example
  pairs of deviations, drawn from normal distributions with a positive covariance and with the
  covariance 0, the sign of their product in each quadrant, and the ellipses of one and two
  standard deviations over each cloud. The caption gives each cloud's number of positive products
  and its sample covariance. The figure has a narrow layout with the two plots one below the
  other and a wide one with the plots side by side.
- includes/math-covariance-axes.html, in "The covariance ellipsoid": the ellipse of one standard
  deviation of the example covariance matrix, its semi-axes along the eigenvectors, and the
  circle that the matrix would give without its covariance.
- includes/math-covariance-shares.html, in "The covariance ellipsoid": the ellipses of one, two
  and three standard deviations of the same matrix, each labeled with the share of the values
  inside it.
- includes/math-quaternion-circle.html, in "Remo's IMU, turned by −90°" on the Quaternions page:
  the two numbers of a yaw's quaternion that aren't 0, as a point on the unit circle at half the
  yaw, for the yaw of Remo's IMU, with the opposite point, which is the same rotation.

Each include holds one figure of one section, and one function writes it.

Every drawing is 300 units wide, so that its text is readable on a phone. The wide layout of the
scatter figure is two such drawings wide, and the CSS of LAYOUT_RULES in localization_figures.py
shows it where the text column has room for it. Explanations are HTML text in the caption.
Figure, the CSS rules, the colors BG, FG and FG_LIGHT, bell() and the functions that write an
include come from localization_figures.py. escape(), Point, Box and normal_values(), the random
values of the example pairs, come from board_drawings.py.

Run from the repository root after changing a figure:

    python3 scripts/figures/math_basics_figures.py
"""

import math
import statistics
from typing import List
from typing import NamedTuple
from typing import Sequence

from board_drawings import Box
from board_drawings import escape
from board_drawings import normal_values
from board_drawings import Point
from localization_figures import arrow_head
from localization_figures import bell
from localization_figures import BG
from localization_figures import DRAWING_RULES
from localization_figures import FG
from localization_figures import FG_LIGHT
from localization_figures import Figure
from localization_figures import LARGE_RULES
from localization_figures import NARROW_LAYOUT
from localization_figures import RULES
from localization_figures import Span
from localization_figures import SYMBOL_RULES
from localization_figures import WIDE_LAYOUT
from localization_figures import write_layout_parts
from localization_figures import write_parts

DRAWING_WIDTH = 300
# The size of the drawings' text, as in LARGE_RULES, of a variable, as in DRAWING_RULES, and of
# the signs in the quadrants of the example pairs
TEXT_SIZE = 12  # px
VARIABLE_SIZE = 14  # px
SIGN_SIZE = 17  # px
# The size of the smallest text, the numbers along the axes: in a narrow drawing as in LARGE_RULES,
# and in the wide layout of the scatter figure. The text column shows that layout a little
# smaller than its largest width, so its smallest text is set larger. It is then as large on the
# page as a narrow drawing's on a phone, which tests/test_math_basics_figures.py checks
NOTE_SIZE = 11  # px
WIDE_NOTE_SIZE = 11.5  # px
# The widest character of the drawings' text, as a share of its size: an upper bound, so that
# a label's box is never smaller than the label
CHARACTER_WIDTH = 0.62
# Where a line of text has its baseline, below the middle of its letters, as a share of its size
BASELINE_SHIFT = 0.38
# The noise is the IMU's series color, the example position estimate the estimate's
MATH_RULES = (
    ('.lv-math .lv-fig .noise-band', 'fill:var(--lv-imu);fill-opacity:.17;stroke:none'),
    ('.lv-math .lv-fig .noise-curve', 'fill:none;stroke:var(--lv-imu);stroke-width:2'),
    ('.lv-math .lv-fig .band-edge', 'fill:none;stroke:var(--lv-imu);stroke-width:1'),
    ('.lv-math .lv-fig .share-bracket', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.1'),
    ('.lv-math .lv-fig .ring-fill', 'fill:var(--lv-estimate);fill-opacity:.11;stroke:none'),
    ('.lv-math .lv-fig .ring', 'fill:none;stroke:var(--lv-estimate);stroke-width:1.6'),
    ('.lv-math .lv-fig .no-covariance', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3;'
                                        'stroke-dasharray:5 3'),
    ('.lv-math .lv-fig .semi-axis', f'fill:none;stroke:{FG};stroke-width:2.2'),
    ('.lv-math .lv-fig .zero-line', f'fill:none;stroke:{FG_LIGHT};stroke-width:.9'),
    ('.lv-math .lv-fig .pair', f'fill:{FG};fill-opacity:.7;stroke:none'),
    ('.lv-math .lv-fig .sign', f'font-size:{SIGN_SIZE}px;font-weight:700'),
    ('.lv-math .lv-fig .angle-arc', f'fill:none;stroke:{FG};stroke-width:1.2'),
    ('.lv-math .lv-fig .angle-head', f'fill:{FG};stroke:none'),
    ('.lv-math .lv-fig .axis-head', f'fill:{FG_LIGHT};stroke:none'),
    ('.lv-math .lv-fig .unit-circle', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    # The quaternion's line is solid and the opposite one's dashed, and each point has its name
    ('.lv-math .lv-fig .quaternion-radius', 'fill:none;stroke:var(--lv-estimate);'
                                            'stroke-width:2.2'),
    ('.lv-math .lv-fig .opposite-radius', 'fill:none;stroke:var(--lv-estimate);'
                                          'stroke-width:1.5;stroke-dasharray:5 3'),
    ('.lv-math .lv-fig .opposite-dot', f'fill:{BG};stroke:var(--lv-estimate);stroke-width:1.6'),
    # A label over the example pairs: a band in the page's color around its letters keeps the
    # label readable, and the dots stay visible around the letters
    ('.lv-math .lv-fig .halo', f'paint-order:stroke;stroke:{BG};stroke-width:3px;'
                               'stroke-linejoin:round'),
    (f'.lv-math .lv-fig.{WIDE_LAYOUT} .note', f'font-size:{WIDE_NOTE_SIZE}px'),
)
FIGURE_RULES = RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES + MATH_RULES
FIGURE_CLASSES = 'lv-large lv-drawing lv-symbols lv-math'
INCLUDE_COMMENT = '<!-- Written by scripts/figures/math_basics_figures.py -->'

# The noise of the simulated IMU: the plugin's parameter gaussianNoise in
# diffbot_description/urdf/include/common_sensors.xacro of the diffbot repository, in rad/s
SIMULATED_IMU_NOISE_SIGMA = 0.005
STANDARD_DEVIATIONS = (1, 2, 3)

# The figure of the normal distribution. Its axis shows the noise in 10⁻³ rad/s, and the name of
# the curve stands next to the curve at CURVE_NAME_POSITION standard deviations
NOISE_AXIS_EXPONENT = -3
CURVE_NAME_POSITION = 1.35
BELL_CENTER_X = 150
BELL_PIXELS_PER_SIGMA = 37
BELL_PEAK = 112
BELL_BASE_Y = 150
BRACKET_FIRST_Y = BELL_BASE_Y + 66
BRACKET_SPACING = 18
BRACKET_LABEL_GAP = 25  # half the gap in a bracket's line for its label


class Covariance(NamedTuple):
    """The covariance matrix of two values x and y: ((xx, xy), (xy, yy))."""

    xx: float  # the variance of x
    xy: float  # the covariance of x and y
    yy: float  # the variance of y


class PrincipalAxes(NamedTuple):
    """The eigenvalues of a covariance matrix of two values, and its ellipses' angle."""

    major: float  # the larger eigenvalue
    minor: float  # the smaller eigenvalue
    angle: float  # degrees from the x axis to the larger one's eigenvector, counterclockwise


class Deviation(NamedTuple):
    """A pair of deviations from the mean, in cm."""

    x: float
    y: float


# The example covariance matrix of a position estimate (x, y) on the page, in cm², and the same
# matrix without its covariance
EXAMPLE_COVARIANCE = Covariance(xx=5.0, xy=3.0, yy=5.0)
UNCORRELATED_COVARIANCE = EXAMPLE_COVARIANCE._replace(xy=0.0)


class ExampleCloud(NamedTuple):
    """The example pairs of one plot of the scatter figure: their distribution and their seed."""

    covariance: Covariance  # the matrix of the normal distribution that the pairs come from
    seed: int  # for Python's generator
    shape: str  # the word for the cloud in the figure's caption, such as "tilted"

    def pairs(self) -> List[Deviation]:
        """Return the cloud's SCATTER_COUNT pairs, the same in every run."""
        return example_pairs(self.covariance, self.seed, SCATTER_COUNT)


# The example pairs of deviations: their number, and a cloud of pairs for each plot. With each
# cloud's seed, every pair lies inside the plot and off the quadrants' signs, and the pairs'
# sample covariance lies within COVARIANCE_TOLERANCE of the distribution's, which
# tests/test_math_basics_figures.py checks. In the cloud without the covariance, each quadrant
# also holds a quarter of the pairs, within the tests' QUADRANT_TOLERANCE
SCATTER_COUNT = 200
SCATTER_CLOUDS = (ExampleCloud(EXAMPLE_COVARIANCE, seed=150, shape='tilted'),
                  ExampleCloud(UNCORRELATED_COVARIANCE, seed=494, shape='round'))
# The ellipses over the example pairs
SCATTER_STANDARD_DEVIATIONS = STANDARD_DEVIATIONS[:2]


class Plot(NamedTuple):
    """A plot of the deviations of two values in cm, with the same scale on both axes."""

    origin: Point  # where both deviations are 0
    pixels_per_cm: float
    low: float  # cm, where both axes start
    high_x: float  # cm, where the x axis ends
    high_y: float  # cm, where the y axis ends
    tick_spacing: float  # cm

    def at(self, x: float, y: float) -> Point:
        """Return the drawing's position of the deviations x and y."""
        return Point(self.origin.x + x * self.pixels_per_cm,
                     self.origin.y - y * self.pixels_per_cm)

    def shifted(self, offset: Point) -> 'Plot':
        """Return the same plot, moved by offset in the drawing."""
        return self._replace(origin=Point(self.origin.x + offset.x, self.origin.y + offset.y))

    def drawing_bottom(self) -> int:
        """Return the y under the plot's x axis with its labels and name."""
        return round(self.at(0, self.low).y) + X_AXIS_SPACE


PLOT_LEFT = 34  # the y axis' labels stand to the left of the plot
PLOT_TOP = 26  # the y axis' name and the plot's title stand above the plot
# Where the y axis' name starts, from the top left corner of the plot
Y_NAME_OFFSET = Point(4 - PLOT_LEFT, 14 - PLOT_TOP)
X_AXIS_SPACE = 44  # px under the plot for the x axis' labels and name
# The plots of the example pairs and of the shares inside the ellipses: the ellipse of three
# standard deviations fits, and its label above it
OVERVIEW_PLOT = Plot(origin=Point(PLOT_LEFT + 7 * 18.5, PLOT_TOP + 7.5 * 18.5),
                     pixels_per_cm=18.5, low=-7, high_x=7, high_y=7.5, tick_spacing=2)
# The plot of the ellipse of one standard deviation and its axes, enlarged
CLOSE_PLOT = Plot(origin=Point(PLOT_LEFT + 3 * 42, PLOT_TOP + 3 * 42),
                  pixels_per_cm=42, low=-3, high_x=3, high_y=3, tick_spacing=1)
# The direction from the mean, counterclockwise from the x axis, in which each ellipse of the
# shares figure carries its label: there, the next ellipses lie far enough away from the label
SHARE_LABEL_DIRECTION = 55  # degrees
# The direction in which the ellipses over the example pairs carry their labels: across the cloud
# with the positive covariance, as the labels of the axes figure
SCATTER_LABEL_DIRECTION = 135  # degrees


class ScatterLayout(NamedTuple):
    """One of the scatter figure's two layouts, each with the two plots of the example pairs."""

    name: str  # the drawing's name, unique on the page
    css_class: str  # NARROW_LAYOUT or WIDE_LAYOUT, by which the CSS shows one layout
    width: int
    second_plot_offset: Point  # where the second plot stands, from the first one


# The narrow layout has the second plot below the first one, PLOT_SPACING apart. The wide layout
# has the plots side by side and is two drawings wide: at its figure's largest width of 36rem,
# its text is as large as the narrow layout's at that layout's largest width of 18rem
PLOT_SPACING = 16  # px
SCATTER_LAYOUTS = (
    ScatterLayout('math-covariance-scatter', NARROW_LAYOUT, DRAWING_WIDTH,
                  Point(0, OVERVIEW_PLOT.drawing_bottom() + PLOT_SPACING)),
    ScatterLayout('math-covariance-scatter-wide', WIDE_LAYOUT, 2 * DRAWING_WIDTH,
                  Point(DRAWING_WIDTH, 0)),
)
# The example pairs' dots, and the signs of their products, which stand this far from both axes
PAIR_RADIUS = 2
QUADRANT_SIGN_POSITION = 6.2  # cm
# A label along a line: the gap between the two, and the height of the label's letters
LABEL_GAP = 4  # px
CAP_HEIGHT = 9  # px
# The angle of the ellipse's turn: its arc and label, from the mean, and the reference line
ANGLE_ARC_RADIUS = 30  # px
ANGLE_LABEL_RADIUS = 46  # px
ANGLE_REFERENCE_LENGTH = 1.6  # cm


def principal_axes(covariance: Covariance) -> PrincipalAxes:
    """Return the eigenvalues of a covariance matrix and the angle of its ellipses."""
    mean = (covariance.xx + covariance.yy) / 2
    radius = math.hypot((covariance.xx - covariance.yy) / 2, covariance.xy)
    angle = math.degrees(math.atan2(2 * covariance.xy, covariance.xx - covariance.yy)) / 2
    return PrincipalAxes(mean + radius, mean - radius, angle)


def mahalanobis_distance(covariance: Covariance, deviation: Deviation) -> float:
    """Return the Mahalanobis distance of a deviation from the mean.

    The distance is the deviation's length when each of its components along a principal axis
    of the covariance matrix counts in units of that axis's standard deviation. The matrix has
    to be positive definite, so that its determinant isn't 0.
    """
    determinant = covariance.xx * covariance.yy - covariance.xy ** 2
    squared = (covariance.yy * deviation.x ** 2 - 2 * covariance.xy * deviation.x * deviation.y
               + covariance.xx * deviation.y ** 2) / determinant
    return math.sqrt(squared)


def share_within_for_one_value(standard_deviations: float) -> float:
    """Return the share of a normally distributed value within this many standard deviations."""
    return math.erf(standard_deviations / math.sqrt(2))


def share_within_for_two_values(standard_deviations: float) -> float:
    """Return the share of jointly normal pairs inside the ellipse of this many σ.

    The squared Mahalanobis distance of a pair follows the chi-squared distribution with two
    degrees of freedom.
    """
    return 1 - math.exp(-standard_deviations ** 2 / 2)


SUPERSCRIPTS = str.maketrans('-0123456789', '⁻⁰¹²³⁴⁵⁶⁷⁸⁹')


def listed(values: Sequence[str]) -> str:
    """Return values as a list in words, such as "1, 2 and 3"."""
    return ', '.join(values[:-1]) + ' and ' + values[-1] if len(values) > 1 else values[0]


def percent(share: float) -> str:
    """Return a share as a percentage with one decimal, such as "68.3 %"."""
    return f'{100 * share:.1f} %'


def minus_signed(value: float) -> str:
    """Return a number with a minus sign instead of a hyphen."""
    return f'{value:g}'.replace('-', '−')


def three_decimals(value: float) -> str:
    """Return a number with three decimals and a minus sign, such as "−0.009"."""
    return f'{round(value, 3) + 0:.3f}'.replace('-', '−')


def multiple_of_sigma(multiple: int) -> Sequence[Span]:
    """Return the spans of a multiple of σ, such as −2σ, 0 or σ."""
    if multiple == 0:
        return (Span('0'),)
    factor = '' if abs(multiple) == 1 else str(abs(multiple))
    sign = '−' if multiple < 0 else ''
    return (Span(f'{sign}{factor}'), Span('σ', 'variable'))


def text_width(text: str, size: float = TEXT_SIZE) -> float:
    """Return an upper bound of the width of a line of text in px."""
    return len(text) * size * CHARACTER_WIDTH


def write_figure(path: str, figure: Figure, caption: str) -> None:
    """Write an include with one drawing and its caption, in this script's style.

    The include starts with a comment that names this script.
    """
    write_parts(path, [figure.svg()], caption, FIGURE_RULES, FIGURE_CLASSES, INCLUDE_COMMENT)


def write_layout_figure(path: str, layouts: Sequence[Figure], caption: str) -> None:
    """Write an include with a drawing's narrow and wide layout and its caption.

    The include starts with a comment that names this script. write_layout_parts() puts the
    figure into the block whose width decides which layout shows.
    """
    write_layout_parts(path, [layout.svg() for layout in layouts], caption, FIGURE_RULES,
                       FIGURE_CLASSES, INCLUDE_COMMENT)


def bell_band(standard_deviations: float) -> str:
    """Return the path of the area under the bell curve within this many standard deviations."""
    half = standard_deviations * BELL_PIXELS_PER_SIGMA
    points = []
    steps = round(standard_deviations * 10)
    for step in range(-steps, steps + 1):
        multiple = step / 10
        points.append(f'{BELL_CENTER_X + multiple * BELL_PIXELS_PER_SIGMA:.1f},'
                      f'{BELL_BASE_Y - bell_height(multiple):.1f}')
    return (f'M{BELL_CENTER_X - half:.1f},{BELL_BASE_Y} L' + ' L'.join(points)
            + f' L{BELL_CENTER_X + half:.1f},{BELL_BASE_Y} z')


def bell_height(multiple: float) -> float:
    """Return the height of the bell curve in px, at this many standard deviations."""
    return BELL_PEAK * math.exp(-multiple ** 2 / 2)


def share_bracket(figure: Figure, standard_deviations: int, y: float, label: str) -> None:
    """Add a bracket from −k σ to +k σ under the axis, with its label in a gap in its middle."""
    left = BELL_CENTER_X - standard_deviations * BELL_PIXELS_PER_SIGMA
    right = BELL_CENTER_X + standard_deviations * BELL_PIXELS_PER_SIGMA
    figure.path(f'M{left:.1f},{y - 5} V{y} H{BELL_CENTER_X - BRACKET_LABEL_GAP} '
                f'M{BELL_CENTER_X + BRACKET_LABEL_GAP},{y} H{right:.1f} V{y - 5}',
                'share-bracket')
    figure.text(BELL_CENTER_X, y + 4, label, 'strong', 'middle')


def normal_shares() -> List[str]:
    """Return the shares of a normally distributed value within 1, 2 and 3 σ, as percentages."""
    return [percent(share_within_for_one_value(multiple)) for multiple in STANDARD_DEVIATIONS]


def normal_distribution_figure() -> Figure:
    """Return the drawing of the simulated IMU's noise and the shares within ±1, 2 and 3 σ."""
    shares = normal_shares()
    height = BRACKET_FIRST_Y + (len(STANDARD_DEVIATIONS) - 1) * BRACKET_SPACING + 10
    figure = Figure('math-normal-distribution', "The bell curve of the simulated IMU's noise, "
                    f'with the shares {listed(shares)} within one, two and three standard '
                    'deviations', height, DRAWING_WIDTH)
    for multiple in reversed(STANDARD_DEVIATIONS):
        figure.path(bell_band(multiple), 'noise-band')
    for multiple in STANDARD_DEVIATIONS:
        for sign in (-1, 1):
            x = BELL_CENTER_X + sign * multiple * BELL_PIXELS_PER_SIGMA
            figure.path(f'M{x:.1f},{BELL_BASE_Y} V{BELL_BASE_Y - bell_height(multiple):.1f}',
                        'band-edge')
    figure.path(bell(BELL_CENTER_X, BELL_PIXELS_PER_SIGMA, BELL_PEAK, BELL_BASE_Y),
                'noise-curve')
    figure.rich_text(BELL_CENTER_X + CURVE_NAME_POSITION * BELL_PIXELS_PER_SIGMA + 8,
                     BELL_BASE_Y - bell_height(CURVE_NAME_POSITION) - 4,
                     (Span('p', 'variable'), Span('('), Span('n', 'variable'), Span(')')))
    figure.path(f'M12,{BELL_BASE_Y} H{DRAWING_WIDTH - 12}', 'axis')
    for multiple in range(-STANDARD_DEVIATIONS[-1], STANDARD_DEVIATIONS[-1] + 1):
        x = BELL_CENTER_X + multiple * BELL_PIXELS_PER_SIGMA
        figure.path(f'M{x:.1f},{BELL_BASE_Y} v4', 'tick')
        figure.rich_text(x, BELL_BASE_Y + 18, multiple_of_sigma(multiple), 'middle')
        on_axis = multiple * SIMULATED_IMU_NOISE_SIGMA / 10 ** NOISE_AXIS_EXPONENT
        figure.text(x, BELL_BASE_Y + 33, minus_signed(round(on_axis, 6)), 'note', 'middle')
    unit = f'10{str(NOISE_AXIS_EXPONENT).translate(SUPERSCRIPTS)} rad/s'
    figure.rich_text(BELL_CENTER_X, BELL_BASE_Y + 49,
                     (Span('n', 'variable'), Span(f' in {unit}')), 'middle', 'note')
    for row, (multiple, share) in enumerate(zip(STANDARD_DEVIATIONS, shares)):
        share_bracket(figure, multiple, BRACKET_FIRST_Y + row * BRACKET_SPACING, share)
    return figure


def write_normal_distribution() -> None:
    """Write the figure of the normal distribution, in "The normal distribution"."""
    shares = normal_shares()
    sigma = SIMULATED_IMU_NOISE_SIGMA
    write_figure(
        'includes/math-normal-distribution.html', normal_distribution_figure(),
        'The density <em>p</em>(<em>n</em>) of the noise <em>n</em> that the simulated IMU adds '
        f'to each value of the angular velocity, with σ = {sigma} rad/s. The shaded areas hold '
        f'the values within one, two and three standard deviations of the mean 0: {shares[0]} '
        f'between −{sigma} and +{sigma} rad/s, {shares[1]} within ±{2 * sigma:.3f} rad/s and '
        f'{shares[2]} within ±{3 * sigma:.3f} rad/s.')


def plot_axes(figure: Figure, plot: Plot) -> None:
    """Add the plot's two axes along its left and bottom edges, with ticks and names.

    The axes are the deviations of x and y from their means μ, in cm.
    """
    left, bottom = plot.at(plot.low, plot.low)
    right, top = plot.at(plot.high_x, plot.high_y)
    figure.path(f'M{left:.1f},{top:.1f} V{bottom:.1f} H{right:.1f}', 'axis')
    ticks = math.floor(plot.high_x / plot.tick_spacing)
    for multiple in range(-ticks, ticks + 1):
        value = multiple * plot.tick_spacing
        x, y = plot.at(value, value)
        figure.path(f'M{x:.1f},{bottom:.1f} v4', 'tick')
        figure.text(x, bottom + 17, minus_signed(value), 'note', 'middle')
        figure.path(f'M{left:.1f},{y:.1f} h-4', 'tick')
        figure.text(left - 7, y + 4, minus_signed(value), 'note', 'end')
    figure.rich_text(right, bottom + 36, (Span('x', 'variable'), Span(' − '),
                                          Span('μ', 'variable'), Span('x', 'subscript index'),
                                          Span(' in cm')), 'end')
    figure.rich_text(left + Y_NAME_OFFSET.x, top + Y_NAME_OFFSET.y,
                     (Span('y', 'variable'), Span(' − '), Span('μ', 'variable'),
                      Span('y', 'subscript index'), Span(' in cm')))


def ellipse_path(plot: Plot, covariance: Covariance, standard_deviations: float) -> str:
    """Return the path of the ellipse of this many standard deviations around the mean."""
    axes = principal_axes(covariance)
    major = standard_deviations * math.sqrt(axes.major) * plot.pixels_per_cm
    minor = standard_deviations * math.sqrt(axes.minor) * plot.pixels_per_cm
    angle = math.radians(axes.angle)
    # The drawing's y points down, so the ellipse turns clockwise in its numbers
    offset_x, offset_y = major * math.cos(angle), -major * math.sin(angle)
    start = Point(plot.origin.x + offset_x, plot.origin.y + offset_y)
    end = Point(plot.origin.x - offset_x, plot.origin.y - offset_y)
    arc = f'A{major:.1f},{minor:.1f} {-axes.angle + 0:g} 0 1'
    return (f'M{start.x:.1f},{start.y:.1f} {arc} {end.x:.1f},{end.y:.1f} '
            f'{arc} {start.x:.1f},{start.y:.1f} z')


def point_in_direction(covariance: Covariance, standard_deviations: float,
                       degrees: float) -> Deviation:
    """Return the point of the ellipse of this many standard deviations in a direction.

    degrees is the direction from the mean, counterclockwise from the x axis.
    """
    direction = Deviation(math.cos(math.radians(degrees)), math.sin(math.radians(degrees)))
    distance = standard_deviations / mahalanobis_distance(covariance, direction)
    return Deviation(distance * direction.x, distance * direction.y)


class LineLabel(NamedTuple):
    """A label centered on a point, such as a point of a line: its text and the space it takes."""

    center: Point
    spans: Sequence[Span]
    size: float  # px, of its text
    box: Box  # at most, so that a gap in a line of this size is never too small


def line_label(center: Point, spans: Sequence[Span], size: float = TEXT_SIZE) -> LineLabel:
    """Return a label centered on a point."""
    width = text_width(''.join(span.content for span in spans), size)
    height = max(size, VARIABLE_SIZE) + 2
    return LineLabel(center, tuple(spans), size,
                     Box(center.x - width / 2, center.y - height / 2, width, height))


def add_line_label(figure: Figure, label: LineLabel, css_class: str = 'strong') -> None:
    """Add a label centered on its point."""
    baseline = label.center.y + BASELINE_SHIFT * max(label.size, VARIABLE_SIZE)
    figure.rich_text(label.center.x, baseline, label.spans, 'middle', css_class)


def add_gap_mask(figure: Figure, boxes: Sequence[Box]) -> str:
    """Add a mask that leaves gaps for labels, and return its reference for a path.

    A path that the mask applies to shows everywhere but in the boxes, as the lines of a contour
    map do around their labels. The mask's id starts with the drawing's name, which is unique
    on the page.
    """
    mask_id = f'{figure.name}-gaps'
    holes = ''.join(f'<rect x="{box.x:.1f}" y="{box.y:.1f}" width="{box.width:.1f}" '
                    f'height="{box.height:.1f}" rx="3" fill="black"/>' for box in boxes)
    figure.elements.append(
        f'<defs><mask id="{mask_id}" maskUnits="userSpaceOnUse" x="0" y="0" '
        f'width="{figure.width}" height="{figure.height}"><rect x="0" y="0" '
        f'width="{figure.width}" height="{figure.height}" fill="white"/>{holes}</mask></defs>')
    return f'url(#{mask_id})'


def add_masked_path(figure: Figure, commands: str, css_class: str, mask: str) -> None:
    """Add a path that shows only where the mask lets it."""
    figure.elements.append(f'<path class="{css_class}" d="{commands}" mask="{mask}"/>')


def sigma_label_spans(standard_deviations: int) -> Sequence[Span]:
    """Return the label of an ellipse of this many standard deviations, such as 2σ."""
    return (Span(str(standard_deviations)), Span('σ', 'variable'))


def share_label_spans(standard_deviations: int) -> Sequence[Span]:
    """Return the label of an ellipse: its standard deviations and the share inside it."""
    share = percent(share_within_for_two_values(standard_deviations))
    return tuple(sigma_label_spans(standard_deviations)) + (Span(f': {share}'),)


def share_labels(plot: Plot, covariance: Covariance) -> List[LineLabel]:
    """Return the labels of the ellipses of 1, 2 and 3 σ, each on its ellipse."""
    labels = []
    for multiple in STANDARD_DEVIATIONS:
        point = point_in_direction(covariance, multiple, SHARE_LABEL_DIRECTION)
        labels.append(line_label(plot.at(point.x, point.y), share_label_spans(multiple)))
    return labels


def covariance_shares_figure() -> Figure:
    """Return the drawing of the ellipses of 1, 2 and 3 σ and the share inside each."""
    covariance, plot = EXAMPLE_COVARIANCE, OVERVIEW_PLOT
    shares = listed([percent(share_within_for_two_values(multiple))
                     for multiple in STANDARD_DEVIATIONS])
    figure = Figure('math-covariance-shares', 'The ellipses of one, two and three standard '
                    'deviations around the mean, labeled with the shares inside: '
                    f'{shares}', plot.drawing_bottom(), DRAWING_WIDTH)
    labels = share_labels(plot, covariance)
    mask = add_gap_mask(figure, [label.box for label in labels])
    for multiple in reversed(STANDARD_DEVIATIONS):
        figure.path(ellipse_path(plot, covariance, multiple), 'ring-fill')
    for multiple in STANDARD_DEVIATIONS:
        add_masked_path(figure, ellipse_path(plot, covariance, multiple), 'ring', mask)
    figure.circle(plot.origin.x, plot.origin.y, 4, 'estimate-dot')
    plot_axes(figure, plot)
    for label in labels:
        add_line_label(figure, label)
    return figure


def write_covariance_shares() -> None:
    """Write the figure of the shares inside the ellipses, in "The covariance ellipsoid"."""
    shares = listed([percent(share_within_for_two_values(multiple))
                     for multiple in STANDARD_DEVIATIONS])
    write_figure(
        'includes/math-covariance-shares.html', covariance_shares_figure(),
        'The ellipses of one, two and three standard deviations σ of the example covariance '
        'matrix, around the mean of the position estimate. The label on each ellipse gives the '
        f'probability that a jointly normal position lies inside it: {shares}, the shares of the '
        'table for two values.')


def label_along(figure: Figure, start: Point, end: Point, text: str,
                above: bool) -> None:
    """Add a label along a line from start to end, turned with the line so that it reads upright.

    above puts the label on the side of the line above the text's baseline, otherwise below it.
    """
    angle = math.degrees(math.atan2(end.y - start.y, end.x - start.x))
    if angle > 90:
        angle -= 180
    elif angle < -90:
        angle += 180
    # The direction in which the turned text's letters stand up from their baseline
    up_x, up_y = math.sin(math.radians(angle)), -math.cos(math.radians(angle))
    offset = LABEL_GAP if above else -(LABEL_GAP + CAP_HEIGHT)
    x = (start.x + end.x) / 2 + up_x * offset
    y = (start.y + end.y) / 2 + up_y * offset
    figure.elements.append(f'<text class="strong" x="{x:.1f}" y="{y:.1f}" '
                           f'text-anchor="middle" transform="rotate({angle:.1f} {x:.1f} '
                           f'{y:.1f})">{escape(text)}</text>')


def length_text(centimeters: float) -> str:
    """Return a length in cm with two decimals, such as "2.83 cm"."""
    return f'{centimeters:.2f} cm'


def covariance_axes_figure() -> Figure:
    """Return the drawing of the ellipse of one σ, its semi-axes and the circle without covariance.

    The semi-axes lie along the eigenvectors of the covariance matrix, and their lengths are the
    square roots of the eigenvalues. Without the covariance, the matrix's ellipse has the
    standard deviations of x and y as its semi-axes, a circle for equal variances.
    """
    covariance, plot = EXAMPLE_COVARIANCE, CLOSE_PLOT
    axes = principal_axes(covariance)
    without_covariance = Covariance(covariance.xx, 0, covariance.yy)
    major, minor = math.sqrt(axes.major), math.sqrt(axes.minor)
    figure = Figure('math-covariance-axes', 'The ellipse of one standard deviation, turned by '
                    f'{axes.angle:g}°, with its semi-axes of {length_text(major)} and '
                    f'{length_text(minor)} and a dashed circle without the covariance',
                    plot.drawing_bottom(), DRAWING_WIDTH)
    across = axes.angle - 90
    major_end = plot.at(major * math.cos(math.radians(axes.angle)),
                        major * math.sin(math.radians(axes.angle)))
    minor_end = plot.at(minor * math.cos(math.radians(across)),
                        minor * math.sin(math.radians(across)))
    # The two labels on lines stand opposite the minor semi-axis
    ring_point = point_in_direction(covariance, 1, across + 180)
    circle_point = point_in_direction(without_covariance, 1, across + 180)
    labels = (line_label(plot.at(ring_point.x, ring_point.y), sigma_label_spans(1)),
              line_label(plot.at(circle_point.x, circle_point.y),
                         (Span(length_text(math.sqrt(covariance.xx))),)))
    mask = add_gap_mask(figure, [label.box for label in labels])
    figure.path(ellipse_path(plot, covariance, 1), 'ring-fill')
    add_masked_path(figure, ellipse_path(plot, without_covariance, 1), 'no-covariance', mask)
    add_masked_path(figure, ellipse_path(plot, covariance, 1), 'ring', mask)
    reference_end = plot.at(ANGLE_REFERENCE_LENGTH, 0)
    figure.path(f'M{plot.origin.x:.1f},{plot.origin.y:.1f} H{reference_end.x:.1f}', 'reference')
    arc_end = Point(plot.origin.x + ANGLE_ARC_RADIUS * math.cos(math.radians(axes.angle)),
                    plot.origin.y - ANGLE_ARC_RADIUS * math.sin(math.radians(axes.angle)))
    figure.path(f'M{plot.origin.x + ANGLE_ARC_RADIUS:.1f},{plot.origin.y:.1f} '
                f'A{ANGLE_ARC_RADIUS},{ANGLE_ARC_RADIUS} 0 0 0 {arc_end.x:.1f},{arc_end.y:.1f}',
                'angle-arc')
    half_angle = math.radians(axes.angle / 2)
    figure.text(plot.origin.x + ANGLE_LABEL_RADIUS * math.cos(half_angle),
                plot.origin.y - ANGLE_LABEL_RADIUS * math.sin(half_angle) + 4,
                f'{axes.angle:g}°', 'strong', 'middle')
    for end in (major_end, minor_end):
        figure.path(f'M{plot.origin.x:.1f},{plot.origin.y:.1f} L{end.x:.1f},{end.y:.1f}',
                    'semi-axis')
    label_along(figure, plot.origin, major_end, length_text(major), above=True)
    label_along(figure, plot.origin, minor_end, length_text(minor), above=False)
    figure.circle(plot.origin.x, plot.origin.y, 4, 'estimate-dot')
    plot_axes(figure, plot)
    for label in labels:
        add_line_label(figure, label)
    return figure


def write_covariance_axes() -> None:
    """Write the figure of the ellipse's axes, in "The covariance ellipsoid"."""
    covariance = EXAMPLE_COVARIANCE
    axes = principal_axes(covariance)
    write_figure(
        'includes/math-covariance-axes.html', covariance_axes_figure(),
        f'The ellipse of one standard deviation σ of the example covariance matrix, turned by '
        f'{axes.angle:g}°. Its semi-axes lie along the eigenvectors: √{axes.major:g} cm = '
        f'{length_text(math.sqrt(axes.major))} and √{axes.minor:g} cm = '
        f'{length_text(math.sqrt(axes.minor))}. The dashed circle with the radius '
        f'√{covariance.xx:g} cm = {length_text(math.sqrt(covariance.xx))} is the ellipse that '
        'the matrix would have without its covariance.')


def example_pairs(covariance: Covariance, seed: int, count: int) -> List[Deviation]:
    """Return count pairs of deviations, normally distributed with the covariance matrix.

    Two independent standard normal values a and b become x = √xx a and y = c a + d b, with
    c and d chosen so that y has the variance yy and the covariance xy with x (the matrix's
    Cholesky factor).
    """
    values = normal_values(seed, 2 * count)
    scale_x = math.sqrt(covariance.xx)
    shared = covariance.xy / scale_x
    own = math.sqrt(covariance.yy - shared ** 2)
    return [Deviation(scale_x * first, shared * first + own * second)
            for first, second in zip(values[0::2], values[1::2])]


def sample_covariance(pairs: Sequence[Deviation]) -> Covariance:
    """Return the sample variances and the sample covariance of pairs, divided by N − 1."""
    xs = [pair.x for pair in pairs]
    ys = [pair.y for pair in pairs]
    return Covariance(statistics.variance(xs), statistics.covariance(xs, ys),
                      statistics.variance(ys))


def positive_product_count(pairs: Sequence[Deviation]) -> int:
    """Return how many pairs have deviations with the same sign: a positive product."""
    return sum(1 for pair in pairs if pair.x * pair.y > 0)


def quadrant_signs(plot: Plot) -> List[LineLabel]:
    """Return the signs of the product of the deviations, one in each quadrant's outer corner."""
    signs = []
    for sign_x in (-1, 1):
        for sign_y in (-1, 1):
            center = plot.at(sign_x * QUADRANT_SIGN_POSITION, sign_y * QUADRANT_SIGN_POSITION)
            signs.append(line_label(center, (Span('+' if sign_x == sign_y else '−'),),
                                    SIGN_SIZE))
    return signs


def scatter_labels(plot: Plot, covariance: Covariance) -> List[LineLabel]:
    """Return the labels of the ellipses over a cloud of example pairs, each on its ellipse."""
    labels = []
    for multiple in SCATTER_STANDARD_DEVIATIONS:
        point = point_in_direction(covariance, multiple, SCATTER_LABEL_DIRECTION)
        labels.append(line_label(plot.at(point.x, point.y), sigma_label_spans(multiple)))
    return labels


def add_plot_title(figure: Figure, plot: Plot, covariance: Covariance) -> None:
    """Add the covariance of the plot's distribution above the plot, such as σxy = 3 cm²."""
    left, top = plot.at(plot.low, plot.high_y)
    right = plot.at(plot.high_x, 0).x
    figure.rich_text((left + right) / 2, top + Y_NAME_OFFSET.y,
                     (Span('σ', 'variable'), Span('xy', 'subscript index'),
                      Span(f' = {covariance.xy:g} cm²')), 'middle', 'strong')


def add_scatter_plot(figure: Figure, plot: Plot, cloud: ExampleCloud, mask: str) -> None:
    """Add one plot of the scatter figure: a cloud of example pairs with its ellipses and signs.

    The plot's title names the covariance of the cloud's distribution. The ellipses' lines have
    gaps for their labels where the mask leaves them.
    """
    left, bottom = plot.at(plot.low, plot.low)
    right, top = plot.at(plot.high_x, plot.high_y)
    figure.path(f'M{left:.1f},{plot.origin.y:.1f} H{right:.1f} '
                f'M{plot.origin.x:.1f},{bottom:.1f} V{top:.1f}', 'zero-line')
    for pair in cloud.pairs():
        point = plot.at(pair.x, pair.y)
        figure.circle(point.x, point.y, PAIR_RADIUS, 'pair')
    for multiple in SCATTER_STANDARD_DEVIATIONS:
        add_masked_path(figure, ellipse_path(plot, cloud.covariance, multiple), 'ring', mask)
    for sign in quadrant_signs(plot):
        add_line_label(figure, sign, 'sign')
    plot_axes(figure, plot)
    add_plot_title(figure, plot, cloud.covariance)
    for label in scatter_labels(plot, cloud.covariance):
        add_line_label(figure, label, 'strong halo')


def scatter_plots(layout: ScatterLayout) -> List[Plot]:
    """Return the plots of a layout of the scatter figure, one for each cloud of SCATTER_CLOUDS."""
    return [OVERVIEW_PLOT, OVERVIEW_PLOT.shifted(layout.second_plot_offset)]


def scatter_title() -> str:
    """Return the scatter figure's accessible name, the same for both of its layouts."""
    covariances = listed([f'{cloud.covariance.xy:g} cm²' for cloud in SCATTER_CLOUDS])
    return f'Position errors drawn from two Gaussian models with covariance {covariances}'


def covariance_scatter_figure(layout: ScatterLayout) -> Figure:
    """Return a layout of the drawing of the two clouds of example pairs."""
    plots = scatter_plots(layout)
    figure = Figure(layout.name, scatter_title(), plots[-1].drawing_bottom(), layout.width,
                    layout.css_class)
    mask = add_gap_mask(figure, [label.box for plot, cloud in zip(plots, SCATTER_CLOUDS)
                                 for label in scatter_labels(plot, cloud.covariance)])
    for plot, cloud in zip(plots, SCATTER_CLOUDS):
        add_scatter_plot(figure, plot, cloud, mask)
    return figure


def scatter_caption() -> str:
    """Return the scatter figure's caption, with the counts and sample covariances of the pairs."""
    same_signs = listed([f'{positive_product_count(cloud.pairs())} pairs in the {cloud.shape} '
                         'cloud' for cloud in SCATTER_CLOUDS])
    sample_covariances = listed([f'{three_decimals(sample_covariance(cloud.pairs()).xy)} cm²'
                                 for cloud in SCATTER_CLOUDS])
    distances = listed([str(multiple) for multiple in SCATTER_STANDARD_DEVIATIONS])
    return (f'Each cloud contains {SCATTER_COUNT} simulated pairs from a model with the variance '
            f'{EXAMPLE_COVARIANCE.xx:g} cm² on each axis. The deviations have the same sign for '
            f'{same_signs}. The sample covariances are about {sample_covariances}. The ellipses '
            f"show distances of {distances} from each model's mean.")


def write_covariance_scatter() -> None:
    """Write the figure of the two clouds of example pairs, in "The covariance of two values"."""
    write_layout_figure('includes/math-covariance-scatter.html',
                        [covariance_scatter_figure(layout) for layout in SCATTER_LAYOUTS],
                        scatter_caption())


class Quaternion(NamedTuple):
    """The four numbers of a rotation, in the order of ROS's geometry_msgs/Quaternion."""

    x: float
    y: float
    z: float
    w: float


def yaw_only_quaternion(yaw: float) -> Quaternion:
    """Return the quaternion of a rotation by yaw degrees about the z axis."""
    half_yaw = math.radians(yaw) / 2
    return Quaternion(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class CirclePlot(NamedTuple):
    """A plot of the numbers w and z of a yaw's quaternion, with the same scale on both axes."""

    center: Point  # where w and z are 0
    pixels_per_unit: float
    axis_reach: float  # how far each axis runs from the center, in the numbers' unit

    def at(self, w: float, z: float) -> Point:
        """Return the drawing's position of the numbers w and z."""
        return Point(self.center.x + w * self.pixels_per_unit,
                     self.center.y - z * self.pixels_per_unit)

    def on_circle(self, degrees: float, radius: float) -> Point:
        """Return the point at an angle from the w axis, counterclockwise, radius px away."""
        return Point(self.center.x + radius * math.cos(math.radians(degrees)),
                     self.center.y - radius * math.sin(math.radians(degrees)))


# The yaw of Remo's IMU on its chassis: imu_rpy = "0 0 -1.5707963" in urdf/remo.urdf.xacro of
# remo_description 2.5.0, in degrees
REMO_IMU_YAW = -90
# The plot of the quaternion's numbers, the radius of the half yaw's arc and where its name stands
QUATERNION_PLOT = CirclePlot(center=Point(150, 142), pixels_per_unit=88, axis_reach=1.32)
HALF_YAW_ARC_RADIUS = 28  # px
HALF_YAW_NAME_RADIUS = 48  # px
QUATERNION_DOT_RADIUS = 4.5  # px
# Where the line with the two angles has its baseline, under the plot
ANGLES_LINE_Y = 284
QUATERNION_DRAWING_HEIGHT = 296


def four_decimals(value: float) -> str:
    """Return a number with four decimals and a minus sign, such as "−0.7071"."""
    return f'{round(value, 4) + 0:.4f}'.replace('-', '−')


def degrees_text(degrees: float) -> str:
    """Return an angle in degrees with a minus sign, such as "−45°"."""
    return f'{minus_signed(degrees)}°'


def add_circle_axes(figure: Figure, plot: CirclePlot) -> None:
    """Add the plot's two axes through its center, with arrowheads, names and the marks at ±1."""
    left, bottom = plot.at(-plot.axis_reach, -plot.axis_reach)
    right, top = plot.at(plot.axis_reach, plot.axis_reach)
    center = plot.center
    figure.path(f'M{left:.1f},{center.y:.1f} H{right:.1f} '
                f'M{center.x:.1f},{bottom:.1f} V{top:.1f}', 'axis')
    figure.path(arrow_head(right, center.y, 1, 0), 'axis-head')
    figure.path(arrow_head(center.x, top, 0, -1), 'axis-head')
    figure.rich_text(right, center.y - 9, (Span('q', 'variable'), Span('w', 'subscript index')),
                     'end')
    figure.rich_text(center.x + 9, top + 6, (Span('q', 'variable'), Span('z', 'subscript index')))
    for unit in (-1, 1):
        on_w_axis = plot.at(unit, 0)
        figure.path(f'M{on_w_axis.x:.1f},{center.y - 4:.1f} v8', 'tick')
        figure.text(on_w_axis.x + unit * 5, center.y + 15, minus_signed(unit), 'note',
                    'start' if unit > 0 else 'end')
        on_z_axis = plot.at(0, unit)
        figure.path(f'M{center.x - 4:.1f},{on_z_axis.y:.1f} h8', 'tick')
        # To the left of the axis: its name stands to the right of its upper end
        figure.text(center.x - 8, on_z_axis.y - unit * 13 + 4, minus_signed(unit), 'note', 'end')


def add_half_yaw_arc(figure: Figure, plot: CirclePlot, half_yaw: float) -> None:
    """Add the arc of half the yaw from the w axis, with an arrowhead and the name ψ/2."""
    start = plot.on_circle(0, HALF_YAW_ARC_RADIUS)
    end = plot.on_circle(half_yaw, HALF_YAW_ARC_RADIUS)
    # The drawing's y points down, so a negative angle turns clockwise on the screen
    sweep = 1 if half_yaw < 0 else 0
    figure.path(f'M{start.x:.1f},{start.y:.1f} A{HALF_YAW_ARC_RADIUS},{HALF_YAW_ARC_RADIUS} '
                f'0 0 {sweep} {end.x:.1f},{end.y:.1f}', 'angle-arc')
    turn = math.copysign(90, half_yaw)
    ahead = plot.on_circle(half_yaw + turn, 1)
    figure.path(arrow_head(end.x, end.y, ahead.x - plot.center.x, ahead.y - plot.center.y, 6),
                'angle-head')
    name = plot.on_circle(half_yaw / 2, HALF_YAW_NAME_RADIUS)
    figure.rich_text(name.x, name.y + BASELINE_SHIFT * VARIABLE_SIZE,
                     (Span('ψ', 'variable'), Span('/2')), 'middle')


def add_quaternion_point(figure: Figure, plot: CirclePlot, quaternion: Quaternion) -> None:
    """Add a yaw's quaternion: its point, its line from the center and its two numbers.

    A dashed line leads from the point to each axis, and the number stands at the axis.
    """
    point = plot.at(quaternion.w, quaternion.z)
    center = plot.center
    figure.path(f'M{point.x:.1f},{center.y:.1f} V{point.y:.1f} H{center.x:.1f}', 'reference')
    figure.text(point.x, center.y - 7, four_decimals(quaternion.w), 'note', 'middle')
    figure.text(center.x - 7, point.y + 4, four_decimals(quaternion.z), 'note', 'end')
    figure.path(f'M{center.x:.1f},{center.y:.1f} L{point.x:.1f},{point.y:.1f}',
                'quaternion-radius')
    figure.circle(point.x, point.y, QUATERNION_DOT_RADIUS, 'estimate-dot')
    figure.rich_text(point.x + 9, point.y + 16, (Span('q', 'vector'),))


def add_opposite_point(figure: Figure, plot: CirclePlot, quaternion: Quaternion) -> None:
    """Add the opposite quaternion, the same rotation: its point and a dashed line to it."""
    point = plot.at(-quaternion.w, -quaternion.z)
    figure.path(f'M{plot.center.x:.1f},{plot.center.y:.1f} L{point.x:.1f},{point.y:.1f}',
                'opposite-radius')
    figure.circle(point.x, point.y, QUATERNION_DOT_RADIUS, 'opposite-dot')
    figure.rich_text(point.x - 9, point.y - 8, (Span('−'), Span('q', 'vector')), 'end')


def quaternion_circle_figure() -> Figure:
    """Return the drawing of the quaternion of Remo's IMU as a point on the unit circle."""
    plot, yaw = QUATERNION_PLOT, REMO_IMU_YAW
    quaternion = yaw_only_quaternion(yaw)
    title = (f'A yaw of {degrees_text(yaw)} as a quaternion: a point on the unit circle at '
             f'{degrees_text(yaw / 2)}, and the opposite point')
    figure = Figure('math-quaternion-circle', title, QUATERNION_DRAWING_HEIGHT, DRAWING_WIDTH)
    figure.circle(plot.center.x, plot.center.y, plot.pixels_per_unit, 'unit-circle')
    add_circle_axes(figure, plot)
    add_opposite_point(figure, plot, quaternion)
    add_half_yaw_arc(figure, plot, yaw / 2)
    add_quaternion_point(figure, plot, quaternion)
    figure.rich_text(DRAWING_WIDTH / 2, ANGLES_LINE_Y,
                     (Span('ψ', 'variable'), Span(f' = {degrees_text(yaw)}, half of it: '),
                      Span('ψ', 'variable'), Span(f'/2 = {degrees_text(yaw / 2)}')), 'middle')
    return figure


def quaternion_circle_caption() -> str:
    """Return the caption of the quaternion figure, with the numbers of Remo's quaternion."""
    yaw = REMO_IMU_YAW
    quaternion = yaw_only_quaternion(yaw)
    return ("The two numbers of a yaw's quaternion that aren't 0, <em>q<sub>w</sub></em> = "
            'cos(<em>ψ</em>/2) and <em>q<sub>z</sub></em> = sin(<em>ψ</em>/2), as a point on the '
            f"unit circle at half the yaw. For the yaw of Remo's IMU, {degrees_text(yaw)}, the "
            f'point <strong>q</strong> lies at {degrees_text(yaw / 2)}: <em>q<sub>w</sub></em> ≈ '
            f'{four_decimals(quaternion.w)} and <em>q<sub>z</sub></em> ≈ '
            f'{four_decimals(quaternion.z)}. The opposite point −<strong>q</strong> is the '
            'same rotation.')


def write_quaternion_circle() -> None:
    """Write the figure of a yaw's quaternion, in "Remo's IMU, turned by −90°"."""
    write_figure('includes/math-quaternion-circle.html', quaternion_circle_figure(),
                 quaternion_circle_caption())


def main() -> None:
    """Write the includes of the two pages, in the order of their sections."""
    write_normal_distribution()
    write_covariance_scatter()
    write_covariance_axes()
    write_covariance_shares()
    write_quaternion_circle()


if __name__ == '__main__':
    main()
