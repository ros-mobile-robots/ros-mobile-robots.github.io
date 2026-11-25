"""Generate the figures of the Math Basics page as inline SVG.

The page is docs/theory/preliminaries/math-basics.md. The script writes these includes, which
follow the site's light and dark colors:

- includes/math-normal-distribution.html, in "The normal distribution": the density of the
  simulated IMU's noise, a normal distribution with the mean 0 and the standard deviation of the
  plugin's gaussianNoise, with the shares within one, two and three standard deviations.
- includes/math-covariance-scatter.html, in "The covariance of two values": example pairs of
  deviations drawn from a normal distribution with a positive covariance, the sign of their
  product in each quadrant, and the ellipses of one and two standard deviations over them.
- includes/math-covariance-axes.html, in "The covariance ellipsoid": the ellipse of one standard
  deviation of the example covariance matrix, its semi-axes along the eigenvectors, and the
  circle that the matrix would give without its covariance.
- includes/math-covariance-shares.html, in "The covariance ellipsoid": the ellipses of one, two
  and three standard deviations of the same matrix, each labeled with the share of the values
  inside it.

Each include holds one figure of one section, and one function writes it. So a figure moves
with its section when the page is split, and the script changes only in this list.

Every drawing is 300 units wide, so that its text is readable on a phone. Explanations are HTML
text in the caption. Figure, the CSS rules, the colors FG and FG_LIGHT and bell() come from
localization_figures.py, and escape(), style_element(), Point and Box from board_drawings.py.

Run from the repository root after changing a figure:

    python3 scripts/figures/math_basics_figures.py
"""

import math
import random
import statistics
from typing import List
from typing import NamedTuple
from typing import Sequence

from board_drawings import Box
from board_drawings import escape
from board_drawings import Point
from board_drawings import style_element
from localization_figures import bell
from localization_figures import DRAWING_RULES
from localization_figures import FG
from localization_figures import FG_LIGHT
from localization_figures import Figure
from localization_figures import figure_element
from localization_figures import LARGE_RULES
from localization_figures import RULES
from localization_figures import Span
from localization_figures import SYMBOL_RULES

DRAWING_WIDTH = 300
# The size of the drawings' text, as in LARGE_RULES, of a variable, as in DRAWING_RULES, and of
# the signs in the quadrants of the example pairs
TEXT_SIZE = 12  # px
VARIABLE_SIZE = 14  # px
SIGN_SIZE = 17  # px
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


# The example covariance matrix of a position estimate (x, y) on the page, in cm²
EXAMPLE_COVARIANCE = Covariance(xx=5.0, xy=3.0, yy=5.0)
# The example pairs of deviations: their number, and a seed for Python's generator with which
# every pair lies inside the plot and off the quadrants' signs, and the pairs' sample covariance
# lies near the distribution's
SCATTER_COUNT = 200
SCATTER_SEED = 150


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


PLOT_LEFT = 34  # the y axis' labels stand to the left of the plot
PLOT_TOP = 26  # the y axis' name stands above the plot
# The plots of the example pairs and of the shares inside the ellipses: the ellipse of three
# standard deviations fits, and its label above it
WIDE_PLOT = Plot(origin=Point(PLOT_LEFT + 7 * 18.5, PLOT_TOP + 7.5 * 18.5),
                 pixels_per_cm=18.5, low=-7, high_x=7, high_y=7.5, tick_spacing=2)
# The plot of the ellipse of one standard deviation and its axes, enlarged
CLOSE_PLOT = Plot(origin=Point(PLOT_LEFT + 3 * 42, PLOT_TOP + 3 * 42),
                  pixels_per_cm=42, low=-3, high_x=3, high_y=3, tick_spacing=1)
# The direction from the mean, counterclockwise from the x axis, in which each ellipse of the
# shares figure carries its label: there, the next ellipses lie far enough away from the label
SHARE_LABEL_DIRECTION = 55  # degrees
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
    """Return how many standard deviations a deviation lies from the mean, in its direction."""
    determinant = covariance.xx * covariance.yy - covariance.xy ** 2
    squared = (covariance.yy * deviation.x ** 2 - 2 * covariance.xy * deviation.x * deviation.y
               + covariance.xx * deviation.y ** 2) / determinant
    return math.sqrt(squared)


def share_within_for_one_value(standard_deviations: float) -> float:
    """Return the share of a normally distributed value within this many standard deviations."""
    return math.erf(standard_deviations / math.sqrt(2))


def share_within_for_two_values(standard_deviations: float) -> float:
    """Return the share of normally distributed pairs inside the ellipse of this many σ.

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
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{INCLUDE_COMMENT}\n{style_element(FIGURE_RULES)}\n'
                      f'{figure_element([figure.svg()], caption, FIGURE_CLASSES)}\n')


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
    figure = Figure('math-normal-distribution', 'The bell curve of the density of the simulated '
                    "IMU's noise n, with the mean 0 and the standard deviation σ = "
                    f'{SIMULATED_IMU_NOISE_SIGMA} rad/s. Within one, two and three standard '
                    f'deviations of the mean lie {listed(shares)} of the values', height,
                    DRAWING_WIDTH)
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
    figure.rich_text(4, 14, (Span('y', 'variable'), Span(' − '), Span('μ', 'variable'),
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
    arc = f'A{major:.1f},{minor:.1f} {-axes.angle:g} 0 1'
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


def share_label_spans(standard_deviations: int) -> Sequence[Span]:
    """Return the label of an ellipse: its standard deviations and the share inside it."""
    share = percent(share_within_for_two_values(standard_deviations))
    return (Span(str(standard_deviations)), Span('σ', 'variable'), Span(f': {share}'))


def share_labels(plot: Plot, covariance: Covariance) -> List[LineLabel]:
    """Return the labels of the ellipses of 1, 2 and 3 σ, each on its ellipse."""
    labels = []
    for multiple in STANDARD_DEVIATIONS:
        point = point_in_direction(covariance, multiple, SHARE_LABEL_DIRECTION)
        labels.append(line_label(plot.at(point.x, point.y), share_label_spans(multiple)))
    return labels


def covariance_shares_figure() -> Figure:
    """Return the drawing of the ellipses of 1, 2 and 3 σ and the share inside each."""
    covariance, plot = EXAMPLE_COVARIANCE, WIDE_PLOT
    shares = listed([percent(share_within_for_two_values(multiple))
                     for multiple in STANDARD_DEVIATIONS])
    figure = Figure('math-covariance-shares', 'Three ellipses around the mean of a position '
                    'estimate, one inside the other, for the covariance matrix with the '
                    f'variances {covariance.xx:g} cm² and the covariance {covariance.xy:g} cm². '
                    'Inside the ellipses of one, two and three standard deviations lie '
                    f'{shares} of the positions', round(plot.at(0, plot.low).y) + 44,
                    DRAWING_WIDTH)
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
        f'share of the positions inside it: {shares}, the shares of the table for two values.')


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
    figure = Figure('math-covariance-axes', 'The ellipse of one standard deviation of the '
                    f'covariance matrix with the variances {covariance.xx:g} cm² and the '
                    f'covariance {covariance.xy:g} cm². It is turned by {axes.angle:g}° and has '
                    f'the semi-axes {length_text(major)} and {length_text(minor)}. A dashed '
                    'circle with the radius '
                    f'{length_text(math.sqrt(covariance.xx))} shows the ellipse without the '
                    'covariance', round(plot.at(0, plot.low).y) + 44, DRAWING_WIDTH)
    across = axes.angle - 90
    major_end = plot.at(major * math.cos(math.radians(axes.angle)),
                        major * math.sin(math.radians(axes.angle)))
    minor_end = plot.at(minor * math.cos(math.radians(across)),
                        minor * math.sin(math.radians(across)))
    # The two labels on lines stand opposite the minor semi-axis
    ring_point = point_in_direction(covariance, 1, across + 180)
    circle_point = point_in_direction(without_covariance, 1, across + 180)
    labels = (line_label(plot.at(ring_point.x, ring_point.y),
                         (Span('1'), Span('σ', 'variable'))),
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


def normal_values(seed: int, count: int) -> List[float]:
    """Return count values of a standard normal distribution, the same for the same seed.

    Box and Muller's transform of the uniform values of Python's generator, whose sequence for
    a seed doesn't change between Python versions.
    """
    generator = random.Random(seed)
    values = []
    for _ in range(count):
        radius = math.sqrt(-2.0 * math.log(1.0 - generator.random()))
        values.append(radius * math.cos(2.0 * math.pi * generator.random()))
    return values


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


def covariance_scatter_figure(pairs: Sequence[Deviation]) -> Figure:
    """Return the drawing of the example pairs, the quadrants' signs and two ellipses."""
    covariance, plot = EXAMPLE_COVARIANCE, WIDE_PLOT
    positive = positive_product_count(pairs)
    figure = Figure('math-covariance-scatter', f'{len(pairs)} example pairs of deviations of x '
                    'and y as dots. The cloud is stretched along the line on which both '
                    'deviations have the same sign. In the two quadrants where they have the '
                    f'same sign, the product of the deviations is positive: {positive} of the '
                    f'{len(pairs)} pairs lie there. Over the cloud lie the ellipses of one and '
                    'two standard deviations', round(plot.at(0, plot.low).y) + 44,
                    DRAWING_WIDTH)
    left, bottom = plot.at(plot.low, plot.low)
    right, top = plot.at(plot.high_x, plot.high_y)
    figure.path(f'M{left:.1f},{plot.origin.y:.1f} H{right:.1f} '
                f'M{plot.origin.x:.1f},{bottom:.1f} V{top:.1f}', 'zero-line')
    for pair in pairs:
        point = plot.at(pair.x, pair.y)
        figure.circle(point.x, point.y, PAIR_RADIUS, 'pair')
    for multiple in STANDARD_DEVIATIONS[:2]:
        figure.path(ellipse_path(plot, covariance, multiple), 'ring')
    for sign in quadrant_signs(plot):
        add_line_label(figure, sign, 'sign')
    plot_axes(figure, plot)
    return figure


def write_covariance_scatter() -> None:
    """Write the figure of the example pairs, in "The covariance of two values"."""
    covariance = EXAMPLE_COVARIANCE
    pairs = example_pairs(covariance, SCATTER_SEED, SCATTER_COUNT)
    sample = sample_covariance(pairs)
    write_figure(
        'includes/math-covariance-scatter.html', covariance_scatter_figure(pairs),
        f'{len(pairs)} example pairs of deviations from the means, in cm, drawn from a normal '
        f'distribution with the variances {covariance.xx:g} cm² and the covariance '
        f'{covariance.xy:g} cm². The sign in each quadrant is the sign of the product '
        f'(<em>x</em> − <em>μ<sub>x</sub></em>)(<em>y</em> − <em>μ<sub>y</sub></em>): '
        f'{positive_product_count(pairs)} of the {len(pairs)} products are positive. The '
        f"pairs' sample covariance is <em>s<sub>xy</sub></em> = {sample.xy:.1f} cm², and their "
        f'sample variances are {sample.xx:.1f} and {sample.yy:.1f} cm². The inner ellipse is the '
        'one of one standard deviation, the outer one of two.')


def main() -> None:
    """Write the includes of the Math Basics page, in the order of its sections."""
    write_normal_distribution()
    write_covariance_scatter()
    write_covariance_axes()
    write_covariance_shares()


if __name__ == '__main__':
    main()
