"""Tests for scripts/figures/math_basics_figures.py.

The script writes the figures of the Noise and Covariance page as four includes.

Run from the repository root: python -m unittest discover -s tests
"""

from collections import Counter
import importlib.util
import math
from pathlib import Path
import re
import sys
from types import ModuleType
from typing import Any
from typing import Iterator
from typing import List
from typing import NamedTuple
from typing import Tuple
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tests'))

from figure_script_runs import run_with_an_ascii_locale  # noqa: E402

FIGURES = ROOT / 'scripts' / 'figures'
SCRIPT = FIGURES / 'math_basics_figures.py'
PAGE = ROOT / 'docs' / 'theory' / 'preliminaries' / 'noise-and-covariance.md'
INCLUDE_NAMES = ['math-covariance-axes.html', 'math-covariance-scatter.html',
                 'math-covariance-shares.html', 'math-normal-distribution.html']
# A row of the page's table of the shares inside: the standard deviations, then the shares for
# one, two and three values, such as "| 1 standard deviation | 68.3 % | 39.3 % | 19.9 % |"
SHARES_ROW = re.compile(r'\| (\d) standard deviations? \| ([\d.]+ %) \| ([\d.]+ %) \| '
                        r'([\d.]+ %) \|')
# The drawings' numbers have one decimal, so a point that the script computed lies up to 0.05 px
# away in the drawing
ROUNDING = 0.05  # px
# The rightmost of an ellipse's points in 360 directions, one degree apart, lies this close to
# the ellipse's rightmost point
ELLIPSE_SAMPLING_TOLERANCE = 0.001  # cm
# The criteria of the scatter figure's seeds: the sample covariance of a cloud's pairs lies this
# close to the covariance of their distribution, and with the covariance 0, each quadrant holds
# a quarter of the pairs, up to this many more or fewer
COVARIANCE_TOLERANCE = 0.1  # cm²
QUADRANT_TOLERANCE = 5  # pairs
# The tolerance of the sample variances and covariance of 20 000 pairs, about 2.5 times the
# standard error of the sample variance 4 of that test's matrix
MANY_PAIRS_TOLERANCE = 0.1


def load_script() -> ModuleType:
    """Return the script as a module, without running its main part.

    The script imports board_drawings.py and localization_figures.py from its own folder, which
    Python finds for a script that it runs, and which this function adds for the import.
    """
    sys.path.insert(0, str(FIGURES))
    try:
        specification = importlib.util.spec_from_file_location(SCRIPT.stem, SCRIPT)
        assert specification is not None and specification.loader is not None
        module = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(module)
    finally:
        sys.path.remove(str(FIGURES))
    return module


class SharesRow(NamedTuple):
    """A row of the page's table of the shares inside, each share as the page writes it."""

    standard_deviations: int
    one_value: str
    two_values: str
    three_values: str


def page_shares() -> List[SharesRow]:
    """Return the rows of the page's table of the shares inside, from the top."""
    rows = []
    for line in PAGE.read_text(encoding='utf-8').splitlines():
        row = SHARES_ROW.match(line)
        if row:
            rows.append(SharesRow(int(row[1]), row[2], row[3], row[4]))
    return rows


def share_within_for_three_values(standard_deviations: float) -> float:
    """Return the share of jointly normal triples inside the ellipsoid of this many σ.

    The chi-squared distribution with three degrees of freedom, at the squared distance.
    """
    return (math.erf(standard_deviations / math.sqrt(2))
            - math.sqrt(2 / math.pi) * standard_deviations
            * math.exp(-standard_deviations ** 2 / 2))


def in_centimeters(figures: ModuleType, plot: Any, x: float, y: float) -> Any:
    """Return the deviations of a point of a plot's drawing, in cm."""
    return figures.Deviation((x - plot.origin.x) / plot.pixels_per_cm,
                             (plot.origin.y - y) / plot.pixels_per_cm)


def first_match(pattern: str, text: str) -> 're.Match[str]':
    """Return the first match of pattern in text, and fail when there is none."""
    match = re.search(pattern, text)
    if match is None:
        raise AssertionError(f'{pattern} is not in {text[:80]}')
    return match


def scatter_layout(figures: ModuleType, css_class: str) -> Any:
    """Return the scatter figure's layout with this CSS class."""
    return next(layout for layout in figures.SCATTER_LAYOUTS if layout.css_class == css_class)


def edge_points(box: Any) -> Iterator[Tuple[float, float]]:
    """Yield points along the four edges of a box, 40 on each."""
    for step in range(41):
        share = step / 40
        yield box.x + share * box.width, box.y
        yield box.x + share * box.width, box.y + box.height
        yield box.x, box.y + share * box.height
        yield box.x + box.width, box.y + share * box.height


class TestIncludes(unittest.TestCase):
    """The script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the minus sign and the Greek sigma.
    """

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        written = run_with_an_ascii_locale(SCRIPT.name)
        self.assertEqual(sorted(written), INCLUDE_NAMES)
        for include_name in INCLUDE_NAMES:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {SCRIPT.name} writes')

    def test_each_include_starts_with_the_comment_that_names_the_script(self) -> None:
        for include_name in INCLUDE_NAMES:
            text = (ROOT / 'includes' / include_name).read_text(encoding='utf-8')
            self.assertTrue(text.startswith(
                '<!-- Written by scripts/figures/math_basics_figures.py -->\n'), include_name)

    def test_includes_have_no_id_in_common(self) -> None:
        # One page shows all four figures
        ids: List[str] = []
        for include_name in INCLUDE_NAMES:
            ids += re.findall(r' id="([^"]+)"',
                              (ROOT / 'includes' / include_name).read_text(encoding='utf-8'))
        self.assertEqual(len(ids), len(set(ids)))


class TestCovarianceMath(unittest.TestCase):
    """The eigenvalues, the Mahalanobis distance and the shares inside, from known values."""

    def test_principal_axes_of_the_example_are_8_and_2_cm2_at_45_degrees(self) -> None:
        figures = load_script()
        axes = figures.principal_axes(figures.EXAMPLE_COVARIANCE)
        self.assertAlmostEqual(axes.major, 8)
        self.assertAlmostEqual(axes.minor, 2)
        self.assertAlmostEqual(axes.angle, 45)

    def test_principal_axes_of_a_diagonal_matrix_lie_along_the_x_axis(self) -> None:
        figures = load_script()
        axes = figures.principal_axes(figures.Covariance(4, 0, 1))
        self.assertEqual((axes.major, axes.minor, axes.angle), (4, 1, 0))

    def test_mahalanobis_distance_of_each_semi_axis_end_of_the_example_is_1(self) -> None:
        figures = load_script()
        for end in (figures.Deviation(2, 2), figures.Deviation(1, -1)):
            self.assertAlmostEqual(
                figures.mahalanobis_distance(figures.EXAMPLE_COVARIANCE, end), 1, msg=str(end))

    def test_mahalanobis_distance_counts_each_value_in_its_own_standard_deviation(
            self) -> None:
        figures = load_script()
        covariance = figures.Covariance(4, 0, 1)  # σx = 2, σy = 1
        self.assertAlmostEqual(
            figures.mahalanobis_distance(covariance, figures.Deviation(2, 0)), 1)
        self.assertAlmostEqual(
            figures.mahalanobis_distance(covariance, figures.Deviation(0, 2)), 2)

    def test_example_ellipse_crosses_the_x_axis_at_4_over_root_5_cm(self) -> None:
        figures = load_script()
        crossing = figures.point_in_direction(figures.EXAMPLE_COVARIANCE, 1, 0)
        self.assertAlmostEqual(crossing.x, 4 / math.sqrt(5))
        self.assertAlmostEqual(crossing.y, 0)

    def test_example_ellipse_is_as_wide_as_two_standard_deviations_of_x(self) -> None:
        figures = load_script()
        rightmost = max(figures.point_in_direction(figures.EXAMPLE_COVARIANCE, 1, degrees).x
                        for degrees in range(360))
        self.assertAlmostEqual(rightmost, math.sqrt(figures.EXAMPLE_COVARIANCE.xx),
                               delta=ELLIPSE_SAMPLING_TOLERANCE)

    def test_page_gives_the_example_ellipses_radius_and_half_width_along_x(self) -> None:
        figures = load_script()
        page = PAGE.read_text(encoding='utf-8')
        crossing = figures.point_in_direction(figures.EXAMPLE_COVARIANCE, 1, 0).x
        half_width = math.sqrt(figures.EXAMPLE_COVARIANCE.xx)
        self.assertIn(f'the ellipse reaches $4/\\sqrt{{5}}$ cm = {crossing:.2f} cm', page)
        self.assertIn(f'at $x$ = {half_width:.2f} cm', page)

    def test_shares_for_one_and_two_values_are_those_of_the_pages_table(self) -> None:
        figures = load_script()
        rows = page_shares()
        self.assertEqual([row.standard_deviations for row in rows],
                         list(figures.STANDARD_DEVIATIONS))
        for row in rows:
            multiple = row.standard_deviations
            self.assertEqual(figures.percent(figures.share_within_for_one_value(multiple)),
                             row.one_value)
            self.assertEqual(figures.percent(figures.share_within_for_two_values(multiple)),
                             row.two_values)

    def test_pages_table_gives_the_chi_squared_shares_for_three_values(self) -> None:
        figures = load_script()
        rows = page_shares()
        self.assertEqual(len(rows), len(figures.STANDARD_DEVIATIONS))
        for row in rows:
            self.assertEqual(
                figures.percent(share_within_for_three_values(row.standard_deviations)),
                row.three_values)

    def test_point_in_direction_lies_on_the_ellipse_of_its_standard_deviations(self) -> None:
        figures = load_script()
        for degrees in (0, 55, 135, 250):
            point = figures.point_in_direction(figures.EXAMPLE_COVARIANCE, 2, degrees)
            self.assertAlmostEqual(
                figures.mahalanobis_distance(figures.EXAMPLE_COVARIANCE, point), 2)
            self.assertAlmostEqual(math.degrees(math.atan2(point.y, point.x)) % 360, degrees)


class TestEllipsePath(unittest.TestCase):
    """An ellipse's path has the semi-axes and the turn of the covariance matrix's ellipse."""

    def test_ellipse_path_starts_at_the_end_of_the_major_axis(self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        for multiple in figures.STANDARD_DEVIATIONS:
            path = figures.ellipse_path(plot, figures.EXAMPLE_COVARIANCE, multiple)
            x, y = (float(value) for value in first_match(r'^M([\d.]+),([\d.]+)', path).groups())
            start = in_centimeters(figures, plot, x, y)
            self.assertAlmostEqual(start.x, multiple * 2, delta=ROUNDING)
            self.assertAlmostEqual(start.y, multiple * 2, delta=ROUNDING)

    def test_ellipse_path_has_the_roots_of_the_eigenvalues_as_radii_turned_by_minus_45(
            self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        path = figures.ellipse_path(plot, figures.EXAMPLE_COVARIANCE, 3)
        arcs = re.findall(r'A([\d.]+),([\d.]+) (-?[\d.]+) 0 1', path)
        self.assertEqual(len(arcs), 2)
        for major, minor, turn in arcs:
            self.assertAlmostEqual(float(major), 3 * math.sqrt(8) * plot.pixels_per_cm,
                                   delta=ROUNDING)
            self.assertAlmostEqual(float(minor), 3 * math.sqrt(2) * plot.pixels_per_cm,
                                   delta=ROUNDING)
            # The drawing's y axis points down, so the turn is clockwise in its numbers
            self.assertEqual(float(turn), -45)

    def test_ellipse_path_of_a_matrix_without_covariance_has_the_turn_0_without_a_sign(
            self) -> None:
        figures = load_script()
        path = figures.ellipse_path(figures.OVERVIEW_PLOT, figures.UNCORRELATED_COVARIANCE, 1)
        self.assertEqual(re.findall(r'A[\d.]+,[\d.]+ (\S+) 0 1', path), ['0', '0'])


class TestLabels(unittest.TestCase):
    """A label on an ellipse stands on its ellipse and leaves the neighboring ones free."""

    def test_each_share_label_is_centered_on_its_ellipse(self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        labels = figures.share_labels(plot, figures.EXAMPLE_COVARIANCE)
        for multiple, label in zip(figures.STANDARD_DEVIATIONS, labels):
            center = in_centimeters(figures, plot, label.center.x, label.center.y)
            self.assertAlmostEqual(
                figures.mahalanobis_distance(figures.EXAMPLE_COVARIANCE, center), multiple)

    def test_each_share_label_lies_between_the_ellipses_inside_and_outside_its_own(
            self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        labels = figures.share_labels(plot, figures.EXAMPLE_COVARIANCE)
        for multiple, label in zip(figures.STANDARD_DEVIATIONS, labels):
            for x, y in edge_points(label.box):
                distance = figures.mahalanobis_distance(figures.EXAMPLE_COVARIANCE,
                                                        in_centimeters(figures, plot, x, y))
                self.assertGreater(distance, multiple - 1, f'{multiple} σ at {x:.1f}, {y:.1f}')
                self.assertLess(distance, multiple + 1, f'{multiple} σ at {x:.1f}, {y:.1f}')

    def test_each_share_label_stays_inside_the_plot(self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        left, bottom = plot.at(plot.low, plot.low)
        right, top = plot.at(plot.high_x, plot.high_y)
        for label in figures.share_labels(plot, figures.EXAMPLE_COVARIANCE):
            self.assertGreater(label.box.x, left)
            self.assertLess(label.box.x + label.box.width, right)
            self.assertGreater(label.box.y, top)
            self.assertLess(label.box.y + label.box.height, bottom)

    def test_share_label_names_its_standard_deviations_and_the_share_inside(self) -> None:
        spans = load_script().share_label_spans(2)
        self.assertEqual([span.content for span in spans], ['2', 'σ', ': 86.5 %'])

    def test_shares_figure_leaves_a_gap_in_each_ellipse_for_its_label(self) -> None:
        svg = load_script().covariance_shares_figure().svg()
        self.assertEqual(svg.count('fill="black"'), 3)
        self.assertEqual(svg.count('class="ring" d="M'), 3)
        self.assertEqual(svg.count('mask="url(#math-covariance-shares-gaps)"'), 3)

    def test_label_along_turns_its_text_with_the_line_and_keeps_it_upright(self) -> None:
        figures = load_script()
        for end in (figures.Point(110, 0), figures.Point(-90, 200)):
            figure = figures.Figure('test', 'Test', 200)
            figures.label_along(figure, figures.Point(10, 100), end, '1 cm', True)
            self.assertIn('transform="rotate(-45.0 ', figure.elements[0])

    def test_label_along_puts_its_baseline_above_or_below_a_line(self) -> None:
        figures = load_script()
        start, end = figures.Point(0, 50), figures.Point(100, 50)
        baselines = []
        for above in (True, False):
            figure = figures.Figure('test', 'Test', 200)
            figures.label_along(figure, start, end, '1 cm', above)
            baselines.append(float(first_match(r' y="([\d.]+)"', figure.elements[0])[1]))
        self.assertEqual(baselines, [50 - figures.LABEL_GAP,
                                     50 + figures.LABEL_GAP + figures.CAP_HEIGHT])


class TestExamplePairs(unittest.TestCase):
    """The scatter figure's two clouds of pairs are examples, each made here from a seed."""

    def test_example_pairs_are_the_same_in_every_run(self) -> None:
        for cloud in load_script().SCATTER_CLOUDS:
            self.assertEqual(cloud.pairs(), cloud.pairs(), str(cloud))

    def test_example_pairs_lie_inside_the_plot(self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        for cloud in figures.SCATTER_CLOUDS:
            for pair in cloud.pairs():
                self.assertTrue(plot.low < pair.x < plot.high_x, f'{cloud}: {pair}')
                self.assertTrue(plot.low < pair.y < plot.high_y, f'{cloud}: {pair}')

    def test_no_example_pair_lies_under_a_quadrants_sign(self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        for cloud in figures.SCATTER_CLOUDS:
            pairs = cloud.pairs()
            for sign in figures.quadrant_signs(plot):
                box = sign.box
                for pair in pairs:
                    point = plot.at(pair.x, pair.y)
                    self.assertFalse(box.x - figures.PAIR_RADIUS < point.x
                                     < box.x + box.width + figures.PAIR_RADIUS
                                     and box.y - figures.PAIR_RADIUS < point.y
                                     < box.y + box.height + figures.PAIR_RADIUS,
                                     f'{cloud}: {pair}')

    def test_example_pairs_have_a_sample_covariance_near_the_distributions(self) -> None:
        figures = load_script()
        for cloud in figures.SCATTER_CLOUDS:
            sample = figures.sample_covariance(cloud.pairs())
            self.assertAlmostEqual(sample.xy, cloud.covariance.xy, delta=COVARIANCE_TOLERANCE,
                                   msg=str(cloud))

    def test_example_pairs_without_covariance_fill_each_quadrant_with_about_a_quarter(
            self) -> None:
        figures = load_script()
        cloud = next(cloud for cloud in figures.SCATTER_CLOUDS if cloud.covariance.xy == 0)
        quadrants = Counter((pair.x > 0, pair.y > 0) for pair in cloud.pairs())
        self.assertEqual(len(quadrants), 4)
        for quadrant, count in quadrants.items():
            self.assertAlmostEqual(count, figures.SCATTER_COUNT / 4, delta=QUADRANT_TOLERANCE,
                                   msg=str(quadrant))

    def test_example_pairs_of_many_values_have_the_distributions_covariance(self) -> None:
        figures = load_script()
        covariance = figures.Covariance(4, -1.5, 1)
        sample = figures.sample_covariance(figures.example_pairs(covariance, 1, 20000))
        for name in ('xx', 'xy', 'yy'):
            self.assertAlmostEqual(getattr(sample, name), getattr(covariance, name),
                                   delta=MANY_PAIRS_TOLERANCE, msg=name)

    def test_sample_covariance_of_the_pages_four_readings_is_0_or_4_thirds(self) -> None:
        figures = load_script()
        signs_apart = [figures.Deviation(x, y) for x, y in ((-1, -1), (-1, 1), (1, -1), (1, 1))]
        signs_together = [figures.Deviation(x, y) for x, y in ((-1, -1), (-1, -1), (1, 1),
                                                               (1, 1))]
        self.assertEqual(figures.sample_covariance(signs_apart).xy, 0)
        self.assertAlmostEqual(figures.sample_covariance(signs_together).xy, 4 / 3)

    def test_sample_covariance_of_the_pages_six_pairs_is_positive_with_four_negative_products(
            self) -> None:
        figures = load_script()
        pairs = [figures.Deviation(x, y)
                 for x, y in ((3, 3), (-3, -3), (1, -1), (1, -1), (-1, 1), (-1, 1))]
        self.assertEqual(len(pairs) - figures.positive_product_count(pairs), 4)
        self.assertAlmostEqual(figures.sample_covariance(pairs).xy, 2.8)

    def test_positive_product_count_counts_only_pairs_with_the_same_signs(self) -> None:
        figures = load_script()
        pairs = [figures.Deviation(x, y) for x, y in ((1, 2), (-1, -2), (1, -2), (0, 2))]
        self.assertEqual(figures.positive_product_count(pairs), 2)


class TestScatterLayouts(unittest.TestCase):
    """The scatter figure has its two plots one below the other, or side by side where wide."""

    def test_narrow_scatter_layout_has_the_second_plot_below_the_first_one(self) -> None:
        figures = load_script()
        layout = scatter_layout(figures, figures.NARROW_LAYOUT)
        first, second = figures.scatter_plots(layout)
        second_drawing_top = second.at(0, second.high_y).y - figures.PLOT_TOP
        self.assertEqual(layout.width, figures.DRAWING_WIDTH)
        self.assertEqual(second.origin.x, first.origin.x)
        self.assertGreaterEqual(second_drawing_top, first.drawing_bottom())

    def test_wide_scatter_layout_has_the_plots_side_by_side(self) -> None:
        figures = load_script()
        layout = scatter_layout(figures, figures.WIDE_LAYOUT)
        first, second = figures.scatter_plots(layout)
        second_drawing_left = second.at(second.low, 0).x - figures.PLOT_LEFT
        self.assertEqual(layout.width, 2 * figures.DRAWING_WIDTH)
        self.assertEqual(second.origin.y, first.origin.y)
        self.assertEqual(second_drawing_left, figures.DRAWING_WIDTH)

    def test_each_scatter_layout_is_as_high_as_its_lower_plot(self) -> None:
        figures = load_script()
        for layout in figures.SCATTER_LAYOUTS:
            figure = figures.covariance_scatter_figure(layout)
            self.assertEqual(figure.height, figures.scatter_plots(layout)[-1].drawing_bottom(),
                             layout.name)

    def test_each_scatter_label_is_centered_on_its_ellipse(self) -> None:
        figures = load_script()
        plot = figures.OVERVIEW_PLOT
        for cloud in figures.SCATTER_CLOUDS:
            labels = figures.scatter_labels(plot, cloud.covariance)
            self.assertEqual(len(labels), len(figures.SCATTER_STANDARD_DEVIATIONS))
            for multiple, label in zip(figures.SCATTER_STANDARD_DEVIATIONS, labels):
                center = in_centimeters(figures, plot, label.center.x, label.center.y)
                self.assertAlmostEqual(
                    figures.mahalanobis_distance(cloud.covariance, center), multiple)

    def test_scatter_figure_leaves_a_gap_in_each_ellipse_for_its_label(self) -> None:
        figures = load_script()
        ellipse_count = len(figures.SCATTER_CLOUDS) * len(figures.SCATTER_STANDARD_DEVIATIONS)
        for layout in figures.SCATTER_LAYOUTS:
            svg = figures.covariance_scatter_figure(layout).svg()
            self.assertEqual(svg.count('fill="black"'), ellipse_count, layout.name)
            self.assertEqual(svg.count('class="ring" d="M'), ellipse_count, layout.name)
            self.assertEqual(svg.count(f'mask="url(#{layout.name}-gaps)"'), ellipse_count,
                             layout.name)

    def test_each_scatter_plot_names_the_covariance_of_its_cloud(self) -> None:
        figures = load_script()
        for layout in figures.SCATTER_LAYOUTS:
            svg = figures.covariance_scatter_figure(layout).svg()
            for cloud in figures.SCATTER_CLOUDS:
                self.assertEqual(svg.count(f'> = {cloud.covariance.xy:g} cm²</tspan>'), 1,
                                 f'{layout.name}: {cloud}')


class TestDrawnValues(unittest.TestCase):
    """The drawings show the values that the page's text gives."""

    def test_axes_figure_draws_each_semi_axis_from_the_mean_to_its_eigenvector_end(self) -> None:
        figures = load_script()
        plot = figures.CLOSE_PLOT
        svg = figures.covariance_axes_figure().svg()
        semi_axes = re.findall(r'class="semi-axis" d="M([\d.]+),([\d.]+) L([\d.]+),([\d.]+)"', svg)
        expected_ends = [(2, 2), (1, -1)]  # cm: √8 along (1, 1) and √2 along (1, −1)
        self.assertEqual(len(semi_axes), len(expected_ends))
        for (start_x, start_y, end_x, end_y), expected in zip(semi_axes, expected_ends):
            self.assertEqual((float(start_x), float(start_y)), (plot.origin.x, plot.origin.y))
            end = in_centimeters(figures, plot, float(end_x), float(end_y))
            self.assertAlmostEqual(end.x, expected[0], delta=ROUNDING / plot.pixels_per_cm)
            self.assertAlmostEqual(end.y, expected[1], delta=ROUNDING / plot.pixels_per_cm)

    def test_axes_figure_labels_the_semi_axes_the_angle_and_the_circle(self) -> None:
        svg = load_script().covariance_axes_figure().svg()
        for label in ('2.83 cm', '1.41 cm', '45°', '2.24 cm'):
            self.assertEqual(svg.count(f'>{label}</'), 1, label)

    def test_normal_distribution_figure_ends_each_bracket_at_its_standard_deviations(
            self) -> None:
        figures = load_script()
        svg = figures.normal_distribution_figure().svg()
        brackets = re.findall(
            r'class="share-bracket" d="M([\d.]+),\S+ V\S+ H\S+ M\S+ H([\d.]+) ', svg)
        self.assertEqual(len(brackets), len(figures.STANDARD_DEVIATIONS))
        for multiple, (left, right) in zip(figures.STANDARD_DEVIATIONS, brackets):
            half = multiple * figures.BELL_PIXELS_PER_SIGMA
            self.assertAlmostEqual(float(left), figures.BELL_CENTER_X - half, delta=ROUNDING)
            self.assertAlmostEqual(float(right), figures.BELL_CENTER_X + half, delta=ROUNDING)

    def test_normal_distribution_figure_labels_the_brackets_with_the_shares(self) -> None:
        svg = load_script().normal_distribution_figure().svg()
        for share in ('68.3 %', '95.4 %', '99.7 %'):
            self.assertEqual(svg.count(f'>{share}</text>'), 1, share)

    def test_bell_height_at_one_standard_deviation_is_the_peak_times_e_to_minus_a_half(
            self) -> None:
        figures = load_script()
        self.assertAlmostEqual(figures.bell_height(1), figures.BELL_PEAK * math.exp(-0.5))

    def test_each_scatter_layout_draws_one_dot_for_each_pair_at_its_deviations(self) -> None:
        figures = load_script()
        for layout in figures.SCATTER_LAYOUTS:
            svg = figures.covariance_scatter_figure(layout).svg()
            dots = re.findall(r'<circle class="pair" cx="([\d.]+)" cy="([\d.]+)"', svg)
            positions = [plot.at(pair.x, pair.y)
                         for plot, cloud in zip(figures.scatter_plots(layout),
                                                figures.SCATTER_CLOUDS)
                         for pair in cloud.pairs()]
            self.assertEqual(len(dots), len(positions), layout.name)
            for (x, y), position in zip(dots, positions):
                self.assertAlmostEqual(float(x), position.x, delta=ROUNDING)
                self.assertAlmostEqual(float(y), position.y, delta=ROUNDING)


class TestTexts(unittest.TestCase):
    """The figures' texts are built from the script's values."""

    def test_listed_joins_the_last_value_with_and(self) -> None:
        self.assertEqual(load_script().listed(['1', '2', '3']), '1, 2 and 3')

    def test_listed_gives_a_single_value_alone(self) -> None:
        self.assertEqual(load_script().listed(['1']), '1')

    def test_multiple_of_sigma_writes_one_sigma_without_a_factor(self) -> None:
        figures = load_script()
        self.assertEqual([[span.content for span in figures.multiple_of_sigma(multiple)]
                          for multiple in (-2, -1, 0, 1)],
                         [['−2', 'σ'], ['−', 'σ'], ['0'], ['', 'σ']])

    def test_minus_signed_writes_a_minus_sign(self) -> None:
        self.assertEqual(load_script().minus_signed(-15), '−15')

    def test_one_decimal_writes_a_minus_sign_but_not_before_zero(self) -> None:
        figures = load_script()
        self.assertEqual(figures.one_decimal(-1.26), '−1.3')
        self.assertEqual(figures.one_decimal(-0.009), '0.0')

    def test_normal_distribution_axis_shows_the_noise_in_thousandths(self) -> None:
        svg = load_script().normal_distribution_figure().svg()
        self.assertIn('>−15</text>', svg)
        self.assertIn('> in 10⁻³ rad/s</tspan>', svg)

    def test_bell_band_spans_its_standard_deviations_on_both_sides(self) -> None:
        figures = load_script()
        path = figures.bell_band(2)
        half = 2 * figures.BELL_PIXELS_PER_SIGMA
        self.assertTrue(path.startswith(
            f'M{figures.BELL_CENTER_X - half:.1f},{figures.BELL_BASE_Y} '))
        self.assertTrue(path.endswith(
            f' L{figures.BELL_CENTER_X + half:.1f},{figures.BELL_BASE_Y} z'))


if __name__ == '__main__':
    unittest.main()
