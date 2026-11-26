"""Generate the figures about the IMU's bias and noise at rest as inline SVG.

The pages are docs/packages/imu/driver-bno055.md, section "Bias and noise at rest", and
docs/packages/imu/noise-at-rest-tests.md, the walkthrough of the tests of
diffbot_bringup/scripts/imu_noise_at_rest.py in the diffbot repository. The script writes these
includes, which follow the site's light and dark colors:

- includes/imu-noise-trace.html: yaw-rate readings at rest against time, with zero, the mean and
  the mean plus and minus one standard deviation, and a legend of the marks. The readings are a
  synthetic example, made here from a seed: a gyroscope with a bias large enough to see, on the
  BNO055's output steps. The dots are the first second, the lines come from all 180 s. The
  figure has a narrow layout and a wide one for a text column with room for it.
- includes/imu-noise-three-readings.html: three readings one output step apart, their mean and
  their deviations from it. The worked example of the sample variance.
- includes/imu-noise-test-variance.html: the four values 1, 2, 3 and 4 of the test of the sample
  variance, their mean and their deviations from it.
- includes/imu-noise-test-intervals.html: five header stamps with one interval of 35 ms.
- includes/imu-noise-test-stamps-back.html: five messages in the order of the recording, of
  which one repeats the stamp before and one has an earlier stamp.
  The two stamp figures' wide layouts are their narrow ones with a longer time axis.
- includes/imu-noise-test-measured.html: three messages of which the middle one marks its
  angular velocity as not measured.
- includes/imu-noise-test-gravity.html: the mean acceleration (2, 3, 6) m/s² of a tilted IMU and
  its length 7 m/s².
- includes/imu-noise-test-flow.html: the tool's three steps from a bag file to the report, the
  test classes that call each step's functions, the exit with one line and status 1, and the
  command tests, which span all three steps. Its wide layout puts the three steps side by side.

Every drawing is 300 units wide and carries short labels only, so that its text is readable on a
phone. The trace figure, the flow figure and the two stamp figures also have a wide layout
of 540 units, which the CSS of LAYOUT_RULES in
localization_figures.py shows where the text column has room for it. Explanations are HTML text
below a drawing. Figure, the CSS rules and the helpers come from localization_figures.py.

The tests' values are copied here, each with the name of its constant in
diffbot_bringup/test/test_imu_noise_at_rest.py, because this repository can't import the tests.
So are the names of its test classes and their numbers of tests.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_noise_figures.py
"""

import math
import random
import statistics
from typing import Callable
from typing import List
from typing import NamedTuple
from typing import Sequence

from board_drawings import Drawing
from board_drawings import style_element
from localization_figures import arrow_head
from localization_figures import DRAWING_RULES
from localization_figures import Figure
from localization_figures import figure_element
from localization_figures import LARGE_RULES
from localization_figures import LAYOUT_RULES
from localization_figures import LAYOUT_SPACE
from localization_figures import NARROW_LAYOUT
from localization_figures import RULES
from localization_figures import Span
from localization_figures import SYMBOL_RULES
from localization_figures import WIDE_LAYOUT
from localization_figures import WIDE_WIDTH

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
BG = 'var(--md-default-bg-color)'

DRAWING_WIDTH = 300
CLASS_NAME_FONT_SIZE = 9.5  # px, of the test class names in the flow figure
NOISE_RULES = (
    ('.lv-noise .lv-fig .level', f'fill:none;stroke:{FG_LIGHT};stroke-width:.7;'
                                 'stroke-dasharray:1.5 3'),
    ('.lv-noise .lv-fig .zero-line', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-noise .lv-fig .spread-band', 'fill:var(--lv-imu);fill-opacity:.14;stroke:none'),
    ('.lv-noise .lv-fig .mean-line', 'fill:none;stroke:var(--lv-imu);stroke-width:2.4'),
    ('.lv-noise .lv-fig .spread-line', 'fill:none;stroke:var(--lv-imu);stroke-width:1.6;'
                                       'stroke-dasharray:5 3'),
    ('.lv-noise .lv-fig .reading', f'fill:{FG};fill-opacity:.85;stroke:none'),
    ('.lv-noise .lv-fig .deviation-line', 'fill:none;stroke:var(--lv-imu);stroke-width:1.6'),
    ('.lv-noise .lv-fig .deviation-head', 'fill:var(--lv-imu);stroke:none'),
    ('.lv-noise .lv-fig .step-line', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.5'),
    ('.lv-noise .lv-fig .step-bad', 'fill:none;stroke:var(--lv-bad);stroke-width:2;'
                                    'stroke-dasharray:4 2.5'),
    ('.lv-noise .lv-fig .long-interval', 'fill:none;stroke:var(--lv-bad);stroke-width:1.6'),
    ('.lv-noise .lv-fig .box-edge', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;'
                                    'stroke-dasharray:3 3'),
    ('.lv-noise .lv-fig .gravity-line', 'fill:none;stroke:var(--lv-imu);stroke-width:2.4'),
    ('.lv-noise .lv-fig .gravity-head', 'fill:var(--lv-imu);stroke:none'),
    ('.lv-noise .lv-fig .class-name', 'font-family:var(--md-code-font-family,"Roboto Mono"),'
                                      f'monospace;font-size:{CLASS_NAME_FONT_SIZE}px'),
    ('.lv-noise .lv-fig .flow-line', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-noise .lv-fig .flow-head', f'fill:{FG};stroke:none'),
    ('.lv-noise .lv-fig .refused-line', 'fill:none;stroke:var(--lv-bad);stroke-width:1.6;'
                                        'stroke-dasharray:4 2.5'),
    ('.lv-noise .lv-fig .refused-head', 'fill:var(--lv-bad);stroke:none'),
    ('.lv-noise .lv-fig .command-span', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
)
FIGURE_RULES = RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES + NOISE_RULES
FIGURE_CLASSES = 'lv-large lv-drawing lv-symbols lv-noise'
INCLUDE_COMMENT = '<!-- Written by scripts/figures/imu_noise_figures.py -->'

# The BNO055's output step of the angular velocity: 900 LSB per rad/s
GYROSCOPE_LSB_PER_RAD_S = 900
GYROSCOPE_STEP = 1 / GYROSCOPE_LSB_PER_RAD_S  # rad/s
GYROSCOPE_STEP_TEXT = f'1/{GYROSCOPE_LSB_PER_RAD_S} rad/s'
SAMPLE_RATE = 50  # Hz, the firmware's

# The synthetic recording of the trace figure: 180 s at rest. Its bias is about 30 times Remo's,
# so that the mean's line stands clear of the zero line
TRACE_SEED = 178
TRACE_DURATION = 180  # s
TRACE_BIAS = 0.0015  # rad/s
TRACE_NOISE = 0.0007  # rad/s, the standard deviation before the readings snap to output steps
TRACE_SHOWN_DURATION = 1  # s, the readings that the figure shows as dots
TRACE_LEVELS = (-1, 0, 1, 2, 3, 4)  # the output steps that the figure's y axis shows
TRACE_PLOT_LEFT = 52
TRACE_PLOT_TOP = 24
TRACE_PIXELS_PER_STEP = 26
TRACE_LEGEND_ROW_HEIGHT = 18
TRACE_LEGEND_SWATCH_WIDTH = 24

# test_imu_noise_at_rest.py's values
YAW_RATES_ONE_STEP_APART = (0, 1, 2)  # in output steps
# test_axis_statistics_gives_the_sample_variance
VARIANCE_TEST_VALUES = (1.0, 2.0, 3.0, 4.0)
STAMPS_WITH_ONE_LONG_INTERVAL = (0, 20, 40, 75, 95)  # ms after the first stamp
STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK = (0, 20, 20, 10, 80)  # ms after the first stamp
LONG_INTERVAL = 30  # ms
# test_imu_statistics_takes_a_field_only_from_the_samples_that_measured_it: the yaw rates, None
# for the sample without the angular velocity
MEASURED_YAW_RATES = (0.2, None, 0.4)  # rad/s


class Point(NamedTuple):
    """A point or a vector in a frame with x, y and z."""

    x: float
    y: float
    z: float


class DrawingPoint(NamedTuple):
    """A position in a drawing, in SVG coordinates."""

    x: float
    y: float


class AxisLabel(NamedTuple):
    """The end of a drawn axis, its name, and where the name stands next to the end."""

    end: Point
    name: str
    offset: DrawingPoint


class TraceLayout(NamedTuple):
    """One of the trace figure's two layouts: its width, its plot and its legend."""

    name: str  # the drawing's name, unique on the page
    css_class: str  # NARROW_LAYOUT or WIDE_LAYOUT, by which the CSS shows one layout
    width: int
    plot_right: float  # the plot's right edge. Its left edge is TRACE_PLOT_LEFT
    tick_tenths: int  # tenths of a second between two ticks of the time axis
    legend_columns: int


class PlotLayout(NamedTuple):
    """One of the two layouts of a stamp figure: the wide one has its x positions stretched."""

    name_end: str  # added to the drawing's name, which is unique on the page
    css_class: str  # NARROW_LAYOUT or WIDE_LAYOUT, by which the CSS shows one layout
    stretch: float  # of the x positions: a longer time axis, with marks and text as they are


PLOT_LAYOUTS = (
    PlotLayout(name_end='', css_class=NARROW_LAYOUT, stretch=1),
    PlotLayout(name_end='-wide', css_class=WIDE_LAYOUT, stretch=WIDE_WIDTH / DRAWING_WIDTH),
)


TRACE_LAYOUTS = (
    TraceLayout(name='imu-noise-trace', css_class=NARROW_LAYOUT, width=DRAWING_WIDTH,
                plot_right=182, tick_tenths=5, legend_columns=1),
    TraceLayout(name='imu-noise-trace-wide', css_class=WIDE_LAYOUT, width=WIDE_WIDTH,
                plot_right=412, tick_tenths=2, legend_columns=2),
)


# test_imu_statistics_gives_gravity_as_the_length_of_the_mean_acceleration: the mean of
# (1, 3, 5) and (3, 3, 7)
TILTED_MEAN_ACCELERATION = Point(2.0, 3.0, 6.0)  # m/s^2


class ClassOfTests(NamedTuple):
    """A test class of test_imu_noise_at_rest.py and its number of tests."""

    name: str
    test_count: int


class ToolStep(NamedTuple):
    """One of the tool's three steps, the function for it, and the test classes for it."""

    label: str  # the step's label in the list of the steps on the tests page
    function: str  # the function that main() calls for the step
    test_classes: Sequence[ClassOfTests]  # the classes that call the step's functions


# The steps in the order in which main() calls their functions. Each step's test classes are in
# the order of the list "The test classes" on the tests page
TOOL_STEPS = (
    ToolStep('Read and check', 'read_samples()', (
        ClassOfTests('TestMeasuredVector', 2), ClassOfTests('TestToSample', 2),
        ClassOfTests('TestTopicChecks', 8), ClassOfTests('TestSampleChecks', 10))),
    ToolStep('Calculate', 'imu_statistics()', (
        ClassOfTests('TestAxisStatistics', 8), ClassOfTests('TestVectorStatistics', 1),
        ClassOfTests('TestFieldStatistics', 2), ClassOfTests('TestStampStatistics', 10),
        ClassOfTests('TestImuStatistics', 5))),
    ToolStep('Print', 'format_report()', (
        ClassOfTests('TestFormatDuration', 3), ClassOfTests('TestFormatLongestInterval', 2),
        ClassOfTests('TestFormatTable', 1), ClassOfTests('TestFormatReport', 5))),
)
# The class whose tests run the script as a command, most of them on a bag that the test writes:
# the tests that compare a report, and the tests of an input that the tool refuses
COMMAND_TEST_CLASS = 'TestToolOnBags'
COMMAND_TESTS_WITH_A_REPORT = 7
COMMAND_TESTS_WITH_ONE_LINE = 16


def listed(values: Sequence[str]) -> str:
    """Return values as a list in words, such as "1, 2, 3 and 4"."""
    return ', '.join(values[:-1]) + ' and ' + values[-1] if len(values) > 1 else values[0]


NUMBER_WORDS = ('zero', 'one', 'two', 'three', 'four', 'five')
ORDINAL_WORDS = ('first', 'second', 'third', 'fourth', 'fifth')


def steps_in_words(steps: int) -> str:
    """Return a number of output steps in words, such as "two output steps", or "zero"."""
    if steps == 0:
        return NUMBER_WORDS[0]
    return f'{NUMBER_WORDS[steps]} output step' + ('' if steps == 1 else 's')


def signed_value(value: float) -> str:
    """Return a value with its sign, with a minus sign instead of a hyphen."""
    return f'{value:+g}'.replace('-', '−')


def write_figure(path: str, figure: Figure, caption: str) -> None:
    """Write an include with one drawing and its caption, in this script's style.

    The include starts with a comment that names this script.
    """
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{INCLUDE_COMMENT}\n{style_element(FIGURE_RULES)}\n'
                      f'{figure_element([figure.svg()], caption, FIGURE_CLASSES)}\n')


def write_layout_figure(path: str, layouts: Sequence[Figure], caption: str) -> None:
    """Write an include with a drawing's narrow and wide layout and its caption.

    The include starts with a comment that names this script. The figure stands in a block of
    the class LAYOUT_SPACE, whose width decides which layout shows.
    """
    figure = figure_element([layout.svg() for layout in layouts], caption,
                            f'{FIGURE_CLASSES} lv-layouts')
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{INCLUDE_COMMENT}\n{style_element(FIGURE_RULES + LAYOUT_RULES)}\n'
                      f'<div class="{LAYOUT_SPACE}">\n{figure}\n</div>\n')


class Trace(NamedTuple):
    """A recording of yaw-rate readings at rest and its statistics."""

    readings: Sequence[float]  # rad/s, one every 1 / SAMPLE_RATE seconds
    mean: float  # rad/s
    standard_deviation: float  # rad/s, the root of the sample variance
    distinct_value_count: int


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


def synthetic_trace() -> Trace:
    """Return the synthetic recording: a bias plus noise, on the gyroscope's output steps."""
    readings = [round((TRACE_BIAS + TRACE_NOISE * value) / GYROSCOPE_STEP) * GYROSCOPE_STEP
                for value in normal_values(TRACE_SEED, TRACE_DURATION * SAMPLE_RATE)]
    return Trace(readings, statistics.fmean(readings), statistics.stdev(readings),
                 len(set(readings)))


def signed_yaw_rate(value: float) -> str:
    """Return a value in rad/s with five decimals, its sign and a typographic minus."""
    return f'{value:+.5f}'.replace('-', '−')


def trace_y(yaw_rate: float) -> float:
    """Return the y position of a yaw rate in the trace figure."""
    top_rate = (TRACE_LEVELS[-1] + 0.5) * GYROSCOPE_STEP
    return TRACE_PLOT_TOP + (top_rate - yaw_rate) / GYROSCOPE_STEP * TRACE_PIXELS_PER_STEP


def trace_x(seconds: float, layout: TraceLayout) -> float:
    """Return the x position of a time in a layout of the trace figure."""
    return (TRACE_PLOT_LEFT
            + seconds / TRACE_SHOWN_DURATION * (layout.plot_right - TRACE_PLOT_LEFT))


class LegendEntry(NamedTuple):
    """An entry of the trace figure's legend: the function that draws its mark, and its text."""

    # Gets the figure, the mark's left edge and the baseline of the entry's text
    draw_mark: Callable[[Figure, float, float], None]
    label: str


def reading_mark(figure: Figure, x: float, y: float) -> None:
    """Add the legend's mark of a reading: a dot, larger than the plot's dots so that it shows."""
    figure.circle(x + TRACE_LEGEND_SWATCH_WIDTH / 2, y - 4, 2, 'reading')


def mean_mark(figure: Figure, x: float, y: float) -> None:
    """Add the legend's mark of the mean: its solid line."""
    figure.path(f'M{x:.1f},{y - 4} H{x + TRACE_LEGEND_SWATCH_WIDTH:.1f}', 'mean-line')


def spread_mark(figure: Figure, x: float, y: float) -> None:
    """Add the legend's mark of the mean plus and minus s: the band between its dashed lines."""
    end_x = x + TRACE_LEGEND_SWATCH_WIDTH
    figure.rect(x, y - 9, TRACE_LEGEND_SWATCH_WIDTH, 10, 'spread-band')
    figure.path(f'M{x:.1f},{y - 9} H{end_x:.1f} M{x:.1f},{y + 1} H{end_x:.1f}', 'spread-line')


def level_mark(figure: Figure, x: float, y: float) -> None:
    """Add the legend's mark of the output steps: a dotted line."""
    figure.path(f'M{x:.1f},{y - 4} H{x + TRACE_LEGEND_SWATCH_WIDTH:.1f}', 'level')


TRACE_LEGEND = (
    LegendEntry(reading_mark, f'reading, one every {1000 // SAMPLE_RATE} ms'),
    LegendEntry(mean_mark, 'mean: its offset from zero is the bias'),
    LegendEntry(spread_mark, 'mean ± s: the noise'),
    LegendEntry(level_mark, f'output steps, {GYROSCOPE_STEP_TEXT} apart'),
)


def trace_legend(figure: Figure, top: float, layout: TraceLayout) -> float:
    """Add the legend of the trace figure's marks, in the layout's columns, from top.

    Return the y below the legend's last row.
    """
    column_width = (layout.width - 8) / layout.legend_columns
    for index, entry in enumerate(TRACE_LEGEND):
        x = 4 + index % layout.legend_columns * column_width
        y = top + index // layout.legend_columns * TRACE_LEGEND_ROW_HEIGHT
        entry.draw_mark(figure, x, y)
        figure.text(x + TRACE_LEGEND_SWATCH_WIDTH + 6, y, entry.label)
    rows = math.ceil(len(TRACE_LEGEND) / layout.legend_columns)
    return top + (rows - 1) * TRACE_LEGEND_ROW_HEIGHT + 6


def trace_figure(trace: Trace, layout: TraceLayout) -> Figure:
    """Return a layout of the drawing of the first second's readings and their statistics."""
    bottom = trace_y((TRACE_LEVELS[0] - 0.5) * GYROSCOPE_STEP)
    figure = Figure(layout.name, 'Yaw-rate readings of a gyroscope at rest against time. '
                    'The readings lie on output steps. Their mean lies above zero: the bias. '
                    'A band of one standard deviation around the mean shows their spread',
                    0, layout.width, layout.css_class)
    plot_right = layout.plot_right
    figure.text(4, 13, 'angular_velocity.z in rad/s', 'mono')
    for level in TRACE_LEVELS:
        y = trace_y(level * GYROSCOPE_STEP)
        figure.path(f'M{TRACE_PLOT_LEFT},{y:.1f} H{plot_right}', 'level')
        label = '0' if level == 0 else f'{level * GYROSCOPE_STEP:+.4f}'.replace('-', '−')
        figure.text(TRACE_PLOT_LEFT - 5, y + 4, label, 'note', 'end')
    upper, lower = (trace_y(trace.mean + sign * trace.standard_deviation) for sign in (1, -1))
    figure.rect(TRACE_PLOT_LEFT, upper, plot_right - TRACE_PLOT_LEFT, lower - upper,
                'spread-band')
    figure.path(f'M{TRACE_PLOT_LEFT},{trace_y(0):.1f} H{plot_right}', 'zero-line')
    lines = ((trace.mean + trace.standard_deviation, 'spread-line', 'mean + s'),
             (trace.mean, 'mean-line', 'mean'),
             (trace.mean - trace.standard_deviation, 'spread-line', 'mean − s'))
    for yaw_rate, css_class, label in lines:
        y = trace_y(yaw_rate)
        figure.path(f'M{TRACE_PLOT_LEFT},{y:.1f} H{plot_right}', css_class)
        figure.text(plot_right + 5, y + 4, label, 'strong' if label == 'mean' else '')
        figure.text(layout.width - 2, y + 4, f'{yaw_rate:+.4f}'.replace('-', '−'),
                    'strong' if label == 'mean' else '', 'end')
    figure.text(plot_right + 5, trace_y(0) + 4, 'zero')
    figure.text(layout.width - 2, trace_y(0) + 4, '0', '', 'end')
    for index, reading in enumerate(trace.readings[:TRACE_SHOWN_DURATION * SAMPLE_RATE]):
        figure.circle(trace_x(index / SAMPLE_RATE, layout), trace_y(reading), 1.3, 'reading')
    figure.path(f'M{TRACE_PLOT_LEFT},{TRACE_PLOT_TOP} V{bottom:.1f} H{plot_right}', 'axis')
    for tenths in range(0, TRACE_SHOWN_DURATION * 10 + 1, layout.tick_tenths):
        x = trace_x(tenths / 10, layout)
        figure.path(f'M{x:.1f},{bottom:.1f} v4', 'tick')
        figure.text(x, bottom + 17, f'{tenths / 10:g}', 'note', 'middle')
    figure.text((TRACE_PLOT_LEFT + plot_right) / 2, bottom + 36,
                'seconds after the first header stamp', 'note', 'middle')
    legend_bottom = trace_legend(figure, bottom + 62, layout)
    figure.height = int(legend_bottom) + 4
    return figure


def write_trace() -> None:
    """Write the trace figure in both layouts, with the recording's statistics in its caption."""
    trace = synthetic_trace()
    shown_count = TRACE_SHOWN_DURATION * SAMPLE_RATE
    caption = (
        '<strong>A synthetic example, not a recording of a robot.</strong> A gyroscope at rest '
        f'that reads in steps of {GYROSCOPE_STEP_TEXT}, like the BNO055. '
        f'The dots are the {shown_count} readings of the first second. The lines come from all '
        f'{len(trace.readings):,} readings of {TRACE_DURATION} s: '
        f'the mean {signed_yaw_rate(trace.mean)} rad/s is the bias, '
        f'and the standard deviation <em>s</em> = {trace.standard_deviation:.5f} rad/s is the '
        f'noise. The readings take {trace.distinct_value_count} distinct values.')
    write_layout_figure('includes/imu-noise-trace.html',
                        [trace_figure(trace, layout) for layout in TRACE_LAYOUTS], caption)


def deviation_arrow(figure: Figure, mean_x: float, x: float, y: float, label: str) -> None:
    """Add an arrow from the mean's line to a reading's x position, with the deviation above."""
    direction = 1 if x > mean_x else -1
    figure.path(f'M{mean_x:.1f},{y} H{x - direction * 6:.1f}', 'deviation-line')
    figure.path(arrow_head(x - direction, y, direction, 0), 'deviation-head')
    figure.text((mean_x + x) / 2, y - 6, label, 'strong', 'middle')


def three_readings_figure() -> Figure:
    """Return the drawing of three readings one step apart, their mean and deviations."""
    readings = listed([steps_in_words(steps) for steps in YAW_RATES_ONE_STEP_APART])
    mean_steps = round(statistics.fmean(YAW_RATES_ONE_STEP_APART))
    figure = Figure('imu-noise-three-readings', f'Three yaw-rate readings on an axis: {readings}. '
                    f'Their mean is {NUMBER_WORDS[mean_steps]} step. The outer readings deviate '
                    'from the mean by one step each', 126, DRAWING_WIDTH)
    axis_y, first_x, pixels_per_step = 78, 62, 88
    positions = [first_x + steps * pixels_per_step for steps in YAW_RATES_ONE_STEP_APART]
    mean_x = sum(positions) / len(positions)
    figure.path(f'M{first_x - 40},{axis_y} H{positions[-1] + 40}', 'axis')
    figure.path(f'M{mean_x:.1f},{axis_y - 44} V{axis_y + 8}', 'mean-line')
    figure.text(mean_x, axis_y - 50, 'mean', 'strong', 'middle')
    for x, deviation in ((positions[0], '−Δ'), (positions[-1], '+Δ')):
        deviation_arrow(figure, mean_x, x, axis_y - 24, deviation)
    for x, steps in zip(positions, YAW_RATES_ONE_STEP_APART):
        figure.circle(x, axis_y, 5.5, 'imu-dot')
        label = {0: '0', 1: 'Δ'}.get(steps, f'{steps}Δ')
        figure.text(x, axis_y + 22, label, 'strong', 'middle')
        figure.text(x, axis_y + 38, f'{steps * GYROSCOPE_STEP:.5f}', 'note', 'middle')
    figure.text(4, 13, 'angular_velocity.z in rad/s', 'mono')
    return figure


def write_three_readings() -> None:
    """Write the figure of the worked example."""
    write_figure(
        'includes/imu-noise-three-readings.html', three_readings_figure(),
        f'Three readings one output step Δ = {GYROSCOPE_STEP_TEXT} apart. Their mean is Δ. The '
        'arrows are the deviations from the mean: −Δ, 0 and +Δ.')


def variance_test_figure() -> Figure:
    """Return the drawing of the four values of the sample variance's test."""
    values = VARIANCE_TEST_VALUES
    mean = statistics.fmean(values)
    deviations = listed([signed_value(value - mean) for value in values])
    value_list = listed([f'{value:g}' for value in values])
    figure = Figure('imu-noise-test-variance', f'The {NUMBER_WORDS[len(values)]} values '
                    f'{value_list} on an axis. Their mean is {mean:g}. Their deviations from the '
                    f'mean are {deviations}', 164, DRAWING_WIDTH)
    axis_y, first_x, pixels_per_unit = 96, 48, 68

    def axis_x(value: float) -> float:
        """Return the x position of a value on the axis."""
        return first_x + (value - values[0]) * pixels_per_unit

    figure.path(f'M{first_x - 30},{axis_y} H{axis_x(values[-1]) + 30}', 'axis')
    figure.path(f'M{axis_x(mean):.1f},{axis_y - 66} V{axis_y + 8}', 'mean-line')
    figure.text(axis_x(mean), axis_y - 72, f'mean {mean:g}', 'strong', 'middle')
    for value in values:
        deviation = value - mean
        # The two outer values' arrows above the two inner ones'
        arrow_y = axis_y - (48 if abs(deviation) > 1 else 22)
        deviation_arrow(figure, axis_x(mean), axis_x(value), arrow_y, signed_value(deviation))
        figure.circle(axis_x(value), axis_y, 5.5, 'imu-dot')
        figure.text(axis_x(value), axis_y + 22, f'{value:g}', 'strong', 'middle')
        figure.text(axis_x(value), axis_y + 38, f'{deviation ** 2:g}', 'note', 'middle')
    figure.text(axis_x(mean), axis_y + 56, 'the deviations, squared', 'note', 'middle')
    return figure


def write_variance_test() -> None:
    """Write the figure of the sample variance's test."""
    values = VARIANCE_TEST_VALUES
    squares = sum((value - statistics.fmean(values)) ** 2 for value in values)
    write_figure(
        'includes/imu-noise-test-variance.html', variance_test_figure(),
        f'The squared deviations add up to {squares:g}. The sample variance is '
        f'{squares:g} / ({len(values)} − 1) = {statistics.variance(values):.4f}. Dividing by '
        f'{len(values)} would give {squares / len(values):g}.')


def interval_x(milliseconds: float, pixels_per_ms: float) -> float:
    """Return the x position of a header stamp on a time axis that starts at 26."""
    return 26 + milliseconds * pixels_per_ms


def numbered_mark(figure: Figure, x: float, y: float, number: int, css_class: str) -> None:
    """Draw a numbered stamp or step at the given position, with the shared numbered disc."""
    disc = Drawing()
    disc.number_disc(x, y, number, css_class, 'letter')
    figure.elements.extend(disc.elements)


def intervals_figure(layout: PlotLayout) -> Figure:
    """Return a layout of the five header stamps on a time axis, with the intervals between."""
    stamps = STAMPS_WITH_ONE_LONG_INTERVAL
    pairs = list(zip(stamps, stamps[1:]))
    regular = {later - earlier for earlier, later in pairs if later - earlier <= LONG_INTERVAL}
    long_pairs = [(earlier, later) for earlier, later in pairs if later - earlier > LONG_INTERVAL]
    if len(regular) != 1 or len(long_pairs) != 1:
        raise ValueError('the figure explains equal regular intervals and one long interval')
    regular_count = len(pairs) - 1
    (long_start, long_end), = long_pairs
    figure = Figure(f'imu-noise-test-intervals{layout.name_end}',
                    f'{NUMBER_WORDS[len(stamps)].capitalize()} header '
                    f'stamps on a time axis, at {listed([str(stamp) for stamp in stamps])} ms. '
                    f'{NUMBER_WORDS[regular_count].capitalize()} intervals are '
                    f'{regular.pop()} ms long. The interval from {long_start} to {long_end} ms is '
                    f'{long_end - long_start} ms long, more than {LONG_INTERVAL} ms', 104,
                    round(DRAWING_WIDTH * layout.stretch), layout.css_class, layout.stretch)
    axis_y, pixels_per_ms = 60, 2.5
    figure.path(f'M12,{axis_y} H{interval_x(stamps[-1], pixels_per_ms) + 14:.1f}', 'axis')
    for earlier, later in zip(stamps, stamps[1:]):
        start, end = (interval_x(stamp, pixels_per_ms) for stamp in (earlier, later))
        is_long = later - earlier > LONG_INTERVAL
        figure.path(f'M{start:.1f},{axis_y - 24} H{end:.1f} M{start:.1f},{axis_y - 27} v6 '
                    f'M{end:.1f},{axis_y - 27} v6', 'long-interval' if is_long else 'dimension')
        figure.text((start + end) / 2, axis_y - 30, f'{later - earlier} ms',
                    'strong' if is_long else '', 'middle')
    for number, stamp in enumerate(stamps, start=1):
        x = interval_x(stamp, pixels_per_ms)
        numbered_mark(figure, x, axis_y, number, 'pass-mark')
        figure.text(x, axis_y + 26, str(stamp), 'note', 'middle')
    figure.text(interval_x(stamps[-1], pixels_per_ms) + 14, axis_y + 26, 'ms', 'note')
    return figure


def write_intervals() -> None:
    """Write the figure of the stamps with one long interval."""
    stamps = STAMPS_WITH_ONE_LONG_INTERVAL
    duration = stamps[-1] - stamps[0]
    interval_count = len(stamps) - 1
    write_layout_figure(
        'includes/imu-noise-test-intervals.html',
        [intervals_figure(layout) for layout in PLOT_LAYOUTS],
        f'<code>STAMPS_WITH_ONE_LONG_INTERVAL</code>: {len(stamps)} header stamps, '
        f'{interval_count} intervals in {duration} ms. The rate is {interval_count} / '
        f'{duration / 1000:g} s = {interval_count / (duration / 1000):.1f} Hz. One interval is '
        f'longer than <code>LONG_INTERVAL</code>, {LONG_INTERVAL} ms.')


def stamps_back_name(stamps: Sequence[int]) -> str:
    """Return the accessible name of the stamps figure: each stamp that doesn't advance."""
    sentences = [f'{NUMBER_WORDS[len(stamps)].capitalize()} messages in the order of the '
                 'recording, each at its header stamp']
    for index in range(1, len(stamps)):
        this, before = ORDINAL_WORDS[index], ORDINAL_WORDS[index - 1]
        if stamps[index] == stamps[index - 1]:
            sentences.append(f'The {this} message repeats the stamp of the {before}, '
                             f'{stamps[index]} ms')
        elif stamps[index] < stamps[index - 1]:
            sentences.append(f'The {this} is stamped {stamps[index]} ms, earlier than the '
                             f'{before}')
        elif index == len(stamps) - 1:
            sentences.append(f'The {this} is stamped {stamps[index]} ms')
    return '. '.join(sentences)


def stamps_back_figure(layout: PlotLayout) -> Figure:
    """Return a layout of five messages in recording order against their header stamps."""
    stamps = STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK
    row_height, first_row_y, pixels_per_ms = 26, 34, 2.6
    axis_y = first_row_y + len(stamps) * row_height
    figure = Figure(f'imu-noise-test-stamps-back{layout.name_end}', stamps_back_name(stamps),
                    axis_y + 40, round(DRAWING_WIDTH * layout.stretch), layout.css_class,
                    layout.stretch)
    figure.text(4, 13, 'message, in the order of the recording', 'note')
    points = [(interval_x(stamp, pixels_per_ms), first_row_y + index * row_height)
              for index, stamp in enumerate(stamps)]
    for (start_x, start_y), (end_x, end_y), earlier, later in zip(points, points[1:], stamps,
                                                                  stamps[1:]):
        advances = later > earlier
        figure.path(f'M{start_x:.1f},{start_y} L{end_x:.1f},{end_y}',
                    'step-line' if advances else 'step-bad')
        if not advances:
            label = 'repeated' if later == earlier else f'{earlier - later} ms earlier'
            # The label keeps its distance from the line in the stretched layout too
            figure.text(max(start_x, end_x) + figure.fixed(14), (start_y + end_y) / 2 + 4,
                        label, 'strong')
    for number, (x, y) in enumerate(points, start=1):
        numbered_mark(figure, x, y, number, 'pass-mark')
    figure.path(f'M12,{axis_y} H{interval_x(stamps[-1], pixels_per_ms) + 14:.1f}', 'axis')
    for stamp in sorted(set(stamps)):
        x = interval_x(stamp, pixels_per_ms)
        figure.path(f'M{x:.1f},{axis_y} v4', 'tick')
        figure.text(x, axis_y + 17, str(stamp), 'note', 'middle')
    figure.text(interval_x(stamps[-1], pixels_per_ms) + 14, axis_y + 17, 'ms', 'note')
    figure.text(interval_x(stamps[-1] / 2, pixels_per_ms), axis_y + 34, 'header stamp', 'note',
                'middle')
    return figure


def write_stamps_back() -> None:
    """Write the figure of the stamps that don't advance."""
    stamps = STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK
    interval_count = len(stamps) - 1
    duration = stamps[-1] - stamps[0]
    write_layout_figure(
        'includes/imu-noise-test-stamps-back.html',
        [stamps_back_figure(layout) for layout in PLOT_LAYOUTS],
        '<code>STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK</code>: the last stamp is '
        f'{duration} ms after the first, and {interval_count} intervals in {duration} ms '
        f'would be {interval_count / (duration / 1000):.0f} Hz. The dashed red lines are the '
        "two stamps that don't advance, so the tool gives no rate.")


def measured_figure() -> Figure:
    """Return the drawing of three messages, of which one has no angular velocity."""
    row_height, top, covariance_x = 52, 22, 86
    with_reading = [(index, yaw_rate) for index, yaw_rate in enumerate(MEASURED_YAW_RATES)
                    if yaw_rate is not None]
    without_reading = [index for index, yaw_rate in enumerate(MEASURED_YAW_RATES)
                       if yaw_rate is None]
    if len(without_reading) != 1:
        raise ValueError('the figure explains one message without the angular velocity')
    messages = listed([f'the {ORDINAL_WORDS[index]}' for index, _ in with_reading])
    yaw_rates = listed([f'{yaw_rate:g}' for _, yaw_rate in with_reading])
    figure = Figure('imu-noise-test-measured',
                    f'{NUMBER_WORDS[len(MEASURED_YAW_RATES)].capitalize()} messages. '
                    f'{messages.capitalize()} have a covariance that starts with 0 and yaw rates '
                    f'of {yaw_rates} rad/s: {NUMBER_WORDS[len(with_reading)]} readings. The '
                    f'{ORDINAL_WORDS[without_reading[0]]} has a covariance that starts with −1: '
                    'no reading', top + len(MEASURED_YAW_RATES) * row_height, DRAWING_WIDTH)
    figure.text(covariance_x, 14, 'angular_velocity_covariance[0]', 'mono quiet')
    for index, yaw_rate in enumerate(MEASURED_YAW_RATES):
        y = top + index * row_height
        measured = yaw_rate is not None
        figure.rect(4, y, DRAWING_WIDTH - 8, row_height - 8,
                    'card-pass' if measured else 'card-drop', corner_radius=5)
        figure.text(12, y + 26, f'message {index + 1}', 'note')
        figure.text(covariance_x, y + 17, '0.0' if measured else '−1.0', 'mono strong')
        figure.text(DRAWING_WIDTH - 12, y + 17, 'angular_velocity.z', 'mono quiet', 'end')
        if measured:
            figure.text(covariance_x, y + 36, '✓ a reading', 'strong')
            figure.text(DRAWING_WIDTH - 12, y + 36, f'{yaw_rate} rad/s', 'strong', 'end')
        else:
            figure.text(covariance_x, y + 36, '✕ not measured', 'strong')
            figure.text(DRAWING_WIDTH - 12, y + 36, 'not used', 'note', 'end')
    return figure


def write_measured() -> None:
    """Write the figure of the field that one message didn't measure."""
    readings = [yaw_rate for yaw_rate in MEASURED_YAW_RATES if yaw_rate is not None]
    write_figure(
        'includes/imu-noise-test-measured.html', measured_figure(),
        f'{len(readings)} readings with the mean {statistics.fmean(readings):g} rad/s. '
        f'Counting the second message as a reading of zero would give {len(MEASURED_YAW_RATES)} '
        f'readings with the mean {sum(readings) / len(MEASURED_YAW_RATES):g} rad/s.')


def isometric(point: Point, origin: DrawingPoint, pixels_per_unit: float) -> DrawingPoint:
    """Return where a point with x, y and z lies in a drawing with z up and x and y receding."""
    return DrawingPoint(
        origin.x + (point.y - point.x) * math.cos(math.radians(30)) * pixels_per_unit,
        origin.y + (point.x + point.y) * math.sin(math.radians(30)) * pixels_per_unit
        - point.z * pixels_per_unit)


def gravity_figure() -> Figure:
    """Return the drawing of the mean acceleration of a tilted IMU as a vector in a box."""
    mean_x, mean_y, mean_z = TILTED_MEAN_ACCELERATION
    length = math.hypot(mean_x, mean_y, mean_z)
    origin, pixels_per_unit = DrawingPoint(86.0, 150.0), 19.0
    figure = Figure('imu-noise-test-gravity', 'The mean acceleration of a tilted IMU as an '
                    f'arrow from the origin to the point x = {mean_x:g}, y = {mean_y:g}, '
                    f'z = {mean_z:g} m/s². The arrow is {length:g} m/s² long', 206,
                    DRAWING_WIDTH)

    def svg_point(x: float, y: float, z: float) -> str:
        """Return a point's position as SVG coordinates."""
        drawing = isometric(Point(x, y, z), origin, pixels_per_unit)
        return f'{drawing.x:.1f},{drawing.y:.1f}'

    figure.path(f'M{svg_point(mean_x, 0, 0)} L{svg_point(mean_x, mean_y, 0)} '
                f'L{svg_point(0, mean_y, 0)} '
                f'M{svg_point(mean_x, mean_y, 0)} L{svg_point(mean_x, mean_y, mean_z)} '
                f'M{svg_point(0, 0, mean_z)} L{svg_point(mean_x, 0, mean_z)} '
                f'L{svg_point(mean_x, mean_y, mean_z)} L{svg_point(0, mean_y, mean_z)} z '
                f'M{svg_point(mean_x, 0, 0)} L{svg_point(mean_x, 0, mean_z)} '
                f'M{svg_point(0, mean_y, 0)} L{svg_point(0, mean_y, mean_z)}', 'box-edge')
    axis_labels = (AxisLabel(Point(mean_x + 1.4, 0, 0), 'x', DrawingPoint(-10, 6)),
                   AxisLabel(Point(0, mean_y + 1.2, 0), 'y', DrawingPoint(9, 6)),
                   AxisLabel(Point(0, 0, mean_z + 1.0), 'z', DrawingPoint(-4, -6)))
    for axis_label in axis_labels:
        end = isometric(axis_label.end, origin, pixels_per_unit)
        figure.path(f'M{origin.x},{origin.y} L{end.x:.1f},{end.y:.1f}', 'axis')
        figure.text(end.x + axis_label.offset.x, end.y + axis_label.offset.y, axis_label.name,
                    'variable', 'middle')
    tip_x, tip_y = isometric(TILTED_MEAN_ACCELERATION, origin, pixels_per_unit)
    figure.arrow(origin.x, origin.y, tip_x, tip_y, 'gravity')
    # To the right of the box, whose right edge is at y = mean_y
    label_x = isometric(Point(0, mean_y, 0), origin, pixels_per_unit).x + 14
    figure.text(label_x, tip_y + 2, 'mean acceleration', 'strong')
    figure.text(label_x, tip_y + 18, f'({mean_x:g}, {mean_y:g}, {mean_z:g}) m/s²')
    figure.text(label_x, tip_y + 40, f'length {length:g} m/s²', 'strong')
    figure.text(label_x, tip_y + 56, f'√({mean_x:g}² + {mean_y:g}² + {mean_z:g}²)')
    return figure


def write_gravity() -> None:
    """Write the figure of gravity as the length of the mean acceleration."""
    write_figure(
        'includes/imu-noise-test-gravity.html', gravity_figure(),
        'A tilted IMU measures gravity on all three axes. The length of the mean acceleration '
        "doesn't depend on the tilt.")


# The flow figure: the cards of the steps right of the command tests' bracket, and the two
# columns of test class names in each card
FLOW_TOP = 4  # of the bag file's card and of the bracket
FLOW_CARD_LEFT = 14
FLOW_CARD_RIGHT = DRAWING_WIDTH - 4
FLOW_PATH_X = 104  # the arrows from the bag file to the report
FLOW_EXIT_X = 223  # the arrow from the first step to the one line of a refused input
FLOW_BRACKET_X = 6
FLOW_BRACKET_END = 4  # the length of the bracket's two end strokes
# A character of the code font, Roboto Mono, is 0.6 times its font size wide
CLASS_NAME_WIDTH = 0.6 * CLASS_NAME_FONT_SIZE
CLASS_ROW_HEIGHT = 14
CARD_HEADER_HEIGHT = 26
CARD_BOTTOM_PADDING = 4  # below the last row of class names, for letters such as p
CARD_LABEL_LEFT = 28  # from a step card's left edge to its label, right of its number
# The texts that both layouts of the flow figure show
FLOW_INPUT = ['a bag file']
FLOW_ARROW_LABELS = ('samples', 'statistics')  # what the first two steps hand on
FLOW_EXIT_REASON = 'a check fails'
FLOW_EXIT = ['✕ one line on stderr', 'status 1']
FLOW_REPORT = ['✓ the report on stdout', 'status 0']
FLOW_COMMAND_LABEL = 'runs the tool as a command'


def class_columns(steps: Sequence[ToolStep]) -> Sequence[float]:
    """Return the x positions of the two columns of test class names in the steps' cards.

    Raise a ValueError when a name in the second column would leave its card.
    """
    first = FLOW_CARD_LEFT + 14
    first_width = max(len(test_class.name) for step in steps
                      for test_class in step.test_classes[::2]) * CLASS_NAME_WIDTH
    second = first + first_width + 10
    second_width = max(len(test_class.name) for step in steps
                       for test_class in step.test_classes[1::2]) * CLASS_NAME_WIDTH
    if second + second_width > FLOW_CARD_RIGHT - 6:
        raise ValueError('the second column of test class names is wider than the cards')
    return (first, second)


def step_card(figure: Figure, top: float, number: int, step: ToolStep,
              columns: Sequence[float]) -> float:
    """Add a step's card: its number, label and function, then its test classes in columns.

    Return the y of the card's bottom edge.
    """
    rows = math.ceil(len(step.test_classes) / len(columns))
    bottom = top + CARD_HEADER_HEIGHT + rows * CLASS_ROW_HEIGHT + CARD_BOTTOM_PADDING
    figure.rect(FLOW_CARD_LEFT, top, FLOW_CARD_RIGHT - FLOW_CARD_LEFT, bottom - top, 'item',
                corner_radius=5)
    numbered_mark(figure, FLOW_CARD_LEFT + 14, top + 15, number, 'item')
    figure.text(FLOW_CARD_LEFT + CARD_LABEL_LEFT, top + 19, step.label, 'strong')
    figure.text(FLOW_CARD_RIGHT - 8, top + 19, step.function, 'mono', 'end')
    for index, test_class in enumerate(step.test_classes):
        figure.text(columns[index % len(columns)],
                    top + CARD_HEADER_HEIGHT + 11 + index // len(columns) * CLASS_ROW_HEIGHT,
                    test_class.name, 'class-name')
    return bottom


def end_card(figure: Figure, center_x: float, top: float, width: float, lines: Sequence[str],
             css_class: str) -> float:
    """Add a card with centered lines, the first one bold. Return the y of its bottom edge."""
    bottom = top + 12 + len(lines) * 14
    figure.rect(center_x - width / 2, top, width, bottom - top, css_class, corner_radius=5)
    for index, line in enumerate(lines):
        figure.text(center_x, top + 20 + index * 14, line, 'strong' if index == 0 else 'note',
                    'middle')
    return bottom


def flow_name(steps: Sequence[ToolStep]) -> str:
    """Return the accessible name of the flow figure: the steps, the exit, the command tests."""
    step_names = listed([f'{step.label} with {step.function}' for step in steps])
    return (f"The tool's {NUMBER_WORDS[len(steps)]} steps from a bag file to the report, each "
            f'with its test classes: {step_names}. A failed check leaves the first step to one '
            f'line on stderr and status 1. {COMMAND_TEST_CLASS} runs the tool as a command')


def flow_caption(steps: Sequence[ToolStep]) -> str:
    """Return the flow figure's caption, with the number of tests at each step."""
    at_steps = listed([f'{sum(test_class.test_count for test_class in step.test_classes)} tests '
                       f'at {step.label}' for step in steps])
    return (f"The test classes in a step's card call that step's functions: {at_steps}. "
            f'<code>{COMMAND_TEST_CLASS}</code> runs the script as a command, most of its tests '
            f'on a small bag that the test writes. {COMMAND_TESTS_WITH_A_REPORT} of its tests '
            f'compare the report and status 0, and {COMMAND_TESTS_WITH_ONE_LINE} compare the one '
            'line and status 1.')


def flow_figure() -> Figure:
    """Return the narrow layout of the flow figure: the steps from top to bottom."""
    figure = Figure('imu-noise-test-flow', flow_name(TOOL_STEPS), 0, DRAWING_WIDTH, NARROW_LAYOUT)
    columns = class_columns(TOOL_STEPS)
    bag_bottom = end_card(figure, FLOW_PATH_X, FLOW_TOP, 124, FLOW_INPUT, 'item')
    top = bag_bottom + 18
    figure.arrow(FLOW_PATH_X, bag_bottom, FLOW_PATH_X, top, 'flow')
    for number, step in enumerate(TOOL_STEPS, start=1):
        bottom = step_card(figure, top, number, step, columns)
        if number == 1:
            exit_top = bottom + 16
            figure.arrow(FLOW_EXIT_X, bottom, FLOW_EXIT_X, exit_top, 'refused')
            figure.text(FLOW_EXIT_X - 6, bottom + 13, FLOW_EXIT_REASON, 'note', 'end')
            exit_bottom = end_card(figure, FLOW_EXIT_X, exit_top, 146, FLOW_EXIT, 'card-drop')
            top = exit_bottom + 8
        else:
            top = bottom + 20
        figure.arrow(FLOW_PATH_X, bottom, FLOW_PATH_X, top, 'flow')
        if number <= len(FLOW_ARROW_LABELS):
            figure.text(FLOW_PATH_X - 6, top - 6, FLOW_ARROW_LABELS[number - 1], 'note', 'end')
    report_bottom = end_card(figure, FLOW_PATH_X, top, 170, FLOW_REPORT, 'card-pass')
    label_y = report_bottom + 18
    # A bracket with two equal end strokes, from the bag file down to its label
    figure.path(f'M{FLOW_BRACKET_X + FLOW_BRACKET_END},{FLOW_TOP} H{FLOW_BRACKET_X} '
                f'V{label_y - 4} H{FLOW_BRACKET_X + FLOW_BRACKET_END}', 'command-span')
    figure.text(FLOW_BRACKET_X + 6, label_y, COMMAND_TEST_CLASS, 'class-name')
    figure.text(FLOW_BRACKET_X + 6 + len(COMMAND_TEST_CLASS) * CLASS_NAME_WIDTH + 6, label_y,
                FLOW_COMMAND_LABEL, 'note')
    figure.height = int(label_y) + 8
    return figure


# The flow figure's wide layout: the bag file above the first step, the three steps side by side,
# the one line of a refused input below the first step and the report below the last one
WIDE_FLOW_MARGIN = 4
WIDE_FLOW_MIN_GAP = 32  # between two step cards, for the arrow
WIDE_CARD_PADDING = 14  # left of a card's text, and at least as much right of it
WIDE_CARD_HEADER_HEIGHT = 42  # the number and label, then the function below them
WIDE_CARD_NUMBER_Y = 15  # from a card's top to the middle of its number
# A generous width of a character of the labels, bold text of 12 px. In Roboto, the widest of
# the three labels has 6.2 units per character
BOLD_CHARACTER_WIDTH = 7.4
FUNCTION_CHARACTER_WIDTH = 0.6 * 11.5  # the code font of the drawings' mono style


class WideCard(NamedTuple):
    """Where a step's card stands in the flow figure's wide layout."""

    left: float
    width: float

    def middle(self) -> float:
        """Return the card's horizontal middle."""
        return self.left + self.width / 2


def wide_cards(steps: Sequence[ToolStep]) -> Sequence[WideCard]:
    """Return the step cards of the wide layout, each as wide as its text, spread over the width.

    Raise a ValueError when the cards leave less than WIDE_FLOW_MIN_GAP between two of them.
    """
    widths = [2 * WIDE_CARD_PADDING + max(
        CARD_LABEL_LEFT + len(step.label) * BOLD_CHARACTER_WIDTH - WIDE_CARD_PADDING,
        len(step.function) * FUNCTION_CHARACTER_WIDTH,
        max(len(test_class.name) for test_class in step.test_classes) * CLASS_NAME_WIDTH)
        for step in steps]
    gap = (WIDE_WIDTH - 2 * WIDE_FLOW_MARGIN - sum(widths)) / (len(steps) - 1)
    if gap < WIDE_FLOW_MIN_GAP:
        raise ValueError('the step cards leave too little room for the arrows between them')
    cards = []
    left: float = WIDE_FLOW_MARGIN
    for width in widths:
        cards.append(WideCard(left, width))
        left += width + gap
    return cards


def middle_inside_margins(center: float, width: float) -> float:
    """Return a card's horizontal middle, moved so that the card stays inside the margins."""
    half_width = width / 2
    return min(max(center, WIDE_FLOW_MARGIN + half_width),
               WIDE_WIDTH - WIDE_FLOW_MARGIN - half_width)


def wide_step_card(figure: Figure, top: float, height: float, number: int, step: ToolStep,
                   card: WideCard) -> None:
    """Add a step's card in the wide layout: number and label, function, one class per row."""
    figure.rect(card.left, top, card.width, height, 'item', corner_radius=5)
    numbered_mark(figure, card.left + 14, top + WIDE_CARD_NUMBER_Y, number, 'item')
    figure.text(card.left + CARD_LABEL_LEFT, top + 19, step.label, 'strong')
    figure.text(card.left + WIDE_CARD_PADDING, top + 35, step.function, 'mono')
    for index, test_class in enumerate(step.test_classes):
        figure.text(card.left + WIDE_CARD_PADDING,
                    top + WIDE_CARD_HEADER_HEIGHT + 11 + index * CLASS_ROW_HEIGHT,
                    test_class.name, 'class-name')


def wide_flow_figure() -> Figure:
    """Return the wide layout of the flow figure: the steps from left to right."""
    figure = Figure('imu-noise-test-flow-wide', flow_name(TOOL_STEPS), 0, WIDE_WIDTH, WIDE_LAYOUT)
    cards = wide_cards(TOOL_STEPS)
    first, last = cards[0], cards[-1]
    bag_bottom = end_card(figure, first.middle(), FLOW_TOP, 124, FLOW_INPUT, 'item')
    top = bag_bottom + 18
    figure.arrow(first.middle(), bag_bottom, first.middle(), top, 'flow')
    rows = max(len(step.test_classes) for step in TOOL_STEPS)
    height = WIDE_CARD_HEADER_HEIGHT + rows * CLASS_ROW_HEIGHT + CARD_BOTTOM_PADDING
    for number, (step, card) in enumerate(zip(TOOL_STEPS, cards), start=1):
        wide_step_card(figure, top, height, number, step, card)
        if number < len(cards):
            following = cards[number]
            # At the height of the steps' numbers, so that the arrow's label, which is wider
            # than the gap, stands right above the arrow and above the cards
            arrow_y = top + WIDE_CARD_NUMBER_Y
            figure.arrow(card.left + card.width, arrow_y, following.left, arrow_y, 'flow')
            figure.text((card.left + card.width + following.left) / 2, top - 6,
                        FLOW_ARROW_LABELS[number - 1], 'note', 'middle')
    bottom = top + height
    exit_top = bottom + 22
    figure.arrow(first.middle(), bottom, first.middle(), exit_top, 'refused')
    figure.text(first.middle() + 6, bottom + 14, FLOW_EXIT_REASON, 'note')
    exit_bottom = end_card(figure, middle_inside_margins(first.middle(), 150), exit_top, 150,
                           FLOW_EXIT, 'card-drop')
    figure.arrow(last.middle(), bottom, last.middle(), exit_top, 'flow')
    end_card(figure, middle_inside_margins(last.middle(), 170), exit_top, 170, FLOW_REPORT,
             'card-pass')
    # A bracket under the whole path, with two equal end strokes, and its label below
    bracket_y = exit_bottom + 14
    figure.path(f'M{WIDE_FLOW_MARGIN},{bracket_y - FLOW_BRACKET_END} V{bracket_y} '
                f'H{WIDE_WIDTH - WIDE_FLOW_MARGIN} V{bracket_y - FLOW_BRACKET_END}',
                'command-span')
    figure.rich_text(WIDE_WIDTH / 2, bracket_y + 16,
                     (Span(COMMAND_TEST_CLASS, 'class-name'),
                      Span(f' {FLOW_COMMAND_LABEL}', 'note')), 'middle')
    figure.height = int(bracket_y) + 24
    return figure


def write_flow() -> None:
    """Write the flow figure in its two layouts, with the number of tests at each step."""
    write_layout_figure('includes/imu-noise-test-flow.html', [flow_figure(), wide_flow_figure()],
                        flow_caption(TOOL_STEPS))


if __name__ == '__main__':
    write_trace()
    write_three_readings()
    write_variance_test()
    write_intervals()
    write_stamps_back()
    write_measured()
    write_gravity()
    write_flow()
