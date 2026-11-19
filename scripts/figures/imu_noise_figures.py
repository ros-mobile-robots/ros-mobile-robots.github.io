"""Generate the figures about the IMU's bias and noise at rest as inline SVG.

The pages are docs/packages/imu/driver-bno055.md, section "Bias and noise at rest", and
docs/packages/imu/noise-at-rest-tests.md, the walkthrough of the tests of
diffbot_bringup/scripts/imu_noise_at_rest.py in the diffbot repository. The script writes these
includes, which follow the site's light and dark colors:

- includes/imu-noise-trace.html: yaw-rate readings at rest against time, with zero, the mean and
  the mean plus and minus one standard deviation. The readings are a synthetic example, made here
  from a seed: a gyroscope with a bias large enough to see, on the BNO055's output steps. The
  dots are the first second, the lines come from all 180 s.
- includes/imu-noise-three-readings.html: three readings one output step apart, their mean and
  their deviations from it. The worked example of the sample variance.
- includes/imu-noise-test-variance.html: the four values 1, 2, 3 and 4 of the test of the sample
  variance, their mean and their deviations from it.
- includes/imu-noise-test-intervals.html: five header stamps with one interval of 35 ms.
- includes/imu-noise-test-stamps-back.html: five messages in the order of the recording, of
  which one repeats the stamp before and one has an earlier stamp.
- includes/imu-noise-test-measured.html: three messages of which the middle one marks its
  angular velocity as not measured.
- includes/imu-noise-test-gravity.html: the mean acceleration (2, 3, 6) m/s² of a tilted IMU and
  its length 7 m/s².

Every drawing is 300 units wide and carries short labels only, so that its text is readable on a
phone. Explanations are HTML text below a drawing. Figure, the CSS rules and the helpers come
from localization_figures.py.

The tests' values are copied here, each with the name of its constant in
diffbot_bringup/test/test_imu_noise_at_rest.py, because this repository can't import the tests.

Run from the repository root after changing a figure:

    python3 scripts/figures/imu_noise_figures.py
"""

import math
import random
import statistics
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

from localization_figures import arrow_head
from localization_figures import DRAWING_RULES
from localization_figures import Figure
from localization_figures import LARGE_RULES
from localization_figures import RULES
from localization_figures import SYMBOL_RULES
from localization_figures import write_parts

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
BG = 'var(--md-default-bg-color)'

DRAWING_WIDTH = 300
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
)
FIGURE_RULES = RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES + NOISE_RULES
FIGURE_CLASSES = 'lv-large lv-drawing lv-symbols lv-noise'

# The BNO055's output step of the angular velocity: 900 LSB per rad/s
GYROSCOPE_STEP = 1 / 900  # rad/s
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
TRACE_PLOT_RIGHT = 182
TRACE_PLOT_TOP = 24
TRACE_PIXELS_PER_STEP = 26

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
# test_imu_statistics_gives_gravity_as_the_length_of_the_mean_acceleration: the mean of
# (1, 3, 5) and (3, 3, 7)
TILTED_MEAN_ACCELERATION = (2.0, 3.0, 6.0)  # m/s^2


def write_figure(path: str, figure: Figure, caption: str) -> None:
    """Write an include with one drawing and its caption, in this script's style."""
    write_parts(path, [figure.svg()], caption, FIGURE_RULES, FIGURE_CLASSES)


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


def signed(value: float) -> str:
    """Return a value in rad/s with five decimals, its sign and a typographic minus."""
    return f'{value:+.5f}'.replace('-', '−')


def trace_y(yaw_rate: float) -> float:
    """Return the y position of a yaw rate in the trace figure."""
    top_rate = (TRACE_LEVELS[-1] + 0.5) * GYROSCOPE_STEP
    return TRACE_PLOT_TOP + (top_rate - yaw_rate) / GYROSCOPE_STEP * TRACE_PIXELS_PER_STEP


def trace_x(seconds: float) -> float:
    """Return the x position of a time in the trace figure."""
    return TRACE_PLOT_LEFT + seconds / TRACE_SHOWN_DURATION * (TRACE_PLOT_RIGHT - TRACE_PLOT_LEFT)


def trace_figure(trace: Trace) -> Figure:
    """Return the drawing of the readings of the first second and the recording's statistics."""
    bottom = trace_y((TRACE_LEVELS[0] - 0.5) * GYROSCOPE_STEP)
    figure = Figure('imu-noise-trace', 'Yaw-rate readings of a gyroscope at rest against time. '
                    'The readings lie on output steps. Their mean lies above zero: the bias. '
                    'A band of one standard deviation around the mean shows their spread',
                    int(bottom) + 44, DRAWING_WIDTH)
    figure.text(4, 13, 'angular_velocity.z in rad/s', 'mono')
    for level in TRACE_LEVELS:
        y = trace_y(level * GYROSCOPE_STEP)
        figure.path(f'M{TRACE_PLOT_LEFT},{y:.1f} H{TRACE_PLOT_RIGHT}', 'level')
        label = '0' if level == 0 else f'{level * GYROSCOPE_STEP:+.4f}'.replace('-', '−')
        figure.text(TRACE_PLOT_LEFT - 5, y + 4, label, 'note', 'end')
    upper, lower = (trace_y(trace.mean + sign * trace.standard_deviation) for sign in (1, -1))
    figure.rect(TRACE_PLOT_LEFT, upper, TRACE_PLOT_RIGHT - TRACE_PLOT_LEFT, lower - upper,
                'spread-band')
    figure.path(f'M{TRACE_PLOT_LEFT},{trace_y(0):.1f} H{TRACE_PLOT_RIGHT}', 'zero-line')
    lines = ((trace.mean + trace.standard_deviation, 'spread-line', 'mean + s'),
             (trace.mean, 'mean-line', 'mean'),
             (trace.mean - trace.standard_deviation, 'spread-line', 'mean − s'))
    for yaw_rate, css_class, label in lines:
        y = trace_y(yaw_rate)
        figure.path(f'M{TRACE_PLOT_LEFT},{y:.1f} H{TRACE_PLOT_RIGHT}', css_class)
        figure.text(TRACE_PLOT_RIGHT + 5, y + 4, label, 'strong' if label == 'mean' else '')
        figure.text(DRAWING_WIDTH - 2, y + 4, f'{yaw_rate:+.4f}'.replace('-', '−'),
                    'strong' if label == 'mean' else '', 'end')
    figure.text(TRACE_PLOT_RIGHT + 5, trace_y(0) + 4, 'zero')
    figure.text(DRAWING_WIDTH - 2, trace_y(0) + 4, '0', '', 'end')
    for index, reading in enumerate(trace.readings[:TRACE_SHOWN_DURATION * SAMPLE_RATE]):
        figure.circle(trace_x(index / SAMPLE_RATE), trace_y(reading), 1.3, 'reading')
    figure.path(f'M{TRACE_PLOT_LEFT},{TRACE_PLOT_TOP} V{bottom:.1f} H{TRACE_PLOT_RIGHT}', 'axis')
    for tenths in range(0, TRACE_SHOWN_DURATION * 10 + 1, 5):
        x = trace_x(tenths / 10)
        figure.path(f'M{x:.1f},{bottom:.1f} v4', 'tick')
        figure.text(x, bottom + 17, f'{tenths / 10:g}', 'note', 'middle')
    figure.text((TRACE_PLOT_LEFT + TRACE_PLOT_RIGHT) / 2, bottom + 36,
                'seconds after the first header stamp', 'note', 'middle')
    return figure


def write_trace() -> None:
    """Write the trace figure, with the recording's statistics in its caption."""
    trace = synthetic_trace()
    shown_count = TRACE_SHOWN_DURATION * SAMPLE_RATE
    write_figure(
        'includes/imu-noise-trace.html', trace_figure(trace),
        '<strong>A synthetic example, not a recording of a robot.</strong> A gyroscope at rest '
        'that reads in steps of 1/900 rad/s, like the BNO055. '
        f'The dots are the {shown_count} readings of the first second. The lines come from all '
        f'{len(trace.readings):,} readings of {TRACE_DURATION} s: '
        f'the mean {signed(trace.mean)} rad/s is the bias, '
        f'and the standard deviation <em>s</em> = {trace.standard_deviation:.5f} rad/s is the '
        f'noise. The readings take {trace.distinct_value_count} distinct values. The dotted '
        'lines are the output steps.')


def deviation_arrow(figure: Figure, mean_x: float, x: float, y: float, label: str) -> None:
    """Add an arrow from the mean's line to a reading's x position, with the deviation above."""
    direction = 1 if x > mean_x else -1
    figure.path(f'M{mean_x:.1f},{y} H{x - direction * 6:.1f}', 'deviation-line')
    figure.path(arrow_head(x - direction, y, direction, 0), 'deviation-head')
    figure.text((mean_x + x) / 2, y - 6, label, 'strong', 'middle')


def three_readings_figure() -> Figure:
    """Return the drawing of three readings one step apart, their mean and deviations."""
    figure = Figure('imu-noise-three-readings', 'Three yaw-rate readings on an axis: zero, one '
                    'output step and two output steps. Their mean is one step. The outer '
                    'readings deviate from the mean by one step each', 126, DRAWING_WIDTH)
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
        'Three readings one output step Δ = 1/900 rad/s apart. Their mean is Δ. The arrows are '
        'the deviations from the mean: −Δ, 0 and +Δ.')


def variance_test_figure() -> Figure:
    """Return the drawing of the four values of the sample variance's test."""
    values = VARIANCE_TEST_VALUES
    mean = statistics.fmean(values)
    figure = Figure('imu-noise-test-variance', 'The four values 1, 2, 3 and 4 on an axis. Their '
                    'mean is 2.5. Their deviations from the mean are −1.5, −0.5, +0.5 and +1.5',
                    164, DRAWING_WIDTH)
    axis_y, first_x, pixels_per_unit = 96, 48, 68

    def at(value: float) -> float:
        """Return the x position of a value on the axis."""
        return first_x + (value - values[0]) * pixels_per_unit

    figure.path(f'M{first_x - 30},{axis_y} H{at(values[-1]) + 30}', 'axis')
    figure.path(f'M{at(mean):.1f},{axis_y - 66} V{axis_y + 8}', 'mean-line')
    figure.text(at(mean), axis_y - 72, f'mean {mean:g}', 'strong', 'middle')
    for value in values:
        deviation = value - mean
        # The two outer values' arrows above the two inner ones'
        arrow_y = axis_y - (48 if abs(deviation) > 1 else 22)
        deviation_arrow(figure, at(mean), at(value), arrow_y, f'{deviation:+g}'.replace('-', '−'))
        figure.circle(at(value), axis_y, 5.5, 'imu-dot')
        figure.text(at(value), axis_y + 22, f'{value:g}', 'strong', 'middle')
        figure.text(at(value), axis_y + 38, f'{deviation ** 2:g}', 'note', 'middle')
    figure.text(at(mean), axis_y + 56, 'the deviations, squared', 'note', 'middle')
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


def intervals_figure() -> Figure:
    """Return the drawing of five header stamps on a time axis, with the intervals between."""
    figure = Figure('imu-noise-test-intervals', 'Five header stamps on a time axis, at 0, 20, '
                    '40, 75 and 95 ms. Three intervals are 20 ms long. The interval from 40 to '
                    '75 ms is 35 ms long, more than 30 ms', 104, DRAWING_WIDTH)
    axis_y, pixels_per_ms = 60, 2.5
    stamps = STAMPS_WITH_ONE_LONG_INTERVAL
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
        figure.circle(x, axis_y, 8, 'pass-mark')
        figure.text(x, axis_y + 4, str(number), 'letter', 'middle')
        figure.text(x, axis_y + 26, str(stamp), 'note', 'middle')
    figure.text(interval_x(stamps[-1], pixels_per_ms) + 14, axis_y + 26, 'ms', 'note')
    return figure


def write_intervals() -> None:
    """Write the figure of the stamps with one long interval."""
    stamps = STAMPS_WITH_ONE_LONG_INTERVAL
    duration = stamps[-1] - stamps[0]
    interval_count = len(stamps) - 1
    write_figure(
        'includes/imu-noise-test-intervals.html', intervals_figure(),
        f'<code>STAMPS_WITH_ONE_LONG_INTERVAL</code>: {len(stamps)} header stamps, '
        f'{interval_count} intervals in {duration} ms. The rate is {interval_count} / '
        f'{duration / 1000:g} s = {interval_count / (duration / 1000):.1f} Hz. One interval is '
        f'longer than <code>LONG_INTERVAL</code>, {LONG_INTERVAL} ms.')


def stamps_back_figure() -> Figure:
    """Return the drawing of five messages in recording order against their header stamps."""
    stamps = STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK
    row_height, first_row_y, pixels_per_ms = 26, 34, 2.6
    axis_y = first_row_y + len(stamps) * row_height
    figure = Figure('imu-noise-test-stamps-back', 'Five messages in the order of the recording, '
                    'each at its header stamp. The third message repeats the stamp of the '
                    'second, 20 ms. The fourth is stamped 10 ms, earlier than the third. The '
                    'fifth is stamped 80 ms', axis_y + 40, DRAWING_WIDTH)
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
            figure.text(max(start_x, end_x) + 14, (start_y + end_y) / 2 + 4, label,
                        'strong')
    for number, (x, y) in enumerate(points, start=1):
        figure.circle(x, y, 8, 'pass-mark')
        figure.text(x, y + 4, str(number), 'letter', 'middle')
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
    write_figure(
        'includes/imu-noise-test-stamps-back.html', stamps_back_figure(),
        '<code>STAMPS_WITH_A_REPEAT_AND_A_STEP_BACK</code>: the last stamp is '
        f'{duration} ms after the first, and {interval_count} intervals in {duration} ms '
        f'would be {interval_count / (duration / 1000):.0f} Hz. The dashed red lines are the '
        "two stamps that don't advance, so the tool gives no rate.")


def measured_figure() -> Figure:
    """Return the drawing of three messages, of which one has no angular velocity."""
    row_height, top, covariance_x = 52, 22, 86
    figure = Figure('imu-noise-test-measured', 'Three messages. The first and the third have a '
                    'covariance that starts with 0 and yaw rates of 0.2 and 0.4 rad/s: two '
                    'readings. The second has a covariance that starts with −1: no reading',
                    top + len(MEASURED_YAW_RATES) * row_height, DRAWING_WIDTH)
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


def isometric(point: Tuple[float, float, float], origin: Tuple[float, float],
              pixels_per_unit: float) -> Tuple[float, float]:
    """Return where a point with x, y and z lies in a drawing with z up and x and y receding."""
    x, y, z = point
    return (origin[0] + (y - x) * math.cos(math.radians(30)) * pixels_per_unit,
            origin[1] + (x + y) * math.sin(math.radians(30)) * pixels_per_unit
            - z * pixels_per_unit)


def gravity_figure() -> Figure:
    """Return the drawing of the mean acceleration of a tilted IMU as a vector in a box."""
    mean_x, mean_y, mean_z = TILTED_MEAN_ACCELERATION
    length = math.hypot(mean_x, mean_y, mean_z)
    origin, pixels_per_unit = (86.0, 150.0), 19.0
    figure = Figure('imu-noise-test-gravity', 'The mean acceleration of a tilted IMU as an '
                    'arrow from the origin to the point x = 2, y = 3, z = 6 m/s². The arrow is '
                    '7 m/s² long', 206, DRAWING_WIDTH)

    def at(x: float, y: float, z: float) -> str:
        """Return a point's position as SVG coordinates."""
        drawing_x, drawing_y = isometric((x, y, z), origin, pixels_per_unit)
        return f'{drawing_x:.1f},{drawing_y:.1f}'

    figure.path(f'M{at(mean_x, 0, 0)} L{at(mean_x, mean_y, 0)} L{at(0, mean_y, 0)} '
                f'M{at(mean_x, mean_y, 0)} L{at(mean_x, mean_y, mean_z)} '
                f'M{at(0, 0, mean_z)} L{at(mean_x, 0, mean_z)} L{at(mean_x, mean_y, mean_z)} '
                f'L{at(0, mean_y, mean_z)} z M{at(mean_x, 0, 0)} L{at(mean_x, 0, mean_z)} '
                f'M{at(0, mean_y, 0)} L{at(0, mean_y, mean_z)}', 'box-edge')
    for end, label, offset in (((mean_x + 1.4, 0, 0), 'x', (-10, 6)),
                               ((0, mean_y + 1.2, 0), 'y', (9, 6)),
                               ((0, 0, mean_z + 1.0), 'z', (-4, -6))):
        end_x, end_y = isometric(end, origin, pixels_per_unit)
        figure.path(f'M{origin[0]},{origin[1]} L{end_x:.1f},{end_y:.1f}', 'axis')
        figure.text(end_x + offset[0], end_y + offset[1], label, 'variable', 'middle')
    tip_x, tip_y = isometric(TILTED_MEAN_ACCELERATION, origin, pixels_per_unit)
    figure.arrow(origin[0], origin[1], tip_x, tip_y, 'gravity')
    # To the right of the box, whose right edge is at y = mean_y
    label_x = isometric((0, mean_y, 0), origin, pixels_per_unit)[0] + 14
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


if __name__ == '__main__':
    write_trace()
    write_three_readings()
    write_variance_test()
    write_intervals()
    write_stamps_back()
    write_measured()
    write_gravity()
