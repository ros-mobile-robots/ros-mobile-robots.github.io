"""Generate the figures of the two test pages of diffbot_control's EKF tests as inline SVG.

The pages are docs/packages/imu/robot_localization/tests/validator.md and stepped-clock.md. The
script writes three includes that follow the site's light and dark colors:

- includes/localization-test-validator.html: the samples that the unit tests in
  diffbot_control/test/test_imu_validator.py of the diffbot repository give the validator, on
  four time axes: at the two time limits, around the last valid sample's stamp, after the
  clock went back to 0.02 s, and after the clock went back to 50 s.
- includes/localization-test-stepped-clock-step.html: the messages of one step of
  diffbot_control/test/test_ekf_synthetic.py between the test, the validator and the EKF,
  numbered as the docstring of the test's step() numbers what one step does.
- includes/localization-test-stepped-clock.html: the seven phases of that test, with the yaw
  rate that the wheel odometry and the IMU report in each step, what the test asserts about
  the filter's estimate, and the estimates that a run of the test printed.

Every drawing is 300 units wide and carries short labels only, so that its text is readable on
a phone. Headings, legends and results are HTML text around the drawings. Figure, Sample, the
CSS rules and the HTML helpers come from localization_figures.py.

The tests' constants are copied here, each with the constant's name, because this repository
can't import the tests. What a run of the stepped-clock test printed is measured data. It is
read from scripts/figures/data/localization-stepped-clock-test-runs.jsonl, one line per run of
rostest diffbot_control ekf_synthetic.test, with the run's commit of the diffbot repository,
its date and its environment. The figure shows the file's last run.

Run from the repository root after changing a figure:

    python3 scripts/figures/localization_control_test_figures.py
"""

import json
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

from localization_figures import band
from localization_figures import CHECKS_WIDTH
from localization_figures import escape
from localization_figures import Figure
from localization_figures import LARGE_MARK_RADIUS
from localization_figures import LARGE_RULES
from localization_figures import RULES
from localization_figures import Sample
from localization_figures import SAMPLE_PERIOD_MS
from localization_figures import TimeAxis
from localization_figures import write_parts

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
BG = 'var(--md-default-bg-color)'

# The width of every drawing in view box units. The figure class lv-large shows a drawing at
# up to 400 px, and the text sizes of LARGE_RULES are made for this width
DRAWING_WIDTH = CHECKS_WIDTH

# test_imu_validator.py's constants
NOW_S = 100  # NOW, the ROS time of most checks
MAX_AGE_S = 0.1  # MAX_AGE
MAX_FUTURE_S = 0.02  # MAX_FUTURE
ONE_MICROSECOND_S = 1e-6  # ONE_MICROSECOND
JUST_TOO_OLD_S = MAX_AGE_S + ONE_MICROSECOND_S  # JUST_TOO_OLD
JUST_TOO_FAR_AHEAD_S = MAX_FUTURE_S + ONE_MICROSECOND_S  # JUST_TOO_FAR_AHEAD
# test_validator_drops_a_sample_not_later_than_the_last_valid_one
ORDER_OLDER_S = 0.01  # sample(NOW - seconds(0.01))
ORDER_LATER_NS = 1  # sample(NOW + rospy.Duration(0, 1))
# test_validator_forwards_a_new_sample_after_dropping_a_future_one
FAR_AHEAD_S = 5  # sample(NOW + seconds(5))
# test_validator_forwards_new_samples_after_the_clock_goes_back
EARLY_RESTART_S = 0.02  # restart = rospy.Time(0, 20000000)
REPEAT_DELAY_MS = 1  # restart + seconds(0.001), the ROS time of the second sample
# test_validator_still_drops_old_and_future_samples_after_the_clock_goes_back
RESTART_S = 50  # restart = rospy.Time(50, 0)
OLD_AFTER_RESTART_S = 1  # sample(restart - seconds(1))

# The four time axes of the validator figure. Above each axis is a band, ZONE_HEIGHT high,
# that holds the samples' marks
AXIS_START_X = 14
AXIS_END_X = 286  # the arrow head ends 7 further right
ZONE_HEIGHT = 46
ZONE_TOP = 6  # of the axes without dimension lines above the band
AXIS_Y = ZONE_TOP + ZONE_HEIGHT
MARK_ABOVE_AXIS = 10  # from an axis up to the center of a mark
LABEL_BELOW_AXIS = 18  # from an axis down to the baseline of a label
AXIS_HEIGHT = AXIS_Y + LABEL_BELOW_AXIS + 6
# The limits of check 4, to scale, with the two dimension lines above the band
LIMITS_START_MS = -150
LIMITS_END_MS = 80
LIMITS_PIXELS_PER_MS = 1.18
LIMITS_ZONE_TOP = 38
LIMITS_AXIS_Y = LIMITS_ZONE_TOP + ZONE_HEIGHT
LIMITS_APART = 9.5  # px from a limit to each of the two samples that are 1 µs apart around it
# Check 5 around the last valid sample's stamp, to scale up to a cut before the sample 5 s ahead
ORDER_PIXELS_PER_MS = 2.0
ORDER_LAST_X = 110  # of the last valid sample's stamp
ORDER_APART = 18  # px from the last valid sample's stamp to the sample 1 ns later
ORDER_CUT_X = 218
ORDER_FAR_AHEAD_X = 254
# The clock back at 0.02 s: to scale from the ROS time zero up to a cut
EARLY_PIXELS_PER_MS = 1.8
EARLY_ZERO_X = 36
EARLY_CUT_X = 208
EARLY_FORMER_X = 246  # of the last valid sample's stamp from before the clock went back
STACK_PITCH = 18  # px between the centers of two marks with the same stamp
# The clock back at 50 s: to scale around the ROS time, between two cuts
BACK_PIXELS_PER_MS = 0.8
BACK_OLD_X = 32  # of the sample 1 s older than the ROS time
BACK_FIRST_CUT_X = 54
BACK_NOW_X = 160
BACK_SECOND_CUT_X = 222
BACK_FUTURE_X = 254  # of the sample from before the clock went back

# test_ekf_synthetic.py's constants
STEP_S = 0.02  # STEP
START_S = 100  # START
SYNTHETIC_RESTART_S = 50  # RESTART
FUSION_STEPS = 150  # FUSION_STEPS
PHASE_STEPS = 50  # PHASE_STEPS
AFTER_RESET_STEPS = 100  # AFTER_RESET_STEPS
DIAGNOSTICS_WAIT_S = 1.5  # DIAGNOSTICS_WAIT
DELIVERY_MS = 20  # DELIVERY, wall time
WHEEL_RATE = 0.4  # WHEEL_RATE, rad/s
IMU_RATE = 0.5  # IMU_RATE, rad/s
BAD_RATE = 2.0  # BAD_RATE, rad/s
STALE_OFFSET_S = -0.5  # STALE_OFFSET, the stamp of phase 2's samples relative to the step
FUTURE_OFFSET_S = 0.5  # FUTURE_OFFSET, of phase 4's second sample
OLDER_OFFSET_S = -0.01  # OLDER_OFFSET, of phase 5's second sample
YAW_RATE_TOLERANCE = 0.01  # YAW_RATE_TOLERANCE, rad/s
WHEEL_RATE_TOLERANCE = 0.02  # WHEEL_RATE_TOLERANCE, rad/s

# What runs of the test printed: measured data, one line per run
RUNS_FILE = 'scripts/figures/data/localization-stepped-clock-test-runs.jsonl'
COMMIT_LENGTH = 7  # characters of the run's commit that the figure shows

# Phase 7 advances the clock by DIAGNOSTICS_WAIT without IMU samples
DIAGNOSTICS_STEPS = round(DIAGNOSTICS_WAIT_S / STEP_S)
# The IMU samples of one step in each phase: valid (True) or to drop (False)
PHASE_IMU_SAMPLES: Tuple[Tuple[bool, ...], ...] = (
    (True,), (False,), (True, False), (True, False), (True, False), (True,), ())
PHASES_WITH_VALID_SAMPLES = (1, 3, 4, 5, 6)
# (phase, the rate that the phase's last output has to have, ± tolerance)
ACCEPTED_AT_THE_END = (
    (1, IMU_RATE, YAW_RATE_TOLERANCE), (2, WHEEL_RATE, WHEEL_RATE_TOLERANCE),
    (3, IMU_RATE, YAW_RATE_TOLERANCE), (4, IMU_RATE, YAW_RATE_TOLERANCE),
    (5, IMU_RATE, YAW_RATE_TOLERANCE), (6, IMU_RATE, YAW_RATE_TOLERANCE),
)
UPPER_LIMIT = IMU_RATE + YAW_RATE_TOLERANCE  # assert_stays_at_most() in phases 2 to 5
UPPER_LIMIT_PHASES = (2, 5)  # first and last

# The chart of the seven phases: three rows above a plot of the yaw rate over the steps
PLOT_LEFT = 30
PLOT_RIGHT = 292
PLOT_TOP = 78
RATE_AT_TOP = 0.55  # rad/s
RATE_AT_BOTTOM = 0.36
PIXELS_PER_RAD_PER_S = 700
PLOT_BOTTOM = PLOT_TOP + round((RATE_AT_TOP - RATE_AT_BOTTOM) * PIXELS_PER_RAD_PER_S)
PHASE_ROW_Y = 14  # baseline of the phases' numbers
VALID_ROW_Y = 32  # baseline of the signs of the valid IMU samples
DROP_ROW_Y = 48  # baseline of the signs of the IMU samples to drop
ROWS_BOTTOM = 54
DOT_RADIUS = 4
CHART_HEIGHT = PLOT_BOTTOM + 36

# The drawing of one step: three nodes side by side, their messages from top to bottom. The
# numbers are those of the list in the docstring of step(), with the code that does each
STEP_CODE = (
    ('1', 'publish_inputs()'), ('2', 'wait_for_forwarded()'), ('3', 'DELIVERY'),
    ('4', 'advance_clock()'), ('5', 'step()'),
)
TEST_X = 30
VALIDATOR_X = 156
EKF_X = 270
NODE_BOX_TOP = 6
NODE_BOX_HEIGHT = 24
LABEL_ABOVE_ARROW = 6  # from an arrow up to the baseline of its label
NOTE_BELOW_ARROW = 13  # from an arrow down to the baseline of its note
STEP_HEIGHT = 310

EXTRA_RULES = (
    ('.lv-fig .cut', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.lv-fig .cut-gap', f'fill:{BG};stroke:none'),
    ('.lv-fig .former-stamp,.lv-key .former-stamp',
     f'fill:none;stroke:{FG};stroke-width:1.6;stroke-dasharray:3.5 2.5'),
    ('.lv-fig .wheels-line,.lv-key .wheels-line',
     'fill:none;stroke:var(--lv-wheels);stroke-width:2.4'),
    ('.lv-fig .imu-line,.lv-key .imu-line', 'fill:none;stroke:var(--lv-imu);stroke-width:2.4'),
    ('.lv-fig .estimate-dot,.lv-key .estimate-dot',
     f'fill:var(--lv-estimate);stroke:{BG};stroke-width:2'),
    ('.lv-fig .estimate-range,.lv-key .estimate-range',
     'fill:none;stroke:var(--lv-estimate);stroke-width:2'),
    ('.lv-fig .estimate-path,.lv-key .estimate-path',
     'fill:none;stroke:var(--lv-estimate);stroke-width:1.6;stroke-dasharray:2 2.5'),
    ('.lv-fig .limit,.lv-key .limit',
     f'fill:none;stroke:{FG};stroke-width:1.2;stroke-dasharray:5 3'),
    ('.lv-fig .phase-edge', f'fill:none;stroke:{FG_LIGHT};stroke-width:.8;'
                            'stroke-dasharray:1.5 2.5'),
    ('.lv-fig .clock-jump', f'fill:none;stroke:{FG};stroke-width:1.6'),
    ('.lv-large .lv-fig .letter', 'font-size:11px'),
    ('.lv-large .lv-fig .sign', 'font-size:12.5px;font-weight:700'),
    ('.lv-large .lv-fig .index', 'font-size:9.8px'),
    # A label that lies on a line: the page's background around the letters keeps it readable
    ('.lv-fig .halo', f'paint-order:stroke;stroke:{BG};stroke-width:4px;stroke-linejoin:round'),
    ('.lv-fig .lifeline', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:3 3'),
    ('.lv-fig .message', f'fill:none;stroke:{FG};stroke-width:1.3'),
    ('.lv-fig .message-head', f'fill:{FG};stroke:none'),
    # The keys of the HTML legends: a small drawing or a sign in front of each line
    ('.lv-legend .lv-key', 'display:inline-block;width:2em;height:1em;vertical-align:-.12em;'
                           'margin-right:.4em;overflow:visible'),
    ('.lv-legend .lv-sign', 'display:inline-block;width:2em;margin-right:.4em;'
                            'text-align:center;text-indent:0;font-weight:700'),
    ('.md-typeset ul.lv-legend.lv-keys li', 'padding-left:2.4em;text-indent:-2.4em'),
    ('.lv-legend .lv-key.lv-narrow', 'width:1.45em;margin-right:.35em'),
    # A legend whose short items stand side by side and wrap
    ('.md-typeset ul.lv-legend.lv-row', 'display:flex;flex-wrap:wrap;gap:.1em 1.1em'),
    ('.md-typeset ul.lv-legend.lv-row li', 'margin:0;padding-left:0;text-indent:0'),
)


class Run(NamedTuple):
    """What the stepped-clock test printed in a run of RUNS_FILE, yaw rates in rad/s."""

    commit: str  # of the diffbot repository, abbreviated
    end_of_phase_1: float  # the filter's estimate in the last output of the phase
    start_of_phase_2: float
    end_of_phase_2: float
    end_of_phase_6: float
    phase_6_steps: int  # the steps until the filter published again, plus AFTER_RESET_STEPS


class Phases(NamedTuple):
    """The horizontal axis of the chart: the steps of each phase, in the order of the test."""

    step_counts: Tuple[int, ...]

    def start_x(self, phase: int) -> float:
        """Return the x position of the first step of a phase, phases counted from 1."""
        steps_before = sum(self.step_counts[:phase - 1])
        return PLOT_LEFT + steps_before * (PLOT_RIGHT - PLOT_LEFT) / sum(self.step_counts)

    def end_x(self, phase: int) -> float:
        """Return the x position of the end of a phase's last step."""
        return self.start_x(phase + 1)

    def center_x(self, phase: int) -> float:
        """Return the x position of the middle of a phase."""
        return (self.start_x(phase) + self.end_x(phase)) / 2

    def start_time(self, phase: int) -> float:
        """Return the ROS time in s at the first step of a phase."""
        first_phase, clock_start = (6, SYNTHETIC_RESTART_S) if phase >= 6 else (1, START_S)
        return clock_start + sum(self.step_counts[first_phase - 1:phase - 1]) * STEP_S

    def end_time(self, phase: int) -> float:
        """Return the ROS time in s after the last step of a phase."""
        return self.start_time(phase) + self.step_counts[phase - 1] * STEP_S


def load_run() -> Run:
    """Return what the stepped-clock test printed in the last run of RUNS_FILE."""
    with open(RUNS_FILE) as runs_file:
        run = [json.loads(line) for line in runs_file if line.strip()][-1]
    # The last output of phase 6 is stamped with the time of the phase's last step
    last_stamp = run['phase_6']['last_stamp_s']
    phase_6_steps = round((last_stamp - SYNTHETIC_RESTART_S) / STEP_S) + 1
    # Phases 1, 3, 4, 5 and 6 send one valid sample in each step
    forwarded = FUSION_STEPS + 3 * PHASE_STEPS + phase_6_steps
    assert forwarded == run['validator_diagnostics']['Samples forwarded'], (
        "the steps of phase 6 don't fit the run's number of forwarded samples")
    return Run(run['commit'][:COMMIT_LENGTH],
               run['phase_1']['last_estimate_rad_s'],
               run['phase_2']['first_estimate_rad_s'],
               run['phase_2']['last_estimate_rad_s'],
               run['phase_6']['last_estimate_rad_s'],
               phase_6_steps)


def add_with_title(figure: Figure, title: str, element: str) -> None:
    """Add an element that shows title when the reader points at it."""
    figure.elements.append(f'<g><title>{escape(title)}</title>{element}</g>')


def key(drawing: str, css_class: str = 'lv-key') -> str:
    """Return a small SVG drawing for the start of a legend line, 24 by 12 units."""
    return f'<svg class="{css_class}" viewBox="0 0 24 12" aria-hidden="true">{drawing}</svg>'


def sign_key(sign: str) -> str:
    """Return a sign for the start of a legend line, as wide as a key()."""
    return f'<span class="lv-sign">{sign}</span>'


def sample_line(sample: Sample) -> str:
    """Return a legend line for a sample: its mark, what the validator does with it, and why."""
    mark_class = 'lv-mark' if sample.forwarded else 'lv-mark lv-drop'
    verdict = '✓ forwarded' if sample.forwarded else '✕ dropped'
    return (f'<span class="{mark_class}">{escape(sample.letter)}</span>'
            f'<strong>{verdict}:</strong> {escape(sample.explanation)}')


def legend(lines: Sequence[str], css_class: str = 'lv-legend') -> str:
    """Return an HTML list with one legend line per item. Each line starts with its key."""
    items = [f'<li>{line}</li>' for line in lines]
    return f'<ul class="{css_class}">\n' + '\n'.join(items) + '\n</ul>'


def ticks_between(first_x: float, last_x: float, pitch: float) -> List[float]:
    """Return the x positions of an axis's ticks, pitch apart, from first_x up to last_x."""
    count = int((last_x - first_x) / pitch) + 1
    return [first_x + index * pitch for index in range(count)]


def draw_axis(figure: Figure, x_start: float, y: float, tick_xs: Sequence[float]) -> None:
    """Add a time axis with an arrow head, time running to the right, and its ticks."""
    end_x = AXIS_END_X + 7
    figure.path(f'M{x_start},{y} H{end_x}', 'axis')
    figure.path(f'M{end_x},{y} l-6,-3.5 M{end_x},{y} l-6,3.5', 'axis')
    for x in tick_xs:
        figure.path(f'M{x:.1f},{y} v4', 'tick')


def draw_cut(figure: Figure, x: float, y: float) -> None:
    """Add two slanted strokes on an axis where the axis leaves out a stretch of time."""
    figure.rect(x - 3.5, y - 1, 7, 2, 'cut-gap')
    figure.path(f'M{x - 5.5:.1f},{y + 5} l4,-10 M{x + 1.5:.1f},{y + 5} l4,-10', 'cut')


def draw_zones(figure: Figure, zone_top: float, x_start: float, pass_start: float,
               pass_end: float, left_label: Sequence[str], right_label: Sequence[str]) -> None:
    """Add the band above an axis: stamps to drop, stamps to forward, stamps to drop.

    A label is the lines of text for the band's upper corner on its side.
    """
    if pass_start > x_start:
        figure.rect(x_start, zone_top, pass_start - x_start, ZONE_HEIGHT, 'drop-zone')
    if AXIS_END_X > pass_end:
        figure.rect(pass_end, zone_top, AXIS_END_X - pass_end, ZONE_HEIGHT, 'drop-zone')
    figure.rect(pass_start, zone_top, pass_end - pass_start, ZONE_HEIGHT, 'pass-zone')
    for index, line in enumerate(left_label):
        figure.text(x_start + 3, zone_top + 13 + index * 12, line, 'strong')
    for index, line in enumerate(right_label):
        figure.text(AXIS_END_X - 3, zone_top + 13 + index * 12, line, 'strong', 'end')


def draw_reference(figure: Figure, x: float, axis_y: float, label: str) -> None:
    """Add the vertical line that the stamps are compared with, and its label below the axis."""
    figure.path(f'M{x:.1f},{axis_y - ZONE_HEIGHT - 4} V{axis_y + 6}', 'now')
    figure.text(x, axis_y + LABEL_BELOW_AXIS, label, 'name', 'middle')


def draw_marks(figure: Figure, samples: Sequence[Sample], axis_y: float) -> None:
    """Add the samples' marks above an axis.

    A sample with the stamp of the sample before it gets its mark above that sample's mark.
    """
    y = axis_y - MARK_ABOVE_AXIS
    last_x = None
    for sample in samples:
        y = y - STACK_PITCH if sample.x == last_x else axis_y - MARK_ABOVE_AXIS
        figure.mark(sample, sample.x, y, LARGE_MARK_RADIUS)
        last_x = sample.x


def limit_tests() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of the two tests at the limits of check 4, and their samples."""
    axis = TimeAxis(LIMITS_START_MS, LIMITS_END_MS, AXIS_START_X, LIMITS_AXIS_Y,
                    LIMITS_PIXELS_PER_MS)
    figure = Figure('lv-test-limits',
                    f'Check 4 in the tests, at the ROS time t = {NOW_S} s: a sample stamped '
                    f'exactly MAX_AGE, {MAX_AGE_S} s, before t or exactly MAX_FUTURE, '
                    f'{MAX_FUTURE_S} s, after t is forwarded, and a sample 1 µs beyond each '
                    'limit is dropped', LIMITS_AXIS_Y + LABEL_BELOW_AXIS + 6, DRAWING_WIDTH)
    oldest_x = axis.at(-1000 * MAX_AGE_S)
    newest_x = axis.at(1000 * MAX_FUTURE_S)
    now_x = axis.at(0)
    draw_zones(figure, LIMITS_ZONE_TOP, axis.x, oldest_x, newest_x, ('too old',),
               ('too far', 'ahead'))
    figure.text(AXIS_END_X + 7, 12, f'MAX_FUTURE = {MAX_FUTURE_S} s', 'mono', 'end')
    figure.text((oldest_x + now_x) / 2, 25, f'MAX_AGE = {MAX_AGE_S} s', 'mono', 'middle')
    figure.dimension(oldest_x, now_x, 31)
    figure.dimension(now_x, newest_x, 31)
    figure.path(f'M{(now_x + newest_x) / 2:.1f},28 V16', 'dimension')
    draw_axis(figure, axis.x, axis.y,
              ticks_between(axis.at(-140), AXIS_END_X, SAMPLE_PERIOD_MS * axis.pixels_per_ms))
    draw_reference(figure, now_x, axis.y, 'now: t')
    figure.text(oldest_x, axis.y + LABEL_BELOW_AXIS, f't − {MAX_AGE_S} s', 'note', 'middle')
    figure.text(newest_x + 6, axis.y + LABEL_BELOW_AXIS, f't + {MAX_FUTURE_S} s', 'note')

    samples = (
        Sample('A', oldest_x - LIMITS_APART, False,
               f'stamped t − {JUST_TOO_OLD_S:.6f} s, 1 µs older than MAX_AGE allows'),
        Sample('B', oldest_x + LIMITS_APART, True,
               f'stamped t − {MAX_AGE_S} s, exactly MAX_AGE old'),
        Sample('C', newest_x - LIMITS_APART, True,
               f'stamped t + {MAX_FUTURE_S} s, exactly MAX_FUTURE ahead'),
        Sample('D', newest_x + LIMITS_APART, False,
               f'stamped t + {JUST_TOO_FAR_AHEAD_S:.6f} s, 1 µs further ahead than MAX_FUTURE '
               'allows'),
    )
    draw_marks(figure, samples, axis.y)
    return figure, samples


def order_tests() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of the two tests of check 5, the order check, and their samples."""
    figure = Figure('lv-test-order',
                    f'Check 5 in the tests, after a valid sample stamped {NOW_S} s: a sample '
                    'with an older stamp or the same stamp is dropped, a sample stamped 1 ns '
                    'later is forwarded, and so is a later sample that follows a dropped sample '
                    f'stamped {FAR_AHEAD_S} s ahead', AXIS_HEIGHT, DRAWING_WIDTH)
    period = SAMPLE_PERIOD_MS * ORDER_PIXELS_PER_MS
    draw_zones(figure, ZONE_TOP, AXIS_START_X, ORDER_LAST_X, AXIS_END_X, ('not later',),
               ('later',))
    draw_axis(figure, AXIS_START_X, AXIS_Y,
              ticks_between(ORDER_LAST_X - 2 * period, ORDER_CUT_X - 8, period))
    draw_cut(figure, ORDER_CUT_X, AXIS_Y)
    draw_reference(figure, ORDER_LAST_X, AXIS_Y, f'{NOW_S} s: the last valid stamp')
    figure.text(ORDER_FAR_AHEAD_X, AXIS_Y + LABEL_BELOW_AXIS, f'{NOW_S + FAR_AHEAD_S} s', 'note',
                'middle')

    samples = (
        Sample('E', ORDER_LAST_X - 1000 * ORDER_OLDER_S * ORDER_PIXELS_PER_MS, False,
               f'stamped {ORDER_OLDER_S} s before {NOW_S} s. The sample arrived out of order'),
        Sample('F', ORDER_LAST_X, False, f'stamped {NOW_S} s again. The sample came twice'),
        Sample('G', ORDER_LAST_X + ORDER_APART, True,
               f'stamped {ORDER_LATER_NS} ns after {NOW_S} s, the smallest step of a stamp'),
        Sample('H', ORDER_FAR_AHEAD_X, False,
               f'stamped {NOW_S + FAR_AHEAD_S} s. Check 4 drops it as too far ahead'),
        Sample('I', ORDER_LAST_X + 1000 * MAX_FUTURE_S * ORDER_PIXELS_PER_MS, True,
               f'sent after H, stamped {NOW_S + MAX_FUTURE_S} s. The validator compares the '
               f'stamp with {NOW_S} s, not with the stamp of H'),
    )
    draw_marks(figure, samples, AXIS_Y)
    return figure, samples


def early_clock_tests() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of the test whose clock goes back to EARLY_RESTART_S, and its samples.

    The axis starts at the ROS time zero.
    """
    figure = Figure('lv-test-clock-early',
                    f'After the ROS time went back from {NOW_S} s to {EARLY_RESTART_S} s, '
                    f'check 5 starts over: a sample stamped {EARLY_RESTART_S} s is forwarded, '
                    f'although the last valid stamp was {NOW_S} s, and a second sample with the '
                    'same stamp is dropped', AXIS_HEIGHT, DRAWING_WIDTH)
    period = SAMPLE_PERIOD_MS * EARLY_PIXELS_PER_MS
    now_x = EARLY_ZERO_X + 1000 * EARLY_RESTART_S * EARLY_PIXELS_PER_MS
    newest_x = now_x + 1000 * MAX_FUTURE_S * EARLY_PIXELS_PER_MS
    draw_zones(figure, ZONE_TOP, EARLY_ZERO_X, EARLY_ZERO_X, newest_x, (), ('too far ahead',))
    draw_axis(figure, EARLY_ZERO_X, AXIS_Y,
              ticks_between(EARLY_ZERO_X + period, EARLY_CUT_X - 8, period))
    figure.path(f'M{EARLY_ZERO_X},{AXIS_Y - 5} v10', 'axis')
    figure.text(EARLY_ZERO_X - 5, AXIS_Y + 4, '0 s', 'note', 'end')
    draw_cut(figure, EARLY_CUT_X, AXIS_Y)
    draw_reference(figure, now_x, AXIS_Y, f'now: t = {EARLY_RESTART_S} s')
    add_with_title(
        figure, f'{NOW_S} s: the stamp of the last valid sample before the clock went back',
        f'<path class="former-stamp" d="M{EARLY_FORMER_X},{AXIS_Y - 26} V{AXIS_Y + 6}"/>')
    figure.text(EARLY_FORMER_X, AXIS_Y + LABEL_BELOW_AXIS, f'{NOW_S} s', 'note', 'middle')

    samples = (
        Sample('J', now_x, True,
               f'stamped {EARLY_RESTART_S} s, the new ROS time. The stamp is far older than '
               f'the last valid stamp, {NOW_S} s. The validator forgot that stamp when the '
               'clock went back'),
        Sample('K', now_x, False,
               f'stamped {EARLY_RESTART_S} s again, sent {REPEAT_DELAY_MS} ms later. The stamp '
               'is not later than the stamp of J'),
    )
    draw_marks(figure, samples, AXIS_Y)
    return figure, samples


def clock_back_tests() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of the test whose clock goes back to RESTART_S, and its samples."""
    figure = Figure('lv-test-clock-back',
                    f'After the ROS time went back from {NOW_S} s to {RESTART_S} s, check 4 '
                    f'uses the new time: a sample stamped {RESTART_S - OLD_AFTER_RESTART_S} s '
                    f'is too old, and a sample stamped {NOW_S + MAX_FUTURE_S} s is too far '
                    'ahead', AXIS_HEIGHT, DRAWING_WIDTH)
    oldest_x = BACK_NOW_X - 1000 * MAX_AGE_S * BACK_PIXELS_PER_MS
    newest_x = BACK_NOW_X + 1000 * MAX_FUTURE_S * BACK_PIXELS_PER_MS
    period = SAMPLE_PERIOD_MS * BACK_PIXELS_PER_MS
    draw_zones(figure, ZONE_TOP, AXIS_START_X, oldest_x, newest_x, ('too old',),
               ('too far ahead',))
    draw_axis(figure, AXIS_START_X, AXIS_Y,
              ticks_between(oldest_x - period, BACK_SECOND_CUT_X - 8, period))
    draw_cut(figure, BACK_FIRST_CUT_X, AXIS_Y)
    draw_cut(figure, BACK_SECOND_CUT_X, AXIS_Y)
    draw_reference(figure, BACK_NOW_X, AXIS_Y, f'now: t = {RESTART_S} s')
    old_stamp = RESTART_S - OLD_AFTER_RESTART_S
    future_stamp = NOW_S + MAX_FUTURE_S
    figure.text(BACK_OLD_X, AXIS_Y + LABEL_BELOW_AXIS, f'{old_stamp} s', 'note', 'middle')
    figure.text(BACK_FUTURE_X, AXIS_Y + LABEL_BELOW_AXIS, f'{future_stamp} s', 'note', 'middle')

    samples = (
        Sample('L', BACK_OLD_X, False,
               f'stamped {old_stamp} s, {OLD_AFTER_RESTART_S} s older than the new ROS time'),
        Sample('M', BACK_FUTURE_X, False,
               f'stamped {future_stamp} s, {future_stamp - RESTART_S:g} s ahead of the new ROS '
               f'time. At the ROS time {NOW_S} s, this stamp was valid'),
    )
    draw_marks(figure, samples, AXIS_Y)
    return figure, samples


def write_validator_tests() -> None:
    """Write the include with the samples that the validator's unit tests use."""
    limits_figure, limits_samples = limit_tests()
    order_figure, order_samples = order_tests()
    early_figure, early_samples = early_clock_tests()
    back_figure, back_samples = clock_back_tests()
    former_stamp_line = (
        key('<path class="former-stamp" d="M12,-3 V15"/>', 'lv-key lv-narrow')
        + f'{NOW_S} s, the stamp of the last valid sample before the clock went back')
    parts = [
        band(f'Check 4 at the ROS time t = {NOW_S} s: the two limits'),
        limits_figure.svg(),
        legend([sample_line(sample) for sample in limits_samples]),
        band(f'Check 5 after a valid sample stamped {NOW_S} s'),
        order_figure.svg(),
        legend([sample_line(sample) for sample in order_samples]),
        band(f'The clock goes back from {NOW_S} s to {EARLY_RESTART_S} s: check 5 starts over'),
        early_figure.svg(),
        legend([sample_line(sample) for sample in early_samples] + [former_stamp_line]),
        band(f'The clock goes back from {NOW_S} s to {RESTART_S} s: check 4 uses the new time'),
        back_figure.svg(),
        legend([sample_line(sample) for sample in back_samples]),
    ]
    write_parts(
        'includes/localization-test-validator.html', parts,
        'The samples that the unit tests give the validator, on four time axes. Time runs to '
        f"the right, and one tick is {SAMPLE_PERIOD_MS / 1000:.2f} s, the IMU's sample period "
        'at 50 Hz. A solid green mark is a sample that the validator has to forward, a dashed '
        'red mark one that it has to drop. Samples that are 1 µs or 1 ns apart are drawn '
        'apart. Two slanted strokes show where an axis leaves out a stretch of time.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def rate_y(rate: float) -> float:
    """Return the y position of a yaw rate in rad/s."""
    return PLOT_BOTTOM - (rate - RATE_AT_BOTTOM) * PIXELS_PER_RAD_PER_S


def draw_phase_rows(figure: Figure, phases: Phases) -> None:
    """Add the rows above the plot: the phases' numbers, and the IMU samples of one step.

    The valid samples have a row of their own, above the row of the samples to drop.
    """
    between_the_sample_rows = (VALID_ROW_Y + DROP_ROW_Y) / 2 - 1
    figure.text(2, PHASE_ROW_Y, 'Phase', 'note')
    figure.text(2, VALID_ROW_Y - 1, 'IMU valid', 'note')
    figure.text(2, DROP_ROW_Y - 1, 'IMU to drop', 'note')
    for phase in range(1, len(phases.step_counts) + 1):
        center = phases.center_x(phase)
        figure.text(center, PHASE_ROW_Y, str(phase), 'strong', 'middle')
        samples = PHASE_IMU_SAMPLES[phase - 1]
        if not samples:
            figure.text(center, between_the_sample_rows, 'none', 'note', 'middle')
        for valid in samples:
            title = (f'Phase {phase}: a valid sample that reports {IMU_RATE} rad/s, in every '
                     'step. The validator forwards it.' if valid else
                     f'Phase {phase}: a sample that reports {BAD_RATE} rad/s, in every step. '
                     'The validator has to drop it.')
            add_with_title(figure, title,
                           f'<text class="sign" x="{center:.1f}" '
                           f'y="{VALID_ROW_Y if valid else DROP_ROW_Y}" text-anchor="middle">'
                           f'{"✓" if valid else "✕"}</text>')
        if phase > 1:
            x = phases.start_x(phase)
            figure.path(f'M{x:.1f},3 V{ROWS_BOTTOM}', 'phase-edge')
            # In the plot, an edge below the upper limit's label starts at the limit's line
            below_the_label = UPPER_LIMIT_PHASES[0] < phase <= UPPER_LIMIT_PHASES[1]
            top = rate_y(UPPER_LIMIT) if below_the_label else PLOT_TOP
            if phase != 6:
                figure.path(f'M{x:.1f},{top:.1f} V{PLOT_BOTTOM}', 'phase-edge')
    jump_x = phases.start_x(6)
    add_with_title(
        figure, f'The clock goes back from {phases.end_time(5):g} s to {SYNTHETIC_RESTART_S} s',
        f'<path class="clock-jump" d="M{jump_x:.1f},3 V{PLOT_BOTTOM + 4}"/>')


def draw_rate_axes(figure: Figure, phases: Phases) -> None:
    """Add the plot's two axes: the yaw rate, and the ROS time at the phases' first steps."""
    figure.elements.append(
        f'<text class="note" x="2" y="{PLOT_TOP - 9}">Yaw rate ω<tspan class="index" dy="2.5">z'
        '</tspan><tspan dy="-2.5"> in rad/s</tspan></text>')
    figure.path(f'M{PLOT_LEFT},{PLOT_TOP - 4} V{PLOT_BOTTOM} H{PLOT_RIGHT}', 'axis')
    for rate in (WHEEL_RATE, IMU_RATE):
        figure.path(f'M{PLOT_LEFT - 4},{rate_y(rate):.1f} h4', 'tick')
        figure.text(PLOT_LEFT - 6, rate_y(rate) + 4, f'{rate}', 'note', 'end')
    label_y = PLOT_BOTTOM + 15
    figure.text(PLOT_LEFT, label_y, f'{phases.start_time(1):g}', 'note')
    for phase in (2, 3, 4, 5, 7):
        figure.path(f'M{phases.start_x(phase):.1f},{PLOT_BOTTOM} v4', 'tick')
        if phase != 5:  # a label at the first step of phase 5 would touch its neighbors
            figure.text(phases.start_x(phase), label_y, f'{phases.start_time(phase):g}',
                        'note', 'middle')
    jump_x = phases.start_x(6)
    figure.text(jump_x - 4, label_y, f'{phases.end_time(5):g}', 'note', 'end')
    figure.text(jump_x + 4, label_y, f'{phases.start_time(6):g}', 'strong')
    figure.text(PLOT_LEFT, label_y + 14, 'ROS time t in s', 'note')


def draw_inputs(figure: Figure, phases: Phases) -> None:
    """Add the yaw rates that the wheel odometry and the valid IMU samples report."""
    wheels_y = rate_y(WHEEL_RATE)
    imu_y = rate_y(IMU_RATE)
    add_with_title(figure, f'Wheel odometry: {WHEEL_RATE} rad/s in every step',
                   f'<path class="wheels-line" d="M{PLOT_LEFT},{wheels_y:.1f} H{PLOT_RIGHT}"/>')
    figure.text(PLOT_LEFT + 5, wheels_y - 6, 'wheels', 'strong')
    for phase in PHASES_WITH_VALID_SAMPLES:
        add_with_title(
            figure, f'Phase {phase}: the valid IMU samples report {IMU_RATE} rad/s',
            f'<path class="imu-line" d="M{phases.start_x(phase):.1f},{imu_y:.1f} '
            f'H{phases.end_x(phase):.1f}"/>')
    figure.text(PLOT_LEFT + 5, imu_y - 6, 'IMU', 'strong')


def draw_assertions(figure: Figure, phases: Phases) -> None:
    """Add what the test asserts: the range of each phase's last output, and the upper limit."""
    first_phase, last_phase = UPPER_LIMIT_PHASES
    limit_y = rate_y(UPPER_LIMIT)
    start_x = phases.start_x(first_phase)
    end_x = phases.end_x(last_phase)
    add_with_title(
        figure, f'Phases {first_phase} to {last_phase}: no output that the test keeps, one per '
        f'step, may be above {UPPER_LIMIT:g} rad/s',
        f'<path class="limit" d="M{start_x:.1f},{limit_y:.1f} H{end_x:.1f}"/>')
    figure.text((start_x + end_x) / 2, limit_y - 18, 'kept outputs', '', 'middle')
    figure.text((start_x + end_x) / 2, limit_y - 5, f'at most {UPPER_LIMIT:g}', '', 'middle')
    for phase, rate, tolerance in ACCEPTED_AT_THE_END:
        x = phases.end_x(phase) - 2.5
        top = rate_y(rate + tolerance)
        bottom = rate_y(rate - tolerance)
        add_with_title(
            figure, f'Phase {phase}: the last output has to be {rate} ± {tolerance} rad/s',
            f'<path class="estimate-range" d="M{x:.1f},{top:.1f} V{bottom:.1f} '
            f'M{x - 3.5:.1f},{top:.1f} h7 M{x - 3.5:.1f},{bottom:.1f} h7"/>')


def draw_run(figure: Figure, phases: Phases, run: Run) -> None:
    """Add the estimates that the test printed in the run."""
    dots = (
        (phases.end_x(1) - 2.5, run.end_of_phase_1, 'the last output of phase 1'),
        (phases.end_x(2) - 2.5, run.end_of_phase_2, 'the last output of phase 2'),
        (phases.end_x(6) - 2.5, run.end_of_phase_6, 'the last output of phase 6'),
    )
    figure.path(f'M{phases.start_x(2):.1f},{rate_y(run.start_of_phase_2):.1f} '
                f'L{dots[1][0]:.1f},{rate_y(run.end_of_phase_2):.1f}', 'estimate-path')
    for x, rate, what in dots:
        add_with_title(figure, f'{rate} rad/s, {what} in the run at the commit {run.commit}',
                       f'<circle class="estimate-dot" cx="{x:.1f}" cy="{rate_y(rate):.1f}" '
                       f'r="{DOT_RADIUS}"/>')
    below_the_imu_line = rate_y(IMU_RATE) + 18
    figure.text(dots[0][0] - 8, below_the_imu_line, 'estimate', 'strong', 'end')
    figure.text(dots[0][0] - 8, below_the_imu_line + 13, f'{run.end_of_phase_1}', 'strong',
                'end')
    figure.text(dots[1][0] + 8, rate_y(WHEEL_RATE) + 16, f'{run.end_of_phase_2}', 'strong halo')
    figure.text(phases.start_x(6) + 5, below_the_imu_line, f'{run.end_of_phase_6}', 'strong')


def phases_chart(phases: Phases, run: Run) -> Figure:
    """Return the chart of the seven phases of the stepped-clock test."""
    figure = Figure('lv-test-stepped-clock',
                    'The seven phases of the stepped-clock test from left to right: the yaw '
                    'rate that the wheel odometry and the valid IMU samples report, the samples '
                    'that the validator has to drop, the ranges that the test accepts for the '
                    "filter's estimate, and the estimates that a run printed", CHART_HEIGHT,
                    DRAWING_WIDTH)
    draw_phase_rows(figure, phases)
    draw_rate_axes(figure, phases)
    draw_inputs(figure, phases)
    draw_assertions(figure, phases)
    draw_run(figure, phases, run)
    return figure


def chart_legends(run: Run) -> List[str]:
    """Return the HTML parts below the chart: the inputs, the assertions, and the run."""
    first_phase, last_phase = UPPER_LIMIT_PHASES
    inputs = (
        key('<path class="wheels-line" d="M1,6 H23"/>')
        + f'wheel odometry: {WHEEL_RATE} rad/s in every step of every phase',
        key('<path class="imu-line" d="M1,6 H23"/>')
        + f'IMU: the valid samples report {IMU_RATE} rad/s',
        sign_key('✓') + 'a valid IMU sample in each step of the phase. The validator '
        'forwards it',
        sign_key('✕') + 'an IMU sample that the validator has to drop, in each step of the '
        f'phase. It reports {BAD_RATE} rad/s, which is beyond the chart. Its stamp is '
        f'{-STALE_OFFSET_S} s before the time of the step in phase 2, the stamp of the valid '
        f'sample in phase 3, {FUTURE_OFFSET_S} s after the time of the step in phase 4, and '
        f'{-OLDER_OFFSET_S} s before the time of the step in phase 5. In phases 3 to 5, the '
        'test sends it after the valid sample',
    )
    assertions = (
        key('<path class="estimate-range" d="M12,1 V11 M8.5,1 h7 M8.5,11 h7"/>')
        + f'the range for the last output of a phase: {IMU_RATE} ± {YAW_RATE_TOLERANCE} rad/s, '
        f'and {WHEEL_RATE} ± {WHEEL_RATE_TOLERANCE} rad/s in phase 2',
        key('<path class="limit" d="M1,6 H23"/>')
        + f'the limit in phases {first_phase} to {last_phase}: no output that the test keeps, '
        f'one per step, may be above {UPPER_LIMIT:g} rad/s',
    )
    results = (
        key(f'<circle class="estimate-dot" cx="12" cy="6" r="{DOT_RADIUS}"/>')
        + f"the filter's estimate in the last output of phase 1, {run.end_of_phase_1} rad/s, "
        f'of phase 2, {run.end_of_phase_2} rad/s, and of phase 6, {run.end_of_phase_6} rad/s',
        key('<path class="estimate-path" d="M1,6 H23"/>')
        + f'joins the first estimate of phase 2, {run.start_of_phase_2} rad/s, and the last '
        'one. The run printed only these two',
    )
    return [
        band('What each step sends'),
        legend(inputs, 'lv-legend lv-keys'),
        band('What the test asserts'),
        legend(assertions, 'lv-legend lv-keys'),
        band(f'What the test printed in a run at the diffbot commit {run.commit}'),
        legend(results, 'lv-legend lv-keys'),
    ]


def write_stepped_clock_test() -> None:
    """Write the include with the seven phases of the stepped-clock test."""
    run = load_run()
    phases = Phases((FUSION_STEPS, PHASE_STEPS, PHASE_STEPS, PHASE_STEPS, PHASE_STEPS,
                     run.phase_6_steps, DIAGNOSTICS_STEPS))
    write_parts(
        'includes/localization-test-stepped-clock.html',
        [phases_chart(phases, run).svg()] + chart_legends(run),
        'The stepped-clock test from left to right, one step every '
        f'{round(1000 * STEP_S)} ms of ROS time. The rows "IMU valid" and "IMU to drop" '
        'show the samples of one step of each phase. At the solid vertical line, the clock '
        'goes back from '
        f'{phases.end_time(5):g} s to {SYNTHETIC_RESTART_S} s.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def draw_node(figure: Figure, x: float, width: float, name: str, title: str) -> None:
    """Add a node of the step drawing: its box at the top and its line down the drawing."""
    figure.path(f'M{x},{NODE_BOX_TOP + NODE_BOX_HEIGHT} V{STEP_HEIGHT - 6}', 'lifeline')
    figure.elements.append(f'<g><title>{escape(title)}</title>')
    figure.box(x - width / 2, NODE_BOX_TOP, width, NODE_BOX_HEIGHT, ((name, 'name'),))
    figure.elements.append('</g>')


def draw_label(figure: Figure, x: float, y: float, number: str, label: str) -> None:
    """Add a label that starts at x: the number of the step in bold, then the text."""
    if number:
        figure.text(x, y, number, 'strong')
        x += 13
    figure.text(x, y, label)


def draw_message(figure: Figure, y: float, x_from: float, x_to: float, number: str,
                 label: str, note: str = '', also_to: Sequence[float] = ()) -> None:
    """Add a message as an arrow from one node's line to another's.

    The label stands above the arrow and the note below it, both at the arrow's left end.
    also_to are the lines of other nodes that get the same message on the arrow's way.
    """
    direction = 1 if x_to > x_from else -1
    figure.path(f'M{x_from},{y} H{x_to - direction}', 'message')
    for x in (x_to,) + tuple(also_to):
        figure.path(f'M{x},{y} l{-8 * direction},-4 v8 z', 'message-head')
    label_x = min(x_from, x_to) + 6
    draw_label(figure, label_x, y - LABEL_ABOVE_ARROW, number, label)
    if note:
        figure.text(label_x + (13 if number else 0), y + NOTE_BELOW_ARROW, note, 'note')


def one_step() -> Figure:
    """Return the drawing of the messages of one step with a valid IMU sample."""
    figure = Figure('lv-test-step',
                    'One step of the stepped-clock test, from top to bottom: the test publishes '
                    'the wheel odometry and an IMU sample, the validator forwards the valid '
                    'sample to the EKF and to the test, the test waits and publishes the new '
                    'clock, and the EKF publishes its output, the filtered odometry',
                    STEP_HEIGHT, DRAWING_WIDTH)
    draw_node(figure, TEST_X, 48, 'test', 'The test node, test_ekf_synthetic.py')
    draw_node(figure, VALIDATOR_X, 84, 'validator',
              'The IMU sample validator, the node imu_validator')
    draw_node(figure, EKF_X, 52, 'EKF', "robot_localization's EKF, the node ekf_localization")
    draw_message(figure, 62, TEST_X, EKF_X, '1', 'wheel odometry', 'stamped t')
    draw_message(figure, 104, TEST_X, VALIDATOR_X, '1', 'IMU sample', 'stamped t')
    draw_message(figure, 144, VALIDATOR_X, EKF_X, '2', 'valid sample')
    draw_message(figure, 172, VALIDATOR_X, TEST_X, '2', 'valid sample')
    figure.rect(TEST_X - 9, 186, 126, 36, 'item', corner_radius=4)
    draw_label(figure, TEST_X - 3, 201, '3', 'waits DELIVERY')
    figure.text(TEST_X + 10, 214, f'{DELIVERY_MS} ms of wall time', 'note')
    draw_message(figure, 252, TEST_X, EKF_X, '4', 'new clock',
                 f't + {round(1000 * STEP_S)} ms', also_to=(VALIDATOR_X,))
    draw_message(figure, 290, EKF_X, TEST_X, '5', 'EKF output', 'stamped t or later')
    return figure


def step_code_legend() -> str:
    """Return the HTML key below the step drawing: each number with the code that does it."""
    return legend([f'<strong>{number}</strong> <code>{escape(code)}</code>'
                   for number, code in STEP_CODE], 'lv-legend lv-row')


def write_one_step() -> None:
    """Write the include with the messages of one step of the stepped-clock test."""
    write_parts(
        'includes/localization-test-stepped-clock-step.html',
        [one_step().svg(), step_code_legend()],
        'One step of the test with a valid IMU sample, from top to bottom. t is the ROS time '
        'of the step. The numbers are those of the list in the docstring of '
        '<code>step()</code>. The key below the drawing names the code of each number.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def main() -> None:
    """Write the three includes of the test pages."""
    write_validator_tests()
    write_one_step()
    write_stepped_clock_test()


if __name__ == '__main__':
    main()
