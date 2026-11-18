"""Generate the figures of the two test pages of diffbot_control's EKF tests as inline SVG.

The pages are docs/packages/imu/robot_localization/tests/validator.md and stepped-clock.md. The
script writes seven includes that follow the site's light and dark colors. The first three show
the samples that the unit tests in diffbot_control/test/test_imu_validator.py of the diffbot
repository give the validator, on time axes, one include for each section of validator.md that
has a figure. The samples' letters A to M run through the three includes:

- includes/localization-test-validator-limits.html: check 4, the two time limits, A to D.
- includes/localization-test-validator-order.html: check 5, around the last valid sample's
  stamp, E to I.
- includes/localization-test-validator-clock-back.html: the clock goes back, to 0.02 s with J
  and K, and to 50 s with L and M.
- includes/localization-test-stepped-clock-step.html: the messages of one step of
  diffbot_control/test/test_ekf_synthetic.py between the test, the validator and the EKF,
  numbered as the docstring of the test's step() numbers what one step does.
- includes/localization-test-stepped-clock.html: the seven phases of that test's scenario
  invalid_imu_samples, with the yaw rate that the wheel odometry and the IMU report in each
  step, what the test asserts about the filter's estimate, and the estimates that a run of the
  test printed.
- includes/localization-test-stepped-clock-robot-rates.html: the two phases of the scenario
  robot_rates, the IMU in every step and the wheel odometry in every fifth, then the IMU
  stopped, with what the test asserts and the filter's outputs in a recorded run.
- includes/localization-test-stepped-clock-between-wheels.html: what the filter publishes
  between two wheel messages in that scenario, with the IMU running and with the IMU stopped,
  from the same recorded run.

Every drawing is 300 units wide and carries short labels only, so that its text is readable on
a phone. Headings, legends and results are HTML text around the drawings. Figure, Sample, the
CSS rules and the HTML helpers come from localization_figures.py.

The tests' constants are copied here, each with the constant's name, because this repository
can't import the tests. What runs of the stepped-clock test printed or recorded is measured
data. It is read from scripts/figures/data/localization-stepped-clock-test-runs.jsonl, one line
per run, with the run's scenario, the commit of the diffbot repository, the date and the
environment. A line without a scenario is a run of invalid_imu_samples from before the test
had a second scenario. A line of robot_rates also holds every EKF output of a recording run:
its stamp, yaw rate and forward speed, which rosbag recorded, and the ROS time of the filter's
update that published it, with whether a prediction at the sensor timeout set its stamp, from
robot_localization's own debug trace. The figures show the file's last run of each scenario.

Run from the repository root after changing a figure:

    python3 scripts/figures/localization_control_test_figures.py
"""

import json
from typing import Any
from typing import Dict
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
# up to 20rem, 400 px on a phone, and the text sizes of LARGE_RULES are made for this width
DRAWING_WIDTH = CHECKS_WIDTH

# test_imu_validator.py's constants
NOW_S = 100  # NOW, the ROS time of most checks
MAX_AGE_S = 0.1  # MAX_AGE
MAX_FUTURE_S = 0.02  # MAX_FUTURE
ONE_MICROSECOND_S = 1e-6  # ONE_MICROSECOND
JUST_TOO_OLD_S = MAX_AGE_S + ONE_MICROSECOND_S  # JUST_TOO_OLD
JUST_TOO_FAR_AHEAD_S = MAX_FUTURE_S + ONE_MICROSECOND_S  # JUST_TOO_FAR_AHEAD
# test_validator_rejects_a_sample_not_later_than_the_last_valid_one
ORDER_OLDER_S = 0.01  # make_imu_sample(NOW - seconds(0.01))
ORDER_LATER_NS = 1  # make_imu_sample(NOW + rospy.Duration(0, 1))
# test_validator_accepts_a_new_sample_after_rejecting_a_future_one
FAR_AHEAD_S = 5  # make_imu_sample(NOW + seconds(5))
# test_validator_accepts_a_stamp_once_after_the_clock_goes_back
EARLY_RESTART_S = 0.02  # restart = rospy.Time(0, 20000000)
REPEAT_DELAY_MS = 1  # restart + seconds(0.001), the ROS time of the second sample
# test_validator_rejects_stamps_outside_time_limits_after_the_clock_goes_back
RESTART_S = 50  # restart = rospy.Time(50, 0)
OLD_AFTER_RESTART_S = 1  # make_imu_sample(restart - seconds(1))

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
STEPS_PER_WHEEL_MESSAGE = 5  # STEPS_PER_WHEEL_MESSAGE, the robot's wheel odometry period
IMU_STOPPED_WHEEL_PERIODS = 20  # IMU_STOPPED_WHEEL_PERIODS
DELIVERY_MS = 20  # DELIVERY, wall time
WHEEL_RATE = 0.4  # WHEEL_RATE, rad/s
IMU_RATE = 0.5  # IMU_RATE, rad/s
BAD_RATE = 2.0  # BAD_RATE, rad/s
STALE_OFFSET_S = -0.5  # STALE_OFFSET, the stamp of phase 2's samples relative to the step
FUTURE_OFFSET_S = 0.5  # FUTURE_OFFSET, of phase 4's second sample
OLDER_OFFSET_S = -0.01  # OLDER_OFFSET, of phase 5's second sample
YAW_RATE_TOLERANCE = 0.01  # YAW_RATE_TOLERANCE, rad/s
WHEEL_RATE_TOLERANCE = 0.02  # WHEEL_RATE_TOLERANCE, rad/s
SPEED = 0.1  # SPEED, m/s, the wheels' forward speed
CHANGED_SPEED = 0.12  # CHANGED_SPEED, m/s, from SPEED_CHANGE_STEP of robot_rates' phase 1 on
SPEED_CHANGE_STEP = 100  # SPEED_CHANGE_STEP

# What runs of the test printed or recorded: measured data, one line per run
RUNS_FILE = 'scripts/figures/data/localization-stepped-clock-test-runs.jsonl'
COMMIT_LENGTH = 7  # characters of the run's commit that the figure shows
# The scenarios' names, as the test's argument --scenario and the data file name them
INVALID_IMU_SAMPLES = 'invalid_imu_samples'
ROBOT_RATES = 'robot_rates'

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
SMALL_DOT_RADIUS = 2.6
CHART_HEIGHT = PLOT_BOTTOM + 36

# robot_rates: the wheel odometry's period, and ekf.yaml's sensor_timeout, in s
WHEEL_PERIOD_S = STEPS_PER_WHEEL_MESSAGE * STEP_S
SENSOR_TIMEOUT_S = 0.1
# The drawings of three wheel periods: an axis in ms from the first wheel message's stamp
BETWEEN_WHEELS_MESSAGES = 3
BETWEEN_WHEELS_END_MS = 1000 * ((BETWEEN_WHEELS_MESSAGES - 1) * WHEEL_PERIOD_S + 2 * STEP_S)
BETWEEN_WHEELS_AXIS_X = 50
BETWEEN_WHEELS_PIXELS_PER_MS = 0.9
OUTPUT_DOT_RADIUS = 4.6
# The chart of robot_rates leaves room for the rows' names left of the plot
ROBOT_RATES_PLOT_LEFT = 50
BRACKET_ROW = 20  # the height of the row above the chart of robot_rates with the bracket
BRACKET_Y = 9  # of the bracket's line in that row

# The drawing of the validator's decision about one sample: the steps of drop_reason() from top
# to bottom in the left column, each drop reason to the right of its step, and the two ends at
# the bottom
DECISION_LEFT_X = 2
DECISION_LEFT_WIDTH = 154
DECISION_RIGHT_X = 172
DECISION_RIGHT_WIDTH = 126
DECISION_REASON_WIDTH = 112  # of a reason's box: the line that collects the reasons runs beside
DECISION_COLLECT_X = 291  # of that line
DECISION_LINE_HEIGHT = 13  # between the baselines of a box's lines
DECISION_GAP = 12  # between two boxes, for an arrow
DECISION_START_HEIGHT = 22  # of the box of the arriving sample
DECISION_END_LINES = 3  # of each of the two boxes at the bottom

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
NUMBER_WIDTH = 13  # from the start of a step's number to the start of its label
LABEL_ABOVE_ARROW = 6  # from an arrow up to the baseline of its label
NOTE_BELOW_ARROW = 13  # from an arrow down to the baseline of its note
STEP_HEIGHT = 330

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
    ('.lv-large .lv-fig .index', 'font-size:11px'),
    # A label that lies on a line: the page's background around the letters keeps it readable
    ('.lv-fig .halo', f'paint-order:stroke;stroke:{BG};stroke-width:4px;stroke-linejoin:round'),
    ('.lv-fig .lifeline', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:3 3'),
    ('.lv-fig .message,.lv-key .message', f'fill:none;stroke:{FG};stroke-width:1.3'),
    ('.lv-fig .message-head,.lv-key .message-head', f'fill:{FG};stroke:none'),
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


# The marker of the first estimate of phase 2, in the chart of the seven phases only
PHASES_RULES = (
    ('.lv-fig .estimate-first,.lv-key .estimate-first',
     f'fill:{BG};stroke:var(--lv-estimate);stroke-width:1.8'),
)
FIRST_MARKER_OFFSET = 7.5  # from the start of phase 2 to the diamond's center
FIRST_MARKER_HALF_WIDTH = 3.5


# The marks of the two figures of robot_rates only
ROBOT_RATES_RULES = (
    ('.lv-fig .wheels-tick,.lv-key .wheels-tick',
     'fill:none;stroke:var(--lv-wheels);stroke-width:1.6'),
    ('.lv-fig .imu-tick,.lv-key .imu-tick', 'fill:none;stroke:var(--lv-imu);stroke-width:.8'),
    ('.lv-fig .wheels-dot,.lv-key .wheels-dot', f'fill:var(--lv-wheels);stroke:{BG};'
                                                'stroke-width:1'),
    ('.lv-fig .imu-dot,.lv-key .imu-dot', f'fill:var(--lv-imu);stroke:{BG};stroke-width:.8'),
    ('.lv-fig .estimate-trace,.lv-key .estimate-trace',
     'fill:none;stroke:var(--lv-estimate);stroke-width:1.6'),
    ('.lv-key .tick', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.4'),
)


class DecisionStep(NamedTuple):
    """One step of the validator's decision about a sample, as the drawing shows it."""

    number: str  # the check's number on the Localization page, '' for a step that isn't one
    lines: Tuple[str, ...]  # the step's text: what lets the sample go on
    reason_lines: Tuple[str, ...]  # the DropReason member in lines, () for a step that drops none


# drop_reason() and the six functions of sample_checks in imu_validator.py, in their order
DECISION_STEPS = (
    DecisionStep('', ('ROS time t ≠ 0',), ('NO_CLOCK',)),
    DecisionStep('', ('if the clock went back:', 'clear the last valid', "sample's stamp and",
                      'arrival time'), ()),
    DecisionStep('1', ('frame is imu_link',), ('WRONG_FRAME',)),
    DecisionStep('2', ('values are finite',), ('NOT_FINITE',)),
    DecisionStep('3', ('covariance is valid',),
                 ('INVALID_ANGULAR_', 'VELOCITY_', 'COVARIANCE')),
    DecisionStep('4', ('not too far ahead',), ('FROM_THE_FUTURE',)),
    DecisionStep('4', ('not too old',), ('TOO_OLD',)),
    DecisionStep('5', ('no last valid stamp,', 'or this stamp is later'), ('NOT_NEWER',)),
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


def last_run_of(scenario: str) -> Dict[str, Any]:
    """Return the last line of RUNS_FILE for a scenario, as a dictionary."""
    with open(RUNS_FILE) as runs_file:
        runs = [json.loads(line) for line in runs_file if line.strip()]
    return [run for run in runs if run.get('scenario', INVALID_IMU_SAMPLES) == scenario][-1]


def load_run() -> Run:
    """Return what the stepped-clock test printed in the last run of invalid_imu_samples."""
    run = last_run_of(INVALID_IMU_SAMPLES)
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


def draw_gap_label(figure: Figure, x: float, axis_y: float, gap: str,
                   anchor: str = 'middle') -> None:
    """Add the true gap between two marks that the drawing sets apart, above them at x."""
    figure.text(x, axis_y - MARK_ABOVE_AXIS - LARGE_MARK_RADIUS - 4, gap, 'note halo', anchor)


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
    # The label at C and D ends at D's right edge, clear of the band's label "ahead"
    draw_gap_label(figure, oldest_x, axis.y, '1 µs, enlarged')
    draw_gap_label(figure, newest_x + LIMITS_APART + LARGE_MARK_RADIUS, axis.y, '1 µs, enlarged',
                   'end')
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
    draw_gap_label(figure, ORDER_LAST_X + ORDER_APART / 2, AXIS_Y, f'{ORDER_LATER_NS} ns')
    # H lies in the band of check 5's later stamps, but check 4 drops H before check 5 runs
    figure.text(AXIS_END_X, AXIS_Y - MARK_ABOVE_AXIS - LARGE_MARK_RADIUS - 4,
                'check 4: too far ahead', 'note halo', 'end')
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
               f'the last valid stamp, {NOW_S} s. The validator cleared that stamp when the '
               'clock went back'),
        Sample('K', now_x, False,
               f'stamped {EARLY_RESTART_S} s again, sent {REPEAT_DELAY_MS} ms later. The stamp '
               'is not later than the stamp of J'),
    )
    draw_marks(figure, samples, AXIS_Y)
    # The marks of J and K stand on top of each other: their order is written beside them
    label_x = now_x + LARGE_MARK_RADIUS + 5
    figure.text(label_x, AXIS_Y - MARK_ABOVE_AXIS + 4, '1st arrival', 'note halo')
    figure.text(label_x, AXIS_Y - MARK_ABOVE_AXIS - STACK_PITCH + 4,
                f'repeat, {REPEAT_DELAY_MS} ms later', 'note halo')
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


def decision_box_height(line_count: int) -> float:
    """Return the height of a box of the decision drawing with line_count lines of text."""
    return line_count * DECISION_LINE_HEIGHT + 9


def draw_down_arrow(figure: Figure, x: float, top: float, bottom: float) -> None:
    """Add an arrow from top down to bottom at x."""
    figure.path(f'M{x:.1f},{top:.1f} V{bottom - 1:.1f}', 'message')
    figure.path(f'M{x:.1f},{bottom:.1f} l-4,-8 h8 z', 'message-head')


def draw_decision_step(figure: Figure, top: float, step: DecisionStep) -> float:
    """Add one step of the decision with its drop reason, and return the step's height."""
    line_count = max(len(step.lines), len(step.reason_lines))
    height = decision_box_height(line_count)
    figure.rect(DECISION_LEFT_X, top, DECISION_LEFT_WIDTH, height, 'item', corner_radius=4)
    text_top = top + height / 2 - (len(step.lines) - 1) * DECISION_LINE_HEIGHT / 2 + 4
    first_line, *other_lines = step.lines
    draw_label(figure, DECISION_LEFT_X + 7, text_top, step.number, first_line)
    # The other lines start below the first line's text, which stands right of the number
    other_lines_x = DECISION_LEFT_X + 7 + (NUMBER_WIDTH if step.number else 0)
    for index, line in enumerate(other_lines, start=1):
        figure.text(other_lines_x, text_top + index * DECISION_LINE_HEIGHT, line)
    if step.reason_lines:
        middle_y = top + height / 2
        figure.path(f'M{DECISION_LEFT_X + DECISION_LEFT_WIDTH},{middle_y:.1f} '
                    f'H{DECISION_RIGHT_X - 1}', 'message')
        figure.path(f'M{DECISION_RIGHT_X},{middle_y:.1f} l-8,-4 v8 z', 'message-head')
        figure.text((DECISION_LEFT_X + DECISION_LEFT_WIDTH + DECISION_RIGHT_X) / 2 - 1,
                    middle_y - 5, 'no', 'note', 'middle')
        figure.rect(DECISION_RIGHT_X, top, DECISION_REASON_WIDTH, height, 'drop-mark',
                    corner_radius=4)
        # The decision ends with this reason: a short line joins the collecting line
        figure.path(f'M{DECISION_RIGHT_X + DECISION_REASON_WIDTH},{middle_y:.1f} '
                    f'H{DECISION_COLLECT_X}', 'message')
        reason_top = (middle_y - (len(step.reason_lines) - 1) * DECISION_LINE_HEIGHT / 2 + 4)
        for index, line in enumerate(step.reason_lines):
            figure.text(DECISION_RIGHT_X + DECISION_REASON_WIDTH / 2,
                        reason_top + index * DECISION_LINE_HEIGHT, line, 'mono', 'middle')
    return height


def draw_decision_end(figure: Figure, x: float, top: float, width: float, css_class: str,
                      lines: Sequence[Tuple[str, str]]) -> None:
    """Add one of the two ends of the decision: a box with centered lines of text."""
    figure.rect(x, top, width, decision_box_height(len(lines)), css_class, corner_radius=4)
    for index, (line, line_class) in enumerate(lines):
        figure.text(x + width / 2, top + 15 + index * DECISION_LINE_HEIGHT, line, line_class,
                    'middle')


def validator_decision() -> Figure:
    """Return the drawing of the validator's decision about one sample, from top to bottom."""
    figure = Figure('lv-test-decision',
                    "The validator's decision about a sample that arrives at the ROS time t, "
                    'from top to bottom: the clock, the clock check, and checks 1 to 5. A '
                    'sample that fails a step is dropped with the reason to the right of the '
                    'step, and the node counts the reason. For a sample that passes every '
                    "step, the validator records the sample's stamp and arrival time, and the "
                    'node forwards the sample', 0, DRAWING_WIDTH)
    left_middle = DECISION_LEFT_X + DECISION_LEFT_WIDTH / 2
    figure.rect(DECISION_LEFT_X, 2, DRAWING_WIDTH - 2 * DECISION_LEFT_X, DECISION_START_HEIGHT,
                'item', corner_radius=4)
    figure.text(DRAWING_WIDTH / 2, 17, 'a sample arrives at the ROS time t', 'strong', 'middle')
    top: float = 2 + DECISION_START_HEIGHT + DECISION_GAP
    heights = [decision_box_height(max(len(step.lines), len(step.reason_lines)))
               for step in DECISION_STEPS]
    end_top = top + sum(heights) + DECISION_GAP * len(DECISION_STEPS)
    # The line that collects the drop reasons runs beside their boxes down to the end box
    first_reason_y = top + heights[0] / 2
    figure.path(f'M{DECISION_COLLECT_X},{first_reason_y:.1f} V{end_top - 1:.1f}', 'message')
    figure.path(f'M{DECISION_COLLECT_X},{end_top:.1f} l-4,-8 h8 z', 'message-head')
    draw_down_arrow(figure, left_middle, top - DECISION_GAP, top)
    for step in DECISION_STEPS:
        height = draw_decision_step(figure, top, step)
        draw_down_arrow(figure, left_middle, top + height, top + height + DECISION_GAP)
        top += height + DECISION_GAP
    draw_decision_end(figure, DECISION_LEFT_X, end_top, DECISION_LEFT_WIDTH, 'pass-mark',
                      (('valid: returns None', 'strong'), ('records the stamp and t', 'note'),
                       ('node forwards the sample', 'note')))
    draw_decision_end(figure, DECISION_RIGHT_X, end_top, DECISION_RIGHT_WIDTH, 'drop-mark',
                      (('returns the reason', 'strong'), ('node counts the drop', 'note'),
                       ('and throttles warnings', 'note')))
    figure.height = round(end_top + decision_box_height(DECISION_END_LINES) + 2)
    return figure


def write_validator_decision() -> None:
    """Write the include of the validator's decision about one sample."""
    write_parts(
        'includes/localization-test-validator-decision.html',
        [validator_decision().svg()],
        'The decision of <code>drop_reason_or_record()</code> about one sample, from top to '
        'bottom. A check lets the sample go down only if its condition holds. Otherwise the '
        'arrow "no" leads to the drop reason, and the decision ends with this reason. The clock '
        'step drops no sample: the decision always goes on after it.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def write_validator_limits() -> None:
    """Write the include of the section on check 4: the samples A to D at the two limits."""
    figure, samples = limit_tests()
    write_parts(
        'includes/localization-test-validator-limits.html',
        [figure.svg(), legend([sample_line(sample) for sample in samples])],
        f'Check 4 in two tests, at the ROS time t = {NOW_S} s. A lies 1 µs before B, and D 1 µs '
        'after C. The figure draws them apart.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def write_validator_order() -> None:
    """Write the include of the section on check 5: the samples E to I."""
    figure, samples = order_tests()
    write_parts(
        'includes/localization-test-validator-order.html',
        [figure.svg(), legend([sample_line(sample) for sample in samples])],
        f'Check 5 in two tests, after a valid sample stamped {NOW_S} s. G lies {ORDER_LATER_NS} '
        'ns after F. The figure draws them apart.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def write_validator_clock_back() -> None:
    """Write the include of the section on a clock that goes back: the samples J to M."""
    early_figure, early_samples = early_clock_tests()
    back_figure, back_samples = clock_back_tests()
    former_stamp_line = (
        key('<path class="former-stamp" d="M12,-3 V15"/>', 'lv-key lv-narrow')
        + f'{NOW_S} s, the stamp of the last valid sample before the clock went back')
    parts = [
        band(f'The clock goes back from {NOW_S} s to {EARLY_RESTART_S} s: check 5 starts over'),
        early_figure.svg(),
        legend([sample_line(sample) for sample in early_samples] + [former_stamp_line]),
        band(f'The clock goes back from {NOW_S} s to {RESTART_S} s: check 4 uses the new time'),
        back_figure.svg(),
        legend([sample_line(sample) for sample in back_samples]),
    ]
    write_parts(
        'includes/localization-test-validator-clock-back.html', parts,
        f'Two tests in which the ROS time goes back from {NOW_S} s. The bands on each axis are '
        'those of check 4 at the new ROS time.',
        RULES + LARGE_RULES + EXTRA_RULES, 'lv-large')


def write_validator_tests() -> None:
    """Write the four includes of the validator's page: its decision, and the tests' samples."""
    write_validator_decision()
    write_validator_limits()
    write_validator_order()
    write_validator_clock_back()


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
    # The first output of phase 2 is one step after the last of phase 1: its marker stands just
    # right of that dot. The run printed no estimate between the two of phase 2
    first_x = phases.start_x(2) + FIRST_MARKER_OFFSET
    first_y = rate_y(run.start_of_phase_2)
    add_with_title(figure, f'{run.start_of_phase_2} rad/s, the first output of phase 2 in the '
                   f'run at the commit {run.commit}',
                   f'<path class="estimate-first" d="{diamond(first_x, first_y)}"/>')
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


def diamond(x: float, y: float) -> str:
    """Return the path of a diamond with its center at x, y, the marker of a first estimate."""
    half = FIRST_MARKER_HALF_WIDTH
    return f'M{x:.1f},{y - half:.1f} l{half},{half} l{-half},{half} l{-half},{-half} z'


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
        key(f'<path class="estimate-first" d="{diamond(12, 6)}"/>')
        + f"the filter's estimate in the first output of phase 2, {run.start_of_phase_2} "
        "rad/s, drawn a little right of the start of phase 2, so that it doesn't hide the dot "
        'of phase 1. The run printed only the first and the last estimate of phase 2, so the '
        'chart has no line between them',
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
        RULES + LARGE_RULES + EXTRA_RULES + PHASES_RULES, 'lv-large')


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
        x += NUMBER_WIDTH
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
        # A long note can cross a node's line: the halo keeps it readable
        figure.text(label_x + (NUMBER_WIDTH if number else 0), y + NOTE_BELOW_ARROW, note,
                    'note halo')


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
    draw_message(figure, 172, VALIDATOR_X, TEST_X, '2', 'valid sample',
                 'wall clock runs')
    figure.rect(TEST_X - 9, 196, 126, 48, 'item', corner_radius=4)
    draw_label(figure, TEST_X - 3, 211, '3', 'waits DELIVERY')
    figure.text(TEST_X + 10, 224, f'{DELIVERY_MS} ms of wall time', 'note')
    figure.text(TEST_X + 10, 237, 'ROS time stays t', 'note')
    draw_message(figure, 274, TEST_X, EKF_X, '4', 'new clock',
                 f'ROS time t + {round(1000 * STEP_S)} ms', also_to=(VALIDATOR_X,))
    draw_message(figure, 312, EKF_X, TEST_X, '5', 'EKF output',
                 f'stamped t to t + {round(1000 * STEP_S)} ms')
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


class Output(NamedTuple):
    """An EKF output of a recording run, times in s of ROS time."""

    stamp: float
    published: float  # the ROS time of the filter's update that published the output
    predicted: bool  # whether that update's prediction at the sensor timeout set the stamp
    yaw_rate: float  # rad/s


class RobotRatesRun(NamedTuple):
    """A recorded run of the scenario robot_rates, times in s of ROS time, yaw rates in rad/s."""

    commit: str  # of the diffbot repository, abbreviated
    last_imu_stamp: float
    imu_stamps: Tuple[float, ...]  # every IMU sample's stamp, one per step of phase 1
    wheel_stamps: Tuple[float, ...]  # the wheel messages of phase 1 and of phase 2's periods
    outputs: Tuple[Output, ...]
    end_of_phase_1: float  # printed: the estimate in the last output of phase 1
    end_of_phase_2: float  # printed: the estimate in the last output of phase 2

    def phase_1(self) -> Tuple[Output, ...]:
        """Return the outputs while the IMU ran."""
        return tuple(output for output in self.outputs if output.stamp <= self.last_imu_stamp)

    def end(self) -> float:
        """Return the end of the last measured wheel period of phase 2, where the chart ends."""
        return self.wheel_stamps[-1] + WHEEL_PERIOD_S

    def phase_2(self) -> Tuple[Output, ...]:
        """Return the outputs after the IMU stopped that the test checks.

        As in the test, these are the outputs stamped before the end of the last wheel period.
        """
        return tuple(output for output in self.outputs
                     if self.last_imu_stamp < output.stamp < self.end() - STEP_S / 2)


def imu_input_grid(first: float, last: float, count: int) -> Tuple[float, ...]:
    """Return the stamps of the recorded IMU samples from the first and last stamp and the count.

    The test stamps the samples one STEP apart, so the grid is counted in whole ms, without the
    rounding errors of repeated additions.
    """
    first_ms = round(1000 * first)
    step_ms = round(1000 * STEP_S)
    stamps = tuple((first_ms + index * step_ms) / 1000 for index in range(count))
    assert abs(stamps[-1] - last) < 1e-6, 'the IMU samples are one step apart'
    return stamps


def wheel_stamps_of_the_periods(stamps: Sequence[float],
                                last_imu_stamp: float) -> Tuple[float, ...]:
    """Return the wheel messages' stamps of phase 1 and of the wheel periods of phase 2.

    The test runs one more wheel period after the measured ones. Its wheel message is stamped at
    the end of the last measured period, and the figures leave it out.
    """
    phase_1_count = sum(1 for stamp in stamps if stamp <= last_imu_stamp)
    periods = tuple(stamps[:phase_1_count + IMU_STOPPED_WHEEL_PERIODS])
    after = stamps[len(periods):]
    assert len(after) <= 1 and all(abs(stamp - periods[-1] - WHEEL_PERIOD_S) < 1e-6
                                   for stamp in after), (
        'at most one wheel message after the periods, at the end of the last one')
    return periods


def load_robot_rates_run() -> RobotRatesRun:
    """Return the last recorded run of robot_rates in RUNS_FILE, checked against its printout."""
    run = last_run_of(ROBOT_RATES)
    outputs = tuple(Output(stamp, published, set_by == 'timeout', yaw_rate)
                    for stamp, published, set_by, yaw_rate, _ in run['outputs'])
    for output in outputs:
        # The drawings place an output at the update that published it. The filter publishes
        # a fused measurement in the update after its stamp, a prediction in the update at it
        expected = 0 if output.predicted else STEP_S
        assert abs(output.published - output.stamp - expected) < 1e-6, (
            f'the output stamped {output.stamp} s, published at {output.published} s')
    last_imu_stamp = run['imu_stamps_s'][1]
    loaded = RobotRatesRun(run['commit'][:COMMIT_LENGTH], last_imu_stamp,
                           imu_input_grid(*run['imu_stamps_s']),
                           wheel_stamps_of_the_periods(run['wheel_stamps_s'], last_imu_stamp),
                           outputs,
                           run['phase_1']['last_estimate_rad_s'],
                           run['phase_2']['last_estimate_rad_s'])
    phase_2 = loaded.phase_2()
    assert len(loaded.phase_1()) == FUSION_STEPS == len(loaded.imu_stamps), (
        'phase 1 has one IMU sample and one output per step')
    assert len(phase_2) == IMU_STOPPED_WHEEL_PERIODS == run['phase_2']['output_count']
    assert [output.stamp for output in phase_2] == list(loaded.wheel_stamps[-len(phase_2):]), (
        'phase 2 has one output per wheel message, stamped with its time')
    assert round(loaded.phase_1()[-1].yaw_rate, 4) == loaded.end_of_phase_1
    assert round(phase_2[-1].yaw_rate, 4) == loaded.end_of_phase_2
    return loaded


def robot_rates_x(stamp: float, run: RobotRatesRun) -> float:
    """Return the x position of a ROS time in the chart of robot_rates."""
    return (ROBOT_RATES_PLOT_LEFT
            + (stamp - START_S) * (PLOT_RIGHT - ROBOT_RATES_PLOT_LEFT) / (run.end() - START_S))


def draw_robot_rates_rows(figure: Figure, run: RobotRatesRun) -> None:
    """Add the rows above the plot: the phases, the wheel messages and the IMU samples."""
    phase_2_x = robot_rates_x(run.last_imu_stamp + STEP_S, run)
    figure.text(2, PHASE_ROW_Y, 'Phase', 'note')
    figure.text((ROBOT_RATES_PLOT_LEFT + phase_2_x) / 2, PHASE_ROW_Y, '1', 'strong', 'middle')
    figure.text((phase_2_x + PLOT_RIGHT) / 2, PHASE_ROW_Y, '2', 'strong', 'middle')
    figure.text(2, VALID_ROW_Y - 1, 'wheels', 'note')
    figure.text(2, DROP_ROW_Y - 1, 'IMU', 'note')
    wheel_ticks = ' '.join(f'M{robot_rates_x(stamp, run):.1f},{VALID_ROW_Y - 9} v9'
                           for stamp in run.wheel_stamps)
    add_with_title(figure, f'A wheel message every {round(1000 * WHEEL_PERIOD_S)} ms',
                   f'<path class="wheels-tick" d="{wheel_ticks}"/>')
    imu_ticks = ' '.join(f'M{robot_rates_x(stamp, run):.1f},{DROP_ROW_Y - 9} v9'
                         for stamp in run.imu_stamps)
    add_with_title(figure, f'Phase 1: a valid IMU sample every {round(1000 * STEP_S)} ms',
                   f'<path class="imu-tick" d="{imu_ticks}"/>')
    figure.text((phase_2_x + PLOT_RIGHT) / 2, DROP_ROW_Y - 1, 'stopped', 'note', 'middle')
    figure.path(f'M{phase_2_x:.1f},3 V{PLOT_BOTTOM}', 'phase-edge')


def draw_robot_rates_axes(figure: Figure, run: RobotRatesRun) -> None:
    """Add the plot's two axes: the yaw rate, and the ROS time at the phases' edges."""
    figure.elements.append(
        f'<text class="note" x="2" y="{PLOT_TOP - 9}">Yaw rate ω<tspan class="index" dy="2.5">z'
        '</tspan><tspan dy="-2.5"> in rad/s</tspan></text>')
    left = ROBOT_RATES_PLOT_LEFT
    figure.path(f'M{left},{PLOT_TOP - 4} V{PLOT_BOTTOM} H{PLOT_RIGHT}', 'axis')
    for rate in (WHEEL_RATE, IMU_RATE):
        figure.path(f'M{left - 4},{rate_y(rate):.1f} h4', 'tick')
        figure.text(left - 6, rate_y(rate) + 4, f'{rate}', 'note', 'end')
    label_y = PLOT_BOTTOM + 15
    phase_2_start = run.last_imu_stamp + STEP_S
    figure.text(left, label_y, f'{START_S:g}', 'note')
    for stamp in (phase_2_start, run.end()):
        figure.path(f'M{robot_rates_x(stamp, run):.1f},{PLOT_BOTTOM} v4', 'tick')
    figure.text(robot_rates_x(phase_2_start, run), label_y, f'{phase_2_start:g}', 'note',
                'middle')
    figure.text(robot_rates_x(run.end(), run), label_y, f'{run.end():g}', 'note', 'end')
    figure.text(left, label_y + 14, 'ROS time t in s', 'note')


def draw_robot_rates_inputs(figure: Figure, run: RobotRatesRun) -> None:
    """Add the yaw rates that the wheel odometry and the IMU report."""
    wheels_y = rate_y(WHEEL_RATE)
    imu_y = rate_y(IMU_RATE)
    phase_2_x = robot_rates_x(run.last_imu_stamp + STEP_S, run)
    left = ROBOT_RATES_PLOT_LEFT
    add_with_title(figure, f'Wheel odometry: {WHEEL_RATE} rad/s in every message',
                   f'<path class="wheels-line" d="M{left},{wheels_y:.1f} H{PLOT_RIGHT}"/>')
    figure.text(left + 5, wheels_y + 16, 'wheels', 'strong')
    add_with_title(figure, f'Phase 1: the IMU samples report {IMU_RATE} rad/s',
                   f'<path class="imu-line" d="M{left},{imu_y:.1f} H{phase_2_x:.1f}"/>')
    figure.text(left + 5, imu_y - 6, 'IMU', 'strong')


def draw_robot_rates_assertions(figure: Figure, run: RobotRatesRun) -> None:
    """Add the range that the test accepts for the last output of each phase."""
    ends = ((1, robot_rates_x(run.last_imu_stamp, run), IMU_RATE, YAW_RATE_TOLERANCE),
            (2, robot_rates_x(run.wheel_stamps[-1], run) + 5, WHEEL_RATE, WHEEL_RATE_TOLERANCE))
    for phase, x, rate, tolerance in ends:
        top = rate_y(rate + tolerance)
        bottom = rate_y(rate - tolerance)
        add_with_title(
            figure, f'Phase {phase}: the last output has to be {rate} ± {tolerance} rad/s',
            f'<path class="estimate-range" d="M{x:.1f},{top:.1f} V{bottom:.1f} '
            f'M{x - 3.5:.1f},{top:.1f} h7 M{x - 3.5:.1f},{bottom:.1f} h7"/>')


def draw_robot_rates_run(figure: Figure, run: RobotRatesRun) -> None:
    """Add the filter's outputs of the recorded run.

    Phase 1 is a line through one output per step. Phase 2 has a dot per output, and the
    estimate holds from one output to the next.
    """
    phase_1_points = ' L'.join(f'{robot_rates_x(output.stamp, run):.1f},'
                               f'{rate_y(output.yaw_rate):.1f}' for output in run.phase_1())
    add_with_title(figure, "Phase 1: the filter's estimate in each of its "
                   f'{len(run.phase_1())} outputs, one per step',
                   f'<path class="estimate-trace" d="M{phase_1_points}"/>')
    phase_2 = run.phase_2()
    corners = []
    for output, following in zip(phase_2, phase_2[1:] + (None,)):
        x = robot_rates_x(output.stamp, run)
        corners.append(f'{x:.1f},{rate_y(output.yaw_rate):.1f}')
        if following is not None:
            corners.append(f'{robot_rates_x(following.stamp, run):.1f},'
                           f'{rate_y(output.yaw_rate):.1f}')
    figure.path('M' + ' L'.join(corners), 'estimate-trace')
    for output in phase_2:
        add_with_title(figure, f'{output.yaw_rate} rad/s, the output stamped {output.stamp:g} s',
                       f'<circle class="estimate-dot" cx="{robot_rates_x(output.stamp, run):.1f}" '
                       f'cy="{rate_y(output.yaw_rate):.1f}" r="{SMALL_DOT_RADIUS}"/>')
    phase_2_x = robot_rates_x(run.last_imu_stamp + STEP_S, run)
    figure.text(phase_2_x - 4, rate_y(IMU_RATE) + 16, f'{run.end_of_phase_1:.4f}', 'strong',
                'end')
    figure.text(phase_2_x + 14, rate_y(phase_2[0].yaw_rate) + 2, 'estimate', 'strong')
    figure.text(robot_rates_x(run.wheel_stamps[-1], run), rate_y(WHEEL_RATE) - 10,
                f'{run.end_of_phase_2:.4f}', 'strong halo', 'end')


def add_wheel_period_bracket(figure: Figure, run: RobotRatesRun) -> None:
    """Move the drawing down and add a bracket over the first wheel period at the top.

    The ticks of the IMU samples lie closer together than a line's width on a phone, so the
    bracket says what one wheel period holds. Two thin lines lead from its ends down through the
    rows of the wheels and the IMU.
    """
    start_x = robot_rates_x(START_S, run)
    end_x = robot_rates_x(START_S + WHEEL_PERIOD_S, run)
    drawing = figure.elements
    figure.elements = []
    figure.dimension(start_x, end_x, BRACKET_Y)
    figure.path(f'M{start_x:.1f},{BRACKET_Y} V{BRACKET_ROW + DROP_ROW_Y + 2} '
                f'M{end_x:.1f},{BRACKET_Y} V{BRACKET_ROW + DROP_ROW_Y + 2}', 'dimension')
    figure.text(end_x + 5, BRACKET_Y + 4,
                f'{round(1000 * WHEEL_PERIOD_S)} ms: {STEPS_PER_WHEEL_MESSAGE} IMU samples, '
                '1 wheel message', 'note')
    figure.elements.append(f'<g transform="translate(0,{BRACKET_ROW})">')
    figure.elements.extend(drawing)
    figure.elements.append('</g>')
    figure.height += BRACKET_ROW


def robot_rates_chart(run: RobotRatesRun) -> Figure:
    """Return the chart of the two phases of robot_rates."""
    figure = Figure('lv-test-robot-rates',
                    'The two phases of the scenario robot_rates from left to right: the wheel '
                    'messages and the IMU samples, the yaw rate that they report, the ranges '
                    "that the test accepts for the filter's estimate, and the estimate in the "
                    "filter's outputs in a recorded run", CHART_HEIGHT, DRAWING_WIDTH)
    draw_robot_rates_rows(figure, run)
    draw_robot_rates_axes(figure, run)
    draw_robot_rates_inputs(figure, run)
    draw_robot_rates_assertions(figure, run)
    draw_robot_rates_run(figure, run)
    add_wheel_period_bracket(figure, run)
    return figure


def robot_rates_legends(run: RobotRatesRun) -> List[str]:
    """Return the HTML parts below the chart of robot_rates."""
    phase_2 = run.phase_2()
    wheel_messages_of_phase_1 = FUSION_STEPS // STEPS_PER_WHEEL_MESSAGE
    wheel_messages_before_the_change = SPEED_CHANGE_STEP // STEPS_PER_WHEEL_MESSAGE
    inputs = (
        key('<path class="wheels-tick" d="M12,1 v10"/>', 'lv-key lv-narrow')
        + 'a wheel message in every fifth step, '
        f'{round(1000 * WHEEL_PERIOD_S)} ms of ROS time, in both phases, up to the end of the '
        f'{IMU_STOPPED_WHEEL_PERIODS} measured wheel periods. It reports '
        f'{WHEEL_RATE} rad/s, the blue line, and a forward speed of {SPEED} m/s in the first '
        f'{wheel_messages_before_the_change} wheel messages of phase 1, {CHANGED_SPEED} m/s in '
        f'its last {wheel_messages_of_phase_1 - wheel_messages_before_the_change}, and {SPEED} '
        f'm/s again in the {IMU_STOPPED_WHEEL_PERIODS} of phase 2. The chart shows the yaw rates '
        'only',
        key('<path class="imu-tick" d="M12,1 v10"/>', 'lv-key lv-narrow')
        + f'a valid IMU sample in every step of phase 1, {round(1000 * STEP_S)} ms. It reports '
        f'{IMU_RATE} rad/s, the orange line. In phase 2, the IMU sends nothing',
    )
    assertions = (
        key('<path class="estimate-range" d="M12,1 V11 M8.5,1 h7 M8.5,11 h7"/>')
        + f'the range for the last output of a phase: {IMU_RATE} ± {YAW_RATE_TOLERANCE} rad/s '
        f'in phase 1, {WHEEL_RATE} ± {WHEEL_RATE_TOLERANCE} rad/s in phase 2',
        sign_key('=') + 'in phase 2: the header stamp of one output in each wheel period, the '
        f'{round(1000 * WHEEL_PERIOD_S)} ms from one wheel message to the next',
    )
    results = (
        key('<path class="estimate-trace" d="M1,6 H23"/>')
        + f"phase 1: the filter's estimate in each of its {len(run.phase_1())} outputs, one per "
        f'step. It stays between {min(output.yaw_rate for output in run.phase_1()):.4f} and '
        f'{max(output.yaw_rate for output in run.phase_1()):.4f} rad/s, and the last output '
        f'has {run.end_of_phase_1:.4f} rad/s',
        key(f'<circle class="estimate-dot" cx="12" cy="6" r="{SMALL_DOT_RADIUS}"/>')
        + f"phase 2: the filter's {len(phase_2)} outputs in the measured periods, each stamped "
        "with a wheel message's "
        f'time. The estimate goes from {phase_2[0].yaw_rate:.4f} to {run.end_of_phase_2:.4f} '
        'rad/s',
    )
    return [
        band('What the steps send'),
        legend(inputs, 'lv-legend lv-keys'),
        band('What the test asserts'),
        legend(assertions, 'lv-legend lv-keys'),
        band(f'What the filter published in a recorded run at the diffbot commit {run.commit}'),
        legend(results, 'lv-legend lv-keys'),
    ]


def write_robot_rates_test() -> None:
    """Write the include with the two phases of robot_rates."""
    run = load_robot_rates_run()
    write_parts(
        'includes/localization-test-stepped-clock-robot-rates.html',
        [robot_rates_chart(run).svg()] + robot_rates_legends(run),
        'The scenario robot_rates from left to right, one step every '
        f'{round(1000 * STEP_S)} ms of ROS time. At the dotted vertical line, the IMU stops.',
        RULES + LARGE_RULES + EXTRA_RULES + ROBOT_RATES_RULES, 'lv-large')


def first_window(outputs: Sequence[Output], predicted: Sequence[bool]) -> int:
    """Return the index of the first of len(predicted) outputs in that pattern."""
    pattern = list(predicted)
    for index in range(len(outputs) - len(pattern) + 1):
        window = outputs[index:index + len(pattern)]
        if [output.predicted for output in window] == pattern:
            return index
    raise ValueError(f'no outputs in the pattern {pattern}')


class RowLayout(NamedTuple):
    """The heights in a drawing of three wheel periods, from the top."""

    timeout: float  # the dimension line of sensor_timeout, 0 for none
    wheels: float
    imu: float
    outputs: float
    axis: float
    height: int


RUNNING_ROWS = RowLayout(timeout=0, wheels=16, imu=36, outputs=56, axis=78, height=102)
STOPPED_ROWS = RowLayout(timeout=20, wheels=38, imu=58, outputs=78, axis=112, height=136)


def between_wheels_axis(rows: RowLayout) -> TimeAxis:
    """Return the time axis of a drawing between wheel messages, in ms from the first one."""
    return TimeAxis(-STEP_S * 1000, BETWEEN_WHEELS_END_MS, BETWEEN_WHEELS_AXIS_X, rows.axis,
                    BETWEEN_WHEELS_PIXELS_PER_MS)


def draw_between_wheels_rows(figure: Figure, axis: TimeAxis, rows: RowLayout,
                             first_wheel_stamp: float, imu_stamps: Sequence[float]) -> None:
    """Add the rows' names, the axis with a tick per update of the filter, and the inputs.

    imu_stamps are the recorded IMU samples' stamps. Those within the axis, both ends included,
    are drawn. None in the drawing writes "stopped" in the IMU's row.
    """
    figure.text(2, rows.wheels + 4, 'wheels', 'note')
    figure.text(2, rows.imu + 4, 'IMU', 'note')
    figure.text(2, rows.outputs + 4, 'EKF', 'note')
    figure.time_axis(axis, axis.start_ms)
    for index in range(BETWEEN_WHEELS_MESSAGES):
        x = axis.at(index * WHEEL_PERIOD_S * 1000)
        figure.circle(x, rows.wheels, OUTPUT_DOT_RADIUS, 'wheels-dot')
        figure.text(x, axis.y + 16, f'{first_wheel_stamp + index * WHEEL_PERIOD_S:.1f}', 'note',
                    'middle')
    offsets_ms = [round(1000 * (stamp - first_wheel_stamp)) for stamp in imu_stamps]
    shown_ms = [offset for offset in offsets_ms if axis.start_ms <= offset <= axis.end_ms]
    for offset_ms in shown_ms:
        figure.circle(axis.at(offset_ms), rows.imu, SMALL_DOT_RADIUS + 0.6, 'imu-dot')
    if not shown_ms:
        # Between the first two wheel messages, clear of the lines down from their stamps
        figure.text(axis.at(WHEEL_PERIOD_S * 1000 / 2), rows.imu + 4, 'stopped', 'note',
                    'middle')


def draw_between_wheels_outputs(figure: Figure, axis: TimeAxis, rows: RowLayout,
                                first_wheel_stamp: float, outputs: Sequence[Output],
                                labelled: bool) -> None:
    """Add the outputs at the ROS time of the filter's update that published them.

    labelled adds under each output whether the filter fused a wheel message or predicted.
    """
    for output in outputs:
        offset_ms = (output.published - first_wheel_stamp) * 1000
        if not axis.start_ms <= offset_ms <= axis.end_ms:
            continue
        x = axis.at(offset_ms)
        add_with_title(figure, f'{output.yaw_rate} rad/s, stamped {output.stamp:g} s, '
                       f'published at {output.published:g} s',
                       f'<circle class="estimate-dot" cx="{x:.1f}" cy="{rows.outputs}" '
                       f'r="{OUTPUT_DOT_RADIUS}"/>')
        if labelled:
            label = 'predicted' if output.predicted else 'fused'
            figure.text(x, rows.outputs + 18, label, '', 'middle')
            draw_stamp_to_publication(figure, axis, rows, first_wheel_stamp, output)


def draw_stamp_to_publication(figure: Figure, axis: TimeAxis, rows: RowLayout,
                              first_wheel_stamp: float, output: Output) -> None:
    """Add a thin line down from the output's stamp, and an arrow to its publication if later.

    The stamp is the wheel message's time. A fused output is published one update later.
    """
    stamp_x = axis.at((output.stamp - first_wheel_stamp) * 1000)
    published_x = axis.at((output.published - first_wheel_stamp) * 1000)
    figure.path(f'M{stamp_x:.1f},{rows.wheels + OUTPUT_DOT_RADIUS} V{rows.outputs}',
                'dimension')
    if published_x - stamp_x > OUTPUT_DOT_RADIUS:
        tip_x = published_x - OUTPUT_DOT_RADIUS - 0.5
        figure.path(f'M{stamp_x:.1f},{rows.outputs} H{tip_x - 4:.1f}', 'message')
        figure.path(f'M{tip_x:.1f},{rows.outputs} l-5,-3 v6 z', 'message-head')


def between_wheels_running(run: RobotRatesRun) -> Figure:
    """Return the drawing of the last three wheel periods of phase 1, with the IMU running."""
    wheel_stamps = [stamp for stamp in run.wheel_stamps if stamp <= run.last_imu_stamp]
    first_wheel_stamp = wheel_stamps[-BETWEEN_WHEELS_MESSAGES]
    axis = between_wheels_axis(RUNNING_ROWS)
    figure = Figure('lv-test-between-wheels-running',
                    'Phase 1, IMU running: an IMU sample in every step, a wheel message in '
                    'every fifth, and an EKF output in every step', RUNNING_ROWS.height,
                    DRAWING_WIDTH)
    draw_between_wheels_rows(figure, axis, RUNNING_ROWS, first_wheel_stamp, run.imu_stamps)
    draw_between_wheels_outputs(figure, axis, RUNNING_ROWS, first_wheel_stamp, run.phase_1(),
                                False)
    return figure


def between_wheels_stopped(run: RobotRatesRun) -> Figure:
    """Return the drawing of the first three wheel periods of phase 2 that show both cases.

    The three outputs are fused, predicted and fused.
    """
    phase_2 = run.phase_2()
    first_wheel_stamp = phase_2[first_window(phase_2, (False, True, False))].stamp
    axis = between_wheels_axis(STOPPED_ROWS)
    figure = Figure('lv-test-between-wheels-stopped',
                    'Phase 2, IMU stopped: a wheel message in every fifth step, and one EKF '
                    "output per wheel message, stamped with the message's time: fused in the "
                    'next update, or predicted at the time of the message', STOPPED_ROWS.height,
                    DRAWING_WIDTH)
    draw_between_wheels_rows(figure, axis, STOPPED_ROWS, first_wheel_stamp, ())
    draw_between_wheels_outputs(figure, axis, STOPPED_ROWS, first_wheel_stamp, phase_2, True)
    # From the stamp of the fused wheel message, the filter's last measurement, to the update
    # that predicts
    start_x = axis.at(0)
    end_x = axis.at(SENSOR_TIMEOUT_S * 1000)
    figure.dimension(start_x, end_x, STOPPED_ROWS.timeout)
    figure.text((start_x + end_x) / 2, STOPPED_ROWS.timeout - 5, 'sensor_timeout 0.1 s',
                'note', 'middle')
    return figure


def between_wheels_legend(run: RobotRatesRun) -> str:
    """Return the HTML key below the two drawings between wheel messages."""
    lines = (
        key(f'<circle class="wheels-dot" cx="12" cy="6" r="{OUTPUT_DOT_RADIUS}"/>')
        + f'a wheel message, every {round(1000 * WHEEL_PERIOD_S)} ms of ROS time, published '
        'at its stamp. The axis labels its stamps in s',
        key(f'<circle class="imu-dot" cx="12" cy="6" r="{SMALL_DOT_RADIUS + 0.6}"/>')
        + f'a valid IMU sample, every {round(1000 * STEP_S)} ms, published at its stamp',
        key(f'<circle class="estimate-dot" cx="12" cy="6" r="{OUTPUT_DOT_RADIUS}"/>')
        + "an EKF output, at the ROS time of the filter's update that published it, from "
        "robot_localization's debug trace. The filter fuses the inputs of one step in its "
        f'update {round(1000 * STEP_S)} ms later',
        key('<path class="tick" d="M12,3 v7"/>', 'lv-key lv-narrow')
        + "a tick on the axis: one of the filter's updates, 50 per second of ROS time",
        key('<path class="message" d="M2,6 H17"/><path class="message-head" d="M22,6 l-5,-3 '
            'v6 z"/>')
        + "in phase 2: from an output's header stamp, the time of its wheel message, to the "
        f'ROS time at which the filter published it, {round(1000 * STEP_S)} ms later for a '
        'fused output. A predicted output is published at its stamp',
        sign_key('') + '<strong>fused:</strong> in the trace, the filter fused the wheel '
        "message and published the result, stamped with the message's time",
        sign_key('') + '<strong>predicted:</strong> in the trace, the filter predicted to the '
        'time of the '
        'wheel message, without a measurement, because the ROS time was 0.1 s or more, its '
        'sensor_timeout, past its last measurement. It published the prediction, stamped with '
        'that time. It fuses the wheel message in the next update, but publishes no second '
        'output with the same stamp. The next output contains the message',
    )
    return legend(lines, 'lv-legend lv-keys')


def write_between_wheels() -> None:
    """Write the include with what the filter publishes between wheel messages."""
    run = load_robot_rates_run()
    write_parts(
        'includes/localization-test-stepped-clock-between-wheels.html',
        [band('Phase 1: the IMU running'), between_wheels_running(run).svg(),
         band('Phase 2: the IMU stopped'), between_wheels_stopped(run).svg(),
         between_wheels_legend(run)],
        f'Each drawing spans {BETWEEN_WHEELS_MESSAGES} wheel messages of a phase of '
        f'robot_rates, about {round(BETWEEN_WHEELS_END_MS + 1000 * STEP_S)} ms, in a recorded '
        'run at the diffbot '
        f"commit {run.commit}, with robot_localization's debug trace, ROS time to the right.",
        RULES + LARGE_RULES + EXTRA_RULES + ROBOT_RATES_RULES, 'lv-large')


def main() -> None:
    """Write the seven includes of the test pages."""
    write_validator_tests()
    write_one_step()
    write_stepped_clock_test()
    write_robot_rates_test()
    write_between_wheels()


if __name__ == '__main__':
    main()
