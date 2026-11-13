"""Generate the figures of the Localization page as inline SVG with HTML legends.

The page is docs/packages/imu/robot_localization.md. The script writes these includes, which
follow the site's light and dark colors:

- includes/localization-odom-frame.html: where the odom frame sits. The floor from above, with
  the odom frame where the robot started, the robot's frame base_footprint, and the pose x, y
  and θ between the two.
- includes/localization-loop.html: what goes into the filter and what comes out, as a loop in
  one layout with two variants, in two tabs. The values run from the wheels' sensors and the IMU
  through the ROS nodes to the filter, and the filter's two outputs end in a sketch of RViz.
  Velocity commands go back to the wheels. The variants differ only in what reads the sensors
  and drives the wheels: the real robot's Teensy, or Gazebo.
- includes/localization-fused-values.html: the two lists odom0_config and imu0_config of
  diffbot_control/config/ekf.yaml in the diffbot repository, each as a grid of five rows of
  three, with the values that the filter fuses. The two grids stand side by side on a wide
  screen and one below the other on a phone.
- includes/localization-covariances.html: the covariances twist.covariance of the wheel
  odometry and angular_velocity_covariance of the IMU as matrices, with each element's index in
  the message's array, and which variances the filter fuses or two_d_mode replaces.
- includes/localization-stamp-check.html: robot_localization's own check of a sample's stamp,
  against the last stamp of the same input, on a time axis.
- includes/localization-time-step.html: the filter's time step Δt for a current and for a late
  IMU sample: predict and correct, or correct only.
- includes/localization-validator-form.html, localization-validator-time.html and
  localization-validator-order.html: the five checks of diffbot_control/scripts/imu_validator.py
  in three figures. Checks 1 to 3 look at the sample's form. Check 4 compares the sample's stamp
  with the ROS time, drawn on a time axis to scale. Check 5 compares the stamp with the last
  forwarded sample's. The values are those of diffbot_control/launch/ekf.launch.
- includes/localization-distance.html: the distance of robot_localization's rejection test, for
  one value as two bell curves and for two values as ellipses, with example values.

Long explanations are HTML text below a drawing, not text inside the SVG, so that they keep the
page's text size on a phone.

Figure, Sample, write_include(), time_x() and RULES are also the base of the figures on the
chapter's test pages (scripts/figures/localization_*_test_figures.py).

Run from the repository root after changing a figure:

    python3 scripts/figures/localization_figures.py
"""

import math
from typing import List
from typing import NamedTuple
from typing import Sequence
from typing import Tuple

WIDTH = 360
MARGIN = 12

# The validator's parameters in ekf.launch, and the IMU's sample period at 50 Hz
MAX_AGE_MS = 100
MAX_FUTURE_MS = 20
SAMPLE_PERIOD_MS = 20

# A time axis for figures that are WIDTH wide: the ROS time ("now") and the stamps around it
PIXELS_PER_MS = 1.5
AXIS_START_MS = -140
AXIS_END_MS = 80
AXIS_X = 16  # of AXIS_START_MS
MARK_RADIUS = 6.5

# The figure of the five checks: three drawings, each CHECKS_WIDTH wide
CHECKS_WIDTH = 300
FORM_BOX_WIDTH = 88
FORM_BOX_HEIGHT = 40
FORM_BOX_GAP = 10
WINDOW_TOP = 38
WINDOW_HEIGHT = 46
ORDER_TOP = 6
ORDER_HEIGHT = 40
ORDER_LAST_X = 150
LARGE_MARK_RADIUS = 7.5

# The two lists of ekf.yaml: 15 booleans each, five rows of three, in the order of the filter's
# state. FUSED_BY_ODOM0 and FUSED_BY_IMU0 are the positions that are true in the file.
STATE = (
    (('Position',), (('x', ''), ('y', ''), ('z', ''))),
    (('Orientation',), (('roll', ''), ('pitch', ''), ('θ', ''))),
    (('Linear', 'velocity'), (('v', 'x'), ('v', 'y'), ('v', 'z'))),
    (('Angular', 'velocity'), (('ω', 'x'), ('ω', 'y'), ('ω', 'z'))),
    (('Linear', 'acceleration'), (('a', 'x'), ('a', 'y'), ('a', 'z'))),
)
FUSED_BY_ODOM0 = (6, 7, 11)  # vx, vy and the yaw rate
FUSED_BY_IMU0 = (11,)  # the yaw rate
# z, roll, pitch, their velocities and the vertical acceleration: two_d_mode keeps them at 0
KEPT_AT_ZERO = (2, 3, 4, 8, 9, 10, 14)
GRID_LEFT = 72  # the row labels stand to the left of the grid
GRID_TOP = 36
CELL_WIDTH = 44
CELL_HEIGHT = 28
GRID_WIDTH = GRID_LEFT + 3 * CELL_WIDTH + 4
GRID_HEIGHT = GRID_TOP + len(STATE) * CELL_HEIGHT + 4
# The hatching of the cells that two_d_mode keeps at 0: thin lines at 45 degrees
HATCH = ('<defs><pattern id="lv-hatch" width="5" height="5" patternUnits="userSpaceOnUse" '
         'patternTransform="rotate(45)"><path class="hatch" d="M0,0 V5"/></pattern></defs>')

# The loop figure of what goes in and what comes out, LOOP_WIDTH wide. The wheels' values run down
# a left column and the IMU's down a right column. The commands go back up in a lane at the left
# edge, and the estimate goes down to the planner in a lane at the right edge
LOOP_WIDTH = 340
BLOCK_START = 6  # of the boxes that span the drawing
BLOCK_END = 334
COMMAND_LANE_X = 12
ESTIMATE_LANE_X = 330
INNER_START = 22  # of the EKF's box and the RViz panel, between the two lanes
INNER_END = 322
WHEELS_COLUMN_END = 186
IMU_COLUMN_START = 192
WHEELS_ARROW_X = 30
IMU_ARROW_X = 200
LABEL_GAP = 8  # between an arrow and the text next to it
LINE_HEIGHT = 15
# The rows of the loop figure, from the top
ROBOT_CENTER_X = 170
ROBOT_CENTER_Y = 92
INTERFACE_Y = 172  # the bar of what reads the sensors and drives the wheels
INTERFACE_HEIGHT = 24
NODES_Y = 280  # the drive controller and the IMU sample validator
NODES_HEIGHT = 54
INPUTS_HEIGHT = 62  # the arrows into the EKF, with the measurements' symbols
FILTER_HEIGHT = 46
OUTPUTS_HEIGHT = 80  # the arrows out of the EKF, with the estimate's symbols
RVIZ_HEIGHT = 170
COMMANDS_HEIGHT = 86  # the commands' topic and the two boxes of who sends commands
# The robot seen from above, in units from its center, with its front pointing up
BODY_HALF_WIDTH = 46
BODY_HALF_LENGTH = 58
AXLE_Y = -14  # the wheel axle lies in front of the center
WHEEL_WIDTH = 13
WHEEL_HALF_LENGTH = 20
WHEEL_X = BODY_HALF_WIDTH + 2  # of a wheel's inner side
ENCODER_X = 14  # of the right wheel's encoder, the left one is mirrored
IMU_CHIP = (-22, 14, 14)  # x, y and edge length: behind the axle, left of the center line
WHEELS_EXIT_X = ROBOT_CENTER_X - 30  # where the line of the wheels' sensors leaves the robot
# The small robot of a drawing of a pose, and the heading that such a drawing shows, in degrees
SMALL_ROBOT_SCALE = 0.42
POSE_HEADING = 35
# The figure of where the odom frame sits
ODOM_WIDTH = 340
ODOM_HEIGHT = 232

# The two covariances of the inputs' messages, as grids of their elements: each cell holds the
# element's index in the message's array, which stores the matrix row by row
COVARIANCE_CELL = 32
COVARIANCE_LABEL_WIDTH = 34  # the row labels stand to the left of the grid
COVARIANCE_TOP = 56  # the field's name, the source and the column labels stand above the grid
COVARIANCE_MIN_WIDTH = 210  # room for the long field name angular_velocity_covariance
TWIST_VARIABLES = (('v', 'x'), ('v', 'y'), ('v', 'z'), ('ω', 'x'), ('ω', 'y'), ('ω', 'z'))
ANGULAR_VELOCITY_VARIABLES = (('ω', 'x'), ('ω', 'y'), ('ω', 'z'))
# The variances that the filter fuses, and those that two_d_mode replaces, by array index
TWIST_FUSED_VARIANCES = (0, 7, 35)  # vx, vy and the yaw rate
TWIST_REPLACED_VARIANCES = (14, 21, 28)  # vz and the rates of roll and pitch
ANGULAR_VELOCITY_FUSED_VARIANCES = (8,)  # the yaw rate
ANGULAR_VELOCITY_REPLACED_VARIANCES = (0, 4)  # the rates of roll and pitch
COVARIANCE_HATCH = ('<defs><pattern id="lv-hatch-covariance" width="5" height="5" '
                    'patternUnits="userSpaceOnUse" patternTransform="rotate(45)">'
                    '<path class="hatch" d="M0,0 V5"/></pattern></defs>')

# The figures of the filter's own stamp check and of its time step: stamps in ms from the ROS
# time t, on axes that are CHECKS_WIDTH wide. Example values
INPUT_LAST_STAMP_MS = -140  # t_input, the last stamp of the IMU input that the filter took
ROW_LABEL_WIDTH = 62  # the rows' names stand to the left of the time axis
STEP_PIXELS_PER_MS = 1.6
FORWARD_FILTER_TIME_MS = -50  # t_filter when a current sample arrives
FORWARD_SAMPLE_MS = -10  # the current sample's stamp t_s
LATE_FILTER_TIME_MS = -20  # t_filter when a late sample arrives
LATE_SAMPLE_MS = -90  # the late sample's stamp t_s

# The figures of the rejection test. Example values: the prediction's standard deviation is
# chosen, the IMU's is the square root of the firmware's variance 4e-4 (rad/s)²
PREDICTED_YAW_RATE = 0.50  # rad/s
MEASURED_YAW_RATE = 0.55  # rad/s
PREDICTION_SIGMA = 0.015  # rad/s
IMU_SIGMA = 0.02  # rad/s
YAW_RATE_AXIS = (0.43, 0.62)  # rad/s, the ends of the axis
PIXELS_PER_RAD_PER_S = 1400
# Two values: the combined standard deviations of the forward speed and the yaw rate
FORWARD_SPEED_SIGMA = 0.04  # m/s
YAW_RATE_SIGMA = 0.2  # rad/s
PIXELS_PER_FORWARD_SPEED_SIGMA = 46
PIXELS_PER_YAW_RATE_SIGMA = 36
EXAMPLE_THRESHOLD = 2.0

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
BG = 'var(--md-default-bg-color)'
# Colors: forwarded or fused (good) and dropped (bad) are status colors, always shown with a
# sign and a word. Wheels, IMU and estimate are the series colors of the site's other figures.
RULES = (
    ('.md-typeset figure.lv-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.lv-figure,.lv-fig', '--lv-good:#0ca30c;--lv-bad:#d03b3b;'
                           '--lv-good-fill:rgba(12,163,12,.13);--lv-bad-fill:rgba(208,59,59,.10);'
                           '--lv-wheels:#2a78d6;--lv-imu:#eb6834;--lv-estimate:#1baf7a'),
    ('[data-md-color-scheme=slate] .lv-figure,[data-md-color-scheme=slate] .lv-fig',
     '--lv-good-fill:rgba(12,163,12,.22);--lv-bad-fill:rgba(208,59,59,.20);'
     '--lv-wheels:#3987e5;--lv-imu:#d95926;--lv-estimate:#199e70'),
    ('.lv-fig', 'width:100%;height:auto;font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('.lv-fig text', f'fill:{FG};font-size:10px'),
    ('.lv-fig .band', 'font-weight:700'),
    ('.lv-fig .name', 'font-size:11px'),
    ('.lv-fig .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                      'font-size:9.5px'),
    ('.lv-fig .note', f'font-size:9.5px;fill:{FG_LIGHT}'),
    ('.lv-fig .strong', 'font-weight:700'),
    ('.lv-fig .item', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:1'),
    ('.lv-fig .axis', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.lv-fig .tick', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    ('.lv-fig .now', f'fill:none;stroke:{FG};stroke-width:1.6'),
    ('.lv-fig .dimension', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    ('.lv-fig .pass-zone', 'fill:var(--lv-good-fill);stroke:var(--lv-good);stroke-width:1'),
    ('.lv-fig .drop-zone', 'fill:var(--lv-bad-fill);stroke:none'),
    ('.lv-fig .pass-mark', f'fill:{BG};stroke:var(--lv-good);stroke-width:1.8'),
    ('.lv-fig .drop-mark', f'fill:{BG};stroke:var(--lv-bad);stroke-width:1.8;'
                           'stroke-dasharray:2.6 1.8'),
    ('.lv-fig .letter', 'font-size:9px;font-weight:700'),
    ('.lv-fig .cell', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:.8'),
    ('.lv-fig .cell-fused', 'fill:var(--lv-good-fill);stroke:var(--lv-good);stroke-width:1.6'),
    ('.lv-fig .cell-zero', f'fill:url(#lv-hatch);stroke:{FG_LIGHT};stroke-width:.8;'
                           'stroke-dasharray:2 2'),
    ('.lv-fig .hatch', f'fill:none;stroke:{FG_LIGHT};stroke-width:.9'),
    ('.lv-fig .subscript', 'font-size:7.5px'),
)
# The figures of the Localization page itself: larger text in narrower drawings, and the long
# explanations as HTML lists
LARGE_RULES = (
    ('.md-typeset figure.lv-figure.lv-large', 'max-width:400px'),
    ('.md-typeset figure.lv-figure.lv-grids', 'max-width:520px'),
    ('.lv-large .lv-fig text,.lv-grids .lv-fig text', 'font-size:12px'),
    ('.lv-large .lv-fig .name,.lv-grids .lv-fig .name', 'font-size:12.5px'),
    ('.lv-large .lv-fig .mono,.lv-grids .lv-fig .mono', 'font-size:11px'),
    ('.lv-large .lv-fig .note,.lv-grids .lv-fig .note', 'font-size:11px'),
    ('.lv-large .lv-fig .letter', 'font-size:10.5px'),
    ('.lv-grids .lv-fig .subscript', 'font-size:9px'),
    ('.lv-grids .lv-pair', 'display:flex;flex-wrap:wrap;justify-content:center;gap:.4em 1em'),
    ('.lv-grids .lv-pair .lv-fig', 'width:236px;max-width:100%'),
    ('.md-typeset .lv-band', 'margin:.9em 0 .2em;font-weight:700;font-size:.7rem;'
                             'text-align:left'),
    ('.md-typeset ul.lv-legend', 'list-style:none;margin:.3em 0 .6em;padding:0;'
                                 'text-align:left;font-size:.7rem'),
    ('.md-typeset ul.lv-legend li', 'margin:0 0 .25em;padding-left:2em;text-indent:-2em'),
    ('.lv-legend .lv-mark', 'display:inline-block;box-sizing:border-box;width:1.6em;'
                            'height:1.6em;line-height:1.35em;border-radius:50%;'
                            'border:2px solid var(--lv-good);text-align:center;text-indent:0;'
                            'font-weight:700;font-size:.9em;margin-right:.4em'),
    ('.lv-legend .lv-mark.lv-drop', 'border:2px dashed var(--lv-bad)'),
    ('.lv-legend .lv-swatch', 'display:inline-block;box-sizing:border-box;width:1.6em;'
                              'height:1.2em;line-height:1em;vertical-align:-.2em;'
                              f'border:1px solid {FG_LIGHT};text-align:center;text-indent:0;'
                              'font-weight:700;margin-right:.4em'),
    ('.lv-legend .lv-swatch.lv-fused', 'background:var(--lv-good-fill);'
                                       'border:2px solid var(--lv-good)'),
    ('.lv-legend .lv-swatch.lv-zero', 'border-style:dashed;background:repeating-linear-gradient('
                                      '45deg,transparent 0 3px,'
                                      'var(--md-default-fg-color--lighter) 3px 4px)'),
)

# The drawings with symbols and a robot from above. The symbols are set in the fonts of the page's
# formulas, which KaTeX's style sheet loads
KATEX_MAIN = 'font-family:KaTeX_Main,"Times New Roman",serif'
KATEX_MATH = 'font-family:KaTeX_Math,"Times New Roman",serif'
DRAWING_RULES = (
    ('.md-typeset figure.lv-figure.lv-drawing', 'max-width:360px'),
    ('.lv-drawing .lv-fig text', 'font-size:12px'),
    ('.lv-drawing .lv-fig .mono', 'font-size:11.5px'),
    ('.lv-drawing .lv-fig .quiet', f'fill:{FG_LIGHT}'),
    ('.lv-drawing .lv-fig .variable', f'{KATEX_MATH};font-style:italic;font-size:14px'),
    ('.lv-drawing .lv-fig .vector', f'{KATEX_MAIN};font-weight:700;font-size:14px'),
    ('.lv-drawing .lv-fig .subscript', f'{KATEX_MAIN};font-size:11.5px'),
    ('.lv-drawing .lv-fig .index', f'{KATEX_MATH};font-style:italic'),
    ('.lv-drawing .lv-fig .robot-body', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:1.4'),
    ('.lv-drawing .lv-fig .robot-wheel', f'fill:{FG_LIGHT};stroke:none'),
    ('.lv-drawing .lv-fig .robot-part', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    # A pose from above: the odom frame's axes, the transform's line, the heading's angle and
    # the small robot. The small robot is drawn scaled down, so its outline keeps its width
    ('.lv-drawing .lv-fig .frame-axis-line', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-drawing .lv-fig .frame-axis-head', f'fill:{FG};stroke:none'),
    ('.lv-drawing .lv-fig .transform', f'fill:none;stroke:{FG};stroke-width:1.3;'
                                       'stroke-dasharray:4 3'),
    ('.lv-drawing .lv-fig .angle', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-drawing .lv-fig .pose-body', f'fill:{BG};stroke:{FG};stroke-width:1.2;'
                                       'vector-effect:non-scaling-stroke'),
    ('.lv-drawing .lv-fig .pose-wheel', f'fill:{FG};stroke:none'),
)
# The loop figure. The RViz panel has RViz's own dark background in both color schemes, so its
# lines and text have fixed light colors
RVIZ_BG = '#2f3338'
RVIZ_FG = '#eceff1'
LOOP_RULES = (
    # The badge above each of the page's two Mermaid figures of the launch files, which says
    # whether the figure is for the real robot or for the simulation
    ('.md-typeset p.lv-badge-line', 'margin:1em 0 -.4em;text-align:center'),
    ('.md-typeset .lv-badge', f'display:inline-block;padding:.05em .9em;border-radius:1em;'
                              f'background:{FG};color:{BG};font-weight:700;font-size:.68rem'),
    # A tab holds one variant's figure, so the figure needs no margin of its own at the top
    ('.md-typeset .lv-loop-tabs figure.lv-figure', 'margin-top:.6em'),
    ('.lv-loop .lv-fig .encoder', f'fill:var(--lv-wheels);stroke:{BG};stroke-width:1'),
    ('.lv-loop .lv-fig .imu-chip', f'fill:var(--lv-imu);stroke:{BG};stroke-width:1'),
    ('.lv-loop .lv-fig .motion-line', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-loop .lv-fig .motion-head', f'fill:{FG};stroke:none'),
    ('.lv-loop .lv-fig .wheels-line', 'fill:none;stroke:var(--lv-wheels);stroke-width:1.8'),
    ('.lv-loop .lv-fig .wheels-head', 'fill:var(--lv-wheels);stroke:none'),
    ('.lv-loop .lv-fig .wheels-wire', 'fill:none;stroke:var(--lv-wheels);stroke-width:1.3'),
    ('.lv-loop .lv-fig .imu-line', 'fill:none;stroke:var(--lv-imu);stroke-width:1.8'),
    ('.lv-loop .lv-fig .imu-head', 'fill:var(--lv-imu);stroke:none'),
    ('.lv-loop .lv-fig .imu-wire', 'fill:none;stroke:var(--lv-imu);stroke-width:1.3'),
    ('.lv-loop .lv-fig .estimate-line', 'fill:none;stroke:var(--lv-estimate);stroke-width:1.8'),
    ('.lv-loop .lv-fig .estimate-head', 'fill:var(--lv-estimate);stroke:none'),
    # The way back to the wheels is quieter than the way to the filter: thin, gray and dashed
    ('.lv-loop .lv-fig .command-line', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.4;'
                                       'stroke-dasharray:4 3'),
    ('.lv-loop .lv-fig .command-head', f'fill:{FG_LIGHT};stroke:none'),
    ('.lv-loop .lv-fig .estimate-lane-line', 'fill:none;stroke:var(--lv-estimate);'
                                             'stroke-width:1.3'),
    ('.lv-loop .lv-fig .estimate-lane-head', 'fill:var(--lv-estimate);stroke:none'),
    ('.lv-loop .lv-fig .hat', f'fill:none;stroke:{FG};stroke-width:1.1'),
    ('.lv-loop .lv-fig .filter', f'fill:{BG};stroke:var(--lv-estimate);stroke-width:1.8'),
    ('.lv-loop .lv-fig .gazebo', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};'
                                 'stroke-width:1;stroke-dasharray:4 3'),
    ('.lv-loop .lv-fig .rviz', f'fill:{RVIZ_BG};stroke:{FG_LIGHT};stroke-width:1'),
    ('.lv-loop .lv-fig .rviz-grid', 'fill:none;stroke:rgba(255,255,255,.16);stroke-width:1'),
    ('.lv-loop .lv-fig .rviz-scene text', f'fill:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-scene .frame-axis-line', f'stroke:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-scene .frame-axis-head', f'fill:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-scene .transform', f'stroke:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-scene .angle', f'stroke:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-scene .pose-body', f'fill:#59616b;stroke:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-scene .pose-wheel', f'fill:{RVIZ_FG}'),
    ('.lv-loop .lv-fig .rviz-pose-line', 'fill:none;stroke:#35d19a;stroke-width:2.2'),
    ('.lv-loop .lv-fig .rviz-pose-head', 'fill:#35d19a;stroke:none'),
    ('.lv-loop .lv-fig .rviz-trail-line', 'fill:none;stroke:#35d19a;stroke-width:1.6;'
                                          'opacity:.5'),
    ('.lv-loop .lv-fig .rviz-trail-head', 'fill:#35d19a;stroke:none;opacity:.5'),
    # The legend of the four kinds of arrows, as HTML under the drawing
    ('.md-typeset ul.lv-legend.lv-lines', 'display:flex;flex-wrap:wrap;justify-content:center;'
                                          'gap:0 1.3em;margin:.5em 0 0'),
    ('.md-typeset ul.lv-legend.lv-lines li', 'margin:0;padding:0;text-indent:0'),
    ('.lv-lines .lv-line', 'display:inline-block;width:1.7em;margin-right:.45em;'
                           'vertical-align:.25em;border-top:3px solid var(--lv-wheels)'),
    ('.lv-lines .lv-line.lv-from-imu', 'border-top-color:var(--lv-imu)'),
    ('.lv-lines .lv-line.lv-estimated', 'border-top-color:var(--lv-estimate)'),
    ('.lv-lines .lv-line.lv-commanded', f'border-top:2px dashed {FG_LIGHT}'),
)
# The figure of where the odom frame sits: the floor from above, in the page's colors
ODOM_RULES = (
    ('.lv-odom .lv-fig .floor', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};'
                                'stroke-width:1'),
    ('.lv-odom .lv-fig .floor-grid', 'fill:none;stroke:var(--md-default-fg-color--lightest);'
                                     'stroke-width:1'),
    ('.lv-odom .lv-fig .start-robot', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2;'
                                      'stroke-dasharray:3 2.5;vector-effect:non-scaling-stroke'),
    ('.lv-odom .lv-fig .projection', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2;'
                                     'stroke-dasharray:1.5 3'),
    ('.lv-odom .lv-fig .foot', f'fill:none;stroke:{FG};stroke-width:1.4'),
)
# The figures of the filter's checks and of the covariances: symbols in the formulas' fonts
# (DRAWING_RULES), and the series colors of the wheels, the IMU and the estimate
SYMBOL_RULES = (
    ('.md-typeset figure.lv-figure.lv-symbols', 'max-width:360px'),
    ('.lv-symbols .lv-fig .reference', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.1;'
                                       'stroke-dasharray:3 3'),
    ('.lv-symbols .lv-fig .wheels-dot', f'fill:var(--lv-wheels);stroke:{BG};stroke-width:1'),
    ('.lv-symbols .lv-fig .imu-dot', f'fill:var(--lv-imu);stroke:{BG};stroke-width:1'),
    ('.lv-symbols .lv-fig .estimate-dot', f'fill:var(--lv-estimate);stroke:{BG};'
                                          'stroke-width:1'),
    ('.lv-symbols .lv-fig .estimate-line', 'fill:none;stroke:var(--lv-estimate);'
                                           'stroke-width:1.8'),
    ('.lv-symbols .lv-fig .estimate-head', 'fill:var(--lv-estimate);stroke:none'),
    ('.lv-symbols .lv-fig .imu-line', 'fill:none;stroke:var(--lv-imu);stroke-width:1.6'),
    ('.lv-symbols .lv-fig .imu-head', 'fill:var(--lv-imu);stroke:none'),
    ('.lv-symbols .lv-fig .bell-prediction', 'fill:var(--lv-estimate);fill-opacity:.16;'
                                             'stroke:var(--lv-estimate);stroke-width:1.6'),
    ('.lv-symbols .lv-fig .bell-imu', 'fill:var(--lv-imu);fill-opacity:.16;'
                                      'stroke:var(--lv-imu);stroke-width:1.6'),
    ('.lv-symbols .lv-fig .ellipse', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.1'),
    ('.lv-symbols .lv-fig .threshold', 'fill:var(--lv-good-fill);stroke:var(--lv-good);'
                                       'stroke-width:1.6'),
    ('.lv-symbols .lv-fig .limit-box', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.1;'
                                       'stroke-dasharray:5 3'),
    ('.lv-symbols .lv-fig .cell-replaced', 'fill:url(#lv-hatch-covariance);'
                                           f'stroke:{FG_LIGHT};stroke-width:.8;'
                                           'stroke-dasharray:2 2'),
    ('.lv-symbols .lv-pair', 'display:flex;flex-wrap:wrap;justify-content:center;'
                             'align-items:flex-start;gap:.4em 1.2em'),
    ('.lv-symbols .lv-pair .lv-fig.lv-twist', 'width:262px;max-width:100%'),
    ('.lv-symbols .lv-pair .lv-fig.lv-angular', 'width:240px;max-width:100%'),
    ('.md-typeset figure.lv-figure.lv-symbols.lv-covariances', 'max-width:520px'),
)


class Sample(NamedTuple):
    """A sample that a time axis shows: where its stamp is and what the validator does with it."""

    letter: str
    x: float  # of its stamp on the axis
    forwarded: bool
    explanation: str  # for the legend under the axis


class TimeAxis(NamedTuple):
    """A horizontal time axis to scale: its range in ms and where it starts in the drawing."""

    start_ms: float
    end_ms: float
    x: float  # of start_ms
    y: float
    pixels_per_ms: float

    def at(self, milliseconds: float) -> float:
        """Return the x position of a time on the axis."""
        return self.x + (milliseconds - self.start_ms) * self.pixels_per_ms


def escape(text: str) -> str:
    """Return text with the characters escaped that have a meaning in SVG and HTML."""
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def style_element(rules: Sequence[Tuple[str, str]]) -> str:
    """Return a style element with one CSS rule per (selector, declarations) pair."""
    lines = [f'{selector}{{{declarations}}}' for selector, declarations in rules]
    return '<style>\n' + '\n'.join(lines) + '\n</style>'


def time_x(milliseconds: float) -> float:
    """Return the x position, on a WIDTH wide time axis, of a time relative to the ROS time."""
    return AXIS_X + (milliseconds - AXIS_START_MS) * PIXELS_PER_MS


class Span(NamedTuple):
    """A piece of a line of text and how it is set."""

    content: str
    # variable (italics), vector (bold), subscript (small and lowered, with index for a
    # variable as index), strong, mono, or '' for plain text
    css_class: str = ''


def variable(symbol: str, index: str = '') -> Tuple[Span, ...]:
    """Return the spans of a variable in italics, with its index lowered."""
    if index:
        return (Span(symbol, 'variable'), Span(index, 'subscript index'))
    return (Span(symbol, 'variable'),)


def listing(*variables: Tuple[Span, ...]) -> Tuple[Span, ...]:
    """Return the spans of variables in brackets with commas between them, as a vector."""
    spans: Tuple[Span, ...] = (Span('('),)
    for index, variable_spans in enumerate(variables):
        if index:
            spans += (Span(', '),)
        spans += variable_spans
    return spans + (Span(')'),)


# The symbols of the page's formulas, for the flow figures
X = variable('x')
Y = variable('y')
THETA = variable('θ')
V_X = variable('v', 'x')
V_Y = variable('v', 'y')
OMEGA_Z = variable('ω', 'z')
Z_WHEELS = (Span('z', 'vector'), Span('wheels', 'subscript'), Span(' = ')) + listing(
    V_X, V_Y, OMEGA_Z)
Z_IMU = (Span('z', 'vector'), Span('IMU', 'subscript'), Span(' = ')) + listing(OMEGA_Z)
POSE = listing(X, Y, THETA)
# The part of the estimate that the topic carries: what follows the estimate's symbol, and the
# values on the next line
PUBLISHED_INDEX = (Span('published', 'subscript'), Span(' ='))
PUBLISHED = listing(X, Y, THETA, V_X, V_Y, OMEGA_Z)


def arrow_head(x: float, y: float, direction_x: float, direction_y: float,
               size: float = 7) -> str:
    """Return the path commands of a triangle with its tip at x, y, pointing along direction."""
    length = math.hypot(direction_x, direction_y)
    unit_x, unit_y = direction_x / length, direction_y / length
    back_x, back_y = x - unit_x * size, y - unit_y * size
    half_width = size * 0.6
    return (f'M{x:.1f},{y:.1f} L{back_x - unit_y * half_width:.1f},'
            f'{back_y + unit_x * half_width:.1f} L{back_x + unit_y * half_width:.1f},'
            f'{back_y - unit_x * half_width:.1f} z')


class Figure:
    """The elements of one SVG drawing, in drawing order."""

    def __init__(self, name: str, title: str, height: int, width: int = WIDTH,
                 css_class: str = '') -> None:
        self.name = name  # unique on the page: the id of the drawing's title starts with it
        self.title = title
        self.width = width
        self.height = height
        self.css_class = css_class  # added to the SVG element's class lv-fig
        self.elements: List[str] = []

    def text(self, x: float, y: float, content: str, css_class: str = '',
             anchor: str = 'start', subscript: str = '') -> None:
        """Add a line of text at x, y, aligned by anchor (start, middle or end)."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        lowered = (f'<tspan class="subscript" dy="2.5">{escape(subscript)}</tspan>'
                   if subscript else '')
        self.elements.append(f'<text{class_attribute} x="{x:.1f}" y="{y:.1f}" '
                             f'text-anchor="{anchor}">{escape(content)}{lowered}</text>')

    def rect(self, x: float, y: float, width: float, height: float, css_class: str,
             corner_radius: float = 0) -> None:
        """Add a rectangle."""
        self.elements.append(f'<rect class="{css_class}" x="{x:.1f}" y="{y:.1f}" '
                             f'width="{width:.1f}" height="{height:.1f}" rx="{corner_radius}"/>')

    def path(self, commands: str, css_class: str) -> None:
        """Add a path from SVG path commands."""
        self.elements.append(f'<path class="{css_class}" d="{commands}"/>')

    def box(self, x: float, y: float, width: float, height: float,
            lines: Sequence[Tuple[str, str]]) -> None:
        """Add a box with centered lines of text, each a (text, CSS class) pair."""
        self.rect(x, y, width, height, 'item', corner_radius=5)
        first_baseline = y + height / 2 - (len(lines) - 1) * 6.5 + 3.5
        for index, (content, css_class) in enumerate(lines):
            self.text(x + width / 2, first_baseline + index * 13, content, css_class, 'middle')

    def dimension(self, x_start: float, x_end: float, y: float) -> None:
        """Add a horizontal dimension line with an end stroke on each side."""
        self.path(f'M{x_start:.1f},{y} H{x_end:.1f} M{x_start:.1f},{y - 3} V{y + 3} '
                  f'M{x_end:.1f},{y - 3} V{y + 3}', 'dimension')

    def mark(self, sample: Sample, x: float, y: float, radius: float = MARK_RADIUS) -> None:
        """Add a sample's mark: a circle with its letter, solid green or dashed red.

        Pointing at the mark shows what happens to the sample and why.
        """
        css_class = 'pass-mark' if sample.forwarded else 'drop-mark'
        verdict = 'forwarded' if sample.forwarded else 'dropped'
        self.elements.append(
            f'<g><title>{escape(f"Sample {sample.letter}: {verdict}. {sample.explanation}")}'
            f'</title><circle class="{css_class}" cx="{x:.1f}" cy="{y:.1f}" r="{radius}"/>'
            f'<text class="letter" x="{x:.1f}" y="{y + radius / 2:.1f}" text-anchor="middle">'
            f'{escape(sample.letter)}</text></g>')

    def time_axis(self, axis: TimeAxis, first_tick_ms: float) -> None:
        """Add an axis with an arrow at its end and a tick every sample period."""
        end_x = axis.at(axis.end_ms)
        self.path(f'M{axis.x},{axis.y} H{end_x + 7:.1f}', 'axis')
        self.path(f'M{end_x + 7:.1f},{axis.y} l-6,-3.5 M{end_x + 7:.1f},{axis.y} l-6,3.5',
                  'axis')
        tick_ms = first_tick_ms
        while tick_ms <= axis.end_ms:
            self.path(f'M{axis.at(tick_ms):.1f},{axis.y} v4', 'tick')
            tick_ms += SAMPLE_PERIOD_MS

    def rich_text(self, x: float, y: float, spans: Sequence[Span], anchor: str = 'start',
                  css_class: str = '') -> None:
        """Add a line of text from spans, with each subscript span lowered."""
        parts = []
        lowered = False
        for span in spans:
            shift = ''
            is_subscript = span.css_class.startswith('subscript')
            if is_subscript and not lowered:
                shift, lowered = ' dy="2.5"', True
            elif not is_subscript and lowered:
                shift, lowered = ' dy="-2.5"', False
            class_attribute = f' class="{span.css_class}"' if span.css_class else ''
            parts.append(f'<tspan{class_attribute}{shift}>{escape(span.content)}</tspan>')
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(f'<text{class_attribute} x="{x:.1f}" y="{y:.1f}" '
                             f'text-anchor="{anchor}">{"".join(parts)}</text>')

    def estimate_text(self, x: float, y: float, spans: Sequence[Span] = (),
                      css_class: str = '') -> None:
        """Add the estimate's symbol at x, y, a bold x with a hat, and the spans after it."""
        self.path(f'M{x + 1.2:.1f},{y - 8.6:.1f} l2.8,-2.6 l2.8,2.6', 'hat')
        self.rich_text(x, y, (Span('x', 'vector'),) + tuple(spans), css_class=css_class)

    def arrow(self, x_start: float, y_start: float, x_end: float, y_end: float,
              series: str) -> None:
        """Add a straight arrow in the color of a series, for example wheels, imu or estimate."""
        self.route(((x_start, y_start), (x_end, y_end)), series)

    def route(self, points: Sequence[Tuple[float, float]], series: str) -> None:
        """Add an arrow that runs through points, with its head at the last one."""
        (before_x, before_y), (end_x, end_y) = points[-2], points[-1]
        direction_x, direction_y = end_x - before_x, end_y - before_y
        length = math.hypot(direction_x, direction_y)
        # The line ends inside the head, so that its blunt end doesn't show at the tip
        line_end = (end_x - direction_x / length * 5, end_y - direction_y / length * 5)
        corners = ' L'.join(f'{x:.1f},{y:.1f}' for x, y in tuple(points[:-1]) + (line_end,))
        self.path(f'M{corners}', f'{series}-line')
        self.path(arrow_head(end_x, end_y, direction_x, direction_y), f'{series}-head')

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle."""
        self.elements.append(f'<circle class="{css_class}" cx="{x:.1f}" cy="{y:.1f}" '
                             f'r="{radius}"/>')

    def ellipse(self, x: float, y: float, radius_x: float, radius_y: float,
                css_class: str) -> None:
        """Add an ellipse with its center at x, y and its axes along the drawing's axes."""
        self.elements.append(f'<ellipse class="{css_class}" cx="{x:.1f}" cy="{y:.1f}" '
                             f'rx="{radius_x:.1f}" ry="{radius_y:.1f}"/>')

    def svg(self) -> str:
        """Return the drawing as an SVG element."""
        title_id = f'{self.name}-title'
        classes = f'lv-fig {self.css_class}'.strip()
        return (f'<svg class="{classes}" viewBox="0 0 {self.width} {self.height}" role="img" '
                f'aria-labelledby="{title_id}"><title id="{title_id}">{escape(self.title)}'
                '</title>\n' + '\n'.join(self.elements) + '\n</svg>')


def write_include(path: str, figure: Figure, caption: str,
                  rules: Sequence[Tuple[str, str]] = RULES) -> None:
    """Write an include with the CSS rules, one drawing and its caption."""
    write_parts(path, [figure.svg()], caption, rules)


def write_parts(path: str, parts: Sequence[str], caption: str,
                rules: Sequence[Tuple[str, str]] = RULES, css_class: str = '') -> None:
    """Write an include with the CSS rules, a figure of SVG and HTML parts, and its caption."""
    classes = f'lv-figure {css_class}'.strip()
    with open(path, 'w', encoding='utf-8') as include:
        include.write(f'{style_element(rules)}\n<figure class="{classes}">\n'
                      + '\n'.join(parts)
                      + f'\n<figcaption>{caption}</figcaption>\n</figure>\n')


def band(title: str) -> str:
    """Return the HTML heading of one part of a figure."""
    return f'<p class="lv-band">{escape(title)}</p>'


def sample_legend(samples: Sequence[Sample],
                  verdicts: Tuple[str, str] = ('✓ forwarded', '✕ dropped')) -> str:
    """Return an HTML list with one line per sample: its mark, what happens to it, and why.

    verdicts are the words for a sample that passes and for one that doesn't.
    """
    lines = []
    for sample in samples:
        mark_class = 'lv-mark' if sample.forwarded else 'lv-mark lv-drop'
        verdict = verdicts[0] if sample.forwarded else verdicts[1]
        lines.append(f'<li><span class="{mark_class}">{escape(sample.letter)}</span>'
                     f'<strong>{verdict}:</strong> {escape(sample.explanation)}</li>')
    return '<ul class="lv-legend">\n' + '\n'.join(lines) + '\n</ul>'


def form_checks() -> Figure:
    """Return the drawing of checks 1 to 3: the sample's frame, values and variances."""
    figure = Figure('lv-form', "Checks 1 to 3: the sample's frame is imu_link, its values are "
                    'finite, and the variances of its angular velocity are positive',
                    FORM_BOX_HEIGHT + 8, CHECKS_WIDTH)
    boxes = (
        (('1  Frame', 'name'), ('imu_link', 'mono')),
        (('2  Values', 'name'), ('finite', 'note')),
        (('3  Variances', 'name'), ('positive', 'note')),
    )
    for index, lines in enumerate(boxes):
        x = 8 + index * (FORM_BOX_WIDTH + FORM_BOX_GAP)
        figure.box(x, 4, FORM_BOX_WIDTH, FORM_BOX_HEIGHT, lines)
    return figure


def time_check() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of check 4 and its samples: the window around the ROS time."""
    axis = TimeAxis(-150, 80, 14, WINDOW_TOP + WINDOW_HEIGHT, 1.18)
    figure = Figure('lv-time', 'Check 4 on a time axis: a stamp passes when it is at most '
                    'max_age, 0.1 s, older than the ROS time and at most max_future, 0.02 s, '
                    'ahead of it', 108, CHECKS_WIDTH)
    now_x = axis.at(0)
    oldest_x = axis.at(-MAX_AGE_MS)
    newest_x = axis.at(MAX_FUTURE_MS)
    end_x = axis.at(axis.end_ms)

    figure.rect(axis.x, WINDOW_TOP, oldest_x - axis.x, WINDOW_HEIGHT, 'drop-zone')
    figure.rect(newest_x, WINDOW_TOP, end_x - newest_x, WINDOW_HEIGHT, 'drop-zone')
    figure.rect(oldest_x, WINDOW_TOP, newest_x - oldest_x, WINDOW_HEIGHT, 'pass-zone')
    figure.text(axis.x + 3, WINDOW_TOP + 13, 'too old', 'strong')
    figure.text(end_x - 3, WINDOW_TOP + 13, 'too far', 'strong', 'end')
    figure.text(end_x - 3, WINDOW_TOP + 25, 'ahead', 'strong', 'end')

    figure.text(end_x + 7, 12, f'max_future = {MAX_FUTURE_MS / 1000:.2f} s', 'mono', 'end')
    figure.text((oldest_x + now_x) / 2, 25, f'max_age = {MAX_AGE_MS / 1000:.1f} s', 'mono',
                'middle')
    figure.dimension(oldest_x, now_x, 31)
    figure.dimension(now_x, newest_x, 31)
    figure.path(f'M{(now_x + newest_x) / 2:.1f},28 V16', 'dimension')

    figure.time_axis(axis, -140)
    figure.path(f'M{now_x:.1f},{WINDOW_TOP - 4} V{axis.y + 6}', 'now')
    figure.text(oldest_x, axis.y + 18, f't − {MAX_AGE_MS / 1000:.1f} s', 'note', 'middle')
    figure.text(now_x, axis.y + 18, 'now: t', 'name', 'middle')
    figure.text(newest_x + 6, axis.y + 18, f't + {MAX_FUTURE_MS / 1000:.2f} s', 'note')

    samples = (
        Sample('A', axis.at(-130), False,
               'stamped 0.13 s before the ROS time. The sample waited in a buffer'),
        Sample('B', axis.at(-10), True,
               'stamped 0.01 s before the ROS time. The usual delay on the way'),
        Sample('C', axis.at(10), True,
               "stamped 0.01 s ahead of the ROS time. The sender's clock is a little ahead"),
        Sample('D', axis.at(40), False,
               'stamped 0.04 s ahead of the ROS time. That is more than max_future'),
    )
    for sample in samples:
        figure.mark(sample, sample.x, axis.y - 10, LARGE_MARK_RADIUS)
    return figure, samples


def order_check() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of check 5 and its samples: only a later stamp passes."""
    axis = TimeAxis(-120, 120, ORDER_LAST_X - 120 * 1.13, ORDER_TOP + ORDER_HEIGHT, 1.13)
    figure = Figure('lv-order', 'Check 5 on a time axis: a stamp passes when it is later than '
                    "the last forwarded sample's stamp", 68, CHECKS_WIDTH)
    end_x = axis.at(axis.end_ms)
    step = SAMPLE_PERIOD_MS * axis.pixels_per_ms

    figure.rect(axis.x, ORDER_TOP, ORDER_LAST_X - axis.x, ORDER_HEIGHT, 'drop-zone')
    figure.rect(ORDER_LAST_X, ORDER_TOP, end_x - ORDER_LAST_X, ORDER_HEIGHT, 'pass-zone')
    figure.text(axis.x + 3, ORDER_TOP + 13, 'not later', 'strong')
    figure.text(end_x - 3, ORDER_TOP + 13, 'later', 'strong', 'end')
    figure.time_axis(axis, -100)
    figure.path(f'M{ORDER_LAST_X},{ORDER_TOP - 4} V{axis.y + 6}', 'now')
    figure.text(ORDER_LAST_X, axis.y + 18, 'stamp of the last forwarded sample', 'name',
                'middle')

    samples = (
        Sample('E', ORDER_LAST_X - step, False,
               'an older stamp. The sample arrived out of order'),
        Sample('F', ORDER_LAST_X, False, 'the same stamp. The sample came twice'),
        Sample('G', ORDER_LAST_X + step, True, 'the next stamp, 0.02 s later'),
    )
    for sample in samples:
        figure.mark(sample, sample.x, axis.y - 10, LARGE_MARK_RADIUS)
    return figure, samples


def config_grid(name: str, parameter: str, source: str, fused: Sequence[int],
                with_hatch_pattern: bool) -> Figure:
    """Return one input's list as a grid of five rows of three cells, as ekf.yaml writes it."""
    figure = Figure(name, f'The list {parameter} of ekf.yaml, for the {source}, as a grid of '
                    'five rows of three', GRID_HEIGHT, GRID_WIDTH)
    if with_hatch_pattern:
        figure.elements.append(HATCH)
    center = GRID_LEFT + 1.5 * CELL_WIDTH
    figure.text(center, 13, parameter, 'mono', 'middle')
    figure.text(center, 28, source, 'note', 'middle')
    for row, (group, variables) in enumerate(STATE):
        row_middle = GRID_TOP + row * CELL_HEIGHT + CELL_HEIGHT / 2
        first_baseline = row_middle + 4 - 6 * (len(group) - 1)
        for line, word in enumerate(group):
            figure.text(GRID_LEFT - 6, first_baseline + line * 12, word, 'note', 'end')
        for column, (symbol, subscript) in enumerate(variables):
            position = 3 * row + column
            x = GRID_LEFT + column * CELL_WIDTH
            y = GRID_TOP + row * CELL_HEIGHT
            if position in KEPT_AT_ZERO:
                figure.rect(x, y, CELL_WIDTH, CELL_HEIGHT, 'cell-zero')
                figure.text(x + CELL_WIDTH / 2, row_middle + 4, symbol, '', 'middle', subscript)
            elif position in fused:
                figure.rect(x, y, CELL_WIDTH, CELL_HEIGHT, 'cell-fused')
                figure.text(x + CELL_WIDTH / 2 + 6, row_middle + 4, symbol, 'strong', 'middle',
                            subscript)
                figure.text(x + 5, row_middle + 4, '✓', 'strong')
            else:
                figure.rect(x, y, CELL_WIDTH, CELL_HEIGHT, 'cell')
                figure.text(x + CELL_WIDTH / 2, row_middle + 4, symbol, 'note', 'middle',
                            subscript)
    return figure


def write_fused_values() -> None:
    """Write the include with the two lists of ekf.yaml and what their cells mean."""
    grids = (config_grid('lv-odom0', 'odom0_config', 'wheel odometry', FUSED_BY_ODOM0, True),
             config_grid('lv-imu0', 'imu0_config', 'IMU', FUSED_BY_IMU0, False))
    legend = (
        '<ul class="lv-legend">\n'
        '<li><span class="lv-swatch lv-fused">✓</span><code>true</code>: the filter fuses this '
        'value from the input</li>\n'
        '<li><span class="lv-swatch"></span><code>false</code>: the filter ignores this value of '
        'the input</li>\n'
        '<li><span class="lv-swatch lv-zero"></span>kept at 0 by <code>two_d_mode</code>, '
        'whatever the list says</li>\n'
        '</ul>')
    pair = '<div class="lv-pair">\n' + '\n'.join(grid.svg() for grid in grids) + '\n</div>'
    write_parts(
        'includes/localization-fused-values.html', [pair, legend],
        "The two lists of <code>ekf.yaml</code>, in the file's layout: five rows of three, in "
        "the order of the filter's state. θ is the heading, the yaw angle.",
        RULES + LARGE_RULES, 'lv-grids')


CHECKS_CAPTION_END = ('A solid green mark is a forwarded sample, a dashed red mark a dropped one. '
                      'Each example assumes that the sample passes the other checks.')


def write_validator_form_checks() -> None:
    """Write the include with checks 1 to 3 of the IMU sample validator: the sample's form."""
    write_parts(
        'includes/localization-validator-form.html', [form_checks().svg()],
        "Checks 1 to 3 look at the sample's form: its frame, its values and the variances of "
        'its angular velocity.',
        RULES + LARGE_RULES, 'lv-large')


def write_validator_time_check() -> None:
    """Write the include with check 4 of the IMU sample validator: the stamp against the time."""
    figure, samples = time_check()
    write_parts(
        'includes/localization-validator-time.html', [figure.svg(), sample_legend(samples)],
        'Check 4: the stamp against the ROS time. The axis shows stamps, with time running to '
        f"the right and one tick every {SAMPLE_PERIOD_MS / 1000:.2f} s, the IMU's sample period "
        f'at 50 Hz. {CHECKS_CAPTION_END}',
        RULES + LARGE_RULES, 'lv-large')


def write_validator_order_check() -> None:
    """Write the include with check 5 of the IMU sample validator: the stamps' order."""
    figure, samples = order_check()
    write_parts(
        'includes/localization-validator-order.html', [figure.svg(), sample_legend(samples)],
        "Check 5: the stamp against the last forwarded sample's. The ticks are "
        f'{SAMPLE_PERIOD_MS / 1000:.2f} s apart. {CHECKS_CAPTION_END}',
        RULES + LARGE_RULES, 'lv-large')


def symbol(figure: Figure, x: float, y: float, letter: str, index: str, italic_index: bool,
           anchor: str = 'middle', before: str = '') -> None:
    """Add a variable with its index lowered, such as t with the index filter, after before."""
    index_class = 'subscript index' if italic_index else 'subscript'
    spans = ((Span(before),) if before else ()) + (Span(letter, 'variable'),
                                                   Span(index, index_class))
    figure.rich_text(x, y, spans, anchor)


def stamp_check() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of robot_localization's own stamp check of an input, and its samples.

    The check compares a stamp only with the last stamp of the same input, t_input, not with
    the ROS time t.
    """
    axis = TimeAxis(-190, 40, 14, WINDOW_TOP + WINDOW_HEIGHT, 1.18)
    figure = Figure('lv-stamp-check', "robot_localization's stamp check on a time axis: a stamp "
                    'passes when it is not earlier than the last stamp of the same input, '
                    'however far it lies behind the ROS time', 116, CHECKS_WIDTH)
    last_x = axis.at(INPUT_LAST_STAMP_MS)
    now_x = axis.at(0)
    end_x = axis.at(axis.end_ms)
    figure.rect(axis.x, WINDOW_TOP, last_x - axis.x, WINDOW_HEIGHT, 'drop-zone')
    figure.rect(last_x, WINDOW_TOP, end_x - last_x, WINDOW_HEIGHT, 'pass-zone')
    figure.text(axis.x + 3, WINDOW_TOP + 13, 'ignored', 'strong')
    figure.text(end_x - 3, WINDOW_TOP + 13, 'passes', 'strong', 'end')

    sample_q_ms = -110
    figure.dimension(axis.at(sample_q_ms), now_x, 26)
    figure.text((axis.at(sample_q_ms) + now_x) / 2, 20, '0.11 s', 'note', 'middle')

    figure.time_axis(axis, -180)
    figure.path(f'M{last_x:.1f},{WINDOW_TOP - 4} V{axis.y + 6}', 'now')
    figure.path(f'M{now_x:.1f},{WINDOW_TOP - 4} V{axis.y + 6}', 'reference')
    symbol(figure, last_x, axis.y + 20, 't', 'input', False)
    symbol(figure, now_x, axis.y + 20, 't', '', False, before='now: ')

    samples = (
        Sample('P', last_x, True, 'the same stamp as the last one of this input. The filter '
               'takes the sample. The validator drops it in check 5'),
        Sample('Q', axis.at(sample_q_ms), True, 'stamped 0.11 s before the ROS time. The filter '
               'takes the sample, because it compares the stamp only with the last one of this '
               'input. The validator drops it in check 4'),
        Sample('R', axis.at(-165), False, 'earlier than the last stamp of this input. The '
               'filter ignores the sample, or starts over with simulated time'),
    )
    for sample in samples:
        figure.mark(sample, sample.x, axis.y - 10, LARGE_MARK_RADIUS)
    return figure, samples


def write_stamp_check() -> None:
    """Write the include with robot_localization's own stamp check of an input."""
    figure, samples = stamp_check()
    write_parts(
        'includes/localization-stamp-check.html',
        [figure.svg(), sample_legend(samples, ('✓ taken', '✕ ignored'))],
        "robot_localization's own check of an IMU sample's stamp. The axis shows stamps, with "
        'time running to the right, and the ROS time at the dashed line. Example values.',
        RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES, 'lv-large lv-drawing lv-symbols')


def step_rows(figure: Figure, axis: TimeAxis, wheel_stamps_ms: Sequence[float]) -> None:
    """Add the time axis, the rows' names and the wheel odometry's messages of a time step."""
    for name, row_y in (('estimate', 40), ('wheels', 64), ('IMU', 86)):
        figure.text(4, row_y, name, 'quiet')
    for stamp_ms in wheel_stamps_ms:
        figure.circle(axis.at(stamp_ms), 60, 4.5, 'wheels-dot')
    figure.time_axis(axis, axis.start_ms)
    figure.path(f'M{axis.at(0):.1f},{axis.y - 6} V{axis.y + 6}', 'now')
    symbol(figure, axis.at(0), axis.y + 20, 't', '', False, before='now: ')


def time_step_dimension(figure: Figure, axis: TimeAxis, filter_ms: float, sample_ms: float,
                        relation: str) -> None:
    """Add the time step Δt as a dimension line from the filter's time to the sample's stamp."""
    filter_x, sample_x = axis.at(filter_ms), axis.at(sample_ms)
    figure.dimension(min(filter_x, sample_x), max(filter_x, sample_x), 116)
    figure.rich_text((filter_x + sample_x) / 2, 132, (Span('Δ'), Span('t', 'variable'),
                                                      Span(f' {relation} 0')), 'middle')


def time_marks(figure: Figure, axis: TimeAxis, filter_ms: float, sample_ms: float) -> None:
    """Add the lines and the labels of the filter's time and of the sample's stamp."""
    for stamp_ms in (filter_ms, sample_ms):
        figure.path(f'M{axis.at(stamp_ms):.1f},20 V{axis.y}', 'reference')
    symbol(figure, axis.at(filter_ms), 14, 't', 'filter', False)
    symbol(figure, axis.at(sample_ms), 14, 't', 's', True)


def forward_step() -> Figure:
    """Return the drawing of a time step that is positive: the filter predicts, then corrects."""
    axis = TimeAxis(-110, 30, ROW_LABEL_WIDTH, 102, STEP_PIXELS_PER_MS)
    figure = Figure('lv-step-forward', "A current IMU sample: its stamp lies after the filter's "
                    'time, so the filter predicts its estimate forward to the stamp and then '
                    'corrects it', 140, CHECKS_WIDTH)
    filter_x, sample_x = axis.at(FORWARD_FILTER_TIME_MS), axis.at(FORWARD_SAMPLE_MS)
    time_marks(figure, axis, FORWARD_FILTER_TIME_MS, FORWARD_SAMPLE_MS)
    step_rows(figure, axis, (-90, -70, FORWARD_FILTER_TIME_MS))
    figure.circle(filter_x, 36, 4.5, 'estimate-dot')
    figure.arrow(filter_x + 5, 36, sample_x - 5, 36, 'estimate')
    figure.circle(sample_x, 36, 4.5, 'estimate-dot')
    figure.text((filter_x + sample_x) / 2, 30, 'predict', '', 'middle')
    figure.text(sample_x + 8, 40, 'correct')
    figure.circle(sample_x, 82, 4.5, 'imu-dot')
    time_step_dimension(figure, axis, FORWARD_FILTER_TIME_MS, FORWARD_SAMPLE_MS, '>')
    return figure


def late_step() -> Figure:
    """Return the drawing of a time step that is negative: the filter corrects only."""
    axis = TimeAxis(-110, 30, ROW_LABEL_WIDTH, 102, STEP_PIXELS_PER_MS)
    figure = Figure('lv-step-late', "A late IMU sample: its stamp lies before the filter's "
                    "time, so the filter corrects its estimate at the filter's time, with the "
                    "old sample, and doesn't go back to the stamp", 140, CHECKS_WIDTH)
    filter_x, sample_x = axis.at(LATE_FILTER_TIME_MS), axis.at(LATE_SAMPLE_MS)
    time_marks(figure, axis, LATE_FILTER_TIME_MS, LATE_SAMPLE_MS)
    step_rows(figure, axis, (-80, -60, -40, LATE_FILTER_TIME_MS))
    figure.circle(sample_x, 82, 4.5, 'imu-dot')
    # From the late sample to the estimate at the filter's time
    end_x = filter_x - 6
    figure.path(f'M{sample_x:.1f},76 C{sample_x:.1f},40 {sample_x + 40:.1f},36 '
                f'{end_x - 5:.1f},36', 'imu-line')
    figure.path(arrow_head(end_x, 36, 1, 0), 'imu-head')
    figure.circle(filter_x, 36, 4.5, 'estimate-dot')
    figure.text(filter_x + 8, 40, 'correct')
    time_step_dimension(figure, axis, LATE_FILTER_TIME_MS, LATE_SAMPLE_MS, '<')
    return figure


def write_time_step() -> None:
    """Write the include with the filter's time step for a current and for a late sample."""
    write_parts(
        'includes/localization-time-step.html',
        [band('Δt > 0: the filter predicts, then corrects'), forward_step().svg(),
         band('Δt < 0: the filter corrects only'), late_step().svg()],
        "The filter's time step for an IMU sample, with the wheel odometry's messages that the "
        "filter processed before. The dots on the top row are the filter's estimate. The axis "
        'shows stamps. t is the ROS time when the IMU sample arrives. Example values.',
        RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES, 'lv-large lv-drawing lv-symbols')


def bell(center_x: float, sigma_pixels: float, peak: float, base_y: float) -> str:
    """Return the path commands of a normal distribution's curve, closed along its base."""
    points = []
    for step in range(-36, 37):
        offset = step / 10 * sigma_pixels
        points.append(f'{center_x + offset:.1f},'
                      f'{base_y - peak * math.exp(-0.5 * (step / 10) ** 2):.1f}')
    return f'M{center_x - 3.6 * sigma_pixels:.1f},{base_y} L' + ' L'.join(points) + ' z'


def one_value_distance() -> Figure:
    """Return the drawing of the distance between a prediction and a measurement of one value."""
    figure = Figure('lv-distance-one', "The yaw rate on an axis: the prediction and the IMU's "
                    'measurement as two bell curves, 0.05 rad/s apart. The distance d counts '
                    'this difference in units of the two standard deviations together, 0.025 '
                    'rad/s, so d is 2', 172, CHECKS_WIDTH)
    low, high = YAW_RATE_AXIS
    axis_y = 124

    def x_at(rate: float) -> float:
        return 24 + (rate - low) * PIXELS_PER_RAD_PER_S

    prediction_x, measurement_x = x_at(PREDICTED_YAW_RATE), x_at(MEASURED_YAW_RATE)
    figure.path(bell(prediction_x, PREDICTION_SIGMA * PIXELS_PER_RAD_PER_S, 70, axis_y),
                'bell-prediction')
    figure.path(bell(measurement_x, IMU_SIGMA * PIXELS_PER_RAD_PER_S, 70 * PREDICTION_SIGMA
                     / IMU_SIGMA, axis_y), 'bell-imu')
    for x, name, index, sigma in ((prediction_x - 16, 'prediction', 'pred', PREDICTION_SIGMA),
                                  (measurement_x + 16, 'IMU', 'IMU', IMU_SIGMA)):
        figure.text(x, 22, name, '', 'middle')
        figure.rich_text(x, 38, (Span('σ', 'variable'), Span(index, 'subscript'),
                                 Span(f' = {sigma}')), 'middle')

    figure.path(f'M{x_at(low)},{axis_y} H{x_at(high) + 7:.1f}', 'axis')
    figure.path(f'M{x_at(high) + 7:.1f},{axis_y} l-6,-3.5 M{x_at(high) + 7:.1f},{axis_y} l-6,3.5',
                'axis')
    for rate in (0.45, 0.50, 0.55):
        figure.path(f'M{x_at(rate):.1f},{axis_y} v4', 'tick')
        figure.text(x_at(rate), axis_y + 16, f'{rate:.2f}', 'note', 'middle')
    figure.text(x_at(high) + 7, axis_y + 16, 'rad/s', 'note', 'end')
    symbol(figure, 4, axis_y + 4, 'ω', 'z', True, 'start')

    # The distance's scale: one unit is the combined standard deviation
    combined = math.hypot(PREDICTION_SIGMA, IMU_SIGMA) * PIXELS_PER_RAD_PER_S
    ruler_y = 152
    figure.path(f'M{prediction_x:.1f},{ruler_y} H{prediction_x + 3 * combined:.1f}', 'axis')
    for unit in range(4):
        figure.path(f'M{prediction_x + unit * combined:.1f},{ruler_y - 3} v6', 'tick')
        figure.text(prediction_x + unit * combined, ruler_y + 15, str(unit), 'note', 'middle')
    figure.circle(measurement_x, ruler_y, 4.5, 'imu-dot')
    figure.rich_text(prediction_x - 10, ruler_y + 4, (Span('d', 'variable'),), 'end')
    return figure


def two_value_distance() -> Tuple[Figure, Tuple[Sample, ...]]:
    """Return the drawing of the distance for two values, with the ellipses of equal distance."""
    figure = Figure('lv-distance-two', 'The differences between measurement and prediction for '
                    'the forward speed and the yaw rate as a plane. Around the prediction, the '
                    'ellipses of the distances 1 and 2, and the rectangle of two standard '
                    'deviations for each value alone', 236, CHECKS_WIDTH)
    center_x, center_y = 150, 118
    unit_x, unit_y = PIXELS_PER_FORWARD_SPEED_SIGMA, PIXELS_PER_YAW_RATE_SIGMA
    limit = EXAMPLE_THRESHOLD
    figure.rect(center_x - limit * unit_x, center_y - limit * unit_y, 2 * limit * unit_x,
                2 * limit * unit_y, 'limit-box')
    figure.ellipse(center_x, center_y, limit * unit_x, limit * unit_y, 'threshold')
    figure.ellipse(center_x, center_y, unit_x, unit_y, 'ellipse')
    figure.path(f'M26,{center_y} H{center_x + 2.7 * unit_x:.1f}', 'axis')
    figure.path(arrow_head(center_x + 2.7 * unit_x + 6, center_y, 1, 0, 6), 'axis')
    figure.path(f'M{center_x},{center_y + 2.7 * unit_y:.1f} V{center_y - 2.7 * unit_y:.1f}',
                'axis')
    figure.path(arrow_head(center_x, center_y - 2.7 * unit_y - 6, 0, -1, 6), 'axis')
    symbol(figure, center_x + 2.7 * unit_x + 4, center_y - 8, 'v', 'x', True, 'end')
    symbol(figure, center_x + 8, center_y - 2.7 * unit_y + 4, 'ω', 'z', True, 'start')
    for sign in (-1, 1):
        figure.path(f'M{center_x + sign * limit * unit_x:.1f},{center_y} v4', 'tick')
        figure.path(f'M{center_x},{center_y - sign * limit * unit_y:.1f} h4', 'tick')
    speed, rate = limit * FORWARD_SPEED_SIGMA, limit * YAW_RATE_SIGMA
    # The values at the rectangle's edges: below its bottom corners and next to its top and
    # bottom edges, off the ellipses' lines
    box_bottom = center_y + limit * unit_y
    figure.text(center_x + limit * unit_x, box_bottom + 14, f'{speed:.2f} m/s', 'note', 'middle')
    figure.text(center_x - limit * unit_x, box_bottom + 14, f'−{speed:.2f}', 'note', 'middle')
    figure.text(center_x + 7, center_y - limit * unit_y - 4, f'{rate:.1f} rad/s', 'note')
    figure.text(center_x + 7, box_bottom + 14, f'−{rate:.1f}', 'note')
    figure.text(center_x - unit_x * 0.72 - 4, center_y - unit_y * 0.72 - 3, 'd = 1', 'note',
                'end')
    figure.text(center_x - 50, center_y - limit * unit_y + 18, f'd = {limit:.0f}', 'strong')
    figure.circle(center_x, center_y, 4.5, 'estimate-dot')

    samples = (
        Sample('P', center_x + 1.25 * unit_x, True, 'the forward speed 1.25 and the yaw rate '
               '0.6 standard deviations from the prediction: d = 1.4, inside the ellipse'),
        Sample('Q', center_x + 1.6 * unit_x, False, 'each value 1.6 standard deviations from '
               'the prediction, so each value alone lies inside the rectangle. Together, '
               'd = 2.3, outside the ellipse'),
    )
    figure.mark(samples[0], samples[0].x, center_y - 0.6 * unit_y, LARGE_MARK_RADIUS)
    figure.mark(samples[1], samples[1].x, center_y + 1.6 * unit_y, LARGE_MARK_RADIUS)
    return figure, samples


def write_distance() -> None:
    """Write the include with the distance of the rejection test, for one and for two values."""
    two_values, samples = two_value_distance()
    write_parts(
        'includes/localization-distance.html',
        [band('One value: the yaw rate'), one_value_distance().svg(),
         band('Two values: the forward speed and the yaw rate'), two_values.svg(),
         sample_legend(samples, ('✓ fused', '✕ not fused'))],
        'The distance between a measurement and the prediction, counted in standard deviations, '
        f'with an example threshold of {EXAMPLE_THRESHOLD:.0f}. The green dot of the second '
        'drawing is the prediction. '
        "<code>ekf.yaml</code> sets no threshold. Example values: the IMU's standard deviation "
        "is the firmware's, the others are chosen for the figure.",
        RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES, 'lv-large lv-drawing lv-symbols')


def covariance_grid(name: str, field: str, source: str, variables: Sequence[Tuple[str, str]],
                    fused: Sequence[int], replaced: Sequence[int], css_class: str) -> Figure:
    """Return a covariance as a grid: each cell holds its index in the message's array.

    The array stores the matrix row by row, so the element in row i and column j has the
    index n i + j, with n rows and columns.
    """
    size = len(variables)
    grid_width = COVARIANCE_LABEL_WIDTH + size * COVARIANCE_CELL
    width = max(grid_width, COVARIANCE_MIN_WIDTH) + 4
    left = (width - grid_width) / 2 + COVARIANCE_LABEL_WIDTH
    figure = Figure(name, f'The covariance {field} of the {source} as a grid of {size} rows '
                    f'and {size} columns, with the index of each element in the array',
                    COVARIANCE_TOP + size * COVARIANCE_CELL + 4, int(width), css_class)
    figure.elements.append(COVARIANCE_HATCH)
    figure.text(width / 2, 13, field, 'mono', 'middle')
    figure.text(width / 2, 28, source, 'note', 'middle')
    for position, (letter, index) in enumerate(variables):
        middle = position * COVARIANCE_CELL + COVARIANCE_CELL / 2
        symbol(figure, left + middle - 2, COVARIANCE_TOP - 8, letter, index, True)
        symbol(figure, left - 18, COVARIANCE_TOP + middle + 4, letter, index, True)
    for row in range(size):
        for column in range(size):
            element = size * row + column
            x = left + column * COVARIANCE_CELL
            y = COVARIANCE_TOP + row * COVARIANCE_CELL
            text_y = y + COVARIANCE_CELL / 2 + 4
            if element in fused:
                figure.rect(x, y, COVARIANCE_CELL, COVARIANCE_CELL, 'cell-fused')
                figure.text(x + COVARIANCE_CELL / 2, text_y, f'✓{element}', 'strong', 'middle')
            elif element in replaced:
                figure.rect(x, y, COVARIANCE_CELL, COVARIANCE_CELL, 'cell-replaced')
                figure.text(x + COVARIANCE_CELL / 2, text_y, str(element), '', 'middle')
            else:
                figure.rect(x, y, COVARIANCE_CELL, COVARIANCE_CELL, 'cell')
                figure.text(x + COVARIANCE_CELL / 2, text_y, str(element), 'note', 'middle')
    return figure


def write_covariances() -> None:
    """Write the include with the two covariances that the filter reads, as grids."""
    grids = (
        covariance_grid('lv-twist-covariance', 'twist.covariance', 'wheel odometry, 6 × 6',
                        TWIST_VARIABLES, TWIST_FUSED_VARIANCES, TWIST_REPLACED_VARIANCES,
                        'lv-twist'),
        covariance_grid('lv-angular-covariance', 'angular_velocity_covariance', 'IMU, 3 × 3',
                        ANGULAR_VELOCITY_VARIABLES, ANGULAR_VELOCITY_FUSED_VARIANCES,
                        ANGULAR_VELOCITY_REPLACED_VARIANCES, 'lv-angular'),
    )
    legend = (
        '<ul class="lv-legend">\n'
        '<li><span class="lv-swatch lv-fused">✓</span>the variance of a value that the filter '
        'fuses</li>\n'
        '<li><span class="lv-swatch lv-zero"></span>a variance that <code>two_d_mode</code> '
        'replaces with 10⁻⁶</li>\n'
        '<li><span class="lv-swatch"></span>a covariance of two different values, 0 in both '
        'messages</li>\n'
        '</ul>')
    pair = '<div class="lv-pair">\n' + '\n'.join(grid.svg() for grid in grids) + '\n</div>'
    write_parts(
        'includes/localization-covariances.html', [pair, legend],
        "The two covariances as matrices. A row's and a column's symbols name the two values of "
        "an element. The number in a cell is the element's index in the message's array.",
        RULES + LARGE_RULES + DRAWING_RULES + SYMBOL_RULES,
        'lv-large lv-drawing lv-symbols lv-covariances')


class Label(NamedTuple):
    """One line of text next to an arrow: how far below the arrow's start, and how it is set."""

    offset_y: float
    content: str
    css_class: str = ''


class Hardware(NamedTuple):
    """What the two variants of the loop figure don't share: the real robot's parts or Gazebo's."""

    name: str  # of the variant: the drawing's id ends with it
    tab: str  # the label of the variant's tab
    title: str  # of the drawing, for screen readers
    simulated: bool
    wheel_sensor: str  # the three lines of the key next to the robot
    imu: str
    wheel_drive: str
    interface: str  # what reads the sensors and drives the wheels
    wheel_labels: Tuple[Label, ...]  # between the interface and the drive controller
    controller_host: str  # where the drive controller runs
    caption: str


REAL_ROBOT = Hardware(
    name='robot',
    tab='Real robot',
    title='The loop on the real robot: the wheel encoders and the IMU, the firmware on the '
          'Teensy, the drive controller and the IMU sample validator, the EKF, its two outputs '
          'in a sketch of RViz, and the velocity commands that go back to the motors',
    simulated=False,
    wheel_sensor='wheel encoder',
    imu='IMU (BNO055)',
    wheel_drive='to the motors',
    interface='Teensy: firmware, over USB to rosserial',
    wheel_labels=(Label(17, '↓ wheel angles'),
                  Label(32, 'measured_joint_states', 'mono'),
                  Label(54, '↑ target wheel speeds', 'quiet'),
                  Label(69, 'wheel_cmd_velocities', 'mono quiet')),
    controller_host='in the node diffbot_base',
    caption='The loop on the real robot. The robot is drawn from above and not to scale.',
)
SIMULATION = Hardware(
    name='simulation',
    tab='Simulation',
    title="The loop in the simulation: Gazebo's wheel joints and its IMU sensor, Gazebo's "
          'plugins, the drive controller, the adapter and the IMU sample validator, the EKF, '
          'its two outputs in a sketch of RViz, and the velocity commands that go back to the '
          'wheel joints',
    simulated=True,
    wheel_sensor='wheel joint',
    imu='IMU sensor',
    wheel_drive='joint torques',
    interface="gazebo_ros_control and the IMU sensor's plugin",
    wheel_labels=(Label(17, "↓ wheel joints' angles"),
                  Label(39, '↑ target wheel speeds', 'quiet'),
                  Label(61, 'inside Gazebo,', 'quiet'),
                  Label(76, 'without a topic', 'quiet')),
    controller_host='inside gazebo_ros_control',
    caption="The loop in the simulation. Gazebo takes the place of the robot's hardware, and "
            'the rest of the loop is the same.',
)


class Pose(NamedTuple):
    """A pose drawn from above: where the odom frame's origin and the robot are in a drawing."""

    origin_x: float
    origin_y: float
    robot_x: float
    robot_y: float


# The robot of a drawing of a pose started at the odom frame's origin and drove a left arc of a
# circle until it had turned by POSE_HEADING. Its position follows from the arc's radius
PATH_RADIUS = 332
POSE_OFFSET_X = PATH_RADIUS * math.sin(math.radians(POSE_HEADING))
POSE_OFFSET_Y = PATH_RADIUS * (1 - math.cos(math.radians(POSE_HEADING)))


def pose_from(origin_x: float, origin_y: float) -> Pose:
    """Return the pose that a drawing shows, with the odom frame's origin at origin_x, origin_y."""
    return Pose(origin_x, origin_y, origin_x + POSE_OFFSET_X, origin_y - POSE_OFFSET_Y)


def robot_from_above(figure: Figure, center_y: float, simulated: bool) -> float:
    """Draw a differential drive robot seen from above, its front pointing up.

    The real robot has an encoder at each motor. The simulated robot has its wheel joints
    instead. Both have the IMU behind the wheel axle, left of the center line. Thin lines lead
    from the sensors to the robot's rear edge. Return the y position of that edge.
    """
    center_x = ROBOT_CENTER_X
    axle_y = center_y + AXLE_Y
    rear_y = center_y + BODY_HALF_LENGTH
    for side in (-1, 1):
        wheel_x = center_x + side * WHEEL_X - (WHEEL_WIDTH if side < 0 else 0)
        figure.rect(wheel_x, axle_y - WHEEL_HALF_LENGTH, WHEEL_WIDTH, 2 * WHEEL_HALF_LENGTH,
                    'robot-wheel', corner_radius=4)
    figure.rect(center_x - BODY_HALF_WIDTH, center_y - BODY_HALF_LENGTH, 2 * BODY_HALF_WIDTH,
                2 * BODY_HALF_LENGTH, 'robot-body', corner_radius=14)
    figure.circle(center_x, center_y + 44, 6, 'robot-part')  # the caster wheel
    if not simulated:
        for side in (-1, 1):
            motor_x = center_x + side * 20 + (-24 if side < 0 else 0)
            figure.rect(motor_x, axle_y - 7, 24, 14, 'robot-part', corner_radius=2)
    # The wheels' sensors: encoders at the motors, or the joints at the body's sides. Their
    # lines join and leave the robot at its rear
    sensor_x = BODY_HALF_WIDTH if simulated else ENCODER_X
    bus_y = center_y + 4
    bus_start_x = min(center_x - sensor_x, WHEELS_EXIT_X)
    figure.path(f'M{center_x - sensor_x},{axle_y} V{bus_y} M{center_x + sensor_x},{axle_y} '
                f'V{bus_y} H{bus_start_x} M{WHEELS_EXIT_X},{bus_y} V{rear_y}', 'wheels-wire')
    for side in (-1, 1):
        figure.circle(center_x + side * sensor_x, axle_y, 4.5, 'encoder')
    chip_x, chip_y, chip_size = IMU_CHIP
    figure.path(f'M{center_x + chip_x + chip_size / 2},{center_y + chip_y + chip_size} '
                f'V{center_y + 34} H{IMU_ARROW_X} V{rear_y}', 'imu-wire')
    figure.rect(center_x + chip_x, center_y + chip_y, chip_size, chip_size, 'imu-chip',
                corner_radius=2)
    # The two velocities: the forward speed along the robot's front, the yaw rate about its center
    front_y = center_y - BODY_HALF_LENGTH
    figure.arrow(center_x, front_y - 3, center_x, front_y - 22, 'motion')
    figure.rich_text(center_x + 7, front_y - 9, V_X)
    radius = 74
    start, end = math.radians(18), math.radians(52)
    end_x, end_y = center_x + radius * math.cos(end), center_y - radius * math.sin(end)
    figure.path(f'M{center_x + radius * math.cos(start):.1f},'
                f'{center_y - radius * math.sin(start):.1f} '
                f'A{radius},{radius} 0 0 0 {end_x:.1f},{end_y:.1f}', 'motion-line')
    figure.path(arrow_head(end_x - 2 * math.sin(end), end_y - 2 * math.cos(end),
                           -math.sin(end), -math.cos(end)), 'motion-head')
    figure.rich_text(center_x + 68, center_y - 50, OMEGA_Z)
    return rear_y


def robot_key(figure: Figure, y: float, hardware: Hardware) -> None:
    """Add the key left of the robot: its two sensors' marks and the commands' dashed arrows."""
    figure.circle(13, y - 4, 4.5, 'encoder')
    figure.text(23, y, hardware.wheel_sensor)
    figure.rect(8.5, y + 8.5, 9, 9, 'imu-chip', corner_radius=2)
    figure.text(23, y + 17, hardware.imu)
    figure.path(f'M6,{y + 30} H20', 'command-line')
    figure.text(23, y + 34, hardware.wheel_drive, 'quiet')


def node_box(figure: Figure, column_start: float, column_end: float, y: float, height: float,
             lines: Sequence[Tuple[str, str]]) -> None:
    """Add a box that fills a column, with centered lines of text, each a (text, class) pair."""
    figure.rect(column_start, y, column_end - column_start, height, 'item', corner_radius=5)
    middle = (column_start + column_end) / 2
    first_baseline = y + height / 2 + 4 - (len(lines) - 1) * LINE_HEIGHT / 2
    for index, (content, css_class) in enumerate(lines):
        figure.text(middle, first_baseline + index * LINE_HEIGHT, content, css_class, 'middle')


def gazebo_region(figure: Figure) -> None:
    """Draw the dashed area of what runs inside Gazebo, with its name.

    The area holds the robot, the bar of Gazebo's plugins and, below the bar's left part, the
    drive controller.
    """
    left, right, top = BLOCK_START - 3, BLOCK_END + 3, 6
    bar_bottom = INTERFACE_Y + INTERFACE_HEIGHT + 4
    foot_right = WHEELS_COLUMN_END + 3
    foot_bottom = NODES_Y + NODES_HEIGHT + 4
    figure.path(f'M{left + 5},{top} H{right - 5} a5,5 0 0 1 5,5 V{bar_bottom - 5} '
                f'a5,5 0 0 1 -5,5 H{foot_right + 5} a5,5 0 0 0 -5,5 V{foot_bottom - 5} '
                f'a5,5 0 0 1 -5,5 H{left + 5} a5,5 0 0 1 -5,-5 V{top + 5} a5,5 0 0 1 5,-5 z',
                'gazebo')
    figure.text(left + 10, top + 18, 'Gazebo', 'strong')


def hardware_block(figure: Figure, hardware: Hardware) -> None:
    """Draw the part that the two variants don't share, from the robot down to the two nodes.

    1. The robot with its sensors, and below it the bar of what reads the sensors and drives the
       wheels: the Teensy, or Gazebo's plugins.
    2. Between the bar and the drive controller: the wheels' angles down, and the wheels' target
       speeds up.
    3. Between the bar and the IMU sample validator: the IMU's samples down. In the simulation
       they pass the adapter.
    """
    if hardware.simulated:
        gazebo_region(figure)
    rear_y = robot_from_above(figure, ROBOT_CENTER_Y, hardware.simulated)
    robot_key(figure, ROBOT_CENTER_Y + 24, hardware)
    figure.arrow(WHEELS_EXIT_X, rear_y, WHEELS_EXIT_X, INTERFACE_Y, 'wheels')
    figure.arrow(IMU_ARROW_X, rear_y, IMU_ARROW_X, INTERFACE_Y, 'imu')
    wheel_rear_y = ROBOT_CENTER_Y + AXLE_Y + WHEEL_HALF_LENGTH + 2
    for side in (-1, 1):
        wheel_middle_x = ROBOT_CENTER_X + side * (WHEEL_X + WHEEL_WIDTH / 2)
        figure.arrow(wheel_middle_x, INTERFACE_Y, wheel_middle_x, wheel_rear_y, 'command')
    figure.rect(BLOCK_START, INTERFACE_Y, BLOCK_END - BLOCK_START, INTERFACE_HEIGHT, 'item',
                corner_radius=5)
    figure.text(ROBOT_CENTER_X, INTERFACE_Y + 16, hardware.interface, anchor='middle')

    below_y = INTERFACE_Y + INTERFACE_HEIGHT
    figure.arrow(COMMAND_LANE_X, NODES_Y, COMMAND_LANE_X, below_y, 'command')
    figure.arrow(WHEELS_ARROW_X, below_y, WHEELS_ARROW_X, NODES_Y, 'wheels')
    for label in hardware.wheel_labels:
        figure.text(WHEELS_ARROW_X + LABEL_GAP, below_y + label.offset_y, label.content,
                    label.css_class)

    label_x = IMU_ARROW_X + LABEL_GAP
    if hardware.simulated:
        adapter_y = below_y + 27
        adapter_height = 26
        figure.arrow(IMU_ARROW_X, below_y, IMU_ARROW_X, adapter_y, 'imu')
        figure.text(label_x, below_y + 17, 'imu/gazebo', 'mono')
        node_box(figure, IMU_COLUMN_START, BLOCK_END, adapter_y, adapter_height,
                 (('imu_raw_adapter', 'mono'),))
        figure.arrow(IMU_ARROW_X, adapter_y + adapter_height, IMU_ARROW_X, NODES_Y, 'imu')
        figure.text(label_x, NODES_Y - 12, 'imu/data_raw', 'mono')
    else:
        figure.arrow(IMU_ARROW_X, below_y, IMU_ARROW_X, NODES_Y, 'imu')
        figure.text(label_x, below_y + 32, 'IMU samples')
        figure.text(label_x, below_y + 47, 'imu/data_raw', 'mono')


def measurement_nodes(figure: Figure, hardware: Hardware) -> float:
    """Draw the drive controller and the IMU sample validator, each in its box.

    They turn the sensors' values into the filter's measurements. Return the y position of the
    boxes' lower edge.
    """
    node_box(figure, BLOCK_START, WHEELS_COLUMN_END, NODES_Y, NODES_HEIGHT,
             (('drive controller', ''), ('mobile_base_controller', 'mono'),
              (hardware.controller_host, 'quiet')))
    node_box(figure, IMU_COLUMN_START, BLOCK_END, NODES_Y, NODES_HEIGHT,
             (('IMU sample validator', ''), ('imu_validator', 'mono')))
    return NODES_Y + NODES_HEIGHT


def filter_and_outputs(figure: Figure, y: float) -> float:
    """Draw the measurements' arrows into the EKF, the EKF and its two outputs' arrows.

    y is where the arrows into the EKF start. Return the y position where the two output
    arrows end: the left one for the topic, the right one for the transform.
    """
    filter_y = y + INPUTS_HEIGHT
    figure.arrow(WHEELS_ARROW_X, y, WHEELS_ARROW_X, filter_y, 'wheels')
    label_x = WHEELS_ARROW_X + LABEL_GAP
    figure.rich_text(label_x, y + 18, Z_WHEELS)
    figure.text(label_x, y + 34, 'mobile_base_controller', 'mono')
    figure.text(label_x, y + 48, '/odom', 'mono')
    figure.arrow(IMU_ARROW_X, y, IMU_ARROW_X, filter_y, 'imu')
    label_x = IMU_ARROW_X + LABEL_GAP
    figure.rich_text(label_x, y + 18, Z_IMU)
    figure.text(label_x, y + 34, 'imu/data_validated', 'mono')

    figure.rect(INNER_START, filter_y, INNER_END - INNER_START, FILTER_HEIGHT, 'filter',
                corner_radius=5)
    middle_x = (INNER_START + INNER_END) / 2
    figure.rich_text(middle_x, filter_y + 19,
                     (Span('EKF', 'strong'), Span(', the node '),
                      Span('ekf_localization', 'mono')), 'middle')
    figure.estimate_text(middle_x - 107, filter_y + 36,
                         (Span(": the filter's estimate of the state "), Span('x', 'vector')))

    outputs_y = filter_y + FILTER_HEIGHT
    end_y = outputs_y + OUTPUTS_HEIGHT
    figure.arrow(WHEELS_ARROW_X, outputs_y, WHEELS_ARROW_X, end_y, 'estimate')
    label_x = WHEELS_ARROW_X + LABEL_GAP
    figure.estimate_text(label_x, outputs_y + 18, PUBLISHED_INDEX)
    figure.rich_text(label_x, outputs_y + 34, PUBLISHED)
    figure.text(label_x, outputs_y + 50, 'odometry/filtered', 'mono')
    figure.text(label_x, outputs_y + 65, 'with covariances')
    figure.arrow(IMU_ARROW_X, outputs_y, IMU_ARROW_X, end_y, 'estimate')
    label_x = IMU_ARROW_X + LABEL_GAP
    figure.rich_text(label_x, outputs_y + 18, POSE)
    figure.text(label_x, outputs_y + 34, 'as the transform')
    figure.text(label_x, outputs_y + 50, 'odom →', 'mono')
    figure.text(label_x, outputs_y + 64, 'base_footprint', 'mono')
    return end_y


def small_robot(figure: Figure, x: float, y: float, heading: float,
                body_class: str = 'pose-body', wheel_class: str = 'pose-wheel') -> None:
    """Add the robot seen from above, small, turned by heading in degrees.

    x, y is the middle of its wheel axle, where the robot's frame base_footprint lies.
    """
    body_top = -BODY_HALF_LENGTH - AXLE_Y
    wheels = ''.join(
        f'<rect class="{wheel_class}" x="{wheel_x}" y="{-WHEEL_HALF_LENGTH}" '
        f'width="{WHEEL_WIDTH}" height="{2 * WHEEL_HALF_LENGTH}" rx="4"/>'
        for wheel_x in (-WHEEL_X - WHEEL_WIDTH, WHEEL_X))
    figure.elements.append(
        f'<g transform="translate({x:.1f},{y:.1f}) rotate({90 - heading}) '
        f'scale({SMALL_ROBOT_SCALE})">{wheels}<rect class="{body_class}" '
        f'x="{-BODY_HALF_WIDTH}" y="{body_top}" width="{2 * BODY_HALF_WIDTH}" '
        f'height="{2 * BODY_HALF_LENGTH}" rx="14"/></g>')


def odom_axes(figure: Figure, pose: Pose, x_length: float, y_length: float) -> None:
    """Add the two axes of the odom frame, with their letters at the tips."""
    figure.arrow(pose.origin_x, pose.origin_y, pose.origin_x + x_length, pose.origin_y,
                 'frame-axis')
    figure.arrow(pose.origin_x, pose.origin_y, pose.origin_x, pose.origin_y - y_length,
                 'frame-axis')
    figure.rich_text(pose.origin_x + x_length + 5, pose.origin_y + 4, X)
    figure.rich_text(pose.origin_x - 3, pose.origin_y - y_length - 5, Y)


def transform_line(figure: Figure, pose: Pose) -> None:
    """Add the transform odom → base_footprint as a dashed line with its name along the line.

    The name stands a little nearer to the origin, so that it ends before the robot.
    """
    figure.path(f'M{pose.origin_x:.1f},{pose.origin_y:.1f} L{pose.robot_x:.1f},'
                f'{pose.robot_y:.1f}', 'transform')
    slope = math.degrees(math.atan2(pose.robot_y - pose.origin_y, pose.robot_x - pose.origin_x))
    name_x = pose.origin_x + 0.44 * (pose.robot_x - pose.origin_x)
    name_y = pose.origin_y + 0.44 * (pose.robot_y - pose.origin_y)
    figure.elements.append(
        f'<text text-anchor="middle" transform="translate({name_x:.1f},{name_y:.1f}) '
        f'rotate({slope:.1f})" y="-7">odom → base_footprint</text>')


def heading_angle(figure: Figure, pose: Pose, radius: float = 34) -> None:
    """Add the heading's angle θ at the robot, from the odom frame's x direction."""
    heading = math.radians(POSE_HEADING)
    figure.path(f'M{pose.robot_x:.1f},{pose.robot_y:.1f} h{radius + 10}', 'transform')
    figure.path(f'M{pose.robot_x + radius:.1f},{pose.robot_y:.1f} A{radius},{radius} 0 0 0 '
                f'{pose.robot_x + radius * math.cos(heading):.1f},'
                f'{pose.robot_y - radius * math.sin(heading):.1f}', 'angle')
    figure.rich_text(pose.robot_x + radius + 5, pose.robot_y - 7, THETA)


def rviz_panel(figure: Figure, y: float, height: float) -> None:
    """Draw a sketch of RViz with the output: the robot at the transform, and kept pose arrows.

    The transform's dashed line, the axes' letters and the heading's angle are annotations
    that RViz doesn't draw.
    """
    left, right = INNER_START, INNER_END
    figure.rect(left, y, right - left, height, 'rviz', corner_radius=5)
    grid = [f'M{x},{y + 1} V{y + height - 1}' for x in range(left + 30, right, 30)]
    grid += [f'M{left + 1},{line_y} H{right - 1}'
             for line_y in range(int(y) + 28, int(y + height), 30)]
    figure.path(' '.join(grid), 'rviz-grid')
    figure.elements.append('<g class="rviz-scene">')
    figure.rich_text(left + 10, y + 18, (Span('RViz', 'strong'), Span(', fixed frame '),
                                         Span('odom', 'mono')))

    pose = pose_from(left + 42, y + height - 26)
    odom_axes(figure, pose, 50, 50)
    figure.text(pose.origin_x - 2, pose.origin_y + 17, 'odom', 'mono', 'middle')
    # The poses that an Odometry display keeps, along the arc that the robot drove
    for degrees in (11, 16.5, 22, 27.5):
        angle = math.radians(degrees)
        start_x = pose.origin_x + PATH_RADIUS * math.sin(angle)
        start_y = pose.origin_y - PATH_RADIUS * (1 - math.cos(angle))
        figure.arrow(start_x, start_y, start_x + 15 * math.cos(angle),
                     start_y - 15 * math.sin(angle), 'rviz-trail')
    transform_line(figure, pose)
    # The robot where the transform puts it, turned by the heading
    small_robot(figure, pose.robot_x, pose.robot_y, POSE_HEADING)
    heading_angle(figure, pose)
    heading = math.radians(POSE_HEADING)
    figure.arrow(pose.robot_x, pose.robot_y, pose.robot_x + 56 * math.cos(heading),
                 pose.robot_y - 56 * math.sin(heading), 'rviz-pose')
    figure.text(right - 8, y + height - 9, 'Odometry display: kept pose arrows', anchor='end')
    figure.elements.append('</g>')


def way_back(figure: Figure, nodes_bottom_y: float, filter_y: float, y: float) -> None:
    """Draw who sends velocity commands, below the RViz panel that ends at y, and the two lanes.

    1. The estimate goes from the EKF's box, which starts at filter_y, down the right lane to
       the planner.
    2. The commands of the teleop node and of the planner go up the left lane to the drive
       controller, whose box ends at nodes_bottom_y.
    """
    collector_y = y + 41
    boxes_y = y + 50
    boxes_height = COMMANDS_HEIGHT - 56
    teleop_end, planner_start = 136, 146
    node_box(figure, INNER_START, teleop_end, boxes_y, boxes_height, (('a teleop node', ''),))
    node_box(figure, planner_start, BLOCK_END, boxes_y, boxes_height,
             (('the planner in move_base', ''),))
    figure.route(((INNER_END, filter_y + FILTER_HEIGHT / 2),
                  (ESTIMATE_LANE_X, filter_y + FILTER_HEIGHT / 2),
                  (ESTIMATE_LANE_X, boxes_y)), 'estimate-lane')

    teleop_x = (INNER_START + teleop_end) / 2
    planner_x = (planner_start + BLOCK_END) / 2
    figure.path(f'M{teleop_x},{boxes_y} V{collector_y}', 'command-line')
    figure.route(((planner_x, boxes_y), (planner_x, collector_y),
                  (COMMAND_LANE_X, collector_y), (COMMAND_LANE_X, nodes_bottom_y)), 'command')
    figure.text(INNER_START, y + 17, '↑ velocity commands', 'quiet')
    figure.text(INNER_START, y + 32, 'mobile_base_controller/cmd_vel', 'mono quiet')


def filter_and_way_back(figure: Figure, nodes_bottom_y: float) -> None:
    """Draw what both variants share below the two boxes that end at nodes_bottom_y.

    From the top: the measurements' arrows into the EKF, the EKF with its two outputs, the
    sketch of RViz, and the way back with who sends velocity commands.
    """
    rviz_y = filter_and_outputs(figure, nodes_bottom_y)
    rviz_panel(figure, rviz_y, RVIZ_HEIGHT)
    way_back(figure, nodes_bottom_y, nodes_bottom_y + INPUTS_HEIGHT, rviz_y + RVIZ_HEIGHT)


def loop_figure(hardware: Hardware) -> Figure:
    """Return the drawing of the loop for one variant: the real robot or the simulation.

    From the top: the part that the variants don't share, the drive controller and the IMU
    sample validator, and the part that the variants share.
    """
    height = (NODES_Y + NODES_HEIGHT + INPUTS_HEIGHT + FILTER_HEIGHT + OUTPUTS_HEIGHT
              + RVIZ_HEIGHT + COMMANDS_HEIGHT)
    figure = Figure(f'lv-loop-{hardware.name}', hardware.title, height, LOOP_WIDTH)
    hardware_block(figure, hardware)
    filter_and_way_back(figure, measurement_nodes(figure, hardware))
    return figure


LOOP_LEGEND = (
    '<ul class="lv-legend lv-lines">\n'
    '<li><span class="lv-line"></span>from the wheels</li>\n'
    '<li><span class="lv-line lv-from-imu"></span>from the IMU</li>\n'
    '<li><span class="lv-line lv-estimated"></span>the filter\'s estimate</li>\n'
    '<li><span class="lv-line lv-commanded"></span>commands, back to the wheels</li>\n'
    '</ul>')


def tab_set(name: str, tabs: Sequence[Tuple[str, str]]) -> str:
    """Return a set of content tabs as HTML, from (label, content) pairs.

    The HTML has the elements and classes of the tabs that the Markdown extension
    pymdownx.tabbed writes, so the site's theme shows, links and prints the set like a set from
    Markdown. A set from Markdown can't hold a drawing: Markdown parses the indented content of
    a tab as text, and the glossary's tooltips then break the text inside the SVG. The ids
    contain name, which keeps them apart from the numbered ids of the sets from Markdown.
    """
    group = f'__tabbed_{name.replace("-", "_")}'
    inputs = ''.join(f'<input{" checked" if index == 0 else ""} id="{group}_{index + 1}" '
                     f'name="{group}" type="radio">' for index in range(len(tabs)))
    labels = ''.join(f'<label for="{group}_{index + 1}">{escape(label)}</label>'
                     for index, (label, _) in enumerate(tabs))
    blocks = ''.join(f'<div class="tabbed-block">\n{content}\n</div>\n' for _, content in tabs)
    return (f'<div class="tabbed-set tabbed-alternate {name}-tabs" data-tabs="{name}:{len(tabs)}">'
            f'{inputs}<div class="tabbed-labels">{labels}</div>\n'
            f'<div class="tabbed-content">\n{blocks}</div>\n</div>')


def write_loop() -> None:
    """Write the include of the loop figure: a set of two tabs, one for each variant."""
    tabs = [(hardware.tab,
             '<figure class="lv-figure lv-drawing lv-loop">\n'
             f'{loop_figure(hardware).svg()}\n{LOOP_LEGEND}\n'
             f'<figcaption>{hardware.caption}</figcaption>\n</figure>')
            for hardware in (REAL_ROBOT, SIMULATION)]
    with open('includes/localization-loop.html', 'w', encoding='utf-8') as include:
        include.write(f'{style_element(RULES + LARGE_RULES + DRAWING_RULES + LOOP_RULES)}\n'
                      f'{tab_set("lv-loop", tabs)}\n')


def odom_frame() -> Figure:
    """Return the drawing of where the odom frame sits: the floor from above with both frames.

    The odom frame's origin is where the robot started, drawn as a dashed outline. The robot's
    pose is the same as in the loop figure's sketch of RViz.
    """
    figure = Figure('lv-odom-frame', 'The floor seen from above. The odom frame has its origin '
                    'where the robot started, with its x axis in the direction the robot faced. '
                    'The robot has driven a left curve. Its frame base_footprint lies at x and '
                    'y in the odom frame and is turned by the heading θ', ODOM_HEIGHT,
                    ODOM_WIDTH)
    left, right, top, bottom = BLOCK_START, BLOCK_END, 6, ODOM_HEIGHT - 6
    figure.rect(left, top, right - left, bottom - top, 'floor', corner_radius=5)
    grid = [f'M{x},{top + 1} V{bottom - 1}' for x in range(left + 26, right, 30)]
    grid += [f'M{left + 1},{line_y} H{right - 1}' for line_y in range(top + 14, bottom, 30)]
    figure.path(' '.join(grid), 'floor-grid')
    figure.text(left + 10, top + 18, 'The floor from above', 'strong')

    pose = pose_from(left + 56, bottom - 54)
    # Where the robot started: its outline, and the odom frame at the middle of its wheel axle
    small_robot(figure, pose.origin_x, pose.origin_y, 0, 'start-robot', 'start-robot')
    figure.text(left + 10, pose.origin_y + 42, 'the robot at the start', 'quiet')
    # The position's two parts: from the robot to the odom frame's axes
    figure.path(f'M{pose.robot_x:.1f},{pose.robot_y:.1f} V{pose.origin_y} '
                f'M{pose.robot_x:.1f},{pose.robot_y:.1f} H{pose.origin_x}', 'projection')
    odom_axes(figure, pose, 250, 116)
    figure.path(f'M{pose.robot_x:.1f},{pose.origin_y - 4} v8 '
                f'M{pose.origin_x - 4},{pose.robot_y:.1f} h8', 'foot')
    figure.text(pose.origin_x + 24, pose.origin_y + 15, 'odom', 'mono')
    transform_line(figure, pose)
    small_robot(figure, pose.robot_x, pose.robot_y, POSE_HEADING)
    heading_angle(figure, pose, 30)
    # The robot's own frame: its x axis points forward, its y axis to the left
    heading = math.radians(POSE_HEADING)
    figure.arrow(pose.robot_x, pose.robot_y, pose.robot_x + 40 * math.cos(heading),
                 pose.robot_y - 40 * math.sin(heading), 'frame-axis')
    figure.arrow(pose.robot_x, pose.robot_y, pose.robot_x - 40 * math.sin(heading),
                 pose.robot_y - 40 * math.cos(heading), 'frame-axis')
    figure.text(pose.robot_x - 12, pose.robot_y - 46, 'base_footprint', 'mono', 'middle')
    return figure


def write_odom_frame() -> None:
    """Write the include with the drawing of where the odom frame sits."""
    write_parts(
        'includes/localization-odom-frame.html', [odom_frame().svg()],
        "In this setup, after a fresh start, the origin of <code>odom</code> is where the robot's "
        'frame <code>base_footprint</code> was at the start, with its x axis in the direction '
        "that the robot faced. The pose (x, y, θ) is the robot's estimated position and heading "
        'in <code>odom</code>. The figure leaves the drift out.',
        RULES + DRAWING_RULES + ODOM_RULES, 'lv-drawing lv-odom')


def main() -> None:
    """Write the includes of the Localization page."""
    write_odom_frame()
    write_loop()
    write_fused_values()
    write_covariances()
    write_stamp_check()
    write_time_step()
    write_validator_form_checks()
    write_validator_time_check()
    write_validator_order_check()
    write_distance()


if __name__ == '__main__':
    main()
