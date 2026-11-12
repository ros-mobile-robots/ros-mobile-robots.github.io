"""Generate the figures of the Localization page as inline SVG with HTML legends.

The page is docs/packages/imu/robot_localization.md. The script writes three includes that follow
the site's light and dark colors:

- includes/localization-flows.html: what goes into the filter and what comes out, as two
  drawings. The first shows the real robot from above with its wheel encoders and its IMU, and
  the way of their values through the firmware and the ROS nodes to the filter. The second shows
  the same for the simulation, where the values come from Gazebo, and the filter's output as RViz
  shows it. The drawings stand side by side on a wide screen and one below the other on a phone.
- includes/localization-fused-values.html: the two lists odom0_config and imu0_config of
  diffbot_control/config/ekf.yaml in the diffbot repository, each as a grid of five rows of
  three, with the values that the filter fuses. The two grids stand side by side on a wide
  screen and one below the other on a phone.
- includes/localization-validator-checks.html: the five checks of
  diffbot_control/scripts/imu_validator.py. Checks 1 to 3 look at the sample's form. Check 4
  compares the sample's stamp with the ROS time, drawn on a time axis to scale. Check 5 compares
  the stamp with the last forwarded sample's. The values are those of
  diffbot_control/launch/ekf.launch.

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

# The two drawings of what goes in and what comes out: FLOW_WIDTH wide, with a left column for
# the wheel odometry and the topic, and a right column for the IMU and the transform
FLOW_WIDTH = 330
LEFT_COLUMN_START = 6
LEFT_COLUMN_END = 176
RIGHT_COLUMN_START = 182
RIGHT_COLUMN_END = 324
LEFT_ARROW_X = 20
RIGHT_ARROW_X = 190
LABEL_GAP = 8  # between an arrow and the text next to it
LINE_HEIGHT = 15
ROBOT_CENTER_X = 165
# The robot seen from above, in units from its center, with its front pointing up
BODY_HALF_WIDTH = 46
BODY_HALF_LENGTH = 58
AXLE_Y = -14  # the wheel axle lies in front of the center
ENCODER_X = 14  # of the right wheel's encoder, the left one is mirrored
IMU_CHIP = (-22, 14, 14)  # x, y and edge length: behind the axle, left of the center line
# The heading of the robot that the RViz panel shows, in degrees
RVIZ_HEADING = 35

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

# The figures of what goes in and what comes out. The RViz panel has RViz's own dark background
# in both color schemes, so its lines and text have fixed light colors
RVIZ_BG = '#2f3338'
RVIZ_FG = '#eceff1'
KATEX_MAIN = 'font-family:KaTeX_Main,"Times New Roman",serif'
KATEX_MATH = 'font-family:KaTeX_Math,"Times New Roman",serif'
FLOW_RULES = (
    ('.lv-flows', 'display:flex;flex-wrap:wrap;justify-content:center;align-items:flex-start;'
                  'gap:.8em 16px;margin:1em 0'),
    ('.md-typeset .lv-flows figure.lv-figure', 'margin:0;flex:1 1 300px;width:auto;'
                                               'max-width:358px'),
    ('.md-typeset .lv-flows figcaption', 'margin:.4em auto 0;font-size:.7rem'),
    ('.lv-flows .lv-fig text', 'font-size:12px'),
    ('.lv-flows .lv-fig .mono', 'font-size:11.5px'),
    # The symbols are set in the fonts of the page's formulas, which KaTeX's style sheet loads
    ('.lv-flows .lv-fig .variable', f'{KATEX_MATH};font-style:italic;font-size:14px'),
    ('.lv-flows .lv-fig .vector', f'{KATEX_MAIN};font-weight:700;font-size:14px'),
    ('.lv-flows .lv-fig .subscript', f'{KATEX_MAIN};font-size:10.5px'),
    ('.lv-flows .lv-fig .index', f'{KATEX_MATH};font-style:italic'),
    # The same badge as an HTML element, for the page's two Mermaid figures of the launch files
    ('.md-typeset p.lv-badge-line', 'margin:1em 0 -.4em;text-align:center'),
    ('.md-typeset .lv-badge', f'display:inline-block;padding:.05em .9em;border-radius:1em;'
                              f'background:{FG};color:{BG};font-weight:700;font-size:.68rem'),
    ('.lv-flows .lv-fig .badge', f'fill:{FG}'),
    ('.lv-flows .lv-fig .badge-text', f'fill:{BG};font-weight:700'),
    ('.lv-flows .lv-fig .robot-body', f'fill:{BG};stroke:{FG_LIGHT};stroke-width:1.4'),
    ('.lv-flows .lv-fig .robot-wheel', f'fill:{FG_LIGHT};stroke:none'),
    ('.lv-flows .lv-fig .robot-part', f'fill:none;stroke:{FG_LIGHT};stroke-width:1'),
    ('.lv-flows .lv-fig .encoder', f'fill:var(--lv-wheels);stroke:{BG};stroke-width:1'),
    ('.lv-flows .lv-fig .imu-chip', f'fill:var(--lv-imu);stroke:{BG};stroke-width:1'),
    ('.lv-flows .lv-fig .motion-line', f'fill:none;stroke:{FG};stroke-width:1.4'),
    ('.lv-flows .lv-fig .motion-head', f'fill:{FG};stroke:none'),
    ('.lv-flows .lv-fig .wheels-line', 'fill:none;stroke:var(--lv-wheels);stroke-width:1.8'),
    ('.lv-flows .lv-fig .wheels-head', 'fill:var(--lv-wheels);stroke:none'),
    ('.lv-flows .lv-fig .wheels-wire', 'fill:none;stroke:var(--lv-wheels);stroke-width:1.3'),
    ('.lv-flows .lv-fig .imu-line', 'fill:none;stroke:var(--lv-imu);stroke-width:1.8'),
    ('.lv-flows .lv-fig .imu-head', 'fill:var(--lv-imu);stroke:none'),
    ('.lv-flows .lv-fig .imu-wire', 'fill:none;stroke:var(--lv-imu);stroke-width:1.3'),
    ('.lv-flows .lv-fig .estimate-line', 'fill:none;stroke:var(--lv-estimate);stroke-width:1.8'),
    ('.lv-flows .lv-fig .estimate-head', 'fill:var(--lv-estimate);stroke:none'),
    ('.lv-flows .lv-fig .hat', f'fill:none;stroke:{FG};stroke-width:1.1'),
    ('.lv-flows .lv-fig .filter', f'fill:{BG};stroke:var(--lv-estimate);stroke-width:1.8'),
    ('.lv-flows .lv-fig .gazebo', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};'
                                  'stroke-width:1;stroke-dasharray:4 3'),
    ('.lv-flows .lv-fig .rviz', f'fill:{RVIZ_BG};stroke:{FG_LIGHT};stroke-width:1'),
    ('.lv-flows .lv-fig .rviz-grid', 'fill:none;stroke:rgba(255,255,255,.16);stroke-width:1'),
    ('.lv-flows .lv-fig .rviz-text', f'fill:{RVIZ_FG}'),
    ('.lv-flows .lv-fig .rviz-axis-line', f'fill:none;stroke:{RVIZ_FG};stroke-width:1.4'),
    ('.lv-flows .lv-fig .rviz-axis-head', f'fill:{RVIZ_FG};stroke:none'),
    ('.lv-flows .lv-fig .rviz-transform', f'fill:none;stroke:{RVIZ_FG};stroke-width:1.3;'
                                          'stroke-dasharray:4 3'),
    ('.lv-flows .lv-fig .rviz-body', f'fill:#59616b;stroke:{RVIZ_FG};stroke-width:1.2'),
    ('.lv-flows .lv-fig .rviz-wheel', f'fill:{RVIZ_FG};stroke:none'),
    ('.lv-flows .lv-fig .rviz-pose-line', 'fill:none;stroke:#35d19a;stroke-width:2.2'),
    ('.lv-flows .lv-fig .rviz-pose-head', 'fill:#35d19a;stroke:none'),
    ('.lv-flows .lv-fig .rviz-trail-line', 'fill:none;stroke:#35d19a;stroke-width:1.6;'
                                           'opacity:.5'),
    ('.lv-flows .lv-fig .rviz-trail-head', 'fill:#35d19a;stroke:none;opacity:.5'),
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

    def __init__(self, name: str, title: str, height: int, width: int = WIDTH) -> None:
        self.name = name  # unique on the page: the id of the drawing's title starts with it
        self.title = title
        self.width = width
        self.height = height
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
        """Add a straight arrow in the color of a series: wheels, imu, estimate or motion."""
        direction_x, direction_y = x_end - x_start, y_end - y_start
        length = math.hypot(direction_x, direction_y)
        # The line ends inside the head, so that its blunt end doesn't show at the tip
        line_end_x = x_end - direction_x / length * 5
        line_end_y = y_end - direction_y / length * 5
        self.path(f'M{x_start:.1f},{y_start:.1f} L{line_end_x:.1f},{line_end_y:.1f}',
                  f'{series}-line')
        self.path(arrow_head(x_end, y_end, direction_x, direction_y), f'{series}-head')

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle."""
        self.elements.append(f'<circle class="{css_class}" cx="{x:.1f}" cy="{y:.1f}" '
                             f'r="{radius}"/>')

    def svg(self) -> str:
        """Return the drawing as an SVG element."""
        title_id = f'{self.name}-title'
        return (f'<svg class="lv-fig" viewBox="0 0 {self.width} {self.height}" role="img" '
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
    with open(path, 'w') as include:
        include.write(f'{style_element(rules)}\n<figure class="{classes}">\n'
                      + '\n'.join(parts)
                      + f'\n<figcaption>{caption}</figcaption>\n</figure>\n')


def band(title: str) -> str:
    """Return the HTML heading of one part of a figure."""
    return f'<p class="lv-band">{escape(title)}</p>'


def sample_legend(samples: Sequence[Sample]) -> str:
    """Return an HTML list with one line per sample: its mark, what happens to it, and why."""
    lines = []
    for sample in samples:
        mark_class = 'lv-mark' if sample.forwarded else 'lv-mark lv-drop'
        verdict = '✓ forwarded' if sample.forwarded else '✕ dropped'
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


def write_validator_checks() -> None:
    """Write the include with the five checks of the IMU sample validator."""
    time_figure, time_samples = time_check()
    order_figure, order_samples = order_check()
    parts = [
        band("Checks 1 to 3: the sample's form"),
        form_checks().svg(),
        band('Check 4: the stamp against the ROS time'),
        time_figure.svg(),
        sample_legend(time_samples),
        band("Check 5: the stamp against the last forwarded sample's"),
        order_figure.svg(),
        sample_legend(order_samples),
    ]
    write_parts(
        'includes/localization-validator-checks.html', parts,
        'The five checks of the IMU sample validator. Both axes show stamps, with time running '
        f"to the right and one tick every {SAMPLE_PERIOD_MS / 1000:.2f} s, the IMU's sample "
        'period at 50 Hz. A solid green mark is a forwarded sample, a dashed red mark a dropped '
        'one. Each example assumes that the sample passes the other checks.',
        RULES + LARGE_RULES, 'lv-large')


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
        wheel_x = center_x + side * (BODY_HALF_WIDTH + 2) + (-13 if side < 0 else 0)
        figure.rect(wheel_x, axle_y - 20, 13, 40, 'robot-wheel', corner_radius=4)
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
    wheels_exit_x = center_x - 30
    bus_start_x = min(center_x - sensor_x, wheels_exit_x)
    figure.path(f'M{center_x - sensor_x},{axle_y} V{bus_y} M{center_x + sensor_x},{axle_y} '
                f'V{bus_y} H{bus_start_x} M{wheels_exit_x},{bus_y} V{rear_y}', 'wheels-wire')
    for side in (-1, 1):
        figure.circle(center_x + side * sensor_x, axle_y, 4.5, 'encoder')
    chip_x, chip_y, chip_size = IMU_CHIP
    imu_exit_x = center_x + 30
    figure.path(f'M{center_x + chip_x + chip_size / 2},{center_y + chip_y + chip_size} '
                f'V{center_y + 34} H{imu_exit_x} V{rear_y}', 'imu-wire')
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


def sensor_key(figure: Figure, y: float, wheel_sensor: str, imu: str) -> None:
    """Add the key of the two marks on the robot, left of the robot."""
    figure.circle(13, y - 4, 4.5, 'encoder')
    figure.text(23, y, wheel_sensor)
    figure.rect(8.5, y + 9.5, 9, 9, 'imu-chip', corner_radius=2)
    figure.text(23, y + 18, imu)


def badge(figure: Figure, label: str) -> None:
    """Add the pill at the top left that says which of the two figures this is."""
    width = 9 + 6.6 * len(label)
    figure.rect(6, 6, width, 19, 'badge', corner_radius=9.5)
    figure.text(6 + width / 2, 19.5, label, 'badge-text', 'middle')


def node_box(figure: Figure, column_start: float, column_end: float, y: float, height: float,
             lines: Sequence[Tuple[str, str]]) -> None:
    """Add a box that fills a column, with centered lines of text, each a (text, class) pair."""
    figure.rect(column_start, y, column_end - column_start, height, 'item', corner_radius=5)
    middle = (column_start + column_end) / 2
    first_baseline = y + height / 2 + 4 - (len(lines) - 1) * LINE_HEIGHT / 2
    for index, (content, css_class) in enumerate(lines):
        figure.text(middle, first_baseline + index * LINE_HEIGHT, content, css_class, 'middle')


def filter_and_outputs(figure: Figure, y: float) -> float:
    """Draw the inputs' arrows into the EKF, the EKF and its two outputs' arrows.

    y is where the arrows into the EKF start. Return the y position where the two output
    arrows end: the left one for the topic, the right one for the transform.
    """
    filter_y = y + 62
    figure.arrow(LEFT_ARROW_X, y, LEFT_ARROW_X, filter_y, 'wheels')
    label_x = LEFT_ARROW_X + LABEL_GAP
    figure.rich_text(label_x, y + 18, Z_WHEELS)
    figure.text(label_x, y + 34, 'mobile_base_controller', 'mono')
    figure.text(label_x, y + 48, '/odom', 'mono')
    figure.arrow(RIGHT_ARROW_X, y, RIGHT_ARROW_X, filter_y, 'imu')
    label_x = RIGHT_ARROW_X + LABEL_GAP
    figure.rich_text(label_x, y + 18, Z_IMU)
    figure.text(label_x, y + 34, 'imu/data_validated', 'mono')

    figure.rect(LEFT_COLUMN_START, filter_y, RIGHT_COLUMN_END - LEFT_COLUMN_START, 46, 'filter',
                corner_radius=5)
    figure.rich_text(ROBOT_CENTER_X, filter_y + 19,
                     (Span('EKF', 'strong'), Span(', the node '),
                      Span('ekf_localization', 'mono')), 'middle')
    figure.estimate_text(58, filter_y + 36, (Span(": the filter's estimate of the state "),
                                             Span('x', 'vector')))

    outputs_y = filter_y + 46
    end_y = outputs_y + 80
    figure.arrow(LEFT_ARROW_X, outputs_y, LEFT_ARROW_X, end_y, 'estimate')
    label_x = LEFT_ARROW_X + LABEL_GAP
    figure.estimate_text(label_x, outputs_y + 18, PUBLISHED_INDEX)
    figure.rich_text(label_x, outputs_y + 34, PUBLISHED)
    figure.text(label_x, outputs_y + 50, 'odometry/filtered', 'mono')
    figure.text(label_x, outputs_y + 65, 'with covariances')
    figure.arrow(RIGHT_ARROW_X, outputs_y, RIGHT_ARROW_X, end_y, 'estimate')
    label_x = RIGHT_ARROW_X + LABEL_GAP
    figure.rich_text(label_x, outputs_y + 26, POSE + (Span(' as transform'),))
    figure.text(label_x, outputs_y + 42, 'odom →', 'mono')
    figure.text(label_x, outputs_y + 56, 'base_footprint', 'mono')
    return end_y


def robot_flow() -> Figure:
    """Return the drawing for the real robot: from its sensors to the filter's two outputs."""
    figure = Figure('lv-flow-robot', 'On the real robot: the wheel encoders and the IMU, the '
                    'firmware, the drive controller and the IMU sample validator, the EKF, and '
                    "the EKF's two outputs with the nodes that use them", 544, FLOW_WIDTH)
    badge(figure, 'Real robot')
    center_y = 98
    rear_y = robot_from_above(figure, center_y, simulated=False)
    sensor_key(figure, center_y + 34, 'wheel encoder', 'IMU (BNO055)')

    firmware_y = rear_y + 22
    figure.arrow(ROBOT_CENTER_X - 30, rear_y, ROBOT_CENTER_X - 30, firmware_y, 'wheels')
    figure.arrow(ROBOT_CENTER_X + 30, rear_y, ROBOT_CENTER_X + 30, firmware_y, 'imu')
    figure.rect(LEFT_COLUMN_START, firmware_y, RIGHT_COLUMN_END - LEFT_COLUMN_START, 24, 'item',
                corner_radius=5)
    figure.text(ROBOT_CENTER_X, firmware_y + 16, 'Teensy: firmware, over USB to rosserial',
                anchor='middle')

    nodes_y = firmware_y + 24 + 46
    figure.arrow(LEFT_ARROW_X, firmware_y + 24, LEFT_ARROW_X, nodes_y, 'wheels')
    figure.text(LEFT_ARROW_X + LABEL_GAP, nodes_y - 27, 'wheel angles')
    figure.text(LEFT_ARROW_X + LABEL_GAP, nodes_y - 12, 'measured_joint_states', 'mono')
    figure.arrow(RIGHT_ARROW_X, firmware_y + 24, RIGHT_ARROW_X, nodes_y, 'imu')
    figure.text(RIGHT_ARROW_X + LABEL_GAP, nodes_y - 27, 'IMU samples')
    figure.text(RIGHT_ARROW_X + LABEL_GAP, nodes_y - 12, 'imu/data_raw', 'mono')
    node_box(figure, LEFT_COLUMN_START, LEFT_COLUMN_END, nodes_y, 54,
             (('diffbot_base with the', ''), ('drive controller', ''),
              ('mobile_base_controller', 'mono')))
    node_box(figure, RIGHT_COLUMN_START, RIGHT_COLUMN_END, nodes_y, 54,
             (('IMU sample validator', ''), ('imu_validator', 'mono')))

    users_y = filter_and_outputs(figure, nodes_y + 54)
    node_box(figure, LEFT_COLUMN_START, LEFT_COLUMN_END, users_y, 46,
             (('the planners in move_base', ''), ('and Move Base Flex, RViz', '')))
    node_box(figure, RIGHT_COLUMN_START, RIGHT_COLUMN_END, users_y, 46,
             (('TF: RViz, amcl,', ''), ("the planners' costmaps", '')))
    return figure


def rviz_panel(figure: Figure, y: float, height: float) -> None:
    """Draw a sketch of RViz with the output: the robot at the transform, and kept pose arrows.

    The transform's dashed line, the axes' letters and the heading's angle are annotations
    that RViz doesn't draw.
    """
    left, right = LEFT_COLUMN_START, RIGHT_COLUMN_END
    figure.rect(left, y, right - left, height, 'rviz', corner_radius=5)
    grid = [f'M{x},{y + 1} V{y + height - 1}' for x in range(left + 30, right, 30)]
    grid += [f'M{left + 1},{line_y} H{right - 1}'
             for line_y in range(int(y) + 28, int(y + height), 30)]
    figure.path(' '.join(grid), 'rviz-grid')
    figure.rich_text(left + 10, y + 18, (Span('RViz', 'strong'), Span(', fixed frame '),
                                         Span('odom', 'mono')), css_class='rviz-text')

    origin_x, origin_y = left + 42, y + height - 26
    figure.arrow(origin_x, origin_y, origin_x + 50, origin_y, 'rviz-axis')
    figure.arrow(origin_x, origin_y, origin_x, origin_y - 50, 'rviz-axis')
    figure.rich_text(origin_x + 55, origin_y + 4, X, css_class='rviz-text')
    figure.rich_text(origin_x - 3, origin_y - 55, Y, css_class='rviz-text')
    figure.text(origin_x - 2, origin_y + 17, 'odom', 'mono rviz-text', 'middle')

    robot_x, robot_y = left + 216, y + 72
    heading = math.radians(RVIZ_HEADING)
    # The poses that an Odometry display keeps: the robot came along a left curve
    trail = ((70, -1, 0), (100, -5, 9), (128, -14, 18), (152, -28, 26))
    for offset_x, offset_y, degrees in trail:
        angle = math.radians(degrees)
        start_x, start_y = origin_x + offset_x, origin_y + offset_y
        figure.arrow(start_x, start_y, start_x + 15 * math.cos(angle),
                     start_y - 15 * math.sin(angle), 'rviz-trail')
    # The transform odom → base_footprint, with its name along the line, nearer to the origin,
    # so that the name ends before the robot
    figure.path(f'M{origin_x},{origin_y} L{robot_x},{robot_y}', 'rviz-transform')
    slope = math.degrees(math.atan2(robot_y - origin_y, robot_x - origin_x))
    name_x = origin_x + 0.37 * (robot_x - origin_x)
    name_y = origin_y + 0.37 * (robot_y - origin_y)
    figure.elements.append(
        f'<text class="rviz-text" text-anchor="middle" transform="translate({name_x:.1f},'
        f'{name_y:.1f}) rotate({slope:.1f})" y="-7">odom → base_footprint</text>')
    # The robot where the transform puts it, turned by the heading
    figure.elements.append(
        f'<g transform="translate({robot_x},{robot_y}) rotate({90 - RVIZ_HEADING}) scale(.42)">'
        '<rect class="rviz-wheel" x="-61" y="-34" width="13" height="40" rx="4"/>'
        '<rect class="rviz-wheel" x="48" y="-34" width="13" height="40" rx="4"/>'
        '<rect class="rviz-body" x="-46" y="-58" width="92" height="116" rx="14"/></g>')
    figure.path(f'M{robot_x},{robot_y} h44', 'rviz-transform')
    arc_radius = 34
    figure.path(f'M{robot_x + arc_radius},{robot_y} A{arc_radius},{arc_radius} 0 0 0 '
                f'{robot_x + arc_radius * math.cos(heading):.1f},'
                f'{robot_y - arc_radius * math.sin(heading):.1f}', 'rviz-axis-line')
    figure.rich_text(robot_x + arc_radius + 5, robot_y - 7, THETA, css_class='rviz-text')
    figure.arrow(robot_x, robot_y, robot_x + 56 * math.cos(heading),
                 robot_y - 56 * math.sin(heading), 'rviz-pose')
    figure.text(right - 8, y + height - 9, 'Odometry display: kept pose arrows', 'rviz-text',
                'end')


def simulation_flow() -> Figure:
    """Return the drawing for the simulation: from Gazebo to the filter and to RViz."""
    figure = Figure('lv-flow-simulation', "In the simulation: Gazebo's wheel joints and its IMU "
                    'sensor, the drive controller, the adapter and the IMU sample validator, the '
                    "EKF, and the EKF's two outputs in a sketch of RViz", 760, FLOW_WIDTH)
    badge(figure, 'Simulation')
    panel_y = 30
    center_y = 116
    inner_y = center_y + BODY_HALF_LENGTH + 22
    figure.rect(LEFT_COLUMN_START, panel_y, RIGHT_COLUMN_END - LEFT_COLUMN_START,
                inner_y + 54 + 8 - panel_y, 'gazebo', corner_radius=5)
    figure.text(LEFT_COLUMN_START + 10, panel_y + 18, 'Gazebo', 'strong')
    rear_y = robot_from_above(figure, center_y, simulated=True)
    sensor_key(figure, center_y + 34, 'wheel joint', 'IMU sensor')
    figure.arrow(ROBOT_CENTER_X - 30, rear_y, ROBOT_CENTER_X - 30, inner_y, 'wheels')
    figure.arrow(ROBOT_CENTER_X + 30, rear_y, ROBOT_CENTER_X + 30, inner_y, 'imu')
    node_box(figure, LEFT_COLUMN_START + 6, LEFT_COLUMN_END - 2, inner_y, 54,
             (('gazebo_ros_control with', ''), ('the drive controller', ''),
              ('mobile_base_controller', 'mono')))
    node_box(figure, RIGHT_COLUMN_START + 2, RIGHT_COLUMN_END - 6, inner_y, 54,
             (("the IMU sensor's", ''), ('plugin', '')))

    # The IMU's samples pass two nodes on their way, the wheel odometry goes straight on
    adapter_y = inner_y + 54 + 38
    label_x = RIGHT_ARROW_X + LABEL_GAP
    figure.arrow(RIGHT_ARROW_X, inner_y + 54, RIGHT_ARROW_X, adapter_y, 'imu')
    figure.text(label_x, adapter_y - 9, 'imu/gazebo', 'mono')
    node_box(figure, RIGHT_COLUMN_START, RIGHT_COLUMN_END, adapter_y, 30,
             (('imu_raw_adapter', 'mono'),))
    validator_y = adapter_y + 30 + 30
    figure.arrow(RIGHT_ARROW_X, adapter_y + 30, RIGHT_ARROW_X, validator_y, 'imu')
    figure.text(label_x, validator_y - 10, 'imu/data_raw', 'mono')
    node_box(figure, RIGHT_COLUMN_START, RIGHT_COLUMN_END, validator_y, 46,
             (('IMU sample validator', ''), ('imu_validator', 'mono')))
    inputs_y = validator_y + 46
    figure.path(f'M{LEFT_ARROW_X},{inner_y + 54} V{inputs_y}', 'wheels-line')

    rviz_y = filter_and_outputs(figure, inputs_y)
    rviz_panel(figure, rviz_y, 170)
    return figure


def write_flows() -> None:
    """Write the include with the two drawings of what goes into the filter and what comes out."""
    captions = (
        'The real robot, seen from above and not to scale.',
        "The simulation. The dark panel is a sketch of RViz with the EKF's two outputs.",
    )
    figures = [f'<figure class="lv-figure">\n{drawing.svg()}\n<figcaption>{caption}'
               '</figcaption>\n</figure>'
               for drawing, caption in zip((robot_flow(), simulation_flow()), captions)]
    with open('includes/localization-flows.html', 'w') as include:
        include.write(f'{style_element(RULES + LARGE_RULES + FLOW_RULES)}\n'
                      '<div class="lv-flows">\n' + '\n'.join(figures) + '\n</div>\n')


def main() -> None:
    """Write the three includes of the Localization page."""
    write_flows()
    write_fused_values()
    write_validator_checks()


if __name__ == '__main__':
    main()
