"""Generate the figures of the Localization page as inline SVG with HTML legends.

The page is docs/packages/imu/robot_localization.md. The script writes two includes that follow
the site's light and dark colors:

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


def main() -> None:
    """Write the two includes of the Localization page."""
    write_fused_values()
    write_validator_checks()


if __name__ == '__main__':
    main()
