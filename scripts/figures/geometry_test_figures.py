"""Generate the figure on docs/development/geometry-test.md.

It writes includes/geometry-test-flow.html: what test_geometry_consistency.py reads, what it
compares with what, and what comes out, as inline SVG that follows the site's light and dark
colors. The values are Remo's, as in the page's table.

Run from the repository root after changing the figure:

    python3 scripts/figures/geometry_test_figures.py
"""

from typing import List
from typing import Sequence
from typing import Tuple

WIDTH = 360
HEIGHT = 484
# The URDF's side is the left column, the controller configuration's side the right one
LEFT_X = 12
RIGHT_X = 196
COLUMN_WIDTH = 152
LEFT_CENTER = LEFT_X + COLUMN_WIDTH / 2
RIGHT_CENTER = RIGHT_X + COLUMN_WIDTH / 2
ROW_HEIGHT = 36

# Remo's values in m, as on the page: remo_description 2.0.0 (config/remo/front_wheel.yaml) and
# diffbot_control/config/remo/geometry.yaml
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.144
WHEEL_LENGTH = 0.025
# The largest difference that the test accepts, in m: test_geometry_consistency.py's TOLERANCE
TOLERANCE = 0.0001
TOLERANCE_TEXT = f'{TOLERANCE * 1000:g} mm'
# The sketch of the wheels is to scale
PIXELS_PER_METER = 650

# A text line in a box: the text and its CSS class
Line = Tuple[str, str]

FG = 'var(--md-default-fg-color)'
FG_LIGHT = 'var(--md-default-fg-color--light)'
RULES = (
    ('.md-typeset figure.gt-figure', 'margin:1em auto;width:100%;max-width:480px'),
    ('.gt-fig', 'width:100%;height:auto;--gt-compare:#1e63c4;'
                'font-family:var(--md-text-font-family,Roboto),sans-serif'),
    ('[data-md-color-scheme=slate] .gt-fig', '--gt-compare:#82b1ff'),
    ('.gt-fig text', f'fill:{FG};font-size:10px'),
    ('.gt-fig .test', 'fill:var(--md-default-fg-color--lightest);'
                      'stroke:var(--md-default-fg-color--lighter);stroke-width:1'),
    ('.gt-fig .item', f'fill:var(--md-default-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.gt-fig .file', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.gt-fig .arrow', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.3'),
    ('.gt-fig .arrow-head', f'fill:{FG_LIGHT}'),
    ('.gt-fig .compare', 'fill:none;stroke:var(--gt-compare);stroke-width:1.8'),
    ('.gt-fig .compare-head', 'fill:var(--gt-compare)'),
    ('.gt-fig .name', 'font-size:11px'),
    ('.gt-fig .mono', 'font-family:var(--md-code-font-family,"Roboto Mono"),monospace;'
                      'font-size:9.5px'),
    ('.gt-fig .note', f'font-size:9.5px;fill:{FG_LIGHT}'),
    ('.gt-fig .band', 'font-weight:700'),
    ('.gt-fig .wheel', f'fill:var(--md-code-bg-color);stroke:{FG_LIGHT};stroke-width:1'),
    ('.gt-fig .collision', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.2;stroke-dasharray:3 2'),
    ('.gt-fig .ground', f'fill:none;stroke:{FG_LIGHT};stroke-width:1.5'),
    ('.gt-fig .extension', f'fill:none;stroke:{FG_LIGHT};stroke-width:1;stroke-dasharray:2 3'),
    ('.gt-fig .dimension', 'fill:none;stroke:var(--gt-compare);stroke-width:1.8'),
    ('.gt-fig .joint', f'fill:{FG}'),
)


def escape(text: str) -> str:
    """Return text with the characters escaped that have a meaning in SVG."""
    return text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def style_element(rules: Sequence[Tuple[str, str]]) -> str:
    """Return a style element with one CSS rule per (selector, declarations) pair."""
    lines = [f'{selector}{{{declarations}}}' for selector, declarations in rules]
    return '<style>\n' + '\n'.join(lines) + '\n</style>'


class Figure:
    """Collect the SVG elements of the figure."""

    def __init__(self, title: str) -> None:
        self.title = title
        self.elements: List[str] = []

    def box(self, x: float, y: float, width: float, height: float, css_class: str,
            lines: Sequence[Line] = ()) -> None:
        """Add a box with its text lines centered in it, 13 px apart."""
        self.elements.append(f'<rect class="{css_class}" x="{x}" y="{y}" width="{width}" '
                             f'height="{height}" rx="5"/>')
        for index, (text, text_class) in enumerate(lines):
            baseline = y + height / 2 + (index - (len(lines) - 1) / 2) * 13 + 4
            self.text(x + width / 2, baseline, text, text_class, 'middle')

    def arrow(self, path: str) -> None:
        """Add a line along an SVG path with an arrowhead at its end: what flows where."""
        self.elements.append(f'<path class="arrow" d="{path}" marker-end="url(#gt-head)"/>')

    def comparison(self, path: str) -> None:
        """Add a line with an arrowhead at both ends: what the test compares with what."""
        self.elements.append(f'<path class="compare" d="{path}" marker-start="url(#gt-compare)" '
                             'marker-end="url(#gt-compare)"/>')

    def line(self, path: str, css_class: str) -> None:
        """Add a line along an SVG path, without arrowheads."""
        self.elements.append(f'<path class="{css_class}" d="{path}"/>')

    def circle(self, x: float, y: float, radius: float, css_class: str) -> None:
        """Add a circle around (x, y)."""
        self.elements.append(
            f'<circle class="{css_class}" cx="{x:.1f}" cy="{y:.1f}" r="{radius:.1f}"/>')

    def text(self, x: float, y: float, text: str, css_class: str = '',
             anchor: str = 'start') -> None:
        """Add one line of text. anchor is start, middle or end."""
        class_attribute = f' class="{css_class}"' if css_class else ''
        self.elements.append(
            f'<text{class_attribute} x="{x}" y="{y}" text-anchor="{anchor}">{escape(text)}</text>')

    def svg(self) -> str:
        """Return the figure as an SVG element."""
        heads = ''.join(
            f'<marker id="{name}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="{size}" '
            f'markerHeight="{size}" orient="auto-start-reverse"><path class="{css_class}" '
            'd="M0,0 L10,5 L0,10 z"/></marker>'
            for name, css_class, size in (('gt-head', 'arrow-head', 7),
                                          ('gt-compare', 'compare-head', 4.5)))
        return (f'<svg class="gt-fig" viewBox="0 0 {WIDTH} {HEIGHT}" role="img" '
                f'aria-labelledby="gt-title"><title id="gt-title">{escape(self.title)}</title>'
                f'<defs>{heads}</defs>\n' + '\n'.join(self.elements) + '\n</svg>')


def draw_inputs(figure: Figure) -> None:
    """Add what goes in: the files of both sides, and roslaunch, which loads them."""
    figure.text(LEFT_X, 14, 'In', 'band')
    figure.box(LEFT_X, 20, COLUMN_WIDTH, 52, 'file', [
        ('Robot model (URDF)', 'name'), ('front_wheel.yaml', 'mono'), ('base.yaml', 'mono')])
    figure.box(RIGHT_X, 20, COLUMN_WIDTH, 52, 'file', [
        ('Controller configuration', 'name'), ('geometry.yaml', 'mono'),
        ('diffbot_control.yaml', 'mono')])
    figure.arrow(f'M{LEFT_CENTER},72 V89')
    figure.arrow(f'M{RIGHT_CENTER},72 V89')
    figure.box(LEFT_X, 90, RIGHT_X + COLUMN_WIDTH - LEFT_X, 52, 'item', [
        ('roslaunch --dump-params', 'mono'),
        ('reads the launch files of the robot and of the', 'note'),
        ('simulation, and prints their parameters', 'note')])
    figure.arrow(f'M{LEFT_CENTER},142 V199')
    figure.text(LEFT_CENTER + 8, 158, 'robot_description', 'mono')
    figure.arrow(f'M{RIGHT_CENTER},142 V199')
    figure.text(RIGHT_CENTER + 8, 158, 'controller', 'note')
    figure.text(RIGHT_CENTER + 8, 170, 'parameters', 'note')


def draw_wheels(figure: Figure, y: float) -> None:
    """Add Remo's drive wheels seen from behind, in a box that starts at y.

    The sketch shows the two lengths that the URDF holds: the distance between the wheel joints
    and the radius of the wheels' collision spheres.
    """
    radius = WHEEL_RADIUS * PIXELS_PER_METER
    half_length = WHEEL_LENGTH * PIXELS_PER_METER / 2
    joint_y, dimension_y = y + 56, y + 21
    left_x = LEFT_CENTER - WHEEL_SEPARATION * PIXELS_PER_METER / 2
    right_x = LEFT_CENTER + WHEEL_SEPARATION * PIXELS_PER_METER / 2

    figure.box(LEFT_X, y, COLUMN_WIDTH, 98, 'item')
    figure.text(LEFT_CENTER, y + 13, f'wheel joints {WHEEL_SEPARATION} m apart', 'note', 'middle')
    figure.line(f'M{left_x:.1f},{dimension_y} H{right_x:.1f}', 'dimension')
    for joint_x in (left_x, right_x):
        figure.line(f'M{joint_x:.1f},{dimension_y - 4} V{dimension_y + 4}', 'dimension')
        figure.line(f'M{joint_x:.1f},{dimension_y + 4} V{joint_y}', 'extension')
        figure.elements.append(
            f'<rect class="wheel" x="{joint_x - half_length:.1f}" y="{joint_y - radius:.1f}" '
            f'width="{2 * half_length:.1f}" height="{2 * radius:.1f}" rx="4"/>')
        figure.circle(joint_x, joint_y, radius, 'collision')
        figure.circle(joint_x, joint_y, 2.2, 'joint')
    figure.line(f'M{LEFT_X + 6},{joint_y + radius:.1f} H{LEFT_X + COLUMN_WIDTH - 6}', 'ground')
    figure.line(f'M{right_x:.1f},{joint_y} V{joint_y + radius:.1f}', 'dimension')
    figure.text(LEFT_CENTER, y + 92, f'collision radius {WHEEL_RADIUS} m', 'note', 'middle')


def draw_comparisons(figure: Figure) -> None:
    """Add what is compared: values of the URDF with parameters of the controller."""
    figure.box(4, 176, 352, 236, 'test')
    figure.text(LEFT_X, 191, 'Compared', 'band')
    gap_start, gap_end = LEFT_X + COLUMN_WIDTH + 3, RIGHT_X - 3

    # The two lengths, each within the tolerance
    draw_wheels(figure, 198)
    figure.box(RIGHT_X, 198, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('wheel_separation', 'mono'), (f'{WHEEL_SEPARATION}, within {TOLERANCE_TEXT}', 'note')])
    figure.comparison(f'M{gap_start},216 H{gap_end}')
    figure.box(RIGHT_X, 260, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('wheel_radius', 'mono'), (f'{WHEEL_RADIUS}, within {TOLERANCE_TEXT}', 'note')])
    figure.comparison(f'M{gap_start},278 H{gap_end}')

    # The wheel joints that the controller names
    figure.box(LEFT_X, 304, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('wheel joints: continuous,', 'name'), ('with a velocity interface', 'note')])
    figure.box(RIGHT_X, 304, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('left_wheel, right_wheel', 'mono'), ("the wheel joints' names", 'note')])
    figure.comparison(f'M{gap_start},322 H{gap_end}')

    # The hardware interface's joints are compared with the controller's wheels, the row above
    figure.comparison(f'M{RIGHT_CENTER},343 V365')
    figure.box(RIGHT_X, 368, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('hardware_interface/joints', 'mono'), ('the same joints, left first', 'note')])


def draw_results(figure: Figure) -> None:
    """Add what comes out: the test passes, or it fails with a message for each difference."""
    figure.text(LEFT_X, 430, 'Out', 'band')
    figure.arrow(f'M{LEFT_CENTER},412 V439')
    figure.arrow(f'M{RIGHT_CENTER},412 V439')
    figure.box(LEFT_X, 440, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('All agree', 'name'), ('the test passes', 'note')])
    figure.box(RIGHT_X, 440, COLUMN_WIDTH, ROW_HEIGHT, 'item', [
        ('A difference', 'name'), ('the test fails and names it', 'note')])


def geometry_test_figure() -> Figure:
    """Return the figure of the geometry test's inputs, comparisons and results."""
    figure = Figure('The geometry test: the robot model and the controller configuration go in, '
                    'the test compares their wheel values and joints, and it passes or fails')
    draw_inputs(figure)
    draw_comparisons(figure)
    draw_results(figure)
    return figure


def main() -> None:
    """Write the include with the figure and its caption."""
    caption = ("What the geometry test reads, compares and reports, with Remo's values. A blue "
               'double arrow connects two things that must agree. The sketch shows the wheels '
               'from behind, to scale.')
    with open('includes/geometry-test-flow.html', 'w', encoding='utf-8') as include:
        include.write(f'{style_element(RULES)}\n<figure class="gt-figure">\n'
                      f'{geometry_test_figure().svg()}\n<figcaption>{caption}</figcaption>\n'
                      '</figure>\n')


if __name__ == '__main__':
    main()
