"""Browser tests of the interactive figure on the Simulation Test page, in the built site.

Build the site first, then run from the repository root:

    mkdocs build --strict
    python -m unittest discover -s tests/browser -v

The tests take the figure's values from scripts/figures/motion_lab_figures.py, which writes the
figure's include. They compute each expected number with the script's model in the test's own
process and compare the page's numbers with it. The page carries no expected results.
"""

import contextlib
import html
import math
import re
from typing import Any
from typing import Dict
from typing import Iterator
from typing import List
from typing import Literal
from typing import NamedTuple
from typing import Tuple
import unittest

from local_site import chromium
from local_site import open_page
from local_site import OpenedPage
from local_site import served_site
from local_site import SITE_HOSTS
# local_site puts scripts/figures on the import path
import motion_lab_figures as figures
from playwright.sync_api import Browser
from playwright.sync_api import Error as PlaywrightError
from playwright.sync_api import Page

REMO = figures.ROBOTS[figures.EXAMPLE_ROBOT]
PAGE = '/development/simulation-test/'
FIGURE = 'figure[data-ix="motion-lab"]'
# The figure starts within this time, also on a slow CI runner
START_TIMEOUT_MS = 15000
MILLISECONDS_PER_SECOND = 1000
# The model's results are exact up to floating-point rounding
MODEL_PLACES = 9
# The tables show three decimals: a shown value lies within half of its last digit of the value,
# plus floating-point slack
SHOWN_ROUNDING = 0.5e-3 + 1e-9
# The check table's two results, whole
PASSES = '✔ passes'
FAILS = '✖ fails'
# The drawing's sample at the drive's start, its middle and its end
FIRST_SAMPLE = 0
MIDDLE_SAMPLE = figures.DRAWING.samples // 2
LAST_SAMPLE = figures.DRAWING.samples
# Both drawings write coordinates to a tenth of a pixel, the live one sometimes unrounded
DRAWN_ROUNDING = 0.1 + 1e-9
# The shapes of a drawing's robots and the command's cross: each with the view of its group, its
# kind, and its coordinates (a polygon's points, a circle's center and radius, a line's ends, the
# numbers of the cross's path)
SHAPES = """svg => [...svg.querySelectorAll(
    '.ix-robot polygon, .ix-robot circle, .ix-robot line, .ix-cross')].map((shape) => {
  const views = ['ix-start', 'ix-model', 'ix-odometry', 'ix-command'];
  const view = views.find((name) => shape.closest('.ix-robot').classList.contains(name));
  const coordinates = {
    polygon: () => shape.getAttribute('points').split(/[ ,]/),
    circle: () => ['cx', 'cy', 'r'].map((name) => shape.getAttribute(name)),
    line: () => ['x1', 'y1', 'x2', 'y2'].map((name) => shape.getAttribute(name)),
    path: () => shape.getAttribute('d').match(/-?[0-9.]+/g),
  }[shape.tagName]();
  return { view, kind: shape.tagName, coordinates: coordinates.map(Number) };
})"""
# The centers (px) of the live drawing's two markers of base_footprint, each found by its role:
# the odometry's ring, and the true pose's dot, which is not the dot of the axle's midpoint
MARKER_CENTERS = """figure => {
  const center = (selector) => {
    const marker = document.querySelector(`${figure} .ix-live-plot ${selector}`);
    return { x: Number(marker.getAttribute('cx')), y: Number(marker.getAttribute('cy')) };
  };
  return {
    odometry: center('.ix-odometry .ix-ring'),
    model: center('.ix-model .ix-dot:not(.ix-axle)'),
  };
}"""
# The texts of the tables' cells that are wider than their cell
CUT_CELLS = """figure => [...document.querySelectorAll(
    `${figure} .ix-table td, ${figure} .ix-table th`)]
    .filter((cell) => cell.scrollWidth > cell.clientWidth + 1).map((cell) => cell.innerText)"""
# The text of the element that describes the live drawing for screen readers
DESCRIPTION = """figure => document.getElementById(document.querySelector(
    `${figure} .ix-live-plot`).getAttribute('aria-describedby')).textContent"""
# Whether the figure is in the enlarged view's dialog
IN_DIALOG = 'figure => Boolean(figure.closest("dialog"))'
# Whether the keyboard's place is inside the figure
FOCUS_IN_FIGURE = 'figure => document.querySelector(figure).contains(document.activeElement)'
# Call a function of diffdrive.js, the module that motion-lab.js imports from its own folder
MODEL_CALL = """async ([name, parameters]) => {
  const figureScript = document.querySelector('script[src$="/motion-lab.js"]');
  const model = await import(new URL('diffdrive.js', figureScript.src));
  return model[name](...parameters);
}"""
PHONE = {'width': 390, 'height': 844, 'is_mobile': True, 'has_touch': True}
SMALL_PHONE = {**PHONE, 'width': 320, 'height': 700}
FULL_HD = {'width': 1920, 'height': 1080}
# The media that emulate_media takes: the screen and paper
Media = Literal['screen', 'print']
PRINT: Media = 'print'
BOTH_MEDIA: Tuple[Media, ...] = ('screen', PRINT)


class ShownMotionRow(NamedTuple):
    """A row of the motion table, as its cells show it: the view and its three values."""

    view: str
    forward: str
    left: str
    turned: str


class ShownCheckRow(NamedTuple):
    """A row of the check table, as its cells show it."""

    name: str
    allowed: str
    value: str
    result: str


class ShownValue(NamedTuple):
    """A table cell's value such as '−0.020 m': its number and its unit."""

    number: float
    unit: str


class ShownDrive(NamedTuple):
    """An example's drive at its end in the page, and the same drive computed by the script."""

    page: Page
    example: figures.Example
    kind: figures.CommandKind
    drive: figures.Drive

    @property
    def name(self) -> str:
        """Return the example's key and the drive's kind, for an assertion's message."""
        return f'{self.example.key}, {self.kind.value}'


def table_cells(page: Page, selector: str) -> List[List[str]]:
    """Return the rows of a table body as lists of their cells' texts."""
    cells: List[List[str]] = page.eval_on_selector_all(
        selector, 'rows => rows.map(row => [...row.children].map(cell => cell.innerText.trim()))'
    )
    return cells


def shown_motion_rows(page: Page) -> List[ShownMotionRow]:
    """Return the motion table's rows."""
    return [ShownMotionRow(*cells) for cells in table_cells(page, f'{FIGURE} .ix-motion-rows tr')]


def shown_check_rows(page: Page) -> List[ShownCheckRow]:
    """Return the check table's rows."""
    return [ShownCheckRow(*cells) for cells in table_cells(page, f'{FIGURE} .ix-check-rows tr')]


def parse_shown_value(text: str) -> ShownValue:
    """Return the number and the unit of a cell such as '−0.020 m'."""
    number, unit = text.split()
    return ShownValue(float(number.replace('−', '-')), unit)


def html_text(markup: str) -> str:
    """Return the text that a reader sees of the script's HTML, with single spaces."""
    without_tags = re.sub(r'<[^>]+>', '', markup)
    return ' '.join(html.unescape(without_tags).split())


def top_of(page: Page, selector: str) -> float:
    """Return the top of an element in the page, in px from the page's top."""
    top: float = page.eval_on_selector(
        selector, 'element => element.getBoundingClientRect().top + window.scrollY'
    )
    return top


class ShapeKey(NamedTuple):
    """What groups a drawing's shapes: the view of their robot and their kind of SVG element."""

    view: str
    kind: str


class MarkerCenters(NamedTuple):
    """The centers (px) of the odometry's ring and of the true pose's dot in the live drawing."""

    odometry: figures.PixelPoint
    model: figures.PixelPoint


def shapes_by_kind(page: Page, drawing: str) -> Dict[ShapeKey, List[List[float]]]:
    """Return a drawing's shapes grouped by their view and kind, each as its coordinates."""
    groups: Dict[ShapeKey, List[List[float]]] = {}
    for shape in page.eval_on_selector(f'{FIGURE} {drawing}', SHAPES):
        groups.setdefault(ShapeKey(shape['view'], shape['kind']), []).append(shape['coordinates'])
    return groups


def marker_centers(page: Page) -> MarkerCenters:
    """Return where the live drawing shows the odometry's and the true pose's base_footprint."""
    centers = page.evaluate(MARKER_CENTERS, FIGURE)
    return MarkerCenters(
        odometry=figures.PixelPoint(centers['odometry']['x'], centers['odometry']['y']),
        model=figures.PixelPoint(centers['model']['x'], centers['model']['y']),
    )


def call_model(page: Page, function: str, *arguments: Any) -> Any:
    """Return what a function of diffdrive.js returns in the page for the arguments."""
    return page.evaluate(MODEL_CALL, [function, list(arguments)])


def shown_decimal(value: float) -> str:
    """Return a value as the figure writes it: three decimals, a true minus, no negative zero."""
    text = f'{value:.3f}'.replace('-', '−')
    return text.lstrip('−') if float(f'{value:.3f}') == 0 else text


def expected_description(drive: figures.Drive, kind: figures.CommandKind, sample: int) -> str:
    """Return what the drawing's description must say at a sample of a drive.

    Each view's displacement from the start, with its unit, at the sample's time.
    """
    start, shown = drive.samples[0], drive.samples[sample]
    command = figures.relative_motion(start.command, shown.command)
    odometry = figures.relative_motion(start.odometry, shown.odometry)
    model = figures.relative_motion(start.frame, shown.frame)
    duration = figures.COMMANDS[kind].duration
    time = f'At {duration * sample / figures.DRAWING.samples:.1f} s of {duration:.1f} s, '
    if kind is figures.CommandKind.FORWARD:
        return (
            f'{time}the commanded distance is {shown_decimal(command.forward)} m. The odometry'
            f' reports {shown_decimal(odometry.forward)} m and the robot in the model has'
            f' travelled {shown_decimal(model.forward)} m.'
        )
    distance = figures.motion_quantity_value(figures.Quantity.DISTANCE, model)
    return (
        f'{time}the commanded angle is {shown_decimal(command.turned)} rad. The odometry reports'
        f' {shown_decimal(odometry.turned)} rad and the robot in the model has turned'
        f' {shown_decimal(model.turned)} rad. Its base_footprint is {shown_decimal(distance)} m'
        ' from its starting point.'
    )


def expected_marker_centers(
    page: Page, drive: figures.Drive, inputs: figures.ModelInputs, sample: int
) -> MarkerCenters:
    """Return where the live drawing must show the two markers at a sample of a drive.

    The script's scale for the live drawing's width maps the sample's poses to pixels.
    """
    width = page.eval_on_selector(f'{FIGURE} .ix-live-plot', 'svg => svg.viewBox.baseVal.width')
    scale = figures.drawing_scale(width, drive.samples, figures.outline_reach_metres(inputs))
    shown = drive.samples[sample]
    return MarkerCenters(
        odometry=scale.to_pixels(shown.odometry.position),
        model=scale.to_pixels(shown.frame.position),
    )


def open_settings(page: Page) -> None:
    """Open the block of values, as a reader who clicks its summary."""
    page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')


def set_slider(page: Page, slider: figures.Slider, value: int) -> None:
    """Set a slider to a value, as a reader who drags it there."""
    page.fill(f'#{slider.element_id}', str(value))
    page.dispatch_event(f'#{slider.element_id}', 'input')


def set_time(page: Page, steps: int) -> None:
    """Move the time slider to a step of its TIME_SLIDER_STEPS."""
    page.fill(f'{FIGURE} .ix-scrub', str(steps))
    page.dispatch_event(f'{FIGURE} .ix-scrub', 'input')


def slider_for(setting: str) -> figures.Slider:
    """Return the figure's slider of a setting."""
    return next(slider for slider in figures.CONFIGURATION_SLIDERS + figures.WHAT_IF_SLIDERS
                if slider.setting == setting)


class SimulationTestPageCase(unittest.TestCase):
    """A test case that serves the built site and opens the Simulation Test page in Chromium."""

    base: str
    browser: Browser

    @classmethod
    def setUpClass(cls) -> None:
        # The class's cleanup closes what started, also when Chromium fails to start
        resources = contextlib.ExitStack()
        cls.addClassCleanup(resources.close)
        cls.base = resources.enter_context(served_site())
        cls.browser = resources.enter_context(chromium())

    def open_simulation_test_page(self, **options: Any) -> OpenedPage:
        """Open the page in a new browser context, which the test's cleanup closes."""
        opened = open_page(self.browser, self.base, PAGE, **options)
        self.addCleanup(opened.context.close)
        return opened

    def open_live_figure(self, **options: Any) -> OpenedPage:
        """Open the page and wait until the figure has started."""
        opened = self.open_simulation_test_page(**options)
        opened.page.wait_for_selector(f'{FIGURE}.ix-is-live', timeout=START_TIMEOUT_MS)
        return opened


class TestMotionLabPage(SimulationTestPageCase):
    """The figure in the page: it starts, shows the script's numbers, and falls back."""

    def shown_example_drives(self) -> Iterator[ShownDrive]:
        """Open the figure once and show each example's two drives at their end, in turn.

        With reduced motion, the example's button shows its drive's end and its answer at once.
        """
        page = self.open_live_figure(reduced_motion='reduce').page
        for index, example in enumerate(figures.EXAMPLES):
            if index:
                page.click(f'{FIGURE} .ix-next')
            for kind in (example.command, figures.other_command(example.command)):
                page.click(f'{FIGURE} [data-command="{kind.value}"]')
                page.click(f'{FIGURE} [data-example="{example.key}"]')
                yield ShownDrive(page, example, kind, figures.example_drive(example, kind))

    def assert_markers_at(self, page: Page, expected: MarkerCenters, message: str) -> None:
        """Assert that the live drawing shows the two markers of base_footprint at their places."""
        shown = marker_centers(page)
        for marker, place in ((shown.odometry, expected.odometry), (shown.model, expected.model)):
            self.assertAlmostEqual(marker.x, place.x, delta=DRAWN_ROUNDING, msg=message)
            self.assertAlmostEqual(marker.y, place.y, delta=DRAWN_ROUNDING, msg=message)

    def assert_drawing_shows_sample(
        self, page: Page, drive: figures.Drive, kind: figures.CommandKind, sample: int
    ) -> None:
        """Assert the time under the drawing and the drawing's description at a sample."""
        duration = figures.COMMANDS[kind].duration
        elapsed = duration * sample / figures.DRAWING.samples
        time = (f't = {duration:.1f} s, the end' if sample == LAST_SAMPLE
                else f't = {elapsed:.1f} s of {duration:.1f} s')
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-time'), time)
        self.assertEqual(page.evaluate(DESCRIPTION, FIGURE),
                         expected_description(drive, kind, sample))

    def assert_shown_value(
        self, text: str, value: float, quantity: figures.Quantity, message: str
    ) -> None:
        """Assert that a cell shows a value rounded to the table's decimals, with its unit."""
        shown = parse_shown_value(text)
        self.assertAlmostEqual(shown.number, value, delta=SHOWN_ROUNDING, msg=message)
        self.assertEqual(shown.unit, figures.UNITS[quantity], message)

    def assert_motion_table_shows(self, page: Page, drive: figures.Drive, message: str) -> None:
        """Assert that the motion table shows a drive's three views, each with its motion."""
        names = {entry.view: entry.text for entry in figures.LEGEND}
        expected = (
            (names[figures.DrawingView.COMMAND], drive.command),
            (names[figures.DrawingView.ODOMETRY], drive.odometry),
            (names[figures.DrawingView.MODEL], drive.model),
        )
        rows = shown_motion_rows(page)
        self.assertEqual([row.view for row in rows], [name for name, _ in expected], message)
        for row, (name, motion) in zip(rows, expected, strict=True):
            self.assert_shown_value(row.forward, motion.forward, figures.Quantity.FORWARD,
                                    f'{message}, {name}, forward')
            self.assert_shown_value(row.left, motion.left, figures.Quantity.LEFT,
                                    f'{message}, {name}, left')
            self.assert_shown_value(row.turned, motion.turned, figures.Quantity.TURNED,
                                    f'{message}, {name}, turned')

    def assert_check_table_shows(
        self, page: Page, kind: figures.CommandKind, drive: figures.Drive, message: str
    ) -> None:
        """Assert that the check table shows each check of a drive whole, in the test's order.

        A row has the check's name, its allowed range, the drive's value with its unit, and the
        result that the script predicts for the value.
        """
        rows = shown_check_rows(page)
        expected = figures.check_rows(kind)
        self.assertEqual([row.name for row in rows], [row.name for row in expected], message)
        for shown, row in zip(rows, expected, strict=True):
            value = figures.row_value(row, drive)
            self.assertEqual(shown.allowed, row.allowed, f'{message}, {row.name}')
            self.assert_shown_value(shown.value, value, row.check.quantity,
                                    f'{message}, {row.name}')
            result = PASSES if figures.passes(row.check, value) else FAILS
            self.assertEqual(shown.result, result, f'{message}, {row.name}')

    def test_figure_starts_without_page_errors(self) -> None:
        log = self.open_live_figure().log
        self.assertEqual(log.errors, [])

    def test_figure_requests_no_host_beyond_the_sites_own(self) -> None:
        log = self.open_live_figure().log
        self.assertLessEqual(set(log.other_hosts), SITE_HOSTS)

    def test_live_drawing_replaces_the_static_one_once_the_figure_started(self) -> None:
        page = self.open_live_figure().page
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-static'))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-live-plot'))

    def test_live_drawing_at_the_static_width_matches_the_scripts_static_drawing(self) -> None:
        # The static drawing shows the first example's changed drive at its end. With reduced
        # motion, "Show what happens" shows the same at once
        page = self.open_live_figure(reduced_motion='reduce').page
        page.click(f'{FIGURE} [data-example="{figures.EXAMPLES[0].key}"]')
        width = figures.DRAWING.static_width_px
        # The live drawing is as wide as the plot's inside, without its border
        page.eval_on_selector(
            f'{FIGURE} .ix-plot',
            '(plot, width) => {'
            ' plot.style.width = `${width + plot.offsetWidth - plot.clientWidth}px`; }',
            width,
        )
        page.wait_for_function(
            '([figure, width]) => document.querySelector(`${figure} .ix-live-plot`)'
            '.viewBox.baseVal.width === width',
            arg=[FIGURE, width],
        )
        static = shapes_by_kind(page, '.ix-static')
        live = shapes_by_kind(page, '.ix-live-plot')
        self.assertTrue(static, 'the static drawing has no robot')
        self.assertEqual(sorted(live), sorted(static))
        for key, static_shapes in static.items():
            for live_shape, static_shape in zip(live[key], static_shapes, strict=True):
                for live_value, static_value in zip(live_shape, static_shape, strict=True):
                    self.assertAlmostEqual(live_value, static_value, delta=DRAWN_ROUNDING,
                                           msg=str(key))

    def test_every_example_opens_at_the_start_of_its_drive_with_its_results_waiting(
        self,
    ) -> None:
        page = self.open_live_figure().page
        for index, example in enumerate(figures.EXAMPLES):
            if index:
                page.click(f'{FIGURE} .ix-next')
            self.assertEqual(page.inner_text(f'{FIGURE} .ix-step-count'),
                             f'Example {index + 1} of {len(figures.EXAMPLES)}')
            self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), '0', example.key)
            for waiting in ('.ix-shown-answer', '.ix-summary', '.ix-checks'):
                self.assertFalse(page.is_visible(f'{FIGURE} {waiting}'),
                                 f'{example.key}, {waiting}')

    def test_the_time_slider_shows_the_examples_drive_before_its_answer(self) -> None:
        # The first example's configured wheel radius is too big: the odometry and the true pose
        # start together, and halfway through the drive the odometry is ahead
        example = figures.EXAMPLES[0]
        drive = figures.example_drive(example)
        inputs = figures.model_inputs(REMO, example.settings)
        page = self.open_live_figure().page
        at_start = expected_marker_centers(page, drive, inputs, FIRST_SAMPLE)
        self.assertAlmostEqual(at_start.odometry.x, at_start.model.x, delta=DRAWN_ROUNDING)
        self.assert_markers_at(page, at_start, 'at the start')
        set_time(page, figures.TIME_SLIDER_STEPS // 2)
        halfway = expected_marker_centers(page, drive, inputs, MIDDLE_SAMPLE)
        self.assertGreater(halfway.odometry.x, halfway.model.x + 1)
        self.assert_markers_at(page, halfway, 'halfway')
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-shown-answer'))

    def test_the_live_drawing_makes_room_for_a_wider_configured_outline(self) -> None:
        # The odometry's outline is drawn with the configured separation. The drawing's scale
        # leaves room for it, so the markers lie where the script's scale puts them
        separation = slider_for('separation_percent')
        settings = figures.DEFAULT_SETTINGS._replace(separation_percent=separation.highest)
        kind = figures.EXAMPLES[0].command
        drive = figures.simulate(REMO, figures.COMMANDS[kind], settings)
        inputs = figures.model_inputs(REMO, settings)
        page = self.open_live_figure().page
        open_settings(page)
        page.click(f'{FIGURE} .ix-settings .ix-reset')
        set_slider(page, separation, separation.highest)
        set_time(page, figures.TIME_SLIDER_STEPS)
        expected = expected_marker_centers(page, drive, inputs, LAST_SAMPLE)
        self.assert_markers_at(page, expected, 'at the end')

    def test_a_changed_value_keeps_the_time_and_the_results_waiting(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure().page
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.fill(f'#{slider.element_id}', str(slider.highest))
        page.dispatch_event(f'#{slider.element_id}', 'input')
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), '0')
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-summary'))

    def test_shown_results_follow_a_changed_value(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        example = figures.EXAMPLES[0]
        kind = example.command
        changed = figures.simulate(
            REMO, figures.COMMANDS[kind], example.settings._replace(radius_percent=slider.lowest)
        )
        rows = figures.check_rows(kind)
        passing = len(rows) - len(figures.failing_checks(kind, changed))
        page = self.open_live_figure(reduced_motion='reduce').page
        page.click(f'{FIGURE} .ix-play')
        open_settings(page)
        set_slider(page, slider, slider.lowest)
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), str(figures.TIME_SLIDER_STEPS))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-summary'))
        summary = page.inner_text(f'{FIGURE} .ix-summary')
        self.assertIn(f'In the model, the robot drives {shown_decimal(changed.model.forward)} m.',
                      summary)
        self.assertIn(f'{passing} of {len(rows)} checks pass.', summary)

    def test_another_value_hides_the_answer_and_show_what_happens_brings_it_back(self) -> None:
        example = figures.EXAMPLES[0]
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure(reduced_motion='reduce').page
        page.click(f'{FIGURE} [data-example="{example.key}"]')
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.fill(f'#{slider.element_id}', str(slider.lowest))
        page.dispatch_event(f'#{slider.element_id}', 'input')
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-shown-answer'))
        page.click(f'{FIGURE} [data-example="{example.key}"]')
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-shown-answer'))

    def test_changed_values_keep_the_drawing_and_the_controls_in_place(self) -> None:
        radius = figures.CONFIGURATION_SLIDERS[0]
        separation = figures.CONFIGURATION_SLIDERS[1]
        offset = next(s for s in figures.WHAT_IF_SLIDERS if s.setting == 'offset_mm')
        example = figures.EXAMPLES[0]
        for size in ({'width': 1280, 'height': 900}, PHONE):
            with self.subTest(width=size['width']):
                page = self.open_live_figure(reduced_motion='reduce', **size).page
                page.click(f'{FIGURE} [data-example="{example.key}"]')
                page.eval_on_selector(f'{FIGURE} .ix-settings',
                                      'details => { details.open = true; }')
                places = (f'{FIGURE} .ix-plot', f'#{separation.element_id}')
                before = [top_of(page, place) for place in places]
                for slider, value in ((radius, radius.lowest), (radius, radius.highest),
                                      (offset, offset.highest)):
                    page.fill(f'#{slider.element_id}', str(value))
                    page.dispatch_event(f'#{slider.element_id}', 'input')
                    self.assertEqual([top_of(page, place) for place in places], before,
                                     f'{slider.setting} {value}')

    def test_table_cells_show_their_whole_text(self) -> None:
        # The tables have fixed columns: no cell's text may run into the next one
        for size in ({'width': 1024, 'height': 800}, {'width': 768, 'height': 900},
                     {'width': 660, 'height': 900}, PHONE):
            with self.subTest(width=size['width']):
                page = self.open_live_figure(reduced_motion='reduce', **size).page
                page.click(f'{FIGURE} [data-example="{figures.EXAMPLES[0].key}"]')
                self.assertEqual(page.evaluate(CUT_CELLS, FIGURE), [])

    def test_reset_next_to_the_examples_sets_the_robots_own_values(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        default = str(getattr(figures.DEFAULT_SETTINGS, slider.setting))
        page = self.open_live_figure().page
        # The first example opens with its configured wheel radius changed
        self.assertNotEqual(page.input_value(f'#{slider.element_id}'), default)
        page.click(f'{FIGURE} .ix-stepper .ix-reset')
        self.assertEqual(page.input_value(f'#{slider.element_id}'), default)

    def test_a_double_tap_sets_one_slider_back_to_the_robots_value(self) -> None:
        slider, other = figures.CONFIGURATION_SLIDERS
        page = self.open_live_figure(**PHONE).page
        open_settings(page)
        set_slider(page, other, other.lowest)
        page.locator(f'#{slider.element_id}').scroll_into_view_if_needed()
        box = page.locator(f'#{slider.element_id}').bounding_box()
        if box is None:
            raise AssertionError('the slider has no box')
        # Tap at a fifth of the slider: the first tap moves the value there, the second resets it
        x, y = box['x'] + box['width'] / 5, box['y'] + box['height'] / 2
        page.touchscreen.tap(x, y)
        page.touchscreen.tap(x, y)
        self.assertEqual(page.input_value(f'#{slider.element_id}'),
                         str(getattr(figures.DEFAULT_SETTINGS, slider.setting)))
        self.assertEqual(page.input_value(f'#{other.element_id}'), str(other.lowest))

    def test_a_long_hold_then_a_tap_resets_no_slider(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure(**PHONE).page
        open_settings(page)
        set_slider(page, slider, slider.lowest)
        # A touch held still for 1.5 s, then a tap at the same place
        page.evaluate("""async (selector) => {
            const input = document.querySelector(selector);
            const box = input.getBoundingClientRect();
            const x = box.left + 40, y = box.top + box.height / 2;
            const send = (type) => input.dispatchEvent(new PointerEvent(type,
                {pointerType: 'touch', isPrimary: true, pointerId: 7, bubbles: true,
                 clientX: x, clientY: y}));
            send('pointerdown');
            await new Promise((resolve) => setTimeout(resolve, 1500));
            send('pointerup');
            send('pointerdown');
            send('pointerup');
        }""", f'#{slider.element_id}')
        self.assertEqual(page.input_value(f'#{slider.element_id}'), str(slider.lowest))

    def test_two_quick_drags_or_two_distant_taps_reset_no_slider(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure(**PHONE).page
        open_settings(page)
        set_slider(page, slider, slider.lowest)
        # Dispatch two touch sequences in quick succession
        touches = """([selector, presses]) => {
            const input = document.querySelector(selector);
            const box = input.getBoundingClientRect(), y = box.top + box.height / 2;
            const send = (type, x, timeStamp) => input.dispatchEvent(new PointerEvent(type,
                {pointerType: 'touch', isPrimary: true, bubbles: true, clientX: box.left + x,
                 clientY: y}));
            for (const [from, to] of presses) {
                send('pointerdown', from); send('pointermove', to); send('pointerup', to);
            }
        }"""
        # The two drags end close together, the two taps lie far apart
        for name, presses in (('two drags', [[40, 120], [200, 125]]),
                              ('two distant taps', [[40, 40], [200, 200]])):
            page.evaluate(touches, [f'#{slider.element_id}', presses])
            self.assertEqual(page.input_value(f'#{slider.element_id}'), str(slider.lowest), name)

    def test_enlarged_figure_shows_question_drawing_results_and_values_in_one_window(
        self,
    ) -> None:
        page = self.open_live_figure(reduced_motion='reduce', **FULL_HD).page
        page.click(f'{FIGURE} [data-example="{figures.EXAMPLES[0].key}"]')
        page.click(f'{FIGURE} .ix-enlarge')
        parts = ('.ix-current .ix-question', '.ix-plot', '.ix-playbar', '.ix-shown-answer',
                 '.ix-summary', f'#{figures.CONFIGURATION_SLIDERS[0].element_id}',
                 '.ix-check-rows tr:last-child')
        outside = page.evaluate("""([figure, parts]) => parts.filter((part) => {
            const element = document.querySelector(`${figure} ${part}`);
            const box = element.getBoundingClientRect();
            return box.width === 0 || box.height === 0 || !element.checkVisibility()
                || box.top < 0 || box.left < 0
                || box.bottom > innerHeight || box.right > innerWidth;
        })""", [FIGURE, list(parts)])
        self.assertEqual(outside, [])

    def test_enlarged_figure_cuts_no_table_cell(self) -> None:
        # Enlarged on a 1440 px screen, the tables sit in the narrower right-hand column
        page = self.open_live_figure(reduced_motion='reduce', width=1440, height=900).page
        page.click(f'{FIGURE} [data-example="{figures.EXAMPLES[0].key}"]')
        page.click(f'{FIGURE} .ix-enlarge')
        self.assertEqual(page.evaluate(CUT_CELLS, FIGURE), [])

    def test_escape_brings_the_enlarged_figure_back_to_its_place(self) -> None:
        page = self.open_live_figure(**FULL_HD).page
        before = top_of(page, FIGURE)
        page.click(f'{FIGURE} .ix-enlarge')
        self.assertTrue(page.eval_on_selector(FIGURE, IN_DIALOG))
        page.keyboard.press('Escape')
        self.assertFalse(page.eval_on_selector(FIGURE, IN_DIALOG))
        self.assertEqual(top_of(page, FIGURE), before)
        self.assertTrue(page.eval_on_selector(
            f'{FIGURE} .ix-enlarge', 'button => document.activeElement === button'))

    def test_the_keyboard_stays_in_the_enlarged_figure(self) -> None:
        page = self.open_live_figure(**FULL_HD).page
        page.click(f'{FIGURE} .ix-enlarge')
        for key in ('Tab', 'Shift+Tab'):
            for press in range(40):
                page.keyboard.press(key)
                inside = page.evaluate(
                    'figure => !document.activeElement || document.activeElement === document.body'
                    ' || document.querySelector(figure).contains(document.activeElement)', FIGURE)
                self.assertTrue(inside, f'{key} {press + 1}')

    def test_a_narrower_window_brings_the_figure_back_with_its_values_closed(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure(**FULL_HD).page
        page.click(f'{FIGURE} .ix-enlarge')
        page.focus(f'#{slider.element_id}')
        page.set_viewport_size({'width': 800, 'height': 900})
        page.wait_for_function('figure => !document.querySelector(figure).closest("dialog")',
                               arg=FIGURE)
        self.assertFalse(
            page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => details.open'))
        focused = page.evaluate('() => document.activeElement.checkVisibility()'
                                ' && document.activeElement.tagName')
        self.assertEqual(focused, 'SUMMARY')

    def test_printing_brings_an_enlarged_figure_back_to_the_page(self) -> None:
        page = self.open_live_figure(**FULL_HD).page
        page.click(f'{FIGURE} .ix-enlarge')
        page.evaluate('() => dispatchEvent(new Event("beforeprint"))')
        self.assertFalse(page.eval_on_selector(FIGURE, IN_DIALOG))

    def test_enlarge_is_hidden_on_a_phone(self) -> None:
        page = self.open_live_figure(**PHONE).page
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-enlarge'))

    def test_each_drive_shows_its_example_answer(self) -> None:
        for shown in self.shown_example_drives():
            prompt = figures.example_prompt(shown.example, shown.kind)
            values = figures.example_text_values(shown.example, shown.kind)
            answer = html_text(prompt.answer.format(**values))
            self.assertTrue(answer, shown.name)
            self.assertEqual(html_text(shown.page.inner_text(f'{FIGURE} .ix-shown-answer')),
                             f'What happens: {answer}', shown.name)

    def test_motion_table_matches_the_three_predicted_views(self) -> None:
        for shown in self.shown_example_drives():
            self.assert_motion_table_shows(shown.page, shown.drive, shown.name)

    def test_check_table_shows_each_predicted_check(self) -> None:
        for shown in self.shown_example_drives():
            self.assert_check_table_shows(shown.page, shown.kind, shown.drive, shown.name)

    def test_the_other_drive_asks_its_own_question_from_its_start(self) -> None:
        example = figures.EXAMPLES[0]
        other = figures.other_command(example.command)
        item = f'{FIGURE} [data-example-item="{example.key}"]'
        page = self.open_live_figure(reduced_motion='reduce').page
        page.click(f'{FIGURE} .ix-play')
        page.click(f'{FIGURE} [data-command="{other.value}"]')
        shown = page.eval_on_selector_all(
            f'{item} .ix-question',
            'questions => questions.filter((question) => getComputedStyle(question).visibility'
            ' === "visible").map((question) => question.dataset.drive)')
        self.assertEqual(shown, [other.value])
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), '0')
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-summary'))

    def test_values_other_than_the_examples_show_whole_under_the_sliders(self) -> None:
        example = figures.EXAMPLES[0]
        radius = figures.CONFIGURATION_SLIDERS[0]
        offset = slider_for('offset_mm')
        changes = f'{FIGURE} .ix-changes'
        for size in (SMALL_PHONE, PHONE):
            with self.subTest(width=size['width']):
                page = self.open_live_figure(**size).page
                open_settings(page)
                # With the example's own values, neither the list nor the note shows
                self.assertFalse(page.is_visible(changes))
                self.assertFalse(page.is_visible(f'{FIGURE} .ix-differs'))
                set_slider(page, radius, 98)
                set_slider(page, offset, offset.highest)
                page.select_option(f'{FIGURE} .ix-robot-select', 'diffbot')
                set_slider(page, radius, 98)
                self.assertTrue(page.is_visible(changes))
                # Under the play bar, a note in the answer's place points to the list
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-differs'))
                self.assertFalse(page.is_visible(f'{FIGURE} .ix-hint'))
                text = page.inner_text(changes)
                own = getattr(example.settings, radius.setting)
                self.assertIn(f'98 {radius.unit} (the example: {own} {radius.unit})', text)
                self.assertIn('DiffBot (the example: Remo)', text)
                # The whole text shows: it wraps inside the figure, nothing is cut
                cut = page.eval_on_selector(changes, 'line => line.scrollWidth > line.clientWidth'
                                            ' || line.scrollHeight > line.clientHeight + 1')
                self.assertFalse(cut)
                self.assertEqual(page.inner_text(f'{FIGURE} .ix-current .ix-show'),
                                 'Play the example')
                # Playing the example sets its robot and values back: both go again
                page.click(f'{FIGURE} .ix-current .ix-show')
                self.assertFalse(page.is_visible(changes))
                self.assertFalse(page.is_visible(f'{FIGURE} .ix-differs'))

    def test_show_what_happens_plays_the_changed_drive_then_shows_the_answer(self) -> None:
        page = self.open_live_figure().page
        example = figures.EXAMPLES[0]
        scrub = f'{FIGURE} .ix-scrub'
        page.click(f'{FIGURE} [data-example="{example.key}"]')
        # Right after the click, the drive plays from its start, and the answer and the results
        # of the drive's end wait
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-play'), 'Pause')
        self.assertLess(int(page.input_value(scrub)), figures.TIME_SLIDER_STEPS)
        for waiting in ('.ix-shown-answer', '.ix-summary', '.ix-checks'):
            self.assertFalse(page.is_visible(f'{FIGURE} {waiting}'), waiting)
        drive_ms = figures.COMMANDS[example.command].duration * MILLISECONDS_PER_SECOND
        page.wait_for_selector(f'{FIGURE} .ix-shown-answer', state='visible',
                               timeout=drive_ms + START_TIMEOUT_MS)
        self.assertEqual(page.input_value(scrub), str(figures.TIME_SLIDER_STEPS))
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-play'), 'Play')
        for shown in ('.ix-summary', '.ix-checks'):
            self.assertTrue(page.is_visible(f'{FIGURE} {shown}'), shown)

    def test_play_button_keeps_its_height_when_it_says_pause(self) -> None:
        page = self.open_live_figure().page
        height = 'button => button.getBoundingClientRect().height'
        playing = page.eval_on_selector(f'{FIGURE} .ix-play', height)
        page.click(f'{FIGURE} .ix-play')
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-play'), 'Pause')
        self.assertEqual(page.eval_on_selector(f'{FIGURE} .ix-play', height), playing)

    def test_a_shown_answer_stays_when_the_drive_is_scrubbed_back(self) -> None:
        page = self.open_live_figure(reduced_motion='reduce').page
        scrub = f'{FIGURE} .ix-scrub'
        page.click(f'{FIGURE} [data-example="{figures.EXAMPLES[0].key}"]')
        page.fill(scrub, '0')
        page.dispatch_event(scrub, 'input')
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-shown-answer'))

    def test_stepper_question_and_command_sit_above_the_drawing(self) -> None:
        page = self.open_live_figure().page
        drawing = top_of(page, f'{FIGURE} .ix-plot')
        for part in ('.ix-stepper', '.ix-current .ix-question', '[data-command="turn"]'):
            self.assertTrue(page.is_visible(f'{FIGURE} {part}'), part)
            bottom = page.eval_on_selector(
                f'{FIGURE} {part}',
                'element => element.getBoundingClientRect().bottom + window.scrollY',
            )
            self.assertLessEqual(bottom, drawing, part)

    def test_drawing_stays_in_place_from_one_example_to_the_next(self) -> None:
        page = self.open_live_figure(reduced_motion='reduce', **PHONE).page
        first = top_of(page, f'{FIGURE} .ix-plot')
        for index, example in enumerate(figures.EXAMPLES):
            if index:
                page.click(f'{FIGURE} .ix-next')
                self.assertEqual(top_of(page, f'{FIGURE} .ix-plot'), first, example.key)
            page.click(f'{FIGURE} [data-example="{example.key}"]')
            self.assertEqual(top_of(page, f'{FIGURE} .ix-plot'), first, example.key)
            # The other drive's question shares the place
            other = figures.other_command(example.command)
            page.click(f'{FIGURE} [data-command="{other.value}"]')
            self.assertEqual(top_of(page, f'{FIGURE} .ix-plot'), first, f'{example.key}, other')
            page.click(f'{FIGURE} [data-command="{example.command.value}"]')

    def test_a_value_on_a_tolerance_boundary_passes(self) -> None:
        turned = next(
            check for check in figures.CHECKS[figures.CommandKind.TURN].each_view
            if check.quantity is figures.Quantity.TURNED
        )
        boundary = turned.expected - turned.tolerance
        # The wheels reach the share of their speed that ends the turn exactly on the boundary
        tracking = round(boundary / figures.COMMANDS[figures.CommandKind.TURN].angle * 100)
        slider = slider_for('tracking_percent')
        # With reduced motion, Play shows the drive's end and its results at once
        page = self.open_live_figure(reduced_motion='reduce').page
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.click(f'{FIGURE} .ix-reset')
        page.click(f'{FIGURE} [data-command="turn"]')
        page.fill(f'#{slider.element_id}', str(tracking))
        page.dispatch_event(f'#{slider.element_id}', 'input')
        page.click(f'{FIGURE} .ix-play')
        row = next(row for row in shown_check_rows(page) if row.name == 'turned, true pose')
        self.assertEqual(row.value, f'{boundary:.3f} rad')
        self.assertEqual(row.result, PASSES)

    def test_page_has_no_sideways_scrolling_on_a_320_px_screen(self) -> None:
        page = self.open_live_figure(**SMALL_PHONE).page
        self.assertEqual(
            page.evaluate(
                'document.documentElement.scrollWidth - document.documentElement.clientWidth'
            ),
            0,
        )

    def test_figure_keeps_its_parts_inside_its_width_on_a_320_px_screen(self) -> None:
        page = self.open_live_figure(**SMALL_PHONE).page
        outside = page.evaluate(f"""(() => {{
            const figure = document.querySelector('{FIGURE}').getBoundingClientRect();
            return [...document.querySelectorAll('{FIGURE} *')].filter((element) => {{
                const box = element.getBoundingClientRect();
                const outside = box.right > figure.right + 1 || box.left < figure.left - 1;
                return box.width > 0 && outside
                    && !element.closest('svg') && !element.closest('.ix-announce');
            }}).map((element) => element.className || element.tagName);
        }})()""")
        self.assertEqual(outside, [])

    def test_static_version_stays_without_javascript_also_on_paper(self) -> None:
        page = self.open_simulation_test_page(javascript=False).page
        shown = 'parts => parts.filter(part => part.offsetParent).length'
        for media in BOTH_MEDIA:
            with self.subTest(media=media):
                page.emulate_media(media=media)
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'))
                self.assertEqual(page.eval_on_selector_all(f'{FIGURE} .ix-answer', shown),
                                 len(figures.EXAMPLES))
                self.assertEqual(page.eval_on_selector_all(f'{FIGURE} [data-live]', shown), 0)
                # The introduction asks for no control that the static version lacks
                self.assertNotIn('Press Play', page.inner_text(f'{FIGURE} .ix-steps'))

    def test_static_version_stays_when_a_script_fails_to_load(self) -> None:
        shown = 'parts => parts.filter(part => part.offsetParent).length'
        for missing in ('motion-lab.js', 'diffdrive.js'):
            with self.subTest(missing=missing):
                opened = self.open_simulation_test_page(blocked=(missing,))
                page = opened.page
                # The page has asked for the file, and its modules have settled
                page.wait_for_load_state('networkidle')
                self.assertTrue(any(path.endswith(missing) for path in opened.log.blocked))
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'))
                self.assertEqual(page.eval_on_selector_all(f'{FIGURE} .ix-answer', shown),
                                 len(figures.EXAMPLES))

    def test_play_jumps_to_the_end_without_animation_with_reduced_motion(self) -> None:
        example = figures.EXAMPLES[0]
        drive = figures.example_drive(example)
        page = self.open_live_figure(reduced_motion='reduce').page
        scrub = f'{FIGURE} .ix-scrub'
        set_time(page, figures.TIME_SLIDER_STEPS // 5)
        page.click(f'{FIGURE} .ix-play')
        self.assertEqual(page.input_value(scrub), str(figures.TIME_SLIDER_STEPS))
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-play'), 'Play')
        self.assert_drawing_shows_sample(page, drive, example.command, LAST_SAMPLE)

    def test_play_at_the_drives_end_shows_its_start_with_reduced_motion(self) -> None:
        # An example that was played opens at its drive's end. Play then has to show something:
        # the start, and the next Play the end again. The drawing follows the time slider
        page = self.open_live_figure(reduced_motion='reduce').page
        scrub = f'{FIGURE} .ix-scrub'
        last = len(figures.EXAMPLES) - 1
        for _ in range(last):
            page.click(f'{FIGURE} .ix-play')
            page.click(f'{FIGURE} .ix-next')
        page.click(f'{FIGURE} .ix-play')
        page.click(f'{FIGURE} .ix-previous')
        example = figures.EXAMPLES[last - 1]
        drive = figures.example_drive(example)
        self.assertEqual(page.input_value(scrub), str(figures.TIME_SLIDER_STEPS))
        self.assert_drawing_shows_sample(page, drive, example.command, LAST_SAMPLE)
        page.click(f'{FIGURE} .ix-play')
        self.assertEqual(page.input_value(scrub), '0')
        self.assert_drawing_shows_sample(page, drive, example.command, FIRST_SAMPLE)
        page.click(f'{FIGURE} .ix-play')
        self.assertEqual(page.input_value(scrub), str(figures.TIME_SLIDER_STEPS))
        self.assert_drawing_shows_sample(page, drive, example.command, LAST_SAMPLE)

    def test_another_drives_end_never_answers_the_examples_question(self) -> None:
        # Reach the end with other values, go back to the middle, set the example's values again:
        # the results show, but the question waits for its own drive's end. On both drives
        example = figures.EXAMPLES[0]
        slider = slider_for(next(iter(figures.changed_settings(example.settings))))
        own = getattr(example.settings, slider.setting)
        for kind in (example.command, figures.other_command(example.command)):
            with self.subTest(drive=kind.value):
                page = self.open_live_figure().page
                page.click(f'{FIGURE} [data-command="{kind.value}"]')
                open_settings(page)
                set_slider(page, slider, getattr(figures.DEFAULT_SETTINGS, slider.setting))
                set_time(page, figures.TIME_SLIDER_STEPS)
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-summary'))
                set_time(page, figures.TIME_SLIDER_STEPS // 2)
                set_slider(page, slider, own)
                self.assertFalse(page.is_visible(f'{FIGURE} .ix-shown-answer'))
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-hint'))
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-summary'))

    def test_the_examples_own_drive_dragged_to_its_end_answers_and_the_answer_stays(self) -> None:
        page = self.open_live_figure().page
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-hint'))
        set_time(page, figures.TIME_SLIDER_STEPS)
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-shown-answer'))
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-hint'))
        set_time(page, 0)
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-shown-answer'))

    def test_the_drawings_description_tells_what_it_shows_at_the_sliders_time(self) -> None:
        # For a screen reader, the drawing describes the time that it shows: each view's
        # displacement so far, on the straight drive and on the turn
        example = figures.EXAMPLES[0]
        steps = ((0, FIRST_SAMPLE), (figures.TIME_SLIDER_STEPS // 2, MIDDLE_SAMPLE),
                 (figures.TIME_SLIDER_STEPS, LAST_SAMPLE))
        page = self.open_live_figure().page
        for kind in (example.command, figures.other_command(example.command)):
            drive = figures.example_drive(example, kind)
            page.click(f'{FIGURE} [data-command="{kind.value}"]')
            for step, sample in steps:
                with self.subTest(drive=kind.value, step=step):
                    set_time(page, step)
                    self.assert_drawing_shows_sample(page, drive, kind, sample)

    def test_the_radius_drives_description_halfway_has_the_values_worked_out_by_hand(
        self,
    ) -> None:
        # Halfway, the command and the odometry are at 0.2 m/s x 1.5 s = 0.300 m. The radius is
        # 10 % too big, so the robot is at 0.300 m / 1.1 = 0.273 m
        page = self.open_live_figure().page
        set_time(page, figures.TIME_SLIDER_STEPS // 2)
        self.assertEqual(
            page.evaluate(DESCRIPTION, FIGURE),
            'At 1.5 s of 3.0 s, the commanded distance is 0.300 m. The odometry reports 0.300 m'
            ' and the robot in the model has travelled 0.273 m.',
        )

    def test_the_drawings_description_at_the_start_keeps_the_ends_numbers_back(self) -> None:
        example = figures.EXAMPLES[0]
        end = figures.example_drive(example)
        at_start = self.open_live_figure().page.evaluate(DESCRIPTION, FIGURE)
        for value in (end.odometry.forward, end.model.forward):
            self.assertNotIn(shown_decimal(value), at_start)

    def test_dragging_a_slider_through_the_examples_value_keeps_it_in_place(self) -> None:
        # With many values changed, the list of changed values is long. The wheel-speed slider
        # stays where it is from before its first change, through the example's value and back
        separation = figures.EXAMPLES[1]
        changes = (('radius_percent', 98), ('separation_percent', 80), ('offset_mm', 60),
                   ('contact_percent', 100))
        tracking = slider_for('tracking_percent')
        own = getattr(separation.settings, tracking.setting)
        for size in (SMALL_PHONE, PHONE):
            with self.subTest(width=size['width']):
                page = self.open_live_figure(**size).page
                page.click(f'{FIGURE} .ix-next')
                page.click(f'{FIGURE} [data-command="{figures.CommandKind.FORWARD.value}"]')
                open_settings(page)
                page.select_option(f'{FIGURE} .ix-robot-select', 'diffbot')
                for setting, value in changes:
                    set_slider(page, slider_for(setting), value)
                places = [top_of(page, f'#{tracking.element_id}')]
                for value in (own - 1, own, own + 1):
                    set_slider(page, tracking, value)
                    places.append(top_of(page, f'#{tracking.element_id}'))
                self.assertEqual(places, [places[0]] * 4)

    def test_a_replay_with_reduced_motion_hears_no_summary_of_the_values_before(self) -> None:
        example = figures.EXAMPLES[0]
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure(reduced_motion='reduce').page
        page.click(f'{FIGURE} [data-example="{example.key}"]')
        open_settings(page)
        set_slider(page, slider, slider.lowest)
        # The changed value's summary waits to be heard, and the replay comes right after it, in
        # the same task. No summary follows the replay into the live region
        page.evaluate("""([figure, slider, example]) => {
            document.querySelector(slider).dispatchEvent(new Event('change'));
            document.querySelector(`${figure} [data-example="${example}"]`).click();
        }""", [FIGURE, f'#{slider.element_id}', example.key])
        page.wait_for_timeout(1000)
        # textContent on both sides: the summary's hidden failing names make its innerText differ
        self.assertNotEqual(page.text_content(f'{FIGURE} .ix-announce'),
                            page.text_content(f'{FIGURE} .ix-summary'))
        self.assertTrue(page.eval_on_selector(
            f'{FIGURE} .ix-shown-answer', 'answer => document.activeElement === answer'))

    def test_pressing_the_shown_command_keeps_the_time(self) -> None:
        example = figures.EXAMPLES[0]
        page = self.open_live_figure().page
        set_time(page, figures.TIME_SLIDER_STEPS // 2)
        page.click(f'{FIGURE} [data-command="{example.command.value}"]')
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'),
                         str(figures.TIME_SLIDER_STEPS // 2))

    def test_a_changed_value_mid_drive_redraws_the_same_time(self) -> None:
        # Halfway through the first example's drive, the right radius puts the odometry back on
        # the true pose, at the same time and with the answer still waiting
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure().page
        set_time(page, figures.TIME_SLIDER_STEPS // 2)
        open_settings(page)
        set_slider(page, slider, getattr(figures.DEFAULT_SETTINGS, slider.setting))
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'),
                         str(figures.TIME_SLIDER_STEPS // 2))
        # With the robot's own values, both markers lie on each other, halfway along the drive
        kind = figures.EXAMPLES[0].command
        own_drive = figures.simulate(REMO, figures.COMMANDS[kind], figures.DEFAULT_SETTINGS)
        inputs = figures.model_inputs(REMO, figures.DEFAULT_SETTINGS)
        at_start = expected_marker_centers(page, own_drive, inputs, FIRST_SAMPLE)
        halfway = expected_marker_centers(page, own_drive, inputs, MIDDLE_SAMPLE)
        self.assertGreater(halfway.model.x, at_start.model.x + 1)
        self.assertAlmostEqual(halfway.odometry.x, halfway.model.x, delta=DRAWN_ROUNDING)
        self.assert_markers_at(page, halfway, 'halfway')
        self.assert_drawing_shows_sample(page, own_drive, kind, MIDDLE_SAMPLE)
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-shown-answer'))

    def test_pause_holds_the_time_and_play_goes_on_from_there(self) -> None:
        page = self.open_live_figure().page
        scrub = f'{FIGURE} .ix-scrub'
        page.click(f'{FIGURE} .ix-play')
        page.wait_for_function('scrub => document.querySelector(scrub).value > 100', arg=scrub)
        page.click(f'{FIGURE} .ix-play')
        paused = int(page.input_value(scrub))
        page.wait_for_timeout(300)
        self.assertEqual(int(page.input_value(scrub)), paused)
        page.click(f'{FIGURE} .ix-play')
        page.wait_for_function(
            '([scrub, paused]) => document.querySelector(scrub).value > paused',
            arg=[scrub, paused])
        self.assertGreater(int(page.input_value(scrub)), paused)

    def test_another_robot_and_both_resets_keep_the_time(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        default = str(getattr(figures.DEFAULT_SETTINGS, slider.setting))
        half = str(figures.TIME_SLIDER_STEPS // 2)
        for change in ('robot', '.ix-stepper .ix-reset', '.ix-settings .ix-reset'):
            with self.subTest(change=change):
                page = self.open_live_figure().page
                open_settings(page)
                set_time(page, int(half))
                if change == 'robot':
                    page.select_option(f'{FIGURE} .ix-robot-select', 'diffbot')
                else:
                    page.click(f'{FIGURE} {change}')
                self.assertEqual(page.input_value(f'#{slider.element_id}'), default)
                self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), half)

    def test_an_answered_example_opens_at_its_end_with_its_answer(self) -> None:
        page = self.open_live_figure(reduced_motion='reduce').page
        page.click(f'{FIGURE} [data-example="{figures.EXAMPLES[0].key}"]')
        page.click(f'{FIGURE} .ix-next')
        page.click(f'{FIGURE} .ix-previous')
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), str(figures.TIME_SLIDER_STEPS))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-shown-answer'))

    def test_show_what_happens_moves_the_keyboard_to_pause_or_to_the_answer(self) -> None:
        example = figures.EXAMPLES[0]
        for motion, focused in ((None, '.ix-play'), ('reduce', '.ix-shown-answer')):
            with self.subTest(reduced_motion=motion):
                page = self.open_live_figure(reduced_motion=motion).page
                page.focus(f'{FIGURE} [data-example="{example.key}"]')
                page.keyboard.press('Enter')
                self.assertTrue(page.eval_on_selector(
                    f'{FIGURE} {focused}', 'element => document.activeElement === element'))

    def test_the_keyboard_stays_on_the_stepper_at_the_last_example(self) -> None:
        page = self.open_live_figure().page
        page.focus(f'{FIGURE} .ix-next')
        for _ in range(len(figures.EXAMPLES) - 1):
            page.keyboard.press('Enter')
        self.assertTrue(page.is_disabled(f'{FIGURE} .ix-next'))
        self.assertTrue(page.eval_on_selector(
            f'{FIGURE} .ix-previous', 'button => document.activeElement === button'))

    def test_the_keyboard_opens_the_values_and_moves_a_slider(self) -> None:
        slider = figures.CONFIGURATION_SLIDERS[0]
        page = self.open_live_figure().page
        page.focus(f'{FIGURE} .ix-settings > summary')
        page.keyboard.press('Enter')
        self.assertTrue(page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => details.open'))
        before = int(page.input_value(f'#{slider.element_id}'))
        output = f'{FIGURE} output[for="{slider.element_id}"]'
        shown = page.inner_text(output)
        page.focus(f'#{slider.element_id}')
        page.keyboard.press('ArrowRight')
        self.assertEqual(int(page.input_value(f'#{slider.element_id}')), before + slider.step)
        self.assertNotEqual(page.inner_text(output), shown)
        self.assertTrue(page.evaluate(FOCUS_IN_FIGURE, FIGURE))

    def test_paper_shows_the_static_drawing_with_its_own_notes(self) -> None:
        # Printed after the reader moved on to an example whose drive draws the axle's midpoint
        drawing = figures.static_drawing(figures.EXAMPLES[0])
        page = self.open_live_figure().page
        offset = next(index for index, example in enumerate(figures.EXAMPLES)
                      if example.settings.offset_mm)
        for _ in range(offset):
            page.click(f'{FIGURE} .ix-next')
        page.emulate_media(media=PRINT)
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'))
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-live-plot'))
        notes = page.inner_text(f'{FIGURE} .ix-plotnote')
        for note in (drawing.grid_note, drawing.time_note, drawing.example_note):
            self.assertIn(note, notes)
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-legend [data-view="axle"]'))
        shown = 'parts => parts.filter(part => part.checkVisibility()).length'
        self.assertEqual(page.eval_on_selector_all(f'{FIGURE} .ix-answer:not([data-live])', shown),
                         len(figures.EXAMPLES))


class TestPageLog(SimulationTestPageCase):
    """The page's log tells the test's own stopped requests from failures that are the page's."""

    # A local address that the test below lets fail, as a broken connection would
    UNRELATED_FILE = '/unrelated-file.txt'
    CONNECTION_RESET = 'connectionreset'

    def test_a_request_that_the_test_stopped_is_no_error(self) -> None:
        opened = self.open_simulation_test_page(blocked=('diffdrive.js',))
        opened.page.wait_for_load_state('networkidle')
        self.assertTrue(opened.log.blocked, 'the page never asked for the blocked file')
        self.assertEqual(opened.log.errors, [])

    def test_a_failed_request_that_the_test_did_not_stop_is_an_error(self) -> None:
        opened = self.open_simulation_test_page()
        page = opened.page
        page.wait_for_load_state('networkidle')
        # The page's own route fails the request: the log's routing never sees it
        page.route(f'**{self.UNRELATED_FILE}', lambda route: route.abort(self.CONNECTION_RESET))
        page.evaluate('file => fetch(file).catch(() => null)', self.UNRELATED_FILE)
        failed = [error for error in opened.log.errors if self.UNRELATED_FILE in error]
        self.assertTrue(failed, opened.log.errors)
        self.assertTrue(any('ERR_CONNECTION_RESET' in error for error in failed), failed)

    def test_an_error_that_the_page_writes_to_the_console_is_an_error(self) -> None:
        opened = self.open_simulation_test_page()
        opened.page.evaluate('() => console.error("The figure broke")')
        self.assertIn('The figure broke', opened.log.errors)

    def test_a_page_that_fails_to_open_leaves_no_browser_context_open(self) -> None:
        contexts_before = len(self.browser.contexts)
        with self.assertRaises(PlaywrightError):
            open_page(self.browser, self.base, PAGE, blocked=(PAGE,))
        self.assertEqual(len(self.browser.contexts), contexts_before)


class TestDiffDriveModule(SimulationTestPageCase):
    """diffdrive.js gives the results that follow from the geometry, with the limits it is given.

    The cases and their expected results are those of TestKinematics and TestModelLimits in
    tests/test_motion_lab_figures.py. Each test calls the module's function in the page.
    """

    # Values that floating-point numbers hold exactly, so the boundary is the boundary
    EXPECTED = 1.0
    TOLERANCE = 0.25
    BOUNDARY = EXPECTED + TOLERANCE
    SPEED = 1.0
    DURATION = 1.0

    def passes_in_page(self, value: float) -> bool:
        """Return whether a value is within the tolerance, with the script's rounding allowance."""
        page = self.open_simulation_test_page().page
        result: bool = call_model(page, 'withinCheckTolerance', value, self.EXPECTED,
                                  self.TOLERANCE, figures.ROUNDING_ALLOWANCE)
        return result

    def pose_after(self, angular_velocity: float) -> Dict[str, float]:
        """Return the pose after DURATION at SPEED and an angular velocity."""
        page = self.open_simulation_test_page().page
        pose: Dict[str, float] = call_model(page, 'poseAfter', self.SPEED, angular_velocity,
                                            self.DURATION, figures.STRAIGHT_ANGULAR_VELOCITY)
        return pose

    def test_a_motion_is_measured_in_the_frame_of_the_start_pose(self) -> None:
        # The robot looks along the ground's y axis and moves 1 m along it: 1 m forward, not left
        page = self.open_simulation_test_page().page
        start = {'x': 1.0, 'y': 2.0, 'theta': math.pi / 2}
        end = {'x': 1.0, 'y': 3.0, 'theta': math.pi / 2}
        motion = call_model(page, 'relativeMotion', start, end)
        self.assertAlmostEqual(motion['forward'], 1.0, places=MODEL_PLACES)
        self.assertAlmostEqual(motion['left'], 0.0, places=MODEL_PLACES)
        self.assertAlmostEqual(motion['turned'], 0.0, places=MODEL_PLACES)

    def test_a_turning_drive_follows_a_circle(self) -> None:
        # 1 m/s and 1 rad/s give a circle of radius 1 m around (0, 1). After a quarter turn the
        # robot is at (1, 1) and looks along the ground's y axis
        quarter_turn = math.pi / 2
        page = self.open_simulation_test_page().page
        pose = call_model(page, 'poseAfter', 1.0, 1.0, quarter_turn,
                          figures.STRAIGHT_ANGULAR_VELOCITY)
        self.assertAlmostEqual(pose['x'], 1.0, places=MODEL_PLACES)
        self.assertAlmostEqual(pose['y'], 1.0, places=MODEL_PLACES)
        self.assertAlmostEqual(pose['theta'], quarter_turn, places=MODEL_PLACES)

    def test_a_value_on_the_tolerance_boundary_passes(self) -> None:
        self.assertIs(self.passes_in_page(self.BOUNDARY), True)

    def test_a_value_beyond_the_boundary_by_less_than_the_allowance_passes(self) -> None:
        self.assertIs(self.passes_in_page(self.BOUNDARY + figures.ROUNDING_ALLOWANCE / 2), True)

    def test_a_value_beyond_the_boundary_by_more_than_the_allowance_fails(self) -> None:
        self.assertIs(self.passes_in_page(self.BOUNDARY + 2 * figures.ROUNDING_ALLOWANCE), False)

    def test_an_angular_velocity_below_the_straight_limit_drives_straight(self) -> None:
        pose = self.pose_after(figures.STRAIGHT_ANGULAR_VELOCITY / 2)
        self.assertEqual(pose, {'x': self.SPEED * self.DURATION, 'y': 0, 'theta': 0})

    def test_an_angular_velocity_above_the_straight_limit_turns(self) -> None:
        angular = 2 * figures.STRAIGHT_ANGULAR_VELOCITY
        self.assertEqual(self.pose_after(angular)['theta'], angular * self.DURATION)


if __name__ == '__main__':
    unittest.main()
