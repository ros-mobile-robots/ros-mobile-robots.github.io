"""Browser tests of the interactive figure on the Simulation Test page, in the built site.

Build the site first, then run from the repository root:

    mkdocs build --strict
    python -m unittest discover -s tests/browser -v

The tests take the figure's values from scripts/figures/motion_lab_figures.py, which writes the
figure's include, and compare the page's numbers with the ones that the script computed.
"""

import contextlib
import json
from typing import Any
from typing import Dict
from typing import List
from typing import NamedTuple
import unittest

from local_site import chromium
from local_site import open_page
from local_site import OpenedPage
from local_site import served_site
from local_site import SITE_HOSTS
# local_site puts scripts/figures on the import path
import motion_lab_figures as figures
from playwright.sync_api import Browser
from playwright.sync_api import Page

PAGE = '/development/simulation-test/'
FIGURE = 'figure[data-ix="motion-lab"]'
# The figure starts within this time, also on a slow CI runner
START_TIMEOUT_MS = 15000
MILLISECONDS_PER_SECOND = 1000
# A page whose script failed to load has settled after this time
SETTLE_TIME_MS = 500
# The tables show three decimals: a shown value lies within half of its last digit of the value,
# plus floating-point slack
SHOWN_ROUNDING = 0.5e-3 + 1e-9
# The check table's text of a failing check
FAILS = 'fails'
# Both drawings write coordinates to a tenth of a pixel, the live one sometimes unrounded
DRAWN_ROUNDING = 0.1 + 1e-9
# The shapes of a drawing's robots and the command's cross, in drawing order: the group's view,
# the shape's kind, and its numbers (a polygon's points, a circle's center, the cross's path)
SHAPES = """svg => [...svg.querySelectorAll(
    '.ix-robot polygon, .ix-robot circle, .ix-robot line, .ix-cross')].map((shape) => {
  const view = shape.closest('.ix-robot').classList[1];
  const numbers = {
    polygon: () => shape.getAttribute('points').split(/[ ,]/),
    circle: () => ['cx', 'cy', 'r'].map((name) => shape.getAttribute(name)),
    line: () => ['x1', 'y1', 'x2', 'y2'].map((name) => shape.getAttribute(name)),
    path: () => shape.getAttribute('d').match(/-?[0-9.]+/g),
  }[shape.tagName]();
  return [view, shape.tagName, ...numbers.map(Number)];
})"""
PHONE = {'width': 390, 'height': 844, 'is_mobile': True, 'has_touch': True}
SMALL_PHONE = {**PHONE, 'width': 320, 'height': 700}


class MotionRow(NamedTuple):
    """A row of the motion table, as its cells show it: the view and its three values."""

    view: str
    forward: str
    left: str
    turned: str


class CheckRow(NamedTuple):
    """A row of the check table, as its cells show it."""

    name: str
    allowed: str
    value: str
    result: str


def table_cells(page: Page, selector: str) -> List[List[str]]:
    """Return the rows of a table body as lists of their cells' texts."""
    cells: List[List[str]] = page.eval_on_selector_all(
        selector, 'rows => rows.map(row => [...row.children].map(cell => cell.innerText.trim()))'
    )
    return cells


def motion_rows(page: Page) -> List[MotionRow]:
    """Return the motion table's rows."""
    return [MotionRow(*cells) for cells in table_cells(page, f'{FIGURE} .ix-motion-rows tr')]


def check_rows(page: Page) -> List[CheckRow]:
    """Return the check table's rows."""
    return [CheckRow(*cells) for cells in table_cells(page, f'{FIGURE} .ix-check-rows tr')]


def failing_checks(page: Page) -> List[str]:
    """Return the names of the checks that the check table shows as failing."""
    return [row.name for row in check_rows(page) if FAILS in row.result]


def shown_number(text: str) -> float:
    """Return the number at the start of a cell such as '−0.020 m'."""
    return float(text.split()[0].replace('−', '-'))


def expected_results(page: Page, example: figures.Example) -> Dict[str, Any]:
    """Return the results that the script wrote into an example's item for these tests."""
    text = page.get_attribute(f'{FIGURE} [data-example-item="{example.key}"]', 'data-expected')
    if text is None:
        raise ValueError(f'the example {example.key} has no expected results')
    results: Dict[str, Any] = json.loads(text)
    return results


def top_of(page: Page, selector: str) -> float:
    """Return the top of an element in the page, in px from the page's top."""
    top: float = page.eval_on_selector(
        selector, 'element => element.getBoundingClientRect().top + window.scrollY'
    )
    return top


class TestMotionLabPage(unittest.TestCase):
    """The figure in the page: it starts, shows the script's numbers, and falls back."""

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

    def assert_motion_table_shows(
        self, page: Page, expected: Dict[str, Any], example: figures.Example
    ) -> None:
        """Assert that the motion table shows the odometry's and the true pose's motion."""
        names = {entry.view: entry.text for entry in figures.LEGEND}
        rows = {row.view: row for row in motion_rows(page)}
        for view in ('odometry', 'model'):
            row = rows[names[view]]
            for shown, value in zip((row.forward, row.left, row.turned), expected[view]):
                self.assertAlmostEqual(shown_number(shown), value, delta=SHOWN_ROUNDING,
                                       msg=f'{example.key}, {view}')

    def assert_check_table_shows(
        self, page: Page, expected: Dict[str, Any], example: figures.Example
    ) -> None:
        """Assert that the check table has the expected checks and shows which ones fail."""
        self.assertEqual(len(check_rows(page)), expected['checks'], example.key)
        self.assertEqual(failing_checks(page), expected['failing'], example.key)

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
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-step-count'),
                         f'Example 1 of {len(figures.EXAMPLES)}')

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
        static = page.eval_on_selector(f'{FIGURE} .ix-static', SHAPES)
        live = page.eval_on_selector(f'{FIGURE} .ix-live-plot', SHAPES)
        self.assertTrue(static, 'the static drawing has no robot')
        self.assertEqual([shape[:2] for shape in live], [shape[:2] for shape in static])
        for live_shape, static_shape in zip(live, static):
            for live_value, static_value in zip(live_shape[2:], static_shape[2:]):
                self.assertAlmostEqual(live_value, static_value, delta=DRAWN_ROUNDING,
                                       msg=live_shape[:2])

    def test_an_example_before_its_answer_shows_the_robots_values_where_all_checks_pass(
        self,
    ) -> None:
        page = self.open_live_figure().page
        for index, example in enumerate(figures.EXAMPLES):
            if index:
                page.click(f'{FIGURE} .ix-next')
            self.assertFalse(page.is_visible(f'{FIGURE} .ix-shown-answer'), example.key)
            self.assertEqual(failing_checks(page), [], example.key)

    def test_each_answer_shows_under_the_drawing_with_the_scripts_numbers(self) -> None:
        # With reduced motion, the changed drive and its answer show at once
        page = self.open_live_figure(reduced_motion='reduce').page
        for index, example in enumerate(figures.EXAMPLES):
            if index:
                page.click(f'{FIGURE} .ix-next')
            page.click(f'{FIGURE} [data-example="{example.key}"]')
            answer = page.text_content(f'{FIGURE} [data-example-item="{example.key}"] .ix-answer')
            self.assertIn(answer or '', page.inner_text(f'{FIGURE} .ix-shown-answer'), example.key)
            expected = expected_results(page, example)
            self.assert_motion_table_shows(page, expected, example)
            self.assert_check_table_shows(page, expected, example)

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

    def test_a_value_on_a_tolerance_boundary_passes(self) -> None:
        turned = figures.CHECKS[figures.CommandKind.TURN].each_view[0]
        boundary = turned.expected - turned.tolerance
        # The wheels reach the share of their speed that ends the turn exactly on the boundary
        tracking = round(boundary / figures.COMMANDS[figures.CommandKind.TURN].angle * 100)
        slider = next(
            candidate for candidate in figures.WHAT_IF_SLIDERS
            if candidate.setting == 'tracking_percent'
        )
        page = self.open_live_figure().page
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.click(f'{FIGURE} .ix-reset')
        page.click(f'{FIGURE} [data-command="turn"]')
        page.fill(f'#{slider.element_id}', str(tracking))
        page.dispatch_event(f'#{slider.element_id}', 'input')
        row = next(row for row in check_rows(page) if row.name == 'turned, true pose')
        self.assertEqual(row.value, f'{boundary:.3f} rad')
        self.assertIn('passes', row.result)

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

    def test_static_version_stays_without_javascript(self) -> None:
        page = self.open_simulation_test_page(javascript=False).page
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'))
        shown = 'parts => parts.filter(part => part.offsetParent).length'
        self.assertEqual(page.eval_on_selector_all(f'{FIGURE} .ix-answer', shown),
                         len(figures.EXAMPLES))
        self.assertEqual(page.eval_on_selector_all(f'{FIGURE} [data-live]', shown), 0)

    def test_static_version_stays_when_a_script_fails_to_load(self) -> None:
        shown = 'parts => parts.filter(part => part.offsetParent).length'
        for missing in ('motion-lab.js', 'diffdrive.js'):
            with self.subTest(missing=missing):
                page = self.open_simulation_test_page(blocked=(missing,)).page
                page.wait_for_timeout(SETTLE_TIME_MS)
                self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'))
                self.assertEqual(page.eval_on_selector_all(f'{FIGURE} .ix-answer', shown),
                                 len(figures.EXAMPLES))

    def test_play_jumps_to_the_end_without_animation_with_reduced_motion(self) -> None:
        page = self.open_live_figure(reduced_motion='reduce').page
        scrub = f'{FIGURE} .ix-scrub'
        page.fill(scrub, str(figures.TIME_SLIDER_STEPS // 5))
        page.dispatch_event(scrub, 'input')
        page.click(f'{FIGURE} .ix-play')
        self.assertEqual(page.input_value(scrub), str(figures.TIME_SLIDER_STEPS))
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-play'), 'Play')


if __name__ == '__main__':
    unittest.main()
