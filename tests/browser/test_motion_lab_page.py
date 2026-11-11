"""Browser tests of the interactive figure on the Simulation Test page, in the built site.

Build the site first, then run from the repository root:

    mkdocs build --strict
    python -m unittest discover -s tests/browser -v
"""

import json
import unittest

from local_site import chromium, open_page, served_site, SITE_HOSTS

PAGE = '/development/simulation-test/'
FIGURE = 'figure[data-ix="motion-lab"]'


def table_rows(page, selector):
    """Return a table body's rows as lists of cell texts."""
    return page.eval_on_selector_all(
        selector, 'rows => rows.map(row => [...row.children].map(cell => cell.innerText.trim()))'
    )


def number(text):
    """Return the number at the start of a cell such as '−0.020 m'."""
    return float(text.split()[0].replace('−', '-'))


class TestMotionLabPage(unittest.TestCase):
    """The figure in the page: it starts, shows the generator's numbers, and falls back."""

    @classmethod
    def setUpClass(cls):
        cls.site = served_site()
        cls.base = cls.site.__enter__()
        cls.chromium = chromium()
        cls.browser = cls.chromium.__enter__()

    @classmethod
    def tearDownClass(cls):
        cls.chromium.__exit__(None, None, None)
        cls.site.__exit__(None, None, None)

    def open(self, **options):
        context, page, log = open_page(self.browser, self.base, PAGE, **options)
        self.addCleanup(context.close)
        return page, log

    def open_live(self, **options):
        page, log = self.open(**options)
        page.wait_for_selector(f'{FIGURE}.ix-is-live', timeout=15000)
        return page, log

    def test_figure_starts_without_errors_and_adds_no_host(self):
        page, log = self.open_live()
        self.assertEqual(log.errors, [])
        self.assertLessEqual(set(log.other_hosts), SITE_HOSTS)
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-static'))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-live-plot'))
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-step-count'), 'Example 1 of 5')

    def test_examples_show_the_generators_numbers(self):
        page, _ = self.open_live()
        items = page.query_selector_all(f'{FIGURE} [data-example-item]')
        self.assertEqual(len(items), 5)
        for index, item in enumerate(items):
            example = item.get_attribute('data-example-item')
            expected = json.loads(item.get_attribute('data-expected'))
            if index:
                page.click(f'{FIGURE} .ix-next')
                # Before the answer: the robot's own values, with which every check passes
                self.assertFalse(item.query_selector('.ix-answer').is_visible(), example)
                self.assertFalse(
                    [
                        row
                        for row in table_rows(page, f'{FIGURE} .ix-check-rows tr')
                        if 'fails' in row[3]
                    ],
                    example,
                )
                page.click(f'{FIGURE} [data-example="{example}"]')
            self.assertTrue(item.query_selector('.ix-answer').is_visible(), example)
            motion = table_rows(page, f'{FIGURE} .ix-motion-rows tr')
            for row, view in ((motion[1], 'odometry'), (motion[2], 'model')):
                for shown, value in zip(row[1:], expected[view]):
                    self.assertAlmostEqual(
                        number(shown), value, delta=5.1e-4, msg=f'{example}, {view}'
                    )
            checks = table_rows(page, f'{FIGURE} .ix-check-rows tr')
            self.assertEqual(len(checks), expected['checks'], example)
            self.assertEqual(
                [row[0] for row in checks if 'fails' in row[3]], expected['failing'], example
            )

    def test_value_on_a_tolerance_boundary_passes(self):
        page, _ = self.open_live()
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.click(f'{FIGURE} .ix-reset')
        page.click(f'{FIGURE} [data-command="turn"]')
        page.fill('#motion-lab-tracking', '80')
        page.dispatch_event('#motion-lab-tracking', 'input')
        turned = table_rows(page, f'{FIGURE} .ix-check-rows tr')[0]
        self.assertEqual(turned[2], '1.200 rad')
        self.assertIn('passes', turned[3])

    def test_narrow_screen_has_no_sideways_overflow(self):
        page, _ = self.open_live(width=320, height=700, is_mobile=True, has_touch=True)
        self.assertEqual(
            page.evaluate(
                'document.documentElement.scrollWidth - document.documentElement.clientWidth'
            ),
            0,
        )
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

    def test_static_version_stays_without_javascript(self):
        page, _ = self.open(javascript=False)
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'))
        self.assertEqual(
            page.eval_on_selector_all(
                f'{FIGURE} .ix-answer', 'answers => answers.filter(a => a.offsetParent).length'
            ),
            5,
        )
        self.assertEqual(
            page.eval_on_selector_all(
                f'{FIGURE} [data-live]', 'parts => parts.filter(p => p.offsetParent).length'
            ),
            0,
        )

    def test_static_version_stays_when_the_script_fails_to_load(self):
        for missing in ('motion-lab.js', 'diffdrive.js'):
            page, _ = self.open(blocked=(missing,))
            page.wait_for_timeout(500)
            self.assertTrue(page.is_visible(f'{FIGURE} .ix-static'), missing)
            self.assertEqual(
                page.eval_on_selector_all(
                    f'{FIGURE} .ix-answer', 'answers => answers.filter(a => a.offsetParent).length'
                ),
                5,
                missing,
            )

    def test_play_follows_reduced_motion(self):
        page, _ = self.open_live(reduced_motion='reduce')
        page.fill(f'{FIGURE} .ix-scrub', '200')
        page.dispatch_event(f'{FIGURE} .ix-scrub', 'input')
        page.click(f'{FIGURE} .ix-play')
        self.assertEqual(page.input_value(f'{FIGURE} .ix-scrub'), '1000')
        self.assertEqual(page.inner_text(f'{FIGURE} .ix-play'), 'Play')


if __name__ == '__main__':
    unittest.main()
