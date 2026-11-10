"""Browser tests of the URDF explorer on the Robot Description page, in the built site.

Build the site first, then run from the repository root:

    mkdocs build --strict
    python -m unittest discover -s tests/browser -v
"""
import unittest

from local_site import chromium, open_page, served_site, SITE_HOSTS

PAGE = '/robot-description/'
FIGURE = 'figure[data-ix="urdf-explorer"]'
# What the 3D view needs. None of it may be requested before the reader opens the view
VIEW_FILES = ('three-0.170.0', 'urdf-loader-0.13.1', 'urdf-explorer-view.js', '.urdf', '.glb')

# The positions of a robot's link frames in its root frame, in mm
POSITIONS = f"""() => {{
    const robot = document.querySelector('{FIGURE}').ixView.debug().robot;
    const result = {{}};
    for (const link of Object.values(robot.links)) {{
        link.updateWorldMatrix(true, false);
        const m = link.matrixWorld.elements;
        result[link.name] = [m[12], m[13], m[14]].map((value) => Math.round(value * 1e4) / 10);
    }}
    return result;
}}"""
# The direction of a link's x axis in the root frame
X_AXIS = f"""(name) => {{
    const link = document.querySelector('{FIGURE}').ixView.debug().robot.links[name];
    link.updateWorldMatrix(true, false);
    const axis = [...link.matrixWorld.elements].slice(0, 3);
    return axis.map((value) => Math.round(value * 1e3) / 1e3);
}}"""


class TestUrdfExplorerPage(unittest.TestCase):
    """The explorer in the page: nothing loads before it is opened, and it shows the URDF."""

    @classmethod
    def setUpClass(cls):
        cls.site = served_site()
        cls.base = cls.site.__enter__()
        cls.chromium = chromium(webgl=True)
        cls.browser = cls.chromium.__enter__()

    @classmethod
    def tearDownClass(cls):
        cls.chromium.__exit__(None, None, None)
        cls.site.__exit__(None, None, None)

    def open(self, **options):
        context, page, log = open_page(self.browser, self.base, PAGE, **options)
        self.addCleanup(context.close)
        return page, log

    def open_view(self, **options):
        page, log = self.open(**options)
        page.click(f'{FIGURE} .ix-open')
        page.wait_for_selector(f'{FIGURE}.ix-is-live', timeout=120000)
        return page, log

    def test_page_loads_nothing_of_the_3d_view_before_it_is_opened(self):
        page, log = self.open()
        page.wait_for_timeout(500)
        early = [url for url in log.requests if any(part in url for part in VIEW_FILES)]
        self.assertEqual(early, [])
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-poster'))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-open'))
        self.assertEqual(log.errors, [])

    def test_view_opens_without_errors_and_adds_no_host(self):
        page, log = self.open_view()
        self.assertEqual(log.errors, [])
        self.assertLessEqual(set(log.other_hosts), SITE_HOSTS)
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-poster'))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-canvas'))
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-view-hint'))

    def test_diffbot_frames_are_the_urdfs(self):
        page, _ = self.open_view()
        positions = page.evaluate(POSITIONS)
        self.assertEqual(len(positions), 11)
        # base_link above base_footprint at the wheel radius plus the wheel's z offset, and the
        # drive wheels 69 mm left and right of it at the wheel radius above the ground
        self.assertEqual(positions['base_footprint'], [0, 0, 0])
        self.assertEqual(positions['base_link'], [0, 0, 50.3])
        self.assertEqual(positions['front_left_wheel'], [0, 69, 33.3])
        self.assertEqual(positions['front_right_wheel'], [0, -69, 33.3])
        self.assertEqual(positions['chassis_link'], [-18.5, 0, 50.3])

    def test_wheel_frame_turns_with_the_wheel(self):
        page, _ = self.open_view()
        self.assertEqual(page.evaluate(X_AXIS, 'front_left_wheel'), [1, 0, 0])
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.fill(f'{FIGURE} .ix-wheels', '90')
        page.dispatch_event(f'{FIGURE} .ix-wheels', 'input')
        # A quarter turn about the wheel's y axis points its x axis down
        self.assertEqual(page.evaluate(X_AXIS, 'front_left_wheel'), [0, 0, -1])

    def test_steps_show_their_layers_and_the_selected_link(self):
        page, _ = self.open_view()
        page.click(f'{FIGURE} [data-step="collision"]')
        info = page.inner_text(f'{FIGURE} .ix-info')
        self.assertIn('front_left_wheel', info)
        self.assertIn('sphere, radius 33.3 mm', info)
        self.assertIn('32 g', info)
        self.assertIn('model: 65 mm wheel', info)
        self.assertTrue(page.is_checked(f'{FIGURE} [data-layer="collision"]'))
        self.assertFalse(page.is_checked(f'{FIGURE} [data-layer="frames"]'))

    def test_link_list_selects_by_keyboard(self):
        page, _ = self.open_view()
        page.focus(f'{FIGURE} .ix-tree button[data-link="chassis_link"]')
        page.keyboard.press('Enter')
        info = page.inner_text(f'{FIGURE} .ix-info')
        self.assertIn('chassis_link', info)
        self.assertIn('simple shape', info)
        self.assertIn('771 g', info)

    def test_remo_loads(self):
        page, log = self.open_view()
        page.select_option(f'{FIGURE} .ix-robot-select', 'remo')
        loaded = f"document.querySelector('{FIGURE}').ixView.debug().robot.robotName === 'remo'"
        page.wait_for_function(loaded, timeout=120000)
        positions = page.evaluate(POSITIONS)
        self.assertEqual(len(positions), 25)
        self.assertEqual(positions['front_left_wheel'], [0, 72, 33])
        self.assertEqual(log.errors, [])

    def test_mouse_wheel_scrolls_the_page_unless_ctrl_is_held(self):
        page, _ = self.open_view()
        distance = f"document.querySelector('{FIGURE}').ixView.debug().camera.position.length()"
        box = page.locator(f'{FIGURE} .ix-canvas').bounding_box()
        page.mouse.move(box['x'] + box['width'] / 2, box['y'] + box['height'] / 2)
        before, scrolled = page.evaluate(distance), page.evaluate('window.scrollY')
        page.mouse.wheel(0, 200)
        page.wait_for_timeout(300)
        self.assertGreater(page.evaluate('window.scrollY'), scrolled)
        self.assertAlmostEqual(page.evaluate(distance), before, places=6)

    def test_picture_and_tables_stay_without_javascript(self):
        page, _ = self.open(javascript=False)
        self.assertTrue(page.is_visible(f'{FIGURE} .ix-poster'))
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-open'))
        self.assertFalse(page.is_visible(f'{FIGURE} .ix-view-hint'))
        tables = page.eval_on_selector_all(f'{FIGURE} .ix-link-tables table', 'all => all.length')
        self.assertEqual(tables, 2)

    def test_picture_stays_when_the_3d_files_fail_to_load(self):
        for missing in ('three.module.min.js', 'tt_motor_wheel_65mm.glb', 'diffbot.urdf'):
            page, _ = self.open(blocked=(missing,))
            enabled = f"!document.querySelector('{FIGURE} .ix-open').disabled"
            page.wait_for_selector(f'{FIGURE} .ix-open', state='visible')
            page.click(f'{FIGURE} .ix-open')
            page.wait_for_function(enabled, timeout=60000)
            self.assertTrue(page.is_visible(f'{FIGURE} .ix-poster'), missing)
            self.assertFalse(page.is_visible(f'{FIGURE} .ix-explorer'), missing)
            status = page.inner_text(f'{FIGURE} .ix-status')
            self.assertIn('couldn’t start', status, missing)


if __name__ == '__main__':
    unittest.main()
