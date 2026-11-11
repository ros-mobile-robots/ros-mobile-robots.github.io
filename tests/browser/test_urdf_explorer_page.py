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
# The robot that the open explorer shows, and what its controls and texts say about it
SHOWN = f"""() => {{
    const root = document.querySelector('{FIGURE}');
    const {{ robot, state }} = root.ixView.debug();
    const status = root.querySelector('.ix-status');
    return {{
        selected: root.querySelector('.ix-robot-select').value,
        state: state.robot,
        model: robot.robotName,
        links: Object.keys(robot.links).length,
        listed: root.querySelectorAll('.ix-tree button').length,
        label: root.querySelector('.ix-canvas').getAttribute('aria-label'),
        derived: root.querySelector('.ix-derived').textContent,
        status: status.hidden ? '' : status.textContent,
    }};
}}"""
REMO_SHOWN = f"document.querySelector('{FIGURE}').ixView.debug().robot.robotName === 'remo'"
SWITCH_FAILED = f"document.querySelector('{FIGURE} .ix-status').textContent.includes('couldn’t')"
STATUS_HIDDEN = f"document.querySelector('{FIGURE} .ix-status').hidden"
# The slider of the drive wheels: its value, the angle shown next to it, the angle that a screen
# reader speaks, and the angles of the two wheel joints in degrees
WHEELS = f"""() => {{
    const root = document.querySelector('{FIGURE}');
    const {{ robot, state }} = root.ixView.debug();
    const data = JSON.parse(root.querySelector('.ix-data').textContent);
    const slider = root.querySelector('.ix-wheels');
    return {{
        slider: slider.value,
        shown: slider.closest('.ix-control').querySelector('output').textContent,
        spoken: slider.getAttribute('aria-valuetext'),
        joints: data.robots[state.robot].wheelJoints.map(
            (name) => Math.round(robot.joints[name].angle * 180 / Math.PI)),
    }};
}}"""
# The buttons of the link list that reach beyond the inside of their panel or cut their name
# off, with the distance in px by which they do
BUTTONS_OUTSIDE_PANEL = """(tree) => {
    const panel = tree.closest('.ix-panel');
    const box = panel.getBoundingClientRect(), style = getComputedStyle(panel);
    const left = box.left + parseFloat(style.paddingLeft);
    const right = box.right - parseFloat(style.paddingRight);
    const outside = [];
    for (const button of tree.querySelectorAll('button')) {
        const own = button.getBoundingClientRect();
        const beyond = Math.max(left - own.left, own.right - right);
        const cut = button.scrollWidth - button.clientWidth;
        if (beyond > 0.5 || cut > 0) outside.push([button.dataset.link, Math.max(beyond, cut)]);
    }
    return outside;
}"""


def wait_until(page, condition, timeout=60000):
    """Wait until a condition on the test's side holds, while the page keeps working."""
    waited = 0
    while not condition():
        if waited >= timeout:
            raise TimeoutError('the condition did not hold in time')
        page.wait_for_timeout(50)
        waited += 50


def requests_under_way(page):
    """Return the set of the page's requests that haven't ended yet, kept current from now on."""
    under_way = set()
    page.on('request', lambda request: under_way.add(request))
    page.on('requestfinished', lambda request: under_way.discard(request))
    page.on('requestfailed', lambda request: under_way.discard(request))
    return under_way


def wait_for_quiet(page, under_way, quiet=1000, timeout=120000):
    """Wait until no request has been under way for a while, so a load has run to its end."""
    still = waited = 0
    while still < quiet:
        if waited >= timeout:
            raise TimeoutError('requests were still under way')
        page.wait_for_timeout(50)
        waited += 50
        still = 0 if under_way else still + 50


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

    def choose_robot(self, page, key):
        page.select_option(f'{FIGURE} .ix-robot-select', key)

    def show_remo(self, page):
        self.choose_robot(page, 'remo')
        page.wait_for_function(REMO_SHOWN, timeout=120000)

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
        self.show_remo(page)
        positions = page.evaluate(POSITIONS)
        self.assertEqual(len(positions), 25)
        self.assertEqual(positions['front_left_wheel'], [0, 72, 33])
        self.assertEqual(log.errors, [])

    def test_robot_chosen_last_is_shown_when_an_earlier_choice_arrives_later(self):
        page, log = self.open_view()
        under_way = requests_under_way(page)
        held = []
        page.route('**/remo.urdf', lambda route: held.append(route))
        self.choose_robot(page, 'remo')
        wait_until(page, lambda: held)
        self.choose_robot(page, 'diffbot')
        page.wait_for_function(STATUS_HIDDEN, timeout=120000)
        # Remo's URDF arrives only now, after the reader has gone back to DiffBot
        held[0].continue_()
        page.unroute('**/remo.urdf')
        wait_for_quiet(page, under_way)
        shown = page.evaluate(SHOWN)
        self.assertEqual(
            (shown['selected'], shown['state'], shown['model']), ('diffbot', 'diffbot', 'diffbot')
        )
        self.assertEqual((shown['links'], shown['listed']), (11, 11))
        self.assertIn("In DiffBot's model", shown['derived'])
        self.assertEqual(shown['status'], '')
        self.assertEqual(log.errors, [])
        # The reader can still choose Remo afterwards
        self.show_remo(page)
        shown = page.evaluate(SHOWN)
        self.assertEqual((shown['selected'], shown['state'], shown['links']), ('remo', 'remo', 25))

    def test_failed_robot_choice_keeps_the_shown_robot(self):
        page, _ = self.open_view()
        page.route('**/remo.urdf', lambda route: route.abort())
        self.choose_robot(page, 'remo')
        page.wait_for_function(SWITCH_FAILED, timeout=60000)
        # A step writes the canvas's description again, with the robot's name
        page.click(f'{FIGURE} [data-step="collision"]')
        shown = page.evaluate(SHOWN)
        self.assertEqual(
            (shown['selected'], shown['state'], shown['model']), ('diffbot', 'diffbot', 'diffbot')
        )
        self.assertIn("DiffBot's robot model", shown['label'])
        self.assertIn("In DiffBot's model", shown['derived'])

    def test_robot_choice_waits_while_the_view_opens(self):
        page, _ = self.open()
        held = []
        page.route('**/diffbot.urdf', lambda route: held.append(route))
        page.click(f'{FIGURE} .ix-open')
        wait_until(page, lambda: held)
        self.assertTrue(page.is_disabled(f'{FIGURE} .ix-robot-select'))
        held[0].continue_()
        page.unroute('**/diffbot.urdf')
        page.wait_for_selector(f'{FIGURE}.ix-is-live', timeout=120000)
        self.assertTrue(page.is_enabled(f'{FIGURE} .ix-robot-select'))
        shown = page.evaluate(SHOWN)
        self.assertEqual(
            (shown['selected'], shown['state'], shown['links']), ('diffbot', 'diffbot', 11)
        )

    def test_robot_loads_when_chosen_again_after_a_file_failed(self):
        page, _ = self.open_view()
        page.route('**/raspberry_pi_4b.glb', lambda route: route.abort())
        self.choose_robot(page, 'remo')
        page.wait_for_function(SWITCH_FAILED, timeout=60000)
        self.assertEqual(page.evaluate(SHOWN)['model'], 'diffbot')
        # The file can be loaded again, and the reader chooses Remo once more
        page.unroute('**/raspberry_pi_4b.glb')
        self.choose_robot(page, 'remo')
        page.wait_for_function(f'{REMO_SHOWN} || {SWITCH_FAILED}', timeout=120000)
        shown = page.evaluate(SHOWN)
        self.assertEqual(
            (shown['selected'], shown['state'], shown['model'], shown['links']),
            ('remo', 'remo', 'remo', 25),
        )
        self.assertEqual(shown['status'], '')

    def test_link_names_stay_inside_the_links_panel(self):
        phone = {'height': 700, 'is_mobile': True, 'has_touch': True}
        for width, options in ((1280, {}), (320, phone)):
            with self.subTest(width=width):
                page, _ = self.open_view(width=width, **options)
                self.show_remo(page)
                tree = f'{FIGURE} .ix-tree'
                # Remo has the longest name, six levels deep in the tree
                longest = f'{tree} [data-link="slamtec_usb_to_serial_link"]'
                self.assertTrue(page.is_visible(longest))
                self.assertEqual(page.eval_on_selector(tree, BUTTONS_OUTSIDE_PANEL), [])

    def test_wheel_angle_is_shown_and_spoken_when_the_view_opens(self):
        page, _ = self.open_view()
        self.assertEqual(
            page.evaluate(WHEELS),
            {'slider': '0', 'shown': '0°', 'spoken': '0 degrees', 'joints': [0, 0]},
        )

    def test_wheel_angle_starts_at_zero_again_for_another_robot(self):
        page, _ = self.open_view()
        page.eval_on_selector(f'{FIGURE} .ix-settings', 'details => { details.open = true; }')
        page.fill(f'{FIGURE} .ix-wheels', '90')
        page.dispatch_event(f'{FIGURE} .ix-wheels', 'input')
        self.assertEqual(
            page.evaluate(WHEELS),
            {'slider': '90', 'shown': '90°', 'spoken': '90 degrees', 'joints': [90, 90]},
        )
        self.show_remo(page)
        self.assertEqual(
            page.evaluate(WHEELS),
            {'slider': '0', 'shown': '0°', 'spoken': '0 degrees', 'joints': [0, 0]},
        )

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
