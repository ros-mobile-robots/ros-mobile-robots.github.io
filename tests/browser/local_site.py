"""Helpers for the browser tests: serve the built site and open its pages in headless Chromium.

A test page only talks to the local server. Requests to any other host are stopped and
recorded, so a test run never loads an ad, the consent message or Analytics, and a test can
check which hosts a page asks for.
"""

import contextlib
import functools
import http.server
import os
from pathlib import Path
import sys
import threading
from typing import Any
from typing import Iterator
from typing import List
from typing import NamedTuple
from typing import Tuple
from urllib.parse import urlparse

from playwright.sync_api import Browser
from playwright.sync_api import BrowserContext
from playwright.sync_api import ConsoleMessage
from playwright.sync_api import Error
from playwright.sync_api import Page
from playwright.sync_api import Route
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parent.parent.parent
# The tests take a figure's values from the script that writes its include: they import the
# scripts of scripts/figures, which this line puts on the import path
sys.path.insert(0, str(ROOT / 'scripts' / 'figures'))
# The built site: mkdocs build --strict writes it to site/. SITE_DIR names another folder
SITE_DIR = Path(os.environ.get('SITE_DIR', ROOT / 'site'))
# The hosts that the site's own template asks for on every page: the ad and consent script, and
# the repository's star count in the header. A figure must not add a host to them
SITE_HOSTS = {'pagead2.googlesyndication.com', 'api.github.com'}
# A request that open_page() stopped also logs a console error with this text, which isn't the
# page's fault
STOPPED_REQUEST = 'net::ERR_'


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    """Serve files without writing a log line per request."""

    def log_message(self, message_format: str, *args: object) -> None:
        """Write nothing."""


@contextlib.contextmanager
def served_site() -> Iterator[str]:
    """Serve the built site on a free local port and yield its address."""
    if not (SITE_DIR / 'index.html').exists():
        raise FileNotFoundError(f'{SITE_DIR} has no built site: run mkdocs build --strict first')
    handler = functools.partial(QuietHandler, directory=str(SITE_DIR))
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f'http://127.0.0.1:{server.server_address[1]}'
    finally:
        server.shutdown()
        server.server_close()


@contextlib.contextmanager
def chromium() -> Iterator[Browser]:
    """Start headless Chromium. CHROMIUM names another browser binary than Playwright's own."""
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=os.environ.get('CHROMIUM') or None)
        try:
            yield browser
        finally:
            browser.close()


class PageLog:
    """What a page did while a test used it: its errors, the other hosts and the blocked files."""

    def __init__(self) -> None:
        self.errors: List[str] = []
        self.other_hosts: List[str] = []
        self.blocked: List[str] = []

    def add_page_error(self, error: Error) -> None:
        """Note an error that the page's scripts threw."""
        self.errors.append(str(error))

    def add_console_message(self, message: ConsoleMessage) -> None:
        """Note an error that the page wrote to the console, but not a stopped request's."""
        if message.type in ('error', 'assert') and STOPPED_REQUEST not in message.text:
            self.errors.append(message.text)


class OpenedPage(NamedTuple):
    """A page of the served site in a context of its own, with the log of what it did."""

    context: BrowserContext
    page: Page
    log: PageLog


def open_page(
    browser: Browser,
    base: str,
    path: str,
    width: int = 1280,
    height: int = 900,
    javascript: bool = True,
    blocked: Tuple[str, ...] = (),
    **context_options: Any,
) -> OpenedPage:
    """Open a page of the served site in a new browser context.

    `blocked` names parts of local addresses that fail to load, to test what a figure does
    then. Further options go to the browser context, such as color_scheme or is_mobile.
    """
    context = browser.new_context(
        viewport={'width': width, 'height': height},
        java_script_enabled=javascript,
        **context_options,
    )
    log = PageLog()
    site = urlparse(base)

    def handle_request(route: Route) -> None:
        url = urlparse(route.request.url)
        # The served site only: the same scheme, host and port, not an address that starts alike
        if (url.scheme, url.hostname, url.port) != (site.scheme, site.hostname, site.port):
            log.other_hosts.append(url.netloc)
            route.abort()
        elif any(part in url.path for part in blocked):
            log.blocked.append(url.path)
            route.abort()
        else:
            route.continue_()

    context.route('**/*', handle_request)
    page = context.new_page()
    page.on('pageerror', log.add_page_error)
    page.on('console', log.add_console_message)
    page.goto(base + path, wait_until='load')
    return OpenedPage(context, page, log)
