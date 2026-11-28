"""Helpers for the browser tests: serve the built site and open its pages in headless Chromium.

A test page only talks to the local server. Requests to any other host are stopped and
recorded, so a test run never loads an ad, the consent message or Analytics, and a test can
check which hosts a page asks for. Any other request that fails counts as an error of the page.
"""

import contextlib
import functools
import http.server
import os
from pathlib import Path
import sys
import threading
from typing import Any
from typing import Dict
from typing import Iterator
from typing import List
from typing import NamedTuple
from typing import Set
from typing import Tuple
from urllib.parse import urlparse

from playwright.sync_api import Browser
from playwright.sync_api import BrowserContext
from playwright.sync_api import ConsoleMessage
from playwright.sync_api import Error
from playwright.sync_api import Page
from playwright.sync_api import Request
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
# How Chromium starts its console line about a resource that failed to load. For a request that
# the test stopped, this line isn't the page's fault
FAILED_RESOURCE = 'Failed to load resource'
# The error of a request that the test stops: route.abort() without an error code
STOPPED_REQUEST_ERROR = 'net::ERR_FAILED'


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
    """What a page did while a test used it: its errors, the other hosts and the blocked files.

    errors holds the exceptions of the page's scripts, the errors that it wrote to the console,
    and each request that failed although the test didn't stop it, with its address and reason.
    """

    def __init__(self) -> None:
        """Initialize the page's request and error logs."""
        self.errors: List[str] = []
        self.other_hosts: List[str] = []
        self.blocked: List[str] = []
        self.stopped_requests: Set[Request] = set()
        # Per address, the browser's console lines still due for requests that the test stopped
        self.stopped_console_lines: Dict[str, int] = {}

    def add_stopped_request(self, request: Request) -> None:
        """Note a request that the test stops on purpose: its failure is no error."""
        self.stopped_requests.add(request)
        due = self.stopped_console_lines.get(request.url, 0)
        self.stopped_console_lines[request.url] = due + 1

    def add_page_error(self, error: Error) -> None:
        """Note an error that the page's scripts threw."""
        self.errors.append(str(error))

    def add_failed_request(self, request: Request) -> None:
        """Note a request that failed, unless the test stopped this very request.

        Another request to the same address, a retry for example, is still an error.
        """
        if request in self.stopped_requests:
            self.stopped_requests.discard(request)
            return
        self.errors.append(f'{request.url}: {request.failure}')

    def add_console_message(self, message: ConsoleMessage) -> None:
        """Note an error that the page wrote to the console.

        The browser's own line about a request that the test stopped isn't the page's fault: each
        stopped request excuses one such line at its address, with exactly the error of
        route.abort() and no arguments.
        """
        if message.type not in ('error', 'assert'):
            return
        url = message.location.get('url', '')
        # The browser's own line has exactly this text and no arguments. A page's console call
        # has arguments, also when it quotes the same text
        about_stopped_request = (
            message.type == 'error'
            and message.text == f'{FAILED_RESOURCE}: {STOPPED_REQUEST_ERROR}'
            and not message.args
            and self.stopped_console_lines.get(url, 0) > 0
        )
        if about_stopped_request:
            self.stopped_console_lines[url] -= 1
            return
        self.errors.append(message.text)


class OpenedPage(NamedTuple):
    """A page of the served site in a context of its own, with the log of what it did."""

    context: BrowserContext
    page: Page
    log: PageLog


def route_requests(
    context: BrowserContext, base: str, blocked: Tuple[str, ...], log: PageLog
) -> None:
    """Let a context load the served site only, without the local files that a test blocks."""
    site = urlparse(base)

    def route_test_request(route: Route) -> None:
        """Allow same-origin assets and record requests deliberately blocked by the test."""
        url = urlparse(route.request.url)
        # The served site only: the same scheme, host and port, not an address that starts alike
        if (url.scheme, url.hostname, url.port) != (site.scheme, site.hostname, site.port):
            log.other_hosts.append(url.netloc)
            log.add_stopped_request(route.request)
            route.abort()
        elif any(part in url.path for part in blocked):
            log.blocked.append(url.path)
            log.add_stopped_request(route.request)
            route.abort()
        else:
            route.continue_()

    context.route('**/*', route_test_request)


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
    then. Further options go to the browser context, such as color_scheme or is_mobile. If the
    page fails to open, the context is closed again before the error goes on.
    """
    context = browser.new_context(
        viewport={'width': width, 'height': height},
        java_script_enabled=javascript,
        **context_options,
    )
    try:
        log = PageLog()
        route_requests(context, base, blocked, log)
        page = context.new_page()
        page.on('pageerror', log.add_page_error)
        page.on('console', log.add_console_message)
        page.on('requestfailed', log.add_failed_request)
        page.goto(base + path, wait_until='load')
    except BaseException:
        context.close()
        raise
    return OpenedPage(context, page, log)
