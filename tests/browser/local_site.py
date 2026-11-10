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
import threading
from typing import Iterator, List, Tuple
from urllib.parse import urlparse

from playwright.sync_api import Browser, BrowserContext, Page, sync_playwright

ROOT = Path(__file__).resolve().parent.parent.parent
# The built site: mkdocs build --strict writes it to site/. SITE_DIR names another folder
SITE_DIR = Path(os.environ.get('SITE_DIR', ROOT / 'site'))
# The hosts that the site's own template asks for on every page: the ad and consent script, and
# the repository's star count in the header. A figure must not add a host to them
SITE_HOSTS = {'pagead2.googlesyndication.com', 'api.github.com'}


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    """Serve files without writing a log line per request."""

    def log_message(self, format: str, *args: object) -> None:
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
def chromium(webgl: bool = False) -> Iterator[Browser]:
    """Start headless Chromium. CHROMIUM names another browser binary than Playwright's own."""
    arguments = ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] if webgl else []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=os.environ.get('CHROMIUM') or None, args=arguments
        )
        try:
            yield browser
        finally:
            browser.close()


class PageLog:
    """What a page did while a test used it: its errors and the hosts it asked for."""

    def __init__(self) -> None:
        self.errors: List[str] = []
        self.other_hosts: List[str] = []
        self.requests: List[str] = []


def open_page(
    browser: Browser,
    base: str,
    path: str,
    width: int = 1280,
    height: int = 900,
    javascript: bool = True,
    blocked: Tuple[str, ...] = (),
    **context_options: object,
) -> Tuple[BrowserContext, Page, PageLog]:
    """Open a page of the served site and return its context, the page and its log.

    `blocked` names parts of local addresses that fail to load, to test what a figure does
    then. Further options go to the browser context, such as color_scheme or is_mobile.
    """
    context = browser.new_context(
        viewport={'width': width, 'height': height},
        java_script_enabled=javascript,
        **context_options,
    )
    log = PageLog()

    def route(route_) -> None:
        url = route_.request.url
        if not url.startswith(base):
            log.other_hosts.append(urlparse(url).netloc)
            route_.abort()
        elif any(part in url for part in blocked):
            route_.abort()
        else:
            log.requests.append(url[len(base):])
            route_.continue_()

    context.route('**/*', route)
    page = context.new_page()
    page.on('pageerror', lambda error: log.errors.append(str(error)))
    # A request that this helper stopped also logs a console error, which isn't the page's fault
    page.on(
        'console',
        lambda message: (
            log.errors.append(message.text)
            if message.type in ('error', 'assert') and 'net::ERR_' not in message.text
            else None
        ),
    )
    page.goto(base + path, wait_until='load')
    return context, page, log
