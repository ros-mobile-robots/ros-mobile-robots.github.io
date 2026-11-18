"""Tests for the figure scripts of the Localization chapter, scripts/figures/localization_*.py.

localization_figures.py writes the figures of the Localization page as twelve includes.
localization_control_test_figures.py and localization_gazebo_test_figures.py write the figures
of the chapter's test pages as eight and three includes.

Run from the repository root: python -m unittest discover -s tests
"""

import collections
import functools
from html.parser import HTMLParser
import importlib.util
import itertools
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from types import ModuleType
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence
from typing import Tuple
import unittest

ROOT = Path(__file__).resolve().parent.parent
FIGURES = ROOT / 'scripts' / 'figures'
# The includes that each script writes
INCLUDE_NAMES = {
    'localization_figures.py': [
        'localization-covariances.html', 'localization-distance.html',
        'localization-fused-values.html', 'localization-loop.html',
        'localization-odom-frame.html', 'localization-stamp-check.html',
        'localization-time-step.html', 'localization-validator-frame.html',
        'localization-validator-order.html', 'localization-validator-time.html',
        'localization-validator-values.html', 'localization-validator-variances.html'],
    'localization_control_test_figures.py': [
        'localization-test-stepped-clock-between-wheels.html',
        'localization-test-stepped-clock-robot-rates.html',
        'localization-test-stepped-clock-step.html', 'localization-test-stepped-clock.html',
        'localization-test-validator-clock-back.html',
        'localization-test-validator-decision.html', 'localization-test-validator-limits.html',
        'localization-test-validator-order.html'],
    'localization_gazebo_test_figures.py': [
        'localization-test-gazebo.html', 'localization-test-odom-tf.html',
        'localization-test-switch.html'],
}
# The includes whose drawings have two layouts, with the number of drawings in each layout: a
# narrow one, and a wide one that the CSS shows where the figure's block is WIDE_FROM wide
TWO_LAYOUT_DRAWINGS = {
    'localization-test-stepped-clock-between-wheels.html': 2,
    'localization-test-stepped-clock-robot-rates.html': 1,
    'localization-test-stepped-clock.html': 1,
    'localization-test-validator-clock-back.html': 2,
    'localization-test-validator-decision.html': 1,
    'localization-test-validator-limits.html': 1,
    'localization-test-validator-order.html': 1,
}
# A viewBox as the scripts write it: the origin, then the width and the height
VIEW_BOX = re.compile(r'0 0 (\d+) (\d+)')
# With these variables, Python's default encoding for a text file is ASCII: the C locale, without
# Python's own switch to UTF-8 for that locale
ASCII_LOCALE = {'LC_ALL': 'C', 'LANG': 'C', 'PYTHONCOERCECLOCALE': '0', 'PYTHONUTF8': '0'}


def run_with_an_ascii_locale(script_name: str) -> Dict[str, bytes]:
    """Run a script with an ASCII locale and return the includes it writes, by file name.

    The script runs in a folder with only a copy of the measured data that the scripts read, so
    the repository's includes stay as they are.
    """
    with tempfile.TemporaryDirectory() as folder:
        includes = Path(folder) / 'includes'
        includes.mkdir()
        shutil.copytree(FIGURES / 'data', Path(folder) / 'scripts' / 'figures' / 'data')
        run = subprocess.run([sys.executable, str(FIGURES / script_name)], cwd=folder,
                             env={**os.environ, **ASCII_LOCALE}, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f'{script_name} failed: {run.stderr.strip()}')
        return {include.name: include.read_bytes() for include in includes.iterdir()}


def include_text(include_name: str) -> str:
    """Return the text of one of the repository's includes."""
    return (ROOT / 'includes' / include_name).read_text(encoding='utf-8')


def localization_includes_by_page() -> Dict[str, List[str]]:
    """Return the Localization includes that each page of the docs includes, by the page."""
    includes: Dict[str, List[str]] = {}
    for page in sorted((ROOT / 'docs').rglob('*.md')):
        found = re.findall(r'--8<-- "includes/(localization-[a-z-]+\.html)"',
                           page.read_text(encoding='utf-8'))
        if found:
            includes[str(page.relative_to(ROOT))] = found
    return includes


def load_script(script_name: str = 'localization_figures.py') -> ModuleType:
    """Return a figure script as a module, without running its main().

    The scripts import localization_figures.py from their own folder.
    """
    if str(FIGURES) not in sys.path:
        sys.path.insert(0, str(FIGURES))
    script = FIGURES / script_name
    specification = importlib.util.spec_from_file_location(script.stem, script)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


class Element(NamedTuple):
    """An HTML or SVG start tag of an include: its name, attributes and enclosing elements."""

    name: str
    attributes: Dict[str, str]
    enclosing: Tuple[Tuple[str, str], ...]  # (name, class) of each enclosing element, outer first


class ElementParser(HTMLParser):
    """Collect the start tags of an include, each with its enclosing elements."""

    VOID_ELEMENTS = {'br', 'hr', 'img', 'input', 'wbr'}  # HTML elements without an end tag

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.elements: List[Element] = []
        self.open: List[Tuple[str, str]] = []

    def handle_starttag(self, tag: str, attrs: List[Tuple[str, Optional[str]]]) -> None:
        self.handle_startendtag(tag, attrs)
        if tag not in self.VOID_ELEMENTS:
            self.open.append((tag, dict(attrs).get('class') or ''))

    def handle_startendtag(self, tag: str, attrs: List[Tuple[str, Optional[str]]]) -> None:
        attributes = {name: value or '' for name, value in attrs}
        self.elements.append(Element(tag, attributes, tuple(self.open)))

    def handle_endtag(self, tag: str) -> None:
        while self.open:
            if self.open.pop()[0] == tag:
                break


def elements_of(include_name: str) -> List[Element]:
    """Return the start tags of one of the repository's includes."""
    parser = ElementParser()
    parser.feed(include_text(include_name))
    parser.close()
    return parser.elements


def drawings_of(include_name: str, layout_class: str) -> List[Element]:
    """Return the SVG drawings of an include that have a layout's class."""
    return [element for element in elements_of(include_name)
            if element.name == 'svg'
            and layout_class in element.attributes.get('class', '').split()]


class Rule(NamedTuple):
    """One selector of a CSS rule of an include, with the rule's declarations."""

    condition: str  # of the @container or @media block around the rule, '' for none
    selector: str  # one selector of the rule's list
    declarations: Dict[str, str]  # the values by property


# A conditional block with its rules, or a rule outside such a block
BLOCK_OR_RULE = re.compile(r'(@[^{]+)\{((?:[^{}]*\{[^}]*\})*)\}|([^{}@]+)\{([^}]*)\}')
RULE = re.compile(r'([^{}]+)\{([^}]*)\}')


SIDES = ('top', 'right', 'bottom', 'left')
BORDER_STYLES = {'none', 'hidden', 'dotted', 'dashed', 'solid', 'double', 'groove', 'ridge',
                 'inset', 'outset'}


def box_sides(values: str) -> Tuple[str, str, str, str]:
    """Return the values of the four sides that one to four values of a box shorthand set."""
    parts = values.split()
    top = parts[0]
    right = parts[1] if len(parts) > 1 else top
    bottom = parts[2] if len(parts) > 2 else top
    left = parts[3] if len(parts) > 3 else right
    return top, right, bottom, left


def border_parts(value: str) -> Dict[str, str]:
    """Return the width, style and color that a border shorthand such as 1px solid red sets."""
    parts = {'width': 'medium', 'style': 'none', 'color': 'currentcolor'}
    colors = []
    for token in value.split():
        if token in BORDER_STYLES:
            parts['style'] = token
        elif re.fullmatch(r'[\d.]+[a-z%]*|thin|medium|thick', token):
            parts['width'] = token
        else:
            colors.append(token)
    if colors:
        parts['color'] = ' '.join(colors)
    return parts


def longhands(name: str, value: str) -> Dict[str, str]:
    """Return the properties that a declaration sets, with a shorthand as its longhands.

    For the shorthands of the includes that have longhands in other rules: margin, padding,
    border, border-top to border-left, border-style, border-width and border-color.
    """
    if name in ('margin', 'padding'):
        return {f'{name}-{side}': part for side, part in zip(SIDES, box_sides(value))}
    if name in ('border-style', 'border-width', 'border-color'):
        part_name = name.split('-')[1]
        return {f'border-{side}-{part_name}': part for side, part in zip(SIDES, box_sides(value))}
    if name == 'border' or name in (f'border-{side}' for side in SIDES):
        sides = SIDES if name == 'border' else (name.split('-')[1],)
        parts = border_parts(value)
        return {f'border-{side}-{part_name}': part for side in sides
                for part_name, part in parts.items()}
    return {name: value}


def declarations_of(text: str) -> Dict[str, str]:
    """Return the values of a rule's declarations, such as max-width:20rem, by longhand."""
    values: Dict[str, str] = {}
    for declaration in text.split(';'):
        if declaration.strip():
            name, value = declaration.split(':', 1)
            values.update(longhands(name.strip(), value.strip()))
    return values


def css_style(include_name: str) -> str:
    """Return the text of an include's style element."""
    style = re.search(r'<style>(.*?)</style>', include_text(include_name), re.DOTALL)
    assert style is not None, include_name
    return style.group(1)


# What a selector may name around an include: the theme's block of the page's text, and in the
# dark color scheme the page's body. The only attribute selector that the test reads is the
# dark scheme's, with its value quoted or not
TEXT_BLOCK = '.md-typeset'
DARK_SCHEME = '[data-md-color-scheme=slate]'
QUOTED_ATTRIBUTE_VALUE = re.compile(r'\[([\w-]+)=(["\']?)([\w-]+)\2\]')


def checked_selector(selector: str) -> str:
    """Return a selector with its attribute values unquoted, or raise for one the test can't read.

    The only attribute selector that the test reads is DARK_SCHEME as the first compound: the
    page's body around the include.
    """
    unquoted = QUOTED_ATTRIBUTE_VALUE.sub(r'[\1=\3]', selector.strip())
    for position, compound in enumerate(unquoted.split()):
        if '[' in compound and (compound != DARK_SCHEME or position > 0):
            raise ValueError(f'the test reads no attribute selector such as {compound} in '
                             f'{selector}')
    return unquoted


def css_rules(include_name: str) -> List[Rule]:
    """Return the rules of an include's style element in their order, one per selector."""
    rules: List[Rule] = []
    for block in BLOCK_OR_RULE.finditer(css_style(include_name)):
        condition, inner, selectors, declarations = block.groups()
        pairs = RULE.findall(inner) if condition else [(selectors, declarations)]
        for selector_list, text in pairs:
            rules.extend(Rule((condition or '').strip(), checked_selector(selector),
                              declarations_of(text))
                         for selector in selector_list.split(','))
    return rules


def specificity(selector: str) -> Tuple[int, int]:
    """Return a selector's numbers of classes and of element names, as CSS compares them.

    For one selector of the includes: classes, attribute selectors and :not() of a class count
    as classes, and the includes have no ids.
    """
    classes = len(re.findall(r'\.[\w-]+|\[[^\]]+\]', selector))
    names = len(re.findall(r'(?:^|\s)[a-z][\w-]*', selector.strip()))
    return classes, names


# The simple selectors of a compound one, such as figure.lv-figure:not(.lv-drawing)
SIMPLE_SELECTOR = re.compile(r'[a-z][\w-]*|\.[\w-]+|:not\(\.[\w-]+\)|\[[^\]]+\]')


def compound_matches(compound: str, name: str, classes: str) -> bool:
    """Return whether a compound selector matches an element of a name with classes."""
    simple_selectors = SIMPLE_SELECTOR.findall(compound)
    if ''.join(simple_selectors) != compound:
        raise ValueError(f'the test reads no selector such as {compound}')
    class_set = set(classes.split())
    for simple in simple_selectors:
        if simple.startswith(':not('):
            matches = simple[len(':not(.'):-1] not in class_set
        elif simple.startswith('.'):
            matches = simple[1:] in class_set
        elif simple.startswith('['):
            raise ValueError(f'the test reads no attribute selector such as {simple}')
        else:
            matches = simple == name
        if not matches:
            return False
    return True


class ElementKey(NamedTuple):
    """What a selector can match of an element: its name, its classes and enclosing elements."""

    name: str
    classes: str
    enclosing: Tuple[Tuple[str, str], ...]


def key_of(element: Element) -> ElementKey:
    """Return what a selector can match of an element."""
    return ElementKey(element.name, element.attributes.get('class', ''), element.enclosing)


@functools.lru_cache(maxsize=None)
def selector_matches(selector: str, element: ElementKey, dark: bool) -> bool:
    """Return whether a selector of compounds and descendant combinators matches an element.

    The include stands in TEXT_BLOCK, and in DARK_SCHEME when dark is true. The selector comes
    from css_rules(), with its attribute values unquoted.
    """
    *outer, subject = selector.split()
    if not compound_matches(subject, element.name, element.classes):
        return False
    enclosing = list(element.enclosing)
    for compound in reversed(outer):
        if compound == DARK_SCHEME and not dark:
            return False
        if compound in (TEXT_BLOCK, DARK_SCHEME):
            enclosing = []  # the page around: no element of the include is outside it
            continue
        while enclosing:
            name, classes = enclosing.pop()
            if compound_matches(compound, name, classes):
                break
        else:
            return False
    return True


class Declaration(NamedTuple):
    """The value that a rule gives a property, with the rule's specificity."""

    specificity: Tuple[int, int]
    value: str


def winning_declarations(rules: Sequence[Rule], element: ElementKey,
                         dark: bool) -> Dict[str, Declaration]:
    """Return the declarations that win for an element, by property.

    The most specific rule wins, and of equally specific rules the last one.
    """
    winners: Dict[str, Declaration] = {}
    for rule in rules:
        if selector_matches(rule.selector, element, dark):
            rank = specificity(rule.selector)
            for name, value in rule.declarations.items():
                if name not in winners or rank >= winners[name].specificity:
                    winners[name] = Declaration(rank, value)
    return winners


def changes_by_added_rules(page: str, include_names: Sequence[str],
                           added_names: Sequence[str]) -> List[str]:
    """Return the values that the rules of an added include would change on a page.

    The added rules come after the page's rules. Of rules with the same specificity the last
    one wins, so an added rule changes an element where it gives a property another value with
    the same specificity as the page's winning rule or a higher one. In a narrow and a wide
    block for the layouts, and in both color schemes.
    """
    elements = {key_of(element) for include_name in include_names
                for element in elements_of(include_name)}
    changes = []
    for block_is_wide, dark in itertools.product((False, True), (False, True)):
        def applies(rule: Rule) -> bool:
            return rule.condition == '' or (block_is_wide
                                            and rule.condition.startswith('@container'))
        page_rules = [rule for include_name in include_names
                      for rule in css_rules(include_name) if applies(rule)]
        added_rules = {name: [rule for rule in css_rules(name) if applies(rule)]
                       for name in added_names}
        for element in sorted(elements):
            page_values = winning_declarations(page_rules, element, dark)
            for added_name, rules in added_rules.items():
                for name, added in winning_declarations(rules, element, dark).items():
                    before = page_values.get(name)
                    if before is None or (added.specificity >= before.specificity
                                          and added.value != before.value):
                        changes.append(f'{page} with {added_name}: <{element.name} class="'
                                       f'{element.classes}"> {name}: '
                                       f'{before.value if before else "unset"} -> '
                                       f'{added.value}')
    return changes


class TestIncludes(unittest.TestCase):
    """Each script writes the repository's includes, whatever the locale's encoding is.

    The includes have characters outside ASCII, such as the Greek letters of the symbols.
    """

    def assert_script_writes_the_repository_includes(self, script_name: str) -> None:
        """Assert that a script, run with an ASCII locale, writes its includes byte for byte."""
        written = run_with_an_ascii_locale(script_name)
        self.assertEqual(sorted(written), INCLUDE_NAMES[script_name])
        for include_name in INCLUDE_NAMES[script_name]:
            committed = (ROOT / 'includes' / include_name).read_bytes()
            self.assertEqual(written[include_name], committed,
                             f'{include_name} is not what {script_name} writes')

    def test_script_writes_the_repository_includes_with_an_ascii_locale(self) -> None:
        self.assert_script_writes_the_repository_includes('localization_figures.py')

    def test_control_test_script_writes_the_repository_includes_with_an_ascii_locale(
            self) -> None:
        self.assert_script_writes_the_repository_includes(
            'localization_control_test_figures.py')

    def test_gazebo_test_script_writes_the_repository_includes_with_an_ascii_locale(
            self) -> None:
        self.assert_script_writes_the_repository_includes('localization_gazebo_test_figures.py')


class TestLayouts(unittest.TestCase):
    """A figure with two layouts has both drawings, and a page's drawings have their own ids.

    The CSS shows one layout of a drawing and hides the other one with display:none. Both are in
    the page, so an id of the narrow drawing must not appear in the wide one.
    """

    def test_two_layout_figure_stands_in_the_block_that_the_css_measures(self) -> None:
        figures = load_script()
        for include_name in TWO_LAYOUT_DRAWINGS:
            figure_elements = [element for element in elements_of(include_name)
                               if element.name == 'figure']
            self.assertEqual(len(figure_elements), 1, include_name)
            figure = figure_elements[0]
            self.assertEqual(figure.attributes['class'], 'lv-figure lv-large lv-layouts',
                             include_name)
            self.assertEqual(figure.enclosing, (('div', figures.LAYOUT_SPACE),), include_name)
            block_rules = [rule for rule in css_rules(include_name)
                           if rule.selector == f'.md-typeset .{figures.LAYOUT_SPACE}']
            self.assertEqual([rule.declarations.get('container-type') for rule in block_rules],
                             ['inline-size'], include_name)

    def test_css_shows_one_layout_on_each_side_of_the_threshold(self) -> None:
        """The narrow drawings show in a narrower block, and the wide ones from WIDE_FROM on.

        Each drawing's display comes from the rules that match it on that side of the
        threshold. Without such a rule an SVG element shows.
        """
        figures = load_script()
        wide_block = f'@container (min-width:{figures.WIDE_FROM})'
        for include_name, dark in itertools.product(TWO_LAYOUT_DRAWINGS, (False, True)):
            for block_is_wide, shown in ((False, figures.NARROW_LAYOUT),
                                         (True, figures.WIDE_LAYOUT)):
                rules = [rule for rule in css_rules(include_name) if rule.condition == ''
                         or (block_is_wide and rule.condition == wide_block)]
                for layout in (figures.NARROW_LAYOUT, figures.WIDE_LAYOUT):
                    for drawing in drawings_of(include_name, layout):
                        winners = winning_declarations(rules, key_of(drawing), dark)
                        display = winners['display'].value if 'display' in winners else 'inline'
                        self.assertEqual(display != 'none', layout == shown,
                                         f'{include_name}: {layout}, wide block {block_is_wide}')

    def test_two_layout_include_has_its_drawings_in_both_layouts(self) -> None:
        figures = load_script()
        for include_name, count in TWO_LAYOUT_DRAWINGS.items():
            self.assertEqual(len(drawings_of(include_name, figures.NARROW_LAYOUT)), count,
                             include_name)
            self.assertEqual(len(drawings_of(include_name, figures.WIDE_LAYOUT)), count,
                             include_name)

    def test_layout_drawings_have_the_layouts_widths(self) -> None:
        control = load_script('localization_control_test_figures.py')
        widths = {control.NARROW_LAYOUT: control.DRAWING_WIDTH,
                  control.WIDE_LAYOUT: control.WIDE_WIDTH}
        for include_name in TWO_LAYOUT_DRAWINGS:
            for layout_class, width in widths.items():
                for drawing in drawings_of(include_name, layout_class):
                    view_box = VIEW_BOX.fullmatch(drawing.attributes.get('viewbox', ''))
                    self.assertIsNotNone(view_box, f'{include_name}: {drawing.attributes}')
                    assert view_box is not None
                    self.assertEqual(int(view_box.group(1)), width, include_name)
                    self.assertGreater(int(view_box.group(2)), 0, include_name)

    def test_one_layout_include_has_no_drawing_of_a_layout(self) -> None:
        figures = load_script()
        all_names = [name for names in INCLUDE_NAMES.values() for name in names]
        for include_name in sorted(set(all_names) - set(TWO_LAYOUT_DRAWINGS)):
            text = include_text(include_name)
            self.assertFalse(figures.NARROW_LAYOUT in text, include_name)
            self.assertFalse(figures.WIDE_LAYOUT in text, include_name)
            self.assertFalse(figures.LAYOUT_SPACE in text, include_name)

    def test_wide_cap_outranks_the_other_caps_of_a_two_layout_figure(self) -> None:
        """The largest width of a figure with two layouts wins whatever the rules' order.

        Over a link, the theme shows a preview of the linked section and adds that section's
        style element after the page's own. Its rules then come later than the page's.
        """
        all_names = [name for names in INCLUDE_NAMES.values() for name in names]
        figure_classes = {'lv-figure', 'lv-large', 'lv-layouts'}
        caps = set()
        for include_name in all_names:
            for rule in css_rules(include_name):
                figure = re.search(r'figure((?:\.[\w-]+)+)', rule.selector)
                if (figure and 'max-width' in rule.declarations
                        and set(figure.group(1).split('.')[1:]) <= figure_classes):
                    caps.add(rule.selector)
        wide_caps = {selector for selector in caps if 'lv-layouts' in selector}
        self.assertEqual(len(wide_caps), 1)
        wide_cap = wide_caps.pop()
        self.assertGreater(len(caps), 1)
        for selector in caps - {wide_cap}:
            self.assertGreater(specificity(wide_cap), specificity(selector), selector)

    def test_rules_of_any_include_added_to_a_page_change_none_of_its_elements(self) -> None:
        """A previewed section's style element leaves the page's figures as they are.

        Over a link, the theme shows a preview of the linked section, also of another page, and
        adds that section's style element after the page's own. The test adds each include's
        rules in turn after each page's rules.
        """
        all_names = [name for names in INCLUDE_NAMES.values() for name in names]
        # The includes with the same style element add the same rules
        added_names = list({css_style(name): name for name in all_names}.values())
        changes = [change for page, include_names in localization_includes_by_page().items()
                   for change in changes_by_added_rules(page, include_names, added_names)]
        self.assertEqual(changes, [])

    def test_ids_are_unique_on_each_page(self) -> None:
        pages = localization_includes_by_page()
        self.assertTrue(pages)
        for page, include_names in pages.items():
            ids = [found for include_name in include_names
                   for found in re.findall(r'\bid="([^"]+)"', include_text(include_name))]
            repeated = [name for name, count in collections.Counter(ids).items() if count > 1]
            self.assertEqual(repeated, [], f'{page} has an id more than once')


class TestDecisionSteps(unittest.TestCase):
    """The two layouts of the validator's decision break the same text of a step into lines."""

    def test_both_layouts_have_the_same_words(self) -> None:
        control = load_script('localization_control_test_figures.py')
        for step in control.DECISION_STEPS:
            self.assertEqual(' '.join(step.lines), ' '.join(step.wide_lines), step.reason)


class TestStretch(unittest.TestCase):
    """stretched() multiplies the x positions and lengths of an element, and nothing else."""

    def test_rectangle_in_a_group_gets_its_position_and_width_stretched(self) -> None:
        figures = load_script()
        interval = '<g><title>interval</title><rect x="18" y="4" width="20" height="6"/></g>'
        self.assertEqual(
            figures.stretched(interval, 1.8),
            '<g><title>interval</title><rect x="32.4" y="4" width="36.0" height="6"/></g>')

    def test_path_with_a_leading_decimal_point_gets_its_x_positions_stretched(self) -> None:
        figures = load_script()
        self.assertEqual(figures.stretched_path('M.5,10 H10 v4 h-2', 1.8),
                         'M0.9,10 H18.0 v4 h-2')

    def test_rectangle_keeps_the_width_of_its_outline(self) -> None:
        figures = load_script()
        self.assertEqual(figures.stretched('<rect x="10" width="8" stroke-width="2"/>', 1.8),
                         '<rect x="18.0" width="14.4" stroke-width="2"/>')

    def test_path_with_a_trailing_decimal_point_gets_its_x_positions_stretched(self) -> None:
        figures = load_script()
        self.assertEqual(figures.stretched_path('M1.,2 H5', 1.8), 'M1.8,2 H9.0')

    def test_group_shifted_vertically_gets_its_elements_stretched(self) -> None:
        figures = load_script()
        self.assertEqual(
            figures.stretched('<g transform="translate(0,20)"><circle cx="4" r="2"/></g>', 1.8),
            '<g transform="translate(0,20)"><circle cx="7.2" r="2"/></g>')

    def test_mark_keeps_its_size(self) -> None:
        figures = load_script()
        self.assertEqual(figures.stretched('<circle cx="10" cy="5" r="3"/>', 2),
                         '<circle cx="20.0" cy="5" r="3"/>')

    def test_element_with_a_position_that_is_not_stretched_is_rejected(self) -> None:
        figures = load_script()
        for element in ('<polyline points="0,0 10,10"/>', '<line x1="0" x2="10"/>',
                        '<g transform="translate(5,0)"><circle cx="1" r="1"/></g>',
                        '<g transform="translate(0,20) scale(2)"><circle cx="1" r="1"/></g>',
                        '<path d="M0,0 C1,1 2,2 3,3"/>'):
            with self.assertRaises(ValueError, msg=element):
                figures.stretched(element, 1.8)


class TestLoopFigure(unittest.TestCase):
    """The loop figure's two variants differ only above the filter's inputs.

    The page shows the variants in two tabs and says that the real robot and the simulation
    share everything from the drive controller and the IMU sample validator on.
    """

    def test_loop_variants_match_from_measurement_arrows_on(self) -> None:
        figures = load_script()
        robot = figures.loop_figure(figures.REAL_ROBOT)
        simulation = figures.loop_figure(figures.SIMULATION)
        # What both variants are to share, drawn alone: it starts with the two measurements'
        # arrows into the EKF and ends with the boxes that send velocity commands
        shared = figures.Figure('shared', 'The part that both variants share', robot.height,
                                robot.width)
        figures.filter_and_way_back(shared, figures.NODES_Y + figures.NODES_HEIGHT)
        self.assertLess(len(shared.elements), len(robot.elements))
        self.assertLess(len(shared.elements), len(simulation.elements))
        self.assertEqual(robot.elements[-len(shared.elements):], shared.elements)
        self.assertEqual(simulation.elements[-len(shared.elements):], shared.elements)


if __name__ == '__main__':
    unittest.main()
