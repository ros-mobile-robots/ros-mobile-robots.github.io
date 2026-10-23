"""The parts lists on the Components page, generated from includes/parts.yml.

Each part and its store links are written once in the catalog; a robot's list (a configuration)
names the parts it uses. A page shows a list with {{ parts_list("remo") }}.

Every build checks the whole catalog first, including parts that no list shows yet, and stops
with the place of each mistake (includes/parts.yml: configurations.remo.entries[3].part: ...).
The strict build alone wouldn't notice some of them: a link reference that includes/links.md
doesn't define stays on the page as literal text.
"""
import re
from pathlib import Path

import yaml
from mkdocs.exceptions import PluginError

CATALOG = 'includes/parts.yml'
LINKS = 'includes/links.md'

FIELDS = {
    'catalog': ({'parts', 'configurations'}, set()),
    'part': ({'name'}, {'offers'}),
    'offer': ({'label'}, {'ref', 'url'}),
    'configuration': ({'entries'}, set()),
    'entry': ({'part', 'quantity'}, {'legacy_price', 'note'}),
}
ID = re.compile(r'[a-z0-9][a-z0-9.-]*\Z')
DEFINITION = re.compile(r'\[([^\]]+)\]:\s')
REFERENCE_LINK = re.compile(r'\[([^\]]*)\]\[([^\]]*)\]')
INLINE_LINK = re.compile(r'\]\(([^)\s]*)')
EXTERNAL_OR_DOCS_ROOT = re.compile(r'[a-z][a-z0-9+.-]*:|/')

HEADER = ('| **Part**                | **Quantity** | **Cost** | **Store** | **Notes** |\n'
          '|:------------------------|:------------:| --- |  --- |  --- |')


class CatalogError(PluginError):
    pass


class _Loader(yaml.SafeLoader):
    """Stops at a key written twice, which PyYAML would otherwise silently overwrite."""


def _mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise CatalogError(f'{CATALOG}:{key_node.start_mark.line + 1}: {key!r} is written twice')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def load_catalog(text):
    return yaml.load(text, Loader=_Loader)


def link_references(text):
    """The references that includes/links.md defines, in lowercase as Markdown compares them,
    and the errors for references defined twice."""
    references, errors = set(), []
    for number, line in enumerate(text.splitlines(), 1):
        match = DEFINITION.match(line)
        if match:
            reference = match[1].lower()
            if reference in references:
                errors.append(f'{LINKS}:{number}: [{match[1]}] is defined twice')
            references.add(reference)
    return references, errors


def validate(catalog, references):
    """All mistakes in the catalog, each with its place in the file."""
    errors = []

    def error(path, message):
        errors.append(f'{CATALOG}: {path}: {message}')

    def fields(value, kind, path):
        if not isinstance(value, dict):
            error(path, 'expected a mapping')
            return False
        required, optional = FIELDS[kind]
        for key in sorted(value.keys() - required - optional, key=str):
            error(f'{path}.{key}' if path else key, 'unknown field')
        for key in sorted(required - value.keys()):
            error(path or 'top level', f'{key} is missing')
        return True

    def text(value, path, links=False):
        if not isinstance(value, str) or not value.strip():
            error(path, 'expected text')
            return
        if '|' in value or '\n' in value:
            error(path, 'a table cell can hold neither "|" nor a line break')
        if not links:
            if '[' in value or ']' in value:
                error(path, 'expected text without [ or ]')
            return
        for label, reference in REFERENCE_LINK.findall(value):
            if (reference or label).lower() not in references:
                error(path, f'[{reference or label}] is not defined in {LINKS}')
        for target in INLINE_LINK.findall(value):
            if not EXTERNAL_OR_DOCS_ROOT.match(target):
                error(path, f'page links start at docs/, so they work on any page: /{target}')

    if not fields(catalog, 'catalog', ''):
        return errors
    parts = catalog.get('parts')
    if not isinstance(parts, dict):
        error('parts', 'expected a mapping')
        parts = {}
    for part_id, part in parts.items():
        path = f'parts.{part_id}'
        if not isinstance(part_id, str) or not ID.match(part_id):
            error(path, 'an id has lowercase letters, digits, "-" and "."')
        if not fields(part, 'part', path):
            continue
        if 'name' in part:
            text(part['name'], f'{path}.name', links=True)
        offers = part.get('offers', [])
        if not isinstance(offers, list):
            error(f'{path}.offers', 'expected a list')
            continue
        for number, offer in enumerate(offers):
            offer_path = f'{path}.offers[{number}]'
            if not fields(offer, 'offer', offer_path):
                continue
            if 'label' in offer:
                text(offer['label'], f'{offer_path}.label')
            if ('ref' in offer) == ('url' in offer):
                error(offer_path, 'needs either ref or url, not both')
            elif 'ref' in offer:
                if not isinstance(offer['ref'], str) or offer['ref'].lower() not in references:
                    error(f'{offer_path}.ref', f'{offer["ref"]!r} is not defined in {LINKS}')
            elif not isinstance(offer['url'], str) or not offer['url'].startswith('https://'):
                error(f'{offer_path}.url', 'expected an https:// link')

    configurations = catalog.get('configurations')
    if not isinstance(configurations, dict):
        error('configurations', 'expected a mapping')
        configurations = {}
    for name, configuration in configurations.items():
        path = f'configurations.{name}'
        if not fields(configuration, 'configuration', path):
            continue
        entries = configuration.get('entries')
        if not isinstance(entries, list) or not entries:
            error(f'{path}.entries', 'expected a list of parts')
            continue
        for number, entry in enumerate(entries):
            entry_path = f'{path}.entries[{number}]'
            if not fields(entry, 'entry', entry_path):
                continue
            if 'part' in entry and (not isinstance(entry['part'], str) or entry['part'] not in parts):
                error(f'{entry_path}.part', f'there is no part {entry["part"]!r}')
            quantity = entry.get('quantity')
            if 'quantity' in entry and (type(quantity) is not int or quantity < 1):
                error(f'{entry_path}.quantity', 'expected a whole number of at least 1')
            if 'legacy_price' in entry:
                text(entry['legacy_price'], f'{entry_path}.legacy_price')
            if 'note' in entry:
                text(entry['note'], f'{entry_path}.note', links=True)
    return errors


def _link(offer):
    if 'ref' in offer:
        return f'[{offer["label"]}][{offer["ref"]}]'
    return f'[{offer["label"]}]({offer["url"]})'


def render_table(catalog, configuration):
    """A configuration's parts as a Markdown table, in the order of its entries."""
    configurations = catalog['configurations']
    if configuration not in configurations:
        raise CatalogError(f'{CATALOG}: there is no configuration {configuration!r}')
    rows = [HEADER]
    for entry in configurations[configuration]['entries']:
        part = catalog['parts'][entry['part']]
        cells = [
            part['name'],
            str(entry['quantity']),
            entry.get('legacy_price', '--'),
            ', '.join(_link(offer) for offer in part.get('offers', [])),
            entry.get('note', ''),
        ]
        rows.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(rows)


def define_env(env):
    root = Path(env.project_dir)
    catalog = load_catalog((root / CATALOG).read_text(encoding='utf-8'))
    references, errors = link_references((root / LINKS).read_text(encoding='utf-8'))
    errors += validate(catalog, references)
    if errors:
        raise CatalogError('\n'.join(errors))

    @env.macro
    def parts_list(configuration):
        """The parts list of one robot (a configuration in includes/parts.yml) as a table."""
        return render_table(catalog, configuration)
