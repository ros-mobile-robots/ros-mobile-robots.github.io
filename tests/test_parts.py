"""Tests for macros/parts.py, the parts lists from includes/parts.yml.

Run from the repository root: python -m unittest discover -s tests
"""
import sys
import unittest
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'macros'))

import parts  # noqa: E402

REFERENCES = {'shop_us', 'shop_ger'}


def catalog(**entry):
    """A small valid catalog; keyword arguments replace fields of its one entry."""
    return {
        'parts': {
            'motor': {
                'name': 'Motor',
                'offers': [
                    {'label': 'Amazon.com', 'ref': 'shop_us'},
                    {'label': 'Maker', 'url': 'https://example.com/motor'},
                ],
            },
            'breadboard': {'name': 'Breadboard'},
        },
        'configurations': {
            'robot': {
                'entries': [
                    {'part': 'motor', 'quantity': 2, 'legacy_price': '$12.50', **entry},
                    {'part': 'breadboard', 'quantity': 1},
                ],
            },
        },
    }


class ValidateTest(unittest.TestCase):

    def errors(self, data):
        return parts.validate(data, REFERENCES)

    def assertError(self, data, expected):
        errors = self.errors(data)
        self.assertTrue(any(expected in error for error in errors), errors)

    def test_the_repository_catalog_is_valid(self):
        data = parts.load_catalog((ROOT / parts.CATALOG).read_text(encoding='utf-8'))
        references, errors = parts.link_references((ROOT / parts.LINKS).read_text(encoding='utf-8'))
        self.assertEqual(errors + parts.validate(data, references), [])

    def test_a_valid_catalog_has_no_errors(self):
        self.assertEqual(self.errors(catalog()), [])

    def test_unknown_part(self):
        self.assertError(catalog(part='motr'),
                         "configurations.robot.entries[0].part: there is no part 'motr'")

    def test_unknown_reference(self):
        data = catalog()
        data['parts']['motor']['offers'][0]['ref'] = 'shop_usa'
        self.assertError(data, "parts.motor.offers[0].ref: 'shop_usa' is not defined")

    def test_references_compare_without_case(self):
        data = catalog()
        data['parts']['motor']['offers'][0]['ref'] = 'Shop_US'
        self.assertEqual(self.errors(data), [])

    def test_offer_needs_either_ref_or_url(self):
        data = catalog()
        data['parts']['motor']['offers'][0]['url'] = 'https://example.com'
        del data['parts']['motor']['offers'][1]['url']
        self.assertError(data, 'parts.motor.offers[0]: needs either ref or url, not both')
        self.assertError(data, 'parts.motor.offers[1]: needs either ref or url, not both')

    def test_url_must_be_https(self):
        data = catalog()
        data['parts']['motor']['offers'][1]['url'] = 'http://example.com'
        self.assertError(data, 'parts.motor.offers[1].url: expected an https:// link')

    def test_url_cannot_break_the_link_or_the_table(self):
        for url in ('https://example.com/item|extra-cell', 'https://example.com/a\nb',
                    'https://example.com/a\rb', 'https://example.com/a b',
                    'https://example.com/a)b', 'https://example.com/a]b'):
            with self.subTest(url=url):
                data = catalog()
                data['parts']['motor']['offers'][1]['url'] = url
                self.assertError(data, 'parts.motor.offers[1].url: expected an https:// link')

    def test_quantity_must_be_a_whole_number_of_at_least_one(self):
        for quantity in (0, -1, 1.5, '2', True, None):
            with self.subTest(quantity=quantity):
                self.assertError(catalog(quantity=quantity),
                                 'configurations.robot.entries[0].quantity: expected a whole number')

    def test_unknown_and_missing_fields(self):
        data = catalog(quantiy=2)
        del data['configurations']['robot']['entries'][0]['quantity']
        self.assertError(data, 'configurations.robot.entries[0].quantiy: unknown field')
        self.assertError(data, 'configurations.robot.entries[0]: quantity is missing')

    def test_note_reference_must_exist(self):
        self.assertError(catalog(note='see [this][nowhere]'),
                         'configurations.robot.entries[0].note: [nowhere] is not defined')
        self.assertEqual(self.errors(catalog(note='see [this][shop_ger]')), [])

    def test_note_page_links_start_at_docs(self):
        self.assertError(catalog(note='see [step 1](hardware_setup/assembly.md)'),
                         'page links start at docs/, so they work on any page: '
                         '/hardware_setup/assembly.md')
        self.assertEqual(self.errors(catalog(note='see [step 1](/hardware_setup/assembly.md)')), [])
        self.assertEqual(self.errors(catalog(note='see [SBC](https://en.wikipedia.org/)')), [])

    def test_cells_cannot_hold_a_pipe_or_line_break(self):
        self.assertError(catalog(note='a | b'), 'a table cell can hold neither')
        self.assertError(catalog(note='a\nb'), 'a table cell can hold neither')
        self.assertError(catalog(note='a\rb'), 'a table cell can hold neither')

    def test_part_ids(self):
        data = catalog()
        data['parts']['Motor 2'] = {'name': 'Motor 2'}
        self.assertError(data, 'parts.Motor 2: an id has lowercase letters')

    def test_an_empty_list(self):
        data = catalog()
        data['configurations']['robot']['entries'] = []
        self.assertError(data, 'configurations.robot.entries: expected a list of parts')


class LoadTest(unittest.TestCase):

    def test_a_key_written_twice_stops_loading(self):
        with self.assertRaisesRegex(parts.CatalogError, r"parts.yml:3: 'motor' is written twice"):
            parts.load_catalog('parts:\n  motor: {name: A}\n  motor: {name: B}\n')

    def test_a_reference_defined_twice(self):
        references, errors = parts.link_references(
            '[shop_us]: https://a.example\n\n[Shop_US]: https://b.example\n')
        self.assertEqual(references, {'shop_us'})
        self.assertEqual(errors, ['includes/links.md:3: [Shop_US] is defined twice'])

    def test_an_indented_definition_counts_as_a_duplicate(self):
        # Markdown accepts up to three spaces before a definition; this one would replace the link
        references, errors = parts.link_references(
            '[shop_us]: https://a.example\n   [shop_us]: https://b.example\n')
        self.assertEqual(errors, ['includes/links.md:2: [shop_us] is defined twice'])

    def test_a_definition_without_a_link_is_an_error(self):
        references, errors = parts.link_references('[shop_us]: \n')
        self.assertEqual(references, set())
        self.assertEqual(errors, ['includes/links.md:1: not a link reference definition '
                                  '([id]: https://...)'])

    def test_definitions_count_exactly_when_markdown_uses_them(self):
        for line, used in (('[a]: https://a.example', True),
                           ('   [a]: https://a.example', True),
                           ('[a]: https://a.example "Title"', True),
                           ('[a]: ', False),
                           ('[a]:', False),
                           ('    [a]: https://a.example', False),
                           ('<!-- [a]: https://a.example -->', False)):
            with self.subTest(line=line):
                rendered = markdown.markdown('[shop][a]\n\n' + line)
                self.assertEqual('<a href=' in rendered, used)
                references, errors = parts.link_references(line)
                self.assertEqual('a' in references and not errors, used)


class RenderTest(unittest.TestCase):

    def test_rows_in_entry_order(self):
        table = parts.render_table(catalog(note='see [step 1](/assembly.md)'), 'robot')
        self.assertEqual(table.splitlines()[2:], [
            '| Motor | 2 | $12.50 | [Amazon.com][shop_us], [Maker](https://example.com/motor) '
            '| see [step 1](/assembly.md) |',
            '| Breadboard | 1 | -- |  |  |',
        ])

    def test_the_repository_lists_render_every_link(self):
        # The real catalog through Markdown with includes/links.md appended, as on the page: every
        # store and note link becomes a link, and every row keeps its five cells
        data = parts.load_catalog((ROOT / parts.CATALOG).read_text(encoding='utf-8'))
        links = (ROOT / parts.LINKS).read_text(encoding='utf-8')
        for name, configuration in data['configurations'].items():
            with self.subTest(configuration=name):
                entries = configuration['entries']
                expected_links = sum(
                    len(data['parts'][entry['part']].get('offers', []))
                    + len(parts.REFERENCE_LINK.findall(entry.get('note', '')))
                    + len(parts.INLINE_LINK.findall(entry.get('note', '')))
                    for entry in entries)
                html = markdown.markdown(parts.render_table(data, name) + '\n\n' + links,
                                         extensions=['tables'])
                self.assertEqual(html.count('<a href='), expected_links)
                self.assertNotIn('][', html)
                rows = html.split('<tbody>')[1].split('<tr>')[1:]
                self.assertEqual(len(rows), len(entries))
                for row in rows:
                    self.assertEqual(row.count('<td'), 5)

    def test_unknown_configuration(self):
        with self.assertRaisesRegex(parts.CatalogError, "there is no configuration 'robo'"):
            parts.render_table(catalog(), 'robo')


if __name__ == '__main__':
    unittest.main()
