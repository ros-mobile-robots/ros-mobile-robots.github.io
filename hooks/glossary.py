"""Shows the glossary's definitions as tooltips: every term in includes/glossary.yml with tooltip:
true (the default) becomes an abbreviation on every page, so hovering the term shows its
expansion and definition. The Glossary page (docs/glossary.md) lists the same file with the
macros plugin, so the tooltips and the page can't drift apart.

Python-Markdown's abbr extension takes the abbreviations as its glossary option (Markdown 3.7 or
newer). It marks whole words only, and not inside code.
"""
from pathlib import Path

import yaml


def on_config(config):
    path = Path(config.config_file_path).parent / 'includes' / 'glossary.yml'
    glossary = {}
    for term in yaml.safe_load(path.read_text(encoding='utf-8')):
        if not term.get('tooltip', True):
            continue
        title = term['definition']
        if term.get('expansion'):
            expansion = term['expansion']
            title = f"{expansion[0].upper()}{expansion[1:]}. {title}"
        for spelling in [term['term']] + term.get('spellings', []):
            glossary[spelling] = title
    # abbr is listed on its own and is also part of extra, whose copy would otherwise replace
    # this one without the glossary
    config.mdx_configs.setdefault('abbr', {})['glossary'] = glossary
    config.mdx_configs.setdefault('extra', {}).setdefault('abbr', {})['glossary'] = glossary
    return config
