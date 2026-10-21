# Agent instructions for ros-mobile-robots.com

Read [CONTRIBUTING.md](CONTRIBUTING.md) first; this file adds what an agent needs to work here without guessing. The same rules apply to people and agents.

## What this is

The MkDocs (Material) source of [ros-mobile-robots.com](https://ros-mobile-robots.com), the documentation of the DiffBot and Remo robots. Pages are Markdown in `docs/`, with `nav` in `mkdocs.yml`. The robots' code is in other repositories: [diffbot](https://github.com/ros-mobile-robots/diffbot) (ROS 1 Noetic packages, firmware) and [remo_description](https://github.com/ros-mobile-robots/remo_description) (Remo's URDF).

## Commands

```bash
python3 -m venv venv && venv/bin/pip install -r requirements.txt codespell   # Python 3.12, as in CI
venv/bin/mkdocs build --strict                                                # must pass, as in CI
venv/bin/codespell                                                            # must pass, as in CI
python3 scripts/figures/<name>_figures.py                            # regenerate a figure include
```

The strict build downloads the pinned code excerpts, so it needs network access. Pull requests from a branch in this repository get a preview at `https://ros-mobile-robots.com/pr-preview/pr-<number>/`; pull requests from forks don't (`.github/workflows/preview.yml`), so check those with `mkdocs serve` or `mkdocs build`.

## How the site works

- **Code excerpts:** `--8<-- "{{ diffbot_src_raw }}/path:start:end"` includes lines from diffbot at the commit pinned by `diffbot_src_raw` and `diffbot_src_url` in `mkdocs.yml`. When you repin, check that every range still covers the code the text describes, including `hl_lines`.
- **Figures:** `scripts/figures/*.py` write inline SVG to `includes/*.html`, with the site's light and dark colors. Edit the script and regenerate. A figure's example values can be defined in its script; measured data it plots goes in `scripts/figures/data/`.
- **Glossary:** `includes/glossary.yml` feeds the hover definitions on every page (`hooks/glossary.py`) and the Glossary page. A term's `link` points to the page that explains it.
- **Links:** `includes/links.md` is appended to every page and holds the reference definitions for part and book links (Amazon.com and Amazon.de). Reuse them, and mark such links as affiliate links on the page.
- **Math** is KaTeX (`$…$`, `$$…$$`); diagrams are Mermaid fences.

## Before you call a change done

- `mkdocs build --strict` and `codespell` pass.
- Every number, parameter, file name, command and link you write is checked against its source: the pinned code, the official documentation, or a measurement you can point to. Say in the PR what you checked.
- Pages with figures or wide tables: look at them in light and dark, on a narrow screen too.

## Writing rules

- Explain why, in short, plain sentences. SI units with a space.
- Describe the current state; leave the history of a change to its pull request. Don't name contributors or their decisions on pages.
- Link the primary source next to the claim: pinned code, official docs, the measurement.
- Theory pages teach concepts with the robots as examples; package and setup pages say how to do something.

## Commits and pull requests

- One topic per pull request and per commit; branch `docs/…`, `fix/…`, `feat/…` or `chore/…`.
- Squash-merged: the title is the commit subject, `Page: what changed`, 65 characters or fewer before GitHub's PR-number suffix. The whole description becomes the commit message: what and why, then how it was checked, then `Closes #N`. Use the pull request template.
- No attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
