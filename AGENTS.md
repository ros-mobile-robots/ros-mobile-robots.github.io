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
- Every code excerpt has a full sentence before it, and every line range the text points to has its excerpt.
- A page that explains a feature links the feature's tests page, or is that page.
- `grep -n ';'` on the changed pages finds no sentence that two short ones would say better.

## Writing rules

A page is for a reader who is new to its topic. They should be able to follow it from top to bottom without opening the code.

### Explain

- **Why:** explain why, in short, plain sentences, one idea per paragraph. Theory pages teach concepts with the robots as examples. Package and setup pages say how to do something.
- **Terms:** define a term where a page first uses it, and link it to the page that explains it. `includes/glossary.yml` holds the terms that many pages share.
- **Links:** a link's text says what the link leads to: the title of the page or section, the path of the file, or what the issue is about. Never "here", "this" or a bare number.
- **Quantities:** a quantity has its symbol and its unit where a page first names it, and the same symbol after that: "the yaw rate $\omega_z$ (rad/s)". SI units with a space (`0.5 kg`, `3 s`).
- **Which model:** say which one you mean, the robot's URDF model (the xacro files), the SDF model that Gazebo converts it to, or the real robot. Never "the model" alone where two of them are near.
- **Where a command runs:** on the PC, in the dev container or on the robot.
- **The current state:** describe how things are. How the code or the text got there belongs in the pull request: no "now", "no longer", "previously". Don't name contributors or their decisions.
- **Sources:** link the primary source next to the claim: the pinned code, the official documentation, or the measurement. Say so when something was not tested on the robot.
- **Plain words:** no "it" or "this" that the reader has to guess, and no summary in jargon. Say what the thing does.

| Don't | Do |
|:--|:--|
| `see [here](simulation-test.md#three-views-of-the-motion)` | `[Three views of the motion](simulation-test.md#three-views-of-the-motion) compares the command, the odometry and the true pose` |
| `the test of #138 will measure that` | `the [floor odometry test](https://github.com/ros-mobile-robots/diffbot/issues/138) will measure that` |
| `the IMU measures the yaw rate` | `the IMU measures the yaw rate $\omega_z$ (rad/s)` |
| `the wheel separation from the robot's model` | `the wheel separation from the robot's URDF model` |

### Code and configuration on a page

- **Before an excerpt:** a full sentence says what the excerpt shows and what to look at in it.
- **Line ranges:** every line range the text points to gets its excerpt, collapsed (`???`) when it is long. A link to the lines isn't enough.
- **The code's own names:** the text uses the variables and functions as the excerpt has them, in the order the code runs them. Say who calls the code and when.
- **Lists by position:** a configuration list whose meaning lies in the position of its values, such as the six values of a controller's `twist_covariance_diagonal`, gets its excerpt, the symbol of each value that is set, and a table with one row per position: what the value is, where it comes from, and its unit. A link to the upstream parameter page doesn't replace the table.

| Don't | Do |
|:--|:--|
| ``drive(), test_drive.py, lines 147–157:`` | ``The method `drive()` sends the velocity command at `COMMAND_RATE` until `duration` seconds of simulation time have passed:`` |
| `Then the test measures the motion.` | ``Then `measure_motion()` notes the odometry's pose and the true pose, calls `drive()` and `stop()`, and returns both motions relative to their start.`` |

### Structure

- **Order:** sections follow the order of execution, and a heading says what its section is: "The checks before a call publishes", not "When the plugin publishes".
- **Figures:** a figure where a page explains a flow, a structure or a sequence over time. A Mermaid diagram for who calls or includes whom, a generated figure (`scripts/figures/`) for geometry, time lines and signals. The sections' names and order follow the figure.
- **Lists:** a bold label at the start of a bullet ends with a colon, not a period. Steps are a numbered list.
- **Semicolons:** few. Write two short sentences or a list. Run `grep -n ';'` on the pages you changed before you push.

### A feature's tests

- **The walkthrough:** every feature's pages walk through its tests. A tests page (a sub page, several when the feature has several test files) explains each test in the order a reader would run it:
    1. what the test sets up,
    2. what goes in,
    3. what the robot or the node does,
    4. what is checked, and with which tolerance.

    Each step has its code excerpt, and each test has one picture: a time line, the robot from above, or the inputs next to the expected output. The "Code walkthrough" of [Simulation Test](https://ros-mobile-robots.com/development/simulation-test/#code-walkthrough) shows the form.
- **When:** plan the tests page with the code pull request, not after its review.

## Code in this repository

The scripts (`scripts/`, `hooks/`, `macros/`), the tests (`tests/`) and the theme's JavaScript (`docs/custom_theme/js/`) follow the same rules as the robots' code:

- **Functions:** one thing per function, named for it. A `main()` reads as the list of its steps, and a function that checks several things becomes one function per check and one that runs them.
- **Names:** a name says what the thing is or does. A function that returns a figure is called `…_figure()`, not after the quantity it draws. One word for one thing. No one-letter names outside a formula that defines them.
- **Types:** one type for what belongs together (a `NamedTuple` such as `Box(x, y, width, height)`), and an `Enum` for a closed set such as a figure's line kinds. Not bare tuples read by position.
- **One source for every value:** a text on a figure is built from the constant, not typed again. A value that Python and JavaScript both need travels in the include's data.
- **Python:** PEP 8 with the settings of ROS 2's `ament_flake8`, PEP 257 docstrings (a function's first line is a command), type hints on every function, tests included (`-> None`). Python 3.12, as in CI.
- **Figure scripts:** they share their common helpers (escaping, the style element, the site's colors, the figure class) in a module instead of copying them. Every generated include starts with a comment that names its script. Write text files with `encoding='utf-8'`.
- **JavaScript:** the same rules for functions and names, and no new globals. Libraries are served from the site, each under a name with its version, such as `katex-0.19.0`.
- **Tests:** a name says the subject, the observable result and the condition, such as `test_a_valid_catalog_has_no_errors`. One behavior per test, named values instead of bare numbers, and helpers that assert are called `assert_…`. Older tests keep their names until they change. `python -m unittest discover -s tests` runs the tests.
- **Tests for new logic:** new logic comes with a test that CI runs. The workflows **Documentation CI** and **Lint** run for a pull request only when its base is `main`. A stacked pull request gets the preview, and no tests and no spelling check. Start both workflows by hand (`workflow_dispatch`) for the commit under review, and say in the description which commit that was.

The first two lines below type the box's lower edge a second time. The type under them keeps the arrow at the box when the box moves:

```python
CONTROLLER_BOX = (180, 30, 120, 52)  # x, y, width, height
ARROW_START_Y = 82
```

```python
from typing import NamedTuple


class Box(NamedTuple):
    """A box on a figure, in pixels."""

    x: float
    y: float
    width: float
    height: float

    @property
    def bottom(self) -> float:
        """Return the y coordinate of the box's lower edge."""
        return self.y + self.height


CONTROLLER_BOX = Box(x=180, y=30, width=120, height=52)
ARROW_START_Y = CONTROLLER_BOX.bottom
```

In JavaScript, the first name below would also be read as a coordinate frame on a page about a robot's frames. The second one says what the value is:

```js
let frame = requestAnimationFrame(drawNextFrame);
```

```js
let animationRequest = requestAnimationFrame(drawNextFrame);
```

## Commits and pull requests

- One topic per pull request and per commit; branch `docs/…`, `fix/…`, `feat/…` or `chore/…`.
- Squash-merged: the title is the commit subject, `Page: what changed`, 65 characters or fewer before GitHub's PR-number suffix. The whole description becomes the commit message: what and why, then how it was checked, then `Closes #N`. Use the pull request template.
- No attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
