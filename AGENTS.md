# Agent instructions for ros-mobile-robots.com

Read [CONTRIBUTING.md](CONTRIBUTING.md) first; this file adds what an agent needs to work here without guessing. The same rules apply to people and agents.

## Before you push

Find your change in this list. Each line says what to run or check, and links the section with the details.

- **Every change:** the strict build and the spelling check pass on your PC ([Commands](#commands)). Then look at which checks GitHub ran on your pull request: a stacked pull request and a pull request from a fork get fewer ([Checks on a pull request](#checks-on-a-pull-request)).
- **A page:** every number, parameter, file name, command and link you write is checked against its source: the pinned code, the official documentation, or a measurement you can point to. Say in the pull request what you checked. Every code excerpt has a full sentence before it, and every line range the text points to has its excerpt. `grep -n ';'` on the changed pages finds no sentence that two short ones would say better ([Writing rules](#writing-rules)).
- **A page with figures or wide tables:** look at the page in light and dark, on a narrow screen too.
- **A page that explains a feature:** the page links the feature's tests page, or is that page ([A feature's tests](#a-features-tests)).
- **A Python script, hook, macro or test:** the unit tests pass, new logic has a test, and flake8 and mypy report nothing in the code that you added or changed: a new file, a line at module level, and the whole of a function, a class or a test that you touched ([Check the Python style](#check-the-python-style)). After a change to a figure script, regenerate the script's includes and commit them with the script.
- **The theme's JavaScript:** the rules for functions and names hold, and new logic comes with a test that CI runs, as in Python ([Code in this repository](#code-in-this-repository)). No tool checks the style of JavaScript here, so also look at a page that uses the script, with `venv/bin/mkdocs serve`.
- **The commits and the pull request:** one topic, signed commits, and no attribution lines for AI tools ([Commits and pull requests](#commits-and-pull-requests)).

## What this is

The MkDocs (Material) source of [ros-mobile-robots.com](https://ros-mobile-robots.com), the documentation of the DiffBot and Remo robots. Pages are Markdown in `docs/`, with `nav` in `mkdocs.yml`. The robots' code is in other repositories: [diffbot](https://github.com/ros-mobile-robots/diffbot) (ROS 1 Noetic packages, firmware) and [remo_description](https://github.com/ros-mobile-robots/remo_description) (Remo's URDF).

## Commands

On your PC, in the repository's root, with Python 3.12 as in CI:

```bash
python3 -m venv venv && venv/bin/pip install -r requirements.txt codespell   # once
venv/bin/python -m unittest discover -s tests -v    # as in CI. The last line is "OK"
venv/bin/mkdocs build --strict                      # as in CI. Ends with "Documentation built in … seconds"
venv/bin/codespell                                  # as in CI. Prints nothing
venv/bin/python scripts/figures/<name>_figures.py   # regenerate a figure script's includes
```

Each of the three checks passes when its exit status is 0. The strict build downloads the pinned code excerpts, so it needs network access. `venv/bin/mkdocs serve` shows the site at `http://127.0.0.1:8000` while you write.

### Checks on a pull request

GitHub runs up to three workflows by itself when a pull request is opened or gets a new commit. Which ones run depends on the branch that the pull request targets, and on whether its own branch is in this repository or in a fork:

| Pull request | Documentation CI: unit tests and strict build | Lint: spelling | PR preview: strict build and the preview site |
|:--|:--|:--|:--|
| Branch of this repository, into `main` | Yes | Yes | Yes |
| Branch of this repository, into another branch (a stacked pull request) | No | No | Yes |
| Fork, into `main` | Yes | Yes | No |
| Fork, into another branch | No | No | No |

The preview site is at `https://ros-mobile-robots.com/pr-preview/pr-<number>/`. The triggers are in `.github/workflows/ci.yml`, `lint.yml` and `preview.yml`.

- **A check that didn't run hasn't passed:** run the three commands above on the commit you push. Say in the description which commit that was and what the commands printed.
- **Stacked pull request from a branch of this repository:** ask a maintainer to [run Documentation CI and Lint by hand](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow) on the pull request's branch. A manual run needs write access. It checks the newest commit that the branch has at that moment, so put the commit that each run shows into the description.
- **Pull request from a fork:** GitHub publishes no preview for a fork, so look at the pages with `venv/bin/mkdocs serve`. A manual run takes a branch or a tag of this repository, never a fork's branch. For a fork's pull request into another branch than `main`, the results from your PC are therefore the only ones, until the pull request targets `main`.

## How the site works

- **Code excerpts:** `--8<-- "{{ diffbot_src_raw }}/path:start:end"` includes lines from diffbot at the commit pinned by `diffbot_src_raw` and `diffbot_src_url` in `mkdocs.yml`. When you repin, check that every range still covers the code the text describes, including `hl_lines`.
- **Figures:** `scripts/figures/*.py` write inline SVG to `includes/*.html`, with the site's light and dark colors. Edit the script and regenerate. A figure's example values can be defined in its script; measured data it plots goes in `scripts/figures/data/`.
- **Glossary:** `includes/glossary.yml` feeds the hover definitions on every page (`hooks/glossary.py`) and the Glossary page. A term's `link` points to the page that explains it.
- **Links:** `includes/links.md` is appended to every page and holds the reference definitions for part and book links (Amazon.com and Amazon.de). Reuse them, and mark such links as affiliate links on the page.
- **Math** is KaTeX (`$…$`, `$$…$$`); diagrams are Mermaid fences.

## Writing rules

A page is for a reader who is new to its topic. They should be able to follow it from top to bottom without opening the code.

### Explain

- **Why:** explain why, in short, plain sentences, one idea per paragraph. Theory pages teach concepts with the robots as examples. Package and setup pages say how to do something.
- **Terms:** define a term where a page first uses it, and link it to the page that explains it. `includes/glossary.yml` holds the terms that many pages share.
- **Links:** a link's text says what the link leads to: the title of the page or section, the path of the file, or what the issue is about. Never "here", "this" or a bare number.
- **Quantities:** a value always has its unit, in SI units with a space (`0.5 kg`, `3 s`). A quantity also has its symbol on a page that has a formula or a figure with that symbol: from where the page first names the quantity, and the same symbol after that, as in "the yaw rate $\omega_z$ (rad/s)". A page of setup steps without formulas needs no symbols.
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

- **Before an excerpt:** a full sentence says what the excerpt shows and what to look at in it. [`drive()` on the Simulation Test page](https://ros-mobile-robots.com/development/simulation-test/#drive) shows such a sentence above its excerpt.
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

- **Where the rules hold:** in every file you add. In an older file, in every line that you add or change at module level, such as an import or a constant, and in the whole of every function, class and test that you add or change. The rest of an older file keeps its form until it changes. `scripts/figures/localization_figures.py` and `tests/test_localization_figures.py` follow the rules. `scripts/figures/simulation_test_figures.py` and `wheel_dynamics_figures.py` are older: each has its own `esc()`, functions without type hints, and includes that are written without an encoding.
- **What CI checks:** the unit tests, the strict build and the spelling. No workflow runs flake8 or mypy in this repository. Run both yourself on the Python files you changed, as [Check the Python style](#check-the-python-style) shows, and say in the pull request what they reported. Neither tool reads JavaScript.
- **Functions:** one thing per function, named for it. A `main()` reads as the list of its steps, and a function that checks several things becomes one function per check and one that runs them.
- **Names:** a name says what the thing is or does. A function that returns a figure is called `…_figure()`, not after the quantity it draws. One word for one thing. No one-letter names outside a formula that defines them.
- **Types:** one type for what belongs together (a `NamedTuple` such as `Box(x, y, width, height)`), and an `Enum` for a closed set such as a figure's line kinds. Not bare tuples read by position.
- **One source for every value:** a text on a figure is built from the constant, not typed again. A value that Python and JavaScript both need travels in the include's data.
- **Python:** PEP 8 with the settings of ROS 2's `ament_flake8`, PEP 257 docstrings (a function's first line is a command), type hints on every function, tests included (`-> None`). Python 3.12, as in CI.
- **Figure scripts:** they share their common helpers (escaping, the style element, the site's colors, the figure class) in a module instead of copying them. Every generated include starts with a comment that names its script. Write text files with `encoding='utf-8'`.
- **JavaScript:** the same rules for functions and names, and no new globals. Libraries are served from the site, each under a name with its version, such as `katex-0.19.0`.
- **Tests:** a name says the subject, the observable result and the condition, such as `test_a_valid_catalog_has_no_errors`. One behavior per test, named values instead of bare numbers, and helpers that assert are called `assert_…`. Older tests keep their names until they change. `venv/bin/python -m unittest discover -s tests -v` runs the tests.
- **Tests for new logic:** new logic comes with a test that CI runs. CI runs the tests only for a pull request into `main` ([Checks on a pull request](#checks-on-a-pull-request)). For any other pull request, run them on your PC and put the result into the description.

### Check the Python style

flake8 runs with the plugins and settings of ROS 2's `ament_flake8`, and mypy checks the type hints. Both are installed into the site's `venv`, so they run on Python 3.12 with the packages that the site is built with. On your PC, in the repository's root, with the files you changed in place of the two files below:

```bash
venv/bin/pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline \
  flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes \
  mypy==1.10.1 types-PyYAML   # once
venv/bin/flake8 --max-line-length 99 --import-order-style google \
  --extend-ignore B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202 \
  scripts/figures/localization_figures.py tests/test_localization_figures.py
venv/bin/mypy --python-version 3.12 --ignore-missing-imports --disallow-untyped-defs \
  scripts/figures/localization_figures.py tests/test_localization_figures.py
```

For these two files, flake8 prints nothing and mypy prints `Success: no issues found in 2 source files`. A file you add has to give the same result. In an older file, no finding may be left in a line that you added or changed, and none in the rest of a function, a class or a test that you touched. A change inside a function also brings the function's signature in line: the type hints belong to the function, although the changed line is another one. Findings in functions that you didn't touch, and in module-level lines that you didn't touch, may stay. Say in the pull request which files you checked and which findings remain.

### Examples

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

- **One topic:** one topic per pull request and per commit.
- **Branch:** `docs/…`, `fix/…`, `feat/…`, `chore/…`, `refactor/…` or `test/…`, followed by a short topic in kebab-case, such as `docs/dev-container`.
- **Title and description:** pull requests are squash-merged. The title is the commit subject, `Page: what changed`, 65 characters or fewer before GitHub's PR-number suffix. The whole description becomes the commit message: what and why, then how it was checked. Use the pull request template.
- **Issues:** end the description with `Closes #N` when the pull request resolves issue N. For an issue that stays open, name the issue without a closing keyword, as in `Part of #N`.
- **Signed commits:** sign every commit with GPG, `git commit -S`. `git log --show-signature -1` shows the signature of the last commit. A coding agent that can't sign stops and reports that, and doesn't commit unsigned.
- **AI tools:** no attribution lines for AI tools in commits or pull requests (`Co-Authored-By:` for a tool, "Generated with …"). No disclosure of AI use is needed.
