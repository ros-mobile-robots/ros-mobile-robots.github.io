# Decisions

The robots' code and this site are spread over four [repositories](../development/index.md#repositories). Some choices shape more than the pull request that makes them: where the firmware's portable logic ends, which ROS 2 distribution the port targets, or which tag of the robot image a robot downloads. This section keeps such choices, one page each. Such a page is an architecture decision record (ADR). It names the problem, the options that were weighed, the option that was chosen and why, and what follows from the choice.

A record says why something is as it is. How the code works is explained on the code's own page, which the record links.

## The records

| Record | Status | Applies to | Date |
|:--|:--|:--|:--|
| [ADR-0001: Record architecture decisions](0001-record-architecture-decisions.md) | Accepted | Every repository of the project | 2026-11-23 |

## When a change gets a record

A change gets a record when it makes a choice that is hard to reverse, and a later contributor needs to know the reason for it. Such a choice is about one of these:

- **An interface:** a message, a topic, a launch argument, a command, or the link between the robot's computer and its microcontroller.
- **A boundary between parts:** for example, which code may depend on Arduino and rosserial.
- **A promise to the robots' users:** for example, a tag of the robot image that a robot downloads by name.
- **The way contributors work:** for example, where tests live or how figures are made.

A fix, or a choice that concerns only its own pull request, needs no record. Its pull request description says why.

## How to write a record

1. **The file:** `docs/decisions/NNNN-title-with-dashes.md` in this site's repository, with the next free number. A number is never used twice.
2. **The pull request:** write the record in the docs pull request that belongs to the code pull request that makes the choice. The code pull request links it under `## Documentation`. A choice without such a docs pull request gets one for its record. The pull request is where the record is proposed and discussed, and the record merges with the status Accepted.
3. **The list:** add the record to the table in [The records](#the-records) and to the menu in `mkdocs.yml`.
4. **The text:** follow the template below. Write from what anyone can check, such as issues, pull requests, measurements and this site's pages, and link them.
5. **A changed choice:** a new record replaces the old one. The old record keeps its text and gets the status "Superseded by ADR-NNNN", with a link to the new record, and the new record links the old one. A choice that no longer holds and has no successor gets the status Deprecated.

The template below is the minimal template of [MADR 4.0.0](https://adr.github.io/madr/), the Markdown Architectural Decision Records, with three lines added at the top: the status, the date and what the choice applies to. MADR's templates may be used under the MIT license or CC0 1.0. Michael Nygard proposed such records in his article [Documenting Architecture Decisions](https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions) (2011).

```markdown
# ADR-NNNN: {short title of the problem and its solution}

- **Status:** Accepted
- **Date:** {YYYY-MM-DD, the day the record last changed}
- **Applies to:** {the repository and branch, for example diffbot on noetic-devel}

## Context and Problem Statement

{Two or three sentences that describe the problem, then the question that the record
answers. Link the issue.}

## Considered Options

- {option 1}
- {option 2}

## Decision Outcome

Chosen option: "{option 1}", because {the reason}.

### Consequences

- Good, because {…}
- Bad, because {…}
```
