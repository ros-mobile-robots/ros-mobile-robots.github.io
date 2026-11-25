# Decisions

An architecture decision record (ADR) is a short page about one lasting choice of the project: the problem, the options that were weighed, the option that was chosen and why, and what follows from the choice. For example, which tag of the robot image a robot downloads is such a choice. This section keeps the project's records, one page each, for all of its [repositories](../development/index.md#repositories).

A record says why something is as it is. How the code works is explained on the code's own page, which the record links.

## The records

| Record | Status | Applies to | Last updated |
|:--|:--|:--|:--|
| [ADR-0001: Record architecture decisions](0001-record-architecture-decisions.md) | Accepted | All repositories | 2026-11-23 |

## When a change gets a record

A change gets a record when it makes a choice that is hard to reverse, and a later contributor needs to know the reason for it. Such a choice is about one of these:

- **An interface:** a message, a topic, a launch argument, a command, or the link between the robot's computer and its microcontroller.
- **A boundary between parts:** for example, which code may depend on Arduino and rosserial.
- **A promise to the robots' users:** for example, a tag of the robot image that a robot downloads by name.
- **The way contributors work:** for example, where tests live or how figures are made.

A fix can make such a choice too, for example when it changes a message. A routine fix that leaves these choices as they are, or a choice that concerns only its own pull request, needs no record. Its pull request description says why.

## How to write a record

1. **The file:** `docs/decisions/NNNN-title-with-dashes.md` in this site's repository, with the next free number. A number is never used twice.
2. **The pull request:** write the record in the docs pull request that belongs to the code pull request that makes the choice. When the code pull request has no docs pull request, open one for the record. Link the record's pull request from the code pull request, where the repository's pull request template asks for related pull requests: under `## Documentation` in diffbot, under `## Related` in remo_description and robot_component_models.
3. **Proposed, then accepted:** while the pull request is open, the record and its preview are a proposal, and the pull request is where it is discussed. The record says "Accepted" from the start, because merging the pull request is what accepts the choice. Building, releasing and testing on the robot can follow later and have their own evidence.
4. **The list:** add the record to the table in [The records](#the-records) and to the menu in `mkdocs.yml`.
5. **The text:** follow the template below. Write from what anyone can check, such as issues, pull requests, measurements and this site's pages, and link them. A record of an older choice gives the date of that choice in its context, where it is known.
6. **A changed choice:** a new record replaces the old one. The old record keeps its text and gets the status "Superseded by ADR-NNNN", with a link to the new record, and the new record links the old one. A choice that no longer holds and has no successor gets the status Deprecated. In the same pull request, update the old record's row in the table: its status and its date.

The template below adapts the [minimal template of MADR 4.0.0](https://github.com/adr/madr/blob/4.0.0/template/adr-template-minimal.md), the Markdown Architectural Decision Records. It adds three lines at the top: the status, the date the record was last updated, and what the choice applies to. It keeps the consequences, which MADR's template makes optional. MADR's templates may be used under the MIT license or CC0 1.0. Michael Nygard proposed such records in his article [Documenting Architecture Decisions](https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions) (2011).

```markdown
# ADR-NNNN: {short title of the problem and its solution}

- **Status:** Accepted
- **Date:** {YYYY-MM-DD, the day the record was last updated}
- **Applies to:** {the repository and branch, for example diffbot on noetic-devel}

## Context and Problem Statement

{Two or three sentences that describe the problem, then the question that the record
answers. Link the issue or pull request where the choice was discussed. If a code pull
request or commit implements the choice, link it too.}

## Considered Options

- {option 1}
- {option 2}

## Decision Outcome

Chosen option: "{option 1}", because {the reason}.

### Consequences

- Good, because {…}
- Bad, because {…}
```
