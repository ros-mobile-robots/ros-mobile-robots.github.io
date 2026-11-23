# ADR-0001: Record architecture decisions

- **Status:** Accepted
- **Date:** 2026-11-23
- **Applies to:** every repository of the project, on all branches: diffbot, remo_description, robot_component_models and this site's repository

## Context and Problem Statement

The robots' code and this site are spread over four [repositories](../development/index.md#repositories). A choice that shapes more than one pull request is explained where it was made: in an issue, in the pull request's description, which becomes the squash commit's message, and in part on the page that explains the code. For example, the ROS 2 port's issue [diffbot#136](https://github.com/ros-mobile-robots/diffbot/issues/136) compares two ROS 2 distributions for the port, and [Ports and adapters](../packages/diffbot_base/low-level.md#ports-and-adapters) explains why the firmware's wheel control can run on a PC. A contributor who meets such a choice later has to know where it was made. No single place shows which choices hold, what each applies to, and which options were weighed against it.

How should the project record such choices, so that a contributor finds each one, its reason and whether it still holds in one place?

## Considered Options

- Architecture decision records in MADR's minimal format, all in this site's repository and on this site under [Decisions](index.md)
- The same records, each in the repository that the choice concerns, with a list on this site
- An architecture document after the [arc42](https://arc42.org/) template for each repository, whose section 9 holds the decisions
- No records of their own: issues, pull request descriptions and the pages that explain the code

## Decision Outcome

Chosen option: "Architecture decision records in MADR's minimal format, all in this site's repository", because a contributor finds every record in one place, under one numbering, and a choice that concerns several repositories has one home. A code pull request that changes what the robots do usually has a docs pull request next to it, where the record is written and reviewed. A choice without one gets a docs pull request of its own. A record can link the pages that explain the code and use the site's figures.

[Decisions](index.md) says when a change gets a record, how to write one, and has the template.

### Consequences

- Good, because a record is short: the problem, the options, the choice with its reason, and what follows from the choice.
- Good, because the records are searchable on the site, next to the pages that explain the code.
- Bad, because a contributor who reads only a code repository doesn't find the records there. So each record names the repository and branch that it applies to, and each code repository adds a link to [Decisions](index.md) to its `CONTRIBUTING.md` when the guide is next edited.
- Bad, because a record is only as current as its upkeep. A changed choice needs a new record that supersedes the old one.
