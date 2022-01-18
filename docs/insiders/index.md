---
template: overrides/main.html
title: Insiders
---

# Insiders

Diffbot and Remo follow the __sponsorware__ release strategy, which means
that new features are first exclusively released to sponsors as part of
[Insiders]. Read on to learn [what sponsorships achieve],
[how to become a sponsor] to get access to Insiders, and
[what's in for you][available features]!

<figure class="mdx-video" markdown>
  <div class="mdx-video__inner">
    <iframe src="https://streamable.com/e/ihhxw0" allowfullscreen></iframe>
  </div>
  <figcaption markdown>

This documentation is built with Insiders
[squidfunk.github.io/mkdocs-material][Material for MkDocs]

  </figcaption>
</figure>

  [Insiders]: #what-is-insiders
  [what sponsorships achieve]: #what-sponsorships-achieve
  [how to become a sponsor]: #how-to-become-a-sponsor
  [available features]: #available-features
  [Material for MkDocs]: https://squidfunk.github.io/mkdocs-material/

## What is Insiders?

DiffBot and Remo Insiders are private forks of DiffBot and Remo respectively, hosted as
a private GitHub repository. Almost[^1] all new features are developed as part of
these forks, which means that they are immediately available to all eligible
sponsors, as they are made collaborators of this repository.

  [^1]:
    In general, every new feature is first exclusively released to sponsors, but
    sometimes upstream dependencies enhance
    existing features that must be supported by DiffBot.

Every feature is tied to a [funding goal] in monthly subscriptions. When a
funding goal is hit, the features that are tied to it are merged back into
DiffBot or Remo and released for general availability, making them available
to all users. Bugfixes are always released in tandem.

Sponsorships start as low as [__$10 a month__][how to become a sponsor].[^2]

  [^2]:
    Note that $10 a month is the minimum amount to become eligible for
    Insiders. While GitHub Sponsors also allows to sponsor lower amounts or
    one-time amounts, those can't be granted access to Insiders due to
    technical reasons.

## What sponsorships achieve

Sponsorships make this project sustainable, as they buy the maintainers of this
project time – a very scarce resource – which is spent on the development of new
features, bug fixing, stability improvement, issue triage and general support.
The biggest bottleneck in Open Source is time.[^3]

  [^3]:
    Making an Open Source project sustainable is exceptionally hard: maintainers
    burn out, projects are abandoned. That's not great and very unpredictable.
    The sponsorware model ensures that if you decide to use DiffBot or Remo,
    you can be sure that bugs are fixed quickly and new features are added
    regularly.

<!-- If you're unsure if you should sponsor this project, check out the list of
[completed funding goals] to learn whether you're already using features that
were developed with the help of sponsorships. You're most likely using at least
a handful of them, [thanks to our awesome sponsors]!

  [completed funding goals]: #goals-completed
  [thanks to our awesome sponsors]: #how-to-become-a-sponsor
-->
## How to become a sponsor

Thanks for your interest in sponsoring! In order to become an eligible sponsor
with your GitHub account, visit [fjp's sponsor profile], and complete
a sponsorship of __$10 a month or more__. You can use your individual or
organization GitHub account for sponsoring.

<!-- __Important__: If you're sponsoring @fjp through a GitHub organization,
please send a short email to ros@fjp.at with the name of your
organization and the GitHub account of the individual that should be added as a 
collaborator.[^4] -->

You can cancel your sponsorship anytime.[^5]

<!--  [^4]:
    It's currently not possible to grant access to each member of an
    organization, as GitHub only allows for adding users. Thus, after
    sponsoring, please send an email to ros@fjp.at, stating which
    account should become a collaborator of the Insiders repository. We're
    working on a solution which will make access to organizations much simpler.
    To ensure that access is not tied to a particular individual GitHub account,
    create a bot account (i.e. a GitHub account that is not tied to a specific
    individual), and use this account for the sponsoring. After being added to
    the list of collaborators, the bot account can create a private fork of the
    private Insiders GitHub repository, and grant access to all members of the
    organizations.
-->

  [^5]:
    If you cancel your sponsorship, GitHub schedules a cancellation request
    which will become effective at the end of the billing cycle. This means
    that even though you cancel your sponsorship, you will keep your access to
    Insiders as long as your cancellation isn't effective. All charges are
    processed by GitHub through Stripe. As we don't receive any information
    regarding your payment, and GitHub doesn't offer refunds, sponsorships are
    non-refundable.

[:octicons-heart-fill-24:{ .mdx-heart } &nbsp; Join our <span class="mdx-sponsorship-count" data-mdx-component="sponsorship-count"></span> awesome sponsors][fjp's sponsor profile]{ .md-button .md-button--primary .mdx-sponsorship-button }


<div class="mdx-sponsorship" data-mdx-component="sponsorship" hidden>
  <div class="mdx-sponsorship__list"></div>
  <small>
    If you sponsor publicly, you're automatically added here with a link to
    your profile and avatar to show your support for DiffBot and Remo.
    Alternatively, if you wish to keep your sponsorship private, you'll be a
    silent +1. You can select visibility during checkout and change it
    afterwards.
  </small>
</div>

  [fjp's sponsor profile]: https://github.com/sponsors/fjp

## Available features

The following features are solely available via Material for MkDocs Insiders:

<div class="mdx-columns" markdown>

- [x] [TBA] :material-new-box:
- [x] [TBA icons] :material-new-box:
- [x] [TBA]

</div>

## Funding <span class="mdx-sponsorship-total" data-mdx-component="sponsorship-total"></span>

### Goals

The following section lists all funding goals. Each goal contains a list of
features prefixed with a checkmark symbol, denoting whether a feature is
:octicons-check-circle-fill-24:{ style="color: #00e676" } already available or 
:octicons-check-circle-fill-24:{ style="color: var(--md-default-fg-color--lightest)" } planned, but not yet implemented. When the funding goal is hit, the features
are released for general availability.

#### $ 500 – Bärglistock

- [ ] PID tuning on MCU using topic instead of requiring re-flashing
- [ ] Remo: RPLidar 1 lidar platform

#### $ 1,000 – Mont Durand

- [ ] Raspberry Pi Camera support
- [ ] Reset odometry

#### $ 2,000 – Eiger

- [ ] IMU support
- [ ] OAK-1 support
- [ ] OAK-D support

#### $ 2,500 – Castor

- [ ] Display (HMI) support
- [ ] Add `diffbot_mbf` package using `move_base_flex`, the improved version of `move_base`

#### $ 5,000 – Nordend

- [ ] ROS 2


### Goals completed

This section lists all funding goals that were previously completed, which means
that those features were part of Insiders, but are now generally available and
can be used by all users.

#### $ 0 – Schneestock

- [x] Make use of rosdep for system dependencies
- [x] [Parametrize sensor description when using standard laser instead of gpu laser](#27)
- [x] vcstool to simplify external dependency installation

## Frequently asked questions

### Payment

_We don't want to pay for sponsorship every month. Are there any other options?_

Yes. You can sponsor on a yearly basis by [switching your GitHub account to a
yearly billing cycle][billing cycle]. If for some reason you cannot do that, you
could also create a dedicated GitHub account with a yearly billing cycle, which
you only use for sponsoring (some sponsors already do that).

If you have any problems or further questions, please reach out to
sponsors@squidfunk.com.

  [billing cycle]: https://docs.github.com/en/github/setting-up-and-managing-billing-and-payments-on-github/changing-the-duration-of-your-billing-cycle

### Terms

_Are we allowed to use Insiders under the same terms and conditions as
DiffBot and Remo?_

Yes. Whether you're an individual or a company, you may use _DiffBot and Remo
Insiders_ precisely under the same terms as Material for MkDocs, which are given
by the [BSD 3-Clause License]. However, we kindly ask you to respect the following
guidelines:

- Please __don't distribute the source code__ of Insiders. You may freely use
  it for public, private or commercial projects, privately fork or mirror it,
  but please don't make the source code public, as it would counteract the 
  sponsorware strategy.

- If you cancel your subscription, you're automatically removed as a
  collaborator and will miss out on all future updates of Insiders. However, you
  may __use the latest version__ that's available to you __as long as you like__.
  Just remember that [GitHub deletes private forks].

  [BSD 3-Clause License]: ../LICENSE
  [GitHub deletes private forks]: https://docs.github.com/en/github/setting-up-and-managing-your-github-user-account/removing-a-collaborator-from-a-personal-repository
