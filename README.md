# ros-mobile-robots.github.io

Website and documentation for the ros-mobile-robots organization. 
This documentation relies on [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/).
This is a theme for [MkDocs](https://www.mkdocs.org/), a static site generator geared towards (technical) project documentation.

You can access the build documentation at https://ros-mobile-robots.github.io/.


The source code of the documentation is built using a github action, defined in [.github/workflows/ci.yml](.github/workflows/ci.yml).
This basically calls [`mkdocs build`](https://www.mkdocs.org/#building-the-site) on the source code and generates mainly just static files. 
Afterwards, the command also deploys these static files to the [gh-pages branch](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/tree/gh-pages). This triggers the github-pages build, defined in the `Settings/Pages` section of this repository, to build the static files from the `gh-pages` branch and publish the site to https://ros-mobile-robots.github.io/.

See the offical [Getting Started](https://squidfunk.github.io/mkdocs-material/publishing-your-site/#with-github-actions) guide on how to publish
the website using other methods.

## License

In general, the text of this documentation is available under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0. Please refer to [LICENSE.text](LICENSE.text) for the actual legaleese wording of the license. This applies to:

- All md files.
- All illustrations appearing in the text.

The examples and code snippets assoiated with the book are made available under a three clause BSD license, see [LICENSE.code](LICENSE.code) for the actual license text. This is to make it possible for readers to take inspiration from, or even copy and paste code from the documentation without any fear of licensing issues. This applies to all example source code.