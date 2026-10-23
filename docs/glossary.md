# Glossary

Terms and abbreviations used on this site, each with a link to the page that explains it. On every page, hovering an abbreviation, such as URDF, shows its meaning; this page lists them all, which also helps on a phone, where hovering isn't possible.

{% for t in glossary | sort(attribute='term', case_sensitive=False) %}
**{{ t.term }}**{% if t.expansion is defined %}, {{ t.expansion }}{% endif %}
:   {{ t.definition }}{% if t.link is defined %} [More]({{ t.link }}){% endif %}

{% endfor %}

The definitions come from [`includes/glossary.yml`](https://github.com/ros-mobile-robots/ros-mobile-robots.github.io/blob/main/includes/glossary.yml), the same file as the tooltips. To add a term, add it there.
