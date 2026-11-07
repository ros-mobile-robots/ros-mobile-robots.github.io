"""Generate the thread figure on docs/packages/imu/simulation.md, as inline SVG that follows the
site's light and dark colors:

- includes/imu-sim-threads.html: two cases of the controller spawner's switch_controller request
  in the Gazebo server. Each case has the same two lanes, Gazebo's physics thread and one of the
  ROS spinner threads, with time running downward. In the first case a spinner thread runs the
  service's callback. In the second, gazebo_plugins' original IMU plugin runs it in the physics
  thread with ros::spinOnce(), and the physics thread waits for itself.

The two cases are separate SVGs, side by side on a wide screen and one below the other on a
narrow one. Each is 340 units wide, so its text keeps its size on a 360 px wide phone.

Run from the repository root after changing the figure:

    python3 scripts/figures/imu_sim_figures.py
"""

W = 340
# Lanes: x from, x to. The boxes in a lane leave 6 units on each side.
PHYSICS = (16, 166)
SPINNER = (176, 336)
LANE_TOP = 112


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Fig:
    def __init__(self, h, ident, title, desc):
        self.h, self.ident, self.title, self.desc, self.parts = h, ident, title, desc, []

    def rect(self, x, y, w, h, cls, rx=4):
        self.parts.append(f'<rect class="{cls}" x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}"/>')

    def text(self, x, y, s, cls="t", anchor="start"):
        self.parts.append(f'<text class="{cls}" x="{x}" y="{y}" text-anchor="{anchor}">{esc(s)}</text>')

    def lines(self, x, y, rows, anchor="middle", step=13):
        """Rows of (text, class) from baseline y downward."""
        for i, (s, cls) in enumerate(rows):
            self.text(x, y + i * step, s, cls, anchor)

    def path(self, d, cls="a"):
        self.parts.append(f'<path class="{cls}" d="{d}"/>')

    def svg(self):
        body = "\n".join(self.parts)
        markers = ""
        for cls, head in (("a", "h"), ("ab", "h-b"), ("time", "h-t")):
            if f'class="{cls}"' in body:
                markers += (f'<marker id="{self.ident}-{cls}" viewBox="0 0 10 10" refX="9" refY="5" '
                            f'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                            f'<path class="{head}" d="M0,0 L10,5 L0,10 z"/></marker>')
                body = body.replace(f'class="{cls}"', f'class="{cls}" marker-end="url(#{self.ident}-{cls})"')
        return (f'<svg class="th-fig" viewBox="0 0 {W} {self.h}" role="img" '
                f'aria-labelledby="{self.ident}-t {self.ident}-d">'
                f'<title id="{self.ident}-t">{esc(self.title)}</title>'
                f'<desc id="{self.ident}-d">{esc(self.desc)}</desc>'
                f'<defs>{markers}</defs>\n' + body + '\n</svg>')


def common(f, number, title_rows):
    """The case's title, the spawner and the queue above the lanes, and the two lanes."""
    f.text(6, 22, number, "t-num")
    f.lines(22, 22, [(row, "t-title") for row in title_rows], anchor="start", step=15)
    f.rect(176, 6, 160, 34, "box")
    f.lines(256, 20, [("controller spawner", "t-node"), ("another ROS node", "t-sub")])
    f.path("M256,40 V64")
    f.text(262, 56, "request", "t-flow")
    f.rect(176, 66, 160, 34, "queue")
    f.lines(256, 80, [("global callback queue", "t-node"), ("a list, not a thread", "t-sub")])
    for (x0, x1), label in ((PHYSICS, "physics thread"), (SPINNER, "ROS spinner thread")):
        f.rect(x0, LANE_TOP, x1 - x0, f.h - LANE_TOP - 4, "lane", rx=6)
        f.text(x0 + 7, LANE_TOP + 15, label, "t-lane")
    # Time runs downward, along the left edge
    f.path(f"M7,{LANE_TOP + 24} V{f.h - 14}", "time")
    f.parts.append(f'<text class="t-sub" transform="translate(5,{LANE_TOP + 60}) rotate(-90)" '
                   f'text-anchor="end">time</text>')


def step(f, y, name, what, cls="box", lane=PHYSICS, h=30):
    x0, x1 = lane
    f.rect(x0 + 6, y, x1 - x0 - 12, h, cls)
    f.lines((x0 + x1) / 2, y + 13, [(name, "t-mono"), (what, "t-sub")])


# Case 1: a spinner thread runs the callback, the physics thread goes on and calls update()
ok = Fig(338, "th-ok", "Callbacks served by ROS threads",
         "The controller spawner, another ROS node, sends a switch_controller request. It waits in "
         "the global callback queue. A ROS spinner thread takes it and runs the service's callback, "
         "which hands the change to update() and waits. Meanwhile the physics thread keeps "
         "stepping: UpdateChild() publishes the IMU message, update() runs the controllers, and "
         "in a later step update() takes the change. The callback returns, and the spawner gets "
         "its answer.")
common(ok, "1", ["Callbacks served", "by ROS threads"])
step(ok, 136, "UpdateChild()", "publishes the IMU message")
step(ok, 174, "update()", "runs the controllers")
step(ok, 240, "update()", "takes the change", cls="cb-edge")
ok.text(91, 290, "next steps …", "t-sub", "middle")
ok.text(91, 303, "the clock runs on", "t-sub", "middle")
# The request from the queue to the spinner thread, and its callback
ok.path("M318,100 V146", "ab")
ok.rect(182, 148, 148, 128, "cb")
ok.lines(256, 164, [("switch_controller", "t-mono"), ("callback", "t-sub")])
ok.lines(256, 196, [("hands the change", "t-node"), ("to update()", "t-node"), ("and waits", "t-node")])
ok.path("M160,255 H180", "ab")
ok.text(256, 268, "the change is taken", "t-sub", "middle")
ok.path("M256,276 V300", "a")
ok.lines(256, 314, [("returns: the spawner", "t-sub"), ("gets its answer", "t-sub")])

# Case 2: ros::spinOnce() in UpdateChild() runs the callback in the physics thread
dl = Fig(378, "th-dl", "A callback taken by the original IMU plugin",
         "The same request waits in the global callback queue. Before a spinner thread takes it, "
         "the original IMU plugin's UpdateChild() publishes the IMU message and calls "
         "ros::spinOnce(), which runs the service's callback in the physics thread. The callback "
         "hands the change to update() and waits. update() runs only in the physics thread, "
         "which is busy waiting, so the next update() never runs. The step never ends, the "
         "simulation's clock stops, and the spawner never gets its answer. The spinner thread "
         "finds the queue empty.")
common(dl, "2", ["A callback taken", "by the original", "IMU plugin"])
# UpdateChild() holds ros::spinOnce(), which holds the callback: the boxes stay open at the bottom
dl.path("M22,280 V136 H160 V280", "frame")
dl.path("M22,280 V300 M160,280 V300", "frame-open")
dl.lines(91, 149, [("UpdateChild()", "t-mono"), ("publishes the IMU message", "t-sub")])
dl.path("M30,276 V170 H152 V276", "frame")
dl.path("M30,276 V292 M152,276 V292", "frame-open")
dl.lines(91, 183, [("ros::spinOnce()", "t-mono"), ("runs the callback here", "t-sub")])
dl.rect(38, 200, 106, 76, "cb")
dl.lines(91, 214, [("switch_controller", "t-mono"), ("callback", "t-sub")])
dl.lines(91, 244, [("hands the change", "t-node"), ("to update(), waits", "t-node")])
dl.text(91, 290, "waiting for update()", "t-bad", "middle")
# The request goes to the physics thread instead of a spinner thread
dl.path("M184,100 L148,206", "ab")
# The next update() can't run
dl.rect(28, 306, 126, 30, "stuck")
dl.lines(91, 319, [("update()", "t-mono-bad"), ("can't run: thread busy", "t-sub")])
dl.text(91, 356, "deadlock: the clock stops", "t-bad-b", "middle")
dl.text(91, 369, "and the spawner gets no answer", "t-sub", "middle")
# The spinner thread finds nothing to do
dl.rect(182, 136, 148, 30, "idle")
dl.lines(256, 149, [("waits for work", "t-node"), ("finds the queue empty", "t-sub")])
dl.lines(256, 186, [("the physics thread", "t-sub"), ("took the request", "t-sub")])

STYLE = """<style>
.md-typeset .th-figs{display:flex;flex-wrap:wrap;justify-content:center;gap:.8rem 1.6rem;margin:1em 0}
.md-typeset .th-figs figure{margin:0;flex:1 1 300px;max-width:380px}
.md-typeset .th-figs figcaption{flex-basis:100%;max-width:none;margin:0}
.th-fig{width:100%;height:auto;font-family:var(--md-text-font-family,Roboto),sans-serif;--th-cb:#2a78d6;--th-bad:#c62828}
[data-md-color-scheme=slate] .th-fig{--th-cb:#5a9cf0;--th-bad:#ef6a6a}
.th-fig text{fill:var(--md-default-fg-color)}
.th-fig .t-title{font-size:12.5px;font-weight:700}
.th-fig .t-num{font-size:14px;font-weight:700;fill:var(--th-cb)}
.th-fig .t-node{font-size:10.5px}
.th-fig .t-lane{font-size:10.5px;font-weight:700;fill:var(--md-default-fg-color--light)}
.th-fig .t-mono,.th-fig .t-mono-bad{font-family:var(--md-code-font-family,"Roboto Mono"),monospace;font-size:10px}
.th-fig .t-mono-bad{fill:var(--th-bad)}
.th-fig .t-sub,.th-fig .t-flow{font-size:9.5px;fill:var(--md-default-fg-color--light)}
.th-fig .t-flow{paint-order:stroke;stroke:var(--md-default-bg-color);stroke-width:3px;stroke-linejoin:round}
.th-fig .t-bad,.th-fig .t-bad-b{font-size:10.5px;fill:var(--th-bad)}
.th-fig .t-bad-b{font-weight:700}
.th-fig .box{fill:var(--md-default-bg-color);stroke:var(--md-default-fg-color--light);stroke-width:1}
.th-fig .queue{fill:var(--md-code-bg-color);stroke:var(--md-default-fg-color--light);stroke-width:1;stroke-dasharray:4 3}
.th-fig .lane{fill:var(--md-default-fg-color--lightest);stroke:none}
.th-fig .cb{fill:var(--md-default-bg-color);stroke:var(--th-cb);stroke-width:1.6}
.th-fig .cb-edge{fill:var(--md-default-bg-color);stroke:var(--th-cb);stroke-width:1.6}
.th-fig .idle{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1;stroke-dasharray:3 3}
.th-fig .stuck{fill:none;stroke:var(--th-bad);stroke-width:1.4;stroke-dasharray:5 3}
.th-fig .frame{fill:var(--md-default-bg-color);stroke:var(--md-default-fg-color--light);stroke-width:1}
.th-fig .frame-open{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1;stroke-dasharray:2 3}
.th-fig .a{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1.3}
.th-fig .h{fill:var(--md-default-fg-color--light)}
.th-fig .ab{fill:none;stroke:var(--th-cb);stroke-width:1.6}
.th-fig .h-b{fill:var(--th-cb)}
.th-fig .time{fill:none;stroke:var(--md-default-fg-color--lighter);stroke-width:1.2}
.th-fig .h-t{fill:var(--md-default-fg-color--lighter)}
</style>"""

CAPTION = ("Time runs downward. Blue: the controller manager's <code>switch_controller</code> "
           "callback, and where its request goes. In case 1, a ROS spinner thread runs the callback, and the "
           "physics thread keeps stepping. In case 2, <code>ros::spinOnce()</code> runs the callback "
           "in the physics thread, which then waits for an <code>update()</code> that only it can run.")

out = (STYLE + '\n<div class="th-figs">\n'
       + f'<figure>\n{ok.svg()}\n</figure>\n<figure>\n{dl.svg()}\n</figure>\n'
       + f'<figcaption>{CAPTION}</figcaption>\n</div>\n')
open("includes/imu-sim-threads.html", "w").write(out)
