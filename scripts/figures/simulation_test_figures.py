"""Generate the figures on docs/development/simulation-test.md and
docs/theory/state-estimation/robot-motion.md, as inline SVG that follows the site's light and
dark colors:

- includes/simulation-test-figures.html: the two ros_control figures (simulation test and real
  robot) in two content tabs, and the styles for all figures;
- includes/simulation-test-relative-pose.html: the relative pose that test_drive.py's
  relative_pose() computes. It uses the styles from the first include, on the same page;
- includes/simulation-test-wheel-radius.html: why the odometry alone can miss a wrong wheel radius,
  also with the styles from the first include;
- includes/odometry-motion-model.html: the motion between two poses in the odometry motion model
  of Probabilistic Robotics, for Robot Motion, with its own copy of the styles.

The tabs are written as HTML, in the markup pymdownx.tabbed produces with alternate_style:
inside a "===" tab the include would be indented, and Markdown would wrap the SVG in a
paragraph instead of passing it through as an HTML block. Material's JavaScript handles
every element with data-tabs, so the tabs work like the ones written in Markdown.

Run from the repository root after changing a figure:

    python3 scripts/figures/simulation_test_figures.py
"""

import math

W = 360


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Fig:
    def __init__(self, h, title):
        self.h, self.title, self.parts = h, title, []

    def box(self, x, y, w, h, cls, lines=(), label=None, label_pos="bl"):
        self.parts.append(f'<rect class="{cls}" x="{x}" y="{y}" width="{w}" height="{h}" rx="{6 if cls.startswith("c-") else 4}"/>')
        n = len(lines)
        for i, (txt, tcls) in enumerate(lines):
            ty = y + h / 2 + (i - (n - 1) / 2) * 13 + 4
            self.parts.append(f'<text class="{tcls}" x="{x + w / 2}" y="{ty}" text-anchor="middle">{esc(txt)}</text>')
        if label:
            if label_pos == "bl":
                self.parts.append(f'<text class="t-cont" x="{x + 8}" y="{y + h - 7}">{esc(label)}</text>')
            elif label_pos == "tl":
                self.parts.append(f'<text class="t-cont" x="{x + 8}" y="{y + 13}">{esc(label)}</text>')

    def path(self, d, cls="a"):
        self.parts.append(f'<path class="{cls}" d="{d}"/>')

    def text(self, x, y, s, cls="t-topic", anchor="start"):
        self.parts.append(f'<text class="{cls}" x="{round(x, 1)}" y="{round(y, 1)}" text-anchor="{anchor}">{esc(s)}</text>')

    def sub(self, x, y, s, subscript, cls="t-pt", anchor="start"):
        """Text with a subscript, such as the delta with rot1."""
        self.parts.append(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}">{esc(s)}'
                          f'<tspan class="t-subscript" dy="3">{esc(subscript)}</tspan></text>')

    def dot(self, x, y, r=4.5):
        self.parts.append(f'<circle class="dot" cx="{x}" cy="{y}" r="{r}"/>')

    def svg(self, ident):
        body = "\n".join(self.parts)
        # Arrowheads for the arrow classes the figure uses: a and the dotted am2 (gray), ab and the
        # thinner ab2 (blue)
        markers = ""
        for cls, head in (("a", "a-head"), ("am2", "a-head"), ("ab", "a-head-b"), ("ab2", "a-head-b")):
            if f'class="{cls}"' in body:
                markers += (f'<marker id="{ident}-{cls}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                            f'<path class="{head}" d="M0,0 L10,5 L0,10 z"/></marker>')
                body = body.replace(f'class="{cls}"', f'class="{cls}" marker-end="url(#{ident}-{cls})"')
        return (f'<svg class="rc-fig" viewBox="0 0 {W} {self.h}" role="img" aria-labelledby="{ident}-t">'
                f'<title id="{ident}-t">{esc(self.title)}</title>'
                f'<defs>{markers}</defs>\n' + body + '\n</svg>')


def controllers(f):
    """Controller manager with both controllers, identical in both figures (y 160-258). The
    configuration files are on two lines right of the odom arrow (x 160), so no arrow crosses them."""
    f.box(30, 160, 304, 98, "c-shared", label="Controller manager")
    f.text(166, 173, "diffbot_control.yaml", "t-yaml")
    f.text(166, 185, "+ geometry.yaml", "t-yaml")
    f.box(38, 192, 146, 44, "n", [("mobile_base_controller", "t-node"), ("diff_drive_controller", "t-sub")])
    f.box(190, 192, 138, 44, "n", [("joint_state_controller", "t-node")])


def hw_arrows(f):
    """Controllers <-> hardware interface (y 236-290)."""
    f.path("M160,236 V290")  # commands down
    f.text(154, 270, "velocity", "t-flow", "end")
    f.text(154, 282, "commands", "t-flow", "end")
    f.path("M176,290 V236")  # states up to mobile_base_controller
    f.path("M262,290 V236")  # states up to joint_state_controller
    f.text(219, 270, "positions,", "t-flow", "middle")
    f.text(219, 282, "velocities", "t-flow", "middle")


def top(f, left, right, branch):
    """External nodes and topics above the controllers."""
    f.box(30, 24, 140, 32, "n", [(left, "t-mono" if left.endswith(".py") else "t-node")])
    f.box(200, 24, 140, 32, "n", [(right, "t-mono")])
    f.path("M100,56 V192")  # cmd_vel
    f.text(106, 96, "cmd_vel")
    f.path("M160,192 V56")  # odom
    if branch:
        f.path("M160,118 H212 V56")
        f.text(166, 112, "odom")
    else:
        f.text(166, 112, "odom")
    f.path("M300,192 V56")  # joint_states
    f.text(294, 86, "joint_states", anchor="end")


# Simulation test
sim = Fig(536, "Simulation test: ros_control inside the Gazebo server")
sim.box(4, 4, 352, 64, "c-test", label="rostest: sim_drive_diffbot.test", label_pos="tl")
sim.box(4, 132, 352, 400, "c-proc", label="Gazebo server, ROS node /gazebo")
sim.box(18, 146, 324, 272, "c-plugin", label="gazebo_ros_control plugin")
top(sim, "test_drive.py", "publishtest", branch=True)
controllers(sim)
hw_arrows(sim)
sim.box(38, 290, 290, 64, "n", [("Hardware interface", "t-node"), ("DefaultRobotHWSim", "t-mono"), ("readSim(), writeSim()", "t-sub"),
                                 ("config/<model>/pid.yaml", "t-yaml")])
sim.path("M200,444 V354")
sim.text(194, 384, "joint states", "t-flow", "end")
sim.path("M270,354 V444")
sim.text(276, 384, "commands", "t-flow")
sim.box(38, 444, 290, 48, "hw", [("Physics", "t-node"), ("robot model from the URDF", "t-sub")])
sim.path("M38,468 H12 V40 H30")
sim.text(16, 86, "model_states")
sim.text(16, 98, "(true pose)", "t-flow")

# Real robot
real = Fig(640, "Real robot: ros_control inside the diffbot_base node")
real.box(4, 132, 352, 240, "c-proc", label="ROS node /diffbot/diffbot_base")
top(real, "teleop, navigation", "robot_state_publisher", branch=False)
controllers(real)
hw_arrows(real)
real.box(38, 290, 290, 56, "n", [("Hardware interface", "t-node"), ("DiffBotHWInterface", "t-mono"), ("read(), write()", "t-sub")])
real.path("M200,446 V346")
real.text(194, 404, "measured_joint_states", "t-topic", "end")
real.path("M290,346 V446")
real.text(296, 398, "wheel_cmd_")
real.text(296, 410, "velocities")
real.box(150, 446, 190, 30, "n", [("rosserial node", "t-node")])
real.path("M245,476 V526", "a2")
real.text(251, 505, "USB serial", "t-flow")
real.box(4, 516, 352, 52, "c-proc", label="Teensy firmware", label_pos="bl")
real.box(150, 526, 190, 32, "n", [("PID loop per wheel", "t-node")])
real.path("M300,558 V598")
real.box(232, 598, 124, 30, "hw", [("motor driver, motors", "t-node")])
real.path("M200,598 V558")
real.box(110, 598, 110, 30, "hw", [("wheel encoders", "t-node")])

STYLE = """<style>
.md-typeset .rc-figs figure{margin:.6em auto 0;width:100%;max-width:480px}
.rc-fig{width:100%;height:auto;font-family:var(--md-text-font-family,Roboto),sans-serif}
.rc-fig .c-proc,.rc-fig .c-plugin,.rc-fig .c-test{fill:var(--md-default-fg-color--lightest);stroke:var(--md-default-fg-color--lighter);stroke-width:1}
.rc-fig .c-plugin{fill:none;stroke-dasharray:4 3}
.rc-fig .c-test{fill:none}
.rc-fig .c-shared{fill:rgba(68,138,255,.12);stroke:#448aff;stroke-width:1.3}
.rc-fig .n{fill:var(--md-default-bg-color);stroke:var(--md-default-fg-color--light);stroke-width:1}
.rc-fig .hw{fill:var(--md-code-bg-color);stroke:var(--md-default-fg-color--light);stroke-width:1}
.rc-fig text{fill:var(--md-default-fg-color)}
.rc-fig .t-node{font-size:11px}
.rc-fig .t-mono,.rc-fig .t-topic,.rc-fig .t-yaml{font-family:var(--md-code-font-family,"Roboto Mono"),monospace;font-size:10px}
.rc-fig .t-sub,.rc-fig .t-flow{font-size:9.5px;fill:var(--md-default-fg-color--light)}
.rc-fig .t-cont{font-size:10px;font-weight:700;fill:var(--md-default-fg-color--light)}
.rc-fig .t-yaml{fill:#448aff;font-size:9.5px}
.rc-fig .t-topic,.rc-fig .t-flow{paint-order:stroke;stroke:var(--md-default-bg-color);stroke-width:3px;stroke-linejoin:round}
.rc-fig .a,.rc-fig .a2{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1.3}
.rc-fig .a2{stroke-dasharray:3 2}
.rc-fig .a-head{fill:var(--md-default-fg-color--light)}
.rc-fig .ab{fill:none;stroke:#448aff;stroke-width:1.8}
.rc-fig .ab2{fill:none;stroke:#448aff;stroke-width:1.2}
.rc-fig .a-head-b{fill:#448aff}
.rc-fig .am{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1;stroke-dasharray:2 3}
.rc-fig .am2{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1.3;stroke-dasharray:2 3}
.rc-fig .dot{fill:var(--md-default-bg-color);stroke:var(--md-default-fg-color);stroke-width:1.3}
.rc-fig .t-blue{font-size:11px;fill:#448aff;paint-order:stroke;stroke:var(--md-default-bg-color);stroke-width:3px;stroke-linejoin:round}
.rc-fig .t-pt{font-size:11px;paint-order:stroke;stroke:var(--md-default-bg-color);stroke-width:3px;stroke-linejoin:round}
.rc-fig .t-subscript{font-size:8.5px}
.md-typeset figure.rc-single{margin:1em auto;width:100%;max-width:420px}
</style>"""

TABS = [("Simulation test", sim.svg("rc-sim")), ("Real robot", real.svg("rc-real"))]
inputs = "".join(f'<input{" checked" if i == 0 else ""} id="__tabbed_rc_{i + 1}" name="__tabbed_rc" type="radio">'
                 for i in range(len(TABS)))
labels = "".join(f'<label for="__tabbed_rc_{i + 1}">{esc(label)}</label>' for i, (label, _) in enumerate(TABS))
blocks = "".join(f'<div class="tabbed-block">\n<figure>\n{svg}\n</figure>\n</div>\n' for _, svg in TABS)
out = (STYLE + f'\n<div class="tabbed-set tabbed-alternate rc-figs" data-tabs="rc:{len(TABS)}">{inputs}'
       + f'<div class="tabbed-labels">{labels}</div>\n<div class="tabbed-content">\n{blocks}</div>\n</div>\n')
open("includes/simulation-test-figures.html", "w").write(out)

# Robot Motion: the odometry motion model's rot1, trans, rot2
mv = Fig(262, "The motion between two poses as a rotation, a translation and a rotation")
O = (36, 228)
mv.path(f"M{O[0]},{O[1]} H340")  # x axis
mv.path(f"M{O[0]},{O[1]} V16")  # y axis
mv.text(338, 244, "x", "t-pt", "end")
mv.text(24, 24, "y", "t-pt", "end")
mv.text(O[0], 256, "x, y: the frame of the poses, for example the odometry frame", "t-sub")


def pt(p):
    return f"{p[0]:.1f},{p[1]:.1f}"


def along(p, deg, r):
    """The point at distance r from p in direction deg (counterclockwise from x; SVG's y points down)."""
    return (p[0] + r * math.cos(math.radians(deg)), p[1] - r * math.sin(math.radians(deg)))


def arc(p, deg0, deg1, r):
    """Counterclockwise arc around p from direction deg0 to deg1."""
    return f"M{pt(along(p, deg0, r))} A{r},{r} 0 0 0 {pt(along(p, deg1, r))}"


theta0, rot1, rot2 = 10, 35, 40
bearing, theta1 = theta0 + rot1, theta0 + rot1 + rot2
p0 = (84, 196)
p1 = along(p0, bearing, 150)
# Headings at the start and the end
mv.path(f"M{pt(p0)} L{pt(along(p0, theta0, 64))}")
mv.text(*along(p0, theta0 - 4, 76), "\u03b8\u2080", "t-pt")
mv.path(f"M{pt(p1)} L{pt(along(p1, theta1, 56))}")
mv.text(*along(p1, theta1 + 6, 62), "\u03b8\u2081", "t-pt", "end")
# The translation, and the direction it continues in, for rot2
mv.path(f"M{pt(along(p0, bearing, 6))} L{pt(along(p0, bearing, 145))}", "ab")
mv.path(f"M{pt(p1)} L{pt(along(p1, bearing, 44))}", "am")
mv.sub(*along(p0, bearing + 9, 92), "\u03b4", "trans", "t-blue", "end")
# The two rotations
mv.path(arc(p0, theta0, bearing, 42), "ab2")
mv.sub(*along(p0, theta0 + rot1 / 2, 50), "\u03b4", "rot1", "t-blue")
mv.path(arc(p1, bearing, theta1, 34), "ab2")
mv.sub(*along(p1, bearing + rot2 / 2, 42), "\u03b4", "rot2", "t-blue")
mv.dot(*p0)
mv.dot(*p1)
mv.text(p0[0] - 46, p0[1] + 20, "start (x\u2080, y\u2080, \u03b8\u2080)", "t-pt")
mv.text(p1[0] + 12, p1[1] + 14, "end (x\u2081, y\u2081, \u03b8\u2081)", "t-pt")

# Math in the captions as arithmatex spans, which the site's KaTeX script renders
caption = ("The motion from the start to the end pose in the odometry motion model: rotate by "
           '<span class="arithmatex">\\(\\delta_\\text{rot1}\\)</span>, drive straight for '
           '<span class="arithmatex">\\(\\delta_\\text{trans}\\)</span>, rotate by '
           '<span class="arithmatex">\\(\\delta_\\text{rot2}\\)</span>.')
open("includes/odometry-motion-model.html", "w").write(
    STYLE + f'\n<figure class="rc-single">\n{mv.svg("rc-motion")}\n<figcaption>{caption}</figcaption>\n</figure>\n')

# relative_pose() in test_drive.py: the end pose in the robot's frame at the start
rp = Fig(262, "The end pose relative to the start pose, in the robot's frame at the start")
O = (36, 240)
rp.path(f"M{O[0]},{O[1]} H340")  # x axis
rp.path(f"M{O[0]},{O[1]} V16")  # y axis
rp.text(338, 254, "x", "t-pt", "end")
rp.text(24, 24, "y", "t-pt", "end")
rp.text(O[0], 256, "x, y: the frame of the poses (odometry or Gazebo world)", "t-sub")
theta0, turned = 20, 55
forward, left = 180, 85
p0 = (100, 205)
foot = along(p0, theta0, forward)
p1 = along(foot, theta0 + 90, left)
# The robot's frame at the start: x_r along its heading, y_r to its left
rp.path(f"M{pt(along(p0, theta0, 6))} L{pt(along(p0, theta0, 240))}", "am")
rp.sub(*along(p0, theta0 - 4, 246), "x", "r", "t-pt")
rp.path(f"M{pt(along(p0, theta0 + 90, 6))} L{pt(along(p0, theta0 + 90, 150))}", "am")
rp.sub(*along(p0, theta0 + 94, 150), "y", "r", "t-pt", "end")
# theta0, the start heading in the frame of the poses
rp.path(f"M{pt(p0)} L{pt(along(p0, 0, 44))}", "am")
rp.path(arc(p0, 0, theta0, 34), "ab2")
rp.text(*along(p0, theta0 / 2 - 6, 50), "\u03b8\u2080", "t-pt")
# forward and left: the displacement along x_r and y_r
rp.path(f"M{pt(along(p0, theta0, 6))} L{pt(foot)}", "ab")
rp.text(*along(along(p0, theta0, forward / 2 + 20), theta0 - 90, 22), "forward", "t-blue", "middle")
rp.path(f"M{pt(foot)} L{pt(along(foot, theta0 + 90, left - 6))}", "ab")
rp.text(*along(along(foot, theta0 + 90, left / 2), theta0, 10), "left", "t-blue")
# turned: from the start heading (dotted, parallel to x_r) to the end heading
rp.path(f"M{pt(p1)} L{pt(along(p1, theta0, 50))}", "am")
rp.path(f"M{pt(p1)} L{pt(along(p1, theta0 + turned, 48))}")
rp.path(arc(p1, theta0, theta0 + turned, 32), "ab2")
rp.text(*along(p1, theta0 + turned / 2, 40), "turned", "t-blue")
rp.dot(*p0)
rp.dot(*p1)
rp.text(p0[0] - 40, p0[1] + 22, "start (x\u2080, y\u2080, \u03b8\u2080)", "t-pt")
rp.text(p1[0] - 10, p1[1] + 4, "end (x\u2081, y\u2081, \u03b8\u2081)", "t-pt", "end")

caption = ("The end pose relative to the start pose, in the robot's frame at the start "
           '(<span class="arithmatex">\\(x_r, y_r\\)</span>): how far the robot moved forward '
           "and to the left, and the angle it turned.")
open("includes/simulation-test-relative-pose.html", "w").write(
    f'<figure class="rc-single">\n{rp.svg("rc-relpose")}\n<figcaption>{caption}</figcaption>\n</figure>\n')

# Why the odometry alone can miss a wrong wheel radius: the controller uses its configured radius
# on the way out and on the way back, the model rolls on its own
wr = Fig(318, "The configured wheel radius on the way to the wheels and back to the odometry")
wr.box(90, 6, 180, 36, "n", [("commanded speed v", "t-node"), ("m/s, from cmd_vel", "t-sub")])
wr.path("M180,42 V84", "ab")
wr.sub(188, 67, "÷ r", "cfg", "t-blue")
wr.box(90, 84, 180, 36, "c-shared", [("wheel speed command", "t-node"), ("rad/s, diff_drive_controller", "t-sub")])
wr.path("M180,120 V162")
wr.text(188, 139, "wheel velocity control,", "t-flow")
wr.text(188, 151, "for the drive's duration", "t-flow")
wr.box(90, 162, 180, 36, "n", [("wheel rotation Δφ", "t-node"), ("rad, of the wheel joint", "t-sub")])
wr.path("M150,198 V216 H90 V258", "ab")
wr.sub(84, 242, "× r", "cfg", "t-blue", "end")
wr.path("M210,198 V216 H270 V258")
wr.sub(276, 236, "× r", "model", "t-pt")
wr.text(276, 249, "rolling, no slip", "t-flow")
wr.box(10, 258, 160, 44, "c-shared", [("odometry distance", "t-node"), ("diff_drive_controller", "t-sub")])
wr.box(190, 258, 160, 44, "hw", [("distance the model rolls", "t-node"), ("Gazebo physics, ideally", "t-sub")])

caption = ("The configured radius <span class=\"arithmatex\">\\(r_\\text{cfg}\\)</span> from "
           "<code>diffbot_control/config/&lt;model&gt;/geometry.yaml</code> is used twice on the way to the odometry, the model's "
           "radius <span class=\"arithmatex\">\\(r_\\text{model}\\)</span> from the URDF once on the "
           "way to the true pose.")
open("includes/simulation-test-wheel-radius.html", "w").write(
    f'<figure class="rc-single">\n{wr.svg("rc-radius")}\n<figcaption>{caption}</figcaption>\n</figure>\n')

# Turning on the spot: the robot turns about the midpoint of its axle, and base_footprint, which
# is behind it, moves on an arc. Top view of Remo, to scale (14 px per cm): wheel separation
# 14.4 cm, wheel diameter 6.6 cm, width 2.5 cm, base_footprint 3.6 cm behind the axle.
pv = Fig(300, "Turning on the spot: base_footprint moves on an arc around the axle's midpoint")
P = (180, 150)
SCALE, SEP, DIA, WID, D = 14, 14.4, 6.6, 2.5, 3.6
THETA = math.degrees(1.5)


def oriented_rect(c, heading, length, width):
    """A rectangle centered at c, its length along heading (degrees, counterclockwise from x)."""
    pts = [along(along(c, heading, sl * length / 2), heading + 90, sw * width / 2)
           for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    return "M" + " L".join(pt(p) for p in pts) + " Z"


# The wheels, the axle and an arrow to the front: solid at the start (facing up), dotted after
# turning 1.5 rad to the left. The arrow is beside the axle's midpoint, on the robot's right, so it
# turns with the robot and doesn't lie on the other pose's axle
for heading, cls, arrow in ((90, "n", "a"), (90 + THETA, "am", "am2")):
    for side in (1, -1):
        pv.path(oriented_rect(along(P, heading + 90 * side, SEP / 2 * SCALE), heading, DIA * SCALE, WID * SCALE), cls)
    pv.path(f"M{pt(along(P, heading + 90, SEP / 2 * SCALE - WID * SCALE / 2))} "
            f"L{pt(along(P, heading - 90, SEP / 2 * SCALE - WID * SCALE / 2))}", cls)
    side = along(P, heading - 90, 50)
    pv.path(f"M{pt(along(side, heading, 20))} L{pt(along(side, heading, 66))}", arrow)
front0 = along(along(P, 0, 50), 90, 66)
pv.text(front0[0] + 6, front0[1] + 8, "front at the start", "t-pt")
front1 = along(along(P, 90 + THETA - 90, 50), 90 + THETA, 66)
pv.text(front1[0] - 4, front1[1] - 8, "front after", "t-pt", "middle")
b0 = along(P, 270, D * SCALE)
b1 = along(P, 270 + THETA, D * SCALE)
pv.path(arc(P, 270, 270 + THETA, D * SCALE), "am")
pv.path(f"M{pt(P)} L{pt(b0)}", "am")
pv.path(f"M{pt(b0)} L{pt(along(b0, math.degrees(math.atan2(b0[1] - b1[1], b1[0] - b0[0])), math.dist(b0, b1) - 6))}", "ab2")
pv.dot(*P, 3.5)
pv.dot(*b0)
pv.dot(*b1)
pv.text(P[0] - 8, P[1] - 10, "axle's midpoint", "t-pt", "end")
pv.text(b0[0] - 10, b0[1] + 4, "base_footprint at the start", "t-pt", "end")
pv.path(f"M{pt(along(b1, -60, 6))} L{pt((252, 214))}", "am")
pv.text(256, 220, "after 1.5 rad", "t-pt")
pv.text(P[0] - 8, (P[1] + b0[1]) / 2 + 4, "3.6 cm", "t-pt", "end")
# Below the arc, left of the leader to "after 1.5 rad"
pv.text(b0[0] + 22, b0[1] + 18, "4.9 cm", "t-blue")
pv.text(P[0], 286, "Solid: start \u00b7 Dotted: after the turn", "t-node", "middle")
caption = ("Remo from above, with the wheel geometry to scale. In an ideal 1.5 rad turn to the left, the "
           "axle's midpoint stays where it is. <code>base_footprint</code> follows the dotted arc and ends "
           "up 4.9 cm from its start, as the blue arrow shows.")
open("includes/simulation-test-pivot.html", "w").write(
    f'<figure class="rc-single">\n{pv.svg("rc-pivot")}\n<figcaption>{caption}</figcaption>\n</figure>\n')
