"""Generate includes/simulation-test-figures.html: the two ros_control figures on
docs/development/simulation-test.md (simulation test and real robot), as inline SVG that
follows the site's light and dark colors.

Run from the repository root after changing a figure:

    python3 scripts/figures/simulation_test_figures.py
"""

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
        self.parts.append(f'<text class="{cls}" x="{x}" y="{y}" text-anchor="{anchor}">{esc(s)}</text>')

    def svg(self, ident):
        body = "\n".join(self.parts)
        return (f'<svg class="rc-fig" viewBox="0 0 {W} {self.h}" role="img" aria-labelledby="{ident}-t">'
                f'<title id="{ident}-t">{esc(self.title)}</title>'
                f'<defs><marker id="{ident}-arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
                f'<path class="a-head" d="M0,0 L10,5 L0,10 z"/></marker></defs>\n'
                + body.replace('class="a"', f'class="a" marker-end="url(#{ident}-arr)"') + '\n</svg>')


def controllers(f):
    """Controller manager with both controllers, identical in both figures (y 160-240)."""
    f.box(30, 160, 304, 86, "c-shared", label="Controller manager")
    f.text(174, 173, "diffbot_control.yaml", "t-yaml")
    f.box(38, 180, 146, 44, "n", [("mobile_base_controller", "t-node"), ("diff_drive_controller", "t-sub")])
    f.box(190, 180, 138, 44, "n", [("joint_state_controller", "t-node")])


def hw_arrows(f):
    """Controllers <-> hardware interface (y 224-290)."""
    f.path("M160,224 V290")  # commands down
    f.text(154, 258, "velocity", "t-flow", "end")
    f.text(154, 270, "commands", "t-flow", "end")
    f.path("M176,290 V224")  # states up to mobile_base_controller
    f.path("M262,290 V224")  # states up to joint_state_controller
    f.text(219, 258, "positions,", "t-flow", "middle")
    f.text(219, 270, "velocities", "t-flow", "middle")


def top(f, left, right, branch):
    """External nodes and topics above the controllers."""
    f.box(30, 24, 140, 32, "n", [(left, "t-mono" if left.endswith(".py") else "t-node")])
    f.box(200, 24, 140, 32, "n", [(right, "t-mono")])
    f.path("M100,56 V180")  # cmd_vel
    f.text(106, 96, "cmd_vel")
    f.path("M160,180 V56")  # odom
    if branch:
        f.path("M160,118 H212 V56")
        f.text(166, 112, "odom")
    else:
        f.text(166, 112, "odom")
    f.path("M300,180 V56")  # joint_states
    f.text(294, 86, "joint_states", anchor="end")


# Simulation test
sim = Fig(536, "Simulation test: ros_control inside the Gazebo server")
sim.box(4, 4, 352, 64, "c-test", label="rostest: sim_drive_diffbot.test", label_pos="tl")
sim.box(4, 132, 352, 400, "c-proc", label="Gazebo server, ROS node /gazebo")
sim.box(18, 146, 324, 272, "c-plugin", label="gazebo_ros_control plugin")
top(sim, "test_drive.py", "publishtest", branch=True)
controllers(sim)
hw_arrows(sim)
sim.box(38, 290, 290, 56, "n", [("Hardware interface", "t-node"), ("DefaultRobotHWSim", "t-mono"), ("readSim(), writeSim()", "t-sub")])
sim.path("M200,346 V444")
sim.text(194, 384, "commands", "t-flow", "end")
sim.path("M270,444 V346")
sim.text(276, 384, "joint states", "t-flow")
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
.rc-figs{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(340px,100%),1fr));gap:1rem;margin:1.5em 0}
.rc-figs figure{margin:0 auto;width:100%;max-width:480px}
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
</style>"""

out = (STYLE + '\n<div class="rc-figs">\n<figure>\n' + sim.svg("rc-sim")
       + '\n<figcaption>Simulation test (CI)</figcaption>\n</figure>\n<figure>\n' + real.svg("rc-real")
       + '\n<figcaption>Real robot</figcaption>\n</figure>\n</div>\n')
open("includes/simulation-test-figures.html", "w").write(out)
