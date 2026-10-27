"""Generate the figures on docs/theory/modeling-control/wheel-dynamics-simulation.md, as inline
SVG that follows the site's light and dark colors:

- includes/wheel-dynamics-contacts.html: where a cylinder and a sphere touch the ground (front
  view of DiffBot's drive wheels, to scale), and experiment E1: the true turn over the odometry's
  turn with each collision, against the ratio that outer-edge contacts predict. It has the styles
  for all figures on the page;
- includes/wheel-dynamics-sinkage.html: experiment E2, how far Remo's wheels sink into the ground
  at rest, for three minDepth values;
- includes/wheel-dynamics-wheel-speeds.html: experiment E3, Remo's wheel speeds while it turns on
  the spot, with the reference contacts and with one contact setting changed;
- includes/wheel-dynamics-stability.html: the error of a P velocity loop on a free wheel, step by
  step, for four values of P h / J.

The experiments ran in diffbot's dev container image, with diffbot b896a3a and remo_description
ba0b2c5, one fresh simulation per run; their data is in scripts/figures/data/ (wheel-dynamics-runs.jsonl:
E1 and E3, wheel-dynamics-sinkage.jsonl: E2, wheel-dynamics-speeds.csv: E3's left turns). Each run
changes one setting of the reference model: DiffBot as merged (sphere collision, Gazebo's default
contacts) or Remo as merged (sphere, friction 1, kp 1e7, kd 1, minDepth 0, no fdir1). The series colors are the first three slots of a
categorical palette that was checked for color vision deficiencies, with its own steps for the
dark scheme; every series also has a direct label or its own marker shape, and the page has the
data as tables. Hovering a point shows its values.

Run from the repository root after changing a figure:

    python3 scripts/figures/wheel_dynamics_figures.py
"""

import csv
import json
import math
from statistics import mean

STYLE = """<style>
.md-typeset figure.wd-figure{margin:1em auto;width:100%;max-width:500px}
.wd-fig{width:100%;height:auto;font-family:var(--md-text-font-family,Roboto),sans-serif;--wd-s1:#2a78d6;--wd-s2:#eb6834;--wd-s3:#1baf7a}
[data-md-color-scheme=slate] .wd-fig{--wd-s1:#3987e5;--wd-s2:#d95926;--wd-s3:#199e70}
.wd-fig text{fill:var(--md-default-fg-color);font-size:12px}
.wd-fig .t-muted{fill:var(--md-default-fg-color--light);font-size:11px}
.wd-fig .t-head{font-size:12.5px;font-weight:700}
.wd-fig .t-halo{paint-order:stroke;stroke:var(--md-default-bg-color);stroke-width:3px;stroke-linejoin:round}
.wd-fig .grid{stroke:var(--md-default-fg-color--lightest);stroke-width:1;fill:none}
.wd-fig .axis{stroke:var(--md-default-fg-color--lighter);stroke-width:1;fill:none}
.wd-fig .ref{stroke:var(--md-default-fg-color--light);stroke-width:1;stroke-dasharray:4 3;fill:none}
.wd-fig .ln{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
.wd-fig .s1{stroke:var(--wd-s1)}.wd-fig .s2{stroke:var(--wd-s2)}.wd-fig .s3{stroke:var(--wd-s3)}
.wd-fig .f1{fill:var(--wd-s1)}.wd-fig .f2{fill:var(--wd-s2)}.wd-fig .f3{fill:var(--wd-s3)}
.wd-fig .mk{stroke:var(--md-default-bg-color);stroke-width:2}
.wd-fig .hit{fill:transparent;stroke:none}
.wd-fig .pt:hover .mk{stroke:var(--md-default-fg-color);stroke-width:1.5}
.wd-fig .band{fill:transparent}
.wd-fig .band:hover{fill:var(--md-default-fg-color--lightest)}
.wd-fig .part{fill:var(--md-default-fg-color--lightest);stroke:var(--md-default-fg-color--lighter);stroke-width:1}
.wd-fig .coll{fill:none;stroke:var(--md-default-fg-color--light);stroke-width:1.2;stroke-dasharray:3 2}
.wd-fig .ground{stroke:var(--md-default-fg-color--light);stroke-width:1.5}
.wd-fig .dim{stroke:var(--md-default-fg-color--light);stroke-width:1;fill:none}
.wd-fig .dim-head{fill:var(--md-default-fg-color--light)}
.wd-fig .press{stroke:var(--wd-s1);stroke-width:2;fill:none}
.wd-fig .press-head{fill:var(--wd-s1)}
</style>"""


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def f1(x):
    return f"{x:.1f}".rstrip("0").rstrip(".")


class Svg:
    def __init__(self, ident, w, h, title):
        self.ident, self.w, self.h, self.title, self.parts = ident, w, h, title, []

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, cls="", anchor="start"):
        c = f' class="{cls}"' if cls else ""
        self.add(f'<text{c} x="{f1(x)}" y="{f1(y)}" text-anchor="{anchor}">{esc(s)}</text>')

    def line(self, x1, y1, x2, y2, cls):
        self.add(f'<path class="{cls}" d="M{f1(x1)},{f1(y1)} L{f1(x2)},{f1(y2)}"/>')

    def poly(self, pts, cls):
        d = "M" + " L".join(f"{f1(x)},{f1(y)}" for x, y in pts)
        self.add(f'<path class="{cls}" d="{d}"/>')

    def marker(self, x, y, shape, fill, tip, visible=True):
        """A data point: its marker with a surface ring, a larger hit area and a tooltip."""
        if not visible:
            m = ""
        elif shape == "circle":
            m = f'<circle class="mk {fill}" cx="{f1(x)}" cy="{f1(y)}" r="4.5"/>'
        elif shape == "square":
            m = f'<rect class="mk {fill}" x="{f1(x - 4.5)}" y="{f1(y - 4.5)}" width="9" height="9" rx="1"/>'
        else:  # diamond
            m = f'<path class="mk {fill}" d="M{f1(x)},{f1(y - 5.5)} L{f1(x + 5.5)},{f1(y)} L{f1(x)},{f1(y + 5.5)} L{f1(x - 5.5)},{f1(y)} Z"/>'
        self.add(f'<g class="pt"><title>{esc(tip)}</title><circle class="hit" cx="{f1(x)}" cy="{f1(y)}" r="12"/>{m}</g>')

    def legend(self, x, y, items):
        """items: (shape or 'line' or 'ref', fill/stroke class, label); returns the end x."""
        for shape, cls, label in items:
            if shape == "line":
                self.line(x, y - 4, x + 16, y - 4, f"ln {cls}")
                x += 21
            elif shape == "ref":
                self.line(x, y - 4, x + 16, y - 4, "ref")
                x += 21
            else:
                self.add(self._legend_mark(x + 5, y - 4, shape, cls))
                x += 14
            self.text(x, y, label, "t-muted")
            x += 6.3 * len(label) + 14
        return x

    @staticmethod
    def _legend_mark(x, y, shape, fill):
        if shape == "circle":
            return f'<circle class="{fill}" cx="{f1(x)}" cy="{f1(y)}" r="4.5"/>'
        if shape == "square":
            return f'<rect class="{fill}" x="{f1(x - 4.5)}" y="{f1(y - 4.5)}" width="9" height="9" rx="1"/>'
        return f'<path class="{fill}" d="M{f1(x)},{f1(y - 5.5)} L{f1(x + 5.5)},{f1(y)} L{f1(x)},{f1(y + 5.5)} L{f1(x - 5.5)},{f1(y)} Z"/>'

    def svg(self):
        body = "\n".join(self.parts)
        defs = ""
        for cls, head in (("dim", "dim-head"), ("press", "press-head")):
            if f'class="{cls}"' in body:
                defs += (f'<marker id="{self.ident}-{cls}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">'
                         f'<path class="{head}" d="M0,0 L10,5 L0,10 z"/></marker>')
                ends = 'marker-start="url(#{0}-{1})" marker-end="url(#{0}-{1})"' if cls == "dim" else 'marker-end="url(#{0}-{1})"'
                body = body.replace(f'class="{cls}"', f'class="{cls}" ' + ends.format(self.ident, cls))
        return (f'<svg class="wd-fig" viewBox="0 0 {self.w} {self.h}" role="img" aria-labelledby="{self.ident}-t">'
                f'<title id="{self.ident}-t">{esc(self.title)}</title><defs>{defs}</defs>\n{body}\n</svg>')


def figure(svg, caption):
    return f'<figure class="wd-figure">\n{svg.svg()}\n<figcaption>{caption}</figcaption>\n</figure>\n'


# --- Where a cylinder and a sphere touch the ground -------------------------------------------
# DiffBot's measured geometry (diffbot_description/config/diffbot/front_wheel.yaml): wheel
# separation 0.138 m between the wheel centers, wheel width 0.025 m, radius 0.0333 m. 1 px = 1 mm.
L, WW, R = 138, 25, 33.3


def wheels_panel(s, x0, sphere):
    cx, ground, yc = x0 + 105, 160, 160 - R
    s.text(x0 + 105, 18, "Cylinder collision" if not sphere else "Sphere collision", "t-head", "middle")
    s.text(x0 + 105, 32, "hypothesis: contacts at the outer edges" if not sphere else "contact in the center plane", "t-muted", "middle")
    s.add(f'<rect class="part" x="{f1(cx - 40)}" y="{f1(yc - 13)}" width="80" height="26" rx="3"/>')
    for side in (-1, 1):
        c = cx + side * L / 2
        s.line(cx + side * 40, yc, c, yc, "axis")
        s.add(f'<rect class="part" x="{f1(c - WW / 2)}" y="{f1(ground - 2 * R)}" width="{WW}" height="{f1(2 * R)}" rx="4"/>')
        s.line(c, ground - 2 * R - 8, c, ground + 4, "ref")
        if sphere:
            s.add(f'<circle class="coll" cx="{f1(c)}" cy="{f1(yc)}" r="{R}"/>')
            px = c
        else:
            px = c + side * WW / 2
        # The wheel pushes the robot forward at its contact: an arrow out of the page is drawn
        # as the contact dot; its position is what decides the turn
        s.marker(px, ground, "circle", "f1",
                 f"{'Effective contact in the center plane' if sphere else 'Hypothesized effective contact at the outer edge'}: "
                 f"{abs(px - cx) / 1000:.4f} m from the robot's center")
    s.line(x0 + 8, ground, x0 + 202, ground, "ground")
    # Dimensions: between the wheel centers above, between the contacts below
    yt = ground - 2 * R - 16
    s.line(cx - L / 2, yt, cx + L / 2, yt, "dim")
    s.text(cx, yt - 5, "0.138 m between the centers", "t-muted t-halo", "middle")
    span = L if sphere else L + WW
    s.line(cx - span / 2, ground + 18, cx + span / 2, ground + 18, "dim")
    s.text(cx, ground + 33, f"{span / 1000:.3f} m between the contacts", "t-halo", "middle")


contacts = Svg("wd-contacts", 440, 205, "Front view of DiffBot's drive wheels with a cylinder and a sphere collision")
wheels_panel(contacts, 0, sphere=False)
wheels_panel(contacts, 230, sphere=True)
fig_contacts = figure(contacts, "Front view of DiffBot's drive wheels, to scale. The dots show where the wheels would have to "
                                "push to explain the turn ratios of experiment E1 below: for the cylinder, a hypothesis at its "
                                "outer edges, not a measured contact position. The dashed circle is the sphere collision; "
                                "changing it leaves the model's visual and cylinder inertia unchanged.")

# --- E1: true turn over the odometry's turn with each collision -------------------------------
RUNS = [json.loads(line) for line in open("scripts/figures/data/wheel-dynamics-runs.jsonl")]
GEOM = {"diffbot": (0.138, 0.025), "remo": (0.144, 0.025)}
ROWS = [("diffbot", "left"), ("diffbot", "right"), ("remo", "left"), ("remo", "right")]
NAMES = {"diffbot": "DiffBot", "remo": "Remo"}


def e1(robot, shape, turn):
    rs = [r for r in RUNS if r["experiment"] == "E1" and r["config"] == f"{robot}-{shape}"]
    return [(r["repeat"], r[turn]["true"]["turned"], r[turn]["odom"]["turned"]) for r in rs]


turn = Svg("wd-turn", 440, 214, "Experiment E1: the true turn over the odometry's turn with a cylinder and a sphere collision")
lo, hi = 0.80, 1.06
x_of = lambda v: 118 + (min(v, hi) - lo) / (hi - lo) * 300
top = 44
for v in (0.80, 0.85, 0.90, 0.95, 1.00, 1.05):
    turn.line(x_of(v), top - 8, x_of(v), top + 4 * 34 - 6, "grid")
    turn.text(x_of(v), top + 4 * 34 + 8, f"{v:.2f}", "t-muted", "middle")
turn.text(x_of(0.93), top + 4 * 34 + 24, "true turn ÷ odometry turn (1 = they agree)", "t-muted", "middle")
turn.line(x_of(1.0), top - 8, x_of(1.0), top + 4 * 34 - 6, "ref")
turn.text(x_of(1.0) + 4, top - 12, "center plane: 1", "t-muted")
turn.legend(118, 16, [("square", "f2", "cylinder"), ("circle", "f1", "sphere"), ("ref", "", "outer-edge hypothesis")])
for i, (robot, way) in enumerate(ROWS):
    y = top + 12 + i * 34
    turn.text(108, y + 4, f"{NAMES[robot]}, {way}", "", "end")
    sep, width = GEOM[robot]
    pred = sep / (sep + width)
    turn.line(x_of(pred), y - 11, x_of(pred), y + 11, "ref")
    for shape, mark, fill in (("cyl", "square", "f2"), ("ref", "circle", "f1")):
        for rep, true, odom in e1(robot, shape, way):
            ratio = abs(true) / abs(odom)
            x = x_of(ratio) + (rep - 2) * 2.5
            name = "cylinder" if shape == "cyl" else "sphere"
            turn.marker(x, y, mark, fill, f"{NAMES[robot]}, {way} turn, {name}, run {rep}: "
                                          f"{abs(true):.3f} / {abs(odom):.3f} rad = {ratio:.3f}")
            if ratio > hi:
                turn.text(x - 8, y - 9, f"{ratio:.2f} →", "t-muted t-halo", "end")
fig_turn = figure(turn, "Experiment E1: each point is one turn, the angle the robot turned in Gazebo divided by the angle its "
                        "odometry reports. Commanded to turn 1.5 rad on the spot (0.5 rad/s for 3 s) to the left and to the right, "
                        "three runs per shape, with only the drive wheels' collision changed. The dashed ticks are the ratios "
                        "if the wheels pushed at their outer edges: 0.138 / 0.163 m for DiffBot, 0.144 / 0.169 m for Remo.")
turn_rows = []
for robot, way in ROWS:
    for shape, name in (("cyl", "cylinder"), ("ref", "sphere")):
        rs = e1(robot, shape, way)
        ratios = [abs(t) / abs(o) for _, t, o in rs]
        trk = [r for r in RUNS if r["experiment"] == "E1" and r["config"] == f"{robot}-{shape}"]
        tracks = [abs(r[way]["wheels"][w]["mean"]) / abs(r[way]["wheels"][w]["cmd"]) * 100 for r in trk for w in ("left", "right")]
        turn_rows.append([NAMES[robot], way, name, ", ".join(f"{x:.3f}" for x in ratios),
                          f"{min(tracks):.0f} %" if round(min(tracks)) == round(max(tracks)) else f"{min(tracks):.0f} to {max(tracks):.0f} %"])

# --- E2: sinkage at rest -----------------------------------------------------------------------
SINKRUNS = [json.loads(line) for line in open("scripts/figures/data/wheel-dynamics-sinkage.jsonl")]
SINK = [("minDepth 10 mm", "remo-md10", "s1", "f1", "circle"),
        ("minDepth 1 mm", "remo-md1", "s2", "f2", "square"),
        ("minDepth 0", "remo-ref", "s3", "f3", "diamond")]
sink = Svg("wd-sink", 440, 236, "Experiment E2: how far Remo's wheels sink into the ground at rest, for three minDepth values")
x_of = lambda t: 48 + t / 90 * 330
y_of = lambda mm: 196 - mm / 10.5 * 160
for mm in (0, 2, 4, 6, 8, 10):
    sink.line(48, y_of(mm), 378, y_of(mm), "axis" if mm == 0 else "grid")
    sink.text(42, y_of(mm) + 3.5, f"{mm}", "t-muted", "end")
for t in (0, 30, 60, 90):
    sink.text(x_of(t), 211, f"{t} s", "t-muted", "middle")
sink.text(213, 228, "time at rest after the spawn", "t-muted", "middle")
sink.text(14, y_of(5), "mm", "t-muted", "middle")
sink.legend(48, 16, [(shape, fill, label) for label, _, _, fill, shape in SINK])
sink.line(48, y_of(10), 378, y_of(10), "ref")
sink.text(54, y_of(10) - 4, "configured 10 mm", "t-muted t-halo")
sink_series = {}
for label, key, cls, fill, shape in SINK:
    runs = [r for r in SINKRUNS if r["config"] == key]
    times = [smp["t"] for smp in runs[0]["samples"]]
    vals = [[v for r in runs for smp in r["samples"] if smp["t"] == t for v in (smp["left_mm"], smp["right_mm"])] for t in times]
    pts = [(t, mean(v), min(v), max(v)) for t, v in zip(times, vals)]
    sink_series[label] = pts
    sink.poly([(x_of(t), y_of(m)) for t, m, _, _ in pts], f"ln {cls}")
    for t, m, a, b in pts:
        sink.marker(x_of(t), y_of(m), shape, fill, f"{label}: {m:.2f} mm after {t:g} s (both wheels, two runs: {a:.2f} to {b:.2f} mm)",
                    visible=t in (5, 20, 40, 60, 90))
    t, m, _, _ = pts[-1]
    sink.text(x_of(t) + 10, y_of(m) + 4, f"{m:.1f} mm" if m >= 0.005 else "0 mm", "t-halo")
fig_sink = figure(sink, "Experiment E2: Remo at rest with its sphere collisions and compliant contacts, two runs per value; "
                        "the penetration is the wheel radius minus the wheel center's height. A positive minDepth isn't a "
                        "limit: the wheels crept to 8.3 mm with 10 mm configured, and to the configured 1 mm.")

# --- E3: wheel speeds with one contact setting changed -------------------------------------------
SPEED_PANELS = [
    ("remo-ref", "Reference: wheel friction 1, kp 1e7, kd 1"),
    ("remo-mu200", "Wheel friction 200 only"),
    ("remo-kp1e12", "Wheel contact stiffness kp 1e12 only"),
    ("remo-kd0", "Wheel contact damping kd 0 only"),
]
with open("scripts/figures/data/wheel-dynamics-speeds.csv") as f:
    speed_rows = list(csv.DictReader(f))


def tracking(config, way="left"):
    """Each wheel's mean share of the command and its standard deviation, over the runs (E3)."""
    rs = [r for r in RUNS if r["experiment"] == "E3" and r["config"] == config]
    out = {}
    for w in ("left", "right"):
        out[w] = (mean(abs(r[way]["wheels"][w]["mean"]) / abs(r[way]["wheels"][w]["cmd"]) * 100 for r in rs),
                  max(r[way]["wheels"][w]["sd"] for r in rs))
    ratio = [abs(r[way]["true"]["turned"]) / abs(r[way]["odom"]["turned"]) for r in rs]
    return out, ratio


def speed_figure():
    ph, gap, top0 = 80, 48, 40
    h = top0 + len(SPEED_PANELS) * (ph + gap) + 10
    s = Svg("wd-speeds", 440, h, "Experiment E3: Remo's wheel speeds while turning on the spot, with one contact setting changed")
    s.legend(48, 16, [("line", "s1", "left wheel"), ("line", "s2", "right wheel"), ("ref", "", "commanded")])
    t0, t1, vmax = -0.5, 4.0, 2.4
    x_of = lambda t: 48 + (t - t0) / (t1 - t0) * 382
    for i, (key, title) in enumerate(SPEED_PANELS):
        y0 = top0 + i * (ph + gap) + 30
        y_of = lambda v: y0 + ph / 2 - v / vmax * ph / 2
        data = [r for r in speed_rows if r["config"] == key and r["repeat"] == "1"]
        s.text(48, y0 - 20, title, "t-head")
        for v in (-2, -1, 0, 1, 2):
            s.line(48, y_of(v), 430, y_of(v), "axis" if v == 0 else "grid")
            if v in (-2, 0, 2):
                s.text(42, y_of(v) + 3.5, f"{v}", "t-muted", "end")
        if i == len(SPEED_PANELS) - 1:
            for t in range(0, 5):
                s.text(x_of(t), y0 + ph + 13, f"{t} s", "t-muted", "middle")
        on = [float(r["t"]) for r in data if float(r["cmd_right"]) != 0]
        cmd = max(float(r["cmd_right"]) for r in data)
        for sign in (-1, 1):
            s.poly([(x_of(t0), y_of(0)), (x_of(on[0]), y_of(0)), (x_of(on[0]), y_of(sign * cmd)),
                    (x_of(on[-1]), y_of(sign * cmd)), (x_of(on[-1]), y_of(0)), (x_of(t1), y_of(0))], "ref")
        for col, cls in (("left", "s1"), ("right", "s2")):
            s.poly([(x_of(float(r["t"])), y_of(max(-vmax, min(vmax, float(r[col]))))) for r in data
                    if t0 <= float(r["t"]) <= t1], f"ln {cls}")
        k = 0
        while t0 + k * 0.1 < t1 - 1e-9:
            a, b = t0 + k * 0.1, t0 + (k + 1) * 0.1
            mid = min(data, key=lambda r: abs(float(r["t"]) - (a + b) / 2))
            tip = (f"t {float(mid['t']):.2f} s: left {float(mid['left']):.2f}, right {float(mid['right']):.2f} rad/s, "
                   f"commanded {float(mid['cmd_left']):.2f} / {float(mid['cmd_right']):.2f}")
            s.add(f'<rect class="band" x="{f1(x_of(a))}" y="{f1(y0)}" width="{f1(x_of(b) - x_of(a))}" height="{ph}"><title>{esc(tip)}</title></rect>')
            k += 1
        st, ratio = tracking(key)
        s.text(48, y0 - 6, f"left / right wheel {st['left'][0]:.0f} % / {st['right'][0]:.0f} % of the command, true / odometry turn {mean(ratio):.2f}", "t-muted")
    s.text(14, top0 + 30 + ph / 2, "rad/s", "t-muted", "middle")
    return s


fig_speeds = figure(speed_figure(),
                    "Experiment E3: Remo turning on the spot to the left at 0.5 rad/s for 3 s, with its sphere "
                    "collisions, minDepth 0 and its gains (P 0.03, I 0.6); first of two runs per setting. Below each title: "
                    "the left / right wheel's mean speed as a share of the command over the last 2 s of the command, "
                    "and the true turn over the odometry's, both from the two runs. The settings are the wheels'; Gazebo combines them "
                    "with the ground's, so wheel friction 200 acts as the ground's 100 and 50, and wheel damping 0 gives a combined 1.")

# --- Stability of the P velocity loop -----------------------------------------------------------
# e_{k+1} = (1 - P h / J) e_k, from e_0 = 1. Remo: J = 1.7424e-5 kg m^2, h = 1 ms.
J, H = 1.7424e-5, 0.001
CASES = [(0.5, "decays"), (0.03 * H / J, "P 0.03 (Remo): decays"),
         (2.0, "P 0.035, the limit"), (0.06 * H / J, "P 0.06: grows")]
stab = Svg("wd-stab", 440, 296, "The error of a P velocity loop on a free wheel, step by step, for four values of P h / J")
for i, (g, what) in enumerate(CASES):
    px, py = 34 + (i % 2) * 216, 16 + (i // 2) * 142
    pw, ph, vmax = 168, 84, 2.6
    x_of = lambda k: px + k / 8 * pw
    y_of = lambda v: py + 26 + ph / 2 - v / vmax * ph / 2
    stab.text(px, py, f"P h / J = {g:.2f}", "t-head")
    stab.text(px, py + 13, what, "t-muted")
    for v in (-2, -1, 0, 1, 2):
        stab.line(px, y_of(v), px + pw, y_of(v), "axis" if v == 0 else "grid")
        if i % 2 == 0 and v in (-2, 0, 2):
            stab.text(px - 6, y_of(v) + 3.5, f"{v}", "t-muted", "end")
    if i >= 2:
        for k in (0, 4, 8):
            stab.text(x_of(k), py + ph + 40, f"step {k}", "t-muted", "middle")
    pts, e = [], 1.0
    for k in range(9):
        pts.append((k, e))
        e *= 1 - g
    shown = [(k, v) for k, v in pts if abs(v) <= vmax]
    stab.poly([(x_of(k), y_of(v)) for k, v in shown], "ln s1")
    for k, v in shown:
        stab.marker(x_of(k), y_of(v), "circle", "f1", f"step {k}: error {v:.3f} of the first")
    if len(shown) < len(pts):
        k, v = pts[len(shown)]
        stab.text(x_of(k) + 4, y_of(vmax if v > 0 else -vmax) + (8 if v > 0 else -2),
                  f"step {k}: {v:.1f}", "t-muted t-halo")
fig_stab = figure(stab, "The error after each 1 ms step, as a share of the first, for a free wheel with only a P gain: "
                        "it is multiplied by 1 - P h / J in every step. Above P h / J = 2, its magnitude grows.")


def data_block(header, rows, note=""):
    """The figure's data as a table in a collapsed block, the figure's text equivalent."""
    out = '\n??? info "The data"\n\n'
    if note:
        out += f"    {note}\n\n"
    out += "    | " + " | ".join(header) + " |\n    |" + "---|" * len(header) + "\n"
    return out + "".join("    | " + " | ".join(r) + " |\n" for r in rows) + "\n"



contacts_data = data_block(
    ["Robot", "Turn", "Collision", "True / odometry turn, three runs", "Wheel speeds, share of the command"],
    turn_rows,
    "Experiment E1. Each run changes only the drive wheels' collision of the robot's reference model; the wheel "
    "speeds are each wheel's mean over the last 2 s of the command, from the joint states at 50 Hz.")
sink_times = [t for t, *_ in next(iter(sink_series.values()))]
sink_data = data_block(["Time at rest", *sink_series],
                       [[f"{t:g} s", *[f"{pts[i][1]:.2f} mm ({pts[i][2]:.2f} to {pts[i][3]:.2f})" for pts in sink_series.values()]]
                        for i, t in enumerate(sink_times)],
                       "Experiment E2. The mean of both wheels in two runs, with their range; a negative value would be clearance.")
stab_data = data_block(["Step", *[f"P h / J = {g:.2f}" for g, _ in CASES]],
                       [[f"{k}", *[f"{(1 - g) ** k:.3f}" for g, _ in CASES]] for k in range(9)])
e3_rows = []
for key, title in SPEED_PANELS + [("remo-combined", "Wheel friction 200, kp 1e12 and kd 0 together")]:
    st, ratio = tracking(key)
    e3_rows.append([title, f"{st['left'][0]:.0f} %, sd up to {st['left'][1]:.3f} rad/s",
                    f"{st['right'][0]:.0f} %, sd up to {st['right'][1]:.3f} rad/s", ", ".join(f"{r:.3f}" for r in ratio)])
speeds_data = data_block(
    ["Contact setting", "Left wheel", "Right wheel", "True / odometry turn, two runs"], e3_rows,
    "Experiment E3. Two runs per setting. The joint states come at 50 Hz: an oscillation faster than "
    "25 Hz shows up in the traces as a slower one or not at all. Every sample of the left turns is in "
    "scripts/figures/data/wheel-dynamics-speeds.csv.")

open("includes/wheel-dynamics-contacts.html", "w").write(STYLE + "\n" + fig_contacts + fig_turn + contacts_data)
open("includes/wheel-dynamics-sinkage.html", "w").write(fig_sink + sink_data)
open("includes/wheel-dynamics-wheel-speeds.html", "w").write(fig_speeds + speeds_data)
open("includes/wheel-dynamics-stability.html", "w").write(fig_stab + stab_data)
