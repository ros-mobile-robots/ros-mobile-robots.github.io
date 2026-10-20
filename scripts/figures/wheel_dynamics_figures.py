"""Generate the figures on docs/theory/modeling-control/wheel-dynamics-simulation.md, as inline
SVG that follows the site's light and dark colors:

- includes/wheel-dynamics-contacts.html: where a cylinder and a sphere touch the ground (front
  view of DiffBot's drive wheels, to scale), and the measured turn of both robots with each
  collision, against the turn that the contact positions predict. It has the styles for all
  figures on the page;
- includes/wheel-dynamics-wheel-speeds.html: Remo's wheel speeds while it turns on the spot, for
  three wheel models, from scripts/figures/data/remo-wheel-speeds.csv;
- includes/wheel-dynamics-sinkage.html: how far Remo's wheels sink into the ground at rest, for
  three minDepth values;
- includes/wheel-dynamics-stability.html: the error of a P velocity loop on a free wheel, step by
  step, for four values of P h / J.

The measurements come from the sweeps for remo_description#2 and diffbot#120, in diffbot's dev
container image; each data list names its run. The series colors are the first three slots of a
categorical palette that was checked for color vision deficiencies, with its own steps for the
dark scheme; every series also has a direct label or its own marker shape, and the page has the
data as tables. Hovering a point shows its values.

Run from the repository root after changing a figure:

    python3 scripts/figures/wheel_dynamics_figures.py
"""

import csv
import math

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
    s.text(x0 + 105, 32, "contacts at the outer edges" if not sphere else "contact in the center plane", "t-muted", "middle")
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
                 f"Effective contact {'in the center plane' if sphere else 'at the outer edge'}: "
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
fig_contacts = figure(contacts, "Front view of DiffBot's drive wheels, to scale. The dots show the effective contact "
                                "positions that explain the turn ratios below. The dashed circle is the sphere collision; "
                                "changing it leaves the model's visual and cylinder inertia unchanged.")

# --- Measured turn with each collision -------------------------------------------------------
# True turn / odometry turn, turning 1.5 rad on the spot. Predicted for contacts at the outer
# edges: separation / (separation + width); for the center plane: 1.
# DiffBot cylinder: diffbot#119's head with the measured geometry, before #120 (1.14 / 1.35 rad).
# DiffBot sphere: tests13, three suites at diffbot#122 (1.470-1.473 / 1.499-1.501 rad).
# Remo cylinder: sweep S2, cylinder with minDepth 0 and remo_description#2's contacts
# (1.278-1.279 / 1.500-1.501 rad). Remo sphere: tests13 (1.497-1.499 / 1.497-1.499 rad).
TURNS = [
    ("DiffBot", 0.138, 0.025, (1.14, 1.35), (1.472, 1.500)),
    ("Remo", 0.144, 0.025, (1.2785, 1.5005), (1.498, 1.498)),
]
turn = Svg("wd-turn", 440, 168, "Measured true turn over the odometry's turn, with a cylinder and a sphere collision")
x_of = lambda v: 110 + (v - 0.80) / 0.22 * 310
top, bottom = 44, 128
for v in (0.80, 0.85, 0.90, 0.95, 1.00):
    turn.line(x_of(v), top - 8, x_of(v), bottom, "grid")
    turn.text(x_of(v), bottom + 14, f"{v:.2f}", "t-muted", "middle")
turn.text(x_of(0.91), bottom + 30, "true turn / odometry turn", "t-muted", "middle")
turn.line(x_of(1.0), top - 8, x_of(1.0), bottom, "ref")
turn.text(x_of(1.0) - 4, top - 12, "center plane: 1", "t-muted", "end")
turn.legend(110, 16, [("square", "f2", "cylinder"), ("circle", "f1", "sphere"), ("ref", "", "predicted")])
for i, (name, sep, width, cyl, sph) in enumerate(TURNS):
    y = top + 20 + i * 44
    turn.text(100, y + 4, name, "", "end")
    pred = sep / (sep + width)
    turn.line(x_of(pred), y - 12, x_of(pred), y + 12, "ref")
    turn.text(x_of(pred), y - 15, f"outer edges: {pred:.3f}", "t-muted t-halo", "middle")
    rc, rs = cyl[0] / cyl[1], sph[0] / sph[1]
    turn.line(x_of(rc), y, x_of(rs), y, "axis")
    turn.marker(x_of(rc), y, "square", "f2", f"{name}, cylinder: {cyl[0]:.3f} / {cyl[1]:.3f} rad = {rc:.3f}")
    turn.marker(x_of(rs), y, "circle", "f1", f"{name}, sphere: {sph[0]:.3f} / {sph[1]:.3f} rad = {rs:.3f}")
    turn.text(x_of(rc), y + 20, f"{rc:.3f}", "t-halo", "middle")
    turn.text(x_of(rs) - 2, y + 20, f"{rs:.3f}", "t-halo", "middle")
fig_turn = figure(turn, "The true turn over the odometry's turn, when commanding a 1.5 rad turn on the spot. With cylinders, both "
                        "robots turned as their contacts at the outer edges predict (dashed); with spheres, as "
                        "their wheel centers do.")

# --- Wheel speeds while turning ----------------------------------------------------------------
SPEED_PANELS = [
    ("cylinder-old-contacts", "Cylinder, friction 200, kp 1e12, kd 0 (old model)"),
    ("sphere-old-contacts", "Sphere, the same contacts, minDepth 0"),
    ("final", "Sphere, friction 1, kp 1e7, kd 1, minDepth 0 (now)"),
]


def speed_stats(data):
    """Mean share of the command and standard deviation of each wheel, in the last 2 s of the command."""
    on = [float(r["t"]) for r in data if float(r["cmd_right"]) != 0]
    cmd = max(float(r["cmd_right"]) for r in data)
    win = [r for r in data if on[0] + 1.0 <= float(r["t"]) <= on[-1]]
    out = {}
    for c in ("left", "right"):
        vals = [abs(float(r[c])) for r in win]
        mean = sum(vals) / len(vals)
        out[c] = (mean / cmd * 100, math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals)))
    return out


def speed_figure(rows):
    ph, gap, top0 = 92, 34, 40
    h = top0 + len(SPEED_PANELS) * (ph + gap) + 10
    s = Svg("wd-speeds", 440, h, "Remo's wheel speeds while turning on the spot, for three wheel models")
    s.legend(48, 16, [("line", "s1", "left wheel"), ("line", "s2", "right wheel"), ("ref", "", "commanded")])
    t0, t1, vmax = -0.5, 4.0, 3.2
    x_of = lambda t: 48 + (t - t0) / (t1 - t0) * 382
    for i, (key, title) in enumerate(SPEED_PANELS):
        y0 = top0 + i * (ph + gap) + 16
        y_of = lambda v: y0 + ph / 2 - v / vmax * ph / 2
        data = [r for r in rows if r["config"] == key]
        s.text(48, y0 - 6, title, "t-head")
        for v in (-3, -2, -1, 0, 1, 2, 3):
            s.line(48, y_of(v), 430, y_of(v), "axis" if v == 0 else "grid")
            if v in (-3, 0, 3):
                s.text(42, y_of(v) + 3.5, f"{v}", "t-muted", "end")
        for t in range(0, 5):
            if i == len(SPEED_PANELS) - 1:
                s.text(x_of(t), y0 + ph + 13, f"{t} s", "t-muted", "middle")
        on = [float(r["t"]) for r in data if float(r["cmd_right"]) != 0]
        cmd = max(float(r["cmd_right"]) for r in data)
        for sign in (-1, 1):
            s.poly([(x_of(t0), y_of(0)), (x_of(on[0]), y_of(0)), (x_of(on[0]), y_of(sign * cmd)),
                    (x_of(on[-1]), y_of(sign * cmd)), (x_of(on[-1]), y_of(0)), (x_of(t1), y_of(0))], "ref")
        for col, cls in (("left", "s1"), ("right", "s2")):
            s.poly([(x_of(float(r["t"])), y_of(max(-vmax, min(vmax, float(r[col]))))) for r in data
                    if t0 <= float(r["t"]) <= t1], f"ln {cls}")
        # Hover bands of 0.1 s: the speeds in the middle of each band
        k = 0
        while t0 + k * 0.1 < t1 - 1e-9:
            a, b = t0 + k * 0.1, t0 + (k + 1) * 0.1
            mid = min(data, key=lambda r: abs(float(r["t"]) - (a + b) / 2))
            tip = (f"t {float(mid['t']):.2f} s: left {float(mid['left']):.2f}, right {float(mid['right']):.2f} rad/s, "
                   f"commanded {float(mid['cmd_left']):.2f} / {float(mid['cmd_right']):.2f}")
            s.add(f'<rect class="band" x="{f1(x_of(a))}" y="{f1(y0)}" width="{f1(x_of(b) - x_of(a))}" height="{ph}"><title>{esc(tip)}</title></rect>')
            k += 1
        st = speed_stats(data)
        s.text(428, y0 + 11, f"{st['left'][0]:.0f} % / {st['right'][0]:.0f} %, sd {st['left'][1]:.2f} / {st['right'][1]:.2f} rad/s",
               "t-muted t-halo", "end")
    s.text(14, top0 + 16 + ph / 2, "rad/s", "t-muted", "middle")
    return s


# --- Sinkage at rest --------------------------------------------------------------------------
# Wheel radius minus the wheel center's height above the ground, the mean of both drive wheels.
# minDepth 10 mm: Remo with remo_description#2's first version (cylinder, friction 1, kp 1e7,
# kd 1), run creep (0.5-60 s) and sweep S0 (90 s). minDepth 1 mm: the same with minDepth 0.001,
# sweep S1. minDepth 0: remo_description#2 now (sphere), tests13.
SINK = [
    ("minDepth 10 mm", "s1", "f1", "circle",
     [(0.5, 0.01), (1, 0.03), (2, 0.165), (5, 0.615), (10, 1.26), (20, 2.795), (40, 5.895), (60, 8.285), (90, 8.34)]),
    ("minDepth 1 mm", "s2", "f2", "square", [(5, 0.51), (20, 1.00), (60, 1.00), (90, 1.00)]),
    ("minDepth 0 (now)", "s3", "f3", "diamond", [(5, 0.0), (20, 0.0), (60, 0.0), (90, 0.0)]),
]
sink = Svg("wd-sink", 440, 236, "How far Remo's wheels sink into the ground at rest, for three minDepth values")
x_of = lambda t: 48 + t / 90 * 330
y_of = lambda mm: 196 - mm / 10.5 * 160
for mm in (0, 2, 4, 6, 8, 10):
    sink.line(48, y_of(mm), 378, y_of(mm), "axis" if mm == 0 else "grid")
    sink.text(42, y_of(mm) + 3.5, f"{mm}", "t-muted", "end")
for t in (0, 30, 60, 90):
    sink.text(x_of(t), 211, f"{t} s", "t-muted", "middle")
sink.text(213, 228, "time at rest after the spawn", "t-muted", "middle")
sink.text(14, y_of(5), "mm", "t-muted", "middle")
sink.legend(48, 16, [(shape, fill, label) for label, _, fill, shape, _ in SINK])
for mm, label in ((10, "minDepth 10 mm, configured"), (1, "1 mm")):
    sink.line(48, y_of(mm), 378, y_of(mm), "ref")
    sink.text(54, y_of(mm) - 4, label, "t-muted t-halo")
for label, cls, fill, shape, pts in SINK:
    sink.poly([(x_of(t), y_of(v)) for t, v in pts], f"ln {cls}")
    for t, v in pts:
        # The first seconds' points stay in the line, the tooltips and the table, without markers
        sink.marker(x_of(t), y_of(v), shape, fill, f"{label}: {v:.2f} mm after {t:g} s", visible=t >= 5)
    t, v = pts[-1]
    sink.text(x_of(t) + 10, y_of(v) + 4, f"{v:.1f} mm" if v else "0 mm", "t-halo")
fig_sink = figure(sink, "Remo at rest: the wheel radius minus the wheel center's height. A positive minDepth isn't a "
                        "limit: the wheels crept to 8.3 mm with 10 mm configured, and to the configured 1 mm. "
                        "The first two runs had remo_description#2's first version, with cylinders; "
                        "the third has its sphere.")

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
    ["Robot", "Collision", "True / odometry turn", "Ratio", "Predicted"],
    [r for name, sep, width, cyl, sph in TURNS for r in (
        [name, "cylinder", f"{cyl[0]:.3f} / {cyl[1]:.3f} rad", f"{cyl[0] / cyl[1]:.3f}",
         f"{sep:.3f} / {sep + width:.3f} m = {sep / (sep + width):.3f} (outer edges)"],
        [name, "sphere", f"{sph[0]:.3f} / {sph[1]:.3f} rad", f"{sph[0] / sph[1]:.3f}", "1 (center plane)"])],
    "DiffBot's cylinder: diffbot#119 with the measured geometry, before #120. Remo's cylinder: "
    "minDepth 0, with remo_description#2's contacts. Both spheres: remo_description#2 and "
    "diffbot#122, three suite runs.")
sink_data = data_block(["Time at rest", *[label for label, *_ in SINK]],
                       [[f"{t:g} s", *[next((f"{v:.2f} mm" for tt, v in pts if tt == t), "") for _, _, _, _, pts in SINK]]
                        for t in sorted({t for *_, pts in SINK for t, _ in pts})])
stab_data = data_block(["Step", *[f"P h / J = {g:.2f}" for g, _ in CASES]],
                       [[f"{k}", *[f"{(1 - g) ** k:.3f}" for g, _ in CASES]] for k in range(9)])


with open("scripts/figures/data/remo-wheel-speeds.csv") as f:
    speed_rows = list(csv.DictReader(f))
speeds_data = data_block(
    ["Wheel model", "Left wheel", "Right wheel"],
    [[title, *[f"{st[c][0]:.0f} %, sd {st[c][1]:.2f} rad/s" for c in ("left", "right")]]
     for key, title in SPEED_PANELS for st in [speed_stats([r for r in speed_rows if r["config"] == key])]],
    "The mean speed as a share of the command, and its standard deviation, in the last 2 s of "
    "the command. Every sample is in scripts/figures/data/remo-wheel-speeds.csv.")
fig_speeds = figure(speed_figure(speed_rows),
                    "Remo turning on the spot at 0.5 rad/s for 3 s, with diffbot's gains for Remo (P 0.03, I 0.6). "
                    "Top right in each panel: the left / right wheel's mean speed as a share of the command, and their "
                    "standard deviations, in the last 2 s of the command.")

open("includes/wheel-dynamics-contacts.html", "w").write(STYLE + "\n" + fig_contacts + fig_turn + contacts_data)
open("includes/wheel-dynamics-wheel-speeds.html", "w").write(fig_speeds + speeds_data)
open("includes/wheel-dynamics-sinkage.html", "w").write(fig_sink + sink_data)
open("includes/wheel-dynamics-stability.html", "w").write(fig_stab + stab_data)
