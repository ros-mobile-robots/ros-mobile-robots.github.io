// "Three views of one drive": the interactive figure on the Simulation Test page.
// The markup, its texts and its numbers (robots, commands, checks, examples) come from
// includes/motion-lab.html, which scripts/figures/motion_lab.py writes. This file adds the
// behavior. The figure shows its static version until the first drawing here has succeeded.
import { threeViews, relativePose, predictChecks } from './diffdrive.js';

const SVG_NS = 'http://www.w3.org/2000/svg';
const SAMPLES = 150;  // Poses per drive, for the paths

const reducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;
const fixed = (value, digits = 3) => {
  const rounded = Math.abs(value) < 0.5 * 10 ** -digits ? 0 : value;
  return rounded.toFixed(digits).replace('-', '−');
};

function svgElement(name, attributes = {}, text) {
  const element = document.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attributes)) element.setAttribute(key, value);
  if (text !== undefined) element.textContent = text;
  return element;
}

function mount(root) {
  const data = JSON.parse(root.querySelector('.ix-data').textContent);
  const find = (selector) => root.querySelector(selector);
  const findAll = (selector) => [...root.querySelectorAll(selector)];
  const plot = find('.ix-plot');
  const sliders = Object.fromEntries(findAll('input[data-setting]').map((input) => [input.dataset.setting, input]));
  const state = { robot: data.default.robot, command: data.default.command, time: 1, step: 0 };
  const revealed = new Set([data.default.example]);  // The examples whose answer is open
  let frame = null, lastTick = 0, announceTimer = null, drive = null, svg = null;

  // --- The model's inputs, from the controls -------------------------------------------------
  const setting = (name) => Number(sliders[name].value);
  function inputs() {
    const robot = data.robots[state.robot], command = data.commands[state.command];
    return {
      robot, command,
      configuredRadius: robot.radius * setting('radius') / 100,
      configuredSeparation: robot.separation * setting('separation') / 100,
      contactSeparation: robot.separation + robot.wheelWidth * setting('contact') / 100,
      frameOffset: setting('offset') / 1000,
      tracking: setting('tracking') / 100,
    };
  }

  function compute() {
    const p = inputs();
    const views = threeViews({
      v: p.command.v, w: p.command.w, radius: p.robot.radius, separation: p.robot.separation,
      configuredRadius: p.configuredRadius, configuredSeparation: p.configuredSeparation,
      contactSeparation: p.contactSeparation, frameOffset: p.frameOffset, tracking: p.tracking,
    });
    const samples = [];
    for (let i = 0; i <= SAMPLES; i++) samples.push(views.at(p.command.duration * i / SAMPLES));
    const start = samples[0], end = samples[SAMPLES];
    const motion = {
      command: relativePose(start.command, end.command),
      odometry: relativePose(start.odometry, end.odometry),
      model: relativePose(start.frame, end.frame),
    };
    const checks = predictChecks(state.command, motion.odometry, motion.model, data.checks[state.command]);
    return { p, samples, motion, checks };
  }

  // --- Drawing -------------------------------------------------------------------------------
  // The drawing is in screen pixels: it is drawn again when the figure's width changes, so
  // lines and symbols keep their size on a phone.
  function draw() {
    const width = Math.max(240, Math.round(plot.clientWidth));
    const height = Math.round(width * data.aspect);
    const { p, samples } = drive;
    const robot = p.robot, pad = 14;
    const reach = Math.max(robot.separation / 2 + robot.wheelWidth, robot.radius + 0.06, p.frameOffset + 0.02) + 0.02;
    const points = samples.flatMap((s) => [s.command, s.odometry, s.frame, s.axle]);
    const xs = points.map((q) => q.x), ys = points.map((q) => q.y);
    const xMin = Math.min(...xs) - reach, xMax = Math.max(...xs) + reach;
    const yMin = Math.min(...ys) - reach, yMax = Math.max(...ys) + reach;
    // The robot's x axis (forward) points to the right of the page, its y axis (left) up
    const scale = Math.min((width - 2 * pad) / (xMax - xMin), (height - 2 * pad) / (yMax - yMin));
    const cx = (xMin + xMax) / 2, cy = (yMin + yMax) / 2;
    const X = (x) => width / 2 + (x - cx) * scale, Y = (y) => height / 2 - (y - cy) * scale;

    const next = svgElement('svg', {
      class: 'ix-live-plot', viewBox: `0 0 ${width} ${height}`, role: 'img',
      'aria-label': find('.ix-static title').textContent, 'aria-describedby': find('.ix-summary').id,
    });

    const gridStep = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5].find((step) => step * scale >= 22) || 1;
    const grid = svgElement('g', { class: 'ix-grid' });
    for (let x = Math.ceil((cx - width / 2 / scale) / gridStep) * gridStep; x < cx + width / 2 / scale; x += gridStep) {
      grid.append(svgElement('line', { x1: X(x), y1: 0, x2: X(x), y2: height }));
    }
    for (let y = Math.ceil((cy - height / 2 / scale) / gridStep) * gridStep; y < cy + height / 2 / scale; y += gridStep) {
      grid.append(svgElement('line', { x1: 0, y1: Y(y), x2: width, y2: Y(y) }));
    }
    next.append(grid);
    find('.ix-gridnote').textContent = `Grid: ${Math.round(gridStep * 100)} cm`;

    const index = Math.round(state.time * SAMPLES);
    const shown = samples.slice(0, index + 1), now = samples[index];
    const path = (key, cls) => {
      if (shown.length < 2) return;
      const d = shown.map((s) => `${X(s[key].x).toFixed(1)},${Y(s[key].y).toFixed(1)}`).join(' ');
      next.append(svgElement('polyline', { class: cls, points: d }));
    };
    if (p.frameOffset > 0) path('axle', 'ix-path ix-axle');
    path('frame', 'ix-path ix-model');
    path('odometry', 'ix-path ix-odometry');
    path('command', 'ix-path ix-command');

    const outline = (pose, geometry, cls, title) => {
      const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
      const at = (fx, fy) => [X(pose.x + fx * c - fy * s), Y(pose.y + fx * s + fy * c)];
      const list = (pts) => pts.map((q) => q.map((n) => n.toFixed(1)).join(',')).join(' ');
      const group = svgElement('g', { class: `ix-robot ${cls}` });
      group.append(svgElement('title', {}, title));
      for (const side of [1, -1]) {
        const y = side * geometry.separation / 2, hx = geometry.radius, hy = geometry.wheelWidth / 2;
        group.append(svgElement('polygon', { class: 'ix-wheel', points: list([at(hx, y + hy), at(hx, y - hy), at(-hx, y - hy), at(-hx, y + hy)]) }));
      }
      const [lx, ly] = at(0, geometry.separation / 2), [rx, ry] = at(0, -geometry.separation / 2);
      group.append(svgElement('line', { x1: lx, y1: ly, x2: rx, y2: ry }));
      const nose = 0.05, [ox, oy] = at(0, 0), [nx, ny] = at(nose, 0);
      group.append(svgElement('line', { x1: ox, y1: oy, x2: nx, y2: ny }));
      group.append(svgElement('polygon', { class: 'ix-head', points: list([at(nose + 0.012, 0), at(nose, 0.008), at(nose, -0.008)]) }));
      return { group, at };
    };
    const own = { separation: robot.separation, radius: robot.radius, wheelWidth: robot.wheelWidth };
    const configured = { separation: p.configuredSeparation, radius: p.configuredRadius, wheelWidth: robot.wheelWidth };

    next.append(outline(samples[0].axle, own, 'ix-start', 'The robot at the start').group);

    // The true pose of the model: the robot's own geometry at the axle's pose
    const model = outline(now.axle, own, 'ix-model', 'True pose, in this figure’s model');
    const [fx, fy] = model.at(-p.frameOffset, 0), [ax, ay] = model.at(0, 0);
    if (p.frameOffset > 0) {
      model.group.append(svgElement('line', { class: 'ix-link', x1: ax, y1: ay, x2: fx, y2: fy }));
      const axleDot = svgElement('circle', { class: 'ix-dot ix-axle', cx: ax, cy: ay, r: 4 });
      axleDot.append(svgElement('title', {}, 'The axle’s midpoint'));
      model.group.append(axleDot);
    }
    const frameDot = svgElement('circle', { class: 'ix-dot', cx: fx, cy: fy, r: 5 });
    frameDot.append(svgElement('title', {}, 'base_footprint, true'));
    model.group.append(frameDot);
    next.append(model.group);

    // The odometry: the geometry that the controller assumes, at the pose that it computes
    const odometry = outline(now.odometry, configured, 'ix-odometry', 'Odometry: the controller’s geometry at the pose that it computes');
    const [qx, qy] = odometry.at(0, 0);
    odometry.group.append(svgElement('circle', { class: 'ix-ring', cx: qx, cy: qy, r: 5.5 }));
    next.append(odometry.group);

    // The command: a cross where the command alone puts the robot, with its heading
    const c = Math.cos(now.command.theta), s = Math.sin(now.command.theta);
    const [kx, ky] = [X(now.command.x), Y(now.command.y)];
    const [hx, hy] = [X(now.command.x + 0.085 * c), Y(now.command.y + 0.085 * s)];
    const command = svgElement('g', { class: 'ix-robot ix-command' });
    command.append(svgElement('title', {}, 'Command'));
    command.append(svgElement('line', { x1: kx, y1: ky, x2: hx, y2: hy }));
    command.append(svgElement('path', { class: 'ix-cross', d: `M${kx - 5},${ky - 5} l10,10 m0,-10 l-10,10` }));
    next.append(command);

    if (svg) svg.replaceWith(next); else plot.append(next);
    svg = next;
    const duration = p.command.duration;
    const label = state.time >= 1 ? `t = ${duration.toFixed(1)} s, the end` : `t = ${now.t.toFixed(1)} s of ${duration.toFixed(1)} s`;
    find('.ix-time').textContent = label;
    find('.ix-scrub').setAttribute('aria-valuetext', `${now.t.toFixed(1)} of ${duration.toFixed(1)} seconds`);
  }

  // --- The numbers next to the drawing -------------------------------------------------------
  function tables() {
    const { motion, checks, p } = drive;
    const names = { command: 'command', odometry: 'odometry', model: 'true pose (model)' };
    const glyph = (key) => find(`.ix-legend [data-view="${key}"] svg`).outerHTML;
    find('.ix-motion-rows').innerHTML = Object.entries(names).map(([key, name]) => {
      const m = motion[key];
      return `<tr><th scope="row">${glyph(key)}${name}</th><td>${fixed(m.forward)} m</td><td>${fixed(m.left)} m</td><td>${fixed(m.turned)} rad</td></tr>`;
    }).join('');

    const label = { forward: 'forward', left: 'left', turned: 'turned', distance: 'distance from the start' };
    const source = { odometry: 'odometry', model: 'true pose' };
    find('.ix-check-rows').innerHTML = checks.map((check) => {
      const name = check.id === 'agreement'
        ? `odometry and true pose, ${state.command === 'turn' ? 'turned' : 'forward'}`
        : `${label[check.id]}, ${source[check.source]}`;
      const allowed = check.id === 'agreement'
        ? `at most ${check.tolerance} ${check.unit} apart`
        : `${check.expected} ± ${check.tolerance} ${check.unit}`;
      const result = check.ok ? '<span class="ix-pass">✔ passes</span>' : '<span class="ix-fail">✖ fails</span>';
      return `<tr><th scope="row">${name}</th><td data-label="Allowed">${allowed}</td><td data-label="This drive">${fixed(check.value)} ${check.unit}</td><td>${result}</td></tr>`;
    }).join('');

    const passed = checks.filter((check) => check.ok).length;
    const failed = checks.filter((check) => !check.ok).map((check) => (check.id === 'agreement'
      ? 'the comparison of the odometry with the true pose'
      : `${label[check.id]} (${source[check.source]})`));
    const o = motion.odometry, m = motion.model;
    let text = `${p.robot.name}, ${p.command.name}. `;
    if (state.command === 'forward') {
      text += `The odometry reports ${fixed(o.forward)} m. In the model, the robot drives ${fixed(m.forward)} m. `;
    } else {
      text += `The odometry reports a turn of ${fixed(o.turned)} rad. In the model, the robot turns ${fixed(m.turned)} rad`
        + ` and base_footprint ends ${fixed(Math.hypot(m.forward, m.left))} m from its start. `;
    }
    text += `${passed} of ${checks.length} checks pass.`;
    if (failed.length) text += ` Failing: ${failed.join(', ')}.`;
    find('.ix-summary').textContent = text;
  }

  function outputs() {
    const robot = data.robots[state.robot];
    const show = (name, shown, spoken) => {
      sliders[name].closest('.ix-control').querySelector('output').textContent = shown;
      sliders[name].setAttribute('aria-valuetext', spoken);
    };
    const radius = (robot.radius * setting('radius') / 100 * 1000).toFixed(1);
    const separation = (robot.separation * setting('separation') / 100 * 1000).toFixed(1);
    const contact = ((robot.separation + robot.wheelWidth * setting('contact') / 100) * 1000).toFixed(0);
    show('radius', `${setting('radius')} % · ${radius} mm`, `${setting('radius')} percent, ${radius} millimeters`);
    show('separation', `${setting('separation')} % · ${separation} mm`, `${setting('separation')} percent, ${separation} millimeters`);
    show('offset', `${setting('offset')} mm`, `${setting('offset')} millimeters behind the axle`);
    show('contact', `${setting('contact')} % · ${contact} mm apart`, `${setting('contact')} percent, ${contact} millimeters apart`);
    show('tracking', `${setting('tracking')} %`, `${setting('tracking')} percent of the commanded speed`);
    for (const legend of findAll('.ix-legend [data-view="axle"]')) legend.hidden = setting('offset') === 0;
  }

  function update() {
    outputs();
    drive = compute();
    tables();
    draw();
  }

  // Screen readers hear the result once a change is complete, not while a slider moves
  function announce() {
    clearTimeout(announceTimer);
    announceTimer = setTimeout(() => { find('.ix-announce').textContent = find('.ix-summary').textContent; }, 400);
  }

  // --- Controls ------------------------------------------------------------------------------
  const defaults = () => Object.fromEntries(Object.entries(sliders).map(([name, input]) => [name, input.defaultValue]));
  function resetSliders() {
    for (const [name, value] of Object.entries(defaults())) sliders[name].value = value;
  }
  function selectCommand(command) {
    state.command = command;
    for (const button of findAll('[data-command]')) button.setAttribute('aria-pressed', String(button.dataset.command === command));
  }
  function selectRobot(robot) {
    state.robot = robot;
    find('.ix-robot-select').value = robot;
  }
  function stopPlaying() {
    if (frame) cancelAnimationFrame(frame);
    frame = null;
    find('.ix-play').textContent = 'Play';
  }
  function setTime(time) {
    state.time = time;
    find('.ix-scrub').value = Math.round(time * 1000);
  }
  find('.ix-play').addEventListener('click', () => {
    if (frame) { stopPlaying(); return; }
    if (reducedMotion()) { setTime(1); draw(); return; }
    if (state.time >= 1) setTime(0);
    lastTick = performance.now();
    find('.ix-play').textContent = 'Pause';
    const tick = (now) => {
      // Model time: the drive's own seconds, whatever the drawing rate is
      setTime(Math.min(1, state.time + (now - lastTick) / (data.commands[state.command].duration * 1000)));
      lastTick = now;
      draw();
      if (state.time < 1) frame = requestAnimationFrame(tick); else stopPlaying();
    };
    frame = requestAnimationFrame(tick);
  });
  find('.ix-scrub').addEventListener('input', (event) => { stopPlaying(); state.time = event.target.value / 1000; draw(); });
  // A hidden tab or a closed block draws nothing: stop instead of jumping ahead later
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopPlaying(); });

  for (const [name, input] of Object.entries(sliders)) {
    input.addEventListener('input', () => { update(); });
    input.addEventListener('change', announce);
    input.addEventListener('dblclick', () => { input.value = defaults()[name]; update(); announce(); });
  }
  find('.ix-robot-select').addEventListener('change', (event) => {
    selectRobot(event.target.value); resetSliders(); update(); announce();
  });
  for (const button of findAll('[data-command]')) {
    button.addEventListener('click', () => { selectCommand(button.dataset.command); update(); announce(); });
  }
  find('.ix-reset').addEventListener('click', () => { stopPlaying(); resetSliders(); setTime(1); update(); announce(); });

  // The guided examples, one at a time. An example shows its drive with the example robot's own
  // values first, where the three views agree. "Show what happens" changes the example's one
  // value and opens its answer, which stays open.
  function showStep(index) {
    state.step = index;
    const example = data.examples[index], open = revealed.has(example.id);
    for (const item of findAll('[data-example-item]')) {
      const current = item.dataset.exampleItem === example.id;
      item.hidden = !current;
      if (!current) continue;
      item.querySelector('.ix-answer').hidden = !open;
      item.querySelector('.ix-show').hidden = open;
    }
    find('.ix-step-count').textContent = `Example ${index + 1} of ${data.examples.length}`;
    find('.ix-previous').disabled = index === 0;
    find('.ix-next').disabled = index === data.examples.length - 1;
    stopPlaying();
    selectRobot(data.default.robot);
    resetSliders();
    if (open) for (const [name, value] of Object.entries(example.settings)) sliders[name].value = value;
    selectCommand(example.command);
    setTime(1);
    update();
  }
  for (const button of findAll('[data-example]')) {
    button.addEventListener('click', () => {
      revealed.add(button.dataset.example);
      showStep(state.step);
      announce();
      // The button is gone now: the keyboard's place moves to the answer that replaced it
      const answer = find(`[data-example-item="${button.dataset.example}"] .ix-answer`);
      answer.setAttribute('tabindex', '-1');
      answer.focus({ preventScroll: true });
    });
  }
  find('.ix-previous').addEventListener('click', () => { showStep(state.step - 1); announce(); });
  find('.ix-next').addEventListener('click', () => { showStep(state.step + 1); announce(); });

  new ResizeObserver(() => { if (drive) draw(); }).observe(plot);

  // First drawing: the first example with its answer, as in the static version. Only when it
  // has succeeded does the figure switch from the static version to the controls.
  for (const element of findAll('[data-live]')) element.hidden = false;
  showStep(0);
  root.classList.add('ix-is-live');
}

// Back to the static version: the drawing and the answers that the page came with
function showStatic(root) {
  root.classList.remove('ix-is-live');
  for (const element of root.querySelectorAll('[data-live]')) element.hidden = true;
  for (const part of root.querySelectorAll('[data-example-item], .ix-answer')) part.hidden = false;
  for (const drawing of root.querySelectorAll('.ix-live-plot')) drawing.remove();
}

function mountAll() {
  for (const root of document.querySelectorAll('[data-ix="motion-lab"]:not(.ix-is-live)')) {
    try {
      mount(root);
    } catch (error) {
      showStatic(root);
      console.error('The interactive figure could not start:', error);
    }
  }
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mountAll);
else mountAll();
