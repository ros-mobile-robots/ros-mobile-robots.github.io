// "Three views of one drive": the interactive figure on the Simulation Test page.
// The markup, its texts and its numbers (robots, commands, the checks with their texts, the
// examples and the drawing's sizes) come from includes/motion-lab.html, which
// scripts/figures/motion_lab_figures.py writes. This file adds the behavior. The figure shows its
// static version until the first drawing here has succeeded. The drawing's functions follow the
// script's functions that draw the static version: scaleFor() is drawing_scale(), drawGrid() is
// grid_lines(), drawRobotOutline() is robot_outline(), and so on.
import { threeViews, relativePose, predictChecks } from './diffdrive.js';

const SVG_NS = 'http://www.w3.org/2000/svg';
const MILLISECONDS_PER_SECOND = 1000;
// Screen readers hear the result once a change is complete: this long after the last change
const ANNOUNCE_DELAY_MS = 400;

const reducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

/** A value with `digits` decimals and a true minus sign. A value that rounds to 0 shows as 0. */
function fixed(value, digits = 3) {
  const rounded = Math.abs(value) < 0.5 * 10 ** -digits ? 0 : value;
  return rounded.toFixed(digits).replace('-', '−');
}

function svgElement(name, attributes = {}, text) {
  const element = document.createElementNS(SVG_NS, name);
  for (const [key, value] of Object.entries(attributes)) element.setAttribute(key, value);
  if (text !== undefined) element.textContent = text;
  return element;
}

// --- Drawing ---------------------------------------------------------------------------------
// The drawing is in screen pixels: the figure draws it again when its width changes, so lines and
// symbols keep their size on a phone. `sizes` are the drawing's sizes from the include's data:
// lengths on the ground in m, on the screen in px.

/** The point (m) on the ground of a point given in the frame of a robot at a pose. */
function onRobot(pose, forward, left) {
  const cos = Math.cos(pose.theta), sin = Math.sin(pose.theta);
  return { x: pose.x + forward * cos - left * sin, y: pose.y + forward * sin + left * cos };
}

/**
 * How far (m) the drawing reaches beyond the paths' poses: the wheels' outer edges of both
 * outlines, the robot's arrow, the command's line and base_footprint behind the axle, with the
 * margin.
 */
function reachAround(inputs, sizes) {
  const outerEdge = Math.max(...[inputs.wheels, inputs.configured].map(
    (wheels) => wheels.separation / 2 + wheels.wheelWidth / 2));
  const front = Math.max(inputs.wheels.radius, sizes.nose + sizes.headLength, sizes.commandHeading,
    inputs.frameOffset);
  return Math.max(outerEdge, front) + sizes.margin;
}

/**
 * The scale that fits every pose of the samples into a drawing of this width, with the reach.
 * The ground's x axis (the robot's forward at the start) points to the right, its y axis up.
 */
function scaleFor(width, samples, reach, sizes) {
  const poses = samples.flatMap((sample) => [sample.command, sample.odometry, sample.axle, sample.frame]);
  const xs = poses.map((pose) => pose.x), ys = poses.map((pose) => pose.y);
  const left = Math.min(...xs) - reach, right = Math.max(...xs) + reach;
  const bottom = Math.min(...ys) - reach, top = Math.max(...ys) + reach;
  const height = Math.round(width * sizes.aspect);
  const pixelsPerMetre = Math.min((width - 2 * sizes.paddingPx) / (right - left),
    (height - 2 * sizes.paddingPx) / (top - bottom));
  const centerX = (left + right) / 2, centerY = (bottom + top) / 2;
  const toPixels = ({ x, y }) => [width / 2 + (x - centerX) * pixelsPerMetre, height / 2 - (y - centerY) * pixelsPerMetre];
  return { width, height, centerX, centerY, pixelsPerMetre, toPixels };
}

/** The grid's spacing (m): the smallest step whose lines lie far enough apart. */
function gridStep(scale, sizes) {
  return sizes.gridSteps.find((step) => step * scale.pixelsPerMetre >= sizes.gridSpacingPx)
    ?? sizes.gridSteps.at(-1);
}

/** The grid: lines at the multiples of the step across the whole drawing. */
function drawGrid(scale, step) {
  const grid = svgElement('g', { class: 'ix-grid' });
  const halfWidth = scale.width / 2 / scale.pixelsPerMetre, halfHeight = scale.height / 2 / scale.pixelsPerMetre;
  for (let index = Math.ceil((scale.centerX - halfWidth) / step); index * step < scale.centerX + halfWidth; index++) {
    const [x] = scale.toPixels({ x: index * step, y: 0 });
    grid.append(svgElement('line', { x1: x, y1: 0, x2: x, y2: scale.height }));
  }
  for (let index = Math.ceil((scale.centerY - halfHeight) / step); index * step < scale.centerY + halfHeight; index++) {
    const [, y] = scale.toPixels({ x: 0, y: index * step });
    grid.append(svgElement('line', { x1: 0, y1: y, x2: scale.width, y2: y }));
  }
  return grid;
}

const pointsAttribute = (points) => points.map((point) => point.map((value) => value.toFixed(1)).join(',')).join(' ');

/** A view's path through its poses, or null before the path has two poses. */
function drawPath(scale, poses, cssClass) {
  if (poses.length < 2) return null;
  return svgElement('polyline', { class: `ix-path ${cssClass}`, points: pointsAttribute(poses.map(scale.toPixels)) });
}

/** A robot from above at a pose of its axle's midpoint: two wheels, the axle and an arrow to its front. */
function drawRobotOutline(scale, pose, wheels, cssClass, title, sizes) {
  const at = (forward, left) => scale.toPixels(onRobot(pose, forward, left));
  const group = svgElement('g', { class: `ix-robot ${cssClass}` });
  group.append(svgElement('title', {}, title));
  for (const side of [1, -1]) {
    const middle = side * wheels.separation / 2, halfWidth = wheels.wheelWidth / 2;
    const corners = [at(wheels.radius, middle + halfWidth), at(wheels.radius, middle - halfWidth),
      at(-wheels.radius, middle - halfWidth), at(-wheels.radius, middle + halfWidth)];
    group.append(svgElement('polygon', { class: 'ix-wheel', points: pointsAttribute(corners) }));
  }
  const [leftX, leftY] = at(0, wheels.separation / 2), [rightX, rightY] = at(0, -wheels.separation / 2);
  group.append(svgElement('line', { x1: leftX, y1: leftY, x2: rightX, y2: rightY }));
  const [axleX, axleY] = at(0, 0), [noseX, noseY] = at(sizes.nose, 0);
  group.append(svgElement('line', { x1: axleX, y1: axleY, x2: noseX, y2: noseY }));
  const head = [at(sizes.nose + sizes.headLength, 0), at(sizes.nose, sizes.headHalfWidth), at(sizes.nose, -sizes.headHalfWidth)];
  group.append(svgElement('polygon', { class: 'ix-head', points: pointsAttribute(head) }));
  return group;
}

/** The model's markers: a dot at base_footprint, and with an offset the axle's midpoint joined to it. */
function drawTruePoseMarkers(group, scale, axle, frameOffset, sizes) {
  const [frameX, frameY] = scale.toPixels(onRobot(axle, -frameOffset, 0));
  if (frameOffset > 0) {
    const [axleX, axleY] = scale.toPixels(axle);
    group.append(svgElement('line', { class: 'ix-link', x1: axleX, y1: axleY, x2: frameX, y2: frameY }));
    const axleDot = svgElement('circle', { class: 'ix-dot ix-axle', cx: axleX, cy: axleY, r: sizes.axleDotRadiusPx });
    axleDot.append(svgElement('title', {}, 'The axle’s midpoint'));
    group.append(axleDot);
  }
  const frameDot = svgElement('circle', { class: 'ix-dot', cx: frameX, cy: frameY, r: sizes.dotRadiusPx });
  frameDot.append(svgElement('title', {}, 'base_footprint, true'));
  group.append(frameDot);
}

/** The odometry's marker: a ring at its base_footprint. */
function drawOdometryMarker(group, scale, pose, sizes) {
  const [x, y] = scale.toPixels(pose);
  group.append(svgElement('circle', { class: 'ix-ring', cx: x, cy: y, r: sizes.ringRadiusPx }));
}

/** The command's end: a cross where the command alone puts the robot, with a line in its heading. */
function drawCommandMarker(scale, pose, sizes) {
  const [crossX, crossY] = scale.toPixels(pose);
  const [headingX, headingY] = scale.toPixels(onRobot(pose, sizes.commandHeading, 0));
  const half = sizes.crossHalfPx;
  const group = svgElement('g', { class: 'ix-robot ix-command' });
  group.append(svgElement('title', {}, 'Command'));
  group.append(svgElement('line', { x1: crossX, y1: crossY, x2: headingX, y2: headingY }));
  group.append(svgElement('path', {
    class: 'ix-cross', d: `M${crossX - half},${crossY - half} l${2 * half},${2 * half} m0,${-2 * half} l${-2 * half},${2 * half}`,
  }));
  return group;
}

// --- The figure ------------------------------------------------------------------------------

function mount(root) {
  const data = JSON.parse(root.querySelector('.ix-data').textContent);
  const sizes = data.drawing;
  const find = (selector) => root.querySelector(selector);
  const findAll = (selector) => [...root.querySelectorAll(selector)];
  const plot = find('.ix-plot'), scrub = find('.ix-scrub');
  const sliders = Object.fromEntries(findAll('input[data-setting]').map((input) => [input.dataset.setting, input]));
  const state = { robot: data.default.robot, command: data.default.command, time: 0, step: 0 };
  // The examples whose own drive has reached its end: their answer shows while their drive does
  const answered = new Set();
  // A drive has reached its end since the example opened: the results show
  let reachedEnd = false;
  let animationRequest = null, lastTickTime = 0, announceTimer = null, drive = null, liveDrawing = null;

  // --- The model's inputs, from the controls -------------------------------------------------
  const setting = (name) => Number(sliders[name].value);
  /** The model's inputs in SI units, as model_inputs() in the script computes them. */
  function readInputs() {
    const robot = data.robots[state.robot];
    return {
      robot,
      command: data.commands[state.command],
      wheels: { radius: robot.radius, separation: robot.separation, wheelWidth: robot.wheelWidth },
      configured: {
        radius: robot.radius * setting('radius_percent') / 100,
        separation: robot.separation * setting('separation_percent') / 100,
        wheelWidth: robot.wheelWidth,
      },
      contactSeparation: robot.separation + robot.wheelWidth * setting('contact_percent') / 100,
      frameOffset: setting('offset_mm') / 1000,
      tracking: setting('tracking_percent') / 100,
    };
  }

  /** The drive of the current controls: its inputs, its poses over time, its motions and checks. */
  function computeDrive() {
    const inputs = readInputs(), command = inputs.command;
    const views = threeViews({
      v: command.linearVelocity, w: command.angularVelocity,
      radius: inputs.wheels.radius, separation: inputs.wheels.separation,
      configuredRadius: inputs.configured.radius, configuredSeparation: inputs.configured.separation,
      contactSeparation: inputs.contactSeparation, frameOffset: inputs.frameOffset, tracking: inputs.tracking,
    });
    const samples = [];
    for (let index = 0; index <= sizes.samples; index++) samples.push(views.posesAt(command.duration * index / sizes.samples));
    const start = samples[0], end = samples[sizes.samples];
    const motion = {
      command: relativePose(start.command, end.command),
      odometry: relativePose(start.odometry, end.odometry),
      model: relativePose(start.frame, end.frame),
    };
    const checks = predictChecks(data.checks[state.command], motion.odometry, motion.model);
    return { kind: state.command, inputs, samples, motion, checks, reach: reachAround(inputs, sizes) };
  }

  // --- The drawing, at the time that the play bar shows ---------------------------------------
  function updateTimeLabel(time, duration) {
    find('.ix-time').textContent = state.time >= 1
      ? `t = ${duration.toFixed(1)} s, the end` : `t = ${time.toFixed(1)} s of ${duration.toFixed(1)} s`;
    scrub.setAttribute('aria-valuetext', `${time.toFixed(1)} of ${duration.toFixed(1)} seconds`);
  }

  function draw() {
    const { inputs, samples } = drive;
    const scale = scaleFor(Math.max(sizes.minimumWidthPx, Math.round(plot.clientWidth)), samples, drive.reach, sizes);
    const step = gridStep(scale, sizes);
    const shownIndex = Math.round(state.time * sizes.samples);
    const shownSamples = samples.slice(0, shownIndex + 1), shownSample = samples[shownIndex];
    const next = svgElement('svg', {
      class: 'ix-live-plot', viewBox: `0 0 ${scale.width} ${scale.height}`, role: 'img',
      'aria-label': find('.ix-static title').textContent, 'aria-describedby': find('.ix-summary').id,
    });
    next.append(drawGrid(scale, step));
    const paths = [['frame', 'ix-model'], ['odometry', 'ix-odometry'], ['command', 'ix-command']];
    if (inputs.frameOffset > 0) paths.unshift(['axle', 'ix-axle']);
    for (const [view, cssClass] of paths) {
      const path = drawPath(scale, shownSamples.map((sample) => sample[view]), cssClass);
      if (path) next.append(path);
    }
    next.append(drawRobotOutline(scale, samples[0].axle, inputs.wheels, 'ix-start', 'The robot at the start', sizes));
    const truePose = drawRobotOutline(scale, shownSample.axle, inputs.wheels, 'ix-model',
      'True pose, in this figure’s model', sizes);
    drawTruePoseMarkers(truePose, scale, shownSample.axle, inputs.frameOffset, sizes);
    next.append(truePose);
    const odometry = drawRobotOutline(scale, shownSample.odometry, inputs.configured, 'ix-odometry',
      'Odometry: the controller’s geometry at the pose that it computes', sizes);
    drawOdometryMarker(odometry, scale, shownSample.odometry, sizes);
    next.append(odometry);
    next.append(drawCommandMarker(scale, shownSample.command, sizes));
    if (liveDrawing) liveDrawing.replaceWith(next); else plot.append(next);
    liveDrawing = next;
    find('.ix-gridnote').textContent = `Grid: ${Math.round(step * 100)} cm`;
    updateTimeLabel(shownSample.t, inputs.command.duration);
  }

  // --- The numbers next to the drawing -------------------------------------------------------
  function renderMotionTable({ motion }) {
    const glyph = (view) => find(`.ix-legend [data-view="${view}"] svg`).outerHTML;
    find('.ix-motion-rows').innerHTML = Object.entries(data.viewNames).map(([view, name]) => {
      const { forward, left, turned } = motion[view];
      return `<tr><th scope="row">${glyph(view)}${name}</th><td>${fixed(forward)} m</td><td>${fixed(left)} m</td><td>${fixed(turned)} rad</td></tr>`;
    }).join('');
  }

  function renderCheckTable({ checks }) {
    find('.ix-check-rows').innerHTML = checks.map((check) => {
      const result = check.passes ? '<span class="ix-pass">✔ passes</span>' : '<span class="ix-fail">✖ fails</span>';
      return `<tr><th scope="row">${check.name}</th><td data-label="Allowed">${check.allowed}</td><td data-label="This drive">${fixed(check.value)} ${check.unit}</td><td>${result}</td></tr>`;
    }).join('');
  }

  function renderSummary({ kind, inputs, motion, checks }) {
    const { odometry, model } = motion;
    let text = `${inputs.robot.name}, ${inputs.command.text}. `;
    if (kind === 'forward') {
      text += `The odometry reports ${fixed(odometry.forward)} m. In the model, the robot drives ${fixed(model.forward)} m. `;
    } else {
      text += `The odometry reports a turn of ${fixed(odometry.turned)} rad. In the model, the robot turns ${fixed(model.turned)} rad`
        + ` and base_footprint ends ${fixed(Math.hypot(model.forward, model.left))} m from its start. `;
    }
    const failing = checks.filter((check) => !check.passes).map((check) => check.nameInSentence);
    text += `${checks.length - failing.length} of ${checks.length} checks pass.`;
    if (failing.length) text += ` Failing: ${failing.join(', ')}.`;
    find('.ix-summary').textContent = text;
  }

  function renderSliderValues(inputs) {
    const show = (name, shown, spoken) => {
      sliders[name].closest('.ix-control').querySelector('output').textContent = shown;
      sliders[name].setAttribute('aria-valuetext', spoken);
    };
    const millimetres = (metres, digits) => (metres * 1000).toFixed(digits);
    const radius = millimetres(inputs.configured.radius, 1), separation = millimetres(inputs.configured.separation, 1);
    const contact = millimetres(inputs.contactSeparation, 0);
    show('radius_percent', `${setting('radius_percent')} % · ${radius} mm`, `${setting('radius_percent')} percent, ${radius} millimeters`);
    show('separation_percent', `${setting('separation_percent')} % · ${separation} mm`,
      `${setting('separation_percent')} percent, ${separation} millimeters`);
    show('offset_mm', `${setting('offset_mm')} mm`, `${setting('offset_mm')} millimeters behind the axle`);
    show('contact_percent', `${setting('contact_percent')} % · ${contact} mm apart`,
      `${setting('contact_percent')} percent, ${contact} millimeters apart`);
    show('tracking_percent', `${setting('tracking_percent')} %`, `${setting('tracking_percent')} percent of the commanded speed`);
    for (const entry of findAll('.ix-legend [data-view="axle"]')) entry.hidden = inputs.frameOffset === 0;
  }

  function update() {
    drive = computeDrive();
    renderSliderValues(drive.inputs);
    renderMotionTable(drive);
    renderCheckTable(drive);
    renderSummary(drive);
    draw();
  }

  // Screen readers hear the result once a change is complete, not while a slider moves, and only
  // while the results show
  function announce() {
    clearTimeout(announceTimer);
    announceTimer = setTimeout(() => {
      if (reachedEnd) find('.ix-announce').textContent = find('.ix-summary').textContent;
    }, ANNOUNCE_DELAY_MS);
  }

  // --- Controls ------------------------------------------------------------------------------
  // The figure shows one drive: the robot, the command and the values that the controls hold, at
  // the time of the play bar. Only Play, the time slider and an example's opening move the time.
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
    if (animationRequest) cancelAnimationFrame(animationRequest);
    animationRequest = null;
    find('.ix-play').textContent = 'Play';
  }
  // The time is a share of the drive: 0 at the start, 1 at the end
  function setTime(time) {
    state.time = time;
    scrub.value = Math.round(time * Number(scrub.max));
  }
  /** The controls hold the current example's own drive: its robot, its command and its values. */
  function isExampleDrive() {
    const example = data.examples[state.step], own = { ...defaults(), ...example.settings };
    return state.robot === data.default.robot && state.command === example.command
      && Object.entries(sliders).every(([name, input]) => Number(input.value) === Number(own[name]));
  }
  // The summary and the tables show the drive's end: they wait until a drive has reached its end
  // since the example opened, then follow every change. The example's answer shows under the
  // drawing while the controls hold the example's own drive and that drive has reached its end.
  // Returns what has just appeared: 'answer', 'results' or null.
  function showResults() {
    const key = data.examples[state.step].key, answer = find('.ix-shown-answer');
    const resultsAppear = !reachedEnd && state.time >= 1;
    if (resultsAppear) reachedEnd = true;
    const own = isExampleDrive();
    const answerAppears = own && reachedEnd && !answered.has(key);
    if (answerAppears) answered.add(key);
    answer.hidden = !(own && answered.has(key));
    // While the answer shows, the button keeps its place, invisible, so the questions' height
    // stays. With other values it shows again: it brings back the example's own drive.
    find('.ix-current').classList.toggle('ix-revealed', !answer.hidden);
    root.classList.toggle('ix-waiting', !reachedEnd);
    if (answerAppears) return 'answer';
    return resultsAppear ? 'results' : null;
  }
  // Screen readers hear what has just appeared at the drive's end
  function announceAppeared(appeared) {
    if (appeared === 'answer') find('.ix-announce').textContent = find('.ix-shown-answer').textContent;
    if (appeared === 'results') announce();
  }
  function startPlaying() {
    if (state.time >= 1) setTime(0);
    lastTickTime = performance.now();
    find('.ix-play').textContent = 'Pause';
    const tick = (timestamp) => {
      // Model time: the drive's own seconds, whatever the drawing rate is
      const duration = data.commands[state.command].duration * MILLISECONDS_PER_SECOND;
      setTime(Math.min(1, state.time + (timestamp - lastTickTime) / duration));
      lastTickTime = timestamp;
      draw();
      if (state.time < 1) { animationRequest = requestAnimationFrame(tick); return; }
      stopPlaying();
      announceAppeared(showResults());
    };
    animationRequest = requestAnimationFrame(tick);
  }
  // With reduced motion, Play shows the drive's end at once
  function play() {
    if (reducedMotion()) { stopPlaying(); setTime(1); draw(); announceAppeared(showResults()); return; }
    startPlaying();
  }
  find('.ix-play').addEventListener('click', () => {
    if (animationRequest) { stopPlaying(); return; }
    play();
  });
  scrub.addEventListener('input', () => {
    stopPlaying(); state.time = Number(scrub.value) / Number(scrub.max); draw(); announceAppeared(showResults());
  });
  // A hidden tab or a closed block draws nothing: stop instead of jumping ahead later
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopPlaying(); });

  // A changed value, robot or command redraws the drive at the same time
  function redraw() {
    update();
    showResults();
  }
  for (const [name, input] of Object.entries(sliders)) {
    input.addEventListener('input', redraw);
    input.addEventListener('change', announce);
    input.addEventListener('dblclick', () => { input.value = defaults()[name]; redraw(); announce(); });
  }
  find('.ix-robot-select').addEventListener('change', (event) => {
    selectRobot(event.target.value); resetSliders(); redraw(); announce();
  });
  for (const button of findAll('[data-command]')) {
    button.addEventListener('click', () => { selectCommand(button.dataset.command); redraw(); announce(); });
  }
  find('.ix-reset').addEventListener('click', () => { resetSliders(); redraw(); announce(); });

  // The guided examples, one at a time. An example opens with its question and its own drive,
  // with its one value changed, at the drive's start: the time slider and Play show the drive, and
  // its results and answer show once the drive has reached its end. "Show what happens" plays the
  // example's own drive from its start. An answered example opens at its drive's end.
  // The examples' questions share one place above the drawing, as high as the longest of them,
  // so the drawing stays where it is from one example to the next.
  function showStep(index, time = answered.has(data.examples[index].key) ? 1 : 0) {
    state.step = index;
    const example = data.examples[index];
    const shownAnswer = find('.ix-shown-answer');
    for (const item of findAll('[data-example-item]')) {
      const current = item.dataset.exampleItem === example.key;
      item.classList.toggle('ix-current', current);
      item.classList.toggle('ix-revealed', answered.has(item.dataset.exampleItem));
      item.querySelector('.ix-answer').hidden = true;
      if (current) shownAnswer.innerHTML = `<b>What happens:</b> ${item.querySelector('.ix-answer').innerHTML}`;
    }
    find('.ix-step-count').textContent = `Example ${index + 1} of ${data.examples.length}`;
    find('.ix-previous').disabled = index === 0;
    find('.ix-next').disabled = index === data.examples.length - 1;
    stopPlaying();
    selectRobot(data.default.robot);
    resetSliders();
    for (const [name, value] of Object.entries(example.settings)) sliders[name].value = value;
    selectCommand(example.command);
    reachedEnd = false;
    setTime(time);
    update();
    showResults();
  }
  for (const button of findAll('[data-example]')) {
    button.addEventListener('click', () => {
      showStep(state.step, 0);
      play();
      // The button is gone now: the keyboard's place moves to the answer under the drawing, or
      // to Pause while the drive plays
      find(reducedMotion() ? '.ix-shown-answer' : '.ix-play').focus({ preventScroll: true });
    });
  }
  // Screen readers hear the new example's question
  function announceQuestion() {
    clearTimeout(announceTimer);
    const question = find('.ix-current .ix-question').textContent;
    find('.ix-announce').textContent = `${find('.ix-step-count').textContent}. ${question}`;
  }
  find('.ix-previous').addEventListener('click', () => { showStep(state.step - 1); announceQuestion(); });
  find('.ix-next').addEventListener('click', () => { showStep(state.step + 1); announceQuestion(); });

  new ResizeObserver(() => { if (drive) draw(); }).observe(plot);

  // First drawing: the first example's question and its own drive at the start. Only when it has
  // succeeded does the figure switch from the static version to the controls.
  for (const element of findAll('[data-live]')) element.hidden = false;
  showStep(0);
  root.classList.add('ix-is-live');
}

// Back to the static version: the drawing and the answers that the page came with
function showStatic(root) {
  root.classList.remove('ix-is-live');
  for (const element of root.querySelectorAll('[data-live]')) element.hidden = true;
  for (const answer of root.querySelectorAll('.ix-answer')) answer.hidden = false;
  for (const item of root.querySelectorAll('[data-example-item]')) item.classList.remove('ix-current', 'ix-revealed');
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
