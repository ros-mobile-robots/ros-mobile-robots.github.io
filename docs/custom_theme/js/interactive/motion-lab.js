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
// Two taps on a slider within this time reset it: touch screens send no double click. A tap is a
// touch shorter than TAP_MAX_MS that moves less than TAP_SLOP_PX, and the second one lands that
// close to the first.
const DOUBLE_TAP_MS = 350;
const TAP_MAX_MS = 300;
const TAP_SLOP_PX = 10;
const DOUBLE_TAP_DISTANCE_PX = 30;
// The Enlarge button shows from this window width on, as in interactive.css
const WIDE_MIN_EM = 60;

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
  // The questions, one per example and drive ("radius:forward"), whose drive has reached its end
  // with the example's own values: their answer shows while those values are set
  const answered = new Set();
  const questionKey = (command = state.command) => `${data.examples[state.step].key}:${command}`;
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
      'aria-label': find('.ix-static title').textContent, 'aria-describedby': find('.ix-now').id,
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
    describeNow(shownSample);
  }

  // For screen readers, the drawing's description: what it shows at the play bar's time, so the
  // drive's end stays unknown until the drive gets there
  function describeNow(shownSample) {
    const { samples, inputs, kind } = drive, start = samples[0];
    const command = relativePose(start.command, shownSample.command);
    const odometry = relativePose(start.odometry, shownSample.odometry);
    const model = relativePose(start.frame, shownSample.frame);
    const at = `At ${shownSample.t.toFixed(1)} s of ${inputs.command.duration.toFixed(1)} s, `;
    find('.ix-now').textContent = kind === 'forward'
      ? `${at}the commanded distance is ${fixed(command.forward)} m. The odometry reports`
        + ` ${fixed(odometry.forward)} m and the robot in the model has travelled ${fixed(model.forward)} m.`
      : `${at}the commanded angle is ${fixed(command.turned)} rad. The odometry reports ${fixed(odometry.turned)} rad`
        + ` and the robot in the model has turned ${fixed(model.turned)} rad. Its base_footprint is`
        + ` ${fixed(Math.hypot(model.forward, model.left))} m from its starting point.`;
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
    // The failing checks' names are for screen readers: the check table shows them, and the box
    // keeps its size when other checks fail
    const summary = find('.ix-summary');
    summary.textContent = text;
    if (failing.length) {
      const names = document.createElement('span');
      names.className = 'ix-sr';
      names.textContent = ` Failing: ${failing.join(', ')}.`;
      summary.append(names);
    }
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
    // The axle's entry keeps its place while unused, so the legend keeps its lines
    for (const entry of findAll('.ix-legend [data-view="axle"]')) {
      entry.hidden = false;
      entry.style.visibility = inputs.frameOffset === 0 ? 'hidden' : '';
    }
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
  /** The example's own values: the example robot's, with the example's one value changed. */
  const exampleValues = () => ({ ...defaults(), ...data.examples[state.step].settings });
  /** The controls hold the current example's own values, on either drive. */
  function isExampleDrive() {
    const own = exampleValues();
    return state.robot === data.default.robot
      && Object.entries(sliders).every(([name, input]) => Number(input.value) === Number(own[name]));
  }
  // While other values are set: the values that differ from the example's, with the example's
  function renderChanges(own) {
    const line = find('.ix-changes');
    line.textContent = '';
    if (own) return;
    const values = exampleValues(), robots = data.robots;
    const lowerFirst = (text) => text.charAt(0).toLowerCase() + text.slice(1);
    const changes = Object.entries(sliders).filter(([name, input]) => Number(input.value) !== Number(values[name]))
      .map(([name, input]) => {
        const label = lowerFirst(input.closest('.ix-control').querySelector('label').textContent);
        return `${label} ${input.value} ${input.dataset.unit} (the example: ${values[name]} ${input.dataset.unit})`;
      });
    if (state.robot !== data.default.robot) {
      changes.unshift(`${robots[state.robot].name} (the example: ${robots[data.default.robot].name})`);
    }
    const lead = document.createElement('b');
    lead.textContent = data.texts.changesLead;
    line.append(lead, ` ${changes.join(', ')}. ${data.texts.changesEnd}`);
  }
  // The place under the play bar only grows while an example is open, and at one width: a value
  // that crosses the example's own takes its comparison away, and the controls below stay put
  let areaHeight = 0;
  function holdAreaHeight() {
    const area = find('.ix-answer-area');
    area.style.minHeight = '';
    areaHeight = Math.max(areaHeight, area.offsetHeight);
    area.style.minHeight = `${areaHeight}px`;
  }
  // The summary and the tables show the drive's end: they wait until a drive has reached its end
  // since the example opened, then follow every change. A question is answered the first time the
  // controls hold the example's own drive at its end: another drive's end shows its results but
  // answers nothing. The answer then shows while the example's own values are set.
  // Under the play bar, one place holds the answer, the hint before it, or the values that differ
  // from the example's. It is as high as the highest of them, so nothing below moves.
  // Returns what has just appeared: 'answer', 'results' or null.
  function showResults() {
    const key = questionKey();
    const resultsAppear = !reachedEnd && state.time >= 1;
    if (resultsAppear) reachedEnd = true;
    const own = isExampleDrive();
    const answerAppears = own && state.time >= 1 && !answered.has(key);
    if (answerAppears) answered.add(key);
    const showAnswer = own && answered.has(key);
    find('.ix-shown-answer').classList.toggle('ix-off', !showAnswer);
    find('.ix-hint').classList.toggle('ix-off', !own || showAnswer);
    find('.ix-changes').classList.toggle('ix-off', own);
    renderChanges(own);
    // While the answer shows, the example's button keeps its place, invisible, so the questions'
    // height stays. With other values it shows again and sets the example's values back.
    const item = find('.ix-current');
    item.classList.toggle('ix-revealed', showAnswer);
    item.querySelector('.ix-show').textContent = own ? data.texts.show : data.texts.replay;
    holdAreaHeight();
    root.classList.toggle('ix-waiting', !reachedEnd);
    if (answerAppears) return 'answer';
    return resultsAppear ? 'results' : null;
  }
  // Screen readers hear what has just appeared at the drive's end: the answer replaces a summary
  // that waits to be heard
  function announceAppeared(appeared) {
    if (appeared === 'answer') {
      clearTimeout(announceTimer);
      find('.ix-announce').textContent = find('.ix-shown-answer').textContent;
    }
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
  // A double click, or on a touch screen a double tap, sets one slider back to the robot's value.
  // A tap is a short touch that hardly moves: two quick drags are no double tap, and any other
  // press in between starts over. Mice and pens use the browser's double click only.
  let press = null, lastTap = null, lastTouchReset = -Infinity;
  const isTouch = (event) => event.pointerType === 'touch' && event.isPrimary;
  document.addEventListener('pointerdown', (event) => {
    if (lastTap && event.target !== lastTap.input) lastTap = null;
  }, true);
  for (const [name, input] of Object.entries(sliders)) {
    const resetOne = () => { input.value = defaults()[name]; redraw(); announce(); };
    input.addEventListener('input', redraw);
    input.addEventListener('change', announce);
    // A browser that sends a double click after a double tap too resets the slider once
    input.addEventListener('dblclick', (event) => {
      if (event.timeStamp - lastTouchReset > DOUBLE_TAP_MS) resetOne();
    });
    input.addEventListener('pointerdown', (event) => {
      press = isTouch(event)
        ? { input, id: event.pointerId, x: event.clientX, y: event.clientY, time: event.timeStamp, moved: false } : null;
      if (!press) lastTap = null;
    });
    input.addEventListener('pointermove', (event) => {
      if (press && Math.hypot(event.clientX - press.x, event.clientY - press.y) > TAP_SLOP_PX) press.moved = true;
    });
    input.addEventListener('pointercancel', () => { press = null; lastTap = null; });
    input.addEventListener('pointerup', (event) => {
      const short = press && event.timeStamp - press.time < TAP_MAX_MS
        && Math.hypot(event.clientX - press.x, event.clientY - press.y) <= TAP_SLOP_PX;
      const tap = short && press.input === input && press.id === event.pointerId && !press.moved && isTouch(event)
        ? { input, x: event.clientX, y: event.clientY, time: event.timeStamp } : null;
      press = null;
      const second = Boolean(tap && lastTap && lastTap.input === input && tap.time - lastTap.time < DOUBLE_TAP_MS
        && Math.hypot(tap.x - lastTap.x, tap.y - lastTap.y) < DOUBLE_TAP_DISTANCE_PX);
      lastTap = second ? null : tap;
      if (second) { lastTouchReset = event.timeStamp; resetOne(); }
    });
  }
  find('.ix-robot-select').addEventListener('change', (event) => {
    selectRobot(event.target.value); resetSliders(); redraw(); announce();
  });
  // The other drive is the example's other question: it starts at its drive's start, or at its
  // end once answered. The values stay as they are.
  for (const button of findAll('[data-command]')) {
    button.addEventListener('click', () => {
      // The command that shows already: nothing changes, not even the time
      if (button.dataset.command === state.command) return;
      stopPlaying();
      selectCommand(button.dataset.command);
      showQuestion();
      reachedEnd = false;
      setTime(answered.has(questionKey()) ? 1 : 0);
      redraw();
      announceQuestion();
    });
  }
  // Reset: next to the examples and in the block of values
  for (const button of findAll('.ix-reset')) {
    button.addEventListener('click', () => { resetSliders(); redraw(); announce(); });
  }

  // Enlarge: the figure over the whole window in a modal dialog, with the drawing and the values
  // next to each other. The rest of the page is out of the keyboard's and the screen reader's
  // reach meanwhile. The button, Esc, printing or a window that gets too narrow bring the figure
  // back to its place on the page.
  const enlarge = find('.ix-enlarge'), settings = find('.ix-settings'), caption = find('figcaption');
  const stage = find('.ix-stage'), side = find('.ix-side');
  const wideEnough = matchMedia(`(min-width: ${WIDE_MIN_EM}em)`);
  const placeholder = document.createComment('The enlarged figure returns here');
  let dialog = null, settingsWereOpen = false;
  function enlargeFigure() {
    if (dialog) return;
    // Enlarged, the values show first in the right-hand column, and the caption ends the left one
    settingsWereOpen = settings.open;
    settings.open = true;
    side.prepend(settings);
    stage.append(caption);
    dialog = document.createElement('dialog');
    dialog.className = 'md-typeset ix-dialog';
    dialog.setAttribute('aria-label', `Enlarged figure: ${find('.ix-static title').textContent}`);
    dialog.addEventListener('cancel', (event) => { event.preventDefault(); returnToPage(); });
    root.before(placeholder);
    document.body.append(dialog);
    dialog.append(root);
    root.classList.add('ix-wide');
    document.documentElement.classList.add('ix-wide-open');
    enlarge.textContent = 'Back to the page';
    dialog.showModal();
    enlarge.focus();
  }
  function returnToPage() {
    if (!dialog) return;
    const focused = root.contains(document.activeElement) ? document.activeElement : null;
    dialog.close();
    placeholder.replaceWith(root);
    dialog.remove();
    dialog = null;
    side.append(settings);
    root.append(caption);
    settings.open = settingsWereOpen;
    root.classList.remove('ix-wide');
    document.documentElement.classList.remove('ix-wide-open');
    enlarge.textContent = 'Enlarge';
    // The keyboard's place: where it was if it can still be there, else the summary of the closed
    // values, the Enlarge button, or Play. A control in a closed block or a hidden button can't
    // take the focus, so the first one that takes it keeps it.
    const inClosedSettings = focused && !settings.open && settings.contains(focused);
    for (const element of [focused, inClosedSettings ? settings.querySelector('summary') : null, enlarge, find('.ix-play')]) {
      element?.focus();
      if (element && document.activeElement === element) break;
    }
  }
  enlarge.addEventListener('click', () => (dialog ? returnToPage() : enlargeFigure()));
  wideEnough.addEventListener('change', () => { if (!wideEnough.matches) returnToPage(); });
  addEventListener('beforeprint', returnToPage);

  // The guided examples, one at a time. An example opens with its question and its own drive,
  // with its one value changed, at the drive's start: the time slider and Play show the drive, and
  // its results and answer show once the drive has reached its end. "Show what happens" plays the
  // example's own drive from its start. An answered example opens at its drive's end.
  // The examples' questions share one place above the drawing, as high as the longest of them,
  // so the drawing stays where it is from one example to the next.
  // The current example's question and answer for the current drive
  function showQuestion() {
    const item = find('.ix-current');
    for (const question of item.querySelectorAll('.ix-question')) {
      question.classList.toggle('ix-drive', question.dataset.drive === state.command);
    }
    const answer = item.querySelector(`.ix-answer[data-drive="${state.command}"]`);
    find('.ix-shown-answer').innerHTML = `<b>What happens:</b> ${answer.innerHTML}`;
  }
  function showStep(index, command = data.examples[index].command, time) {
    state.step = index;
    const example = data.examples[index];
    for (const item of findAll('[data-example-item]')) {
      item.classList.toggle('ix-current', item.dataset.exampleItem === example.key);
      item.classList.remove('ix-revealed');
      for (const answer of item.querySelectorAll('.ix-answer')) answer.hidden = true;
    }
    find('.ix-step-count').textContent = `Example ${index + 1} of ${data.examples.length}`;
    find('.ix-previous').disabled = index === 0;
    find('.ix-next').disabled = index === data.examples.length - 1;
    stopPlaying();
    selectRobot(data.default.robot);
    resetSliders();
    for (const [name, value] of Object.entries(example.settings)) sliders[name].value = value;
    selectCommand(command);
    showQuestion();
    reachedEnd = false;
    setTime(time ?? (answered.has(questionKey()) ? 1 : 0));
    update();
    areaHeight = 0;
    showResults();
  }
  for (const button of findAll('[data-example]')) {
    button.addEventListener('click', () => {
      // A summary that waits to be heard belongs to the values before the replay
      clearTimeout(announceTimer);
      showStep(state.step, state.command, 0);
      if (reducedMotion()) {
        // The answer shows at once and takes the keyboard's place: screen readers read it there
        stopPlaying();
        setTime(1);
        draw();
        showResults();
        find('.ix-shown-answer').focus();
        return;
      }
      // The keyboard's place moves to Pause while the drive plays, and stays where the reader
      // leaves it when the drive ends
      startPlaying();
      find('.ix-play').focus({ preventScroll: true });
    });
  }
  // Screen readers hear the new example's question
  function announceQuestion() {
    clearTimeout(announceTimer);
    const question = find('.ix-current .ix-question.ix-drive').textContent;
    find('.ix-announce').textContent = `${find('.ix-step-count').textContent}. ${question}`;
  }
  // At the first and the last example, the button just pressed is disabled: the keyboard's place
  // moves to the other one
  function stepTo(index, pressed, other) {
    showStep(index);
    announceQuestion();
    if (pressed.disabled && document.activeElement !== other) other.focus();
  }
  const previousButton = find('.ix-previous'), nextButton = find('.ix-next');
  previousButton.addEventListener('click', () => stepTo(state.step - 1, previousButton, nextButton));
  nextButton.addEventListener('click', () => stepTo(state.step + 1, nextButton, previousButton));

  // Another width redraws the drawing and measures the place under the play bar anew
  new ResizeObserver(() => {
    if (!drive) return;
    draw();
    areaHeight = 0;
    holdAreaHeight();
  }).observe(plot);

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
  for (const answer of root.querySelectorAll('.ix-answer:not([data-live])')) answer.hidden = false;
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
