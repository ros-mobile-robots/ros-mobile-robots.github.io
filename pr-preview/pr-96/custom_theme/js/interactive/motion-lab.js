// "Three views of one drive": the interactive figure on the Simulation Test page.
// The markup, its texts and its numbers (robots, commands, the checks with their texts, the
// model's limits, the examples and the drawing's sizes) come from includes/motion-lab.html, which
// scripts/figures/motion_lab_figures.py writes. This file adds the behavior. The figure shows its
// static version until the first drawing here has succeeded.
//
// The file's parts, in this order: the drawing's functions, which follow the script's functions
// that draw the static version (drawingScale() is drawing_scale(), gridGroup() is grid_lines(),
// robotOutlineGroup() is robot_outline(), buildDriveSvg() is static_drive_svg(), and so on);
// bindSliderResets() for the double click and the double tap on a slider; bindEnlargedView() for
// the enlarged figure's dialog; and mountMotionLab(), which owns one figure's state.
import { computeDriveViews, relativeMotion, predictChecks } from './diffdrive.js';

const SVG_NS = 'http://www.w3.org/2000/svg';
const MILLISECONDS_PER_SECOND = 1000;
// Screen readers hear the result once a change is complete: this long after the last change
const ANNOUNCE_DELAY_MS = 400;
// A tap lasts less than TAP_MAX_MS and stays within TAP_SLOP_PX of its starting point. Two taps
// reset the slider when their releases are less than DOUBLE_TAP_MS apart and less than
// DOUBLE_TAP_DISTANCE_PX apart on screen.
const DOUBLE_TAP_MS = 350;
const TAP_MAX_MS = 300;
const TAP_SLOP_PX = 10;
const DOUBLE_TAP_DISTANCE_PX = 30;
// The Enlarge button shows from this window width on. bindEnlargedView() alone shows and hides it
const WIDE_MIN_EM = 60;

const prefersReducedMotion = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

/** A value with `digits` decimals and a true minus sign. A value that rounds to 0 shows as 0. */
function formatDecimal(value, digits = 3) {
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
function robotPointOnGround(pose, forward, left) {
  const cos = Math.cos(pose.theta), sin = Math.sin(pose.theta);
  return { x: pose.x + forward * cos - left * sin, y: pose.y + forward * sin + left * cos };
}

/**
 * How far (m) the drawing reaches beyond the paths' poses: the wheels' outer edges of both
 * outlines, the robot's arrow, the command's line and base_footprint behind the axle, with the
 * margin.
 */
function outlineReachMetres(inputs, sizes) {
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
function drawingScale(width, samples, reach, sizes) {
  const poses = samples.flatMap((sample) => [sample.command, sample.odometry, sample.axle, sample.frame]);
  const xs = poses.map((pose) => pose.x), ys = poses.map((pose) => pose.y);
  const left = Math.min(...xs) - reach, right = Math.max(...xs) + reach;
  const bottom = Math.min(...ys) - reach, top = Math.max(...ys) + reach;
  const height = Math.round(width * sizes.aspect);
  const pixelsPerMetre = Math.min((width - 2 * sizes.paddingPx) / (right - left),
    (height - 2 * sizes.paddingPx) / (top - bottom));
  const centerX = (left + right) / 2, centerY = (bottom + top) / 2;
  // A point on the ground (m) as the drawing's point (px): x to the right and y down
  const toPixels = (point) => ({
    x: width / 2 + (point.x - centerX) * pixelsPerMetre,
    y: height / 2 - (point.y - centerY) * pixelsPerMetre,
  });
  return { width, height, centerX, centerY, pixelsPerMetre, toPixels };
}

/** The grid's spacing (m): the smallest step whose lines lie far enough apart. */
function gridStep(scale, sizes) {
  return sizes.gridSteps.find((step) => step * scale.pixelsPerMetre >= sizes.gridSpacingPx)
    ?? sizes.gridSteps.at(-1);
}

/** The grid: lines at the multiples of the step across the whole drawing. */
function gridGroup(scale, step) {
  const grid = svgElement('g', { class: 'ix-grid' });
  const halfWidth = scale.width / 2 / scale.pixelsPerMetre, halfHeight = scale.height / 2 / scale.pixelsPerMetre;
  for (let index = Math.ceil((scale.centerX - halfWidth) / step); index * step < scale.centerX + halfWidth; index++) {
    const { x } = scale.toPixels({ x: index * step, y: 0 });
    grid.append(svgElement('line', { x1: x, y1: 0, x2: x, y2: scale.height }));
  }
  for (let index = Math.ceil((scale.centerY - halfHeight) / step); index * step < scale.centerY + halfHeight; index++) {
    const { y } = scale.toPixels({ x: 0, y: index * step });
    grid.append(svgElement('line', { x1: 0, y1: y, x2: scale.width, y2: y }));
  }
  return grid;
}

/** Points of the drawing as an SVG points attribute, to a tenth of a pixel. */
const pointsAttribute = (points) => points.map((point) => `${point.x.toFixed(1)},${point.y.toFixed(1)}`).join(' ');

/** A view's path through its poses, or null before the path has two poses. */
function pathPolyline(scale, poses, cssClass) {
  if (poses.length < 2) return null;
  return svgElement('polyline', { class: `ix-path ${cssClass}`, points: pointsAttribute(poses.map(scale.toPixels)) });
}

/** A robot from above at a pose of its axle's midpoint: two wheels, the axle and an arrow to its front. */
function robotOutlineGroup(scale, pose, wheels, cssClass, title, sizes) {
  // Map robot-frame metres to drawing pixels
  const robotPointInPixels = (forward, left) => scale.toPixels(robotPointOnGround(pose, forward, left));
  const group = svgElement('g', { class: `ix-robot ${cssClass}` });
  group.append(svgElement('title', {}, title));
  for (const side of [1, -1]) {
    const middle = side * wheels.separation / 2, halfWidth = wheels.wheelWidth / 2;
    const corners = [
      robotPointInPixels(wheels.radius, middle + halfWidth), robotPointInPixels(wheels.radius, middle - halfWidth),
      robotPointInPixels(-wheels.radius, middle - halfWidth), robotPointInPixels(-wheels.radius, middle + halfWidth),
    ];
    group.append(svgElement('polygon', { class: 'ix-wheel', points: pointsAttribute(corners) }));
  }
  const leftEnd = robotPointInPixels(0, wheels.separation / 2), rightEnd = robotPointInPixels(0, -wheels.separation / 2);
  group.append(svgElement('line', { x1: leftEnd.x, y1: leftEnd.y, x2: rightEnd.x, y2: rightEnd.y }));
  const middle = robotPointInPixels(0, 0), nose = robotPointInPixels(sizes.nose, 0);
  group.append(svgElement('line', { x1: middle.x, y1: middle.y, x2: nose.x, y2: nose.y }));
  const head = [
    robotPointInPixels(sizes.nose + sizes.headLength, 0),
    robotPointInPixels(sizes.nose, sizes.headHalfWidth), robotPointInPixels(sizes.nose, -sizes.headHalfWidth),
  ];
  group.append(svgElement('polygon', { class: 'ix-head', points: pointsAttribute(head) }));
  return group;
}

/** The model's markers: a dot at base_footprint, and with an offset the axle's midpoint joined to it. */
function appendTruePoseMarkers(group, scale, axle, frameOffset, sizes) {
  const frame = scale.toPixels(robotPointOnGround(axle, -frameOffset, 0));
  if (frameOffset > 0) {
    const middle = scale.toPixels(axle);
    group.append(svgElement('line', { class: 'ix-link', x1: middle.x, y1: middle.y, x2: frame.x, y2: frame.y }));
    const axleDot = svgElement('circle', { class: 'ix-dot ix-axle', cx: middle.x, cy: middle.y, r: sizes.axleDotRadiusPx });
    axleDot.append(svgElement('title', {}, 'The axle’s midpoint'));
    group.append(axleDot);
  }
  const frameDot = svgElement('circle', { class: 'ix-dot', cx: frame.x, cy: frame.y, r: sizes.dotRadiusPx });
  frameDot.append(svgElement('title', {}, 'base_footprint, true'));
  group.append(frameDot);
}

/** The odometry's marker: a ring at its base_footprint. */
function appendOdometryMarker(group, scale, pose, sizes) {
  const ring = scale.toPixels(pose);
  group.append(svgElement('circle', { class: 'ix-ring', cx: ring.x, cy: ring.y, r: sizes.ringRadiusPx }));
}

/** The command's end: a cross where the command alone puts the robot, with a line in its heading. */
function commandMarkerGroup(scale, pose, sizes) {
  const cross = scale.toPixels(pose);
  const heading = scale.toPixels(robotPointOnGround(pose, sizes.commandHeading, 0));
  const half = sizes.crossHalfPx;
  const group = svgElement('g', { class: 'ix-robot ix-command' });
  group.append(svgElement('title', {}, 'Command'));
  group.append(svgElement('line', { x1: cross.x, y1: cross.y, x2: heading.x, y2: heading.y }));
  group.append(svgElement('path', {
    class: 'ix-cross', d: `M${cross.x - half},${cross.y - half} l${2 * half},${2 * half} m0,${-2 * half} l${-2 * half},${2 * half}`,
  }));
  return group;
}

/**
 * The live drawing of a drive up to one of its samples: the grid, the views' paths so far, the
 * robot at the start, and the model, the odometry and the command at the sample. `labels` are the
 * drawing's accessible name and the id of the element that describes it. Returns the SVG element
 * and the grid's spacing (m).
 */
function buildDriveSvg(drive, shownIndex, width, sizes, labels) {
  const { inputs, samples } = drive;
  const scale = drawingScale(width, samples, drive.reach, sizes);
  const gridStepMetres = gridStep(scale, sizes);
  const shownSamples = samples.slice(0, shownIndex + 1), shownSample = samples[shownIndex];
  const svg = svgElement('svg', {
    class: 'ix-live-plot', viewBox: `0 0 ${scale.width} ${scale.height}`, role: 'img',
    'aria-label': labels.name, 'aria-describedby': labels.descriptionId,
  });
  svg.append(gridGroup(scale, gridStepMetres));
  const paths = [['frame', 'ix-model'], ['odometry', 'ix-odometry'], ['command', 'ix-command']];
  if (inputs.frameOffset > 0) paths.unshift(['axle', 'ix-axle']);
  for (const [view, cssClass] of paths) {
    const path = pathPolyline(scale, shownSamples.map((sample) => sample[view]), cssClass);
    if (path) svg.append(path);
  }
  svg.append(robotOutlineGroup(scale, samples[0].axle, inputs.wheels, 'ix-start', 'The robot at the start', sizes));
  const truePose = robotOutlineGroup(scale, shownSample.axle, inputs.wheels, 'ix-model',
    'True pose, in this figure’s model', sizes);
  appendTruePoseMarkers(truePose, scale, shownSample.axle, inputs.frameOffset, sizes);
  svg.append(truePose);
  const odometry = robotOutlineGroup(scale, shownSample.odometry, inputs.configured, 'ix-odometry',
    'Odometry: the controller’s geometry at the pose that it computes', sizes);
  appendOdometryMarker(odometry, scale, shownSample.odometry, sizes);
  svg.append(odometry);
  svg.append(commandMarkerGroup(scale, shownSample.command, sizes));
  return { svg, gridStepMetres };
}

// --- Resetting one slider --------------------------------------------------------------------

/**
 * Bind the gestures that set one slider back to the robot's value: a double click, and on a touch
 * screen a double tap. resetSlider(input) does the reset.
 *
 * A tap is a short touch that hardly moves: two quick drags are no double tap, and any other press
 * in between starts over. Mice and pens use the browser's double click only. The presses' history
 * belongs to these sliders, so each figure has its own.
 */
function bindSliderResets(inputs, resetSlider) {
  let press = null, lastTap = null, lastTouchResetTime = -Infinity;
  const isPrimaryTouch = (event) => event.pointerType === 'touch' && event.isPrimary;

  /** The tap that a release on a slider completes, or null if the press was no tap. */
  function tapFromRelease(event, input) {
    const isTap = press && press.input === input && press.id === event.pointerId && !press.moved
      && isPrimaryTouch(event) && event.timeStamp - press.time < TAP_MAX_MS
      && Math.hypot(event.clientX - press.x, event.clientY - press.y) <= TAP_SLOP_PX;
    return isTap ? { input, x: event.clientX, y: event.clientY, time: event.timeStamp } : null;
  }

  /** Whether a tap follows the tap before it on the same slider, soon enough and close enough. */
  function isSecondTap(tap) {
    return Boolean(tap && lastTap && lastTap.input === tap.input && tap.time - lastTap.time < DOUBLE_TAP_MS
      && Math.hypot(tap.x - lastTap.x, tap.y - lastTap.y) < DOUBLE_TAP_DISTANCE_PX);
  }

  // A press on anything else in between: the next tap is a first tap again
  document.addEventListener('pointerdown', (event) => {
    if (lastTap && event.target !== lastTap.input) lastTap = null;
  }, true);
  for (const input of inputs) {
    // A browser that sends a double click after a double tap too resets the slider once
    input.addEventListener('dblclick', (event) => {
      if (event.timeStamp - lastTouchResetTime > DOUBLE_TAP_MS) resetSlider(input);
    });
    input.addEventListener('pointerdown', (event) => {
      press = isPrimaryTouch(event)
        ? { input, id: event.pointerId, x: event.clientX, y: event.clientY, time: event.timeStamp, moved: false } : null;
      if (!press) lastTap = null;
    });
    input.addEventListener('pointermove', (event) => {
      if (press && Math.hypot(event.clientX - press.x, event.clientY - press.y) > TAP_SLOP_PX) press.moved = true;
    });
    input.addEventListener('pointercancel', () => { press = null; lastTap = null; });
    input.addEventListener('pointerup', (event) => {
      const tap = tapFromRelease(event, input);
      press = null;
      const second = isSecondTap(tap);
      lastTap = second ? null : tap;
      if (second) { lastTouchResetTime = event.timeStamp; resetSlider(input); }
    });
  }
}

// --- The enlarged view -----------------------------------------------------------------------

/**
 * Bind the Enlarge button: the figure over the whole window in a modal dialog, with the drawing
 * and the values next to each other. The rest of the page is out of the keyboard's and the screen
 * reader's reach meanwhile. The button, Esc, printing or a window that gets too narrow bring the
 * figure back to its place on the page.
 */
function bindEnlargedView(root) {
  const find = (selector) => root.querySelector(selector);
  const enlarge = find('.ix-enlarge'), settings = find('.ix-settings'), caption = find('figcaption');
  const stage = find('.ix-stage'), side = find('.ix-side');
  const wideEnough = matchMedia(`(min-width: ${WIDE_MIN_EM}em)`);
  const placeholder = document.createComment('The enlarged figure returns here');
  let dialog = null, settingsWereOpen = false;

  function enlargeFigure() {
    if (dialog || !wideEnough.matches) return;
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
  // Below the width the button hides, and an enlarged figure returns to the page
  function syncEnlargeAvailability() {
    enlarge.hidden = !wideEnough.matches;
    if (enlarge.hidden) returnToPage();
  }

  syncEnlargeAvailability();
  wideEnough.addEventListener('change', syncEnlargeAvailability);
  addEventListener('beforeprint', returnToPage);
}

// --- The figure ------------------------------------------------------------------------------

/**
 * Start one figure. This function owns the figure's state: the robot, the command, the example
 * and the share of the drive that has played, the drive computed from them, and which questions
 * are answered. Its inner functions are grouped by what they do with that state:
 *
 * - compute: readModelInputs() and computeDrive() turn the controls into a drive.
 * - render: renderDriveAtCurrentTime() draws the drive at the play bar's time, with the time's
 *   label and the drawing's description. recomputeAndRenderDrive() computes the drive again and
 *   renders everything that shows it. Call the first after a change of the time only, the second
 *   after a change of a value, the robot or the command.
 * - results: recordDriveCompletion() notes that a drive has reached its end, and
 *   renderResultState() shows what is due. updateResults() does both, after every change.
 * - controls: Play and the time slider, the sliders and the robot, the examples.
 */
function mountMotionLab(root) {
  const data = JSON.parse(root.querySelector('.ix-data').textContent);
  const sizes = data.drawing;
  const find = (selector) => root.querySelector(selector);
  const findAll = (selector) => [...root.querySelectorAll(selector)];
  const plot = find('.ix-plot'), scrub = find('.ix-scrub');
  const sliders = Object.fromEntries(findAll('input[data-setting]').map((input) => [input.dataset.setting, input]));
  // playbackFraction is the share of the drive that has played: 0 at the start, 1 at the end
  const state = { robot: data.default.robot, command: data.default.command, playbackFraction: 0, step: 0 };
  // The questions, one per example and drive ("radius:forward"), whose drive has reached its end
  // with the example's own values: their answer shows while those values are set
  const answered = new Set();
  const questionKey = (command = state.command) => `${data.examples[state.step].key}:${command}`;
  // A drive has reached its end since the example opened: the results show
  let reachedEnd = false;
  let animationRequest = null, lastFrameTimestamp = 0, announceTimer = null, drive = null, liveDrawing = null;

  // --- Compute: the model's inputs from the controls, and the drive --------------------------
  const sliderNumber = (name) => Number(sliders[name].value);
  /** The model's inputs in SI units, as model_inputs() in the script computes them. */
  function readModelInputs() {
    const robot = data.robots[state.robot];
    return {
      robot,
      command: data.commands[state.command],
      wheels: { radius: robot.radius, separation: robot.separation, wheelWidth: robot.wheelWidth },
      configured: {
        radius: robot.radius * sliderNumber('radius_percent') / 100,
        separation: robot.separation * sliderNumber('separation_percent') / 100,
        wheelWidth: robot.wheelWidth,
      },
      contactSeparation: robot.separation + robot.wheelWidth * sliderNumber('contact_percent') / 100,
      frameOffset: sliderNumber('offset_mm') / 1000,
      tracking: sliderNumber('tracking_percent') / 100,
    };
  }

  /** The drive of the current controls: its inputs, its poses over time, its motions and checks. */
  function computeDrive() {
    const inputs = readModelInputs(), command = inputs.command;
    const { posesAt } = computeDriveViews({
      v: command.linearVelocity, w: command.angularVelocity,
      radius: inputs.wheels.radius, separation: inputs.wheels.separation,
      configuredRadius: inputs.configured.radius, configuredSeparation: inputs.configured.separation,
      contactSeparation: inputs.contactSeparation, frameOffset: inputs.frameOffset, tracking: inputs.tracking,
      straightAngularVelocity: data.modelLimits.straightAngularVelocity,
    });
    const samples = [];
    for (let index = 0; index <= sizes.samples; index++) samples.push(posesAt(command.duration * index / sizes.samples));
    const start = samples[0], end = samples[sizes.samples];
    const motion = {
      command: relativeMotion(start.command, end.command),
      odometry: relativeMotion(start.odometry, end.odometry),
      model: relativeMotion(start.frame, end.frame),
    };
    const checks = predictChecks(data.checks[state.command], motion.odometry, motion.model,
      data.modelLimits.roundingAllowance);
    return { kind: state.command, inputs, samples, motion, checks, reach: outlineReachMetres(inputs, sizes) };
  }

  // --- Render: the drawing at the play bar's time --------------------------------------------
  /** The time under the drawing and the time slider's spoken value. */
  function updateTimeLabel(elapsedSeconds, durationSeconds) {
    const elapsed = elapsedSeconds.toFixed(1), duration = durationSeconds.toFixed(1);
    find('.ix-time').textContent = state.playbackFraction >= 1
      ? `t = ${duration} s, the end` : `t = ${elapsed} s of ${duration} s`;
    scrub.setAttribute('aria-valuetext', `${elapsed} of ${duration} seconds`);
  }

  // For screen readers, the drawing's description: what it shows at the play bar's time, so the
  // drive's end stays unknown until the drive gets there
  function updateDrawingDescription(shownSample) {
    const { samples, inputs, kind } = drive, start = samples[0];
    const command = relativeMotion(start.command, shownSample.command);
    const odometry = relativeMotion(start.odometry, shownSample.odometry);
    const model = relativeMotion(start.frame, shownSample.frame);
    const timePrefix = `At ${shownSample.t.toFixed(1)} s of ${inputs.command.duration.toFixed(1)} s, `;
    find('.ix-now').textContent = kind === 'forward'
      ? `${timePrefix}the commanded distance is ${formatDecimal(command.forward)} m. The odometry reports`
        + ` ${formatDecimal(odometry.forward)} m and the robot in the model has travelled ${formatDecimal(model.forward)} m.`
      : `${timePrefix}the commanded angle is ${formatDecimal(command.turned)} rad. The odometry reports ${formatDecimal(odometry.turned)} rad`
        + ` and the robot in the model has turned ${formatDecimal(model.turned)} rad. Its base_footprint is`
        + ` ${formatDecimal(Math.hypot(model.forward, model.left))} m from its starting point.`;
  }

  /**
   * Draw the computed drive at the play bar's time: the drawing, the grid's note, and the time as
   * the eye reads it and as a screen reader hears it. Enough after a change of the time alone.
   */
  function renderDriveAtCurrentTime() {
    const shownIndex = Math.round(state.playbackFraction * sizes.samples);
    const shownSample = drive.samples[shownIndex];
    const width = Math.max(sizes.minimumWidthPx, Math.round(plot.clientWidth));
    const { svg, gridStepMetres } = buildDriveSvg(drive, shownIndex, width, sizes, {
      name: find('.ix-static title').textContent, descriptionId: find('.ix-now').id,
    });
    if (liveDrawing) liveDrawing.replaceWith(svg); else plot.append(svg);
    liveDrawing = svg;
    find('.ix-gridnote').textContent = `Grid: ${Math.round(gridStepMetres * 100)} cm`;
    updateTimeLabel(shownSample.t, drive.inputs.command.duration);
    updateDrawingDescription(shownSample);
  }

  // --- Render: the numbers next to the drawing -----------------------------------------------
  function renderMotionTable({ motion }) {
    const legendGlyphSvg = (view) => find(`.ix-legend [data-view="${view}"] svg`).outerHTML;
    find('.ix-motion-rows').innerHTML = Object.entries(data.viewNames).map(([view, name]) => {
      const { forward, left, turned } = motion[view];
      return `<tr><th scope="row">${legendGlyphSvg(view)}${name}</th><td>${formatDecimal(forward)} m</td><td>${formatDecimal(left)} m</td><td>${formatDecimal(turned)} rad</td></tr>`;
    }).join('');
  }

  function renderCheckTable({ checks }) {
    find('.ix-check-rows').innerHTML = checks.map((check) => {
      const result = check.passes ? '<span class="ix-pass">✔ passes</span>' : '<span class="ix-fail">✖ fails</span>';
      return `<tr><th scope="row">${check.name}</th><td data-label="Allowed">${check.allowed}</td><td data-label="This drive">${formatDecimal(check.value)} ${check.unit}</td><td>${result}</td></tr>`;
    }).join('');
  }

  function renderSummary({ kind, inputs, motion, checks }) {
    const { odometry, model } = motion;
    let text = `${inputs.robot.name}, ${inputs.command.text}. `;
    if (kind === 'forward') {
      text += `The odometry reports ${formatDecimal(odometry.forward)} m. In the model, the robot drives ${formatDecimal(model.forward)} m. `;
    } else {
      text += `The odometry reports a turn of ${formatDecimal(odometry.turned)} rad. In the model, the robot turns ${formatDecimal(model.turned)} rad`
        + ` and base_footprint ends ${formatDecimal(Math.hypot(model.forward, model.left))} m from its start. `;
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

  /** Each slider's value next to its label, as the eye reads it and as a screen reader hears it. */
  function renderSliderOutputs(inputs) {
    const setSliderOutput = (name, shown, spoken) => {
      sliders[name].closest('.ix-control').querySelector('output').textContent = shown;
      sliders[name].setAttribute('aria-valuetext', spoken);
    };
    const formatMillimetres = (metres, digits) => (metres * 1000).toFixed(digits);
    const radius = formatMillimetres(inputs.configured.radius, 1), separation = formatMillimetres(inputs.configured.separation, 1);
    const contact = formatMillimetres(inputs.contactSeparation, 0);
    setSliderOutput('radius_percent', `${sliderNumber('radius_percent')} % · ${radius} mm`, `${sliderNumber('radius_percent')} percent, ${radius} millimeters`);
    setSliderOutput('separation_percent', `${sliderNumber('separation_percent')} % · ${separation} mm`,
      `${sliderNumber('separation_percent')} percent, ${separation} millimeters`);
    setSliderOutput('offset_mm', `${sliderNumber('offset_mm')} mm`, `${sliderNumber('offset_mm')} millimeters behind the axle`);
    setSliderOutput('contact_percent', `${sliderNumber('contact_percent')} % · ${contact} mm apart`,
      `${sliderNumber('contact_percent')} percent, ${contact} millimeters apart`);
    setSliderOutput('tracking_percent', `${sliderNumber('tracking_percent')} %`, `${sliderNumber('tracking_percent')} percent of the commanded speed`);
  }

  // The legend's entry of the axle's midpoint keeps its place while unused, so the legend keeps
  // its lines
  function updateAxleLegend(inputs) {
    for (const entry of findAll('.ix-legend [data-view="axle"]')) {
      entry.hidden = false;
      entry.style.visibility = inputs.frameOffset === 0 ? 'hidden' : '';
    }
  }

  /**
   * Compute the drive of the current controls and render all that shows it: the sliders' values,
   * the legend, the tables, the summary and the drawing. Needed after a change of a value, the
   * robot or the command.
   */
  function recomputeAndRenderDrive() {
    drive = computeDrive();
    renderSliderOutputs(drive.inputs);
    updateAxleLegend(drive.inputs);
    renderMotionTable(drive);
    renderCheckTable(drive);
    renderSummary(drive);
    renderDriveAtCurrentTime();
  }

  // --- Small changes of the state, each with the control that shows it -----------------------
  const defaultSliderValues = () => Object.fromEntries(Object.entries(sliders).map(([name, input]) => [name, input.defaultValue]));
  function resetSliders() {
    for (const [name, value] of Object.entries(defaultSliderValues())) sliders[name].value = value;
  }
  function selectCommand(command) {
    state.command = command;
    for (const button of findAll('[data-command]')) button.setAttribute('aria-pressed', String(button.dataset.command === command));
  }
  function selectRobot(robot) {
    state.robot = robot;
    find('.ix-robot-select').value = robot;
  }
  function setPlaybackFraction(fraction) {
    state.playbackFraction = fraction;
    scrub.value = Math.round(fraction * Number(scrub.max));
  }
  /** The example's own values: the example robot's, with the example's one value changed. */
  const exampleSliderValues = () => ({ ...defaultSliderValues(), ...data.examples[state.step].settings });
  /** Whether the controls hold the current example's own robot and values, on either drive. */
  function matchesExampleSettings() {
    const own = exampleSliderValues();
    return state.robot === data.default.robot
      && Object.entries(sliders).every(([name, input]) => Number(input.value) === Number(own[name]));
  }

  // --- Results: what shows once a drive has reached its end -----------------------------------
  // The summary and the tables show the drive's end: they wait until a drive has reached its end
  // since the example opened, then follow every change. A question is answered the first time the
  // controls hold the example's own drive at its end: another drive's end shows its results but
  // answers nothing. The answer then shows while the example's own values are set.

  /**
   * Note what the drive's end, if the play bar is there, completes: the results show from now on,
   * and the example's question is answered if the controls hold its own values.
   * Returns what has just appeared: 'answer', 'results' or null.
   */
  function recordDriveCompletion() {
    const atEnd = state.playbackFraction >= 1, key = questionKey();
    const resultsAppear = atEnd && !reachedEnd;
    if (resultsAppear) reachedEnd = true;
    const answerAppears = atEnd && matchesExampleSettings() && !answered.has(key);
    if (answerAppears) answered.add(key);
    if (answerAppears) return 'answer';
    return resultsAppear ? 'results' : null;
  }

  // Under the sliders, while other values are set: the values that differ from the example's, with
  // the example's. Below the last slider, the list grows and shrinks without moving a slider.
  function renderChanges(own) {
    const line = find('.ix-changes');
    line.textContent = '';
    line.hidden = own;
    if (own) return;
    const values = exampleSliderValues(), robots = data.robots;
    const lowercaseFirstLetter = (text) => text.charAt(0).toLowerCase() + text.slice(1);
    const changes = Object.entries(sliders).filter(([name, input]) => Number(input.value) !== Number(values[name]))
      .map(([name, input]) => {
        const label = lowercaseFirstLetter(input.closest('.ix-control').querySelector('label').textContent);
        return `${label} ${input.value} ${input.dataset.unit} (the example: ${values[name]} ${input.dataset.unit})`;
      });
    if (state.robot !== data.default.robot) {
      changes.unshift(`${robots[state.robot].name} (the example: ${robots[data.default.robot].name})`);
    }
    const lead = document.createElement('b');
    lead.textContent = data.texts.changesLead;
    line.append(lead, ` ${changes.join(', ')}.`);
  }

  /**
   * Show what the recorded state says: the results or their wait, and under the play bar the
   * answer, the hint before it, or the note that other values are set. The three share one place,
   * as high as the highest of them, so nothing below moves.
   */
  function renderResultState() {
    const own = matchesExampleSettings();
    const showAnswer = own && answered.has(questionKey());
    find('.ix-shown-answer').classList.toggle('ix-off', !showAnswer);
    find('.ix-hint').classList.toggle('ix-off', !own || showAnswer);
    find('.ix-differs').classList.toggle('ix-off', own);
    renderChanges(own);
    // While the answer shows, the example's button keeps its place, invisible, so the questions'
    // height stays. With other values it shows again and sets the example's values back.
    const item = find('.ix-current');
    item.classList.toggle('ix-revealed', showAnswer);
    item.querySelector('.ix-show').textContent = own ? data.texts.show : data.texts.replay;
    root.classList.toggle('ix-waiting', !reachedEnd);
  }

  /** Record a drive's end and show the results' state. Returns what has just appeared. */
  function updateResults() {
    const appeared = recordDriveCompletion();
    renderResultState();
    return appeared;
  }

  // --- Announcements for screen readers ------------------------------------------------------
  // Screen readers hear the result once a change is complete, not while a slider moves, and only
  // while the results show
  function scheduleSummaryAnnouncement() {
    clearTimeout(announceTimer);
    announceTimer = setTimeout(() => {
      if (reachedEnd) find('.ix-announce').textContent = find('.ix-summary').textContent;
    }, ANNOUNCE_DELAY_MS);
  }
  // Screen readers hear what has just appeared at the drive's end: the answer replaces a summary
  // that waits to be heard
  function announceNewResult(appeared) {
    if (appeared === 'answer') {
      clearTimeout(announceTimer);
      find('.ix-announce').textContent = find('.ix-shown-answer').textContent;
    }
    if (appeared === 'results') scheduleSummaryAnnouncement();
  }
  // Screen readers hear the new example's question
  function announceQuestion() {
    clearTimeout(announceTimer);
    const question = find('.ix-current .ix-question.ix-drive').textContent;
    find('.ix-announce').textContent = `${find('.ix-step-count').textContent}. ${question}`;
  }

  // --- Controls: Play and the time slider ----------------------------------------------------
  // The figure shows one drive: the robot, the command and the values that the controls hold, at
  // the time of the play bar. Only Play, the time slider and an example's opening move the time.
  function stopPlaying() {
    if (animationRequest) cancelAnimationFrame(animationRequest);
    animationRequest = null;
    find('.ix-play').textContent = 'Play';
  }
  function startPlaying() {
    if (state.playbackFraction >= 1) setPlaybackFraction(0);
    lastFrameTimestamp = performance.now();
    find('.ix-play').textContent = 'Pause';
    const advancePlaybackFrame = (timestamp) => {
      // Model time: the drive's own seconds, whatever the drawing rate is
      const durationMs = data.commands[state.command].duration * MILLISECONDS_PER_SECOND;
      setPlaybackFraction(Math.min(1, state.playbackFraction + (timestamp - lastFrameTimestamp) / durationMs));
      lastFrameTimestamp = timestamp;
      renderDriveAtCurrentTime();
      if (state.playbackFraction < 1) { animationRequest = requestAnimationFrame(advancePlaybackFrame); return; }
      stopPlaying();
      announceNewResult(updateResults());
    };
    animationRequest = requestAnimationFrame(advancePlaybackFrame);
  }
  // With reduced motion, Play shows the drive's end at once. At the end already, as an example
  // that was played opens, Play shows the start instead, and the next Play the end again
  function play() {
    if (prefersReducedMotion()) {
      stopPlaying();
      setPlaybackFraction(state.playbackFraction >= 1 ? 0 : 1);
      renderDriveAtCurrentTime();
      announceNewResult(updateResults());
      return;
    }
    startPlaying();
  }
  find('.ix-play').addEventListener('click', () => {
    if (animationRequest) { stopPlaying(); return; }
    play();
  });
  scrub.addEventListener('input', () => {
    stopPlaying();
    setPlaybackFraction(Number(scrub.value) / Number(scrub.max));
    renderDriveAtCurrentTime();
    announceNewResult(updateResults());
  });
  // A hidden tab or a closed block draws nothing: stop instead of jumping ahead later
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopPlaying(); });

  // --- Controls: the sliders, the robot and the resets ---------------------------------------
  // A changed value or robot shows its drive at the same time, with the results' state
  function refreshAfterControlChange() {
    recomputeAndRenderDrive();
    updateResults();
  }
  for (const input of Object.values(sliders)) {
    input.addEventListener('input', refreshAfterControlChange);
    input.addEventListener('change', scheduleSummaryAnnouncement);
  }
  bindSliderResets(Object.values(sliders), (input) => {
    input.value = input.defaultValue;
    refreshAfterControlChange();
    scheduleSummaryAnnouncement();
  });
  find('.ix-robot-select').addEventListener('change', (event) => {
    selectRobot(event.target.value);
    resetSliders();
    refreshAfterControlChange();
    scheduleSummaryAnnouncement();
  });
  // Reset: next to the examples and in the block of values
  for (const button of findAll('.ix-reset')) {
    button.addEventListener('click', () => { resetSliders(); refreshAfterControlChange(); scheduleSummaryAnnouncement(); });
  }

  // --- Controls: the guided examples ---------------------------------------------------------
  // The guided examples, one at a time. An example opens with its question and its own drive,
  // with its one value changed, at the drive's start: the time slider and Play show the drive, and
  // its results and answer show once the drive has reached its end. "Show what happens" plays the
  // example's own drive from its start. An answered example opens at its drive's end.
  // The examples' questions share one place above the drawing, as high as the longest of them,
  // so the drawing stays where it is from one example to the next.
  const previousButton = find('.ix-previous'), nextButton = find('.ix-next');

  /** The stepper for the example at an index: its item in the list, the count and the buttons. */
  function renderExampleStepper(index) {
    const example = data.examples[index];
    for (const item of findAll('[data-example-item]')) {
      item.classList.toggle('ix-current', item.dataset.exampleItem === example.key);
      item.classList.remove('ix-revealed');
      for (const answer of item.querySelectorAll('.ix-answer')) answer.hidden = true;
    }
    find('.ix-step-count').textContent = `Example ${index + 1} of ${data.examples.length}`;
    previousButton.disabled = index === 0;
    nextButton.disabled = index === data.examples.length - 1;
  }

  /** The current example's question and answer for the current drive. */
  function renderQuestionAndAnswer() {
    const item = find('.ix-current');
    for (const question of item.querySelectorAll('.ix-question')) {
      question.classList.toggle('ix-drive', question.dataset.drive === state.command);
    }
    const answer = item.querySelector(`.ix-answer[data-drive="${state.command}"]`);
    find('.ix-shown-answer').innerHTML = `<b>What happens:</b> ${answer.innerHTML}`;
  }

  /**
   * Open an example with its own robot and values, on its own command or the given one. Without
   * a playback fraction it opens at its drive's start, or at its end once answered.
   */
  function openExample(index, command = data.examples[index].command, playbackFraction) {
    state.step = index;
    renderExampleStepper(index);
    stopPlaying();
    selectRobot(data.default.robot);
    resetSliders();
    for (const [name, value] of Object.entries(data.examples[index].settings)) sliders[name].value = value;
    selectCommand(command);
    renderQuestionAndAnswer();
    reachedEnd = false;
    setPlaybackFraction(playbackFraction ?? (answered.has(questionKey()) ? 1 : 0));
    recomputeAndRenderDrive();
    updateResults();
  }

  /** Play the current example's own drive from its start, on the command that shows. */
  function replayExample() {
    // A summary that waits to be heard belongs to the values before the replay
    clearTimeout(announceTimer);
    openExample(state.step, state.command, 0);
    if (prefersReducedMotion()) {
      // The answer shows at once and takes the keyboard's place: screen readers read it there
      stopPlaying();
      setPlaybackFraction(1);
      renderDriveAtCurrentTime();
      updateResults();
      find('.ix-shown-answer').focus();
      return;
    }
    // The keyboard's place moves to Pause while the drive plays, and stays where the reader
    // leaves it when the drive ends
    startPlaying();
    find('.ix-play').focus({ preventScroll: true });
  }

  // The other drive is the example's other question: it starts at its drive's start, or at its
  // end once answered. The values stay as they are.
  function switchCommand(command) {
    // The command that shows already: nothing changes, not even the time
    if (command === state.command) return;
    stopPlaying();
    selectCommand(command);
    renderQuestionAndAnswer();
    reachedEnd = false;
    setPlaybackFraction(answered.has(questionKey()) ? 1 : 0);
    refreshAfterControlChange();
    announceQuestion();
  }

  // At the first and the last example, the button just pressed is disabled: the keyboard's place
  // moves to the other one
  function navigateToExample(index, pressed, other) {
    openExample(index);
    announceQuestion();
    if (pressed.disabled && document.activeElement !== other) other.focus();
  }

  for (const button of findAll('[data-example]')) button.addEventListener('click', replayExample);
  for (const button of findAll('[data-command]')) {
    button.addEventListener('click', () => switchCommand(button.dataset.command));
  }
  previousButton.addEventListener('click', () => navigateToExample(state.step - 1, previousButton, nextButton));
  nextButton.addEventListener('click', () => navigateToExample(state.step + 1, nextButton, previousButton));

  // --- Start ---------------------------------------------------------------------------------
  bindEnlargedView(root);
  new ResizeObserver(() => { if (drive) renderDriveAtCurrentTime(); }).observe(plot);

  // First drawing: the first example's question and its own drive at the start. Only when it has
  // succeeded does the figure switch from the static version to the controls.
  for (const element of findAll('[data-live]')) element.hidden = false;
  openExample(0);
  root.classList.add('ix-is-live');
}

// Back to the static version: the drawing and the answers that the page came with
function restoreStaticFigure(root) {
  root.classList.remove('ix-is-live');
  for (const element of root.querySelectorAll('[data-live]')) element.hidden = true;
  for (const answer of root.querySelectorAll('.ix-answer:not([data-live])')) answer.hidden = false;
  for (const item of root.querySelectorAll('[data-example-item]')) item.classList.remove('ix-current', 'ix-revealed');
  for (const drawing of root.querySelectorAll('.ix-live-plot')) drawing.remove();
}

function mountMotionLabs() {
  for (const root of document.querySelectorAll('[data-ix="motion-lab"]:not(.ix-is-live)')) {
    try {
      mountMotionLab(root);
    } catch (error) {
      restoreStaticFigure(root);
      console.error('The interactive figure could not start:', error);
    }
  }
}

if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mountMotionLabs);
else mountMotionLabs();
