// The kinematic model of a differential drive robot that the interactive figures share.
// Pure functions without any page access, so a test can call them directly.
//
// Symbols: v (m/s) and w (rad/s) are the robot's linear and angular velocity, r the wheel radius
// (m), L the wheel separation (m, between the wheels' center planes). The wheels' angular
// velocities are in rad/s.

/** The same angle in -pi..pi, as wrap() in diffbot's test_drive.py. */
export function wrap(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

/**
 * The wheel speeds that diff_drive_controller commands for a velocity command, with the
 * wheel radius and separation of its configuration (geometry.yaml).
 */
export function wheelSpeedsForCommand(v, w, radius, separation) {
  return { left: (v - w * separation / 2) / radius, right: (v + w * separation / 2) / radius };
}

/** The robot's velocity that follows from two wheel speeds, for a wheel radius and separation. */
export function velocityFromWheelSpeeds(left, right, radius, separation) {
  return { v: radius * (right + left) / 2, w: radius * (right - left) / separation };
}

/**
 * The pose after driving with a constant velocity for the time t, starting at x = 0, y = 0 with
 * the heading 0. With w = 0 the robot drives straight. Otherwise it drives on a circle of radius
 * v / w. The heading isn't wrapped, so a path can be drawn through more than half a turn.
 */
export function poseAfter(v, w, t) {
  if (Math.abs(w) < 1e-12) return { x: v * t, y: 0, theta: 0 };
  const theta = w * t;
  return { x: v / w * Math.sin(theta), y: v / w * (1 - Math.cos(theta)), theta };
}

/**
 * The end pose relative to the start pose, in the robot's frame at the start: forward and left
 * (m) and the angle turned (rad, wrapped), as relative_pose() in diffbot's test_drive.py.
 */
export function relativePose(start, end) {
  const dx = end.x - start.x, dy = end.y - start.y;
  const c = Math.cos(start.theta), s = Math.sin(start.theta);
  return { forward: c * dx + s * dy, left: -s * dx + c * dy, turned: wrap(end.theta - start.theta) };
}

/**
 * One drive, seen three ways. The command (v, w) is held for the whole drive.
 *
 * - command: the pose that the command alone gives.
 * - odometry: the wheels turn at `tracking` times their commanded speeds, and the controller
 *   converts their turns back with its configured radius and separation.
 * - model: the same wheel turns, converted with the robot's own radius and the separation at
 *   which its wheels push on the ground (ideal rolling). `axle` is the axle's midpoint, and
 *   `frame` the reported frame, `frameOffset` behind the axle's midpoint along the heading.
 *
 * Returns the wheel speeds, the three velocities and a function that gives the poses at a time.
 */
export function threeViews({ v, w, radius, separation, configuredRadius, configuredSeparation,
  contactSeparation = separation, frameOffset = 0, tracking = 1 }) {
  const cmd = wheelSpeedsForCommand(v, w, configuredRadius, configuredSeparation);
  const left = tracking * cmd.left, right = tracking * cmd.right;
  const odometry = velocityFromWheelSpeeds(left, right, configuredRadius, configuredSeparation);
  const model = velocityFromWheelSpeeds(left, right, radius, contactSeparation);
  const at = (t) => {
    const axle = poseAfter(model.v, model.w, t);
    axle.x += frameOffset;  // The reported frame starts at 0, 0, so the axle starts ahead of it
    const frame = {
      x: axle.x - frameOffset * Math.cos(axle.theta),
      y: axle.y - frameOffset * Math.sin(axle.theta),
      theta: axle.theta,
    };
    return { t, command: poseAfter(v, w, t), odometry: poseAfter(odometry.v, odometry.w, t), axle, frame };
  };
  return { wheelSpeeds: { left, right }, velocities: { command: { v, w }, odometry, model }, at };
}

// A value on a tolerance's boundary can miss it by floating-point rounding: 1.5 - 1.2 gives
// 0.30000000000000004. This small allowance keeps such a value inside its range in the figures.
// It is a choice of the figures only. The ROS test compares without it.
const ROUNDING_ALLOWANCE = 1e-9;

/** Whether a value lies within expected +- tolerance, as assertAlmostEqual(..., delta=tolerance). */
export function within(value, expected, tolerance) {
  return Math.abs(value - expected) <= tolerance + ROUNDING_ALLOWANCE;
}

/**
 * The checks of test_drive.py for one drive, predicted from the odometry's and the model's
 * relative motion. `kind` is 'forward' or 'turn', `checks` the expected values and tolerances.
 * Each result has the check's id, the measured value, its unit and whether it passes.
 */
export function predictChecks(kind, odometry, model, checks) {
  const results = [];
  const add = (id, source, value, expected, tolerance, unit) => {
    results.push({ id, source, value, expected, tolerance, unit, ok: within(value, expected, tolerance) });
  };
  const sources = [['odometry', odometry], ['model', model]];
  if (kind === 'forward') {
    for (const [source, motion] of sources) {
      add('forward', source, motion.forward, checks.forward.expected, checks.forward.tolerance, 'm');
      add('left', source, motion.left, 0, checks.left.tolerance, 'm');
      add('turned', source, motion.turned, 0, checks.turned.tolerance, 'rad');
    }
    add('agreement', 'both', Math.abs(odometry.forward - model.forward), 0, checks.agreement.tolerance, 'm');
  } else {
    for (const [source, motion] of sources) {
      add('turned', source, motion.turned, checks.turned.expected, checks.turned.tolerance, 'rad');
      add('distance', source, Math.hypot(motion.forward, motion.left), 0, checks.distance.tolerance, 'm');
    }
    add('agreement', 'both', Math.abs(odometry.turned - model.turned), 0, checks.agreement.tolerance, 'rad');
  }
  return results;
}
