// The kinematic model of a differential drive robot that the interactive figures share.
// Pure functions without any page access, so a test can call them directly.
// scripts/figures/motion_lab_figures.py has the same model in Python, for the static version and
// the examples' answers. The browser tests compare the page's numbers with the script's.
//
// Symbols: v (m/s) and w (rad/s) are the robot's linear and angular velocity, r the wheel radius
// (m), L the wheel separation (m, between the wheels' center planes). The wheels' angular
// velocities are in rad/s.

// Below this angular velocity (rad/s), a drive counts as straight: the radius of its circle, v / w,
// would overflow. motion_lab_figures.py has the same limit
const STRAIGHT_ANGULAR_VELOCITY = 1e-12;

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
  if (Math.abs(w) < STRAIGHT_ANGULAR_VELOCITY) return { x: v * t, y: 0, theta: 0 };
  const theta = w * t, circleRadius = v / w;
  return { x: circleRadius * Math.sin(theta), y: circleRadius * (1 - Math.cos(theta)), theta };
}

/**
 * The end pose relative to the start pose, in the robot's frame at the start: forward and left
 * (m) and the angle turned (rad, wrapped), as relative_pose() in diffbot's test_drive.py.
 */
export function relativePose(start, end) {
  const dx = end.x - start.x, dy = end.y - start.y;
  const cos = Math.cos(start.theta), sin = Math.sin(start.theta);
  return { forward: cos * dx + sin * dy, left: -sin * dx + cos * dy, turned: wrap(end.theta - start.theta) };
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
 * Returns the wheel speeds, the three velocities and posesAt(t), the poses at a time.
 */
export function threeViews({ v, w, radius, separation, configuredRadius, configuredSeparation,
  contactSeparation = separation, frameOffset = 0, tracking = 1 }) {
  const commanded = wheelSpeedsForCommand(v, w, configuredRadius, configuredSeparation);
  const left = tracking * commanded.left, right = tracking * commanded.right;
  const odometry = velocityFromWheelSpeeds(left, right, configuredRadius, configuredSeparation);
  const model = velocityFromWheelSpeeds(left, right, radius, contactSeparation);
  const posesAt = (t) => {
    const axle = poseAfter(model.v, model.w, t);
    axle.x += frameOffset;  // The reported frame starts at 0, 0, so the axle starts ahead of it
    const frame = {
      x: axle.x - frameOffset * Math.cos(axle.theta),
      y: axle.y - frameOffset * Math.sin(axle.theta),
      theta: axle.theta,
    };
    return { t, command: poseAfter(v, w, t), odometry: poseAfter(odometry.v, odometry.w, t), axle, frame };
  };
  return { wheelSpeeds: { left, right }, velocities: { command: { v, w }, odometry, model }, posesAt };
}

// A value on a tolerance's boundary can miss it by floating-point rounding: 1.5 - 1.2 gives
// 0.30000000000000004. This small allowance keeps such a value inside its range in the figures,
// as in motion_lab_figures.py. It is a choice of the figures only. The ROS test compares without it.
const ROUNDING_ALLOWANCE = 1e-9;

/** Whether a value lies within expected +- tolerance, as assertAlmostEqual(..., delta=tolerance). */
export function within(value, expected, tolerance) {
  return Math.abs(value - expected) <= tolerance + ROUNDING_ALLOWANCE;
}

/** The value of a quantity ('forward', 'left', 'turned' or 'distance' from the start) in a motion. */
export function measured(quantity, motion) {
  return quantity === 'distance' ? Math.hypot(motion.forward, motion.left) : motion[quantity];
}

/**
 * The checks of test_drive.py for one drive, predicted from the odometry's and the model's
 * relative motion. `rows` are the test's checks in its order, as the include's data has them:
 * each with the view it measures ('odometry', 'model', or 'agreement' for how far apart the two
 * are), the quantity, the expected value and the tolerance. Each result is its row with the
 * measured value and whether it passes.
 */
export function predictChecks(rows, odometry, model) {
  const motions = { odometry, model };
  return rows.map((row) => {
    const value = row.view === 'agreement'
      ? Math.abs(measured(row.quantity, odometry) - measured(row.quantity, model))
      : measured(row.quantity, motions[row.view]);
    return { ...row, value, passes: within(value, row.expected, row.tolerance) };
  });
}
