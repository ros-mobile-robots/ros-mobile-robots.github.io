# Wheel Dynamics in Simulation

A simulated robot can show effects that its kinematic model can't explain:

- Its wheels turn exactly as commanded, but the robot turns less than its odometry says.
- Its wheels jitter when it turns on the spot.
- A higher motor gain, meant to follow the commands better, makes the model explode at spawn.

All three come from parts of the simulation that the [kinematic model](modeling-differential-drive-robot.md) and [odometry](odometry.md) leave out: where the wheels touch the ground, how the physics engine computes the contact forces, and how a simulated motor turns a velocity command into a torque. This page explains these parts and the parameters that set them, for ROS 1 Noetic with Gazebo Classic 11 and its default physics engine, ODE (Open Dynamics Engine). The new Gazebo, ROS 2 and other physics engines name and implement some of these parameters differently.

After this page, you can:

- tell a wheel's visual, collision and inertial models apart, and say which one a symptom points to;
- read wheel speed tracking, the agreement of odometry and true motion, and the wheels' settling height as three different measurements;
- choose a controlled check for a contact or velocity-loop problem, including what it assumes about the time step.

The page uses DiffBot and Remo as worked examples. Their current settings, the reasons for them and a tuning procedure are on the Simulation page, in [Tuning the simulated robot](../../diffbot_gazebo.md#tuning-the-simulated-robot).

## What determines the motion

A velocity command passes through several models before the robot moves:

``` mermaid
graph TD
  CMD[cmd_vel] --> C[diff_drive_controller];
  C -- "wheel speed commands ω*" --> PID[velocity PID in DefaultRobotHWSim];
  PID -- "torque τ" --> W[wheel with inertia J];
  G[ground contact: collision shape, friction, kp, kd] -- "contact forces" --> W;
  W -- "measured wheel speeds ω" --> PID;
  W -- "joint states" --> ODOM["odometry (diff_drive_controller)"];
  W -- "motion of the robot" --> TRUE[true pose in Gazebo];
```

So there are three views of the same motion, and the [Simulation Test](../../development/simulation-test.md) compares them:

- the **command**: what `cmd_vel` asks for;
- the **odometry**: what the wheel turns add up to, with the configured wheel radius and separation;
- the **true pose**: where Gazebo's physics put the robot.

Each comparison points to a different part of the chain. Wheel speeds that miss their commands point to the motor and the contact load. Wheels that follow their commands while the true pose differs from the odometry point to geometry and contacts. And a wheel that sinks into the ground at rest points to the contact settings, whatever the motion does.

## Representing a wheel

A URDF link has three separate descriptions ([URDF link specification](https://wiki.ros.org/urdf/XML/link)):

- **Visual:** what you see in RViz and Gazebo, often a mesh. The physics engine doesn't use it.
- **Collision:** the geometry that the physics engine checks for contacts. Its shape decides where the wheel touches the ground.
- **Inertial:** the mass, the center of mass and the inertia tensor. They decide how the wheel accelerates under a torque.

They don't have to agree. DiffBot's and Remo's wheels look like their mesh, collide as a sphere and are modeled with a solid cylinder's inertia.

### Where does a wheel push on the ground?

The [kinematic model](modeling-differential-drive-robot.md) assumes that each wheel pushes on the ground in its center plane, so the turn rate is $\omega = (v_r - v_l) / L$, with the wheel separation $L$ between the wheel centers. The odometry uses the same $L$. If the wheels push at an effective separation $L_\text{eff}$ instead, and their speeds are the same, the robot turns $L / L_\text{eff}$ of what its odometry reports.

On flat ground, a sphere with the wheel's radius touches at one point in the wheel's center plane. A cylinder touches along a line across its width, which the physics engine reduces to a few contact points. Where the resulting forces act depends on the contact solver; the turn ratio shows the effective position.

*Predict before you look:* if the effective separation grows while the wheel speeds stay the same, does the robot turn more or less than its odometry says?

**Experiment E1** changes only the drive wheels' collision, cylinder or sphere, on both robots. Each robot turns 1.5 rad on the spot to the left and to the right, three times each, in a fresh simulation per run.

--8<-- "includes/wheel-dynamics-contacts.html"

In every run, the wheels follow their commands (Remo's within 1 %, DiffBot's within about 7 %), so the motor isn't what differs:

- **Spheres:** the true turn is 0.98 to 0.99 of the odometry's on DiffBot, and 1.000 on Remo.
- **Cylinders:** DiffBot turns 0.847 of its odometry's angle in all six turns, the ratio that contacts at the outer edges predict, 0.138 m / 0.163 m. Remo turns 0.852 to the left, again its outer-edge ratio, but 0.87 to the right, and once 1.21.

So the cylinders' contacts act as if they were at the outer edges, or less predictably; the experiment shows their effect, not where the contact forces act. On flat ground, a sphere puts the effective contact where the kinematic model assumes it.

A sphere has limits too: it only represents an upright wheel on flat ground. On a slope or an edge it can touch the ground beside the real tire.

### How does the mass distribution enter?

For a wheel that spins about its axle, the inertia that matters is the one about that axis. A solid cylinder of mass $m$ and radius $r$ has

$$
J = \frac{1}{2} m r^2 .
$$

A torque $\tau$ on the wheel accelerates it at $\dot{\omega} = \tau / J$, so a light wheel responds faster to the same torque. That is good for following commands, and it also makes the velocity loop easier to destabilize ([Why the time step matters](#why-the-time-step-matters)). Remo's 32 g wheels with a radius of 0.033 m have $J = 0.5 \cdot 0.032 \cdot 0.033^2 = 1.74 \cdot 10^{-5}\,\mathrm{kg\,m^2}$.

The collision shape doesn't change the inertia. A wheel with a sphere collision is still modeled with a cylinder's inertia; a solid sphere's $\frac{2}{5} m r^2$ would be 20 % less. A real wheel with a heavy tire has more inertia than a uniform disk of the same mass, so the solid cylinder is an approximation.

The inertia tensor is given in the inertial frame. A URDF cylinder's axis is its z axis, while the wheel's joint axis is its y axis, so the inertial origin has to rotate the cylinder's $\frac{1}{2} m r^2$ onto the joint axis. A wrong rotation gives the wheel the smaller inertia about a diameter, $\frac{1}{12} m (3r^2 + l^2)$, which doesn't show in RViz but changes its dynamics. *Modern Robotics*, [section 8.2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-2-dynamics-of-a-single-rigid-body-part-1-of-2/), explains the rotational inertia of a rigid body and how it changes with the frame.

## Modeling ground contact

ODE treats a contact as a constraint: the two bodies must not move into each other, and the friction force must not exceed its limit. Gazebo creates these constraints from the collision shapes in each time step, with the parameters below. The [ODE user guide](https://ode.org/ode-latest-userguide.html) and Gazebo's [Friction](https://classic.gazebosim.org/tutorials?tut=friction) and [Physics Parameters](https://classic.gazebosim.org/tutorials?tut=physics_params) tutorials describe them in full.

### How much tangential force can a contact carry?

Coulomb's friction model limits the tangential contact force $f_t$ by the normal force $f_n$:

$$
\lVert f_t \rVert \le \mu \, f_n .
$$

The allowed forces form a cone. ODE approximates the cone by a pyramid with two directions in the contact plane, with one coefficient each ([ODE user guide, section 3.11.1](https://ode.org/ode-latest-userguide.html#sec_3_11_1)):

$$
\lvert f_1 \rvert \le \mu_1 f_n , \qquad \lvert f_2 \rvert \le \mu_2 f_n .
$$

`mu1` and `mu2` are these two coefficients, not a static and a dynamic friction coefficient. For each contact, Gazebo uses the smaller coefficient of the two surfaces ([`ODEPhysics.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODEPhysics.cc#L1257-L1261)). Gazebo's ground plane has `mu` 100 and `mu2` 50, so with lower wheel coefficients, the wheel's decide.

`fdir1` sets the first friction direction, in the collision's own frame; without it, ODE picks the two directions itself. ODE requires the direction to lie in the contact plane, and it uses it as given, without projecting it ([`contact.cpp`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/deps/opende/src/joints/contact.cpp#L272-L287)). Gazebo turns it with the collision's pose ([`ODEPhysics.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODEPhysics.cc#L1127-L1133)). That makes `fdir1` easy to get wrong on a wheel:

- A URDF wheel cylinder is usually rotated so that its z axis is the axle, which doesn't move as the wheel spins. `fdir1` `0 0 1` then stays along the axle, in the contact plane.
- A sphere has no such rotation: its frame spins with the wheel, so the same `0 0 1` tilts into the contact normal. With equal coefficients, leave `fdir1` unset; otherwise choose a direction along the axle in the sphere's frame. On Remo, a sphere with `fdir1` `0 0 1` made the robot drive about 2 % less than its odometry; without `fdir1`, the two agree.

Friction limits the force; it doesn't make a wheel roll. Higher friction doesn't fix a wrong wheel speed or a wrong geometry.

### How stiff is a contact?

In each time step, ODE corrects a part of the penetration between two bodies. Two parameters control how hard a constraint is ([ODE user guide, sections 3.7 and 3.8](https://ode.org/ode-latest-userguide.html#sec_3_7_0)):

- **ERP**, the error reduction parameter, between 0 and 1: the fraction of the penetration that is corrected in the next step.
- **CFM**, the constraint force mixing: 0 makes the constraint hard; a positive value lets the bodies press into each other, more the larger it is.

A spring with stiffness $k_p$ and a damper with damping $k_d$ correspond to ([ODE user guide, section 3.8.2](https://ode.org/ode-latest-userguide.html#sec_3_8_2))

$$
\mathrm{ERP} = \frac{h k_p}{h k_p + k_d}, \qquad
\mathrm{CFM} = \frac{1}{h k_p + k_d},
$$

where $h$ is the physics time step. ODE then behaves like that spring and damper, integrated with an implicit first-order method. The `kp` and `kd` of a Gazebo surface are these spring and damping coefficients. They describe the contact, not the motor, and have nothing to do with the P and D gains of the wheel's velocity controller.

For each contact, Gazebo combines the two surfaces' settings: the stiffnesses like two springs in series, $k_p = 1 / (1/k_{p,1} + 1/k_{p,2})$, and the dampings by their sum, $k_d = k_{d,1} + k_{d,2}$. It computes ERP and CFM with the world's time step and uses them for this contact instead of the world's `<erp>` and `<cfm>` ([`ODEPhysics.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODEPhysics.cc#L1112-L1120)). Two example wheel surfaces on Gazebo's ground plane ($k_p = 10^{12}$, $k_d = 1$), at $h$ = 1 ms:

| Wheel surface | Wheel `kp`, `kd` | Combined $k_p$, $k_d$ | ERP | CFM |
|---|---|---|---|---|
| Stiff: Gazebo's defaults | $10^{12}$, 1 | $5 \cdot 10^{11}$, 2 | 0.999999996 | $2.0 \cdot 10^{-9}$ |
| Compliant | $10^{7}$, 1 | $10^{7}$, 2 | 0.9998 | $1.0 \cdot 10^{-4}$ |

These are equivalences, not material properties: `kp` and `kd` set ERP and CFM, and it doesn't help to treat the two pairs as independent settings. A real tire's stiffness is a starting point at most. Each robot's settings are in [Tuning the simulated robot](../../diffbot_gazebo.md#contacts).

### How much penetration is tolerated?

Two more parameters limit the correction:

- **`minDepth`:** a penetration allowance, in meters. ODE subtracts it from the penetration before it corrects the rest. It is a dead band, not a limit on the depth.
- **`maxVel`:** the maximum velocity at which a penetration is corrected, in m/s. It isn't a speed limit for the wheel or the robot.

For a wheel on the static ground plane, ODE uses the wheel's own `minDepth` and `maxVel`, and limits `maxVel` by the world's `contact_max_correcting_vel` ([`contact.cpp`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/deps/opende/src/joints/contact.cpp#L127-L149), [lines 222–237](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/deps/opende/src/joints/contact.cpp#L222-L237)). Inside the dead band, nothing pushes a wheel back out, and a positive CFM lets it press into the ground under its load.

*Predict before you look:* with a `minDepth` of 10 mm, does a wheel sink until it is 10 mm deep?

**Experiment E2** leaves Remo at rest for 90 s, with its sphere collisions and compliant contacts, and changes only `minDepth`:

--8<-- "includes/wheel-dynamics-sinkage.html"

With 1 mm, the wheels sink to the configured 1 mm within 3 s. With 10 mm, they creep at a steady 0.37 mm/s for about 20 s and stop at about 8.3 mm, a quarter of the wheel radius, not at 10 mm. With 0, no sinkage is measurable. The run doesn't show why the creep stops at 8.3 mm; the compliance, the load and the other contacts all play a part. DiffBot's stiff default contacts, with `minDepth` 0, don't sink either.

A positive allowance isn't always wrong: the ODE user guide suggests a small surface layer to keep contacts from being created and broken in every step ([ODE user guide, section 5.2](https://ode.org/ode-latest-userguide.html#sec_5_2_0)). For small, light wheels with compliant contacts, start with zero, and check the settling height and the wheel speeds.

### From URDF to SDF

Gazebo converts the URDF to SDF, its own format. The `<gazebo reference="...">` elements of a link set its collision's surface ([Using a URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf)). `gz sdf -p robot.urdf` shows the result:

| URDF `<gazebo>` element | SDF element in `collision/surface` | Meaning | Default |
|---|---|---|---|
| `mu1`, `mu2` | `friction/ode/mu`, `mu2` | friction coefficients in the two pyramid directions | 1, 1 |
| `fdir1` | `friction/ode/fdir1` | first friction direction, in the collision frame | none (0 0 0) |
| `kp`, `kd` | `contact/ode/kp`, `kd` | contact stiffness (N/m) and damping (N s/m) | $10^{12}$, 1 |
| `minDepth` | `contact/ode/min_depth` | penetration allowance (m) | 0 |
| `maxVel` | `contact/ode/max_vel` | maximum correction velocity (m/s) | 0.01 |

The defaults are from the SDFormat 1.7 specification that Gazebo 11 uses ([`surface.sdf`](https://github.com/gazebosim/sdformat/blob/1774642ee6ac2d46f9fb335470dd7e881d0e6cc5/sdf/1.7/surface.sdf#L52-L199)).

## Turning a wheel-speed command into torque

On the real robot, the [motor PID in the firmware](../../packages/diffbot_base/pid.md) makes the motors reach the wheel speeds that `diff_drive_controller` commands. In Gazebo, `gazebo_ros_control`'s `DefaultRobotHWSim` takes the place of the firmware. If the parameter server has gains for a wheel joint under `/<robot namespace>/gazebo_ros_control/pid_gains/<joint>`, it controls the joint with a PID that computes a torque from the speed error and applies it to the joint ([`default_robot_hw_sim.cpp`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L217-L231), [lines 370–380](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L370-L380)). Without gains, it sets the joint's velocity directly, a different behavior. DiffBot and Remo load gains.

The PID is control_toolbox's `Pid` ([`pid.cpp`](https://github.com/ros-controls/control_toolbox/blob/14b2a6120de57b038a05562839ee1ba4dabee2aa/src/pid.cpp#L336-L369)). With the speed error $e = \omega^* - \omega$ in rad/s, it computes the torque

$$
\tau = P e + I \int e \, dt + D \dot{e} ,
$$

in N m, so P is in N m s/rad and I in N m/rad. Its parameters:

- **`p`, `i`, `d`:** the gains. A P gain alone needs a speed error to produce a torque, so a wheel that needs torque, for example to turn the robot on the spot against friction, stays slower than commanded. The I term grows until it supplies that torque, and the error goes to zero, as long as that torque is within `i_clamp` and the loop is stable.
- **`i_clamp`:** limits the I term's torque to ±`i_clamp` (N m), not the total torque. It defaults to 0, so an I gain without `i_clamp` does nothing.
- **`antiwindup`:** with `true`, the integral of the error stops growing once the I term reaches `i_clamp`. With `false`, the I term is still limited, but the integral keeps growing and takes longer to come back when the error changes sign.

The [PID Controllers](../../packages/diffbot_base/pid.md) page and [*Feedback Systems*](https://fbsbook.org), chapter 11, explain PID control, integral windup and tuning in general.

`DefaultRobotHWSim` limits the torque to the joint's effort limit, from the URDF's `<limit effort="...">` or the `joint_limits` parameters ([lines 404–454](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L404-L454)). Without either, as on DiffBot and Remo, a simulated motor has no torque limit, unlike a real one.

### Which contact setting changes wheel tracking?

The wheel's velocity loop works against the contact: the ground decides how much torque a wheel needs and how its speed responds. Remo's reference contacts are compliant and damped, with friction 1, on a sphere collision with `minDepth` 0. **Experiment E3** compares them with runs that change one setting to a much harder value each, and with a run that changes all three. Remo turns on the spot to the left, twice per setting.

*Predict before you look:* which of the three settings would you expect to matter?

--8<-- "includes/wheel-dynamics-wheel-speeds.html"

Friction 200 and damping 0 don't change anything measurable. The stiffness alone does: with $k_p = 10^{12}$, the wheels miss their commands, one at 122 % and the other at 71 %, and the robot turns only 0.76 of what its odometry says. All three changes together behave like the stiffness alone. So in this configuration, the contact stiffness decides; the experiment doesn't show why. DiffBot runs with the same stiff default contacts and its wheels follow their commands, but they are modeled with 0.5 kg instead of 32 g, so the effect depends on more than the stiffness. The joint states arrive at 50 Hz, so oscillations faster than that don't show here.

## Why the time step matters

Gazebo updates the wheel torque once per physics step $h$. Take a wheel that doesn't touch the ground, as at spawn, and only a P gain. Within one step, the torque is constant:

$$
e_k = \omega^* - \omega_k , \qquad \tau_k = P e_k , \qquad
\omega_{k+1} = \omega_k + \frac{h}{J} \tau_k .
$$

With a constant command $\omega^*$, the error in the next step is

$$
e_{k+1} = \left( 1 - \frac{P h}{J} \right) e_k .
$$

The error decays only if this factor's magnitude is below 1:

$$
\left| 1 - \frac{P h}{J} \right| < 1
\quad \Longleftrightarrow \quad
0 < \frac{P h}{J} < 2 .
$$

At $P h / J = 2$, the error changes sign in every step without decaying; above it, it grows in every step. A continuous-time P loop on an inertia has no such upper limit. The limit comes from holding the torque over a step, which is typical of sampled systems; Franklin, Powell and Workman's *Digital Control of Dynamic Systems* treats them in depth.

--8<-- "includes/wheel-dynamics-stability.html"

*Before you read on:* what happens to $P h / J$ when the wheel mass is halved?

For Remo, $J = 1.74 \cdot 10^{-5}\,\mathrm{kg\,m^2}$ and $h$ = 1 ms give $P < 2J/h = 0.035$ N m s/rad. With P 0.03, the factor is $1 - \frac{0.03 \cdot 0.001}{1.74 \cdot 10^{-5}} \approx -0.72$: the error changes sign and decays. With P 0.06, it is $-2.44$, and Remo's model spawned at the origin with NaN velocities. Halving the mass halves $J$, doubles $P h / J$, and halves the gain that the loop tolerates. So a gain has to be checked again whenever the wheel mass changes: DiffBot's wheels are modeled with a placeholder of 0.5 kg ($J = 2.8 \cdot 10^{-4}\,\mathrm{kg\,m^2}$, limit about 0.55), while its real 32 g wheels would allow only about 0.035, below its P 0.06.

The estimate assumes a lot: one free wheel with a constant inertia, only a P gain, fresh speed feedback in every step, and no torque limit, friction in the bearings, delay or contact forces. The actual loop has an I term, and a wheel on the ground also moves the robot. So a gain below the limit doesn't prove that the simulation is stable, and a gain above it doesn't mean that Gazebo must explode. It tells the scale at which the gain becomes a problem.

The time step is the physics step of the world, `max_step_size`, not its real-time update rate, nor the rate of `/cmd_vel` or of the joint states. DiffBot's simulation test uses `diffbot_empty.world`, with 1 ms. Other worlds can differ: `playground.world` has 10 ms, which lowers Remo's limit tenfold, to about 0.0035, below its P 0.03. Check the physics of the world you use before you carry gains over. `gazebo_ros_control` writes the torque in every physics step, but reads the joint states and updates the controllers only every `controlPeriod`, which defaults to the physics step ([`gazebo_ros_control_plugin.cpp`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L224-L256)). A longer `controlPeriod` adds a delay that this estimate doesn't include.

## Choosing the next check

A symptom rarely has one cause, but it suggests which measurement to take first. Change one setting at a time, as in [Tuning the simulated robot](../../diffbot_gazebo.md#how-to-tune):

| Symptom | First checks |
|---|---|
| The wheels follow their commands, but the true turn differs from the odometry's | The configured wheel separation and radius; the effective contact separation (collision shape) |
| The wheels jitter or miss their commands while turning | The collision shape and the contact stiffness; the velocity loop's $P h / J$ |
| The wheels stay slower than commanded | The I term and `i_clamp`; the torque the contacts demand; a torque limit |
| The model explodes or doesn't move at spawn | $P h / J$ with the wheel's inertia and the world's step; the inertia's axis |
| The robot sinks or tilts at rest | `minDepth` and the contact stiffness; the masses |

## Further reading

To go further on one topic, start with the official reference next to its explanation above, then one chapter of a book:

- **ODE user guide** ([ode.org](https://ode.org/ode-latest-userguide.html)) and Gazebo Classic's tutorials [Friction](https://classic.gazebosim.org/tutorials?tut=friction), [Physics Parameters](https://classic.gazebosim.org/tutorials?tut=physics_params), [Using a URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf) and [ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control): the reference for the contact model and the parameters that Gazebo Classic actually uses.
- Karl Johan Åström and Richard M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd edition, Princeton University Press 2021 ([free online at fbsbook.org](https://fbsbook.org); [Amazon.com][amazon_book_feedback_systems_us], [Amazon.de][amazon_book_feedback_systems_ger], affiliate links): chapter 11, PID control, with integral windup and tuning.
- Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press 2017 ([free preprint and videos at modernrobotics.org](http://modernrobotics.org); [Amazon.com][amazon_book_modern_robotics_us], [Amazon.de][amazon_book_modern_robotics_ger], affiliate links): section 8.2 for the inertia of a rigid body, chapter 13 for wheeled mobile robots and their rolling constraints.

For more depth:

- Gene F. Franklin, J. David Powell and Michael Workman, *Digital Control of Dynamic Systems*, 3rd edition ([e-edition at SC Solutions](https://scsolutions.com/publication/digital-control-of-dynamic-systems-third-edition/)): sampled systems and the choice of the sample rate (chapters 3 to 5 and 11), the background of the stability estimate above.
- David Baraff, [*Rigid Body Simulation II: Nonpenetration Constraints*](https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf), SIGGRAPH course notes: how a simulator computes contact and friction forces as constraints. A concept, not ODE's exact solver.
- Roland Siegwart, Illah R. Nourbakhsh and Davide Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd edition, MIT Press 2011: wheel types and their rolling and sliding constraints, an alternative to *Modern Robotics*' chapter 13.
- Roy Featherstone, *Rigid Body Dynamics Algorithms*, Springer 2008: the algorithms of physics engines, with a chapter on contact and impact. Advanced.
