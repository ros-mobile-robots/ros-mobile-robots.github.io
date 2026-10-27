# Wheel Dynamics in Simulation

A simulated robot can show effects that its kinematic model can't explain:

- Its wheels turn exactly as commanded, but the robot turns less than its odometry says.
- Its wheels jitter when it turns on the spot.
- A higher motor gain, meant to follow the commands better, makes the model explode at spawn.

All three come from parts of the simulation that the [kinematic model](modeling-differential-drive-robot.md) and [odometry](odometry.md) leave out: where the wheels touch the ground, how the physics engine computes the contact forces, and how a simulated motor turns a velocity command into a torque. This page explains these parts and the parameters that set them, for ROS 1 Noetic with Gazebo Classic 11 and its default physics engine, ODE (Open Dynamics Engine). The [new Gazebo](../../simulators.md#the-new-gazebo-for-ros-2), ROS 2 and other physics engines name and implement some of these parameters differently.

After this page, you can:

- tell apart a wheel's three descriptions in the URDF: how it looks, where it collides and how its mass is distributed;
- use three measurements, the wheel speeds, the odometry compared with Gazebo's motion, and the wheels' height at rest, to narrow down a simulation problem;
- change one setting at a time, and estimate how a wheel's inertia and the physics time step affect the stability of a simple motor loop.

The page uses DiffBot and Remo as worked examples. Their current settings, the reasons for them and a tuning procedure are on the Simulation page, in [Tuning the simulated robot](../../diffbot_gazebo.md#tuning-the-simulated-robot).

??? info "About the experiments E1 to E3"
    The measurements on this page come from three controlled experiments in Gazebo Classic, run for this page in the [dev container](../../development/dev-container.md) with the [diffbot repository](https://github.com/ros-mobile-robots/diffbot/tree/b896a3a1a233e7131835895ad343832d035b18f2) at `b896a3a` and [remo_description](https://github.com/ros-mobile-robots/remo_description/tree/ba0b2c567e41adcd4ef0e5f652e6d49c919c9462) at `ba0b2c5`:

    - **E1** compares the drive wheels' collision shapes, cylinder and sphere, on both robots: three runs per shape.
    - **E2** measures how far Remo's wheels sink into the ground at rest, for three `minDepth` values: two runs per value.
    - **E3** changes one of the contact settings of Remo's wheels at a time, and all of them together: two runs per setting.

    Each run starts a fresh simulation. In E1 and E3, the robot then turns left, turns right and drives forward, one after the other, each for 3 s. The measured data and the script that draws the figures are in this repository, in [`scripts/figures`]({{ docs_repo_url }}/scripts/figures); the script that ran the simulations isn't published yet. The results describe these model revisions and settings, not every robot or later revision.

## What determines the motion

To find a simulation problem, first follow a velocity command through the model. The diagram is a simplified chain for this Gazebo Classic setup: it shows where a wheel-speed error, a disagreement with the odometry or a ground-contact problem can enter. It follows the code of these packages, and the data flow that Gazebo's [ROS Control tutorial](https://classic.gazebosim.org/tutorials?tut=ros_control) and the [ros_control paper](https://joss.theoj.org/papers/10.21105/joss.00456) (Chitta et al., 2017) describe:

``` mermaid
graph TD
  CMD["1 Command: cmd_vel"] --> C[diff_drive_controller];
  C -- "wheel speed commands ω*" --> PID[velocity PID in DefaultRobotHWSim];
  PID -- "torque τ" --> W["robot mechanics: wheels with inertia J, body, caster"];
  G[ground contact: collision shape, friction, kp, kd] -- "contact forces" --> W;
  W -- "measured wheel speeds ω" --> PID;
  W -- "wheel positions, via the hardware interface" --> ODOM["2 Odometry (diff_drive_controller)"];
  W -- "motion of the robot" --> TRUE["3 True pose in Gazebo"];
```

The numbers mark three views of the same motion, which the [Simulation Test](../../development/simulation-test.md#three-views-of-the-motion) compares:

1. the **command**: what `cmd_vel` asks for;
2. the **odometry**: what the wheel turns add up to, with the configured wheel radius and separation;
3. the **true pose**: where Gazebo's physics put the robot.

Comparing them narrows down where to look; it doesn't prove a single cause. Wheel speeds that miss their commands point first to the motor loop and the contact load. Wheels that follow their commands while the true pose differs from the odometry point first to the geometry and the contacts. And a wheel that sinks into the ground at rest points to the contact settings, whatever the motion does.

The next sections follow the chain: [Representing a wheel](#representing-a-wheel) and [Modeling ground contact](#modeling-ground-contact) explain the wheel and the contact, [Turning a wheel-speed command into torque](#turning-a-wheel-speed-command-into-torque) the PID, and [Why the time step matters](#why-the-time-step-matters) how often they act on each other. The robot body's inertia, the caster's contacts and the solver's settings affect the motion too, but the diagram doesn't show them separately. It describes the simulated motor, not the electrical dynamics of the real one.

## Representing a wheel

A URDF link has three separate descriptions ([URDF link specification](https://wiki.ros.org/urdf/XML/link)):

- **Visual:** what you see in RViz and Gazebo, often a mesh. The physics engine doesn't use it.
- **Collision:** the geometry that the physics engine checks for contacts. Its shape decides where the wheel touches the ground.
- **Inertial:** the mass, the center of mass and the inertia tensor. They decide how the wheel accelerates under a torque.

They don't have to agree. DiffBot's and Remo's wheels look like their mesh, collide as a sphere and are modeled with a solid cylinder's inertia.

### Where does a wheel push on the ground?

The [kinematic model](modeling-differential-drive-robot.md) assumes that each wheel pushes on the ground in its center plane. With the wheel separation $b$ between the wheel centers, the robot then turns at $\omega = (v_r - v_l) / b$, and the odometry uses the same $b$. (The kinematic model and [odometry](odometry.md) pages write this distance as $2L$.) If the wheels push at an effective separation $b_\text{eff}$ instead, the same wheel speeds turn the robot $b / b_\text{eff}$ of what its odometry reports: less, if $b_\text{eff}$ is larger than $b$.

On flat ground, a sphere with the wheel's radius touches at one point in the wheel's center plane. A cylinder touches along a line across its width, which the physics engine reduces to a few contact points. Where the resulting forces act depends on the contact solver, and the experiment below measures only their effect on the turn. One explanation to test is that a cylinder pushes at its outer edges. For DiffBot, with 0.025 m wide wheels, the effective separation would then be $b_\text{eff} = 0.138 + 0.025 = 0.163$ m, and the robot would turn $0.138 / 0.163 = 0.847$ of what its odometry reports, about 15 % less. The front-view sketch below shows this outer-edge hypothesis next to the sphere's single contact.

**Experiment E1** changes only the drive wheels' collision, cylinder or sphere, on both robots. Each robot turns 1.5 rad on the spot (0.5 rad/s for 3 s), to the left and then to the right, in three runs per shape. The E1 turn-ratio plot below the sketch shows each turn as the angle the robot really turned in Gazebo divided by the angle its odometry reports, both as magnitudes: at 1 they agree, below 1 the robot turned less than its odometry says, above 1 more.

--8<-- "includes/wheel-dynamics-contacts.html"

In every turn, the wheels' mean speeds over the last 2 s of the command are close to their commands (Remo's within 1.2 %, DiffBot's within about 7 %), so the motor isn't what differs. The E1 turn-ratio plot then shows:

- **Spheres:** the true turn is 0.98 to 0.99 of the odometry's on DiffBot, and 1.000 on Remo.
- **Cylinders:** DiffBot turns about 0.847 of its odometry's angle in all six turns, the outer-edge ratio. Remo turns 0.852 to the left, again its outer-edge ratio, $0.144 / 0.169$, but 0.87 to the right in these runs, and once 1.21, the point at the plot's right edge.

So the cylinders behave as if they pushed at their outer edges, or less predictably; the experiment shows this effect, not where the contact forces act. On flat ground, a sphere puts the effective contact where the kinematic model assumes it.

A sphere has limits too. It doesn't reproduce the tire's width, deformation or contact patch, and it only represents an upright wheel on flat ground: on a slope or an edge, it can touch the ground beside the real tire. Other tasks may need another shape.

### How does the mass distribution enter?

How fast a wheel spins up depends on its moment of inertia $J$ about the axle: its resistance to angular acceleration, which grows with its mass and with how far that mass is from the axle. A solid cylinder of mass $m$ and radius $r$ has

$$
J = \frac{1}{2} m r^2 .
$$

The net torque $\tau$ about the axle, the motor's torque together with the torque of the contact forces, accelerates the wheel at $\dot{\omega} = \tau / J$. This is Newton's second law for rotation; OpenStax's free *University Physics*, volume 1, explains [torque](https://openstax.org/books/university-physics-volume-1/pages/10-6-torque), the [moment of inertia](https://openstax.org/books/university-physics-volume-1/pages/10-4-moment-of-inertia-and-rotational-kinetic-energy) and [the second law for rotation](https://openstax.org/books/university-physics-volume-1/pages/10-7-newtons-second-law-for-rotation). So a light wheel responds faster to the same torque. That is good for following commands, but it also means that the same motor correction changes the wheel's speed more within one physics step; [Why the time step matters](#why-the-time-step-matters) estimates when a simple P controller starts to overshoot. Remo's 32 g wheels with a radius of 0.033 m have $J = 0.5 \cdot 0.032 \cdot 0.033^2 = 1.74 \cdot 10^{-5}\,\mathrm{kg\,m^2}$.

The collision shape doesn't change the inertia. A wheel with a sphere collision is still modeled with a cylinder's inertia; a solid sphere's $\frac{2}{5} m r^2$ would be 20 % less. A real wheel with a heavy tire has more inertia than a uniform disk of the same mass, so the solid cylinder is an approximation.

Check that the model's $\frac{1}{2} m r^2$ is about the wheel's joint axis. A wrong rotation of the inertia doesn't show in RViz, but changes how the wheel spins up.

??? info "Why the inertia has to be rotated onto the joint axis"
    The inertia tensor is given in the inertial frame. A URDF cylinder's axis is its z axis, while the wheel's joint axis is its y axis, so the inertial origin has to rotate the cylinder's $\frac{1}{2} m r^2$ onto the joint axis. A wrong rotation gives the wheel the smaller inertia about a diameter, $\frac{1}{12} m (3r^2 + l^2)$, with the wheel width $l$. *Modern Robotics*, [section 8.2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-2-dynamics-of-a-single-rigid-body-part-1-of-2/), explains the rotational inertia of a rigid body and how it changes with the frame.

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

`fdir1` sets the first friction direction, in the collision's own frame; without it, ODE picks the two directions itself. **For a sphere wheel with equal coefficients, as on DiffBot and Remo, leave `fdir1` unset.** The direction is easy to get wrong on a wheel: ODE requires it to lie in the contact plane and uses it as given, without projecting it ([`contact.cpp`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/deps/opende/src/joints/contact.cpp#L272-L287)), and Gazebo turns it with the collision's pose ([`ODEPhysics.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODEPhysics.cc#L1127-L1133)):

- A URDF wheel cylinder is usually rotated so that its z axis is the axle, which doesn't move as the wheel spins. `fdir1` `0 0 1` then stays along the axle, in the contact plane.
- A sphere has no such rotation: its frame spins with the wheel, so the same `0 0 1` tilts into the contact normal. On Remo, a sphere with `fdir1` `0 0 1` made the robot drive about 2 % less than its odometry; without `fdir1`, the two agree.

A direction along the axle in the sphere's frame would stay in the contact plane. But Gazebo Classic applies only the `fdir1` of the first of the two collisions in a contact and ignores the second's ([`ODEPhysics.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODEPhysics.cc#L1127-L1166)), so whether a wheel's `fdir1` takes effect depends on whether Gazebo treats the wheel as the first or the second collision.

Friction limits the force; it doesn't make a wheel roll. Higher friction doesn't fix a wrong wheel speed or a wrong geometry.

### How stiff is a contact?

A contact in ODE isn't perfectly rigid: two bodies can press into each other a little, and the solver pushes them apart again. In each time step, ODE corrects a part of the penetration between the two bodies. Two parameters control how hard such a constraint is ([ODE user guide, sections 3.7 and 3.8](https://ode.org/ode-latest-userguide.html#sec_3_7_0)):

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

Both rows have an ERP close to 1: both correct almost all of a penetration in the next step. They differ in the CFM, by a factor of about 50 000. The CFM sets how soft the constraint is: the compliant contact acts like a spring of $10^7$ N/m, the stiff one like $5 \cdot 10^{11}$ N/m.

These are equivalences, not material properties: `kp` and `kd` set ERP and CFM, and it doesn't help to treat the two pairs as independent settings. A real tire's stiffness is a starting point at most. Each robot's settings are in [Tuning the simulated robot](../../diffbot_gazebo.md#contacts).

### How much penetration is tolerated?

Two more parameters limit the correction:

- **`minDepth`:** a penetration allowance, in meters. ODE subtracts it from the penetration before it corrects the rest. It is a dead band, not a limit on the depth.
- **`maxVel`:** the maximum velocity at which a penetration is corrected, in m/s. It isn't a speed limit for the wheel or the robot.

For a wheel on the static ground plane, ODE uses the wheel's own `minDepth` and `maxVel`, and limits `maxVel` by the world's `contact_max_correcting_vel` ([`contact.cpp`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/deps/opende/src/joints/contact.cpp#L127-L149), [lines 222–237](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/deps/opende/src/joints/contact.cpp#L222-L237)). Inside the dead band, no position correction pushes a wheel back out, and a positive CFM lets it press into the ground under its load. Gazebo copies each wheel collision's `minDepth` to its ODE body ([`ODELink.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODELink.cc#L350-L379)), where it takes precedence over the world's `contact_surface_layer`: the test world sets that layer to 0.01 m ([`diffbot_empty.world`](https://github.com/ros-mobile-robots/diffbot/blob/b896a3a1a233e7131835895ad343832d035b18f2/diffbot_gazebo/worlds/diffbot_empty.world#L29)), but Remo's wheels, with `minDepth` 0, have no dead band.

*Predict before you look:* with a `minDepth` of 10 mm, does a wheel sink until it is 10 mm deep?

**Experiment E2** leaves Remo at rest for 90 s, with its sphere collisions and compliant contacts, and changes only `minDepth`. The E2 settling plot shows how deep the wheels are in the ground over time: the wheel radius minus the height of the wheel's center above the ground.

--8<-- "includes/wheel-dynamics-sinkage.html"

In the E2 settling plot, with 1 mm, the wheels sink to the configured 1 mm within 3 s. With 10 mm, they creep at a steady 0.37 mm/s for about 20 s and stop at about 8.3 mm, a quarter of the wheel radius, not at 10 mm. With 0, no sinkage is measurable. The run doesn't show why the creep stops at 8.3 mm; the compliance, the load and the other contacts all play a part. DiffBot's stiff default contacts, with `minDepth` 0, don't sink either.

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

For example, this is the collision of Remo's front left wheel in the output of `gz sdf -p`, from [remo_description 2.0.0](https://github.com/ros-mobile-robots/remo_description/tree/2.0.0) with its default arguments and Gazebo 11.15.1. The sphere is the collision shape, `contact` holds the stiffness, damping and penetration settings, and `friction` the two coefficients. `fdir1` doesn't appear, because Remo leaves it unset:

```xml
<collision name='front_left_wheel_collision'>
  <pose>0 0 0 0 -0 0</pose>
  <geometry>
    <sphere>
      <radius>0.033</radius>
    </sphere>
  </geometry>
  <surface>
    <contact>
      <ode>
        <kp>1e+07</kp>
        <kd>1</kd>
        <max_vel>0.01</max_vel>
        <min_depth>0</min_depth>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>1</mu>
        <mu2>1</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

To see your own model's output, expand its xacro file to a URDF and convert it, as in [Troubleshooting](../../diffbot_gazebo.md#troubleshooting) on the Simulation page.

## Turning a wheel-speed command into torque

On the real robot, the [motor PID in the firmware](../../packages/diffbot_base/pid.md) makes the motors reach the wheel speeds that `diff_drive_controller` commands. In Gazebo, `gazebo_ros_control`'s `DefaultRobotHWSim` takes the place of the firmware. If the parameter server has gains for a wheel joint under `/<robot namespace>/gazebo_ros_control/pid_gains/<joint>`, it controls the joint with a PID that computes a torque from the speed error and applies it to the joint ([`default_robot_hw_sim.cpp`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L217-L231), [lines 370–380](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L370-L380)). Without PID gains, Gazebo Classic 11 with ODE sends the wheel-speed target to ODE's joint motor instead, with the joint's effort limit as its maximum torque ([lines 355–380](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L355-L380), [`ODEJoint.cc`](https://github.com/gazebosim/gazebo-classic/blob/b22c6e15e52299865b31093b8feebc9ca19e26e8/gazebo/physics/ode/ODEJoint.cc#L440-L454)). That bypasses the torque PID described here; with a finite limit, the wheel can still fall short of the requested speed under load. DiffBot and Remo load PID gains, so the examples on this page use the torque-controlled path.

The PID is control_toolbox's `Pid` ([`pid.cpp`](https://github.com/ros-controls/control_toolbox/blob/14b2a6120de57b038a05562839ee1ba4dabee2aa/src/pid.cpp#L336-L369)). In every update, it goes from the speed error to a torque, and `DefaultRobotHWSim` then limits that torque:

1. **Speed error:** $e = \omega^* - \omega$, in rad/s.
2. **Torque:** $\tau = P e + I \int e \, dt + D \dot{e}$, in N m, so P is in N m s/rad and I in N m/rad. The I term alone is limited to ±`i_clamp`.
3. **Torque limit:** the total torque is limited to the joint's effort limit, if it has one (see below).

Its parameters:

- **`p`, `i`, `d`:** the gains. The `d` here is the motor loop's derivative gain; it has nothing to do with a contact's damping `kd`. A P gain alone needs a speed error to produce a torque, so a wheel that needs torque, for example to turn the robot on the spot against friction, stays slower than commanded. The I term grows until it supplies that torque, and the error goes to zero, as long as that torque is within `i_clamp` and the loop is stable.
- **`i_clamp`:** limits the I term's torque to ±`i_clamp` (N m), not the total torque. It defaults to 0, so an I gain without `i_clamp` does nothing.
- **`antiwindup`:** with `true`, the integral of the error stops growing once the I term reaches `i_clamp`. With `false`, the I term is still limited, but the integral keeps growing and takes longer to come back when the error changes sign.

*Example:* Remo's gains are P 0.03, I 0.6 and `i_clamp` 0.05 N m. A lasting speed error of 1 rad/s gives 0.03 N m from the P term at once, and the I term grows by 0.6 N m per second, until it reaches 0.05 N m after about 0.08 s. Remo has no effort limit, so nothing limits their sum.

The [PID Controllers](../../packages/diffbot_base/pid.md) page and [*Feedback Systems*](https://fbsbook.org), chapter 11, explain PID control, integral windup and tuning in general.

`DefaultRobotHWSim` limits the torque to the joint's effort limit, from the URDF's `<limit effort="...">` or the `joint_limits` parameters ([lines 404–454](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/default_robot_hw_sim.cpp#L404-L454)). Without either, as on DiffBot and Remo, a simulated motor has no torque limit, unlike a real one.

### Which contact setting changes wheel tracking?

The wheel's velocity loop works against the contact: the ground decides how much torque a wheel needs and how its speed responds. **Experiment E3** asks which of the wheels' contact settings makes Remo's wheels miss their speed commands. It starts from Remo's reference contacts, a sphere collision with friction 1, `kp` $10^7$, `kd` 1 and `minDepth` 0, and changes one wheel setting to a much harder value per run: friction 200, `kp` $10^{12}$ or `kd` 0, and in one more run all three. Gazebo combines each with the ground's surface, as explained above: wheel friction 200 acts as the ground's 100 and 50, and wheel damping 0 still gives a combined damping of 1. Remo turns on the spot to the left and to the right, in two runs per setting. The E3 wheel-speed plot shows the left turns; the right turns agree with them.

--8<-- "includes/wheel-dynamics-wheel-speeds.html"

In these runs, a stiffer wheel contact was the only single change that clearly worsened the tracking. With `kp` $10^{12}$, one wheel reached about 120 % of its command on average and the other about 70 %, and the robot turned only 0.76 of what its odometry says. Wheel friction 200 and wheel damping 0 changed nothing measurable, and all three changes together behave like the stiffness alone. The experiment doesn't show why the stiffness matters. DiffBot runs with the same stiff default contacts and its wheels follow their commands, but they are modeled with 0.5 kg instead of 32 g, so the effect depends on more than the stiffness. The joint states come at 50 Hz: an oscillation faster than 25 Hz shows up in these samples as a slower one or not at all, so a smooth trace doesn't rule it out.

## Why the time step matters

A motor correction acts for a whole physics step: `gazebo_ros_control` writes the wheel torque once per physics step $h$, and the torque stays constant until the next one ([`gazebo_ros_control_plugin.cpp`](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/cd9cc952402375be5f274f39b29399c4087aa839/gazebo_ros_control/src/gazebo_ros_control_plugin.cpp#L224-L256)). If one step's correction is too large, the wheel overshoots its commanded speed, and repeated overshoots can grow. For a free wheel with only a P gain, fresh speed feedback in every step and no torque limit, the error decays only if $0 < P < 2J/h$. This gives a first estimate of the gain scale, to check again whenever you change a wheel's inertia or the physics step. It isn't a stability guarantee for the whole robot, but it helps to understand the model that exploded at spawn at the top of this page.

A free wheel, one that doesn't touch the ground, is the simplest case: only its inertia counts. It is close to a wheel at spawn, before the robot settles on the ground. For example, with $P h / J = 1.5$ and a speed error of 1 rad/s, one step's torque changes the wheel's speed by 1.5 rad/s: the wheel overshoots, and the error becomes −0.5 rad/s. With $P h / J = 2.5$, the error becomes −1.5 rad/s, larger than before. In general, with only a P gain, the torque is constant within one step:

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

For Remo, $J = 1.74 \cdot 10^{-5}\,\mathrm{kg\,m^2}$ and $h$ = 1 ms give $P < 2J/h = 0.035$ N m s/rad. With P 0.03, the factor is $1 - \frac{0.03 \cdot 0.001}{1.74 \cdot 10^{-5}} \approx -0.72$: the error changes sign and decays. With P 0.06, it is $-2.44$, and in our test, Remo's model spawned at the origin with NaN velocities, which fits this estimate. Halving the mass halves $J$, doubles $P h / J$, and halves the gain that the loop tolerates. So a gain has to be checked again whenever the wheel mass changes: DiffBot's wheels are modeled with a placeholder of 0.5 kg ($J = 2.8 \cdot 10^{-4}\,\mathrm{kg\,m^2}$, limit about 0.55), while its real 32 g wheels would allow only about 0.035, below its P 0.06.

The estimate assumes a lot: one free wheel with a constant inertia, only a P gain, fresh speed feedback in every step, and no torque limit, friction in the bearings, delay or contact forces. The actual loop has an I term, and a wheel on the ground also moves the robot. So a gain below the limit doesn't prove that the simulation is stable, and a gain above it doesn't mean that Gazebo must explode. It tells the scale at which the gain becomes a problem.

The time step is the physics step of the world, `max_step_size`, not its real-time update rate, nor the rate of `/cmd_vel` or of the joint states. DiffBot's simulation test uses `diffbot_empty.world`, with 1 ms. Other worlds can differ: `playground.world` has 10 ms, which lowers Remo's limit tenfold, to about 0.0035, below its P 0.03. Check the physics of the world you use before you carry gains over. `gazebo_ros_control` writes the torque in every physics step, but reads the joint states and updates the controllers only every `controlPeriod`, which defaults to the physics step (same source as above). A longer `controlPeriod` adds a delay that this estimate doesn't include.

## Troubleshooting: which check first?

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
- OpenStax, *University Physics*, volume 1, [chapter 10](https://openstax.org/books/university-physics-volume-1/pages/10-introduction) (free online): torque, the moment of inertia and Newton's second law for rotation, the basics behind [How does the mass distribution enter?](#how-does-the-mass-distribution-enter)
- Karl Johan Åström and Richard M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd edition, Princeton University Press 2021 ([book website](https://fbsbook.org); [Amazon.com][amazon_book_feedback_systems_us], [Amazon.de][amazon_book_feedback_systems_ger], affiliate links): chapter 11, PID control, with integral windup and tuning.
- Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press 2017 ([free preprint and videos at modernrobotics.org](http://modernrobotics.org); [Amazon.com][amazon_book_modern_robotics_us], [Amazon.de][amazon_book_modern_robotics_ger], affiliate links): section 8.2 for the inertia of a rigid body, chapter 13 for wheeled mobile robots and their rolling constraints.

For more depth:

- Gene F. Franklin, J. David Powell and Michael Workman, *Digital Control of Dynamic Systems*, 3rd edition ([e-edition at SC Solutions](https://scsolutions.com/publication/digital-control-of-dynamic-systems-third-edition/)): sampled systems and the choice of the sample rate (chapters 3 to 5 and 11), the background of the stability estimate above.
- ETH Zurich, Robotic Systems Lab, [*Robot Dynamics* lecture notes](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2017/RD_HS2017script.pdf) (2017, free PDF): the dynamics of rigid bodies and robots with the Newton–Euler equations, and contact forces, as soft contacts or as constraints.
- David Baraff, [*Rigid Body Simulation II: Nonpenetration Constraints*](https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf), SIGGRAPH course notes: how a simulator computes contact and friction forces as constraints. A concept, not ODE's exact solver.
- Roland Siegwart, Illah R. Nourbakhsh and Davide Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd edition, MIT Press 2011: wheel types and their rolling and sliding constraints, an alternative to *Modern Robotics*' chapter 13.
- Roy Featherstone, *Rigid Body Dynamics Algorithms*, Springer 2008: the algorithms of physics engines, with a chapter on contact and impact. Advanced.
