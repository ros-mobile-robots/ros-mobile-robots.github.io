# Modeling of a differential drive robot

Mathematical models are powerful tools because they allow us to predict the future.

$$
\begin{align}
    \dot{x}_t &= f(x_t,{\color{orange}{u_t}}) \\
    {\color{green}{y}_t} &= g(x_t,{\color{orange}{u_t}})
\end{align}
$$

Models map between inputs and outputs of systems and can be derived from first principles or learned from data.

!!! todo "TODO"
    Add robot input output image


We use models to quantify some essential variable that is useful to accomplish a task, 
not to provide a faithful description of the exact reality of all the physical processes going on.

The Diffbot is a differential drive robot, where the motion of each wheel is controlled by one DC motor.

!!! todo "TODO"
    Add image of robot and two dc motors as input


DC motors receive voltages ${\color{orange}V_{l/r,t}}$ as inputs and produce torques on the motor drive axis that spins the wheels,
and leads to angular velocity $\dot{\phi}_{l/r,t}$ of the motor shaft and wheel.
The movement of the wheels will produce an evolution of the robots pose ${\color{green}q_t}$ over time, which is what we want to quantify.

# Forward and Inverse Kinematics

Through these models, we can answer two questions.

1. (Forward Kinematics) Given a sequence of commands to the wheels $\dot{\phi}_{1}, \dot{\phi}_{2}, \cdots, \dot{\phi}_{t}$, how will the robot move?
2. (Inverse Kinematics) If we want the robot to move in a certain way, given a desired movement $(q_1, q_2, \cdots, q_t)$, what
commands should we send to the wheels?

# Notations

We know that the pose of a robot is the position and the orientation of the body frame with respect to the world frame.
We define the robot body frame so that the origin, $A$, is in the mid-axle point.

{% raw %}
- World Frame: $\{{\color{blue}x^{w}}, {\color{blue}y^{w}}\}$
- Body (robot) frame: $\{{\color{orange}x^{r}}, {\color{orange}y^{r}}\}$
{% endraw %}

!!! todo "TODO"
    Add image of robot including orange reference frame and blue world reference frame.


**Assumption 1**: robot is **symmetric** along longitudinal axis ($x^r$) and we take it as the x direction of the robot frame.

- Equidsitand wheels (axle length = $2L$). Both wheels will be at a distance $L$ from point $A$.
- Identical wheels with diameter $R$ ($R_l = R_r = R)
- Center of mass of the robot will lie on x-axis $x^r$ at distance $c$ from $A$


**Assumption 2**: robot chassis is **rigid body**.

- Distance between any two points of the robot does not change in time.
- in particular $\dot{c} = 0$, whre $(\dot{\star}) = \frac{d(\star)}{dt}$.