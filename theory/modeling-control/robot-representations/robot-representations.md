# Robot Representations

Representations of the robot and its environment are fundamental to the capabilities that make a vehicle autonomous.

To sense, to plan, and to act. Different tasks might require different representations. For example, navigating the city or avoiding a pedestrian on the road.

## State

To quantify representations, we use states.
The state $x_t$ of a robot and of the world has the following properties

  1. $x_t \in X$ exists independently of us and the algorithms that we choose to determine it.
  2. The state evolves over time,

    $$
    \underbrace{x_0, x_1, \dots,}_{\color{orange}Past} \quad \overbrace{x_t}^{\color{green}Present} \quad \underbrace{x_{t+1}, x_{t+n}, \dots}_{\color{red}Future}
    $$

  3. the robot will need to estimate the $\color{green}present$ and $\color{red}future$ state on the fly,
  so it should be efficiently computable. 

    $$
    {\color{red}x_{t+1}} =  f(\color{green}x_t, \color{orange}x_{t-1}, \dots, x_0; \color{green}u_t, \dots, \color{orange}u_0)
    $$



Good choices of state are such that given the present information,
the future state is independent of the past. This is called [Markov property](https://en.wikipedia.org/wiki/Markov_property), 
and it's very desirable because it allows the robot not having to keep track of all the information gathered in the past.

The state is typically observed from the sensor measurements, 
but taking the whole history of the measurements as choice of a state is inefficient, 
because measurements contain redundant information and increase over time, 
so they require more and more computation and memory to process.

## Robot Pose

A sufficient and efficient representation of a mobile robot is the pose $q_t$.

!!! note "Pose Definition"
    $q_t$: position and orientation of the ${\color{orange}\text{robot}}$ (${\color{orange}\text{body}}$) frame relative to a
    ${\color{red}\text{world}}$ (${\color{red}\text{fixed}}$) reference frame.

That is, the position and the orientation of the robot in space.
The pose may also include the linear and the angular velocities.
The environment of a mobile robot can be seen as a 2D world, but pose can be generalized to 3D as well.



!!! todo "TODO"
    Add reference frame image

## Reference Frames ($\mathbb{R}^2$)

To formalize a robot's pose, we need to introduce reference systems.
We take a world frame with origin in W and a robot, or body, frame, 
which moves with the robot and has origin in point A at position (x,y) in the world frame.

- ${\color{red}\text{World frame }}(x^w, y^w)$, origin in $W$
- ${\color{orange}\text{Robot frame }}(x^r, y^r)$, origin in $A$ orientation $\theta$ with $x^w$

!!! todo "TODO"
    Add reference frames image

It is important to express the coordinates of any point
with respect to the robot and the world frames, which in the general case
are rotated and translated one with respect to the other.


## Moving between Reference Frames

Let's look at the math on how to move between frames, 
starting from the simpler case of translations.

!!! todo "TODO"
    Add points frames image


### Translations


Take two reference frames and assume that they are purely translated
with respect to each other by $x_A$ and $y_A$.
