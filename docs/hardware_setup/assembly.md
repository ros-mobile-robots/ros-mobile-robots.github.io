# Assembly

The following video gives an overview of the robot's components and how it will be assembled:

<iframe width="560" height="315" src="https://www.youtube.com/embed/6aAEbtfVbAk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

More detailed assembly instructions are found in the next few sections.
The parts are listed in the [components](../components.md) and the [3D printed parts](3D_print.md).

## Frame

The frame is assembled in 14 steps. In each drawing, the dashed magenta lines show where a part goes.
Steps 2 to 9 work on the bottom of the chassis, so turn it upside down after step 1.

### Step 1: Threaded inserts

![Step 1: four threaded inserts go into the corner posts on top of the chassis](images/frame/01-threaded-inserts.svg)

Press the four M2 brass threaded inserts into the holes of the corner posts on top of the chassis, for example with a soldering iron.
The decks are screwed into these inserts later, so they can be swapped often without wearing out the plastic threads.

**Parts:** chassis, 4 × M2 brass threaded insert

### Step 2: First motor

![Step 2: the first motor goes into its pocket on one side of the chassis](images/frame/02-first-motor.svg)

Turn the chassis upside down and place the first DG01D-E motor in its pocket on one side.

**Parts:** 1 × DG01D-E hobby motor with encoder

### Step 3: Second motor

![Step 3: the second motor goes into the pocket on the other side](images/frame/03-second-motor.svg)

Place the second motor in the pocket on the other side.

**Parts:** 1 × DG01D-E hobby motor with encoder

### Step 4: Motor screws

![Step 4: two long screws through each side wall hold the motors, with an M3 nut on the inside of each motor](images/frame/04-motor-screws.svg)

Fix each motor with two M3 × 25 mm screws through the side wall of the chassis and the motor.
On the inside, put an M3 nut on each screw, in the opening of the motor.
The drawing shows the nuts for the far motor; the other motor gets the same two nuts, hidden from this angle.

**Parts:** 4 × M3 × 25 mm screw, 4 × M3 nut

### Step 5: Motor driver

![Step 5: the motor driver board goes between the motors](images/frame/05-motor-driver.svg)

Place the motor driver board between the two motors.

Some boards come with the screw terminals and pin headers loose, others with everything soldered on.
If yours are loose, solder them on before this step. The drawings show the board without them.
The cables are connected in step 7, once the board is screwed in.

**Parts:** Adafruit DC Motor (+ Stepper) FeatherWing

### Step 6: Motor driver screws

![Step 6: four screws hold the motor driver board](images/frame/06-motor-driver-screws.svg)

Fix the board with four screws.

**Parts:** 4 × M2 self-tapping screw

### Step 7: Motor cables

![Step 7: in the Fritzing view, the red and brown wire of each motor go to the screw terminals on the right side of the motor driver](../fritzing/remo_architecture.svg)

Connect the motors to the motor driver as shown in the Fritzing view above (also on the [Electronics](electronics.md) page):

- Screw the red (M+) and brown (M−) wire of each motor cable into the `M3`/`M4` terminal block of the board.
  The firmware expects the left motor on `M4` and the right motor on `M3` (`MOTOR_LEFT` and `MOTOR_RIGHT` in `diffbot_base_config.h`, see [low-level base controller](../packages/diffbot_base/low-level.md)).
  If a wheel turns the wrong way later, swap its red and brown wire.
- The other four wires of each motor cable (blue G, green H1, yellow H2, orange V) belong to the encoder.
  They go to the Teensy on the breadboard, not to the motor driver.
  Keep each encoder with its motor: H1 and H2 of the left motor go to Teensy pins 5 and 6, those of the right motor to pins 7 and 8.

!!! warning "The Fritzing view pairs the motors the other way round"
    In the Fritzing view, the motor on `M3` has its encoder on pins 5 and 6.
    The firmware reads pins 5 and 6 for the motor on `M4`.
    Follow the text above until the diagram is fixed.

The battery cable follows in step 10. The Fritzing view also shows the I2C wires from the board to the Grove I2C hub.

**Parts:** 2 × motor cable (comes with the motors)

### Step 8: Caster

![Step 8: the caster goes into the round opening of the chassis](images/frame/08-caster.svg)

Put the caster ball into the printed caster base and shroud, then place the caster in the round opening of the chassis.

**Parts:** caster base and shroud (printed), 1 × caster ball (25.4 mm)

### Step 9: Caster screws

![Step 9: four screws hold the caster](images/frame/09-caster-screws.svg)

Fix the caster with four screws.

**Parts:** 4 × M2 self-tapping screw

### Step 10: Battery pack

![Step 10: the battery pack slides into the compartment under the top plate](images/frame/10-battery-pack.svg)

While the chassis is still upside down, connect the cable of the empty battery pack to the 2-pin power terminal of the motor driver: black to − and red to +, as marked on the board.
Then turn the chassis right side up again and slide the battery pack into the compartment under the top plate.
Leave the batteries out until all the wiring is done.

**Parts:** battery pack (for four or eight batteries)

### Step 11: Camera mount

![Step 11: the camera mount closes the battery compartment](images/frame/11-camera-mount.svg)

Slide the camera mount over the open side of the battery compartment.
It keeps the battery pack in place and later holds the adapter for your camera (see [Camera Mount](3D_print.md#camera-mount) on the 3D printing page).

**Parts:** camera mount (printed, `camera_mount.stl`)

### Step 12: Camera mount screws

![Step 12: screws from the sides and the top hold the camera mount](images/frame/12-camera-mount-screws.svg)

Fix the camera mount with screws from both sides and from the top.

**Parts:** 6 × M2 self-tapping screw

### Step 13: Powerbank

![Step 13: the powerbank goes on top of the chassis](images/frame/13-powerbank.svg)

Place the powerbank in its compartment on top of the chassis. It fits snugly; two adhesive pads keep it from shifting.
It can be at most 135.5 × 71 × 18 mm (L × W × H).

**Parts:** powerbank, 2 × adhesive pad

### Step 14: Wheels

![Step 14: the wheels go onto the motor shafts](images/frame/14-wheels.svg)

Push the wheels onto the motor shafts.

**Parts:** 2 × wheel

## Bread Board

![bread board](images/bread_board.svg)
