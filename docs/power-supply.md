# Power Supply

DiffBot and Remo each have two separate power supplies:

- A USB power bank runs the Raspberry Pi.
- AA cells in a battery holder run the motors, through the motor driver.

The motors get their own supply because they need more current than a USB connection provides ([Adafruit's FeatherWing guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts)).

| | DiffBot | Remo |
| --- | --- | --- |
| Raspberry Pi | USB power bank | USB power bank |
| Motor driver | [Grove I2C Motor Driver V1.3](grove_motor_driver.md), with an L298 | [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927), with TB6612 driver chips |
| Motor supply | Four AA batteries in the robot kit's holder under the chassis | Six NiMH AA cells in the holder under the top plate |
| Motors | Two [DG01D-E](DG01D-E-motor-with-encoder.md), rated 3 V to 9 V | The same |

The sections on [DiffBot](#diffbot-four-aa-batteries-and-the-grove-i2c-motor-driver) and [Remo](#remo-six-nimh-cells-and-the-adafruit-featherwing) each have a figure of the robot's power flow.

## Raspberry Pi

The Raspberry Pi 4 B gets 5 V through its USB-C port.
Raspberry Pi recommends a supply that delivers 3.0 A.
The bare board typically draws 600 mA when active, and USB devices such as the LiDAR add to that ([Raspberry Pi documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#power-supply)).
On both robots, a power bank with a USB-C cable supplies these 5 V. The [Components](components.md) page lists the power banks.

If the supply voltage drops below 4.63 V (±5 %), the Raspberry Pi writes a low-voltage warning to its kernel log, which `dmesg` shows.
If you see such warnings, use a power bank that delivers more current, or a better cable.

## From the battery to the motors

The motor driver sits between the battery and the motors.
For each motor, it has an H-bridge: four switches that let the current flow through the motor in either direction.
To set the speed, the driver switches the battery voltage on and off many times per second, with PWM.
The longer the switches stay on in each period, the higher the average voltage at the motor, and the faster it turns.
At full speed, the switches stay on.
Then the motor gets the battery voltage minus the voltage that the driver itself drops.

So three numbers decide whether a battery suits a robot:

1. The battery voltage, from fresh to nearly empty.
2. The input range of the motor driver. The battery voltage has to stay inside it.
3. The voltage drop in the motor driver. What's left reaches the motors. The DG01D-E motors are rated for 3 V to 9 V, and their [datasheet](https://cdn.sparkfun.com/assets/8/3/b/e/4/DS-16413-DG01D-E_Motor_with_Encoder.pdf) gives 90 RPM at 4.5 V.

## Alkaline and NiMH cells

DiffBot runs on four alkaline AA batteries, Remo on six rechargeable NiMH AA cells.
In series, the voltages of the cells add up:

| | Four alkaline AA (DiffBot) | Six NiMH AA (Remo) |
| --- | --: | --: |
| Per cell, nominal | 1.5 V | 1.2 V |
| Per cell, fresh or fully charged | about 1.6 V | about 1.4 V |
| Pack, nominal | 6.0 V | 7.2 V |
| Pack, fresh or fully charged | about 6.4 V | about 8.4 V |
| Pack, nearly empty | about 4.4 V (1.1 V per cell) | about 6.0 V (1.0 V per cell) |

The values come from Energizer's data:

- The [Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf) gives about 1.6 V for a fresh alkaline cell without load. Its battery test loads a cell with about 10 Ω for one to two seconds. A fresh cell then shows about 1.5 V, and a cell that shows 1.1 V has about 20 % of its service left. The table's 4.4 V uses this test value. A running motor draws more current and pulls the voltage lower, so take it as a guide, not a limit.
- The [NH15-1500 datasheet](https://energizer.com/canada-french/wp-content/uploads/sites/3/2024/09/nh15-1500.pdf) gives 1.2 V nominal for a NiMH AA cell. Its discharge curves start at 1.4 V. It rates the capacity for a discharge down to 1.0 V per cell, which the table uses as nearly empty. The curves go on below that, so a cell can be discharged further.

The two kinds of cells also discharge differently.
The voltage of an alkaline cell slopes down during the whole discharge.
A NiMH cell stays near 1.2 V for most of its discharge and drops only at the end (see the curves in both documents).
So Remo's top speed stays nearly the same until its cells are nearly empty, while DiffBot's top speed falls as its batteries run down.

Neither robot has a battery-protection cutoff.
The motor battery's voltage isn't measured, and nothing stops the motors when it gets low.
Remo's firmware, for example, stops the motors only when no wheel commands arrive ([`main.cpp`]({{ diffbot_src_url }}/diffbot_base/scripts/base_controller/src/main.cpp#L44-L51)).
So watch the speed. When a robot gets noticeably slower, replace DiffBot's batteries or recharge Remo's cells.
The slowdown is a hint, not a measurement of the charge that's left.

## DiffBot: four AA batteries and the Grove I2C Motor Driver

DiffBot's four AA batteries sit in the holder of the robot kit, under the chassis.
The holder's cable goes to the power terminal of the [Grove I2C Motor Driver V1.3](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/).

--8<-- "includes/power-supply-flow-diffbot.html"

The figure shows the two supplies and the parts that are known to be on them.
It leaves out the Teensy, the encoders, the LiDAR and the camera, because their supply on DiffBot isn't checked yet.
The same holds for the I2C connection that brings the motor commands to the motor driver.

- **Input range:** Seeed specifies 6 V to 15 V at this terminal. The board also makes the 5 V for its own logic from this input. Four alkaline batteries give about 6 V under load, right at the lower end of the range. As they discharge, their voltage falls below 6 V.
- **Voltage drop:** On its way to a motor, the current passes a protection diode (D9, a 1N4007, in Seeed's [schematic](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/Grove%20-%20I2C%20Motor%20Driver%20%20v1.3b.pdf)) and two transistors of the L298. At 1 A, the [1N4007 datasheet](https://www.vishay.com/docs/88503/1n4001.pdf) gives up to 1.1 V for the diode. The [L298 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/L298datasheet.pdf) gives 1.8 V to 3.2 V for the two transistors together. Both drops are smaller at lower currents. The L298's typical curve still shows more than 2 V at 0.8 A, its lowest current.
- **What the motors get:** At full speed, a motor gets the battery voltage minus both drops. For example, if the drops add up to 3 V, a motor gets 3 V of the battery's 6 V. That's the lower end of its 3 V to 9 V. So DiffBot's top speed depends a lot on the state of its batteries.

!!! warning "Four NiMH cells are too few for DiffBot"
    Four NiMH cells give 4.8 V nominal, below the 6 V minimum of the Grove I2C Motor Driver.
    Use alkaline AA batteries in DiffBot's holder.

## Remo: six NiMH cells and the Adafruit FeatherWing

Remo's six NiMH AA cells sit in the battery holder under the top plate ([assembly step 10](hardware_setup/assembly.md#step-10-battery-pack)).
The holder's cable goes to the 2-pin power terminal of the [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927).

--8<-- "includes/power-supply-flow-remo.html"

- **From the power bank:** the Raspberry Pi gets 5 V at its USB-C port. The Teensy gets 5 V from a USB port of the Raspberry Pi, over the cable that also carries the data. The Teensy's 3.3 V pin supplies the encoders, the IMU and the logic of the motor driver ([bench check](https://github.com/ros-mobile-robots/diffbot/issues/169), section 8).
- **From the motor battery:** the motors only. The motor driver switches the battery voltage to them.
- **The motor driver** is the only part on both supplies. Its logic runs on the Teensy's 3.3 V and gets the motor commands over I2C. Its motor side runs on the battery. On the board, the battery's minus pole and the ground wire (GND) to the Teensy are one connection ([schematic](https://github.com/adafruit/Adafruit-DC-Stepper-Motor-FeatherWing-PCB)), so both supplies share one ground.
- **Without the Teensy, the motors stop.** The motor driver's logic loses its 3.3 V together with the Teensy. In the bench check, pulling the Teensy's USB cable while driving stopped the wheels at once.
- **The LiDAR and the camera** are drawn with a dashed line, their data connection to the Raspberry Pi. Their supply isn't drawn, because it isn't checked on Remo yet.

- **Input range:** Adafruit's product page specifies motors on 4.5 V to 13.5 V, and its [guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts) asks for 5 V to 12 V. Six NiMH cells stay inside both ranges, from 8.4 V fully charged down to 6.0 V at the end of the rated discharge.
- **Voltage drop:** The TB6612 switches with MOSFETs. Its [datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660) gives a drop of 0.5 V (typical, 0.7 V at most) at 1 A, and 0.15 V (typical) at 0.3 A, for the two switches of a bridge together. So the motors get almost the full battery voltage.
- **What the motors get:** About 8 V with fully charged cells, falling to about 6 V when they're nearly empty. This stays below the motors' 9 V maximum.

Six is also the largest number of NiMH cells that Remo can use.
Seven fully charged cells give about 9.8 V, more than the motors' 9 V.
Six alkaline cells don't fit either: fresh, they give about 9.6 V without load.
With the small drop of the TB6612, the motors can get more than 9 V at full speed, especially at a light load.

## Why NiMH cells

- **Voltage:** Six NiMH cells keep the FeatherWing in its range and the motors below 9 V, as shown above.
- **Steady speed:** Their voltage stays near 1.2 V for most of the discharge.
- **High currents:** The higher the current, the longer NiMH cells last compared with alkaline cells. Energizer's comparison gives a NiMH cell more than twice the capacity of an alkaline cell at 1000 mA, and about 1.2 times at 250 mA (Figure 7 in the [Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)).
- **Recharging:** They can be charged again many times, so you don't need new batteries after each run.

Remo uses Amazon Basics NiMH AA cells with 2000 mAh and 1.2 V, sold as an 8-pack ([Amazon.com][amazon_nimh_aa_2000mah_8x_us], [Amazon.de][amazon_nimh_aa_2000mah_8x_ger], affiliate links).
Six of them go into the robot, and two are spare.

## Charging the cells

NiMH and lithium-ion cells need different charging methods, so the charger has to know which kind of cell it charges.
The Nitecore UMS4 below, for example, lists different charging voltages for the two kinds.
It detects the kind of each cell and picks the charging mode by itself: constant current, constant voltage, or −dV/dt, which ends a charge when the cell voltage starts to fall ([Nitecore's specifications](https://www.nitecore.com/product/ums4)).
So charge the cells only in a charger made for NiMH cells.
Energizer warns against chargers that aren't designed for the battery type, and against recharging non-rechargeable batteries, which can rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings).

Remo's cells are charged with the [Nitecore UMS4](https://www.nitecore.com/product/ums4) ([Amazon.com][amazon_nitecore_ums4_us], [Amazon.de][amazon_nitecore_ums4_ger], affiliate links).
It has four slots, detects NiMH and lithium-ion cells by itself, and stops charging when they're full.
It gets its power over USB from a USB power adapter.
Nitecore's product page describes a USB-C port. The first units from 2018 have a micro-USB port ([review on zeroair.org](https://zeroair.org/2018/12/04/nitecore-ums2-and-ums4-charger-review/)), so check which cable your unit needs.

To charge Remo's cells:

1. Take the six cells out of the battery holder.
2. Connect the charger to a USB power adapter.
   With all four slots in use, each slot charges with up to 500 mA, or up to 1000 mA with a Quick Charge 2.0 adapter ([Nitecore's specifications](https://www.nitecore.com/product/ums4)).
3. Insert four cells. The display shows the charging progress of each slot.
4. When the charger has stopped, take out the full cells and charge the remaining two.
5. Keep the six cells together as one set. Charge all six after each use, and don't mix them with older cells or other types of batteries. Energizer warns that mixing can make cells rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)).
