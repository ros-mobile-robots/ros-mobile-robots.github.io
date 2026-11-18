# Power Supply

DiffBot and Remo each have two separate power supplies:

- A USB power bank runs the Raspberry Pi.
- AA cells in a battery holder run the motors, through the motor driver.

The motors get their own supply because they need more current than a USB connection provides ([Adafruit's FeatherWing guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts)).

| | DiffBot | Remo |
| --- | --- | --- |
| Raspberry Pi | USB power bank | USB power bank |
| Motor driver | [Grove I2C Motor Driver V1.3](grove_motor_driver.md), with an L298 | [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927), with TB6612 driver chips |
| Motor supply | Four alkaline AA batteries, which can't be recharged, in the robot kit's holder under the chassis | Six rechargeable NiMH AA cells in the holder under the top plate |
| When the motor supply runs low | [Replace all four batteries](#when-to-replace-or-recharge) | [Recharge all six cells](#charging-remos-nimh-cells) |
| Motors | Two [DG01D-E](DG01D-E-motor-with-encoder.md), rated 3 V to 9 V | The same |

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
To set the voltage at a motor, the driver switches the battery voltage on and off many times per second, with PWM.
The duty cycle is the share of each period during which the switches are on.
The higher the duty cycle, the higher the average voltage at the motor.

The highest motor command gives the maximum duty cycle.
Remo's driver library, for example, turns its highest command, 255, into 4080 of 4096 PWM counts, a duty cycle of 99.6 % ([`Adafruit_MotorShield.cpp`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/1.1.4/Adafruit_MotorShield.cpp#L254-L256)).
At the maximum duty cycle, the voltage at the motor comes close to the battery voltage under load, minus the voltage that is lost on the way through the driver.
How fast the motor turns at this voltage depends on its load.

So three numbers decide whether a battery suits a robot:

1. The battery voltage under load, from fresh to nearly empty.
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
| Pack equivalent of the cell-test reading | about 4.4 V (1.1 V per cell, battery test) | about 6.0 V (1.0 V per cell, capacity test) |
| Input range of the motor driver | 6 V to 15 V | 4.5 V to 13.5 V |

The cell values come from Energizer's data:

- **Alkaline:** The [Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf) gives about 1.6 V for a fresh alkaline cell without load. Its battery test loads a cell with about 10 Ω for one to two seconds. A fresh cell then shows about 1.5 V, and a cell that shows 1.1 V has about 20 % of its service left. The table's 4.4 V is four times this test value. It is the result of a test on single cells, not a voltage that DiffBot runs on: its motor driver needs at least 6 V.
- **NiMH:** The [NH15-1500 datasheet](https://energizer.com/canada-french/wp-content/uploads/sites/3/2024/09/nh15-1500.pdf) gives 1.2 V nominal for a NiMH AA cell. Its discharge curves start at 1.4 V. It rates the capacity of this 1500 mAh cell for a discharge with 300 mA at 21 °C, down to 1.0 V per cell. The table's 6.0 V is six times this end value. Remo's 2000 mAh cells at the motors' unmeasured current can behave differently, so 6.0 V is a comparison with the datasheet, not a voltage to drive down to. [Recharge Remo's cells](#when-to-replace-or-recharge) after each use.

The two kinds of cells also discharge differently.
The voltage of an alkaline cell slopes down during the whole discharge.
A NiMH cell stays near 1.2 V for most of its discharge and drops only at the end (see the curves in both documents).
So the voltage that Remo's motors can get stays nearly the same until its cells are nearly empty, while the voltage that DiffBot's motors can get falls as its batteries run down.

### When to replace or recharge

Neither robot has a battery-protection cutoff.
The motor battery's voltage isn't measured, and nothing stops the motors when it gets low.
Remo's firmware, for example, has one automatic stop, the command timeout: it releases the motors when no wheel commands arrive ([`wheel_control.h`](https://github.com/ros-mobile-robots/diffbot/blob/f5f282eed7b6a2b146d40c7d5fd641b8a339a77d/diffbot_firmware/lib/wheel_control/wheel_control.h#L98-L109)).
So you decide when the cells are replaced or recharged:

- **DiffBot:** Replace the batteries when the voltage at the motor driver's power terminal is below 6 V while the motors run. Below 6 V, the board is outside the input range that Seeed specifies ([Specifications](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/)). To measure it, set a multimeter to DC volts, with the black lead in COM and the red lead in V. While the motors run, hold the black probe on the − screw and the red probe on the + screw of the power terminal ([Fluke's guide to measuring DC voltage](https://www.fluke.com/en-us/learn/blog/digital-multimeters/how-to-measure-dc-voltage-with-a-digital-multimeter)). Four fresh alkaline batteries give only about 6 V under load, so a set has little margin from the start. The voltage at DiffBot's power terminal isn't measured yet. Replace all four batteries together: Energizer warns that mixing old and new batteries can make them rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings).
- **Remo:** Recharge all six cells after each use. Don't drive until the robot gets slow or stops.

Two things speak against driving Remo until it stops:

- **The speed doesn't show the state of the cells:** Remo's firmware regulates the speed of each wheel. Its [PID controller](packages/diffbot_base/pid.md) compares the measured speed with the target speed and changes the duty cycle. When the voltage of the pack falls, the controller raises the duty cycle. So the speed can look steady while the cells run down, and it gives no reliable sign of when to recharge.
- **A deep discharge damages NiMH cells:** In a pack, the same current flows through all six cells. The cell with the least charge is empty first, and the other five keep driving the current through it. Its voltage then reverses. Energizer calls this battery reversal: the cell can vent, and the damage stays. Energizer recommends removing the load before a NiMH battery is fully discharged ([Nickel Metal Hydride Handbook](https://energizer.com/eu/austria/wp-content/uploads/2024/05/nickelmetalhydride_appman.pdf), Discharge Termination on page 9 and Battery Reversal on page 12).

## DiffBot: four AA batteries and the Grove I2C Motor Driver

DiffBot's four AA batteries sit in the holder of the robot kit, under the chassis.
The holder's cable goes to the power terminal of the [Grove I2C Motor Driver V1.3](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/).

- **Input range:** Seeed specifies 6 V to 15 V at this terminal. The board also makes the 5 V for its own logic from this input. Four alkaline batteries give about 6 V under load, right at the lower end of the range. As they discharge, their voltage falls below 6 V. Then it's time to [replace them](#when-to-replace-or-recharge).
- **Voltage drop:** On its way to a motor, the current passes a protection diode (D9, a 1N4007, in Seeed's [schematic](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/Grove%20-%20I2C%20Motor%20Driver%20%20v1.3b.pdf)) and two transistors of the L298. The diode carries the current of the whole board: both motors and the board's logic. Each bridge of the L298 carries the current of its own motor. At 1 A, the [1N4007 datasheet](https://www.vishay.com/docs/88503/1n4001.pdf) gives up to 1.1 V for the diode. The [L298 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/L298datasheet.pdf) gives 1.8 V to 3.2 V for the two transistors together. Both drops are smaller at lower currents. The L298's typical curve still shows more than 2 V at 0.8 A, its lowest current. These are the datasheets' values at their test currents. DiffBot's own currents aren't measured.
- **What the motors get:** At the maximum duty cycle, a motor gets close to the battery voltage under load minus both drops. An example with assumed values, not a measurement: if the drops add up to 3 V, a motor gets 3 V of the battery's 6 V. That's the lower end of its 3 V to 9 V. So the voltage that DiffBot's motors can get depends a lot on the state of its batteries.

!!! warning "Use alkaline AA batteries in DiffBot's holder"
    Two other kinds of cells have the size of an AA battery and don't suit DiffBot's motor driver:

    - **NiMH cells:** Four of them give 4.8 V nominal, below the driver's 6 V minimum.
    - **14500 lithium-ion cells:** A 14500 cell with 3.6 V or 3.7 V nominal has 4.2 V when fully charged ([Nitecore's charger manual](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), section Default Settings). Four of them give 16.8 V, above the driver's 15 V maximum.

## Remo: six NiMH cells and the Adafruit FeatherWing

Remo's six NiMH AA cells sit in the battery holder under the top plate ([assembly step 10](hardware_setup/assembly.md#step-10-battery-pack)).
The holder's cable goes to the 2-pin power terminal of the [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927).

- **Input range:** Adafruit's product page specifies motors on 4.5 V to 13.5 V, and its [guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts) asks for 5 V to 12 V. Six NiMH cells stay inside both ranges, from 8.4 V fully charged down to 6.0 V at the end value of the capacity test.
- **Voltage drop:** The TB6612 switches with MOSFETs. Its [datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660) gives a drop of 0.5 V (typical, 0.7 V at most) at 1 A, and 0.15 V (typical) at 0.3 A, for the two switches of a bridge together. These are the datasheet's values at its test currents. Remo's own motor currents aren't measured.
- **What the motors get:** At the maximum duty cycle, a motor gets close to the pack's voltage under load minus this small drop. That is about 8 V with fully charged cells and about 7 V for most of the discharge. Both stay below the motors' 9 V maximum.

Six is also the largest number of NiMH cells that Remo can use.
Seven fully charged cells give about 9.8 V, more than the motors' 9 V.
Six alkaline cells don't fit either: fresh, they give about 9.6 V without load.
With the small drop of the TB6612, the motors can get more than 9 V at the maximum duty cycle, especially at a light load.

## Why Remo uses NiMH cells

- **Voltage:** Six NiMH cells keep the FeatherWing in its range and the motors below 9 V, as shown above.
- **Steady voltage:** Their voltage stays near 1.2 V for most of the discharge.
- **High currents:** The higher the current, the longer NiMH cells last compared with alkaline cells. Energizer's comparison gives a NiMH cell more than twice the capacity of an alkaline cell at 1000 mA, and about 1.2 times at 250 mA (Figure 7 in the [Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)).
- **Recharging:** They can be charged again many times, so you don't need new batteries after each run.

Remo uses Amazon Basics NiMH AA cells with 2000 mAh and 1.2 V, sold as an 8-pack ([Amazon.com][amazon_nimh_aa_2000mah_8x_us], [Amazon.de][amazon_nimh_aa_2000mah_8x_ger], affiliate links).
Six of them go into the robot, and two are spare.

## Charging Remo's NiMH cells

Remo's six cells are charged outside the robot, in a [Nitecore UMS4](https://www.nitecore.com/product/ums4) ([Amazon.com][amazon_nitecore_ums4_us], [Amazon.de][amazon_nitecore_ums4_ger], affiliate links).
The charger has four slots, so six cells take two rounds: first four cells, then the other two.
The steps follow Nitecore's [user manual for the UMS4](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), whose English part comes first.
Read the manual's Precautions before the first charge.

1. Take the six cells out of Remo's battery holder.
2. Connect the charger to a USB power adapter with its USB cable.
3. Insert four cells. Each slot has marks for the plus pole and the minus pole. Insert every cell the way the marks of its slot show. The charger starts by itself.
4. Look at the display. A slot that shows `EE EE` has a cell that is inserted the wrong way round or has a short circuit. Take this cell out and check it.
5. Wait until the charger has ended the charge of all four slots. The charger ends the charge of each slot on its own, so one cell can still be charging when the others are full. A short press of the C button steps through the four slots on the display. Check every slot before you take the cells out. A slot that shows `END` below `CHG. STATUS` has charged for 20 hours, and the charger stopped it to protect against a faulty cell (the manual's Overtime Charging Protection). This cell isn't charged.
6. Take out the four cells and charge the remaining two with steps 3 to 5.
7. Unplug the charger and take out the cells when the second round is done. Don't leave the charger unattended while it is connected (precautions 8 and 19 of the manual).
8. Put the six cells back into Remo's holder only if all six have finished their charge without `EE EE` or `END`. Keep them together as one set, and don't mix them with older cells or other types of batteries. Energizer warns that mixing can make cells rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings). So if a cell fails, replace the whole set, not only that cell.

!!! warning "These steps are for Remo's NiMH cells only"
    - **Never charge DiffBot's alkaline AA batteries**, or any other battery that isn't marked as rechargeable. Recharging a battery that isn't rechargeable can make it rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings). Nitecore's manual forbids such batteries in the charger as well (precautions 1 and 10).
    - **Other rechargeable cells** can need settings that this page doesn't describe. Follow Nitecore's manual for them.

### Why NiMH cells need a charger that is made for them

Overcharging shortens the life of a NiMH cell.
So a charger has to notice when a cell is full, and then stop.
Energizer recommends a smart charger for this, one that protects the cells from overcharge ([Nickel Metal Hydride Handbook](https://energizer.com/eu/austria/wp-content/uploads/2024/05/nickelmetalhydride_appman.pdf), pages 10 and 11).
That's also why Energizer warns against chargers that aren't designed for the battery type ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings).

For Remo's cells, the UMS4 does the following:

- **Charging mode:** For NiMH cells, it selects the charging mode by itself ([manual](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), the note under Default Settings).
- **End of the charge:** Nitecore lists −dV/dt as one of the charger's modes ([specifications](https://www.nitecore.com/product/ums4)). This method ends a charge when the cell voltage starts to fall. A NiMH cell's voltage peaks when the cell is full and then falls slightly (Energizer's handbook above, page 10).
- **Charging current:** With all four slots in use, each slot charges with up to 500 mA from a standard USB power adapter, or up to 1000 mA from a Quick Charge 2.0 adapter ([specifications](https://www.nitecore.com/product/ums4)). At 500 mA, an empty 2000 mAh cell takes more than four hours.
- **USB port:** Nitecore's product page describes a USB-C port. The first units from 2018 have a micro-USB port ([review on zeroair.org](https://zeroair.org/2018/12/04/nitecore-ums2-and-ums4-charger-review/)), so check which cable your unit needs.
