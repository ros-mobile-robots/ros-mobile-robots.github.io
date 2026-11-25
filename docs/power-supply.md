# Power Supply

DiffBot and Remo each have two separate power supplies. A USB power bank runs the Raspberry Pi, and eight NiMH AA cells in a battery holder run the motors, through the motor driver.
This page explains what each supply has to deliver, what the cells give each robot's motor driver and motors, when to recharge the cells, and how to charge them.
Read the page before you buy cells or a charger, and before you put other cells into a robot's holder: the voltage of the cells decides whether they fit the motor driver's input range and what the motors get.

The motors get their own supply because they need more current than a USB connection provides ([Adafruit's FeatherWing guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts)).

| | DiffBot | Remo |
| --- | --- | --- |
| Raspberry Pi | USB power bank | USB power bank |
| Motor driver | [Grove I2C Motor Driver V1.3](grove_motor_driver.md), with an L298 | [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927), with TB6612 driver chips |
| Motor supply | Eight rechargeable NiMH AA cells | Eight rechargeable NiMH AA cells in the holder under the top plate |
| When the motor supply runs low | [Recharge all eight cells](#charging-the-nimh-cells) | Recharge all eight cells |
| Motors | Two [DG01D-E](DG01D-E-motor-with-encoder.md), rated 3 V to 9 V | The same |

## Raspberry Pi

The Raspberry Pi 4 B gets 5 V through its USB-C port.
Raspberry Pi recommends a supply that delivers 3.0 A.
The bare board typically draws 600 mA when active, and USB devices such as the LiDAR add to that ([Raspberry Pi documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#power-supply)).
On both robots, a power bank with a USB-C cable supplies these 5 V. The [Components](components.md) page lists the power banks.
DiffBot's power bank delivers 2.1 A, less than this recommendation. The current that a robot's Raspberry Pi draws with its USB devices isn't measured.

If the supply voltage drops below 4.63 V (±5 %), the Raspberry Pi writes a low-voltage warning to its kernel log, which `dmesg` shows on the Raspberry Pi.
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

1. The battery voltage under load, from fully charged to nearly empty.
2. The input range of the motor driver: the battery voltages for which its manufacturer specifies the driver.
3. The voltage drop in the motor driver. What's left reaches the motors. The DG01D-E motors are rated for 3 V to 9 V, and their [datasheet](https://cdn.sparkfun.com/assets/8/3/b/e/4/DS-16413-DG01D-E_Motor_with_Encoder.pdf) gives 90 RPM at 4.5 V.

## The NiMH cells

Both robots run their motors on eight rechargeable NiMH AA cells.
In series, the voltages of the cells add up:

| | Per cell | Eight cells |
| --- | --: | --: |
| Nominal | 1.2 V | 9.6 V |
| Fully charged | about 1.4 V | about 11.2 V |
| End of the datasheet's capacity test | 1.0 V | 8.0 V |

The cell values come from Energizer's [NH15-1500 datasheet](https://energizer.com/canada-french/wp-content/uploads/sites/3/2024/09/nh15-1500.pdf). It gives 1.2 V nominal for a NiMH AA cell, and its discharge curves start at 1.4 V. It rates the capacity of this 1500 mAh cell for a discharge with 300 mA at 21 °C, down to 1.0 V per cell. The table's 8.0 V is eight times this end value. The robots' cells, with 2000 mAh, can behave differently at the motors' unmeasured currents. So 8.0 V is a comparison with the datasheet, not a voltage to drive down to. [Recharge the cells](#when-to-recharge) after each use.

A NiMH cell stays near 1.2 V for most of its discharge and drops only at the end (see the curves in the datasheet).
So the voltage that the motors can get stays nearly the same until the cells are nearly empty.

### When to recharge

Neither robot has a battery-protection cutoff.
The motor battery's voltage isn't measured, and nothing stops the motors when it gets low.
Remo's firmware, for example, has one automatic stop, [the command timeout](packages/diffbot_base/low-level.md#the-command-timeout): the firmware releases the motors when no wheel commands arrive.
So you decide when the cells are recharged: on both robots, recharge all eight cells after each use, as [Charging the NiMH cells](#charging-the-nimh-cells) describes. Don't drive until the robot gets slow or stops.

Two things speak against driving a robot until it stops. The first is shown on Remo:

- **The speed doesn't show the state of the cells:** Remo's firmware regulates the speed of each wheel. Its [PID controller](packages/diffbot_base/pid.md) compares the measured speed with the target speed and changes the duty cycle. When the voltage of the pack falls, the controller raises the duty cycle. So the speed can look steady while the cells run down, and it gives no reliable sign of when to recharge.
- **A deep discharge damages NiMH cells:** In a pack, the same current flows through all eight cells. The cell with the least charge is empty first, and the others keep driving the current through it. Its voltage then reverses. Energizer calls this battery reversal: the cell can vent, and the damage stays. Energizer recommends removing the load before a NiMH battery is fully discharged ([Nickel Metal Hydride Handbook](https://energizer.com/eu/austria/wp-content/uploads/2024/05/nickelmetalhydride_appman.pdf), Discharge Termination on page 9 and Battery Reversal on page 12).

## DiffBot: eight NiMH cells and the Grove I2C Motor Driver

DiffBot's eight NiMH AA cells are connected to the power terminal of the [Grove I2C Motor Driver V1.3](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/).

- **Input range:** Seeed specifies 6 V to 15 V at this terminal. Eight NiMH cells stay inside this range, from about 11.2 V fully charged down to 8.0 V at the end value of the capacity test.
- **Logic supply:** The board makes the supply for its own logic from this input. Behind a protection diode (D9 in Seeed's [schematic](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/Grove%20-%20I2C%20Motor%20Driver%20%20v1.3b.pdf)), a 78M05 regulator supplies the board's microcontroller, the logic of the L298 and the VCC pin of the I2C connector. The [78M05 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/ST_78M05DataSheet.pdf) specifies its 5 V for inputs from 7 V to 20 V. The diode loses up to 1.1 V at 1 A (see below), so at the cells' nominal 9.6 V the regulator gets at least about 8.5 V.
- **Voltage drop:** On its way to a motor, the current passes the protection diode (D9, a 1N4007, in Seeed's schematic) and two transistors of the L298. The diode carries the current of the whole board: both motors and the board's logic. Each bridge of the L298 carries the current of its own motor. At 1 A, the [1N4007 datasheet](https://www.vishay.com/docs/88503/1n4001.pdf) gives up to 1.1 V for the diode. The [L298 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/L298datasheet.pdf) gives 1.8 V to 3.2 V for the two transistors together. Both drops are smaller at lower currents. Seeed limits the board to 0.5 A per channel ([Specifications](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/)). On the bench, with the board on 10 V, a DG01D-E got 7.9 V at the highest command, a drop of 2.1 V ([Wheel Encoder Measurements](DG01D-E-motor-with-encoder.md#wheel-encoder-measurements)).
- **What the motors get:** At the maximum duty cycle, a motor gets close to the battery voltage under load minus the drop. With the bench's 2.1 V, that is about 7.5 V at the cells' nominal 9.6 V and about 9.1 V when they are fully charged (11.2 V), slightly more than the motors' rated 9 V. DiffBot's own currents and motor voltages aren't measured.

## Remo: eight NiMH cells and the Adafruit FeatherWing

Remo's eight NiMH AA cells sit in the battery holder under the top plate ([assembly step 10](hardware_setup/assembly.md#step-10-battery-pack)).
The holder's cable goes to the 2-pin power terminal of the [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927).

- **Input range:** Adafruit's product page specifies motors on 4.5 V to 13.5 V, and its [guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts) asks for 5 V to 12 V. Eight NiMH cells stay inside both ranges, from about 11.2 V fully charged down to 8.0 V at the end value of the capacity test.
- **Voltage drop:** The TB6612 switches with MOSFETs, transistors that lose much less voltage than the L298's. Its [datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660) gives a drop of 0.5 V (typical, 0.7 V at most) at 1 A, and 0.15 V (typical) at 0.3 A, for the two switches of a bridge together. These are the datasheet's values at its test currents. Remo's own motor currents aren't measured.
- **What the motors get:** Remo's PID controller limits its output to `PWM_MAX`, 2⁸ − 1 = 255, the highest command ([`diffbot_base_config.h`](https://github.com/ros-mobile-robots/diffbot/blob/101f179a50afa92c625eb9f72dcd5443c9cc49e2/diffbot_firmware/lib/config/diffbot_base_config.h#L83)). At this command, a motor gets close to the pack's voltage under load minus the small drop: about 9.1 V to 9.5 V at the cells' nominal 9.6 V, and up to about 11 V with fully charged cells. That is more than the motors' rated 9 V. How often the controller reaches its highest command depends on the target speed and the load.

## Other AA-sized cells

Both holders take any AA-sized cell, and two other kinds don't suit the robots:

- **Alkaline AA batteries:** They can't be recharged. A fresh alkaline cell has about 1.6 V without load ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)), so eight of them give about 12.8 V, and the motors get even more than with charged NiMH cells.
- **14500 lithium-ion cells:** A 14500 cell with 3.6 V or 3.7 V nominal has 4.2 V when fully charged ([Nitecore's charger manual](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), section Default Settings). Eight of them give 33.6 V, far above the 15 V of the Grove I2C Motor Driver and the 13.5 V of the FeatherWing.

## Why the robots use NiMH cells

- **Voltage:** Eight NiMH cells stay inside the input ranges of both motor drivers, as shown above.
- **Steady voltage:** Their voltage stays near 1.2 V for most of the discharge.
- **High currents:** The higher the current, the longer NiMH cells last compared with alkaline cells. Energizer's comparison gives a NiMH cell more than twice the capacity of an alkaline cell at 1000 mA, and about 1.2 times at 250 mA (Figure 7 in the [Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)).
- **Recharging:** They can be charged again many times, so you don't need new batteries after each run.

The robots use Amazon Basics NiMH AA cells with 2000 mAh and 1.2 V, sold as an 8-pack ([Amazon.com][amazon_nimh_aa_2000mah_8x_us], [Amazon.de][amazon_nimh_aa_2000mah_8x_ger], affiliate links).
One pack fills a robot's holder.

## Charging the NiMH cells

The NiMH cells are charged outside the robot, in a [Nitecore UMS4](https://www.nitecore.com/product/ums4) ([Amazon.com][amazon_nitecore_ums4_us], [Amazon.de][amazon_nitecore_ums4_ger], affiliate links).
The charger has four slots, so the eight cells take two rounds of four.
The steps follow Nitecore's [user manual for the UMS4](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), whose English part comes first.
Read the manual's Precautions before the first charge.

1. Take the cells out of the robot's battery holder. On DiffBot, shut down the Raspberry Pi and unplug its power bank first, and plug the power bank in again only when the cells are back. The [Grove I2C Motor Driver page](grove_motor_driver.md#connection) explains this order.
2. Connect the charger to a USB power adapter with its USB cable.
3. Insert four of the eight cells. Each slot has marks for the plus pole and the minus pole. Insert every cell the way the marks of its slot show. The charger starts by itself.
4. Check each slot on the display. A short press of the C button selects the next slot, and short presses of the V button step through the slot's values, among them Battery Health.
    - **`EE EE`:** The cell is inserted the wrong way round or has a short circuit. Take it out and check it. If it was only the wrong way round, insert it correctly, and the checks of this slot start again. If `EE EE` comes back, the cell is faulty.
    - **Battery Health `Poor`:** The cell's internal resistance is above 250 mΩ, and the manual says that the cell should be replaced (Automatic Battery Internal Resistance Detection).
5. Wait until the display shows for each slot that its charge has ended. The charger ends the charge of each slot on its own, so one cell can still be charging when the others are full. Check every slot before you take the cells out. A slot that shows `END` below `CHG. STATUS` has charged for 20 hours, and the charger stopped it to protect against a faulty cell (the manual's Overtime Charging Protection). This cell isn't charged.
6. Take out the four cells and repeat steps 3 to 5 with the other four.
7. Unplug the charger and take out the cells when the second round is done. Don't leave the charger unattended while it is connected (precautions 8 and 19 of the manual).
8. Put the cells back into the robot's holder only if all of them have finished their charge and none is faulty or showed `Poor` or `END`. Keep the eight cells together as one set, and don't mix them with older cells or other types of batteries. Energizer warns that mixing can make cells rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings). So if a cell fails, replace the whole set, not only that cell.

!!! warning "These steps are for the robots' NiMH cells only"
    - **Never charge alkaline AA batteries**, or any other battery that isn't marked as rechargeable. Recharging a battery that isn't rechargeable can make it rupture or leak (Alkaline Handbook, section Testing / Care / Warnings). Nitecore's manual forbids such batteries in the charger as well (precautions 1 and 10).
    - **Other rechargeable cells** can need settings that this page doesn't describe. Follow Nitecore's manual for them.

### Why NiMH cells need a charger that is made for them

Overcharging shortens the life of a NiMH cell.
So a charger has to notice when a cell is full, and then stop.
Energizer recommends a smart charger for this, one that protects the cells from overcharge ([Nickel Metal Hydride Handbook](https://energizer.com/eu/austria/wp-content/uploads/2024/05/nickelmetalhydride_appman.pdf), pages 10 and 11).
That's also why Energizer warns against chargers that aren't designed for the battery type ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings).

For the robots' NiMH cells, the UMS4 does the following:

- **Charging mode:** For NiMH cells, it selects the charging mode by itself ([manual](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), the note under Default Settings).
- **End of the charge:** Nitecore lists −dV/dt as one of the charger's modes ([specifications](https://www.nitecore.com/product/ums4)). This method ends a charge when the cell voltage starts to fall. A NiMH cell's voltage peaks when the cell is full and then falls slightly (Energizer's handbook above, page 10).
- **Charging current:** With all four slots in use, each slot charges with up to 500 mA from a standard USB power adapter, or up to 1000 mA from a Quick Charge 2.0 adapter ([specifications](https://www.nitecore.com/product/ums4)). At 500 mA, an empty 2000 mAh cell takes more than four hours.
- **USB port:** Nitecore's product page describes a USB-C port. The first units from 2018 have a micro-USB port ([review on zeroair.org](https://zeroair.org/2018/12/04/nitecore-ums2-and-ums4-charger-review/)), so check which cable your unit needs.
