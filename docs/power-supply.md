# Power Supply

On both robots, a USB power bank runs the Raspberry Pi, and a separate battery runs the motors through their driver. DiffBot uses four primary alkaline AA cells in the kit's holder. Remo uses eight rechargeable nickel-metal hydride (NiMH) AA cells. Check the driver's input range before you choose cells of another type.

| | DiffBot | Remo |
| --- | --- | --- |
| Raspberry Pi | USB power bank | USB power bank |
| Motor driver | [Grove I2C Motor Driver V1.3](grove_motor_driver.md), with an L298 | [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927), with TB6612 driver chips |
| Motor supply | Four alkaline AA cells in the robot car kit's holder | Eight rechargeable NiMH AA cells in the holder under the top plate |
| Used cells | [Replace all four at the same time](#diffbot-four-alkaline-cells-and-the-grove-i2c-motor-driver) | [Recharge all eight after each use](#charging-the-nimh-cells) |
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
The driver uses pulse-width modulation (PWM) to change the fraction of each cycle for which it applies voltage to the motor.
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

The power flow figures in the sections on [DiffBot](#diffbot-four-alkaline-cells-and-the-grove-i2c-motor-driver) and [Remo](#remo-eight-nimh-cells-and-the-adafruit-featherwing) show the path from the battery through the motor driver to the motors for each robot.

## Remo's NiMH cells

Remo runs its motors on eight rechargeable NiMH AA cells.
In series, the voltages of the cells add up:

| | Per cell | Eight cells |
| --- | --: | --: |
| Nominal | 1.2 V | 9.6 V |
| At the start of the example discharge curves | about 1.4 V | about 11.2 V |
| End of the datasheet's capacity test | 1.0 V | 8.0 V |

The cell values come from Energizer's [NH15-1500 datasheet](https://energizer.com/canada-french/wp-content/uploads/sites/3/2024/09/nh15-1500.pdf). It gives 1.2 V nominal for a NiMH AA cell. Energizer's curves for the NH15-1500 start near 1.4 V per cell. Eight cells at that voltage would give about 11.2 V. The voltage of Remo's pack under load has not been measured. The datasheet rates the capacity of this 1500 mAh cell for a discharge with 300 mA at 21 °C, down to 1.0 V per cell. The table's 8.0 V is eight times this end value. Remo's 2000 mAh cells can behave differently at the currents its motors draw. So 8.0 V is a comparison with the datasheet, not a voltage to drive down to. [Recharge the cells](#when-to-recharge) after each use.

A NiMH cell stays near 1.2 V for most of its discharge and drops only at the end (see the curves in the datasheet).
So the voltage that the motors can get stays nearly the same until the cells are nearly empty.

### When to recharge

Remo's firmware doesn't monitor the motor battery's voltage.
Its [command timeout](packages/diffbot_base/low-level.md#the-command-timeout) releases the motors when wheel commands stop arriving.
Recharge Remo's eight cells after each use, as [Charging the NiMH cells](#charging-the-nimh-cells) describes.

Two things speak against driving Remo until it stops:

- **The speed doesn't show the state of the cells:** Remo's firmware regulates the speed of each wheel. Its [PID controller](packages/diffbot_base/pid.md) compares the measured speed with the target speed and changes the duty cycle. When the voltage of the pack falls, the controller raises the duty cycle. So the speed can look steady while the cells run down, and it gives no reliable sign of when to recharge.
- **A deep discharge damages NiMH cells:** In a pack, the same current flows through all eight cells. The cell with the least charge is empty first, and the others keep driving the current through it. Its voltage then reverses. Energizer calls this battery reversal: the cell can vent, and the damage stays. Energizer recommends removing the load before a NiMH battery is fully discharged ([Nickel Metal Hydride Handbook](https://energizer.com/eu/austria/wp-content/uploads/2024/05/nickelmetalhydride_appman.pdf), Discharge Termination on page 9 and Battery Reversal on page 12).

## DiffBot: four alkaline cells and the Grove I2C Motor Driver

DiffBot's four alkaline AA cells sit in the battery holder of its robot car kit and are connected to the power terminal of the [Grove I2C Motor Driver V1.3](grove_motor_driver.md#connection).

--8<-- "includes/power-supply-flow-diffbot.html"

- **From the power bank:** The Raspberry Pi gets 5 V at its USB-C port. It supplies the LiDAR through the LiDAR's USB adapter, and the microcontroller (MCU), a Teensy, over the USB cable that also carries the data. The Teensy's 3.3 V supplies the IMU. The Raspberry Pi's 5 V also supplies both wheel encoders, through the breadboard's power rail, and the encoders send their signals to the Teensy.
- **Camera:** An optional Raspberry Pi Camera Module 2 connects to the Raspberry Pi's camera port. Its ribbon cable carries 3.3 V and camera data (the connector J3 in the [Raspberry Pi 4's schematics](https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-reduced-schematics.pdf), J1 in the [Camera Module 2's schematic](https://datasheets.raspberrypi.com/camera/camera-module-2-schematics.pdf)).
- **Motor driver:** The driver is wired to the Raspberry Pi's I2C, through a Grove I2C hub. The driver makes the supply for its own logic from the battery, as the bullet on its logic supply below explains. The ground wire joins the grounds of the two supplies, and the I2C signals need it as their reference. The Grove I2C Motor Driver page gives the wiring, the jumper J4 and the signal levels.

The battery and the motor driver fit together like this:

- **Voltage:** A primary alkaline AA cell is rated at 1.5 V. Energizer gives an open-circuit example of 1.6 V per cell ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)). Four in series give 6.0 V nominal, or about 6.4 V fresh without load.
- **Input range:** Seeed specifies 6 V to 15 V at the screw terminal ([specifications](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/#specifications)). The pack's nominal voltage is at that lower limit. When the motors draw current, even fresh cells can drop below it. At Energizer's example device end point of 0.9 V per cell, the pack would give 3.6 V, already below the driver's range. Energizer gives 0.8 V per cell as the minimum discharge cutoff, because a deep discharge increases gassing. DiffBot has no measured replacement threshold.
- **Logic supply:** The motor battery feeds the board's 78M05 regulator through the protection diode D9 (Seeed's [schematic](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/Grove%20-%20I2C%20Motor%20Driver%20%20v1.3b.pdf)). The regulator supplies the ATmega8 and the logic of the L298. The driver's jumper J4 connects that rail to the Grove connector's VCC pin when it is fitted. The [78M05 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/ST_78M05DataSheet.pdf) specifies a 5 V output for 7 V to 20 V at the regulator's input. Even the 6.4 V open-circuit example is below 7 V before the diode's drop. The logic voltage on DiffBot has not been measured.
- **Voltage drop:** On its way to a motor, the current passes the protection diode (D9, a 1N4007, in Seeed's schematic) and two transistors of the L298. The diode carries the current of the whole board: both motors and the board's logic. Each bridge of the L298 carries the current of its own motor. At 1 A, the [1N4007 datasheet](https://www.vishay.com/docs/88503/1n4001.pdf) gives up to 1.1 V for the diode. The [L298 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/L298datasheet.pdf) gives 1.8 V to 3.2 V for the two transistors together. Both drops are smaller at lower currents. Seeed limits the board to 0.5 A per channel ([Specifications](https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.3/#specifications)).
- **What the motors get:** At speed 100 on the bench, the motor received 7.9 V with 10 V at the board, a difference of 2.1 V ([Wheel Encoder Measurements](DG01D-E-motor-with-encoder.md#wheel-encoder-measurements)). If the board operated at 6.0 V under load and the difference were still 2.1 V, the motor would receive about 3.9 V. The motor is rated for 3 V to 9 V. DiffBot's loaded pack voltage, motor voltage and motor current have not been measured.
- **Used cells:** Replace all four primary alkaline cells together, with fresh AA cells of the same type and make. Never recharge these cells or mix them with used cells ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), Care and Handling).

## Remo: eight NiMH cells and the Adafruit FeatherWing

Remo's eight NiMH AA cells sit in the battery holder under the top plate ([assembly step 10](hardware_setup/assembly.md#step-10-battery-pack)).
The holder's cable goes to the 2-pin power terminal of the [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927).

--8<-- "includes/power-supply-flow-remo.html"

- **From the power bank:** The Raspberry Pi gets 5 V at its USB-C port. The microcontroller (MCU), a Teensy that runs the [low-level base controller](packages/diffbot_base/low-level.md), gets 5 V from a USB port of the Raspberry Pi, over the cable that also carries the data. The Raspberry Pi also powers the LiDAR, through the LiDAR's USB adapter on another of its USB ports. The adapter has no supply of its own. The Teensy's 3.3 V pin supplies the encoders, the IMU and the logic of the motor driver ([bench check](https://github.com/ros-mobile-robots/diffbot/issues/169), section 8).
- **From the motor battery:** The eight cells supply the FeatherWing's motor power terminal. During a drive pulse, the motor current flows from the pack through the bridge and the motor, and returns to the pack through the board's motor power connection.
- **Motor driver:** The Teensy supplies the FeatherWing's 3.3 V logic and sends the motor commands over I2C. On the FeatherWing, the PCA9685 turns those commands into direction and PWM signals for the TB6612. The ground wire completes the 3.3 V logic circuit and gives the Teensy and the FeatherWing a common reference for I2C. The TB6612 reads its control signals relative to that same ground ([TB6612FNG datasheet](https://cdn.sparkfun.com/assets/0/1/b/b/3/TB6612FNG.pdf), input voltage). In the wiring shown, the FeatherWing joins the Teensy's ground to the battery's minus pole ([schematic](https://github.com/adafruit/Adafruit-DC-Stepper-Motor-FeatherWing-PCB)).
- **Without the Teensy:** The motors stop. The motor driver's logic loses its 3.3 V together with the Teensy. In the bench check, pulling the Teensy's USB cable while driving stopped the wheels at once.
- **Camera:** The camera's ribbon cable carries 3.3 V and data between the Raspberry Pi and the Camera Module 2.

The battery and the motor driver fit together like this:

- **Input range:** Adafruit's product page specifies motors on 4.5 V to 13.5 V, and its [guide](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts) asks for 5 V to 12 V. Eight NiMH cells stay inside both ranges, from about 11.2 V at the start of the example discharge curves down to 8.0 V at the end value of the capacity test.
- **Voltage drop:** The TB6612 switches with MOSFETs, transistors that lose much less voltage than the L298's. Its [datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660) gives a drop of 0.5 V (typical, 0.7 V at most) at 1 A, and 0.15 V (typical) at 0.3 A, for the two switches of a bridge together, with both chip supplies at 5 V. These are the datasheet's values at its test currents. Remo's own motor currents aren't measured.
- **What the motors get:** Remo's PID controller limits its output to `PWM_MAX`, 2⁸ − 1 = 255, the highest command ([`diffbot_base_config.h`](https://github.com/ros-mobile-robots/diffbot/blob/101f179a50afa92c625eb9f72dcd5443c9cc49e2/diffbot_firmware/lib/config/diffbot_base_config.h#L83)). At the highest command the driver applies power for 99.6 % of each cycle. For example, with 9.6 V at the driver's motor supply, the two typical chip drops above give about 9.1 V and 9.5 V across the motor during a drive pulse. With 11.2 V at the driver, the motor could see about 11 V. Remo's motors can therefore receive more than their 9 V rating. The actual voltage depends on the pack under load and the motor current. It has not been measured on Remo.

## Other AA-sized cells

Each holder takes cells of the AA size: DiffBot's four, Remo's eight. Other cells of this size have different voltages:

- **Alkaline AA cells in Remo's holder:** They can't be recharged. A fresh alkaline cell has about 1.6 V without load ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)). Eight fresh cells give about 12.8 V without load. This exceeds the FeatherWing guide's 12 V supply recommendation.
- **NiMH AA cells in DiffBot's holder:** Four NiMH cells give 4.8 V nominal, below the 6 V that Seeed specifies for the Grove I2C Motor Driver.
- **14500 lithium-ion cells:** A 14500 cell with 3.6 V or 3.7 V nominal has 4.2 V when fully charged ([Nitecore's charger manual](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), section Default Settings). Four of them give 16.8 V, above the 15 V of the Grove I2C Motor Driver, and eight give 33.6 V, far above the 13.5 V of the FeatherWing.

## Why Remo uses NiMH cells

- **Voltage:** At the three example pack voltages in the NiMH table, Remo's battery is within the FeatherWing's input range.
- **Steady voltage:** Their voltage stays near 1.2 V for most of the discharge.
- **High currents:** The higher the current, the longer NiMH cells last compared with alkaline cells. Energizer's comparison gives a NiMH cell more than twice the capacity of an alkaline cell at 1000 mA, and about 1.2 times at 250 mA (Figure 7 in the [Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf)).
- **Recharging:** They can be charged again many times, so you don't need new batteries after each run.

Remo uses Amazon Basics NiMH AA cells with 2000 mAh and 1.2 V, sold as an 8-pack ([Amazon.com][amazon_nimh_aa_2000mah_8x_us], [Amazon.de][amazon_nimh_aa_2000mah_8x_ger], affiliate links).
One pack fills Remo's holder.

## Charging the NiMH cells

Charge Remo's NiMH cells outside the robot, in a [Nitecore UMS4](https://www.nitecore.com/product/ums4) ([Amazon.com][amazon_nitecore_ums4_us], [Amazon.de][amazon_nitecore_ums4_ger], affiliate links).
The charger has four slots, so the eight cells take two rounds of four.
The steps follow Nitecore's [user manual for the UMS4](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), whose English part comes first.
Read the manual's Precautions before the first charge.

1. Take the cells out of Remo's battery holder.
2. Connect the charger to a USB power adapter with its USB cable.
3. Insert four of the eight cells. Each slot has marks for the plus pole and the minus pole. Insert every cell the way the marks of its slot show. The charger starts by itself.
4. Check each slot on the display. A short press of the C button selects the next slot, and short presses of the V button step through the slot's values, among them Battery Health.
    - **`EE EE`:** The cell is inserted the wrong way round or has a short circuit. Take it out and check it. If it was only the wrong way round, insert it correctly, and the checks of this slot start again. If `EE EE` comes back, the cell is faulty.
    - **Battery Health `Poor`:** The cell's internal resistance is above 250 mΩ, and the manual says that the cell should be replaced (Automatic Battery Internal Resistance Detection).
5. Wait until the display shows for each slot that its charge has ended. The charger ends the charge of each slot on its own, so one cell can still be charging when the others are full. Check every slot before you take the cells out. If charging in one slot exceeds 20 hours, the UMS4 stops that slot and shows `END` below `CHG. STATUS` (the manual's Overtime Charging Protection). Treat the cell's state of charge as unknown and leave it out of the pack.
6. Take out the four cells and repeat steps 3 to 5 with the other four.
7. Unplug the charger and take out the cells when the second round is done. Don't leave the charger unattended while it is connected (precautions 8 and 19 of the manual).
8. Put the cells back into Remo's holder only if all of them have finished their charge and none is faulty or showed `Poor` or `END`. Keep the eight cells together as one set, and don't mix them with older cells or other types of batteries. Energizer warns that mixing can make cells rupture or leak ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings). So if a cell fails, replace the whole set, not only that cell.

!!! warning "These steps are for Remo's NiMH cells only"
    - **Never charge alkaline AA batteries**, or any other battery that isn't marked as rechargeable. Recharging a battery that isn't rechargeable can make it rupture or leak (Alkaline Handbook, section Testing / Care / Warnings). Nitecore's manual forbids such batteries in the charger as well (precautions 1 and 10).
    - **Other rechargeable cells** can need settings that this page doesn't describe. Follow Nitecore's manual for them.

### Why NiMH cells need a charger that is made for them

Overcharging shortens the life of a NiMH cell.
So a charger has to notice when a cell is full, and then stop.
Energizer recommends a smart charger for this, one that protects the cells from overcharge ([Nickel Metal Hydride Handbook](https://energizer.com/eu/austria/wp-content/uploads/2024/05/nickelmetalhydride_appman.pdf), pages 10 and 11).
That's also why Energizer warns against chargers that aren't designed for the battery type ([Alkaline Handbook](https://data.energizer.com/pdfs/alkaline_appman.pdf), section Testing / Care / Warnings).

For Remo's NiMH cells, the UMS4 does the following:

- **Charging mode:** For NiMH cells, it selects the charging mode by itself ([manual](https://www.nitecore.com/storage/images/album/uploads/FLASHLIGHTS/download/ums4.pdf), the note under Default Settings).
- **End of the charge:** Nitecore lists −dV/dt as one of the charger's modes ([specifications](https://www.nitecore.com/product/ums4)). This method ends a charge when the cell voltage starts to fall. A NiMH cell's voltage peaks when the cell is full and then falls slightly (Energizer's handbook above, page 10).
- **Charging current:** With all four slots in use, each slot charges with up to 500 mA from a standard USB power adapter, or up to 1000 mA from a Quick Charge 2.0 adapter ([specifications](https://www.nitecore.com/product/ums4)). At 500 mA, an empty 2000 mAh cell takes more than four hours.
- **USB port:** Check the charger's input connector and use the matching USB-C or micro-USB cable.
