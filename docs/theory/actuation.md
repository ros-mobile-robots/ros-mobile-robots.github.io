# DC Motors and Motor Drivers

Each wheel of DiffBot and Remo is driven by a brushed DC motor, the [DG01D-E](../DG01D-E-motor-with-encoder.md), through a motor driver.
Remo's driver is the [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927) with TB6612 chips, and DiffBot's is the [Grove I2C Motor Driver V1.3](../grove_motor_driver.md) with an L298.
This page explains how a motor driver sets the direction and the speed of a motor: with an H-bridge for the direction, and with pulse-width modulation (PWM) for the speed.
The [PID controller](../packages/diffbot_base/pid.md) of the firmware and the robots' [Power Supply](../power-supply.md) build on both.

## The brushed DC motor

A brushed DC motor turns when a voltage is applied to its two terminals.
Its speed is proportional to that voltage, and when the voltage is reversed, the current flows the other way through the winding and the motor turns in the other direction ([Microchip's application note AN905, Brushed DC Motor Fundamentals](https://ww1.microchip.com/downloads/en/AppNotes/00905B.pdf), Speed Control and the H-bridge section).
The DG01D-E is rated for 3 V to 9 V, and its [datasheet](https://cdn.sparkfun.com/assets/8/3/b/e/4/DS-16413-DG01D-E_Motor_with_Encoder.pdf) gives 90 RPM at 4.5 V.
It has a gearbox and an encoder, which the [motor's page](../DG01D-E-motor-with-encoder.md) describes.

So a motor driver has to do two things: switch the polarity of the voltage at the motor for the direction, and set the size of the voltage for the speed.

## The H-bridge

An H-bridge connects a motor to its supply in either direction.
It has four switches, Q1 to Q4, in two half-bridges, one on each side of the motor.
Each half-bridge connects its motor terminal either to the supply voltage +V or to ground (AN905, the H-bridge section).
Drawn as a circuit, the two half-bridges and the motor between them look like the letter H, which gives the circuit its name.

--8<-- "includes/actuation-h-bridge.html"

The two motor drivers have the same four modes, under their own names.
The TB6612 sets them with its inputs IN1, IN2 and PWM ([TB6612FNG datasheet](https://cdn.sparkfun.com/assets/0/1/b/b/3/TB6612FNG.pdf), H-SW Control Function), and the L298 with its inputs C, D and its enable input ([L298 datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/L298datasheet.pdf), Figure 6):

| Mode | Closed switches (AN905, Table 1) | TB6612 (Remo) | L298 (DiffBot) |
| --- | --- | --- | --- |
| Forward | Q1 and Q4 | CW | Forward |
| Reverse | Q2 and Q3 | CCW | Reverse |
| Coast | none | Stop, outputs off (high impedance) | Free running motor stop |
| Brake | Q2 and Q4 | Short brake | Fast motor stop |

The two switches of one half-bridge must never be closed together.
They would connect the supply directly to ground, and the short circuit can destroy the switches (AN905, the H-bridge section).
The TB6612 prevents this itself: when it changes from one mode to another, it inserts a dead time in which the switches it turns off are already off before the others turn on (TB6612FNG datasheet, H-SW Operating Description).

## PWM and the duty cycle

To set the speed, the motor driver doesn't make a lower voltage.
It switches the full supply voltage on and off many times per second, which is called pulse-width modulation (PWM).
The time of one on-and-off cycle is the period \(T\), and the duty cycle \(D\) is the share of the period during which the motor is connected to the supply:

$$
D = \frac{t_\text{on}}{T}
$$

The motor's winding smooths the pulses, so at a high enough frequency the current in the winding is steady (AN905, Speed Control).
The motor then behaves as with the average voltage \(V_\text{average}\), which is the duty cycle times the supply voltage \(V_\text{supply}\) (AN905, Equation 1):

$$
V_\text{average} = D \cdot V_\text{supply}
$$

--8<-- "includes/actuation-pwm.html"

On Remo, the firmware's PID controller computes a command from −255 to 255 for each wheel.
[`AdafruitMotorController::setSpeed()`](https://github.com/ros-mobile-robots/diffbot/blob/101f179a50afa92c625eb9f72dcd5443c9cc49e2/diffbot_firmware/lib/motor_controllers/adafruit_feather_wing/adafruit_feather_wing.cpp#L16-L36) turns the sign of the command into the bridge's mode and passes the size of the command on as the speed:

```cpp
void diffbot::AdafruitMotorController::setSpeed(int value)
{
    if (value > 0)
    {
        pMotor_->run(FORWARD);
    }
    else if (value < 0)
    {
        pMotor_->run(BACKWARD);
        // AdafruAdafruit_MotorShield requires uint8 values from 0 to 255
        // Make sure to convert the negative value to a positive one
        value = value * -1;
    }
    else // zero speed
    {
        // Cut power to the motor
        pMotor_->run(RELEASE);
    }

    pMotor_->setSpeed(value);
}
```

Adafruit's library does the rest ([`Adafruit_MotorShield.cpp`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/1.1.4/Adafruit_MotorShield.cpp#L231-L256), version 1.1.4):

- **Direction:** `run(FORWARD)` sets the TB6612's input IN1 high and IN2 low, its mode CW. `run(BACKWARD)` sets IN1 low and IN2 high, CCW.
- **Stop:** A command of 0 calls `run(RELEASE)`, which sets IN1 and IN2 low. The TB6612 then switches its outputs off, and the motor coasts. The firmware's [command timeout](../packages/diffbot_base/low-level.md#the-command-timeout) stops the motors this way too.
- **Speed:** `setSpeed()` sets the PWM input to 16 counts per step of the command, of 4096 counts per period. The command 128, for example, gives 2048 of 4096 counts, a duty cycle of 50 %.
- **Frequency:** The FeatherWing makes the PWM with a PCA9685 chip ([Adafruit's guide, Pinouts](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/pinouts)). The firmware asks for 1600 Hz, the default of [`AdafruitMotorController::begin()`](https://github.com/ros-mobile-robots/diffbot/blob/101f179a50afa92c625eb9f72dcd5443c9cc49e2/diffbot_firmware/lib/motor_controllers/adafruit_feather_wing/adafruit_feather_wing.h#L49). The library lowers the request by 10 % and rounds the chip's prescaler to 3 ([`Adafruit_MS_PWMServoDriver::setPWMFreq()`](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/1.1.4/utility/Adafruit_MS_PWMServoDriver.cpp#L38-L57)). The PCA9685 divides its 25 MHz oscillator by 4096 times the prescaler plus 1, so it runs at its maximum of about 1526 Hz ([PCA9685 datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf), section 7.3.5), and a period lasts about 0.66 ms.
- **Between the pulses:** While the PWM input is low, the TB6612's table gives short brake for both directions, so the motor's terminals are connected to ground until the next pulse.

The highest command, 255, gives 4080 of 4096 counts, a duty cycle of 99.6 %.

## The voltage that the driver loses

Real switches aren't ideal: each closed switch loses some voltage, and the motor gets the supply voltage minus these drops.
How much depends on the kind of transistor:

- **L298 (DiffBot):** Its switches are bipolar transistors. Its [datasheet](https://files.seeedstudio.com/wiki/Grove-I2C_Motor_Driver_V1.3/res/L298datasheet.pdf) gives 1.8 V to 3.2 V for the two transistors of a bridge together at 1 A.
- **TB6612 (Remo):** Its switches are MOSFETs, which lose much less. Its [datasheet](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660) gives 0.5 V (typical) at 1 A for the two switches of a bridge together.

The [Power Supply](../power-supply.md) page uses these drops with the robots' batteries and shows what the motors of each robot get.

## Gearbox

Gearboxes can be used to fine-tune the performance characteristics of a motion axis. 
They're most commonly considered torque multipliers but actually serve several other functions, 
including speed matching, inertia reduction, and resolution increase[ref](https://www.motioncontrolonline.org/content-detail.cfm/Motion-Control-Technical-Features/How-to-Specify-Gearboxes-for-Motion-Control-Systems/content_id/3256).

<iframe title="vimeo-player" data-consent-src="https://player.vimeo.com/video/219688810" width="640" height="360" frameborder="0" allowfullscreen></iframe>
