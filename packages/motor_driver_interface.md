# Motor Driver Interface Package

The `motor_driver_interface` package is meant to provide an interface between
the ROS Control `hardware_interface::RobotHW` that diffbot uses and generic motor drivers.

The function exposed by the interface is named `write()` and used to send pwm signals to a motor driver.
It has access to the angular target wheel velocities, provided from the ROS Control hardware interface of diffbot,
that is found in the `diffbot_base` package. In order to use this `motor_driver_interface` it is necessary to implement this
`write()` function for a specific motor driver by converting the target wheel velocities to (pwm) signals that 
operate the motor to reach the commanded target velocities.

Subscribers to the desired and measured velocity, from the wheel encoders, are already implemented in this interface.
Specifically the following functions are responsible to update these velocities:
 
- `target_velocity_callback()`: a callback that is called by the velocity subscriber subscribing to the desired left and right wheel rotational velocities. The wheel velocities are measured in rad/s and provided from the ROS Control hardware interface.
- `measured_velocity_callback()`: called when a new velocity message is received from the.

The `motor_driver_interface` comes with two separate PID controllers, for each motor, that are optional and when enabled, 
output commands for each motor. The output value of each PID is calculated by the error between the target and measured velocities. 

!!! note
    Note that the `diffbot_base` package has its own "high level" PID controller that provides the same functionality and can be use instead of the "low level" PID available in this package. It is recommended to use only one of these PID controllers but not both. In case you make use of the low level PID of the `motor_driver_interface` it is recommended to disable the high level PID of the `diffbot_base` package.



## Arduino IDE





Library is already installed: Adafruit ILI9341:1.5.6
Library is already installed: Adafruit GFX Library:1.10.6
Library is already installed: Adafruit BusIO:1.7.2
Library is already installed: Adafruit STMPE610:1.1.2
Library is already installed: Adafruit TouchScreen:1.1.1
Library is already installed: Servo:1.1.6




# References

- https://www.arduino.cc/reference/en/libraries/servo/
- https://www.arduino.cc/en/Tutorial/Knob