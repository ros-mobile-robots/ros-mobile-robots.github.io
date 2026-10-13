# Hardware

You can build one of two robots:

<div class="grid cards" markdown>

-   __DiffBot__

    ---

    ![DiffBot from the front, with its RPLidar, camera and ultrasonic sensor](../resources/diffbot/diffbot-front-900.jpg)

    Your own two- or four-wheeled differential drive robot, like the one in the [`diffbot_description`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_description) package.

-   __Remo__

    ---

    ![Remo with its 3D printed frame, RPLidar, Raspberry Pi and camera](../resources/remo/remo-front-side-900.jpg)

    A modular robot platform based on NVIDIA's JetBot, which you 3D print. Its robot description is the [Remo Description](../packages/remo_description.md) package. You need a 3D printer with a build volume of about 15×15×15 cm, or a local or online print service, and the STL files, see [3D Printing](../hardware_setup/3D_print.md).

</div>

The [Components](../components.md) page has the bill of materials. The sections after Build Remo in this menu describe the parts in detail: [Processing Units](../processing_units/index.md), actuators, sensors and the power supply.

The following pages guide you on how to setup the hardware of your robot (either DiffBot or Remo).

- [**3D Printing**](../hardware_setup/3D_print.md) is only relevant for Remo robot (not DiffBot). 
  Here you will learn which parts to print and some suggestions to configure your slicer.
- [**Electronics**](../hardware_setup/electronics.md) provides instructions to connect the single board computer (e.g. Raspberry Pi), microcontroller (e.g. Teensy) as well as
  the other components such as the motor driver, motors and laser scanner.

    ??? info "Preview of connection schematic"
        The bread board view from [Fritzing](https://fritzing.org/) shows the connection schematic. Both models (DiffBot and Remo) use
        slightly different hardware (e.g. motor driver) which you can see in the following:

        === "Remo"

            [![Remo Fritzing]][Remo Fritzing]

            [Remo Fritzing]: /fritzing/remo_architecture.svg
            

        === "DiffBot"

            [![DiffBot Fritzing]][DiffBot Fritzing]

            [DiffBot Fritzing]: /fritzing/diffbot_architecture.svg
        

- [**Assembly**](../hardware_setup/assembly.md) give instructions to assemble Remo robot.



