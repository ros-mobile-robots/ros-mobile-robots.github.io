# Hardware Setup Overview

You can build one of two robots:

- **DiffBot:** your own two- or four-wheeled differential drive robot, like the one in the [`diffbot_description`](https://github.com/ros-mobile-robots/diffbot/tree/noetic-devel/diffbot_description) package.
- **Remo:** a modular robot platform based on NVIDIA's JetBot, which you 3D print. Its robot description is the [Remo Description](../packages/remo_description.md) package. You need a 3D printer with a build volume of about 15×15×15 cm, or a local or online print service. The public `remo_description` repository only has empty placeholders for the STL files. You get them from the Gumroad download, see [3D Printing](3D_print.md), or from [Remo Insiders](../insiders/index.md#remo-stl-files).

The [Components](../components.md) page has the bill of materials and details about each part.

The following pages guide you on how to setup the hardware of your robot (either DiffBot or Remo).

- [**3D Printing**](3D_print.md) is only relevant for Remo robot (not DiffBot). 
  Here you will learn which parts to print and some suggestions to configure your slicer.
- [**Electronics**](electronics.md) provides instructions to connect the single board computer (e.g. Raspberry Pi), microcontroller (e.g. Teensy) as well as
  the other components such as the motor driver, motors and laser scanner.

    ??? info "Preview of connection schematic"
        The bread board view from [Fritzing](https://fritzing.org/) shows the connection schematic. Both models (DiffBot and Remo) use
        slgithly different hardware (e.g. motor driver) which you can see in the following:

        === "Remo"

            [![Remo Fritzing]][Remo Fritzing]

            [Remo Fritzing]: /fritzing/remo_architecture.svg
            

        === "DiffBot"

            [![DiffBot Fritzing]][DiffBot Fritzing]

            [DiffBot Fritzing]: /fritzing/diffbot_architecture.svg
        

- [**Assembly**](assembly.md) give instructions to assemble Remo robot.



