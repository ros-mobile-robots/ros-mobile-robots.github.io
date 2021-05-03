# Docker

## Docker Containers

A container includes an application and its dependencies. 
It's goal is to easily ship (deploy) and handle applications.

Like real world shipping containers, Docker containers wrap up an application in a filesystem containing
everything the application needs to run:

- source code
- runtime libraries
- system tools
- configruation files

The result is that a containerzied application will run identically on any host. 
And there are no incompatibilieties of any kind.

## Why Containerization

In the traditional operating systems like Linux Ubuntu a package manager install apps.
These apps run on shared runtime libraries. Therfore, applications are coupled,
because they share the same dependencies. This can can lead to **compatibility issues**,
when, for example, two or more applications requires a different version of the same shared library.

When using containerization, each application running in a container comes with its own needed set of libraries.
This way, each application container is isolated and can be updated independently.

## Difference between Containerization and Virtual Machines

Containers are different to virtual machines. In a virtual machine environment there
is a host system that runs a hypervisor (e.g. VMWare). The hypervisor divides the physical 
hardware resources among the virtual machines. Each virtual machine runs its own operating system.

The disadvantage of this is that there is a considerable overhead and processes cannot communicate 
over different virtual machines.

On the other hand, containerization has minimal overhead and it allow us to deploy multiple applications
so that they can communicate with each other.