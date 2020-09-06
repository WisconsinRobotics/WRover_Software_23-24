# ROS Abstraction Overview

This document serves as an overview of the ROS-level abstractions that govern the software system at large.

## Four Layers of Abstraction

The software system is partitioned into four layers of abstraction.
The layers are arranged as a stack, where the topmost layer is farthest from the robot hardware and the bottommost layer is closest.
Between each pair of adjacent layers is an abstraction barrier, through which no information should pass except via inter-layer topics.

Each topic in the system is either an inter-layer topic, which lies on an abstraction barrier, or an intra-layer topic, which allows for communication between two nodes on the same layer.
Inter-layer topics should be unidirectional; one of the layers should only read from the topic and the other layer should only write to it.
The same restrictions apply to services and actions, where applicable.

Each node in the system belongs to exactly one abstraction layer and should only interface with intra-layer topics in that layer or with inter-layer topics on adjacent abstraction barriers.
Similar restrictions apply to interfacing with services and actions, where applicable.

The four layers are as follows:

* **Human-Computer Interaction (HCI)** -- This is the highest level of abstraction. The nodes here handle interaction with human operators, including HUDs, input devices, and such. This layer is also unique in that all code here will be running on the driver station rather than on the robot. Information is received from and operator commands are sent to the logic layer.

* **High-Level Logic** -- This is where the majority of robot logic lives. The nodes here receive commands from the HCI layer and state information from the control systems layer to make decisions about what the robot should be doing; this includes both teleop control logic and autonomous decision-making logic. These high-level commands are then relayed to the control systems layer. Some nodes here may run on the driver station, such as nodes that interpret operator inputs and produce robot commands.

* **Control Systems** -- This is where high-level robot commands are translated to low-level hardware instructions. For example, a drive system control node might receive a "drive forwards" command and translate it into two motor controller commands for the drive system motors. These low-level instructions are then relayed to the HSI layer. All nodes here should run on the robot.

* **Hardware-Software Interface (HSI)** -- This is the lowest level of abstraction. Nodes here represent individual hardware devices and translate the messages they receive directly into hardware-level commands that are sent straight to the hardware devices. All nodes here necessarily run on the robot, since they need to be able to interface with robot hardware.

Each abstraction barrier should have the inter-layer topics crossing it documented in an appropriate description file in the `/docs/topics` directory of this repository.

## Namespace Organization

While the robot system is online, the namespacing of nodes and topics should reflect the architecture of the abstractions described here.
In particular, there are four root namespaces which correspond to each abstraction layer:

* Human-Computer Interface: `/hci`
* High-Level Logic: `/logic`
* Control Systems: `/control`
* Hardware-Software Interface: `/hsi`

Under each root namespace should be a set of namespaces representing particular robot subsystems.
For instance, the AI subsystem might be contained in the namespace `/logic/ai`.
All organization below this level is left to the discretion of the implementer; it is encouraged for implementers to spend some time designing a good organizational structure.

Inter-layer topics should be placed directly under the root namespace that is on the **receiving** end.
For instance, a topic relaying drive commands between the high-level logic and control systems layers might be called `/control/drive_cmd`.

The organization of services and actions should follow the same rules as topics, where applicable.
Parameters should be confined to a node's local namespace if possible; otherwise, there are no strong rules concerning the organization of parameters.
