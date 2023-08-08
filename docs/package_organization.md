# Package Organization

@defgroup wr_system WRover System

This document describes the way ROS packages should be organized in this repository.
You should read `ros_abstraction_overview.md` first, if you haven't already.

There are three types of ROS packages that we will consider:

* **Subsystem packages** -- These packages represent one specific robot subsystem and any nodes related to that subsystem. Most nodes will live in subsystem packages.
* **Global packages** -- These packages provide some kind of data (e.g. message definitions or launch files) that is independent of any specific robot subsystem.
* **Dependency packages** -- These are packages that have been downloaded and built by the build script or included as submodules. These are generally less organizationally constrained, but are treated as global packages otherwise.

Within each package, the [standard ROS conventions](http://wiki.ros.org/Packages) should be followed.

## Subsystem Packages

Subsystem packages are the most strictly-constrained packages, since they are where most robot behaviour lives.
Each subsystem package is associated with a specific abstraction layer and should only ship nodes that lie in that abstraction layer.
Furthermore, subsystem packages should not provide any message definitions that are used by other packages; only internally-used messages are permitted in subsystem packages.

These packages should be named as follows:

```
wr_[layer]_[description]
```

Here, the `layer` should be the abstraction layer for the subsystem and `description` should be a short unique identifier for the package.
The `layer` should be one of "hci", "logic", "control", or "hsi".
As an example, an HCI package where the code for the drive station HUD lives might be called `wr_hci_hud`.

## Global Packages

Global packages are packages that provide data that's used by many other packages.
This might include inter-package message definitions, launch files, library code, and so on.
The only restriction on global packages is that they may not contain any data that's subsystem-specific; such data should lie in a subsystem package.

These packages should be named as follows:

```
wr_[description]
```

Here, the `description` should be a short unique identifier for the package and should not begin with one of the abstraction layer names to ensure that it does not clash with a subsystem package name.
Furthermore, if the package's primary function is to provide message definitions, it should end in `_msgs`.
Similarly, service definition packages should end with `_srvs` and action definitions with `_acts`.
As an example, a global package defining messages related to controller inputs might be called `wr_controller_msgs`.

## Dependency Packages

Dependency packages are packages pulled by the build script as dependencies.
Such packages should be excluded from the repository using the `.gitignore`.
There are no strong naming or organizational restrictions on dependency packages, since they're externally managed.
However, if the dependency package is one under the control of Wisconsin Robotics, it is recommended that its name be prefixed with `wr_`.
