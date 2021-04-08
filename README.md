# Wisconsin Robotics -- 2021 Software System

## Prerequisites

You should have the following installed beforehand:

* ROS Noetic (preferrably on Ubuntu 20.04; it is recommended, but not necessarily required, to dual-boot for this)
* Python 3.8.x+ with pip and virtualenv

## Setting Up the Workspace

This repository consists of the ROS catkin workspace for the WRover software system.
You'll want to clone the repo to wherever you wish to store your workspaces.
Once you have a local copy, you'll want to open a terminal in the workspace.
For initial setup, do the following:

```sh
$ ./assemble.py init  # initialize the catkin workspace + venv
$ . setup.sh          # acquire the workspace environment
$ ./assemble.py build # build dependencies
```

Every time you want to work in the workspace, you'll have to run `source setup.sh` again to set up your shell with the workspace environment. Alternatively, you can do this automatically in your `.bashrc` if you don't anticipate working in any other ROS workspaces.

## Booting the Full Robot System

*(TODO)*

## Documentation

Documents describing the structure and organization of the software system can be found in the `/docs` subdirectory of this repository.
You should probably read through these before starting any substantial work.

There is no particular code style guide that we use, but you should try to maintain a consistent style for all the code you write. Additionally, you should make sure your code is readable and sensible; if there's anything that's confusing or unintuitive, use comments to clarify it for future maintainers.
