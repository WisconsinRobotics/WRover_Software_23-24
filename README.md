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

## Setting up the Rover

On the rover, you'll need to clone a copy of this repo to `/home/wiscrobo/catkin_ws/WRover21_Software`.
Once this is done, you'll want to set up the workspace as above using `assemble.py`.
This will prepare the ROS workspace for the rover to run ROS nodes in.

Additionally, you'll need to install an mDNS provider on the rover.
This allows the base station to discover the rover without the need for static IP addresses or bulky DNS servers.
If the rover is running a standard Ubuntu distribution, then it probably already comes with the Avahi daemon.
In that case, all you'll need to do is update the rover's hostname to `wrover-nano` in the Avahi configuration file `/etc/avahi/avahi-daemon.conf`:

```ini
[server]
host-name=wrover-nano
```

Once this is done, you can restart the Avahi daemon using the command:

```sh
$ sudo systemctl restart avahi-daemon
```

To make sure this worked correctly, you can try pinging the rover by hostname from the base station:

```sh
$ ping wrover-nano.local
```

*TODO: something about setting up keypair auth for ssh*

## Booting the Full Robot System

To launch the rover, you can use the launcher UI, which is opened with the command:

```sh
$ ./launch.sh
```

You should see a small window that looks like this:

![](launcher_ui.png)

From here, you can select a launch configuration from the drop-down box and press the "Launch!" button to launch the robot system.
To shut down the rover system, you can simply send an interrupt to the terminal window using `Ctrl`+`C`.

Alternatively, you can directly launch a full-system launch file from the `wr_entry_point` package.
There are the `auto_nav.launch`, `eq_service.launch`, `erdm.launch` and `science.launch` files, each of which configures the robot system for a specific URC task.
Additionally, several test configurations are available in launch files prefixed by `test_`, each of which allows for testing one robot subsystem in isolation.

To use these launch files, you'll first need to start `roscore` on the rover.
You'll also need to set certain environment variables which are described in the `README.md` document in the `wr_entry_point` package.
These env vars provide information about the environment of the launch to the robot system.
An example of a successful launch on real rover hardware might be:

```sh
$ ssh wiscrobo@wrover-nano.local '~/catkin_ws/WRover21_Software/env.sh roscore'
$ export ROS_MASTER_URI='http://wrover-nano.local:11311'
$ export ROS_IP='192.168.1.111'
$ export WROVER_LOCAL=false
$ export WROVER_HW=REAL
$ roslaunch wr_entry_point erdm.launch
```

## Documentation

Documents describing the structure and organization of the software system can be found in the `/docs` subdirectory of this repository.
You should probably read through these before starting any substantial work.

There is no particular code style guide that we use, but you should try to maintain a consistent style for all the code you write. Additionally, you should make sure your code is readable and sensible; if there's anything that's confusing or unintuitive, use comments to clarify it for future maintainers.
