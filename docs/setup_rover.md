# Setup: WRover

This document describes the setup process for the rover.

## Hardware Configuration

If the hardware is already configured (i.e. the rover is already put together), you can skip this section.

To begin with, you'll want to procure a Mars rover.
The rover should have roughly the following hardware subsystems:

* A Jetson Nano running Ubuntu 20.04
* A tank drive train, with both sides controlled by a single Basicmicro RoboClaw
* A central camera mast with an RSTP-compatible IP camera
* A GPS unit (TODO: get exact gps model)
* An inertial measurement unit (TODO: get exact imu model)
* A radio system (e.g. an antenna) for communicating with the base station radio
* A swappable module for the robotic arm, which includes:
  * A 6-DOF robotic arm driven by continuous motors controled by RoboClaws
    * (TODO: describe the exact motor/encoder configurations on the arm)
  * Two IP cameras pointed at the end effector
  * A mechanical gripper driven by a linear actuator
* A swappable module for autonomous navigation, which includes:
  * A SICK LIDAR unit (TODO: get exact lidar model)
* A swappable module for the planetary science analysis suite, which includes:
  * (TODO: describe science module)

## Preparing the Jetson Nano

### Flashing Ubuntu 20.04

TODO: instructions on flashing ubuntu 20.04

### Setting Up Python 3

TODO: instructions on installing python 3 + pip + virtualenv

### Installing ROS

TODO: instructions on installing ros

### Setting Up the SSH Server

TODO: instructions on installing sshd

### Setting Up mDNS/zeroconf

The software system uses mDNS/zeroconf to find the rover on the network.
To allow for this, you'll need to install an mDNS service on the rover which responds to name resolution requests.
Ubuntu should already ship with the Avahi daemon, a popular mDNS implementation, but if not, you can install it with:

```sh
$ sudo apt install avahi-daemon
```

Now, you'll need to do is update the rover's mDNS hostname to `wrover-nano` in the Avahi configuration file `/etc/avahi/avahi-daemon.conf`:

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

### Setting Up IP Cameras

TODO: instructions on setting hostname mappings for ip cameras

## Getting the WRover Code

You'll want to start by cloning this repository to `/home/wiscrobo/catkin_ws/WRover21_Software`.
Next, navigate into the repository and run the commands:

```sh
$ ./assemble.py init  # initialize the catkin workspace + venv
$ source setup.sh     # acquire the workspace environment
$ ./assemble.py build # build dependencies + workspace
```
