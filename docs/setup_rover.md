# Setup: WRover

This document describes the setup process for the rover.

## Hardware Configuration

If the hardware is already configured (i.e. the rover is already put together), you can skip this section.

To begin with, you'll want to procure a Mars rover.
The rover should have roughly the following hardware subsystems:

* A Raspberry Pi running Ubuntu 20.04
* A tank drive train, driven by Falcon 500 motors.
* A central camera mast with a USB camera
* A GPS unit (TODO: get exact gps model)
* An inertial measurement unit (TODO: get exact imu model)
* A radio system (e.g. an antenna) for communicating with the base station radio
* A swappable module for the robotic arm, which includes:
  * A 6-DOF robotic arm driven by continuous motors controled by RoboClaws
    * (TODO: describe the exact motor/encoder configurations on the arm)
  * Two USB cameras pointed at the end effector
  * A mechanical gripper driven by a linear actuator
* A swappable module for autonomous navigation, which includes:
  * A LIDAR unit (TODO: get exact lidar model)
* A swappable module for the planetary science analysis suite, which includes:
  * (TODO: describe science module)

## Preparing the Raspberry Pi

### Flashing Ubuntu 20.04

You'll want to start by flashing the Raspberry Pi's SD card with Ubuntu 20.04 "Focal Fossa".

Canonical distributes server versions of Ubuntu 20.04, these can be deloyed to an SD card via the Raspberry Pi.

After re-inserting the SD card, you'll want to boot up the Jetson Nano and complete the first-time setup for Xubuntu, which can be done with an external monitor and keyboard.
Make sure you set the user account's name to `wiscrobo`!

### Setting Up Python 3

You'll need Python 3 to run the ROS-based robot software, as well as a couple tools for installing dependencies.
To install Python 3 and the `pip` package management tool, you can use the command:

```sh
$ sudo apt install python3 python3-pip
```

Then, you can use `pip` to install the `virtualenv` package:

```sh
$ python3 -m pip install virtualenv
```

### Installing ROS

You'll now want to install ROS Noetic itself.
Follow the [installation instructions on the ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) to do this.
It is recommended that you install the `ros-noetic-ros-base` ("bare-bones") meta-package only, as the rover doesn't necessarily need the GUI tools included in `ros-noetic-desktop`.

### Setting Up the SSH Server

For the base station (and any other remote machine) to interact with the rover, you'll need to install an SSH server.
The recommended implementation is the OpenSSH server daemon `sshd`, which can be installed with the command:

```sh
$ sudo apt install openssh-server
```

Now, you'll want to modify the configuration to allow for public key authentication.
Open the file `/etc/ssh/sshd_config` in your text editor of choice and ensure that the following line is uncommented:

```ssh-config
PubkeyAuthentication yes
```

Then, to start up the server, you can enable the SSH service with the commands:

```sh
$ sudo systemctl enable ssh
$ sudo systemctl start ssh
```

You could now test to see if it's possible to SSH to the rover from a remote machine.
You'll also have to configure public key authentication for any base station machines; see `setup_dev.md` for more details on this.

### Setting Up mDNS/zeroconf

This section does not work reliably in practice.  Consider the time cost of setting this up/debugging, and don't fall for the sunk-cost fallacy.

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

Use GStreamer/similar to convert the USB cameras to RTSP streams.  Then, use the `simple-rtsp-server` program found online to act as a transmission manager for the RTSP streams and serve cameras to the base station.

The cameras typically involve a lot of tuning to get optimal streaming characteristics.  The CPU load incurred by the cameras may cause the team to use a separate camera co-processor to alliviate load from the main CPU.  Good camera characteristics would probably be:

* Low-latency
* High quality

These needn't be achieved at the same time, but if they aren't, the swap should be quick.

## Getting the WRover Code

You'll want to start by cloning this repository to `/home/wiscrobo/catkin_ws/WRover21_Software`.
Next, navigate into the repository and run the commands:

```sh
$ ./assemble.py init  # initialize the catkin workspace + venv
$ source setup.sh     # acquire the workspace environment
$ ./assemble.py build # build dependencies + workspace
```

Then, continue setting up the WRover environment according to [the Developer Setup Instructions](setup_dev.md).  Keep in mind that the WRover is a shared machine, so be careful adding things like GitHub SSH keys.  You will have to locally build the code to get the executables and message definitions needed to run the code.
