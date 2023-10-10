# Wisconsin Robotics -- 2021 Software System

## Setting Up the Software System

To set up the software system on a base station or dev machine, see [`/docs/setup_dev.md`](@ref wr_system_setup_dev).
To set up the software system on a rover, see [`docs/setup_rover.md`](@ref wr_system_setup_wrover).

## Booting the Full Robot System

To launch the rover, you'll first want to start the base station radio and ensure that both the rover and base station are connected to it.
Then, you can start the launcher UI on the base station, which is opened with the command:

```sh
$ ./launch.sh
```

You should see a small window that looks like this:

![](docs/launcher_ui.png)

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
