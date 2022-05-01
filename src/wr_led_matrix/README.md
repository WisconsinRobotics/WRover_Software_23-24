# Changes
## Root Level Changes
* `setup.py` - This is a boilerplate file so ROS can find the Python files to run the node (copied from other Python nodes)
* Other Folders - See below (just adding standard organization)
* `CMakeLists.txt` - There's no C++ code to build, but this file contains requirements for generating the template for the custom service (see [srv](##srv) and [this](http://wiki.ros.org/catkin/CMakeLists.txt#msgs_srvs_actions) for setting up the service generation in this file; the req'd code should be in there, but commented out)
* `package.xml` - Add dependencies to service generation, the service runtime, and `rospy`.

## `src`
* This is where the main code of the node will go.  The implementation will be similar to (and may even be partially copied from) `flashy.py`, but will also include ROS hooks to run on ROS messages
* See the sample file for implementation details

## `srv`
* Due to the stateful nature of the lights and the fact that we just want to update them (semantically), this task should be implemented as a ROS service (call and response architecture)
* This folder will contain the service definition, since no sample service has the right data.  Follow the guide [here](http://wiki.ros.org/srv) and create the sample service file with 3 8-bit numbers for colors and (optional) response (not really needed, you can decide).

