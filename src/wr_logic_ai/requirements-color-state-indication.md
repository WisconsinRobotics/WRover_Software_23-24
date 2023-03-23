# Color-State Autonomous Indicator
## Description
During the autonomous navigation mission, we need to display a color on the robot to indicate what the robot is "thinking" (i.e. navigation state) according to the following table:

| COLOR | INTERPRETATION |
|---|---|
| BLUE | Autonomous Navigation (between waypoints) |
| GREEN | Waypoint Complete |
| RED | Error encountered |

The system will have to publish internal messages representing the current state.  The goal of the new node is to convert this message to a color publication to the node that actually writes the color to the LED display.

## Input
* `/navigation_state`
  * A topic that contains the current navigation state
  * Type: `NavigationState` (**NEW**)
    * Recommended Layout:
      * `status` (int) - Contains an integer representation of the navigation status
        * `NAVIGATION_STATE_AUTONOMOUS` - A constant integer
        * `NAVIGATION_STATE_SUCCESS` - A constant integer
        * `NAVIGATION_STATE_ERROR` - A constant integer

## Output
* `/navigation_color`
  * A service that can set the color of the LED panel
  * Type: `led_matrix`
    * `RED` (`uint8`)
    * `GREEN` (`uint8`)
    * `BLUE` (`uint8`)

## Implementation Notes
This node can probably be done in Python.  This node probably does not have to constantly call the service output, even though it will constantly receive inputs (maybe only on changes?).  The `NavigationState` is a new message type that needs to be created and generated in the `CMakeLists.txt` file.

### Steps
1. Make a simple Python node.  It doesn't have to do anything, just set up a node and call `rospy.spin()`
2. Add a Subscriber to the `/navigation_state` topic (and define the type).  Make a callback (ie. a function and 3rd argument in subscriber) that prints received messages and use `rostopic pub` to publish sample data to see it print out correctly.
3. Edit the callback so it only prints out a value when the subscribed message changes.
4. Add a Service call to the `/navigation_color` topic from the subscriber callback.  It won't work right now, but we will set up the responding node later. (Check [ROS tutorial](https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) for more info on Service, or ask).
5. Reach out to learn about the `wr_led_matrix` node and learn how to start it from a launch file.
