# Shortrange AI

@defgroup wr_shortrange_ai Shortrange Navigation
@ingroup wr_logic_ai
@brief Shortrange Navigation code

## State Machine

![](ShortrangeStateMachine.png)

The [shortrange state machine](@ref wr_shortrange_ai_state_machine) controls the execution logic of shortrange navigation.
It is implemented using an ActionServer.
ActionServers provide a way for long running services to provide periodic feedback and be preempted.

See http://wiki.ros.org/actionlib for more details.

## Vision Navigation

### Target Detection

ArUco tag detection is implemented using OpenCV.
The [vision_target_detection node](@ref wr_shortrange_ai_vision_node) publishes the ID of a ArUco tag, the x-position of the tag, the distance from the tag, and whether the message is valid or not.

### Navigation

[Vision navigation](@ref wr_shortrange_ai_navigation) drives to an ArUco tag based on data from target detection.
It is implemented as a state of the shortrange state machine.

## Goals for URC 2024

Implement a search pattern with obstacle avoidance to find ArUco tags and vision detection objects.

Implement algorithms for object detection for the URC2024 Autonomous Navigation Challenge.
