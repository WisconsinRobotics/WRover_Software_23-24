# wr_logic_ai

@defgroup wr_logic_ai wr_logic_ai
@brief Package containing URC Autonomous Navigation mission logic

## Autonomous Navigation State Machine

![](NavigationStateMachine.png)

See [State Machine](@ref wr_logic_ai_state_machine_ai) for details.

To test state machine for development run:
```
$ roscore
$ export WROVER_LOCAL=true
$ export WROVER_HW=mock
$ roslaunch wr_logic_ai state_machine.launch
```

## Long Range Navigation

Long range navigation uses GPS and IMU sensors to navigate the rover to a given GPS coordinate.
See [Longrange](@ref wr_logic_ai_longrange_ai) for details.

## Short Range Navigation

Short range navigation uses a camera to find and navigate the rover to an ArUco tag.
See [Shortrange](@ref wr_shortrange_ai) for details.
