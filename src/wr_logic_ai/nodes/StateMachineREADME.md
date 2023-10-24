# Longrange AI

@defgroup wr_logic_ai_state_machine_ai State Machine
@ingroup wr_logic_ai


## Introduction

This README provides an overview of the code contained in `state_machine.py`, which serves as a layer of abstraction between long range/short range logics and rover control. We use this state machine to control the rover's actions during autonomous navigation, switching between the two logics in different scenarios.


## State Machine Overview

The state machine operates based on states and events, where states represent what the robot is currently doing, and events trigger state transitions. The robot's navigation logic is driven by this state machine.

![Navigation State Machine Diagram](NavigationStateMachine.png)

## State and Event Descriptions

The state machine defines the following states and events:

### States
1. `stInit`: The initial state where the robot sets up and initializes necessary components.
2. `stLongRange`: Represents the state where the robot is running in long-range navigation mode using provided GPS coordinates
3. `stLongRangeRecovery`: State for recovery from an error using the long-range navigation mode. When an error occurs, this state would drive the robot to the previous long-range navigation waypoint, and the rover would reattempt the long-range navigation. 
4. `stShortRange`: Represents the state where the robot is running in short-range navigation mode.
5. `stWaypointSuccess`: The state indicating that the robot has successfully completed a task at a waypoint.
6. `stComplete`: The state indicating that all autonomous navigation tasks are complete.

### Events
1. `evSuccess`: Event for successful task execution, causing transitions between states.
2. `evError`: Event for handling error conditions during navigation.
3. `evNotWaiting`: An event that triggers when we have completed all tasks at a given waypoint and is ready to move to the next waypoint, triggering a transition from `stWaypointSuccess` to `stLongRange`. 
4. `evUnconditional`: An unconditional state transition from `stInit` to `stLongRange`.
5. `evComplete`: Event indicating that all navigation tasks are complete, leading to a transition to `stComplete`.

## Initialization and ROS Setup

The code initializes ROS, defines helper functions, sets up LED matrix colors, and prepares the navigation state machine for operation. The main components include:

- Helper functions for setting the LED matrix color to indicate the robot's state.
- Initialization of ROS nodes, publishers, and timers.
- Configuration of action clients for long-range and short-range navigation actions.
- Methods for handling state transitions and errors.

## State Descriptions

Each state has associated methods for handling actions upon entering and exiting the state. These methods control the execution of tasks and transitions between states. Some of the key state descriptions include:

- `on_enter_stInit`: Actions to be taken when entering the initial state, including setting up coordinates and running calibration.
- `on_exit_stInit`: Actions to be taken when exiting the initial state, such as checking for new coordinates.
- `on_enter_stLongRange`: Actions to be taken when entering the long-range navigation state, including setting the LED matrix color and sending navigation coordinates.
- `on_exit_stLongRange`: Actions to be taken when exiting the long-range navigation state.
- `on_enter_stLongRangeRecovery`: Actions to be taken when entering the long-range recovery state, including setting the LED matrix color and recovering from errors.
- `on_exit_stLongRangeRecovery`: Actions to be taken when exiting the long-range recovery state.
- `on_enter_stShortRange`: Actions to be taken when entering the short-range navigation state, including setting the LED matrix color and running short-range actions.
- `on_exit_stShortRange`: Actions to be taken when exiting the short-range navigation state.

Additional methods handle transitions, LED matrix color changes, and user input.