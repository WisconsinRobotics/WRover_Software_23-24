# Longrange AI

@defgroup wr_logic_ai_state_machine_ai State Machine
@ingroup wr_logic_ai


# Navigation State Machine README

This README provides an overview of the code contained in the `state_machine.py` script for the autonomous navigation state machine used in a robotics application. The script is designed to control the navigation of a robot through various states and events, coordinating the robot's actions and responses to different scenarios.

## Table of Contents
- [Introduction](#introduction)
- [State Machine Overview](#state-machine-overview)
- [Dependencies](#dependencies)
- [Code Structure](#code-structure)
- [State and Event Descriptions](#state-and-event-descriptions)
- [Initialization and ROS Setup](#initialization-and-ros-setup)
- [State Descriptions](#state-descriptions)
- [Running the Navigation State Machine](#running-the-navigation-state-machine)
- [License](#license)

## Introduction

The `state_machine.py` script is part of a larger robotics system, and its primary purpose is to manage the navigation of a robot through different states, such as long-range navigation, short-range navigation, and recovery from errors. This script is responsible for coordinating the robot's actions, transitioning between states, and handling various events, such as successful task completion and error handling.

## State Machine Overview

The state machine operates based on states and events, where states represent what the robot is currently doing, and events trigger state transitions. The robot's navigation logic is driven by this state machine.

![Navigation State Machine Diagram](NavigationStateMachine.png)

## Dependencies

This code has several dependencies that are required to run successfully. These dependencies include:
- `statemachine`: The `statemachine` library is used to create and manage the state machine.
- `wr_logic_ai.coordinate_manager`: This module handles the retrieval of target waypoint GPS coordinates.
- ROS (Robot Operating System): This code is integrated with ROS for communication and control of various robot functions.
- Other ROS packages and services: This code interacts with other ROS nodes, services, and action servers to execute navigation tasks and receive feedback.

## Code Structure

The code is structured into several sections, including state and event definitions, functions to handle state transitions, and ROS setup. Here's a breakdown of the code's structure:

## State and Event Descriptions

The state machine defines the following states and events:

### States
1. `stInit`: The initial state where the robot sets up and initializes necessary components.
2. `stLongRange`: Represents the state where the robot is running in long-range navigation mode.
3. `stLongRangeRecovery`: State for recovery from an error in the long-range navigation mode.
4. `stShortRange`: Represents the state where the robot is running in short-range navigation mode.
5. `stWaypointSuccess`: The state indicating that the robot has successfully completed a task at a waypoint.
6. `stComplete`: The state indicating that all autonomous navigation tasks are complete.

### Events
1. `evSuccess`: Event for successful task execution, causing transitions between states.
2. `evError`: Event for handling error conditions during navigation.
3. `evNotWaiting`: An event that triggers an unconditional state transition from `stWaypointSuccess` to `stLongRange`.
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

## Running the Navigation State Machine

To run the navigation state machine, the script initializes a `NavStateMachine` instance and sets up ROS. The state machine is then started, and the script enters the ROS spin loop to listen for events and manage state transitions.

## License

This code may be subject to licensing terms and agreements. Ensure compliance with any applicable licenses when using or modifying this code for your own purposes.

Please note that this README provides an overview of the code's structure and purpose. For more in-depth details, consult the code and any relevant documentation or comments within the script.