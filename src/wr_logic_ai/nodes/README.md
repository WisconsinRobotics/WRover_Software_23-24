# Longrange AI

@defgroup wr_logic_ai_longrange_ai Longrange Navigation
@ingroup wr_logic_ai
@brief Longrange Navigation Code

This document describes the existing code and logic for long range navigation, which is primarily used in the autonomous navigation mission of URC. 

## Long Range Navigation
Long range navigation is primarily driven by a GPS sensor (which provides us the coordinate of the rover) and an IMU sensor (which provides us the heading of the rover). By listing the provided GNSS coordinates in the `coordinates.json` file, the rover would visit those locations in that specific order. The rover does so by finding the difference in coordinates and calculating a target heading using the difference.

## Obstacle Avoidance Logic
Obstacle avoidance makes up most of the long range navigation code. During autonomous navigation, it's possible that the rover will encounter obstacles such as rocks, steep slopes, or other man-made structures. Thus, it is crucial that the rover is capable of navigating around these obstacles.

Obstacle avoidance is implemented using the rover's LiDAR sensor, which scans the landscape in front of the robot with a vision range of 180 degrees and an effective distance of 12 meters. The LiDAR data is read as an array, where the number of indexes is the LiDAR's angular resolution, and the value stored at an index represents the distance reading in that direction. 

Whenever the LiDAR detects an object within a threshold distance, it first calculates a window around that object that must be cleared in order for the robot to drive around the obstacle without collision. After all obstacle windows are calculated, obstacle avoidance code chooses a heading that is closest to the target heading while not intersecting with an obstacle window, and drives in that heading. 
