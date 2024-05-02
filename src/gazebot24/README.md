# GAZEBOT24

Gazebot24 is a package focused around building an simulation testing environment. However, due to various incompatatbilites between our ros version and gazebo, the project never came to fruition.

Thus, this document is mainly to record what has been done for the sake of anyone trying to continue the project.

## What Works

The folder contains a urdf model of the rover, and the related launch files to display said urdf in rvis. The launch command is as shown.

```
roslaunch gazebot24 display.launch
```

## The issue 

Gazebo uses a differnt format for their robots (sdf), and converting urdf into sdf destroys the joins and their heirarchy.

You can launch gazebo and see the problem by running the following command.

```
roslaunch gazebot24 gazebo.launch
```


This seems to be an issue with how urdf internally gets converted to sdf.

```
gz sdf -p src/gazebot24/config/robot.urdf
```

As you can see in the sdf, all the parts that would have been held by joints snap to the origin.


## Potential Solutions

I was in the process of looking into Webots as an alternative to gazebo, but due to time contraints I was unable to properly explore it.

Here are some links that I was looking into.

https://www.cyberbotics.com/doc/guide/tutorial-9-using-ros

https://cyberbotics.com/doc/guide/using-ros

https://github.com/cyberbotics/webots_ros/tree/master

https://github.com/ros-simulation/gazebo_ros_demos/tree/kinetic-devel

https://cyberbotics.com/doc/guide/tutorial-7-your-first-proto
