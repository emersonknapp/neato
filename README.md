# neato_description

This repository contains the following packages:

## neato_description

This contains the URDF description of a Neato D-series robot, including Gazebo configuration and visual models.

Render `neato.urdf.xacro` using the `xacro` command - this is the top-level entrypoint for the description.

## neato_gazebo

Provides Gazebo utilities for simulating the Neato robot:

### Controller Plugin

The `neato_gazebo_diff_drive` plugin takes a `NeatoWheelCommand` message and translates it to wheel motor velocities.

### Keyboard Teleoperation Tool

```
ros2 run neato_gazebo neato_teleop_keyboard.py
```

This utility will let you sent `NeatoWheelCommand` messages via a simple CLI keyboard interface - useful for debugging robot behavior without a navigation controller.

## neato_msgs

Provides the `NeatoWheelCommand` message used to control the robot.
