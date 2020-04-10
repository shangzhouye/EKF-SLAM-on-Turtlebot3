## Overview

This package includes the description of a differential driving two-wheel robot for simulation.

## How to use it

- Run `roslaunch nuturtle_description view_diff_drive.launch` to visualize the robot.
- Use `roslaunch nuturtle_description view_diff_drive.launch --ros-args` to check optional arguments.

## Files

- `/config/diff_params.yaml` provide a complete parametric description of a differential drive robot.
- `/urdf/diff_drive.urdf.xacro` describes the differential drive robot. It includes `/urdf/diff_drive_macro.xacro`.
- `/launch/view_diff_drive.launch` launches Rviz to visualize the robot.