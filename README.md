# Feature-based EKF SLAM on Turtlebot3 from Scratch

## Overview

This repository builds feature-based EKF SLAM on Turtlebot3 from scratch. The demo below shows the algorithm in action (2x speed).

![](https://github.com/shangzhouye/portfolio-website/blob/master/content/featured-projects/ekf_slam/side_by_side.gif?raw=true)

- Trajectories
  - The pink path shows the odometer estimated path.
  - The green path is the groundtruth.
  - Yello path is SLAM results.
- Landmarks
  - Blue landmarks are groundtruth.
  - Green landmarks are measurements.
  - Indigo landmarks show where the SLAM algorithm thinks their positions are.

The figure below shows the result of the landmark detection algorithm using a 2D laser scanner:

![](https://github.com/shangzhouye/EKF-SLAM-on-Turtlebot3/blob/public/figures/landmark_detection.gif?raw=true)

The system has the following major components:

- A **2D Lie Group** library for differential drive robots with complete unit testing
- A waypoint following **feedback controller**
- **Turtlebot3 URDF** built from scratch for Gazebo simulation
- **Gazebo plugins** to control the robot and return the groundtruth data for evaluation
- An **odometer** that estimates robot states based on encoder reading
- **Turtlebot3 interface** that controls the motors with given velocity command
- **Feature detection** algorithm that identifies landmarks using a 2D laser scanner
- **EKF SLAM** algorithm that estimates robots states

A detailed [description](https://shangzhouye.tech/featured-projects/ekf_slam/) can be found in my portfolio.

## File structure

### Packages

Six packages are in this repository.

- `nuturtle_description` develops Turtlebot3 URDF, and visualizes the wheeled robot in `Rviz`
- `nuturtle_gazebo` includes Gazebo plugins to simulate the robot in `Gazebo`
- `nuturtle_robot` implements the Turtlebot3 interface, and includes the test node for the odometer on the real robot
- `nuturtle_slam` includes the feature detection algorithm and the EKF SLAM algorithm
- `tsim` implements the waypoints following feedback controller
- `rigid2d` is the 2D Lie Group library, including SO(2), SE(2) calculations, the odometer, and the fake encoder

### Major launch files

- `nuturtle_description/launch/view_diff_drive.launch` launches the URDF model in `Rviz`
- `nuturtle_robot/launch/test_waypoint.launch` drives the robot through waypoints using the fake encoder with visualization in `Rviz`
  - Service `/start_waypoint` would start the movement
- `nuturtle_gazebo/launch/diff_drive_gazebo.launch` drives the robot through waypoints in `Gazebo` simulation
- `nuturtle_slam/launch/landmarks.launch` launches the landmark detection algorithm with visualization in `Rviz`
- `nuturtle_slam/launch/slam_in_control.launch` launches the actual SLAM algorithm
    - `Rviz` visualization is in `map` frame
    - Pink path shows the odometer estimated path.
    - Green path is the groundtruth.
    - Yello path is SLAM results.
    - Blue landmarks are groundtruth.
    - Green landmarks are measurements.
    - Indigo landmarks show where the SLAM algorithm thinks their positions are.

## Dependencies

- **ROS Melodic** ([Link](http://wiki.ros.org/melodic/Installation/Ubuntu))
- **Eigen** ([Official Site](http://eigen.tuxfamily.org/))

## Quick Start guide

- Install all the dependencies
- `fork` this repository, then clone the package using `wstool`
  - `rosinstall` file is included in the repository
- Build the package using `catkin_make`
- Use `roslaunch nuturtle_slam slam_in_control` to launch the SLAM algorithm

## Future work

- Data association is currently assumed to be known. SLAM with unknown data association can be achieved by calculating the Mahalanobis Distance