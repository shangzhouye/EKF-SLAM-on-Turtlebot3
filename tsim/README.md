# ME495 Sensing, Navigation, and Machine Learning
Author: Shangzhou Ye

## Overview

This package make the turtle in turtle simulation move in a rectangular trajectory.

## How to use it

- Use `roslaunch tsim trect.launch` to launch the simulation.
- The `plot_gui` (default to `true`) would also launch the error plot.

## Files

- `config/trect.yaml` includes the parameter of the rectangular trajectory.
- `msg/PoseError.msg` is the defined error message.
- `src/turtle_rect.cpp` is the node that moves the turtle.

## State Machine

The following figures illustrates the state machine coded in `src/turtle_rect.cpp`.

![state_machine](https://github.com/ME495-Navigation/main-assignment-shangzhouye/blob/master/figures/tsim_state_machine.png?raw=true "state_machine")

## Demo

Screenshot of the error for a full pentagon cycle:

![turtle_rqt_plot](https://github.com/ME495-Navigation/main-assignment-shangzhouye/blob/master/figures/turtle_way_rqt_plot.png?raw=true "turtle_rqt_plot")

Screenshot of the turtle:

![turtle](https://github.com/ME495-Navigation/main-assignment-shangzhouye/blob/master/figures/tsim_turtle.png?raw=true "turtle")

Screenshot of rqt_plot:

![rqt_plot](https://github.com/ME495-Navigation/main-assignment-shangzhouye/blob/master/figures/tsim_rqt_plot.png?raw=true "rqt_plot")

Video link of the turtle following the trajectory:

[Link](https://github.com/ME495-Navigation/main-assignment-shangzhouye/raw/master/figures/tsim_follow_traj.mkv)
