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

## Demo

Screenshot of the turtle:

Screenshot of rqt_plot:

Video link of the turtle following the trajectory:
