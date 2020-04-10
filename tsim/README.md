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