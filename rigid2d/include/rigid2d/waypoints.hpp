#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for Waypoints class

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <iostream>
#include <vector>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

namespace rigid2d
{

enum CurrentState
{
    Rot_l,
    Rot_r,
    Trans
};

class Waypoints
{
    // learned: to write a class
    // - list the members and functions needed
    // - write the pipeline function together with methods
public:
    Waypoints() : waypoints_(std::vector<Vector2D>{Vector2D(3, 2), Vector2D(7, 2), Vector2D(7, 7), Vector2D(3, 7)}),
                  state_(Trans), current_waypoint_num_(0), frequency_(10){};

    Waypoints(std::vector<Vector2D> waypoints, double frequency, DiffDrive my_diffdrive) : waypoints_(waypoints), state_(Trans), current_waypoint_num_(0),
                                                                                           frequency_(frequency), my_diffdrive_(my_diffdrive){};

    Waypoints(CurrentState state, int current_waypoint_num, std::vector<Vector2D> waypoints, std::vector<Twist2D> cmd_sequence,
              std::vector<double> pose_sequence_x, std::vector<double> pose_sequence_y, DiffDrive my_diffdrive, double frequency)
        : state_(state), current_waypoint_num_(current_waypoint_num), waypoints_(waypoints),
          cmd_sequence_(cmd_sequence), pose_sequence_x_(pose_sequence_x), pose_sequence_y_(pose_sequence_y),
          my_diffdrive_(my_diffdrive), frequency_(frequency) {}

    /// \brief the pipeline for generating sequence of commands
    /// \param num_iter - number of iterations
    void pipeline(int num_iter);

    /// \brief find the next command according to current pose and state
    /// \returns Twist2D command
    Twist2D nextWaypoint();

    /// \brief update the pose belief using feedforward, record cmd and x, y pose
    /// \param cmd - a twist command
    /// \returns if successful
    int update_pose(Twist2D cmd);

    /// \brief update current state according to current pose
    ///
    /// 1. if close - jump to next waypoint (if the last waypoint, go to the first one)
    /// 2. if direction correct - translate
    /// 3. if direction incorrect - rotate
    /// \returns if successful
    int update_state();

    /// \brief generate a command to move forward
    /// \returns a Twist2D command
    Twist2D move_forward_cmd();

    /// \brief generate a command to rotate to turn left
    /// \returns a Twist2D command
    Twist2D rotate_l_cmd();

    /// \brief generate a command to rotate to turn right
    /// \returns a Twist2D command
    Twist2D rotate_r_cmd();

    /// \brief if current pose is close to the target waypoint
    /// \param pos_1_x
    /// \param pos_1_y
    /// \param pos_2_x
    /// \param pos_2_y
    /// \returns true if it is close
    bool if_close(double pos_1_x, double pos_1_y, double pos_2_x, double pos_2_y);

    /// \brief if the robot is in the right direction
    /// \param pos_1_x - robot position
    /// \param pos_1_y
    /// \param pos_2_x - waypoint position
    /// \param pos_2_y
    /// \param theta - current heading
    /// \returns the next state
    CurrentState if_right_direct(double pos_1_x, double pos_1_y, double pos_2_x, double pos_2_y, double theta);


    /// \brief give the internal belief of the robot's current pose
    /// \param x - pose x
    /// \param y - pose y
    /// \param theta - pose theta
    /// \returns if successful
    int pose_belief(double &x, double &y, double &theta);

public:
    // make them public for now for the use of real_waypoint node
    CurrentState state_;
    int current_waypoint_num_;

private:
    std::vector<Vector2D> waypoints_;
    std::vector<Twist2D> cmd_sequence_;
    std::vector<double> pose_sequence_x_;
    std::vector<double> pose_sequence_y_;
    DiffDrive my_diffdrive_;
    double frequency_;
};

} // namespace rigid2d

#endif
