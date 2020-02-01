/// \file
/// \brief Library for Waypoints class
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

namespace rigid2d
{

Waypoints::Waypoints()
    : waypoints_(std::vector<Vector2D>{Vector2D(3, 2), Vector2D(7, 2), Vector2D(7, 7), Vector2D(3, 7)}),
      my_diffdrive_(DiffDrive(Transform2D(Vector2D(0, 0), 0), 0.4, 0.1)), state_(Trans), current_waypoint_num_(0), frequency_(10) {}

Waypoints::Waypoints(CurrentState state, int current_waypoint_num, std::vector<Vector2D> waypoints, std::vector<Twist2D> cmd_sequence,
                     std::vector<double> pose_sequence_x, std::vector<double> pose_sequence_y, DiffDrive my_diffdrive, double frequency)
    : state_(state), current_waypoint_num_(current_waypoint_num), waypoints_(waypoints),
      cmd_sequence_(cmd_sequence), pose_sequence_x_(pose_sequence_x), pose_sequence_y_(pose_sequence_y),
      my_diffdrive_(my_diffdrive), frequency_(frequency) {}

void Waypoints::pipeline(int num_iter)
{
    for (int i = 0; i < num_iter; i++)
    {
        Twist2D cmd = nextWaypoint();
        update_pose(cmd);
        update_state();
    }
}

Twist2D Waypoints::move_forward_cmd()
{
    // can modify this to a porpotional controller
    Twist2D cmd(0.0, 1.0, 0.0);
    return cmd;
}

Twist2D Waypoints::rotate_cmd()
{
    // can modify this to a porpotional controller
    Twist2D cmd(1.0, 0.0, 0.0);
    return cmd;
}

Twist2D Waypoints::nextWaypoint()
{
    if (state_ == Trans)
    {
        return move_forward_cmd();
    }
    else
    {
        return rotate_cmd();
    }
}

int Waypoints::update_pose(Twist2D cmd)
{
    // correct the twist according to frequency
    cmd.omega = cmd.omega * (1.0 / frequency_);
    cmd.v_x = cmd.v_x * (1.0 / frequency_);
    cmd.v_y = cmd.v_y * (1.0 / frequency_);

    my_diffdrive_.feedforward(cmd);
    cmd_sequence_.push_back(cmd);
    double pose_x, pose_y, pose_theta;
    Transform2D pose = my_diffdrive_.get_pose();
    pose.displacement(pose_x, pose_y, pose_theta);
    pose_sequence_x_.push_back(pose_x);
    pose_sequence_y_.push_back(pose_y);

    // debug info
    std::cout << "x is: " << pose_x << "; y is: " << pose_y << "; num is: " << current_waypoint_num_ << std::endl;

    return 0;
}

int Waypoints::update_state()
{
    double pose_x, pose_y, pose_theta;
    Transform2D pose = my_diffdrive_.get_pose();
    pose.displacement(pose_x, pose_y, pose_theta);

    if (if_close(pose_x, pose_y, waypoints_[current_waypoint_num_].x, waypoints_[current_waypoint_num_].y))
    {
        current_waypoint_num_++;
        if (current_waypoint_num_ == waypoints_.size())
        {
            current_waypoint_num_ = 0;
        }
    }

    if (if_right_direct(pose_x, pose_y, waypoints_[current_waypoint_num_].x, waypoints_[current_waypoint_num_].y, pose_theta))
    {
        state_ = Trans;
    }
    else
    {
        state_ = Rot;
    }

    return 0;
}

bool Waypoints::if_close(double pos_1_x, double pos_1_y, double pos_2_x, double pos_2_y)
{
    return std::sqrt(std::pow(pos_1_x - pos_2_x, 2) + std::pow(pos_1_y - pos_2_y, 2)) < 0.5;
}

bool Waypoints::if_right_direct(double pos_1_x, double pos_1_y, double pos_2_x, double pos_2_y, double theta)
{
    double angle_diff = normalize_angle(std::atan2(pos_2_y - pos_1_y, pos_2_x - pos_1_x) - theta);
    return angle_diff < 0.2 && -0.2 < angle_diff;
}

} // namespace rigid2d