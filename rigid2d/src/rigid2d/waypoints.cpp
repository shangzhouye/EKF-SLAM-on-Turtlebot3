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
    Twist2D cmd(0.0, 0.5, 0.0);
    return cmd;
}

Twist2D Waypoints::rotate_l_cmd()
{
    // can modify this to a porpotional controller
    Twist2D cmd(0.5, 0.0, 0.0);
    return cmd;
}

Twist2D Waypoints::rotate_r_cmd()
{
    // can modify this to a porpotional controller
    Twist2D cmd(-0.5, 0.0, 0.0);
    return cmd;
}

Twist2D Waypoints::nextWaypoint()
{
    if (state_ == Trans)
    {
        return move_forward_cmd();
    }
    else if (state_ == Rot_l)
    {
        return rotate_l_cmd();
    }

    else
    {
        return rotate_r_cmd();
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

    state_ = if_right_direct(pose_x, pose_y, waypoints_[current_waypoint_num_].x, waypoints_[current_waypoint_num_].y, pose_theta);

    return 0;
}

bool Waypoints::if_close(double pos_1_x, double pos_1_y, double pos_2_x, double pos_2_y)
{
    return std::sqrt(std::pow(pos_1_x - pos_2_x, 2) + std::pow(pos_1_y - pos_2_y, 2)) < 0.03;
}

CurrentState Waypoints::if_right_direct(double pos_1_x, double pos_1_y, double pos_2_x, double pos_2_y, double theta)
{
    double angle_diff = normalize_angle(std::atan2(pos_2_y - pos_1_y, pos_2_x - pos_1_x) - theta);
    if (angle_diff < 0.1 && -0.1 < angle_diff)
    {
        return Trans;
    }
    else if (angle_diff > 0.1)
    {
        return Rot_l;
    }
    else
    {
        return Rot_r;
    }
}

int Waypoints::pose_belief(double &x, double &y, double &theta)
{
    Transform2D pose = my_diffdrive_.get_pose();
    pose.displacement(x, y, theta);
    return 0;
}

} // namespace rigid2d