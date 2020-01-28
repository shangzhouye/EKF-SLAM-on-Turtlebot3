/// \file
/// \brief Library for DiffDrive class
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

namespace rigid2d
{

WheelVelocities::WheelVelocities()
{
    v_left = 0;
    v_right = 0;
}

DiffDrive::DiffDrive()
{
    Vector2D init_vec;
    init_vec.x = 0;
    init_vec.y = 0;
    Transform2D init_pose(init_vec, 0);
    pose_ = init_pose;
    wheel_base_ = 0.4;
    wheel_radius_ = 0.1;
}

DiffDrive::DiffDrive(Transform2D init_pose, double init_wheel_base, double init_wheel_radius)
{
    pose_ = init_pose;
    wheel_base_ = init_wheel_base;
    wheel_radius_ = init_wheel_radius;
}

WheelVelocities DiffDrive::twistToWheels(Twist2D twist)
{
    WheelVelocities wheel_v;
    wheel_v.v_right = (2 * twist.v_x + twist.omega * wheel_base_) / (2 * wheel_radius_);
    wheel_v.v_left = (2 * twist.v_x - twist.omega * wheel_base_) / (2 * wheel_radius_);
    return wheel_v;
}

Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel)
{
    Twist2D twist_b;
    twist_b.v_x = wheel_radius_ * (vel.v_left + vel.v_right) / 2.0;
    twist_b.omega = wheel_radius_ * (vel.v_right - vel.v_left) / wheel_base_;
    return twist_b;
}

void DiffDrive::updateOdometry(double left, double right)
{
    double delta_x_b = wheel_radius_ * (left + right) / 2.0;
    Vector2D delta_vec_b(delta_x_b, 0);
    double delta_theta_b = wheel_radius_ * (right - left) / wheel_base_;
    Transform2D step_displacement(delta_vec_b, delta_theta_b);
    pose_ *= step_displacement;
}

void DiffDrive::feedforward(Twist2D cmd)
{
    Vector2D delta_vec_b(cmd.v_x, 0);
    Transform2D step_displacement(delta_vec_b, cmd.omega);
    pose_ *= step_displacement;
}

Transform2D DiffDrive::get_pose()
{
    return pose_;
}

void DiffDrive::reset(Twist2D ps)
{
    Transform2D reset_pose;
    reset_pose = integrateTwist(ps);
    pose_ = reset_pose;
}

WheelVelocities DiffDrive::wheelVelocities(double left_last, double right_last,
                                           double left_current, double right_current) const
{
    WheelVelocities result;
    result.v_left = left_current - left_last;
    result.v_right = right_current - right_last;
    return result;
}

} // namespace rigid2d