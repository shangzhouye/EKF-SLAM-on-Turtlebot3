#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for DiffDrive class

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <iostream>
#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{

struct WheelVelocities
{
public:
    double v_left;
    double v_right;

    WheelVelocities();
};

class DiffDrive
{
public:
    /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
    DiffDrive();

    /// \brief create a DiffDrive model by specifying the pose, and geometry
    ///
    /// \param init_pose - the current position of the robot
    /// \param init_wheel_base - the distance between the wheel centers
    /// \param init_wheel_radius - the raidus of the wheels
    DiffDrive(Transform2D init_pose, double init_wheel_base, double init_wheel_radius);

    /// \brief determine the wheel velocities required to make the robot
    ///        move with the desired linear and angular velocities
    /// \param twist - the desired twist in the body frame of the robot
    /// \returns - the wheel velocities to use
    /// \throws std::exception ToDo!!!
    WheelVelocities twistToWheels(Twist2D twist);

    /// \brief determine the body twist of the robot from its wheel velocities
    /// \param vel - the velocities of the wheels, assumed to be held constant
    ///  for one time unit
    /// \returns twist in the original body frame of the robot
    Twist2D wheelsToTwist(WheelVelocities vel);

    /// \brief Update the robot's odometry based on the current encoder readings
    /// \param left - the left encoder angle (in radians)
    /// \param right - the right encoder angle (in radians)
    void updateOdometry(double left, double right);

    /// \brief update the odometry of the diff drive robot, assuming that
    /// it follows the given body twist for one time  unit
    /// \param cmd - the twist command to send to the robot
    void feedforward(Twist2D cmd);

    /// \brief get the current pose of the robot
    Transform2D get_pose();

    /// \brief get the wheel speeds, based on the last encoder update
    /// \param left_last - last left encoder update
    /// \param right_last - last right encoder update
    /// \param left_current - current left encoder update
    /// \param right_current - current right encoder update
    /// \returns the velocity of the wheels, which is equivalent to
    /// displacement because \Delta T = 1
    WheelVelocities wheelVelocities(double left_last, double right_last,
                                    double left_current, double right_current) const;

    /// \brief reset the robot to the given position/orientation
    /// \param ps - the twist to be set
    void reset(Twist2D ps);

private:
    Transform2D pose_;
    double wheel_base_;
    double wheel_radius_;
};

} // namespace rigid2d

#endif
