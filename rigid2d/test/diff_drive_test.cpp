// Created by Shangzhou Ye, MSR, Northwestern University

#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <string>
#include <cmath>

TEST(DiffDriveTest, twistToWheels_translate)
{
    rigid2d::Twist2D twist;
    twist.v_x = 1;
    twist.v_y = 0;
    twist.omega = 0;

    rigid2d::WheelVelocities result;

    rigid2d::DiffDrive diff_drive_test;
    result = diff_drive_test.twistToWheels(twist);

    // same as default value
    double wheel_radius = 0.1;

    ASSERT_DOUBLE_EQ(1.0, result.v_left * wheel_radius);
    ASSERT_DOUBLE_EQ(1.0, result.v_right * wheel_radius);
}

TEST(DiffDriveTest, twistToWheels_rotate)
{
    rigid2d::Twist2D twist;
    twist.v_x = 0;
    twist.v_y = 0;
    twist.omega = 1;

    rigid2d::WheelVelocities result;

    rigid2d::DiffDrive diff_drive_test;
    result = diff_drive_test.twistToWheels(twist);

    // same as default value
    double wheel_radius = 0.1;
    double wheel_base = 0.4;

    ASSERT_DOUBLE_EQ(-(1.0 / 2.0) * wheel_base, result.v_left * wheel_radius);
    ASSERT_DOUBLE_EQ((1.0 / 2.0) * wheel_base, result.v_right * wheel_radius);
}

TEST(DiffDriveTest, wheelToTwist)
{

    rigid2d::WheelVelocities wheel_vel;
    wheel_vel.v_left = 0;
    wheel_vel.v_right = 1;

    rigid2d::DiffDrive diff_drive_test;

    rigid2d::Twist2D result;
    result = diff_drive_test.wheelsToTwist(wheel_vel);

    // same as default value
    double wheel_radius = 0.1;
    double wheel_base = 0.4;

    ASSERT_DOUBLE_EQ(result.v_x, wheel_radius / 2.0);
    ASSERT_DOUBLE_EQ(result.omega, wheel_radius / wheel_base);
}

TEST(DiffDriveTest, MovingForward)
{
    rigid2d::WheelVelocities wheel_vel;
    wheel_vel.v_left = 1;
    wheel_vel.v_right = 1;

    rigid2d::Twist2D cmd;
    rigid2d::DiffDrive diff_drive_test;
    cmd = diff_drive_test.wheelsToTwist(wheel_vel);

    // same as default value
    double wheel_radius = 0.1;

    diff_drive_test.updateOdometry(wheel_vel.v_left, wheel_vel.v_right);

    rigid2d::Transform2D result1 = diff_drive_test.get_pose();

    double x, y, theta;
    result1.displacement(x, y, theta);

    ASSERT_DOUBLE_EQ(x, wheel_radius);

    diff_drive_test.feedforward(cmd);

    rigid2d::Transform2D result2 = diff_drive_test.get_pose();

    result2.displacement(x, y, theta);

    ASSERT_DOUBLE_EQ(x, wheel_radius * 2.0);
}

TEST(DiffDriveTest, MovingForwardAndTurning1)
{
    rigid2d::WheelVelocities wheel_vel;
    wheel_vel.v_left = 0;
    wheel_vel.v_right = 1;

    rigid2d::DiffDrive diff_drive_test;

    // same as default value
    double wheel_radius = 0.1;
    double wheel_base = 0.4;

    diff_drive_test.updateOdometry(wheel_vel.v_left, wheel_vel.v_right);

    rigid2d::Transform2D result1 = diff_drive_test.get_pose();

    double x, y, theta;
    result1.displacement(x, y, theta);

    ASSERT_DOUBLE_EQ(x, wheel_base * std::sin(wheel_radius / wheel_base) / 2.0);
    ASSERT_NEAR(y, wheel_base / 2.0 - (wheel_base * std::cos(wheel_radius / wheel_base) / 2.0), 0.000001);
    ASSERT_DOUBLE_EQ(theta, wheel_radius / wheel_base);
}

TEST(DiffDriveTest, MovingForwardAndTurning2)
{
    rigid2d::WheelVelocities wheel_vel;
    wheel_vel.v_left = 0;
    wheel_vel.v_right = 1;

    rigid2d::Twist2D cmd;
    rigid2d::DiffDrive diff_drive_test;
    cmd = diff_drive_test.wheelsToTwist(wheel_vel);

    // same as default value
    double wheel_radius = 0.1;
    double wheel_base = 0.4;

    double x, y, theta;
    diff_drive_test.feedforward(cmd);

    rigid2d::Transform2D result2 = diff_drive_test.get_pose();

    result2.displacement(x, y, theta);

    ASSERT_DOUBLE_EQ(x, wheel_base * std::sin(wheel_radius / wheel_base) / 2.0);
    ASSERT_NEAR(y, wheel_base / 2.0 - (wheel_base * std::cos(wheel_radius / wheel_base) / 2.0), 0.000001);
    ASSERT_DOUBLE_EQ(theta, wheel_radius / wheel_base);
}

// google test main function
int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}