// Created by Shangzhou Ye, MSR, Northwestern University

#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <string>

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

// google test main function
int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}