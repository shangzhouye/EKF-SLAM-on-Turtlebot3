#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <string>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/JointState.h"

static double wheel_left_vel = 0;
static double wheel_right_vel = 0;
static double wheel_left_pos = 0;
static double wheel_right_pos = 0;
static bool if_sub = false;

void wheel_vel_callback(const nuturtlebot::WheelCommands &msg)
{
    wheel_left_vel = msg.left_velocity;
    wheel_right_vel = msg.right_velocity;
    if_sub = true;
}

TEST(TurtleInterfaceTest, wheel_cmd_test_1)
{
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    ros::Subscriber sub;
    sub = nh.subscribe("/wheel_cmd", 1000, wheel_vel_callback);

    geometry_msgs::Twist twist;
    twist.angular.z = 0;
    twist.linear.x = 1;

    while (pub.getNumSubscribers() == 0)
    {
        ros::spinOnce();
    }

    pub.publish(twist);

    while (!if_sub)
    {
        ros::spinOnce();
    }

    if_sub = false;

    ASSERT_DOUBLE_EQ(256, wheel_left_vel);
    ASSERT_DOUBLE_EQ(256, wheel_right_vel);
}

TEST(TurtleInterfaceTest, wheel_cmd_test_2)
{
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    ros::Subscriber sub;
    sub = nh.subscribe("/wheel_cmd", 1000, wheel_vel_callback);

    geometry_msgs::Twist twist;
    twist.angular.z = 1;
    twist.linear.x = 0;

    while (pub.getNumSubscribers() == 0)
    {
        ros::spinOnce();
    }

    pub.publish(twist);

    while (!if_sub)
    {
        ros::spinOnce();
    }

    if_sub = false;

    ASSERT_DOUBLE_EQ(-98, wheel_left_vel);
    ASSERT_DOUBLE_EQ(98, wheel_right_vel);
}

TEST(TurtleInterfaceTest, wheel_cmd_test_3)
{
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    ros::Subscriber sub;
    sub = nh.subscribe("/wheel_cmd", 1000, wheel_vel_callback);

    geometry_msgs::Twist twist;
    twist.angular.z = 1;
    twist.linear.x = 0.1;

    while (pub.getNumSubscribers() == 0)
    {
        ros::spinOnce();
    }

    pub.publish(twist);

    while (!if_sub)
    {
        ros::spinOnce();
    }

    if_sub = false;

    ASSERT_DOUBLE_EQ(24, wheel_left_vel);
    ASSERT_DOUBLE_EQ(220, wheel_right_vel);
}

void sensor_data_callback(const sensor_msgs::JointState &msg)
{
    wheel_right_pos = msg.position.at(0);
    wheel_left_pos = msg.position.at(1);
    if_sub = true;
}

TEST(TurtleInterfaceTest, sensor_data_test)
{
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<nuturtlebot::SensorData>("sensor_data", 10);

    ros::Subscriber sub;
    sub = nh.subscribe("joint_state", 1000, sensor_data_callback);

    nuturtlebot::SensorData sensor_data_msg;
    sensor_data_msg.left_encoder = 500;
    sensor_data_msg.right_encoder = 600;

    while (pub.getNumSubscribers() == 0)
    {
        ros::spinOnce();
    }

    pub.publish(sensor_data_msg);

    while (!if_sub)
    {
        ros::spinOnce();
    }

    if_sub = false;

    ASSERT_NEAR(0.76699, wheel_left_pos, 1.0e-3);
    ASSERT_NEAR(0.92038846, wheel_right_pos, 1.0e-3);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");
    ros::NodeHandle nh;

    ros::Duration(3).sleep();
    return RUN_ALL_TESTS();
}