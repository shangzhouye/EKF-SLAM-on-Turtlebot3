/// \file
/// \brief Node for the fake encoder
///
/// PARAMETERS:
///     left_wheel_joint_: name of the left wheel joint
///     right_wheel_joint_: name of the right wheel joint
///     wheel_base_: wheel_base parameter for the diff drive robot
///     wheel_radius_: wheel radius of the diff drive robot
///     current_l_: absolute position of the left wheel in current time step
///     current_r_: absolute position of the right wheel in current time step
///     last_time_now_: time stamp of last time step
///     seq_: sequence of the current joint state
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): publish joint states as an encoder
/// SUBSCRIBES:
///     turtle1/cmd_vel (geometry_msgs/Twist): subscribe to current cmd_vel commands

#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "ros/ros.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <string>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include <vector>

class FakeDiffEncoder
{
public:
    FakeDiffEncoder(ros::NodeHandle &nh)
    {
        cmd_vel_sub_ = nh.subscribe("turtle1/cmd_vel", 1000, &FakeDiffEncoder::cmd_vel_callback, this);
        joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

        // set param here for testing
        // nh.setParam("/wheel_base", 0.4);
        // nh.setParam("/wheel_radius", 0.1);
        nh.setParam("/left_wheel_joint", "left_wheel_axle");
        nh.setParam("/right_wheel_joint", "right_wheel_axle");

        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);
        nh.getParam("/left_wheel_joint", left_wheel_joint_);
        nh.getParam("/right_wheel_joint", right_wheel_joint_);

        my_robot_ = rigid2d::DiffDrive(rigid2d::Transform2D(), wheel_base_, wheel_radius_);

        current_l_ = 0;
        current_r_ = 0;
        last_time_now_ = ros::Time::now();

        seq_ = 0;
    }

    /// \brief read the cmd_vel command, publish wheel velocity like an encoder
    void cmd_vel_callback(const geometry_msgs::Twist &msg)
    {

        sensor_msgs::JointState joint_state;

        // fill in the message
        joint_state.header.seq = seq_;
        seq_ += 1;

        joint_state.header.stamp = ros::Time::now();

        std::vector<std::string> name;
        name.push_back(right_wheel_joint_);
        name.push_back(left_wheel_joint_);
        joint_state.name = name;

        rigid2d::Twist2D twist;
        twist.v_x = msg.linear.x;
        twist.omega = msg.angular.z;
        rigid2d::WheelVelocities wheel_vel;
        wheel_vel = my_robot_.twistToWheels(twist);

        ros::Time current_time_now = ros::Time::now();
        current_l_ = current_l_ + (wheel_vel.v_left * (current_time_now - last_time_now_).toSec());
        current_l_ = rigid2d::normalize_angle(current_l_);
        current_r_ = current_r_ + (wheel_vel.v_right * (current_time_now - last_time_now_).toSec());
        current_r_ = rigid2d::normalize_angle(current_r_);

        last_time_now_ = current_time_now;
        ros::spinOnce();

        std::vector<double> position;
        position.push_back(current_r_);
        position.push_back(current_l_);
        joint_state.position = position;

        joint_states_pub_.publish(joint_state);
    }

private:
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher joint_states_pub_;

    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    double wheel_base_;
    double wheel_radius_;
    rigid2d::DiffDrive my_robot_;

    // define two wheel states: last and current
    double current_l_;
    double current_r_;

    ros::Time last_time_now_;

    int seq_;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "fake_diff_encoders");
    ros::NodeHandle nh;
    FakeDiffEncoder encoder = FakeDiffEncoder(nh);

    ros::spin();
    return 0;
}