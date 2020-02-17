/// \file
/// \brief Node for the turtle interface:  low-level control and odometry routines
///
/// PUBLISHES:
///     joint_state (sensor_msgs/JointState): publish joint states from sensor data
///     wheel_cmd (nuturtlebot/WheelCommands): publish wheel command to move the robot
/// SUBSCRIBES:
///     turtle1/cmd_vel (geometry_msgs/Twist): subscribe to current cmd_vel commands
///     sensor_data (nuturtlebot/SensorData): subscribe to the sensor (encoder) data

#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "ros/ros.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <string>
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/JointState.h"
#include <vector>

static constexpr double PI = 3.14159265358979323846;

class TurtleInterface
{
public:
    TurtleInterface(ros::NodeHandle &nh)
    {
        cmd_vel_sub_ = nh.subscribe("turtle1/cmd_vel", 1000, &TurtleInterface::cmd_vel_callback, this);

        sensor_data_sub_ = nh.subscribe("sensor_data", 1000, &TurtleInterface::sensor_data_callback, this);

        wheel_cmd_pub_ = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 10);

        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("joint_state", 10);

        nh.getParam("/max_trans_vel", max_trans_vel_);
        nh.getParam("/max_rot_vel", max_rot_vel_);
        nh.getParam("/max_mot_vel", max_mot_vel_);
        nh.getParam("/encoder_ticks_per_rev", encoder_ticks_per_rev_);
        nh.getParam("/left_wheel_joint", left_wheel_joint_);
        nh.getParam("/right_wheel_joint", right_wheel_joint_);

        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);

        my_robot_ = rigid2d::DiffDrive(rigid2d::Transform2D(), wheel_base_, wheel_radius_);
    }

    void cmd_vel_callback(const geometry_msgs::Twist &msg)
    {
        double linear = msg.linear.x;
        double angular = msg.angular.z;

        // clamp largest velocity
        if (linear > max_trans_vel_)
        {
            linear = max_trans_vel_;
        }

        if (angular > max_rot_vel_)
        {
            angular = max_rot_vel_;
        }

        // ROS_INFO("Calculated linear, angular twist are: %f and %f", linear, angular);

        rigid2d::Twist2D twist_cmd(angular, linear, 0);

        rigid2d::WheelVelocities wheel_cmd;

        wheel_cmd = my_robot_.twistToWheels(twist_cmd);

        // transfer to command integer
        if ((wheel_cmd.v_left > max_mot_vel_) || (wheel_cmd.v_left < -max_mot_vel_))
        {
            wheel_cmd.v_left = (wheel_cmd.v_left / abs(wheel_cmd.v_left)) * max_mot_vel_;
        }

        if ((wheel_cmd.v_right > max_mot_vel_) || (wheel_cmd.v_right < -max_mot_vel_))
        {
            wheel_cmd.v_right = (wheel_cmd.v_right / abs(wheel_cmd.v_right)) * max_mot_vel_;
        }

        wheel_cmd.v_left = (wheel_cmd.v_left) / max_mot_vel_ * 256;
        wheel_cmd.v_right = (wheel_cmd.v_right) / max_mot_vel_ * 256;

        // ROS_INFO("Calculated wheel vel are: %f and %f", wheel_cmd.v_left, wheel_cmd.v_right);

        nuturtlebot::WheelCommands wheel_cmd_msg;

        wheel_cmd_msg.left_velocity = std::round(wheel_cmd.v_left);
        wheel_cmd_msg.right_velocity = std::round(wheel_cmd.v_right);

        wheel_cmd_pub_.publish(wheel_cmd_msg);
    }

    void sensor_data_callback(const nuturtlebot::SensorData &msg)
    {
        double left_pos, right_pos;
        left_pos = (msg.left_encoder / encoder_ticks_per_rev_) * 2 * PI;
        right_pos = (msg.right_encoder / encoder_ticks_per_rev_) * 2 * PI;

        sensor_msgs::JointState joint_state;

        // fill in the message
        joint_state.header.stamp = ros::Time::now();

        std::vector<std::string> name;
        name.push_back(right_wheel_joint_);
        name.push_back(left_wheel_joint_);

        joint_state.name = name;
        std::vector<double> position;
        position.push_back(right_pos);
        position.push_back(left_pos);
        joint_state.position = position;

        joint_state_pub_.publish(joint_state);
    }

private:
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher wheel_cmd_pub_;
    double max_trans_vel_;
    double max_rot_vel_;
    double max_mot_vel_;
    double encoder_ticks_per_rev_;
    rigid2d::DiffDrive my_robot_;

    ros::Subscriber sensor_data_sub_;
    ros::Publisher joint_state_pub_;

    std::string right_wheel_joint_;
    std::string left_wheel_joint_;

    double wheel_base_;
    double wheel_radius_;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;
    TurtleInterface my_turtle_interface = TurtleInterface(nh);

    ros::spin();
    return 0;
}