/// \file
/// \brief Node for odometry
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "ros/ros.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <string>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Odometer
{

private:
    std::string odom_frame_id_;
    std::string body_frame_id_;
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher nav_odo_pub_;
    double wheel_base_;
    double wheel_radius_;
    rigid2d::DiffDrive my_robot_;

    // define two wheel states: last and current
    double last_l_;
    double last_r_;
    double current_l_;
    double current_r_;

public:
    Odometer(ros::NodeHandle &nh)
    {
        nav_odo_pub_ = nh.advertise<nav_msgs::Odometry>("/nav_odo", 10);
        joint_state_sub_ = nh.subscribe("/joint_states", 1000, &Odometer::joint_states_callback, this);

        // set param here for testing
        nh.setParam("/wheel_base", 0.4);
        nh.setParam("/wheel_radius", 0.1);
        nh.setParam("/left_wheel_joint", "left_wheel_axle");
        nh.setParam("/right_wheel_joint", "right_wheel_axle");
        body_frame_id_ = "base_link";
        odom_frame_id_ = "odom";

        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);
        nh.getParam("/left_wheel_joint", left_wheel_joint_);
        nh.getParam("/right_wheel_joint", right_wheel_joint_);

        my_robot_ = rigid2d::DiffDrive();

        last_l_ = 0;
        last_r_ = 0;
    }

    void joint_states_callback(const sensor_msgs::JointState msg)
    {
        current_l_ = msg.position.at(1);
        current_r_ = msg.position.at(0);
        double dist_l = current_l_ - last_l_;
        double dist_r = current_r_ - last_r_;
        my_robot_.updateOdometry(dist_l, dist_r);

        nav_msgs::Odometry odo_msg;
        odo_msg.header.frame_id = odom_frame_id_;
        odo_msg.child_frame_id = body_frame_id_;

        rigid2d::Transform2D current_pose = my_robot_.get_pose();
        double pose_x, pose_y, pose_theta;
        current_pose.displacement(pose_x, pose_y, pose_theta);
        odo_msg.pose.pose.position.x = pose_x;
        odo_msg.pose.pose.position.y = pose_y;

        // convert orientation to quaternion
        tf2::Quaternion q_rot;
        double r = 0, p = 0, y = pose_theta;
        q_rot.setRPY(r, p, y);
        q_rot.normalize();
        tf2::convert(q_rot, odo_msg.pose.pose.orientation);

        // double vel_l = msg.velocity.at(1);
        // double vel_r = msg.velocity.at(0);
        // rigid2d::WheelVelocities wheel_vel;
        // wheel_vel.v_left = vel_l;
        // wheel_vel.v_right = vel_r;
        // rigid2d::Twist2D twist = my_robot_.wheelsToTwist(wheel_vel);
        // odo_msg.twist.twist.linear.x = twist.v_x;
        // odo_msg.twist.twist.linear.y = twist.v_y;
        // odo_msg.twist.twist.angular.z = twist.omega;

        last_l_ = current_l_;
        last_r_ = current_r_;

        nav_odo_pub_.publish(odo_msg);
    }
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;
    Odometer odo = Odometer(nh);

    ros::spin();
    return 0;
}