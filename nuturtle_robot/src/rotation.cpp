/// \file
/// \brief Node for rotating the turtlebot in place / translating the robot for testing
///
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs/Twist): publish twist command on cmd_vel
/// SERVICES:
///     start (nuturtle_robot/Start): service to start the rotation

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
#include "nuturtle_robot/Start.h"
#include "rigid2d/SetPose.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>

static constexpr double PI = 3.14159265358979323846;

class Rotation
{

public:
    Rotation(ros::NodeHandle &nh)
    {
        nh.getParam("/max_trans_vel", max_trans_vel_);
        nh.getParam("/max_rot_vel", max_rot_vel_);
        nh.getParam("/max_mot_vel", max_mot_vel_);
        nh.getParam("/encoder_ticks_per_rev", encoder_ticks_per_rev_);
        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);

        start_rotation_ = nh.advertiseService("start",
                                              &Rotation::start_callback, this);

        set_pose_client_ = nh.serviceClient<rigid2d::SetPose>("set_pose");

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

        frac_vel_ = 0.7;

        freq_ = 110;
    }

    bool start_callback(nuturtle_robot::Start::Request &req,
                        nuturtle_robot::Start::Response &res)
    {
        if (req.if_rotation)
        {
            int if_clockwise = req.if_clockwise;
            if (if_clockwise)
            {
                if_clockwise = -1;
            }
            else
            {
                if_clockwise = 1;
            }

            // reset the odometry

            rigid2d::SetPose set_pose_req;
            set_pose_req.request.x = 0;
            set_pose_req.request.y = 0;
            set_pose_req.request.theta = 0;
            set_pose_client_.call(set_pose_req);

            // pause after full rotation
            ros::Rate rate(freq_);

            geometry_msgs::Twist twist;
            twist.linear.x = 0;
            twist.angular.z = if_clockwise * frac_vel_ * max_rot_vel_;

            int pub_times = abs((2 * PI) / (twist.angular.z * (1.0 / static_cast<double>(freq_))));

            for (int k = 0; k < 20; k++)
            {
                for (int i = 0; i < pub_times; i++)
                {
                    twist.linear.x = 0;
                    twist.angular.z = if_clockwise * frac_vel_ * max_rot_vel_;
                    cmd_vel_pub_.publish(twist);
                    ros::spinOnce();
                    rate.sleep();
                }
                for (int j = 0; j < (pub_times / 20.0); j++)
                {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    cmd_vel_pub_.publish(twist);
                    ros::spinOnce();
                    rate.sleep();
                }
                ros::spinOnce();
            }

            // stop the motor
            twist.linear.x = 0;
            twist.angular.z = 0;
            cmd_vel_pub_.publish(twist);
        }
        else
        {
            int if_forward = req.if_forward;
            if (if_forward)
            {
                if_forward = 1;
            }
            else
            {
                if_forward = -1;
            }

            // reset the odometry

            rigid2d::SetPose set_pose_req;
            set_pose_req.request.x = 0;
            set_pose_req.request.y = 0;
            set_pose_req.request.theta = 0;
            set_pose_client_.call(set_pose_req);

            // pause after full rotation
            ros::Rate rate(freq_);

            geometry_msgs::Twist twist;
            twist.linear.x = if_forward * frac_vel_ * max_trans_vel_;
            twist.angular.z = 0;

            int pub_times = abs(0.2 / (twist.linear.x * (1.0 / static_cast<double>(freq_))));

            for (int k = 0; k < 10; k++)
            {
                for (int i = 0; i < pub_times; i++)
                {
                    twist.linear.x = if_forward * frac_vel_ * max_trans_vel_;
                    twist.angular.z = 0;
                    cmd_vel_pub_.publish(twist);
                    ros::spinOnce();
                    rate.sleep();
                }
                for (int j = 0; j < (pub_times / 10.0); j++)
                {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    cmd_vel_pub_.publish(twist);
                    ros::spinOnce();
                    rate.sleep();
                }
                ros::spinOnce();
            }

            // stop the motor
            twist.linear.x = 0;
            twist.angular.z = 0;
            cmd_vel_pub_.publish(twist);
        }

        return true;
    }

private:
    double max_trans_vel_;
    double max_rot_vel_;
    double max_mot_vel_;
    double encoder_ticks_per_rev_;
    double wheel_base_;
    double wheel_radius_;

    ros::ServiceServer start_rotation_;
    ros::ServiceClient set_pose_client_;
    ros::Publisher cmd_vel_pub_;

    double frac_vel_;

    int freq_;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "rotation");
    ros::NodeHandle nh;
    Rotation my_rotation = Rotation(nh);

    ros::spin();
    return 0;
}