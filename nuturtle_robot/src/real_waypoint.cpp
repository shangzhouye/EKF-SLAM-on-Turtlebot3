/// \file
/// \brief Node for controlling the robot to follow the waypoints in real world
///
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs/Twist): publish twist command on cmd_vel
///     visualization_marker (visualization_msgs/Marker): publish markers at waypoints
/// SUBSCRIBERS:
///     nav_odo (nav_msgs/Odometry): subscribe to current robot pose
/// SERVICES:
///     stop (nuturtle_robot/Stop): stop the robot motion
///     start_waypoint (nuturtle_robot/StartWaypoint): start following the waypoint

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
#include "nuturtle_robot/StartWaypoint.h"
#include "nuturtle_robot/Stop.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <visualization_msgs/Marker.h>

static constexpr double PI = 3.14159265358979323846;

class RealWaypoint
{

public:
    RealWaypoint(ros::NodeHandle &nh)
    {
        nh.getParam("/max_trans_vel", max_trans_vel_);
        nh.getParam("/max_rot_vel", max_rot_vel_);
        nh.getParam("/max_mot_vel", max_mot_vel_);
        nh.getParam("/encoder_ticks_per_rev", encoder_ticks_per_rev_);
        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);

        stop_server_ = nh.advertiseService("stop", &RealWaypoint::stop_callback, this);
        start_waypoint_ = nh.advertiseService("start_waypoint", &RealWaypoint::start_waypoint_callback, this);

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

        pose_sub_ = nh.subscribe("nav_odo", 1000, &RealWaypoint::save_pose_callback, this);

        frac_vel_ = 0.7;

        freq_ = 110;

        // read waypoints from the yaml file
        std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;
        nh.getParam("/waypoint_x", waypoint_x);
        nh.getParam("/waypoint_y", waypoint_y);

        // write the waypoints into vector
        for (int i = 0; i < waypoint_x.size(); i++)
        {
            rigid2d::Vector2D this_point(waypoint_x.at(i), waypoint_y.at(i));
            waypoints_.push_back(this_point);
        }

        // initialize diffdrive starting at (0,0)
        rigid2d::DiffDrive my_diff(rigid2d::Transform2D(rigid2d::Vector2D(0, 0), 0), wheel_base_, wheel_radius_);
        traj_generator = rigid2d::Waypoints(waypoints_, 110, my_diff);

        if_stop_ = false;

        marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    }

    bool stop_callback(nuturtle_robot::Stop::Request &req,
                       nuturtle_robot::Stop::Response &res)
    {
        // stop the motor
        if_stop_ = true;
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.angular.z = 0;
        cmd_vel_pub_.publish(twist);

        return true;
    }

    /// \brief save the pose from the odometer
    void save_pose_callback(const nav_msgs::Odometry &msg)
    {
        current_x_ = msg.pose.pose.position.x;
        current_y_ = msg.pose.pose.position.y;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        current_theta_ = yaw;

        // ROS_INFO("The current pose is: %f, %f, %f", current_x_, current_y_, current_theta_);
    }

    bool start_waypoint_callback(nuturtle_robot::StartWaypoint::Request &req,
                                 nuturtle_robot::StartWaypoint::Response &res)
    {
        ros::Rate rate(freq_);

        // publish the markers
        for (int i = 0; i < waypoints_.size(); i++)
        {
            publish_waypoint_marker(waypoints_.at(i).x, waypoints_.at(i).y, i);
            // wait marker publishing to finish
            ros::spinOnce();
            ros::Duration(0.2).sleep();
            // ROS_INFO("Published Marker at %f, %f;", waypoints_.at(i).x, waypoints_.at(i).y);
        }

        while (ros::ok())
        {
            if (traj_generator.current_waypoint_num_ == waypoints_.size())
            {
                geometry_msgs::Twist twist_cmd;
                twist_cmd.linear.x = 0;
                twist_cmd.angular.z = 0;
                cmd_vel_pub_.publish(twist_cmd);
                return true;
            }

            // std::cout << "Current waypoint num: " << traj_generator.current_waypoint_num_ << std::endl;

            // get the next cmd_vel command
            rigid2d::Twist2D cmd = traj_generator.nextWaypoint();

            geometry_msgs::Twist twist_cmd;
            // 0.5 is because that the waypoint class defined velocity to be 0.5
            twist_cmd.linear.x = (cmd.v_x / 0.5) * max_trans_vel_ * frac_vel_;
            twist_cmd.angular.z = (cmd.omega / 0.5) * max_rot_vel_ * frac_vel_;
            cmd_vel_pub_.publish(twist_cmd);

            update_state();

            if (if_stop_)
            {
                break;
            }

            rate.sleep();
            ros::spinOnce();
        }
        return true;
    }

    int update_state()
    {

        if (traj_generator.if_close(current_x_, current_y_,
                                    waypoints_[traj_generator.current_waypoint_num_].x,
                                    waypoints_[traj_generator.current_waypoint_num_].y))
        {
            traj_generator.current_waypoint_num_++;
        }

        traj_generator.state_ = traj_generator.if_right_direct(current_x_, current_y_,
                                                               waypoints_[traj_generator.current_waypoint_num_].x,
                                                               waypoints_[traj_generator.current_waypoint_num_].y,
                                                               current_theta_);

        return 0;
    }

    int publish_waypoint_marker(double x, double y, int marker_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();

        marker.ns = "markers";
        marker.id = marker_id;

        uint32_t shape = visualization_msgs::Marker::CYLINDER;
        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Publish the marker
        marker_pub_.publish(marker);
        ros::spinOnce();

        return 0;
    }

private:
    double max_trans_vel_;
    double max_rot_vel_;
    double max_mot_vel_;
    double encoder_ticks_per_rev_;
    double wheel_base_;
    double wheel_radius_;

    ros::Publisher cmd_vel_pub_;
    double frac_vel_;
    int freq_;

    ros::ServiceServer stop_server_;
    ros::ServiceServer start_waypoint_;
    ros::Subscriber pose_sub_;

    // current robot pose from encoder/odometer
    double current_x_, current_y_, current_theta_;

    std::vector<rigid2d::Vector2D> waypoints_;

    rigid2d::Waypoints traj_generator;

    bool if_stop_;

    ros::Publisher marker_pub_;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "real_waypoint");
    ros::NodeHandle nh;
    RealWaypoint my_real_waypoint = RealWaypoint(nh);

    ros::spin();
    return 0;
}