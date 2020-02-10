/// \file
/// \brief Make the turtle move in a pentagonal trajectory.
///
/// PUBLISHES:
///     pose_error (tsim::PoseError): Publish the error of feedforward control
///     turtle1/cmd_vel (geometry_msgs::Twist): Publish the control commands to move the turtle
/// SUBSCRIBES:
///     turtle1/pose (turtlesim::Pose): The current actual pose (x, y, heading) of the turtle

#include <cmath>
#include "ros/ros.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "tsim/PoseError.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <string>

const double PI = 3.14159265358979323846;

class TurtleWay
{
public:
    TurtleWay(ros::NodeHandle &nh)
    {
        // define the publishers, subscriber
        pose_sub_ = nh.subscribe("turtle1/pose", 1000, &TurtleWay::callback_save_pose, this);
        vel_control_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 60);
        error_pub_ = nh.advertise<tsim::PoseError>("pose_error", 60);

        // read waypoints from the yaml file
        std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;
        nh.getParam("/waypoint_x", waypoint_x);
        nh.getParam("/waypoint_y", waypoint_y);

        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);

        // write the waypoints into vector
        for (int i = 0; i < waypoint_x.size(); i++)
        {
            rigid2d::Vector2D this_point(waypoint_x.at(i), waypoint_y.at(i));
            waypoints_.push_back(this_point);
        }

        // print out waypoints for debug
        // for (std::vector<rigid2d::Vector2D>::const_iterator i = waypoints_.begin(); i != waypoints_.end(); ++i)
        //     std::cout << *i << std::endl;

        // initialize diffdrive starting at the first waypoint
        rigid2d::DiffDrive my_diff(rigid2d::Transform2D(rigid2d::Vector2D(waypoints_.at(0).x, waypoints_.at(0).y), 0), wheel_base_, wheel_radius_);
        traj_generator = rigid2d::Waypoints(waypoints_, 60, my_diff);

        set_pen_client_ = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
        teleport_turtle_client_ =
            nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    }

    /// \brief the callback function to save the actual pose
    void callback_save_pose(const turtlesim::Pose &msg)
    {
        // save the actual pose from the turtle simulator
        actual_x_ = msg.x;
        actual_y_ = msg.y;
        actual_theta_ = msg.theta;
    }

    /// \brief follow the trajectory with rotate and translate strategy
    void pipeline()
    {
        // lift the pen
        turtlesim::SetPen set_pen_req;
        set_pen_req.request.r = 255;
        set_pen_req.request.g = 255;
        set_pen_req.request.b = 255;
        set_pen_req.request.width = 2;
        set_pen_req.request.off = true;

        set_pen_client_.waitForExistence();
        if (set_pen_client_.call(set_pen_req))
        {
            ROS_INFO("Pen lifted");
        }

        // teleport the turtle
        turtlesim::TeleportAbsolute teleport_absolute_req;
        teleport_absolute_req.request.x = waypoints_.at(0).x;
        teleport_absolute_req.request.y = waypoints_.at(0).y;
        teleport_absolute_req.request.theta = 0;

        teleport_turtle_client_.waitForExistence();
        if (teleport_turtle_client_.call(teleport_absolute_req))
        {
            ROS_INFO("Turtle teleported");
        }

        // put the pen down
        turtlesim::SetPen set_pen_down;
        set_pen_down.request.r = 255;
        set_pen_down.request.g = 255;
        set_pen_down.request.b = 255;
        set_pen_down.request.width = 2;
        set_pen_down.request.off = false;

        set_pen_client_.waitForExistence();
        if (set_pen_client_.call(set_pen_down))
        {
            ROS_INFO("Put pen down");
        }

        // wait for the turtle to teleport
        ros::Duration(1).sleep();

        ros::Rate rate(60);
        while (ros::ok())
        {
            // get the next cmd_vel command
            rigid2d::Twist2D cmd = traj_generator.nextWaypoint();
            geometry_msgs::Twist twist_cmd;
            twist_cmd.linear.x = cmd.v_x;
            twist_cmd.angular.z = cmd.omega;

            vel_control_.publish(twist_cmd);
            traj_generator.update_pose(cmd);
            traj_generator.update_state();

            tsim::PoseError error_msg;

            // calculate error
            double target_x, target_y, target_theta;
            traj_generator.pose_belief(target_x, target_y, target_theta);
            error_msg.x_error = actual_x_ - target_x;
            error_msg.y_error = actual_y_ - target_y;
            error_msg.theta_error = rigid2d::normalize_angle(actual_theta_ - target_theta);
            error_pub_.publish(error_msg);

            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::Subscriber pose_sub_;
    ros::Publisher vel_control_;
    // create a publisher to publish error
    ros::Publisher error_pub_;
    double actual_x_;
    double actual_y_;
    double actual_theta_;
    std::vector<rigid2d::Vector2D> waypoints_;
    rigid2d::Waypoints traj_generator;
    ros::ServiceClient set_pen_client_;
    ros::ServiceClient teleport_turtle_client_;

    double wheel_base_;
    double wheel_radius_;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "turtle_way");
    ros::NodeHandle nh;
    TurtleWay turtle = TurtleWay(nh);
    turtle.pipeline();

    ros::spin();
    return 0;
}