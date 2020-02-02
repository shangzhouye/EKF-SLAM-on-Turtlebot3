/// \file
/// \brief Make the turtle move in a rectangular trajectory.
///
/// PARAMETERS:
///     x (double): The x coordinate of the lower left corner of a rectangle
///     y (double): The y coordinate of the lower left corner of a rectangle
///     width (double): The width of the rectangle
///     height (double): The height of the rectangle
///     trans_vel (double): The translational velocity of the robot
///     rot_vel (double): The rotational velocity of the robot
///     frequency (int): The frequency of the control loop
/// PUBLISHES:
///     pose_error (tsim::PoseError): Publish the error of feedforward control
///     turtle1/cmd_vel (geometry_msgs::Twist): Publish the control commands to move the turtle
/// SUBSCRIBES:
///     turtle1/pose (turtlesim::Pose): The current actual pose (x, y, heading) of the turtle
/// SERVICES:
///     traj_reset (std_srvs::Empty): Reset the trajectory

#include <cmath>
#include "ros/ros.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "tsim/PoseError.h"
const double PI = 3.14159265358979323846;

// define global variables
// better to wrap everything into a class
static double actual_x;
static double actual_y;
static double actual_theta;
static int if_reset;
static double target_x, target_y, target_theta;

/// \brief four states defined
enum CurrentState
{
    /// SW: go straight (width)
    SW,
    /// RW: rotate after go straight (width)
    RW,
    /// SH: go straight (height)
    SH,
    /// RH: rotate after go straight (height)
    RH
};

/// \brief Warp the angle between -pi and pi
///
/// \param x - The angle to be wrapped
/// \returns Wrapped angle
double angle_wrapping(double x)
{
    x = std::fmod(x + PI, PI * 2);
    if (x < 0)
        x += PI * 2;
    return x - PI;
}

/// \brief Control the turtle to go straight
///
/// \param length - The length of this leg
/// \param velocity - Moving velocity
/// \param frequency - Publishing frequency
int go_straight(double length, double velocity, double frequency, ros::Publisher *pub, ros::Publisher *error_pub)
{
    int times = (int)(length / velocity * frequency);
    ros::Rate rate(frequency);

    for (int i = 0; i < times; i++)
    {
        ros::spinOnce();
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = velocity;
        control_msg.angular.z = 0;
        pub->publish(control_msg);
        target_x = target_x + (1.0 / frequency) * velocity * std::cos(target_theta);
        // ROS_INFO("target_x is %f", target_x);
        // ROS_INFO("actual_x is %f", actual_x);
        // ROS_INFO("error_x is %f", actual_x - target_x);
        target_y = target_y + (1.0 / frequency) * velocity * std::sin(target_theta);
        tsim::PoseError error_msg;
        error_msg.x_error = actual_x - target_x;
        error_msg.y_error = actual_y - target_y;
        error_msg.theta_error = angle_wrapping(actual_theta - target_theta);
        error_pub->publish(error_msg);
        rate.sleep();
    }

    return 0;
}

/// \brief Control the turtle to rotate
///
/// \param degrees - The degrees to rotate
/// \param velocity - Rotating velocity
/// \param frequency - Publishing frequency
int rotate(double degrees, double velocity, double frequency, ros::Publisher *pub, ros::Publisher *error_pub)
{
    int times = (int)(degrees / velocity * frequency);
    ros::Rate rate(frequency);

    for (int i = 0; i < times; i++)
    {
        ros::spinOnce();
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = 0;
        control_msg.angular.z = velocity;
        pub->publish(control_msg);
        target_theta = angle_wrapping(target_theta + (1.0 / frequency) * velocity);
        tsim::PoseError error_msg;
        error_msg.x_error = actual_x - target_x;
        error_msg.y_error = actual_y - target_y;
        error_msg.theta_error = angle_wrapping(actual_theta - target_theta);
        error_pub->publish(error_msg);
        rate.sleep();
    }

    return 0;
}

/// \brief Initialize the turle to the starting position
///
/// \param nh - Node handle object
/// \param x - Initial x position
/// \param y - Initial y position
int initialize_turtle(ros::NodeHandle *nh, double x, double y)
{
    // lift the pen
    ros::ServiceClient set_pen_client = nh->serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    turtlesim::SetPen set_pen_req;
    set_pen_req.request.r = 255;
    set_pen_req.request.g = 255;
    set_pen_req.request.b = 255;
    set_pen_req.request.width = 2;
    set_pen_req.request.off = true;

    set_pen_client.waitForExistence();
    if (set_pen_client.call(set_pen_req))
    {
        ROS_INFO("Pen lifted");
    }

    // teleport the turtle
    ros::ServiceClient teleport_turtle_client =
        nh->serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleport_absolute_req;
    teleport_absolute_req.request.x = x;
    teleport_absolute_req.request.y = y;
    teleport_absolute_req.request.theta = 0;

    teleport_turtle_client.waitForExistence();
    if (teleport_turtle_client.call(teleport_absolute_req))
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

    set_pen_client.waitForExistence();
    if (set_pen_client.call(set_pen_down))
    {
        ROS_INFO("Put pen down");
    }

    target_x = x;
    target_y = y;
    target_theta = 0;

    return 0;
}

/// \brief traj_reset service handle function
bool handle_traj_reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if_reset = 1;
    return true;
}

/// \brief the callback function to save the actual pose
void callback_save_pose(const turtlesim::Pose &msg)
{
    actual_x = msg.x;
    actual_y = msg.y;
    actual_theta = msg.theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle nh;
    ROS_INFO("Node has been started.");

    // create a service to reset the turtle
    ros::ServiceServer traj_reset_server = nh.advertiseService("/traj_reset", handle_traj_reset);

    // create a subscriber to subscribe the actual pose
    ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1000, callback_save_pose);

    // create a publisher to publish error
    ros::Publisher error_pub = nh.advertise<tsim::PoseError>("pose_error", 10);

    // read parameters from the parameter server
    double x, y, width, height, trans_vel, rot_vel;
    int frequency;
    nh.getParam("/x", x);
    nh.getParam("/y", y);
    nh.getParam("/width", width);
    nh.getParam("/height", height);
    nh.getParam("/trans_vel", trans_vel);
    nh.getParam("/rot_vel", rot_vel);
    nh.getParam("/frequency", frequency);
    ROS_INFO("\n x: %.2f \n y: %.2f \n width: %.2f \n height: %.2f \n trans_vel: %.2f \n rot_vel: %.2f \n frequency: %d \n",
             x, y, width, height, trans_vel, rot_vel, frequency);

    if_reset = 1;

    // cmd_vel publisher
    ros::Publisher vel_control =
        nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    CurrentState current_state = SW;

    // define a msg to stop the turtle
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0;
    stop_msg.angular.z = 0;

    while (ros::ok())
    {
        // first level of the state machine: reset the turtle or not?
        switch (if_reset)
        {
        // reset the turtle
        case 1:
            vel_control.publish(stop_msg);
            initialize_turtle(&nh, x, y);
            // wait for 1 sec for the turtle to actually teleport to the right position
            ros::Duration(1).sleep();
            if_reset = 0;
            current_state = SW;
            break;

        // move the turtle
        case 0:
            switch (current_state)
            {
            case RH:
                rotate(PI / 2.0, rot_vel, frequency, &vel_control, &error_pub);
                current_state = SW;
                break;

            case SW:
                go_straight(width, trans_vel, frequency, &vel_control, &error_pub);
                current_state = RW;
                break;

            case RW:
                rotate(PI / 2.0, rot_vel, frequency, &vel_control, &error_pub);
                current_state = SH;
                break;

            case SH:
                go_straight(height, trans_vel, frequency, &vel_control, &error_pub);
                current_state = RH;
                break;

            default:
                ROS_INFO("Unexpected State.");
                break;
            }
            break;

        default:
            ROS_INFO("Unexpected state.");
            break;
        }
        // Learned: ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
        ros::spinOnce();
    }

    return 0;
}