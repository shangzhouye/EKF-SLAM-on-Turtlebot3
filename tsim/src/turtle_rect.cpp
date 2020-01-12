#include "ros/ros.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"
#define PI 3.14159265358979323846 /* pi */

// four states defined
// 1. SW: go straight (width)
// 2. RW: rotate after go straight (width)
// 3. SH: go straight (height)
// 4. RH: rotate after go straight (height)

enum CurrentState
{
    SW,
    RW,
    SH,
    RH
};

// control the turtle to go straight
int go_straight(double length, double velocity, double frequency, ros::Publisher *pub)
{
    int times = (int)(length / velocity * frequency);
    ros::Rate rate(frequency);

    for (int i = 0; i < times; i++)
    {
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = velocity;
        control_msg.angular.z = 0;
        pub->publish(control_msg);
        rate.sleep();
    }

    return 0;
}

// control the turtle to rotate
int rotate(double degrees, double velocity, double frequency, ros::Publisher *pub)
{
    int times = (int)(degrees / velocity * frequency);
    ros::Rate rate(frequency);

    for (int i = 0; i < times; i++)
    {
        geometry_msgs::Twist control_msg;
        control_msg.linear.x = 0;
        control_msg.angular.z = velocity;
        pub->publish(control_msg);
        rate.sleep();
    }

    return 0;
}

// initialize the turle to the starting position
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

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle nh;
    ROS_INFO("Node has been started.");

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

    // set the if_reset parameter at the ROS parameter server level, so it can be modified by the service
    // and is visiable to the state machine
    nh.setParam("/if_reset", 1);

    // cmd_vel publisher
    ros::Publisher vel_control =
        nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    CurrentState current_state = SW;
    int if_reset = 1;

    // define a msg to stop the turtle
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0;
    stop_msg.angular.z = 0;

    while (ros::ok())
    {
        // first level of the state machine: reset the turtle or not?
        nh.getParam("/if_reset", if_reset);
        switch (if_reset)
        {
        case 1:
            vel_control.publish(stop_msg);
            initialize_turtle(&nh, x, y);
            // wait for 1 sec for the turtle to actually teleport to the right position
            ros::Duration(1).sleep();
            nh.setParam("/if_reset", 0);
            current_state = SW;
            break;

        case 0:
            switch (current_state)
            {
            case RH:
                rotate(PI / 2.0, rot_vel, frequency, &vel_control);
                current_state = SW;
                break;

            case SW:
                go_straight(width, trans_vel, frequency, &vel_control);
                current_state = RW;
                break;

            case RW:
                rotate(PI / 2.0, rot_vel, frequency, &vel_control);
                current_state = SH;
                break;

            case SH:
                go_straight(height, trans_vel, frequency, &vel_control);
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
    }

    ros::spin();
    return 0;
}