/// \file
/// \brief Node for the EKF-SLAM algorithn is controlled environment
///
/// PARAMETERS:
///     odom_frame_id_: frame id of the odom frame
///     body_frame_id_: frame id of the body frame
///     left_wheel_joint_: name of the left wheel joint
///     right_wheel_joint_: name of the right wheel joint
///     wheel_base_: wheel_base parameter for the diff drive robot
///     wheel_radius_: wheel radius of the diff drive robot
///     last_l_: absolute position of the left wheel in last time step
///     last_r_: absolute position of the right wheel in last time step
///     current_l_: absolute position of the left wheel in current time step
///     current_r_: absolute position of the right wheel in current time step
///     last_time_now_: time stamp of last time step
/// PUBLISHES:
///     nav_odo (nav_msgs/Odometry): publish current robot pose
/// SUBSCRIBES:
///     joint_state (sensor_msgs/JointState): the joint states of l/r wheels
///     fake_landmarks (nuslam/TurtleMap): subscribe the fake landmarks with known data associations

#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "ros/ros.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"
#include <string>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include <nuturtle_slam/TurtleMap.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class SLAMinControl
{

private:
    std::string odom_frame_id_;
    std::string body_frame_id_;
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber landmarks_sub_;
    ros::Publisher nav_odo_pub_;
    double wheel_base_;
    double wheel_radius_;
    rigid2d::DiffDrive my_robot_;

    // define two wheel states: last and current
    double last_l_;
    double last_r_;
    double current_l_;
    double current_r_;

    bool if_init_;

    ros::Time last_time_now_;

    Eigen::VectorXd mu_ = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd sigma_ = Eigen::MatrixXd::Zero(3, 3);

    const Eigen::Matrix<double, 3, 3> Rx_ = (Eigen::Matrix<double, 3, 3>() << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001).finished();

public:
    SLAMinControl(ros::NodeHandle &nh)
    {
        nav_odo_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odo", 10);
        joint_state_sub_ = nh.subscribe("joint_state", 1000, &SLAMinControl::joint_states_callback, this);
        landmarks_sub_ = nh.subscribe("fake_landmarks", 1000, &SLAMinControl::landmarks_callback, this);

        ros::param::get("~body_frame_id_", body_frame_id_);
        ros::param::get("~odom_frame_id_", odom_frame_id_);

        nh.getParam("/wheel_base", wheel_base_);
        nh.getParam("/wheel_radius", wheel_radius_);
        nh.getParam("/left_wheel_joint", left_wheel_joint_);
        nh.getParam("/right_wheel_joint", right_wheel_joint_);

        my_robot_ = rigid2d::DiffDrive(rigid2d::Transform2D(), wheel_base_, wheel_radius_);

        last_l_ = 0;
        last_r_ = 0;
        last_time_now_ = ros::Time::now();

        if_init_ = false;
    }

    /// \brief read messages from joint states publisher (encoder) and do the prediction
    ///     Also do correction steps if a new measurement comes
    ///     Measurements come at around 5Hz while odometry updating at higher frequency
    void joint_states_callback(const sensor_msgs::JointState &msg)
    {
        

        // initialize encoders
        if (!if_init_)
        {
            last_l_ = msg.position.at(1);
            last_r_ = msg.position.at(0);
            if_init_ = true;
        }

        // read current joint state
        current_l_ = msg.position.at(1);
        current_r_ = msg.position.at(0);

        // convert to twist
        rigid2d::Twist2D twist;
        double dist_l = 0;
        double dist_r = 0;
        encoder_to_twist(twist, dist_l, dist_r);

        my_robot_.updateOdometry(dist_l, dist_r);
        Eigen::VectorXd mu_hat = mu_;
        state_prediction_mu(mu_hat);

        std::cout << "MU: " << mu_ << std::endl;
        std::cout << "MU_HAT: " << mu_hat << std::endl;

        state_prediction_sigma(twist);

        // publish robot pose tf using odom
        publish_robot_tf(mu_hat(0), mu_hat(1), mu_hat(2));

        last_l_ = current_l_;
        last_r_ = current_r_;
        mu_ = mu_hat;

        std::cout << "MU: " << mu_ << std::endl;
        std::cout << "SIGMA: " << sigma_ << std::endl;
    }

    /// \brief state prediction - update covariance
    void state_prediction_sigma(const rigid2d::Twist2D &twist)
    {
        // calculate G
        Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(sigma_.rows(), sigma_.cols());

        if (rigid2d::almost_equal(twist.omega, 0.0))
        {
            Gt(0, 2) = -twist.v_x * std::sin(mu_(2));
            Gt(1, 2) = twist.v_x * std::cos(mu_(2));
        }
        else
        {
            Gt(0, 2) = -(twist.v_x / twist.omega) * std::cos(mu_(2)) +
                       (twist.v_x / twist.omega) * std::cos(mu_(2) + twist.omega);
            Gt(1, 2) = -(twist.v_x / twist.omega) * std::sin(mu_(2)) +
                       (twist.v_x / twist.omega) * std::sin(mu_(2) + twist.omega);
        }
        Eigen::MatrixXd Rt = Eigen::MatrixXd::Zero(sigma_.rows(), sigma_.cols());
        Rt.block(0,0,3,3) = Rx_;
        sigma_ = Gt * sigma_ * Gt.transpose() + Rt;
    }

    /// \brief state prediction - update mu_hat
    void state_prediction_mu(Eigen::VectorXd &mu_hat)
    {
        rigid2d::Transform2D current_pose = my_robot_.get_pose();
        double pose_x, pose_y, pose_theta;
        current_pose.displacement(pose_x, pose_y, pose_theta);

        mu_hat(0) = pose_x;
        mu_hat(1) = pose_y;
        mu_hat(2) = pose_theta;
    }

    /// \brief convert the encoder reading to twist (similar to control input as v and w)
    void encoder_to_twist(rigid2d::Twist2D &twist, double &dist_l, double &dist_r)
    {
        dist_l = current_l_ - last_l_;
        // angle wrapping will not cause issue given the current maximum motor speed and update frequency
        dist_l = rigid2d::normalize_angle(dist_l);
        dist_r = current_r_ - last_r_;
        dist_r = rigid2d::normalize_angle(dist_r);
        rigid2d::WheelVelocities wheel_v;
        wheel_v.v_left = dist_l;
        wheel_v.v_right = dist_r;
        twist = my_robot_.wheelsToTwist(wheel_v);
    }

    /// \brief publish tf to baselink
    void publish_robot_tf(double pose_x, double pose_y, double pose_theta)
    {
        // convert orientation to quaternion
        tf2::Quaternion q_rot;
        double r = 0, p = 0, y = pose_theta;
        q_rot.setRPY(r, p, y);
        q_rot.normalize();
        // broadcast the tf
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = odom_frame_id_;
        transformStamped.child_frame_id = body_frame_id_;
        transformStamped.transform.translation.x = pose_x;
        transformStamped.transform.translation.y = pose_y;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation.x = q_rot.x();
        transformStamped.transform.rotation.y = q_rot.y();
        transformStamped.transform.rotation.z = q_rot.z();
        transformStamped.transform.rotation.w = q_rot.w();

        br.sendTransform(transformStamped);
        ros::spinOnce();
    }

    /// \brief publish nav_odo message
    void publish_nav_odo()
    {
        // nav_msgs::Odometry odo_msg;
        // odo_msg.header.frame_id = odom_frame_id_;
        // odo_msg.child_frame_id = body_frame_id_;

        // odo_msg.pose.pose.position.x = pose_x;
        // odo_msg.pose.pose.position.y = pose_y;

        // // convert orientation to quaternion
        // tf2::Quaternion q_rot;
        // double r = 0, p = 0, y = pose_theta;
        // q_rot.setRPY(r, p, y);
        // q_rot.normalize();
        // tf2::convert(q_rot, odo_msg.pose.pose.orientation);

        // // std::cout << ros::Time::now() - last_time_now_;
        // ros::Time current_time_now = ros::Time::now();
        // double vel_diff_l = current_l_ - last_l_;
        // double vel_diff_r = current_r_ - last_r_;

        // vel_diff_l = rigid2d::normalize_angle(vel_diff_l);
        // vel_diff_r = rigid2d::normalize_angle(vel_diff_r);

        // double vel_l = (vel_diff_l) / (current_time_now - last_time_now_).toSec();
        // double vel_r = (vel_diff_r) / (current_time_now - last_time_now_).toSec();

        // std::cout << "Time difference: " << (current_time_now - last_time_now_).toSec() << std::endl;
        // std::cout << "Wheel Velocities: " << vel_l << " and " << vel_r << std::endl;

        // rigid2d::WheelVelocities wheel_vel;
        // wheel_vel.v_left = vel_l;
        // wheel_vel.v_right = vel_r;
        // rigid2d::Twist2D twist = my_robot_.wheelsToTwist(wheel_vel);
        // odo_msg.twist.twist.linear.x = twist.v_x;
        // odo_msg.twist.twist.linear.y = twist.v_y;
        // odo_msg.twist.twist.angular.z = twist.omega;

        // last_time_now_ = current_time_now;

        // last_l_ = current_l_;
        // last_r_ = current_r_;

        // nav_odo_pub_.publish(odo_msg);
    }

    /// \brief save the measurements and trigger the correction
    void landmarks_callback(const nuturtle_slam::TurtleMap &msg)
    {
    }
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "slam_in_control");
    ros::NodeHandle nh;
    SLAMinControl slam_algo_in_control = SLAMinControl(nh);

    ros::spin();
    return 0;
}