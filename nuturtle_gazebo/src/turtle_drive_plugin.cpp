/// \file
/// \brief Plugin for sending joint velocities to control the robot in Gazebo, and get joint states from Gazebo
///
/// PUBLISHES:
///     sensor_data (nuturtlebot/SensorData): publish to the sensor (encoder) data
/// SUBSCRIBES:
///     wheel_cmd (nuturtlebot/WheelCommands): subscribe wheel command to move the robot in Gazebo

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include "ros/ros.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include <unordered_map>

namespace gazebo
{
static constexpr double PI = 3.14159265358979323846;

class SetJointVelocityPlugin : public ModelPlugin
{

public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&SetJointVelocityPlugin::Update, this, std::placeholders::_1));

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL("A ROS node for Gazebo has not been initialized."
                      "Unable to load plugin. Load the Gazebo system plugin"
                      "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
            return;
        }

        ros::NodeHandle nh("gazebo_client");

        // get sensor frequency
        if (!_sdf->HasElement("sensor_frequency"))
        {
            ROS_INFO("No sensor frequency specified.");
        }
        else
        {
            this->sensor_frequency_ = _sdf->GetElement("sensor_frequency")->Get<double>();
        }

        // get wheel joint
        if (!_sdf->HasElement("left_wheel_joint"))
        {
            ROS_ASSERT("Plugin missing joint names.");
        }
        else
        {
            sdf::ElementPtr element = _sdf->GetElement("left_wheel_joint");
            this->left_wheel_joint_ = element->Get<std::string>();
        }

        if (!_sdf->HasElement("right_wheel_joint"))
        {
            ROS_ASSERT("Plugin missing joint names.");
        }
        else
        {
            sdf::ElementPtr element = _sdf->GetElement("right_wheel_joint");
            this->right_wheel_joint_ = element->Get<std::string>();
        }

        // get topics
        if (!_sdf->HasElement("sensor_data_topic"))
        {
            ROS_ASSERT("No sensor topic specified.");
        }
        else
        {
            sdf::ElementPtr element = _sdf->GetElement("sensor_data_topic");
            this->sensor_data_topic_ = element->Get<std::string>();
        }

        if (!_sdf->HasElement("wheel_cmd_topic"))
        {
            ROS_ASSERT("No wheel command topic specified.");
        }
        else
        {
            sdf::ElementPtr element = _sdf->GetElement("wheel_cmd_topic");
            this->wheel_cmd_topic_ = element->Get<std::string>();
        }

        this->wheel_sub_ = nh.subscribe(wheel_cmd_topic_, 1000, &SetJointVelocityPlugin::callback_wheel_cmd, this);

        this->sensor_data_pub_ = nh.advertise<nuturtlebot::SensorData>(sensor_data_topic_, 10);

        nh.getParam("/max_trans_vel", max_trans_vel_);
        nh.getParam("/max_rot_vel", max_rot_vel_);
        nh.getParam("/max_mot_vel", max_mot_vel_);
        nh.getParam("/encoder_ticks_per_rev", encoder_ticks_per_rev_);
    }

    void callback_wheel_cmd(const nuturtlebot::WheelCommands &msg)
    {
        wheel_command_ = msg;
        // std::cout << "Subscribed left/right wheel velocity (rad/s): " << msg.left_velocity << " / " << msg.right_velocity << std::endl;
    }

public:
    double normalize_angle(double rad)
    {
        double result = std::remainder(rad, 2.0 * PI);
        return result;
    }

    void Update(const common::UpdateInfo &_info)
    {
        double left_vel = wheel_command_.left_velocity / 256.0 * max_mot_vel_;
        double right_vel = wheel_command_.right_velocity / 256.0 * max_mot_vel_;

        // std::cout << "Left/Right wheel velocity (rad/s): " << left_vel << " / " << right_vel << std::endl;

        // Joint velocity using joint motors
        // left wheel
        this->model->GetJoint(left_wheel_joint_)->SetParam("fmax", 0, 100.0);
        this->model->GetJoint(left_wheel_joint_)->SetParam("vel", 0, left_vel);

        // right wheel
        this->model->GetJoint(right_wheel_joint_)->SetParam("fmax", 0, 100.0);
        this->model->GetJoint(right_wheel_joint_)->SetParam("vel", 0, right_vel);

        ros::Time current_time = ros::Time::now();
        double time_gap = (current_time - last_time_).toSec();

        if (time_gap >= (1.0 / sensor_frequency_))
        {
            last_time_ = current_time;
            double left_pos = normalize_angle(this->model->GetJoint(left_wheel_joint_)->Position());
            double right_pos = normalize_angle(this->model->GetJoint(right_wheel_joint_)->Position());

            // std::cout << "Left/Right Position is: " << left_pos << " / " << right_pos << std::endl;

            nuturtlebot::SensorData sensor_data;
            sensor_data.left_encoder = (left_pos / (2 * PI)) * encoder_ticks_per_rev_;
            sensor_data.right_encoder = (right_pos / (2 * PI)) * encoder_ticks_per_rev_;

            sensor_data_pub_.publish(sensor_data);
        }
    }

public:
    physics::ModelPtr model;

public:
    event::ConnectionPtr updateConnection;

private:
    ros::Subscriber wheel_sub_;
    ros::Publisher sensor_data_pub_;
    nuturtlebot::WheelCommands wheel_command_;
    double max_trans_vel_;
    double max_rot_vel_;
    double max_mot_vel_;
    double encoder_ticks_per_rev_;
    std::string right_wheel_joint_;
    std::string left_wheel_joint_;
    double sensor_frequency_ = 200;

    std::string sensor_data_topic_;
    std::string wheel_cmd_topic_;

    ros::Time last_time_ = ros::Time::now();
};

GZ_REGISTER_MODEL_PLUGIN(SetJointVelocityPlugin)
} // namespace gazebo