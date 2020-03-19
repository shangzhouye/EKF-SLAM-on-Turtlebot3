/// \file
/// \brief Generates fake data assiciation from the groundtruth with controlable noise
///
///
/// PUBLISHES:
///     fake_landmarks (nuslam/TurtleMap): publish the fake landmarks with known data associations
/// PARAMETERS:
///     Noise level: noise of the landmarks positions
///     Radius of detection: landmarks inside the radius can be detected.

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include <unordered_map>
#include <nuturtle_slam/TurtleMap.h>
#include <ignition/math/Pose3.hh>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>

using namespace gazebo;

static constexpr double PI = 3.14159265358979323846;

class GetDataAssociationPlugin : public ModelPlugin
{

public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GetDataAssociationPlugin::Update, this, std::placeholders::_1));

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL("A ROS node for Gazebo has not been initialized."
                      "Unable to load plugin. Load the Gazebo system plugin"
                      "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
            return;
        }

        ros::NodeHandle nh;

        this->data_association_pub_ = nh.advertise<nuturtle_slam::TurtleMap>("fake_landmarks", 10);
    }

public:
    void Update(const common::UpdateInfo &_info)
    {

        ros::Time current_time = ros::Time::now();
        double time_gap = (current_time - last_time_).toSec();

        if (time_gap >= (1.0 / sensor_frequency_))
        {
            last_time_ = current_time;
            auto pose = this->model->WorldPose();
            double x = pose.Pos().X();
            double y = pose.Pos().Y();
            double theta = pose.Rot().Yaw();
            // ROS_INFO("Curren pose at: %f, %f, %f", x, y, theta);
            // std::cout << "Known map" << known_map << std::endl;

            nuturtle_slam::TurtleMap turtle_map;

            for (int i = 0; i < 12; i++)
            {
                double landmark_x = known_map.row(i)(1);
                double landmark_y = known_map.row(i)(2);
                // std::cout << "Landmark x: " << landmark_x << std::endl;
                // std::cout << "Landmark y: " << landmark_y << std::endl;

                double distance = calculate_distance(landmark_x, landmark_y, x, y);
                // std::cout << "Distance: " << distance << std::endl;

                // transform back to the robot coordinate
                // Transform2D T_wr(Vector2D(x, y), theta);
                // Transform2D T_rw = T_wr.inv();
                // Vector2D landmark_r = T_rw(Vector2D(landmark_x, landmark_y));

                // manually completing the steps above
                double inv_x = -x * std::cos(theta) - y * std::sin(theta);
                double inv_y = x * std::sin(theta) - y * std::cos(theta);
                double inv_theta = -theta;
                double result_x = landmark_x * std::cos(inv_theta) - landmark_y * std::sin(inv_theta) + inv_x;
                double result_y = landmark_x * std::sin(inv_theta) + landmark_y * std::cos(inv_theta) + inv_y;

                // publish the landmarks that are within the threshold
                if (distance < distance_threshold)
                {
                    turtle_map.x.emplace_back(result_x);
                    turtle_map.y.emplace_back(result_y);
                    turtle_map.radius.emplace_back(0.07);
                    turtle_map.id.emplace_back(i);
                }
            }

            data_association_pub_.publish(turtle_map);
        }
    }

    /// \brief calculate the distance between two points
    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2));
    }

public:
    physics::ModelPtr model;

public:
    event::ConnectionPtr updateConnection;

private:
    ros::Publisher data_association_pub_;
    // known map
    const Eigen::Matrix<double, 12, 3> known_map = (Eigen::Matrix<double, 12, 3>() << 0, -0.844742, 0.664209,
                                                    1, -0.825084, 0.010533,
                                                    2, -0.81738, -0.829914,
                                                    3, -0.526104, -0.516092,
                                                    4, -0.215669, 0.688805,
                                                    5, -0.22333, -0.016393,
                                                    6, -0.215862, -0.848873,
                                                    7, 0.03497, 0.407057,
                                                    8, 0.43455, 0.449779,
                                                    9, 0.399071, -0.446748,
                                                    10, 0.719942, -0.02845,
                                                    11, 0.742731, -0.792074)
                                                       .finished();
    double distance_threshold = 1;
    double sensor_frequency_ = 5.0;
    ros::Time last_time_ = ros::Time::now();
};

GZ_REGISTER_MODEL_PLUGIN(GetDataAssociationPlugin)