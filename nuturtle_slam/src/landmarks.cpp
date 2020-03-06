/// \file
/// \brief Node for detecting and publishing landmarks
///
/// PUBLISHES:
///     landmarks (nuslam/TurtleMap): publish the center and radius of the landmarks the node detects
///     clustered points (visualization_msgs/Marker): publish markers of clustered points
/// SUBSCRIBERS:
///     scan (sensor_msgs/LaserScan): subscribe to the laser data

#include <iostream>
#include "ros/ros.h"
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/LaserScan.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class LandmarkDetection
{
public:
    LandmarkDetection(ros::NodeHandle &nh)
    {
        scan_sub_ = nh.subscribe("scan", 1000, &LandmarkDetection::save_scan_callback, this);
        cluster_pub_ = nh.advertise<visualization_msgs::MarkerArray>("clusters", 100, true);

        // create eight colors for clusters
        colors.push_back(Eigen::Vector3d(0, 0, 0));
        colors.push_back(Eigen::Vector3d(1, 0, 0));
        colors.push_back(Eigen::Vector3d(0, 1, 0));
        colors.push_back(Eigen::Vector3d(0, 0, 1));
        colors.push_back(Eigen::Vector3d(1, 1, 0));
        colors.push_back(Eigen::Vector3d(1, 0, 1));
        colors.push_back(Eigen::Vector3d(0, 1, 1));
        colors.push_back(Eigen::Vector3d(1, 1, 1));
    }

    /// \brief save the scan from the lidar
    void save_scan_callback(const sensor_msgs::LaserScan &msg)
    {
        std::cout << "Callback called" << std::endl;
        int i = 0;
        while (i < 360)
        {
            // create a new cluster
            std::vector<Eigen::Vector2d> cluster;
            // put the ith point into a new vector
            cluster.push_back(Eigen::Vector2d(bearing_to_pose(i, msg.ranges.at(i))));
            i++;

            while (i < 360)
            {
                // if distance between two points are less than the threshold, then they are the same cluster
                double dist = distance(Eigen::Vector2d(bearing_to_pose(i - 1, msg.ranges.at(i - 1))),
                                       Eigen::Vector2d(bearing_to_pose(i, msg.ranges.at(i))));
                if (dist < threshold_)
                {
                    cluster.push_back(Eigen::Vector2d(bearing_to_pose(i, msg.ranges.at(i))));
                    i++;
                }
                else
                {
                    break;
                }
            }

            // insert the vector into point_clusters_
            point_clusters_.push_back(cluster);
        }

        // if last component in last vector
        // and first component in first vector are close to each other
        // combine those two vectors
        if (distance(point_clusters_.front().front(), point_clusters_.back().back()) < threshold_)
        {
            std::vector<Eigen::Vector2d> cluster;
            cluster.reserve(point_clusters_.front().size() + point_clusters_.back().size());
            cluster.insert(cluster.end(), point_clusters_.front().begin(), point_clusters_.front().end());
            cluster.insert(cluster.end(), point_clusters_.back().begin(), point_clusters_.back().end());

            point_clusters_.pop_back();
            point_clusters_.erase(point_clusters_.begin()); // not ideal to remove a component at front in a vector
            point_clusters_.push_back(cluster);
        }

        // remove the cluster if there are less than three points
        point_clusters_.erase(std::remove_if(
                                  point_clusters_.begin(), point_clusters_.end(),
                                  [](const std::vector<Eigen::Vector2d> &x) {
                                      return x.size() <= 3;
                                  }),
                              point_clusters_.end());

        std::cout << "Numbers of clusters: " << point_clusters_.size() << std::endl;

        visualization_msgs::MarkerArray marker_array;
        color_id_ = 0;
        for (auto const &cl : point_clusters_)
        {
            for (auto const &point : cl)
            {
                visualization_msgs::Marker marker = publish_marker_clustered(point(0), point(1));
                marker_array.markers.push_back(marker);
            }
            std::cout << "Size of cluster: " << cl.size() << std::endl;
            color_id_ += 1;
            if (color_id_ == 8)
            {
                color_id_ = 0;
            }
        }
        cluster_pub_.publish(marker_array);

        ros::spinOnce();

        point_clusters_.clear();
    }

    /// \brief calculate the position of the laser scan point in robot frame
    Eigen::Vector2d bearing_to_pose(int deg, double dist)
    {
        double x = dist * std::cos(rigid2d::deg2rad(static_cast<double>(deg)));
        double y = dist * std::sin(rigid2d::deg2rad(static_cast<double>(deg)));

        return Eigen::Vector2d(x, y);
    }

    /// \brief calculate the distance between two points
    double distance(Eigen::Vector2d p1, Eigen::Vector2d p2)
    {
        return std::sqrt(std::pow((p1(0) - p2(0)), 2) + std::pow((p1(1) - p2(1)), 2));
    }

    /// \brief publish markers for clustered points
    visualization_msgs::Marker publish_marker_clustered(double x, double y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time(0);

        marker.ns = "clusters";
        marker.id = marker_id_;
        marker_id_++;

        uint32_t shape = visualization_msgs::Marker::SPHERE;
        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.r = colors.at(color_id_)(0);
        marker.color.g = colors.at(color_id_)(1);
        marker.color.b = colors.at(color_id_)(2);

        marker.color.a = 1.0;
        
        // 5 Hz publish rate
        marker.lifetime = ros::Duration(1/5.0);

        return marker;
    }

private:
    ros::Subscriber scan_sub_;
    std::vector<std::vector<Eigen::Vector2d>> point_clusters_;
    double threshold_ = 0.07;

    int marker_id_ = 0;

    ros::Publisher cluster_pub_;

    std::vector<Eigen::Vector3d> colors;
    int color_id_ = 0;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    LandmarkDetection my_landmark_detector = LandmarkDetection(nh);

    ros::spin();
    return 0;
}