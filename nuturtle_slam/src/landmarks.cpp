/// \file
/// \brief Node for detecting and publishing landmarks
///
/// PUBLISHES:
///     landmarks (nuslam/TurtleMap): publish the center and radius of the landmarks the node detects
///     clusters (visualization_msgs/Marker): publish markers of clustered points
/// SUBSCRIBERS:
///     scan (sensor_msgs/LaserScan): subscribe to the laser data

#include <iostream>
#include "ros/ros.h"
#include <string>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nuturtle_slam/TurtleMap.h>
#include <algorithm>

// circle data structure
struct Circle
{
    double radius;
    Eigen::Vector2d center;

    Circle(double r, double x, double y) : radius(r), center(x, y) {}
};

class LandmarkDetection
{
public:
    LandmarkDetection(ros::NodeHandle &nh)
    {
        scan_sub_ = nh.subscribe("scan", 1000, &LandmarkDetection::save_scan_callback, this);
        cluster_pub_ = nh.advertise<visualization_msgs::MarkerArray>("clusters", 100, true);
        circles_pub_ = nh.advertise<nuturtle_slam::TurtleMap>("landmarks", 100, true);

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

    /// \brief save the scan from the lidar, cluste the points, call circle fitting algorithm
    void save_scan_callback(const sensor_msgs::LaserScan &msg)
    {
        // std::cout << "Callback called" << std::endl;
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
        // insert the points in the last custer into the first cluster
        // remove the last cluster
        if (distance(point_clusters_.front().front(), point_clusters_.back().back()) < threshold_)
        {
            // insert the first vector to the last one to ensure the point cluster is in sequence
            // std::cout << "Same cluster" << std::endl;
            point_clusters_.back().insert(point_clusters_.back().end(), point_clusters_.front().begin(), point_clusters_.front().end());
            point_clusters_.erase(point_clusters_.begin());
        }

        // remove the cluster if there are less than three points
        point_clusters_.erase(std::remove_if(
                                  point_clusters_.begin(), point_clusters_.end(),
                                  [](const std::vector<Eigen::Vector2d> &x) {
                                      return x.size() <= 3;
                                  }),
                              point_clusters_.end());

        // std::cout << "Numbers of clusters: " << point_clusters_.size() << std::endl;

        visualization_msgs::MarkerArray marker_array;
        color_id_ = 0;
        for (auto const &cl : point_clusters_)
        {
            for (auto const &point : cl)
            {
                visualization_msgs::Marker marker = publish_marker_clustered(point(0), point(1));
                marker_array.markers.push_back(marker);
            }
            // std::cout << "Size of cluster: " << cl.size() << std::endl;
            color_id_ += 1;
            if (color_id_ == 8)
            {
                color_id_ = 0;
            }
        }
        cluster_pub_.publish(marker_array);

        // put all the clusters into the circle fitting algorithm
        nuturtle_slam::TurtleMap turtle_map;

        for (auto clu : point_clusters_)
        {
            if (!circle_classification(clu))
            {

                // std::cout << " Not circle" << std::endl;
                continue;
            }

            Circle circle_result = circle_fitting(clu);

            // remove circles with too large or small radius
            if (circle_result.radius >= radius_threshold_small_ &&
                circle_result.radius <= radius_threshold_large_)
            {
                turtle_map.x.emplace_back(circle_result.center(0));
                turtle_map.y.emplace_back(circle_result.center(1));
                turtle_map.radius.emplace_back(circle_result.radius);
            }
        }

        circles_pub_.publish(turtle_map);
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
        marker.lifetime = ros::Duration(1 / 5.0);

        return marker;
    }

    /// \brief circular fitting algorithm
    ///
    ///     A. Al-Sharadqah and N. Chernov, Error Analysis for Circle Fitting Algorithms,
    ///     Electronic Journal of Statistics (2009), Volume 3 p 886-911
    /// \param points - the points to be fitted with a circle
    /// \return the circle with radius and center point
    Circle circle_fitting(std::vector<Eigen::Vector2d> points)
    {
        // compute the mean x and y
        double x_mean = 0;
        double y_mean = 0;
        for (auto const &pt : points)
        {
            x_mean += pt(0);
            y_mean += pt(1);
        }
        x_mean = x_mean / static_cast<double>(points.size());
        y_mean = y_mean / static_cast<double>(points.size());

        // std::cout << "Mean: " << x_mean << " and " << y_mean << std::endl;

        // shift the centroid of data points
        for (auto &pt : points)
        {
            pt(0) -= x_mean;
            pt(1) -= y_mean;
        }

        // std::cout << "Shifted first point: " << points.at(0)(0) << " and " << points.at(0)(1) << std::endl;

        // compute z and z_mean
        std::vector<double> z;
        for (auto &pt : points)
        {
            z.push_back(std::pow(pt(0), 2) + std::pow(pt(1), 2));
        }

        double z_mean = 0;
        for (auto const &z_i : z)
        {
            z_mean += z_i;
        }
        z_mean = z_mean / static_cast<double>(z.size());
        // std::cout << "First z: " << z.at(0) << std::endl;
        // std::cout << "Z mean: " << z_mean << std::endl;

        // form data matrix Z
        Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(points.size(), 4);

        for (int i = 0; i < points.size(); i++)
        {
            Eigen::VectorXd vec(4);
            vec << z.at(i), points.at(i)(0), points.at(i)(1), 1;
            Z.row(i) = vec.transpose();
        }

        // std::cout << "Z Matrix: \n"
        //           << Z << std::endl;

        // form data matrix M

        Eigen::MatrixXd M = (1 / static_cast<double>(points.size())) * Z.transpose() * Z;

        // std::cout << "M Matrix: \n"
        //           << M << std::endl;

        // form H and H_inv
        Eigen::Matrix4d H, H_inv;
        H << 8 * z_mean, 0, 0, 2,
            0, 1, 0, 0,
            0, 0, 1, 0,
            2, 0, 0, 0;

        H_inv << 0, 0, 0, (1.0 / 2.0),
            0, 1, 0, 0,
            0, 0, 1, 0,
            (1.0 / 2.0), 0, 0, -2 * z_mean;

        // std::cout << "H Matrix: \n"
        //           << H << std::endl;

        // std::cout << "H_inv Matrix: \n"
        //           << H_inv << std::endl;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd sigular_values = svd.singularValues();
        auto U = svd.matrixU();
        auto V = svd.matrixV();

        if (sigular_values(3) > 10e-12)
        {

            Eigen::Matrix<double, 4, 4> sigma = sigular_values.array().matrix().asDiagonal();

            // std::cout << "sigma Matrix: \n"
            //           << sigma << std::endl;

            // std::cout << "V Matrix: \n"
            //           << V << std::endl;

            Eigen::MatrixXd Y = V * sigma * V.transpose();

            // std::cout << "Y Matrix: \n"
            //           << Y << std::endl;

            // form Q matrix
            Eigen::MatrixXd Q = Y * H_inv * Y;
            // std::cout << "Q Matrix: \n"
            //           << Q << std::endl;

            // fine the eigenvector corresponding to the smallest positive eigenvalue
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q);
            Eigen::VectorXd ev = es.eigenvalues();

            double smallest_ev = 99999;
            double smallest_id = 0;
            for (int i = 0; i < ev.size(); i++)
            {
                if (ev[i] > 0 && ev[i] < smallest_ev)
                {
                    smallest_ev = ev[i];
                    smallest_id = i;
                }
            }
            Eigen::MatrixXd A_star = es.eigenvectors().col(smallest_id);

            // std::cout << "A_star Matrix: \n"
            //           << A_star << std::endl;

            // solve YA = A_star
            Eigen::MatrixXd A = Y.colPivHouseholderQr().solve(A_star);

            // std::cout << "A Matrix: \n"
            //           << A << std::endl;

            double a = (-A(1)) / (2 * A(0));
            double b = (-A(2)) / (2 * A(0));
            double R = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2)));

            // std::cout << "Radius " << R << "; a " << a << "; b " << b << std::endl;
            // std::cout << "x " << a + x_mean << "; y " << b + y_mean << std::endl;

            return Circle(R, a + x_mean, b + y_mean);
        }

        else
        {
            Eigen::MatrixXd A = V.col(3);

            double a = (-A(1)) / (2 * A(0));
            double b = (-A(2)) / (2 * A(0));
            double R = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2)));

            // std::cout << "Radius " << R << "; a " << a << "; b " << b << std::endl;
            // std::cout << "x " << a + x_mean << "; y " << b + y_mean << std::endl;

            return Circle(R, a + x_mean, b + y_mean);
        }
    }

    /// \brief circle classification algorithm
    ///
    ///     J. Xavier et. al., Fast line, arc/circle and leg detection from laser
    ///     scan data in a Player driver, ICRA 2005
    bool circle_classification(const std::vector<Eigen::Vector2d> &points)
    {
        Eigen::Vector2d P1 = points.front();
        Eigen::Vector2d P2 = points.back();
        int size = points.size();
        std::vector<double> angles;
        double sum = 0;

        for (int i = 1; i < size - 1; i++)
        {
            Eigen::Vector2d P = points.at(i);

            // calculate the angle using cosine law
            double c = distance(P1, P2);
            double a = distance(P1, P);
            double b = distance(P2, P);
            double angle = std::acos((std::pow(a, 2) + std::pow(b, 2) - std::pow(c, 2)) / (2 * a * b));
            angles.push_back(angle);
            sum += angle;
        }

        double mean = sum / (size - 2);

        double variance = 0;
        double std = 0;

        for (double &val : angles)
        {
            variance += pow(val - mean, 2);
        }

        variance = variance / 5;

        std = sqrt(variance);

        // std::cout << "Mean: " << mean << " STD: " << std << std::endl;

        if (std < 0.5 && mean > 1.5708 && mean < 2.35619)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    ros::Subscriber scan_sub_;
    std::vector<std::vector<Eigen::Vector2d>> point_clusters_;
    double threshold_ = 0.09;

    int marker_id_ = 0;

    ros::Publisher cluster_pub_;
    ros::Publisher circles_pub_;

    std::vector<Eigen::Vector3d> colors;
    int color_id_ = 0;

    double radius_threshold_large_ = 0.11;
    double radius_threshold_small_ = 0.01;
};

int main(int argc, char **argv)
{
    // init the node
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    LandmarkDetection my_landmark_detector = LandmarkDetection(nh);

    // testing circular fitting algorithm
    // std::vector<Eigen::Vector2d> test_points;
    // test_points.push_back(Eigen::Vector2d(1, 7));
    // test_points.push_back(Eigen::Vector2d(2, 6));
    // test_points.push_back(Eigen::Vector2d(5, 8));
    // test_points.push_back(Eigen::Vector2d(7, 7));
    // test_points.push_back(Eigen::Vector2d(9, 5));
    // test_points.push_back(Eigen::Vector2d(3, 7));
    // my_landmark_detector.circle_fitting(test_points);

    // std::vector<Eigen::Vector2d> test_points2;
    // test_points2.push_back(Eigen::Vector2d(-1, 0));
    // test_points2.push_back(Eigen::Vector2d(-0.3, -0.06));
    // test_points2.push_back(Eigen::Vector2d(0.3, 0.1));
    // test_points2.push_back(Eigen::Vector2d(1, 0));
    // my_landmark_detector.circle_fitting(test_points2);

    ros::spin();
    return 0;
}