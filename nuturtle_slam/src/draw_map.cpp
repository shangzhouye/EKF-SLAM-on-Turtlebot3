/// \file
/// \brief Node for drawing landmarks in Rviz
///
/// PUBLISHES:
///     map_rviz (visualization_msgs/Marker): publish markers of the map for visualization
/// SUBSCRIBERS:
///     landmarks (nuslam/TurtleMap): subscribe the center and radius of the landmarks

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nuturtle_slam/TurtleMap.h>
#include <string>

class DrawMap
{
public:
    DrawMap(ros::NodeHandle &nh)
    {
        map_pub_ = nh.advertise<visualization_msgs::MarkerArray>("map_rviz", 100, true);
        turtlemap_sub_ = nh.subscribe("landmarks", 1000, &DrawMap::sub_landmark, this);
        ros::param::get("~what_map", what_map_);

        if (what_map_ == "measurements")
        {
            publish_frame_ = "base_link";
        }
        else if (what_map_ == "groundtruth_map")
        {
            publish_frame_ = "map";
        }
        else
        {
            publish_frame_ = "map";
        }
    }

    /// \brief receive landmark positions
    void sub_landmark(const nuturtle_slam::TurtleMap &msg)
    {
        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < msg.x.size(); i++)
        {
            visualization_msgs::Marker marker_landmark = create_landmark_marker(msg.x.at(i),
                                                                                msg.y.at(i),
                                                                                msg.radius.at(i),
                                                                                publish_frame_);
            marker_array.markers.push_back(marker_landmark);
        }
        map_pub_.publish(marker_array);
        ros::spinOnce();
    }

    /// \brief create markers for landmarks
    visualization_msgs::Marker create_landmark_marker(double x, double y, double radius, std::string in_frame)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = in_frame;
        marker.header.stamp = ros::Time(0);

        marker.ns = "map_rviz";
        marker.id = marker_id_;
        marker_id_++;

        uint32_t shape = visualization_msgs::Marker::CYLINDER;
        marker.type = shape;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = radius;
        marker.scale.y = radius;

        if (what_map_ == "measurements")
        {
            marker.scale.z = 0.2;
        }
        else if (what_map_ == "groundtruth_map")
        {
            marker.scale.z = 0.4;
        }
        else
        {
            marker.scale.z = 0.3;
        }

        marker.color.r = 0.0;

        if (what_map_ == "measurements")
        {
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if (what_map_ == "groundtruth_map")
        {
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else
        {
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }

        marker.color.a = 1.0;

        // 5 Hz publish rate
        marker.lifetime = ros::Duration(1 / 4.0);

        return marker;
    }

private:
    ros::Publisher map_pub_;
    ros::Subscriber turtlemap_sub_;
    int marker_id_ = 0;
    std::string publish_frame_;
    std::string what_map_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_map");
    ros::NodeHandle nh;
    DrawMap draw_map(nh);

    ros::spin();
    return 0;
}
