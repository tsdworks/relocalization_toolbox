/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef GICP_2D_H
#define GICP_2D_H

#include <ros/ros.h>
#include <random>
#include <map>
#include <unordered_map>
#include <utils/math_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;

namespace gicp_2d
{
    class gicp_2d
    {
    public:
        gicp_2d(const float &max_correspondence_distance = 1.0f,
                const float &transformation_epsilon = 1e-6,
                const float &euclidean_fitness_epsilon = 1e-5,
                const int &max_iterations = 50,
                const int &map_occupied_threshold = 50);
        ~gicp_2d() {};

        void set_scan(const sensor_msgs::LaserScan &scan);
        void set_map(const nav_msgs::OccupancyGrid &map);

        geometry_msgs::TransformStamped match(const geometry_msgs::TransformStamped &predict);

    private:
        float max_correspondence_distance_;
        float transformation_epsilon_;
        float euclidean_fitness_epsilon_;
        int max_iterations_;
        float map_occupied_threshold_;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    };
}

#endif