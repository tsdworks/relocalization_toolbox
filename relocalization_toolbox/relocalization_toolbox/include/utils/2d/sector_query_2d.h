/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef SECTOR_QUERY_2D_H
#define SECTOR_QUERY_2D_H

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

// if to use fenwick tree
#define USE_FENWICK_TREE 1

using namespace std;

namespace sector_query_2d
{
    class sector_query_2d
    {
    public:
        sector_query_2d(const int &sector_sample_num = 1200);
        ~sector_query_2d() {};

        void add_range(const float &angle, const float &range);

        float get_sum(const float &angle_start, const float &angle);
        float get_mean(const float &angle_start, const float &angle);

    private:
        int sector_sample_num_;
        vector<float> tree_;
        vector<float> data_;

        int angle_to_index(const float &angle);

        void update(int index, float value);
        float query(int index);
    };
}

#endif