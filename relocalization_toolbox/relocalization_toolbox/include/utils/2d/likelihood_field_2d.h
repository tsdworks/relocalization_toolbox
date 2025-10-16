/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef LIKELIHOOD_FIELD_2D_H
#define LIKELIHOOD_FIELD_2D_H

#include <ros/ros.h>
#include <algorithm>
#include <random>
#include <map>
#include <unordered_map>
#include <utils/math_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;

namespace likelihood_field_2d
{
    class likelihood_field_2d
    {
    public:
        likelihood_field_2d(const float &sigma_hit = 0.2f, const float &z_hit = 1.0f, const float &z_rand = 0.0f, const int &map_occupied_threshold = 50);
        ~likelihood_field_2d() {};

        tuple<float, float, float> get_likelihood_field_score(const geometry_msgs::TransformStamped &tf_map_to_lidar,
                                                              const sensor_msgs::LaserScan &scan,
                                                              const nav_msgs::OccupancyGrid &map,
                                                              const vector<float> &obs_dist_table,
                                                              const int &lidar_sampling_step,
                                                              const bool &enable_visualization = false);
        tuple<float, float, float> get_likelihood_field_score(const geometry_msgs::TransformStamped &tf_map_to_lidar,
                                                              const sensor_msgs::LaserScan &scan,
                                                              const nav_msgs::OccupancyGrid &map,
                                                              const int &lidar_sampling_step,
                                                              const bool &enable_visualization = false);

    private:
        float sigma_hit_ = 0.2f;
        float z_hit_ = 1.0f;
        float z_rand_ = 0.0f;
        int map_occupied_threshold_ = 50;

        float gaussian_prob(float dist);

        ros::NodeHandle node_handle_;
        ros::Publisher debug_cloud_pub_;
    };
}

#endif