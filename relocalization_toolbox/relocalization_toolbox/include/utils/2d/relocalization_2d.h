/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef RELOCALIZATION_2D_H
#define RELOCALIZATION_2D_H

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
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <utils/2d/map_io_2d.h>
#include <utils/2d/likelihood_field_2d.h>
#include <utils/2d/submap_set_2d.h>
#include <utils/2d/gicp_2d.h>

using namespace std;

namespace relocalization_2d
{
    class relocalization_2d
    {
    public:
        relocalization_2d(
            // general parameters
            const bool &enable_visualization = false, const bool &use_reachability_sampling = false,
            const float &rrt_min_expand_dist = 0.5, const float &rrt_max_expand_dist = 1.0, const int &reachability_sampling_duration = 5,
            const float &fit_score_threshold_min = 0.8, const float &fit_score_threshold_max = 0.9,
            const float &angle_search_step = 0.174, const int &area_batch_proc_number = 100, const int &area_candidate_number = 10,
            const float &min_dist_to_obstacle = 0.4,
            // likelihood field model parameters
            const float &sigma_hit = 0.2f, const float &z_hit = 1.0f, const float &z_rand = 0.0f, const float &max_tolerance_dist = 1.0f,
            // map-related parameters
            const int &map_free_threshold = 30, const int &map_occupied_threshold = 50,
            // submap partitioning parameters
            const float &sensing_radius = 10.0f, const float anchor_point_min_dist = 3.0f,
            // ICP matching parameters
            const float &max_correspondence_distance = 1.0f, const float &transformation_epsilon = 1e-6, const float &euclidean_fitness_epsilon = 1e-5, const int &max_iterations = 50);
        ~relocalization_2d() {};

        void set_map(const nav_msgs::OccupancyGrid &map);

        tuple<bool, float, geometry_msgs::TransformStamped> relocalize(
            const sensor_msgs::LaserScan &scan,
            const bool &lidar_reverted,
            const int &lidar_sampling_step,
            const nav_msgs::OccupancyGrid &map);

    private:
        // general parameters
        bool enable_visualization_;

        bool use_reachability_sampling_;
        float rrt_min_expand_dist_, rrt_max_expand_dist_;
        int reachability_sampling_duration_;

        float fit_score_threshold_min_;
        float fit_score_threshold_max_;
        float angle_search_step_;
        int area_batch_proc_number_;
        int area_candidate_number_;
        float min_dist_to_obstacle_;

        // likelihood field model parameters
        float sigma_hit_;
        float z_hit_;
        float z_rand_;
        float max_tolerance_dist_;

        // map-related parameters
        int map_free_threshold_;
        int map_occupied_threshold_;

        // submap partitioning parameters
        float sensing_radius_;
        float anchor_point_min_dist_;

        // ICP matching parameters
        float max_correspondence_distance_;
        float transformation_epsilon_;
        float euclidean_fitness_epsilon_;
        int max_iterations_;

        // related data
        bool lidar_reverted_;
        int lidar_sampling_step_;
        sensor_msgs::LaserScan scan_data_;
        nav_msgs::OccupancyGrid map_data_;
        vector<float> obs_dist_table_global_;
        vector<tuple<geometry_msgs::Point32, nav_msgs::OccupancyGrid, vector<float>>> submap_set_raw_;
        vector<tuple<geometry_msgs::Point32, nav_msgs::OccupancyGrid, float>> submap_set_;

        unique_ptr<submap_set_2d::submap_set_2d> submap_set_2d_instance_;
        unique_ptr<likelihood_field_2d::likelihood_field_2d> likelihood_field_2d_instance_;
        unique_ptr<gicp_2d::gicp_2d> gicp_2d_instance_;

        ros::NodeHandle node_handle_;
        ros::Publisher submap_pub_;

        float calc_confidence(
            const nav_msgs::OccupancyGrid &map,
            const float &likelihood_field_score, const float &mean_min_dist, const float &consistency_score);
    };
}

#endif