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
#include <relocalization/utils/math_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <relocalization/utils/relocalization_2d/map_io_2d.h>
#include <relocalization/utils/relocalization_2d/likelihood_field_2d.h>
#include <relocalization/utils/relocalization_2d/area_set_2d.h>
#include <relocalization/utils/relocalization_2d/gicp_2d.h>

using namespace std;

namespace relocalization_2d
{
    class relocalization_2d
    {
    public:
        relocalization_2d(
            // general parameters
            const bool &enable_visualization = false, const bool &use_traversability_sampling = false,
            const float &rrt_min_expand_dist = 0.5, const float &rrt_max_expand_dist = 1.0, const int &traversability_sampling_duration = 5,
            const float &fit_score_threshold_min = 0.8, const float &fit_score_threshold_max = 0.9,
            const float &angle_search_step = 0.174, const int &area_batch_proc_number = 100, const int &area_candidate_number = 10,
            const float &min_dist_to_obstacle = 0.4,
            // likelihood field model parameters
            const float &sigma_hit = 0.2f, const float &z_hit = 1.0f, const float &z_rand = 0.0f, const float &max_tolerance_dist = 1.0f,
            const float &keep_ratio_1_min = 0.8f, const float &keep_ratio_1_max = 0.9f, const float &keep_ratio_2_min = 0.7f, const float &keep_ratio_2_max = 0.85f,
            const bool &geometric_mean_fusion = true,
            // map-related parameters
            const int &map_free_threshold = 30, const int &map_occupied_threshold = 50,
            // submap partitioning parameters
            const float &sensing_radius = 10.0f, const float anchor_point_min_dist = 3.0f,
            // ICP matching parameters
            const float &max_correspondence_distance = 1.0f, const float &transformation_epsilon = 1e-6, const float &euclidean_fitness_epsilon = 1e-5, const int &max_iterations = 50);
        ~relocalization_2d() {};

        void set_map(const nav_msgs::OccupancyGrid &map, const int &map_sampling_ratio);

        tuple<bool, vector<float>, vector<geometry_msgs::TransformStamped>> get_candidates(
            const sensor_msgs::LaserScan &scan,
            const bool &lidar_reverted,
            const int &lidar_sampling_step,
            const int &map_sampling_ratio,
            const nav_msgs::OccupancyGrid &map,
            const int &max_num_of_candidates = 1);
        tuple<bool, float, geometry_msgs::TransformStamped> relocalize(
            const sensor_msgs::LaserScan &scan,
            const bool &lidar_reverted,
            const int &lidar_sampling_step,
            const int &map_sampling_ratio,
            const nav_msgs::OccupancyGrid &map)
        {
            tuple<bool, float, geometry_msgs::TransformStamped> ret;
            get<0>(ret) = false;
            get<1>(ret) = 0.0f;

            auto cands = get_candidates(scan, lidar_reverted, lidar_sampling_step, map_sampling_ratio, map, 1);

            if (!get<1>(cands).empty() && !get<2>(cands).empty())
            {
                get<0>(ret) = get<0>(cands);
                get<1>(ret) = get<1>(cands)[0];
                get<2>(ret) = get<2>(cands)[0];
            }

            return ret;
        }

    private:
        // general parameters
        bool enable_visualization_;

        bool use_traversability_sampling_;
        float rrt_min_expand_dist_, rrt_max_expand_dist_;
        int traversability_sampling_duration_;

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
        float keep_ratio_1_min_ = 0.8f;
        float keep_ratio_1_max_ = 0.9f;
        float keep_ratio_2_min_ = 0.70f;
        float keep_ratio_2_max_ = 0.85f;
        bool geometric_mean_fusion_ = true;

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
        int map_sampling_ratio_;
        sensor_msgs::LaserScan scan_data_;
        nav_msgs::OccupancyGrid map_data_;
        vector<float> obs_dist_table_global_;
        vector<tuple<geometry_msgs::Point32, vector<float>>> area_set_raw_;
        vector<tuple<geometry_msgs::Point32, float>> area_set_;

        unique_ptr<area_set_2d::area_set_2d> area_set_2d_instance_;
        unique_ptr<likelihood_field_2d::likelihood_field_2d> likelihood_field_2d_instance_;
        unique_ptr<gicp_2d::gicp_2d> gicp_2d_instance_;

        ros::NodeHandle node_handle_;
        ros::Publisher candidates_pub_;
    };
}

#endif