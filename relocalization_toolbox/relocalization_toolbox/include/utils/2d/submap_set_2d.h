/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef SUBMAP_SET_2D_H
#define SUBMAP_SET_2D_H

#include <ros/ros.h>
#include <random>
#include <map>
#include <unordered_map>
#include <utils/math_utils.h>
#include <utils/2d/sector_query_2d.h>
#include <utils/2d/sampling_2d.h>
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
#include <atomic>

using namespace std;

namespace submap_set_2d
{
    class submap_set_2d
    {
    public:
        submap_set_2d(const bool &use_reachability_sampling = false,
                      const float &rrt_min_expand_dist = 0.5, const float &rrt_max_expand_dist = 1.0, const int &reachability_sampling_duration = 5,
                      const int &map_free_threshold = 30, const int &map_occupied_threshold = 50,
                      const float &sensing_radius = 10.0f, const float &anchor_point_min_dist = 2.0f,
                      const float &angle_search_step = 0.174);
        ~submap_set_2d() {};

        vector<tuple<geometry_msgs::Point32, nav_msgs::OccupancyGrid, vector<float>>> get_submap_set(
            const nav_msgs::OccupancyGrid &map,
            const sensor_msgs::LaserScan &scan,
            const bool &lidar_reverted,
            const int &lidar_sampling_step,
            const vector<float> &min_dist_to_obs_table,
            float &min_dist_to_obstacle,
            const bool &enable_visualization = false);

    private:
        bool use_reachability_sampling_;
        float rrt_min_expand_dist_, rrt_max_expand_dist_;
        int reachability_sampling_duration_;

        int map_free_threshold_;
        int map_occupied_threshold_;
        float sensing_radius_;
        float anchor_point_min_dist_;
        float angle_search_step_;

        ros::NodeHandle node_handle_;
        ros::Publisher anchor_point_pub_;

        nav_msgs::OccupancyGrid get_submap(const nav_msgs::OccupancyGrid &map,
                                           const sensor_msgs::LaserScan &scan,
                                           const bool &lidar_reverted,
                                           const int &lidar_sampling_step,
                                           const geometry_msgs::Point32 &anchor_point,
                                           const float &max_radius);

        vector<float> get_mean_dist_list(const nav_msgs::OccupancyGrid &map,
                                         const sensor_msgs::LaserScan &scan,
                                         const bool &lidar_reverted,
                                         const int &lidar_sampling_step,
                                         const geometry_msgs::Point32 &anchor_point,
                                         const float &max_radius);
    };
}

#endif