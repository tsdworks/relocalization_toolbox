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
#include <relocalization/utils/math_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
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
        likelihood_field_2d(const float &sigma_hit = 0.2f, const float &z_hit = 1.0f, const float &z_rand = 0.0f,
                            const float &keep_ratio_1_min = 0.8f, const float &keep_ratio_1_max = 0.9f, const float &keep_ratio_2_min = 0.7f, const float &keep_ratio_2_max = 0.85f,
                            const int &map_occupied_threshold = 50);
        ~likelihood_field_2d() {};

        tuple<float, float, float> get_likelihood_field_score(const geometry_msgs::TransformStamped &tf_map_lidar,
                                                              const sensor_msgs::LaserScan &scan,
                                                              const nav_msgs::OccupancyGrid &map,
                                                              const vector<float> &obs_dist_table,
                                                              const int &lidar_sampling_step);
        tuple<float, float, float> get_likelihood_field_score(const geometry_msgs::TransformStamped &tf_map_lidar,
                                                              const sensor_msgs::LaserScan &scan,
                                                              const nav_msgs::OccupancyGrid &map,
                                                              const int &lidar_sampling_step);
        float get_confidence(const geometry_msgs::TransformStamped &tf_map_lidar,
                             const sensor_msgs::LaserScan &scan,
                             const nav_msgs::OccupancyGrid &map,
                             const vector<float> &obs_dist_table,
                             const float &max_tolerance_dist,
                             const int &lidar_sampling_step,
                             const bool &geometric_mean_fusion = true);
        float get_confidence(const geometry_msgs::PoseWithCovarianceStamped &robot_pose,
                             const sensor_msgs::LaserScan &scan,
                             const nav_msgs::OccupancyGrid &map,
                             const vector<float> &obs_dist_table,
                             const float &max_tolerance_dist,
                             const int &lidar_sampling_step,
                             const string &base_frame = "base_link",
                             const bool &geometric_mean_fusion = true)
        {
            if (!tf_base_lidar_received_)
            {
                try
                {
                    tf_base_lidar_ = tf_buffer_->lookupTransform(
                        base_frame,
                        scan.header.frame_id,
                        ros::Time(0),
                        ros::Duration(60.0));

                    tf_base_lidar_received_ = true;
                }
                catch (...)
                {
                    return 0.0f;
                }
            }

            Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
            tf2::fromMsg(robot_pose.pose.pose, T_map_base);
            Eigen::Isometry3d T_base_lidar = tf2::transformToEigen(tf_base_lidar_);
            Eigen::Isometry3d T_map_lidar = T_map_base * T_base_lidar;
            geometry_msgs::TransformStamped tf_map_lidar;
            tf_map_lidar.header.stamp = robot_pose.header.stamp;
            tf_map_lidar.header.frame_id = map.header.frame_id;
            tf_map_lidar.child_frame_id = scan.header.frame_id;
            tf_map_lidar.transform.translation.x = T_map_lidar.translation().x();
            tf_map_lidar.transform.translation.y = T_map_lidar.translation().y();
            tf_map_lidar.transform.translation.z = T_map_lidar.translation().z();
            Eigen::Quaterniond q(T_map_lidar.rotation());
            q.normalize();
            tf_map_lidar.transform.rotation = tf2::toMsg(q);

            return get_confidence(tf_map_lidar,
                                  scan,
                                  map,
                                  obs_dist_table,
                                  max_tolerance_dist,
                                  lidar_sampling_step,
                                  geometric_mean_fusion);
        }

    private:
        float sigma_hit_ = 0.2f;
        float z_hit_ = 1.0f;
        float z_rand_ = 0.0f;
        int map_occupied_threshold_ = 50;
        float keep_ratio_1_min_ = 0.8f;
        float keep_ratio_1_max_ = 0.9f;
        float keep_ratio_2_min_ = 0.70f;
        float keep_ratio_2_max_ = 0.85f;

        float gaussian_prob(float dist);

        unique_ptr<tf2_ros::Buffer> tf_buffer_;
        unique_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::TransformStamped tf_base_lidar_;
        bool tf_base_lidar_received_ = false;
    };
}

#endif